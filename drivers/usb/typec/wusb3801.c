// SPDX-License-Identifier: GPL-2.0
/*
 * Willsemi WUSB3801 Type-C port controller driver
 *
 * Copyright (C) 2022 Samuel Holland <samuel@sholland.org>
 *
 * Ported to this 4.19 tree for pepito (2026-07-16): pepito's stock hardware
 * has this chip at i2c-2/0x60, sharing the bus with the bhy@28 sensor hub,
 * left completely unmanaged since the BHy bring-up ("driver state on 4.19
 * unverified — track separately", see the comment on &i2c_2 in pepito.dts).
 * Suspected of causing I2C bus contention with bhy during USB accessory-mode
 * negotiation (observed: ~47s of i2c-msm-v2 TIMEOUT_ERROR on this bus right
 * after an Android Auto USB connection, cascading into a WindowManager ANR).
 *
 * This kernel predates several APIs the original driver used:
 *  - i2c_driver.probe here still takes the old two-arg
 *    (client, const struct i2c_device_id *) signature, and .remove returns
 *    int, not void (see tcpci_rt1711h.c's rt1711h_probe/rt1711h_remove for
 *    the working reference on this exact kernel).
 *  - struct typec_capability on this kernel embeds the role-change callbacks
 *    directly (try_role/port_type_set/etc, taking `const struct
 *    typec_capability *`) rather than a separate `struct typec_operations
 *    *ops` + typec_get_drvdata()/typec_set_drvdata() (neither of which exists
 *    here). container_of() on the embedded `cap` member replaces drvdata.
 *  - typec_get_fw_cap() doesn't exist on this kernel; reimplemented locally
 *    using typec_find_port_power_role()/typec_find_port_data_role()/
 *    typec_find_power_role(), which do exist (drivers/usb/typec/class.c).
 *  - typec_find_pwr_opmode() doesn't exist either; reimplemented locally
 *    against the same string table class.c uses internally for the reverse
 *    (enum -> string) direction.
 *
 * Also added: enable-GPIO handling. Pepito's stock DTS wires a
 * "wusb3801,enb-gpio" chip-enable pin that upstream's driver never touches
 * (whatever board it was written for evidently has this hardwired always-on)
 * -- gated behind the standard "enable-gpios" property here instead, driven
 * active before the chip is ever addressed over I2C.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/usb/typec.h>

#define WUSB3801_REG_DEVICE_ID		0x01
#define WUSB3801_REG_CTRL0		0x02
#define WUSB3801_REG_INT		0x03
#define WUSB3801_REG_STAT		0x04
#define WUSB3801_REG_CTRL1		0x05
#define WUSB3801_REG_TEST00		0x06
#define WUSB3801_REG_TEST01		0x07
#define WUSB3801_REG_TEST02		0x08
#define WUSB3801_REG_TEST03		0x09
#define WUSB3801_REG_TEST04		0x0a
#define WUSB3801_REG_TEST05		0x0b
#define WUSB3801_REG_TEST06		0x0c
#define WUSB3801_REG_TEST07		0x0d
#define WUSB3801_REG_TEST08		0x0e
#define WUSB3801_REG_TEST09		0x0f
#define WUSB3801_REG_TEST0A		0x10
#define WUSB3801_REG_TEST0B		0x11
#define WUSB3801_REG_TEST0C		0x12
#define WUSB3801_REG_TEST0D		0x13
#define WUSB3801_REG_TEST0E		0x14
#define WUSB3801_REG_TEST0F		0x15
#define WUSB3801_REG_TEST10		0x16
#define WUSB3801_REG_TEST11		0x17
#define WUSB3801_REG_TEST12		0x18

#define WUSB3801_DEVICE_ID_VERSION_ID	GENMASK(7, 3)
#define WUSB3801_DEVICE_ID_VENDOR_ID	GENMASK(2, 0)

#define WUSB3801_CTRL0_DIS_ACC_SUPPORT	BIT(7)
#define WUSB3801_CTRL0_TRY		GENMASK(6, 5)
#define WUSB3801_CTRL0_TRY_NONE		(0x0 << 5)
#define WUSB3801_CTRL0_TRY_SNK		(0x1 << 5)
#define WUSB3801_CTRL0_TRY_SRC		(0x2 << 5)
#define WUSB3801_CTRL0_CURRENT		GENMASK(4, 3) /* SRC */
#define WUSB3801_CTRL0_CURRENT_DEFAULT	(0x0 << 3)
#define WUSB3801_CTRL0_CURRENT_1_5A	(0x1 << 3)
#define WUSB3801_CTRL0_CURRENT_3_0A	(0x2 << 3)
#define WUSB3801_CTRL0_ROLE		GENMASK(2, 1)
#define WUSB3801_CTRL0_ROLE_SNK		(0x0 << 1)
#define WUSB3801_CTRL0_ROLE_SRC		(0x1 << 1)
#define WUSB3801_CTRL0_ROLE_DRP		(0x2 << 1)
#define WUSB3801_CTRL0_INT_MASK		BIT(0)

#define WUSB3801_INT_ATTACHED		BIT(0)
#define WUSB3801_INT_DETACHED		BIT(1)

#define WUSB3801_STAT_VBUS_DETECTED	BIT(7)
#define WUSB3801_STAT_CURRENT		GENMASK(6, 5) /* SNK */
#define WUSB3801_STAT_CURRENT_STANDBY	(0x0 << 5)
#define WUSB3801_STAT_CURRENT_DEFAULT	(0x1 << 5)
#define WUSB3801_STAT_CURRENT_1_5A	(0x2 << 5)
#define WUSB3801_STAT_CURRENT_3_0A	(0x3 << 5)
#define WUSB3801_STAT_PARTNER		GENMASK(4, 2)
#define WUSB3801_STAT_PARTNER_STANDBY	(0x0 << 2)
#define WUSB3801_STAT_PARTNER_SNK	(0x1 << 2)
#define WUSB3801_STAT_PARTNER_SRC	(0x2 << 2)
#define WUSB3801_STAT_PARTNER_AUDIO	(0x3 << 2)
#define WUSB3801_STAT_PARTNER_DEBUG	(0x4 << 2)
#define WUSB3801_STAT_ORIENTATION	GENMASK(1, 0)
#define WUSB3801_STAT_ORIENTATION_NONE	(0x0 << 0)
#define WUSB3801_STAT_ORIENTATION_CC1	(0x1 << 0)
#define WUSB3801_STAT_ORIENTATION_CC2	(0x2 << 0)
#define WUSB3801_STAT_ORIENTATION_BOTH	(0x3 << 0)

#define WUSB3801_CTRL1_SM_RESET		BIT(0)

#define WUSB3801_TEST01_VENDOR_SUB_ID	(BIT(8) | BIT(6))

#define WUSB3801_TEST02_FORCE_ERR_RCY	BIT(8)

#define WUSB3801_TEST0A_WAIT_VBUS	BIT(5)

struct wusb3801 {
	struct typec_capability	cap;
	struct device		*dev;
	struct typec_partner	*partner;
	struct typec_port	*port;
	struct regmap		*regmap;
	struct regulator	*vbus_supply;
	struct gpio_desc	*enable_gpio;
	unsigned int		partner_type;
	enum typec_port_type	port_type;
	enum typec_pwr_opmode	pwr_opmode;
	bool			vbus_on;
};

/*
 * Local replacement for typec_find_pwr_opmode(), which doesn't exist on
 * this kernel -- mirrors the enum -> string table drivers/usb/typec/class.c
 * keeps privately for the opposite (sysfs display) direction.
 */
static int wusb3801_find_pwr_opmode(const char *name)
{
	static const char * const modes[] = {
		[TYPEC_PWR_MODE_USB]	= "default",
		[TYPEC_PWR_MODE_1_5A]	= "1.5A",
		[TYPEC_PWR_MODE_3_0A]	= "3.0A",
		[TYPEC_PWR_MODE_PD]	= "usb_power_delivery",
	};

	return match_string(modes, ARRAY_SIZE(modes), name);
}

/*
 * Local replacement for typec_get_fw_cap(), which doesn't exist on this
 * kernel -- built from typec_find_port_power_role()/typec_find_power_role()/
 * typec_find_port_data_role(), which do (drivers/usb/typec/class.c).
 */
static int wusb3801_get_fw_cap(struct typec_capability *cap,
			       struct fwnode_handle *connector)
{
	const char *str;
	int ret;

	cap->fwnode = connector;
	cap->prefer_role = TYPEC_NO_PREFERRED_ROLE;

	ret = fwnode_property_read_string(connector, "power-role", &str);
	if (ret)
		return ret;
	ret = typec_find_port_power_role(str);
	if (ret < 0)
		return ret;
	cap->type = ret;

	ret = fwnode_property_read_string(connector, "data-role", &str);
	if (ret)
		return ret;
	ret = typec_find_port_data_role(str);
	if (ret < 0)
		return ret;
	cap->data = ret;

	if (!fwnode_property_read_string(connector, "try-power-role", &str)) {
		ret = typec_find_power_role(str);
		if (ret >= 0)
			cap->prefer_role = ret;
	}

	return 0;
}

static enum typec_role wusb3801_get_default_role(struct wusb3801 *wusb3801)
{
	switch (wusb3801->port_type) {
	case TYPEC_PORT_SRC:
		return TYPEC_SOURCE;
	case TYPEC_PORT_SNK:
		return TYPEC_SINK;
	case TYPEC_PORT_DRP:
	default:
		if (wusb3801->cap.prefer_role == TYPEC_SOURCE)
			return TYPEC_SOURCE;
		return TYPEC_SINK;
	}
}

static int wusb3801_map_port_type(enum typec_port_type type)
{
	switch (type) {
	case TYPEC_PORT_SRC:
		return WUSB3801_CTRL0_ROLE_SRC;
	case TYPEC_PORT_SNK:
		return WUSB3801_CTRL0_ROLE_SNK;
	case TYPEC_PORT_DRP:
	default:
		return WUSB3801_CTRL0_ROLE_DRP;
	}
}

static int wusb3801_map_pwr_opmode(enum typec_pwr_opmode mode)
{
	switch (mode) {
	case TYPEC_PWR_MODE_USB:
	default:
		return WUSB3801_CTRL0_CURRENT_DEFAULT;
	case TYPEC_PWR_MODE_1_5A:
		return WUSB3801_CTRL0_CURRENT_1_5A;
	case TYPEC_PWR_MODE_3_0A:
		return WUSB3801_CTRL0_CURRENT_3_0A;
	}
}

static unsigned int wusb3801_map_try_role(int role)
{
	switch (role) {
	case TYPEC_NO_PREFERRED_ROLE:
	default:
		return WUSB3801_CTRL0_TRY_NONE;
	case TYPEC_SINK:
		return WUSB3801_CTRL0_TRY_SNK;
	case TYPEC_SOURCE:
		return WUSB3801_CTRL0_TRY_SRC;
	}
}

static enum typec_orientation wusb3801_unmap_orientation(unsigned int status)
{
	switch (status & WUSB3801_STAT_ORIENTATION) {
	case WUSB3801_STAT_ORIENTATION_NONE:
	case WUSB3801_STAT_ORIENTATION_BOTH:
	default:
		return TYPEC_ORIENTATION_NONE;
	case WUSB3801_STAT_ORIENTATION_CC1:
		return TYPEC_ORIENTATION_NORMAL;
	case WUSB3801_STAT_ORIENTATION_CC2:
		return TYPEC_ORIENTATION_REVERSE;
	}
}

static enum typec_pwr_opmode wusb3801_unmap_pwr_opmode(unsigned int status)
{
	switch (status & WUSB3801_STAT_CURRENT) {
	case WUSB3801_STAT_CURRENT_STANDBY:
	case WUSB3801_STAT_CURRENT_DEFAULT:
	default:
		return TYPEC_PWR_MODE_USB;
	case WUSB3801_STAT_CURRENT_1_5A:
		return TYPEC_PWR_MODE_1_5A;
	case WUSB3801_STAT_CURRENT_3_0A:
		return TYPEC_PWR_MODE_3_0A;
	}
}

static int wusb3801_try_role(const struct typec_capability *cap, int role)
{
	struct wusb3801 *wusb3801 = container_of(cap, struct wusb3801, cap);

	return regmap_update_bits(wusb3801->regmap, WUSB3801_REG_CTRL0,
				  WUSB3801_CTRL0_TRY,
				  wusb3801_map_try_role(role));
}

static int wusb3801_port_type_set(const struct typec_capability *cap,
				  enum typec_port_type type)
{
	struct wusb3801 *wusb3801 = container_of(cap, struct wusb3801, cap);
	int ret;

	ret = regmap_update_bits(wusb3801->regmap, WUSB3801_REG_CTRL0,
				 WUSB3801_CTRL0_ROLE,
				 wusb3801_map_port_type(type));
	if (ret)
		return ret;

	wusb3801->port_type = type;

	return 0;
}

static int wusb3801_hw_init(struct wusb3801 *wusb3801)
{
	return regmap_write(wusb3801->regmap, WUSB3801_REG_CTRL0,
			    wusb3801_map_try_role(wusb3801->cap.prefer_role) |
			    wusb3801_map_pwr_opmode(wusb3801->pwr_opmode) |
			    wusb3801_map_port_type(wusb3801->port_type));
}

static void wusb3801_hw_update(struct wusb3801 *wusb3801)
{
	struct typec_port *port = wusb3801->port;
	struct device *dev = wusb3801->dev;
	unsigned int partner_type, status;
	int ret;

	ret = regmap_read(wusb3801->regmap, WUSB3801_REG_STAT, &status);
	if (ret) {
		/*
		 * i2c-2 is shared with the bhy sensor hub; a transient read
		 * glitch must NOT be treated as a CC event. The old code faked
		 * status = 0, which decodes as PARTNER_STANDBY and then
		 * unregisters the partner + resets the port roles below -- a
		 * phantom detach on a healthy connection. Keep the last known
		 * state and let the next IRQ re-read the real status instead.
		 */
		dev_warn(dev, "Failed to read port status: %d (keeping last state)\n",
			 ret);
		return;
	}
	dev_dbg(dev, "status = 0x%02x\n", status);

	partner_type = status & WUSB3801_STAT_PARTNER;

	if (partner_type == WUSB3801_STAT_PARTNER_SNK) {
		if (!wusb3801->vbus_on) {
			ret = regulator_enable(wusb3801->vbus_supply);
			if (ret)
				dev_warn(dev, "Failed to enable VBUS: %d\n", ret);
			wusb3801->vbus_on = true;
		}
	} else {
		if (wusb3801->vbus_on) {
			regulator_disable(wusb3801->vbus_supply);
			wusb3801->vbus_on = false;
		}
	}

	if (partner_type != wusb3801->partner_type) {
		struct typec_partner_desc desc = {};
		enum typec_data_role data_role;
		enum typec_role pwr_role = wusb3801_get_default_role(wusb3801);

		switch (partner_type) {
		case WUSB3801_STAT_PARTNER_STANDBY:
			break;
		case WUSB3801_STAT_PARTNER_SNK:
			pwr_role = TYPEC_SOURCE;
			break;
		case WUSB3801_STAT_PARTNER_SRC:
			pwr_role = TYPEC_SINK;
			break;
		case WUSB3801_STAT_PARTNER_AUDIO:
			desc.accessory = TYPEC_ACCESSORY_AUDIO;
			break;
		case WUSB3801_STAT_PARTNER_DEBUG:
			desc.accessory = TYPEC_ACCESSORY_DEBUG;
			break;
		}

		if (wusb3801->partner) {
			typec_unregister_partner(wusb3801->partner);
			wusb3801->partner = NULL;
		}

		if (partner_type != WUSB3801_STAT_PARTNER_STANDBY) {
			wusb3801->partner = typec_register_partner(port, &desc);
			if (IS_ERR(wusb3801->partner))
				dev_err(dev, "Failed to register partner: %ld\n",
					PTR_ERR(wusb3801->partner));
		}

		data_role = pwr_role == TYPEC_SOURCE ? TYPEC_HOST : TYPEC_DEVICE;
		typec_set_data_role(port, data_role);
		typec_set_pwr_role(port, pwr_role);
		typec_set_vconn_role(port, pwr_role);
	}

	typec_set_pwr_opmode(wusb3801->port,
			     partner_type == WUSB3801_STAT_PARTNER_SRC
				? wusb3801_unmap_pwr_opmode(status)
				: wusb3801->pwr_opmode);
	typec_set_orientation(wusb3801->port,
			      wusb3801_unmap_orientation(status));

	wusb3801->partner_type = partner_type;
}

static irqreturn_t wusb3801_irq(int irq, void *data)
{
	struct wusb3801 *wusb3801 = data;
	unsigned int dummy;

	/*
	 * The interrupt register must be read in order to clear the IRQ,
	 * but all of the useful information is in the status register.
	 */
	regmap_read(wusb3801->regmap, WUSB3801_REG_INT, &dummy);

	wusb3801_hw_update(wusb3801);

	return IRQ_HANDLED;
}

static const struct regmap_config config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= WUSB3801_REG_TEST12,
};

static int wusb3801_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *connector;
	struct wusb3801 *wusb3801;
	const char *cap_str;
	int ret;

	wusb3801 = devm_kzalloc(dev, sizeof(*wusb3801), GFP_KERNEL);
	if (!wusb3801)
		return -ENOMEM;

	i2c_set_clientdata(client, wusb3801);

	wusb3801->dev = dev;

	/*
	 * Stock DTS wires a chip-enable pin ("wusb3801,enb-gpio") that
	 * upstream never handles -- drive it before touching the bus.
	 *
	 * First real hardware test (2026-07-16) hit a clean, fast NACK
	 * (-ENOTCONN via i2c-msm-v2's I2C_MSM_ERR_NACK mapping, 34ms after
	 * boot -- not a bus timeout, a completed transaction the chip simply
	 * didn't acknowledge). That's consistent with the chip not yet being
	 * ready right after its enable pin goes high: gpiod_direction_output()
	 * returns as soon as the GPIO is asserted, with no allowance for the
	 * chip's own power-on settle time before the very next statement
	 * (regmap_init_i2c, then an immediate register write in hw_init())
	 * addresses it. No WUSB3801 datasheet was available to pull an exact
	 * t_POR spec from; a few ms is a conventional safe minimum for this
	 * class of I2C port controller. If NACKs persist after this, the
	 * enable-GPIO number/polarity itself (GPIO 131, active-high -- both
	 * decoded from the stock DTB, not independently verified) is the
	 * next thing to question.
	 */
	wusb3801->enable_gpio = devm_gpiod_get_optional(dev, "enable",
							GPIOD_OUT_HIGH);
	if (IS_ERR(wusb3801->enable_gpio))
		return PTR_ERR(wusb3801->enable_gpio);
	if (wusb3801->enable_gpio)
		usleep_range(5000, 10000);

	wusb3801->regmap = devm_regmap_init_i2c(client, &config);
	if (IS_ERR(wusb3801->regmap))
		return PTR_ERR(wusb3801->regmap);

	wusb3801->vbus_supply = devm_regulator_get(dev, "vbus");
	if (IS_ERR(wusb3801->vbus_supply))
		return PTR_ERR(wusb3801->vbus_supply);

	connector = device_get_named_child_node(dev, "connector");
	if (!connector)
		return -ENODEV;

	ret = wusb3801_get_fw_cap(&wusb3801->cap, connector);
	if (ret)
		goto err_put_connector;
	wusb3801->port_type = wusb3801->cap.type;

	ret = fwnode_property_read_string(connector, "typec-power-opmode", &cap_str);
	if (ret)
		goto err_put_connector;

	ret = wusb3801_find_pwr_opmode(cap_str);
	if (ret < 0 || ret == TYPEC_PWR_MODE_PD)
		goto err_put_connector;
	wusb3801->pwr_opmode = ret;

	/* Initialize the hardware with the devicetree settings. */
	ret = wusb3801_hw_init(wusb3801);
	if (ret)
		goto err_put_connector;

	wusb3801->cap.revision		= USB_TYPEC_REV_1_2;
	wusb3801->cap.accessory[0]	= TYPEC_ACCESSORY_AUDIO;
	wusb3801->cap.accessory[1]	= TYPEC_ACCESSORY_DEBUG;
	wusb3801->cap.try_role		= wusb3801_try_role;
	wusb3801->cap.port_type_set	= wusb3801_port_type_set;

	wusb3801->port = typec_register_port(dev, &wusb3801->cap);
	if (IS_ERR(wusb3801->port)) {
		ret = PTR_ERR(wusb3801->port);
		goto err_put_connector;
	}

	/* Initialize the port attributes from the hardware state. */
	wusb3801_hw_update(wusb3801);

	ret = request_threaded_irq(client->irq, NULL, wusb3801_irq,
				   IRQF_ONESHOT, dev_name(dev), wusb3801);
	if (ret)
		goto err_unregister_port;

	fwnode_handle_put(connector);

	return 0;

err_unregister_port:
	typec_unregister_port(wusb3801->port);
err_put_connector:
	fwnode_handle_put(connector);

	return ret;
}

static int wusb3801_remove(struct i2c_client *client)
{
	struct wusb3801 *wusb3801 = i2c_get_clientdata(client);

	free_irq(client->irq, wusb3801);

	if (wusb3801->partner)
		typec_unregister_partner(wusb3801->partner);
	typec_unregister_port(wusb3801->port);

	if (wusb3801->vbus_on)
		regulator_disable(wusb3801->vbus_supply);

	return 0;
}

static const struct of_device_id wusb3801_of_match[] = {
	{ .compatible = "willsemi,wusb3801" },
	{}
};
MODULE_DEVICE_TABLE(of, wusb3801_of_match);

static struct i2c_driver wusb3801_driver = {
	.probe		= wusb3801_probe,
	.remove		= wusb3801_remove,
	.driver		= {
		.name		= "wusb3801",
		.of_match_table	= wusb3801_of_match,
	},
};

module_i2c_driver(wusb3801_driver);

MODULE_AUTHOR("Samuel Holland <samuel@sholland.org>");
MODULE_DESCRIPTION("Willsemi WUSB3801 Type-C port controller driver");
MODULE_LICENSE("GPL");
