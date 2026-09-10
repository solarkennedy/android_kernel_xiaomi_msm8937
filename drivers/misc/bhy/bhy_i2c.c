/*!
* @section LICENSE
 * (C) Copyright 2011~2015 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
*
* @filename bhy_i2c.c
* @date     "Fri Feb 13 14:57:45 2015 +0800"
* @id       "a51313e"
*
* @brief
* The implementation file for BHy I2C bus driver
*/

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/jiffies.h>

#include "bhy_core.h"
#include "bs_log.h"

#define BHY_MAX_RETRY_I2C_XFER		10
#define BHY_I2C_WRITE_DELAY_TIME	1000
#define BHY_I2C_MAX_BURST_WRITE_LEN	64

/*
 * Circuit breaker: a fully-exhausted BHY_MAX_RETRY_I2C_XFER loop already
 * costs ~10 * (i2c-msm-v2's own per-xfer timeout, ~2.3s when the bus is
 * wedged) = tens of seconds. bhy_read_fifo_data() is called again on every
 * subsequent FIFO-ready IRQ with no backoff of its own, so a bus that stays
 * wedged previously meant each IRQ re-ran the full ~23s retry loop back to
 * back, and since every bhy_i2c_read/write call is serialized under
 * client_data->mutex_bus_op, that also starves the sensors HAL's whole
 * binder thread pool for the same duration (first observed 2026-07-16: ~47s
 * of continuous I2C timeouts froze WindowManager's orientation-listener
 * enable/disable, cascading into a ~143s system-wide ANR after an Android
 * Auto USB connection).
 *
 * Once a full retry cycle is exhausted, fail fast for a cooldown instead of
 * immediately repeating it -- this bounds the worst case to one ~23s stall
 * instead of compounding indefinitely, and frees the binder thread pool
 * quickly so unrelated sensor/HAL calls aren't starved by a single wedged
 * transaction.
 *
 * The original 2000ms cooldown was measured (2026-07-16, second real-hardware
 * test) to be too short to actually engage: individual failed attempts were
 * recurring every ~2.2-2.4s (a single attempt's own i2c-msm-v2 timeout was
 * ~2.37s and climbing, since each retry re-reads a growing FIFO backlog and
 * the per-xfer timeout scales with byte count -- see
 * i2c_msm_xfer_calc_timeout() in i2c-msm-v2.c), so the cooldown window was
 * *shorter* than the caller's own natural retry interval and had expired by
 * the time the next call arrived -- 170 back-to-back timeouts over 6+
 * minutes, no throttling. Widened well past the observed cadence so a
 * detected wedge actually gets breathing room.
 *
 * Root cause found 2026-07-16, and it is NOT what the paragraphs above
 * assume. The bus is not wedged at all -- only the hub is. In a full
 * reproduction, all 76 TIMEOUT_ERRORs were slv_addr:0x28 (this chip) and
 * *zero* were 0x60, while the wusb3801 driver on the same bus read 0x60
 * successfully throughout the storm; a wedged bus would have failed those
 * too. Nor is USB the trigger: the storm began 2m42s after a plug and kept
 * climbing after unplug. What actually precedes it is suspend/resume churn
 * (42 wcnss_wlan resumes in 51s; bhy died 376ms after the full wake that
 * ended the burst), i.e. the hub hangs and stops answering.
 *
 * So this cooldown is not the fix -- it is damage control, and it works:
 * it bounds the stall and keeps the system responsive (measured cadence =
 * 15s cooldown + one ~2.37s retry, no ANR). The actual repair is making
 * the driver's existing recovery reachable (see check_watchdog_reset() and
 * reset() in bhy_core.c) so the hub gets reset instead of retried forever.
 * Note bhy_i2c_clear_degraded() below exists precisely so this breaker
 * cannot block that recovery.
 */
#define BHY_I2C_FAIL_COOLDOWN_MS	15000
static unsigned long bhy_i2c_last_fail_jiffies;
static bool bhy_i2c_bus_degraded;

static bool bhy_i2c_cooldown_active(void)
{
	if (!bhy_i2c_bus_degraded)
		return false;

	if (time_after(jiffies,
			bhy_i2c_last_fail_jiffies +
			msecs_to_jiffies(BHY_I2C_FAIL_COOLDOWN_MS))) {
		bhy_i2c_bus_degraded = false;
		return false;
	}

	return true;
}

static void bhy_i2c_note_result(int ret)
{
	if (ret < 0) {
		bhy_i2c_bus_degraded = true;
		bhy_i2c_last_fail_jiffies = jiffies;
	} else {
		bhy_i2c_bus_degraded = false;
	}
}

/*
 * Drop the breaker on demand, for reset() in bhy_core.c.
 *
 * The recovery path reloads the RAM patch over this same bus, but it only
 * runs *because* the bus wedged -- so the cooldown is essentially always
 * armed by the time it starts, and would fail-fast (-EIO) every op of the
 * reload, including the BHY_REG_RESET_REQ write meant to soft-reset the
 * MCU. The breaker would thus block the one thing that can clear the
 * condition it exists to survive, and the sensors would stay dead exactly
 * as if no recovery existed at all.
 *
 * Anything the breaker learned describes the hub state we are about to
 * tear down, so it is stale by construction at that point. If the reload
 * genuinely cannot get through, the very first failure re-arms it.
 */
void bhy_i2c_clear_degraded(void)
{
	bhy_i2c_bus_degraded = false;
}
EXPORT_SYMBOL(bhy_i2c_clear_degraded);

/*
 * The retry loops below exist for transient NACK/arbitration glitches, which
 * fail in microseconds. A failure that took hundreds of milliseconds is the
 * controller's own transfer timeout (bus held, slave not answering) and
 * repeating it BHY_MAX_RETRY_I2C_XFER times only multiplies the stall while
 * mutex_bus_op is held: measured 2026-09-09, one 8254-byte FIFO read that
 * timed out at 2.36 s was retried 10x back to back = 24.8 s during which
 * every sensors-HAL call blocked, WindowManager sat in
 * WindowOrientationListener.enable() holding its lock, and touch input was
 * frozen for 23 s ("PointerEventDispatcher0 spent 23053ms"). Give up after
 * the first slow failure instead; the breaker and the recovery path take it
 * from there.
 */
#define BHY_I2C_SLOW_FAIL_MS		100

static bool bhy_i2c_failed_slowly(unsigned long t0)
{
	return time_after(jiffies, t0 + msecs_to_jiffies(BHY_I2C_SLOW_FAIL_MS));
}

static s32 bhy_i2c_read_internal(struct i2c_client *client,
		u8 reg, u8 *data, u16 len)
{
	int ret, retry;

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	if (bhy_i2c_cooldown_active())
		return -EIO;

	for (retry = 0; retry < BHY_MAX_RETRY_I2C_XFER; retry++) {
		unsigned long t0 = jiffies;

		ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		if (ret >= 0)
			break;
		if (bhy_i2c_failed_slowly(t0))
			break;
		usleep_range(BHY_I2C_WRITE_DELAY_TIME,
				BHY_I2C_WRITE_DELAY_TIME);
	}

	bhy_i2c_note_result(ret);
	return ret;
	/*int ret;
	if ((ret = i2c_master_send(client, &reg, 1)) < 0)
		return ret;
	return i2c_master_recv(client, data, len);*/
}

static s32 bhy_i2c_write_internal(struct i2c_client *client,
		u8 reg, u8 *data, u16 len)
{
	int ret, retry;
	u8 buf[BHY_I2C_MAX_BURST_WRITE_LEN + 1];
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
	};

	if (len > BHY_I2C_MAX_BURST_WRITE_LEN)
		return -EINVAL;

	buf[0] = reg;
	memcpy(&buf[1], data, len);
	msg.len = len + 1;
	msg.buf = buf;

	if (bhy_i2c_cooldown_active())
		return -EIO;

	for (retry = 0; retry < BHY_MAX_RETRY_I2C_XFER; retry++) {
		unsigned long t0 = jiffies;

		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret >= 0)
			break;
		if (bhy_i2c_failed_slowly(t0))
			break;
		usleep_range(BHY_I2C_WRITE_DELAY_TIME,
				BHY_I2C_WRITE_DELAY_TIME);
	}

	bhy_i2c_note_result(ret);
	return ret;
}

static s32 bhy_i2c_read(struct device *dev, u8 reg, u8 *data, u16 len)
{
	struct i2c_client *client;
	client = to_i2c_client(dev);
	return bhy_i2c_read_internal(client, reg, data, len);
}

static s32 bhy_i2c_write(struct device *dev, u8 reg, u8 *data, u16 len)
{
	struct i2c_client *client;
	client = to_i2c_client(dev);
	return bhy_i2c_write_internal(client, reg, data, len);
}

#ifdef CONFIG_PM
static int bhy_pm_op_suspend(struct device *dev)
{
	return bhy_suspend(dev);
}

static int bhy_pm_op_resume(struct device *dev)
{
	return bhy_resume(dev);
}

static const struct dev_pm_ops bhy_pm_ops = {
	.suspend = bhy_pm_op_suspend,
	.resume = bhy_pm_op_resume,
};
#endif

/*!
 * @brief	bhy version of i2c_probe
 */
static int bhy_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *dev_id) {
	struct bhy_data_bus data_bus = {
		.read = bhy_i2c_read,
		.write = bhy_i2c_write,
		.dev = &client->dev,
		.irq = client->irq,
		.bus_type = BUS_I2C,
	};

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		PERR("i2c_check_functionality error!");
		return -EIO;
	}

	return bhy_probe(&data_bus);
}

static void bhy_i2c_shutdown(struct i2c_client *client)
{
}

static int bhy_i2c_remove(struct i2c_client *client)
{
	return bhy_remove(&client->dev);
}

static const struct i2c_device_id bhy_i2c_id[] = {
	{ "bhy", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, bhy_i2c_id);

static const struct of_device_id device_of_match[] = {
	{ .compatible = "bst,bhy", },
	{},
};

static struct i2c_driver bhy_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "bhy",
		.of_match_table = of_match_ptr(device_of_match),
#ifdef CONFIG_PM
		.pm = &bhy_pm_ops,
#endif
	},
	.id_table = bhy_i2c_id,
	.probe = bhy_i2c_probe,
	.shutdown = bhy_i2c_shutdown,
	.remove = bhy_i2c_remove,
};

module_i2c_driver(bhy_i2c_driver);

MODULE_AUTHOR("Contact <contact@bosch-sensortec.com>");
MODULE_DESCRIPTION("BHY I2C DRIVER");
MODULE_LICENSE("GPL v2");
