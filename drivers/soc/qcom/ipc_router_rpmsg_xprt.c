// SPDX-License-Identifier: GPL-2.0-only
/*
 * IPC Router RPMSG XPRT — [qmux/2b, pepito bring-up]
 *
 * Binds the legacy msm_ipc_router to the modem edge's "IPCRTR" SMD channel
 * through the mainline rpmsg qcom_smd driver, replacing the 3.18/4.9
 * ipc_router_smd_xprt (which spoke the raw msm_smd API this tree doesn't
 * have).  Modeled on the CAF msm-4.9 ipc_router_smd_xprt.c /
 * ipc_router_fifo_xprt.c.
 *
 * Simplifications vs the SMD xprt, both safe on this transport:
 * - rpmsg delivers one COMPLETE SMD packet per rx callback, so the
 *   partial-packet reassembly logic is unnecessary: every callback payload
 *   is one full router packet.
 * - writes linearize the rr_packet fragment queue and issue a single
 *   rpmsg_send(), which blocks for FIFO space internally (the same SMD
 *   packet framing the QRTR transport uses on this channel today).  Packets
 *   larger than the channel FIFO are rejected by rpmsg; router control and
 *   QMI traffic is far below that.
 *
 * Runtime A/B (no reflash): the `enable` module parameter defaults to 0 and
 * the probe then declines every channel, so QRTR keeps the edge exactly as
 * before.  With enable=1 the probe accepts the MODEM edge only (parent DT
 * node with qcom,smd-edge == 0); since drivers/ initcalls precede net/ in
 * link order, this driver out-ranks qcom_smd_qrtr at re-probe time, so a
 * modem SSR after flipping `enable` moves the edge between the two stacks
 * in either direction.  adsp/wcnss always stay on QRTR.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/skbuff.h>
#include <linux/rpmsg.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/ipc_router_xprt.h>

static bool enable;
module_param(enable, bool, 0644);
MODULE_PARM_DESC(enable, "Accept the modem IPCRTR channel (flip + modem SSR)");

static int debug_mask;
module_param(debug_mask, int, 0644);

#define D(x...) do { if (debug_mask) pr_info(x); } while (0)

#define XPRT_NAME "ipc_rtr_rpmsg_ipcrtr"
#define XPRT_LINK_ID 1
#define XPRT_VERSION 1

struct ipcr_rpmsg_xprt {
	struct rpmsg_device *rpdev;
	struct msm_ipc_router_xprt xprt;
	struct workqueue_struct *wq;
	struct sk_buff_head rx_queue;
	struct work_struct rx_work;
	spinlock_t ss_reset_lock;
	int ss_reset;
	struct completion sft_close_complete;
	unsigned int xprt_version;
};

static void ipcr_rpmsg_set_xprt_version(struct msm_ipc_router_xprt *xprt,
					unsigned int version)
{
	struct ipcr_rpmsg_xprt *xprtp;

	if (!xprt)
		return;
	xprtp = container_of(xprt, struct ipcr_rpmsg_xprt, xprt);
	xprtp->xprt_version = version;
	D("%s: version %u set on %s\n", __func__, version, xprt->name);
}

static int ipcr_rpmsg_get_xprt_version(struct msm_ipc_router_xprt *xprt)
{
	struct ipcr_rpmsg_xprt *xprtp;

	if (!xprt)
		return -EINVAL;
	xprtp = container_of(xprt, struct ipcr_rpmsg_xprt, xprt);
	return (int)xprtp->xprt_version;
}

static int ipcr_rpmsg_get_xprt_option(struct msm_ipc_router_xprt *xprt)
{
	return 0;
}

static int ipcr_rpmsg_write(void *data, uint32_t len,
			    struct msm_ipc_router_xprt *xprt)
{
	struct rr_packet *pkt = (struct rr_packet *)data;
	struct ipcr_rpmsg_xprt *xprtp =
		container_of(xprt, struct ipcr_rpmsg_xprt, xprt);
	struct sk_buff *skb;
	unsigned long flags;
	void *buf;
	uint32_t off = 0;
	int ret;

	if (!pkt || !len || pkt->length != len)
		return -EINVAL;

	spin_lock_irqsave(&xprtp->ss_reset_lock, flags);
	if (xprtp->ss_reset) {
		spin_unlock_irqrestore(&xprtp->ss_reset_lock, flags);
		IPC_RTR_ERR("%s: %s channel reset\n", __func__, xprt->name);
		return -ENETRESET;
	}
	spin_unlock_irqrestore(&xprtp->ss_reset_lock, flags);

	buf = kmalloc(len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	skb_queue_walk(pkt->pkt_fragment_q, skb) {
		if (off + skb->len > len) {
			kfree(buf);
			return -EINVAL;
		}
		memcpy(buf + off, skb->data, skb->len);
		off += skb->len;
	}

	ret = rpmsg_send(xprtp->rpdev->ept, buf, len);
	kfree(buf);
	if (ret < 0) {
		IPC_RTR_ERR("%s: rpmsg_send of %u bytes failed on %s: %d\n",
			    __func__, len, xprt->name, ret);
		return ret;
	}
	D("%s: wrote %u bytes over %s\n", __func__, len, xprt->name);
	return len;
}

static int ipcr_rpmsg_close(struct msm_ipc_router_xprt *xprt)
{
	/* Channel lifetime is owned by the rpmsg core. */
	return 0;
}

static void ipcr_rpmsg_sft_close_done(struct msm_ipc_router_xprt *xprt)
{
	struct ipcr_rpmsg_xprt *xprtp =
		container_of(xprt, struct ipcr_rpmsg_xprt, xprt);

	complete_all(&xprtp->sft_close_complete);
}

static void ipcr_rpmsg_rx_work(struct work_struct *work)
{
	struct ipcr_rpmsg_xprt *xprtp =
		container_of(work, struct ipcr_rpmsg_xprt, rx_work);
	struct sk_buff *skb;
	struct rr_packet *in_pkt;
	unsigned long flags;

	while ((skb = skb_dequeue(&xprtp->rx_queue)) != NULL) {
		spin_lock_irqsave(&xprtp->ss_reset_lock, flags);
		if (xprtp->ss_reset) {
			spin_unlock_irqrestore(&xprtp->ss_reset_lock, flags);
			kfree_skb(skb);
			continue;
		}
		spin_unlock_irqrestore(&xprtp->ss_reset_lock, flags);

		in_pkt = create_pkt(NULL);
		if (!in_pkt) {
			IPC_RTR_ERR("%s: Couldn't alloc rr_packet\n", __func__);
			kfree_skb(skb);
			continue;
		}
		skb_queue_tail(in_pkt->pkt_fragment_q, skb);
		in_pkt->length = skb->len;
		D("%s: packet size read %u\n", __func__, in_pkt->length);
		msm_ipc_router_xprt_notify(&xprtp->xprt,
					   IPC_ROUTER_XPRT_EVENT_DATA,
					   (void *)in_pkt);
		release_pkt(in_pkt);
	}
}

/* rpmsg rx callback — atomic context (called under the SMD channel's
 * recv_lock): copy out and defer to the worker, mirroring the SMD xprt's
 * notify-from-worker pattern.
 */
static int ipcr_rpmsg_callback(struct rpmsg_device *rpdev, void *data,
			       int len, void *priv, u32 addr)
{
	struct ipcr_rpmsg_xprt *xprtp = dev_get_drvdata(&rpdev->dev);
	struct sk_buff *skb;

	if (!xprtp || len <= 0)
		return -EINVAL;

	skb = alloc_skb(len, GFP_ATOMIC);
	if (!skb) {
		IPC_RTR_ERR("%s: rx alloc_skb(%d) failed\n", __func__, len);
		return -ENOMEM;
	}
	skb_put_data(skb, data, len);
	skb_queue_tail(&xprtp->rx_queue, skb);
	queue_work(xprtp->wq, &xprtp->rx_work);
	return 0;
}

/* Accept only the modem edge: walk up to the qcom_smd edge device and read
 * its qcom,smd-edge (modem == SMD_APPS_MODEM == 0).
 */
static bool ipcr_rpmsg_edge_is_modem(struct rpmsg_device *rpdev)
{
	struct device *dev;
	u32 edge;

	for (dev = rpdev->dev.parent; dev; dev = dev->parent) {
		if (dev->of_node &&
		    !of_property_read_u32(dev->of_node, "qcom,smd-edge",
					  &edge))
			return edge == 0;
	}
	return false;
}

static int ipcr_rpmsg_probe(struct rpmsg_device *rpdev)
{
	struct ipcr_rpmsg_xprt *xprtp;

	if (!enable)
		return -ENODEV;
	if (!ipcr_rpmsg_edge_is_modem(rpdev))
		return -ENODEV;

	xprtp = kzalloc(sizeof(*xprtp), GFP_KERNEL);
	if (!xprtp)
		return -ENOMEM;

	xprtp->rpdev = rpdev;
	xprtp->xprt_version = XPRT_VERSION;
	xprtp->xprt.name = XPRT_NAME;
	xprtp->xprt.link_id = XPRT_LINK_ID;
	xprtp->xprt.get_version = ipcr_rpmsg_get_xprt_version;
	xprtp->xprt.set_version = ipcr_rpmsg_set_xprt_version;
	xprtp->xprt.get_option = ipcr_rpmsg_get_xprt_option;
	xprtp->xprt.read_avail = NULL;
	xprtp->xprt.read = NULL;
	xprtp->xprt.write_avail = NULL;
	xprtp->xprt.write = ipcr_rpmsg_write;
	xprtp->xprt.close = ipcr_rpmsg_close;
	xprtp->xprt.sft_close_done = ipcr_rpmsg_sft_close_done;
	xprtp->xprt.priv = NULL;

	spin_lock_init(&xprtp->ss_reset_lock);
	skb_queue_head_init(&xprtp->rx_queue);
	INIT_WORK(&xprtp->rx_work, ipcr_rpmsg_rx_work);
	init_completion(&xprtp->sft_close_complete);

	xprtp->wq = create_singlethread_workqueue(XPRT_NAME);
	if (!xprtp->wq) {
		kfree(xprtp);
		return -EFAULT;
	}

	dev_set_drvdata(&rpdev->dev, xprtp);

	msm_ipc_router_xprt_notify(&xprtp->xprt,
				   IPC_ROUTER_XPRT_EVENT_OPEN, NULL);
	pr_info("%s: %s up on modem IPCRTR (rpmsg)\n", __func__, XPRT_NAME);
	return 0;
}

static void ipcr_rpmsg_remove(struct rpmsg_device *rpdev)
{
	struct ipcr_rpmsg_xprt *xprtp = dev_get_drvdata(&rpdev->dev);
	unsigned long flags;

	if (!xprtp)
		return;

	spin_lock_irqsave(&xprtp->ss_reset_lock, flags);
	xprtp->ss_reset = 1;
	spin_unlock_irqrestore(&xprtp->ss_reset_lock, flags);

	flush_work(&xprtp->rx_work);
	skb_queue_purge(&xprtp->rx_queue);

	init_completion(&xprtp->sft_close_complete);
	msm_ipc_router_xprt_notify(&xprtp->xprt,
				   IPC_ROUTER_XPRT_EVENT_CLOSE, NULL);
	D("%s: notified CLOSE for %s\n", __func__, xprtp->xprt.name);
	wait_for_completion(&xprtp->sft_close_complete);

	destroy_workqueue(xprtp->wq);
	dev_set_drvdata(&rpdev->dev, NULL);
	kfree(xprtp);
	pr_info("%s: %s down\n", __func__, XPRT_NAME);
}

static const struct rpmsg_device_id ipcr_rpmsg_match[] = {
	{ .name = "IPCRTR" },
	{}
};

static struct rpmsg_driver ipcr_rpmsg_xprt_driver = {
	.probe = ipcr_rpmsg_probe,
	.remove = ipcr_rpmsg_remove,
	.callback = ipcr_rpmsg_callback,
	.id_table = ipcr_rpmsg_match,
	.drv = {
		.name = "ipc_router_rpmsg_xprt",
	},
};
module_rpmsg_driver(ipcr_rpmsg_xprt_driver);

MODULE_DESCRIPTION("IPC Router RPMSG XPRT (pepito qmux bring-up)");
MODULE_LICENSE("GPL v2");
