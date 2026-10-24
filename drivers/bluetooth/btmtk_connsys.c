// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6878 connsys Bluetooth (BGF subsystem)
 *
 * The Bluetooth controller lives in the "BGF" subsystem of the MediaTek
 * connectivity combo ("connsys"), together with WiFi, GPS and FM. The connsys
 * power domain, the shared A-die (MT6637) and the analog supplies are brought
 * up by the mtk-connsys driver; the bootloader stages the BGF firmware into the
 * connsys EMI. This driver powers the BGF subsystem on top of that, exchanges
 * HCI with the firmware over a shared-memory ring in EMI ("BEIF") and presents
 * a standard HCI device to the Bluetooth stack.
 *
 * Register sequences follow the MediaTek downstream btmtk BTIF driver.
 *
 * The firmware MCU is brought alive at probe time; the HCI device then runs on
 * top: a polled reader drains the C2H half of the ring, TX pushes H4 frames
 * into the H2C half and rings a doorbell, and a vendor "WMT" command enables
 * the Bluetooth function before normal HCI traffic flows.
 */

#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/soc/mediatek/mtk-connsys.h>
#include <linux/unaligned.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

/*
 * Two register windows. Most of the bring-up touches the conn_infra register
 * space at 0x18000000; a handful of BGF "D-die" registers live in a separate
 * aperture at 0x18800000. Both are mapped without requesting the regions, as
 * they are shared with the connsys and WiFi drivers.
 */
#define CONN_WIN_BASE			0x18000000
#define CONN_WIN_SIZE			0x61000
#define BGF_WIN_BASE			0x18800000
#define BGF_WIN_SIZE			0x21000

/* conn_infra wake / ready (conn window) */
#define CONN_INFRA_WAKEUP_BT		0x180601a8	/* BIT0 = force conn_infra on */
#define CONN_INFRA_CFG_VERSION		0x18011000	/* == CONN_INFRA_CFG_ID when ready */
#define CONN_INFRA_CFG_ID		0x02050401
#define CONN_INFRA_CFG_PWRCTRL1		0x18001210	/* BIT16 = conn_infra ready */
#define CONN_INFRA_RDY_B		BIT(16)
#define CONN_INFRA_CFG_HOST_CSR		0x18001600	/* write 0x3ff to clear */
#define CONN_INFRA_HOST_CSR_CLR		0x3ff

/* BGFSYS power / reset (conn window) */
#define RGU_BGFSYS_ON_TOP_PWR_CTL	0x18000020
#define RGU_BGFSYS_CPU_SW_RST		0x18000124
#define BGF_CPU_SW_RST_B		BIT(0)
#define RGU_BGFSYS_ON_TOP_PWR_ACK	0x18000414
#define RGU_BGFSYS_OFF_TOP_PWR_ACK	0x18000424
#define BGF_OFF_PWR_ACK_B		BIT(24)
#define BGF_OFF_PWR_ACK_S		BIT(0)
#define CFG_BT_PWRCTLCR0		0x18001208
#define BT_FUNC_EN_B			BIT(0)

/* CONN2BT / BT2CONN GALS sleep-protect (conn window) */
#define CONN2BT_GALS_SLP_CTL		0x18001450
#define CONN2BT_GALS_SLP_STATUS		0x18001454
#define BT2CONN_GALS_SLP_CTL		0x18001460
#define BT2CONN_GALS_SLP_STATUS		0x18001464
#define GALS_SLP_PROT_TX_EN_B		BIT(0)
#define GALS_SLP_PROT_RX_EN_B		BIT(4)
#define GALS_SLP_PROT_RX_ACK_B		BIT(22)
#define GALS_SLP_PROT_TX_ACK_B		BIT(23)

/* connsys host-CSR-top ("SPM") window (conn window) */
#define CONNSYS_PWR_STATES		0x18060a10
#define BGFSYS_ON_ISO_EN		GENMASK(19, 18)
#define BGFSYS_ON_ISO_EN_VAL		BIT(18)
#define BGF_MONFLG_ON_OUT		0x18060c00
#define BGF_CONNINFRA_READY_B		BIT(15)
#define CR_HOSTCSR2BGF_ON_DBG_SEL	0x18060c04
#define BGF_IRQ_STAT			0x18060034
#define BGF_IRQ_STAT2			0x1806003c
#define BGF_IRQ_FW_OWN_B		BIT(0)

/* conn_infra SYSRAM debug (conn window) */
#define BGF_MCU_PC			0x18023a00
#define BGF_MCU_PC_DBG_SEL		0x18023a04

/* SW IRQ / doorbell (conn window, absolute) */
#define BGF_MCUCIRQ			0x1803f00c	/* TX doorbell to FW */
#define BGF_BTIF0_WAKEUP_OUT		(BIT(0) | BIT(1))
#define BGF_SW_IRQ_STATUS		0x1803f010
#define BGF_SW_IRQ_RESET		0x1803f014
#define BGF_SW_IRQ_FW_LOG_B		BIT(0)
#define BGF_SW_IRQ_SUBSYS_RST_B		BIT(1)

/* BGF D-die registers (BGF window) */
#define BGF_MCCR			0x18800100
#define BGF_MCCR_AHB_AUTO_DIS_B		BIT(31)
#define BGF_MCCR_SET			0x18800104
#define BGF_MCCR_AHB_STOP		(BIT(4) | BIT(5) | BIT(8))
#define BGF_MCU_CFG_SW_DBG_CTL		0x1880016c	/* == 0x1d1e when ROM alive */
#define BGF_ROM_IDLE			0x1d1e
#define BGF_IP_VERSION			0x18812000	/* == 0x02090000 when alive */
#define BGF_IP_VERSION_ALT		0x18810000	/* spec banner disagrees */
#define BGF_IP_VER_ID			0x02090000
#define BGF_HCLK_FRZ_WIN_EN		0x18812100
#define BGF_HCLK_SWITCH_FRZ_WIN_B	BIT(2)
#define BGF_MCUSYS_DLY_CHAIN_CTL	0x18820284

/* BEIF shared-memory HCI ring in connsys EMI */
#define BEIF_EMI_OFFSET			0xba0000
#define BEIF_EMI_SIZE			0x8000
#define BEIF_HEADER_KEY			0x55665566
#define BEIF_GUARD			0xdeadbeef

/* beif_header_t: 8 x u32 at ring offset 0 */
#define BEIF_HDR_KEY			0x00
#define BEIF_HDR_VERSION		0x04
#define BEIF_HDR_HOST_CTRL_OFF		0x08
#define BEIF_HDR_FW_CTRL_OFF		0x0c
#define BEIF_HDR_H2C_BUF_OFF		0x10
#define BEIF_HDR_H2C_BUF_SIZE		0x14
#define BEIF_HDR_C2H_BUF_OFF		0x18
#define BEIF_HDR_C2H_BUF_SIZE		0x1c
/* two beif_ctrl_t (8 x u32 each) follow the header */
#define BEIF_HOST_CTRL_OFF		0x20
#define BEIF_FW_CTRL_OFF		0x40
#define BEIF_CTRL_TX_PUSH		0x00
#define BEIF_CTRL_RX_POP		0x04
#define BEIF_CTRL_GUARDP		0x1c
#define BEIF_BUF_OFF			0x60	/* H2C starts here */
#define BEIF_TEMP_BUF_SIZE		2048	/* max drained per chunk */

/*
 * WMT: a MediaTek vendor protocol carried inside the normal HCI H4 stream.
 * Command = HCI vendor command 0xFC6F; the reply arrives as HCI event 0xE4.
 */
#define WMT_HCI_OPCODE			0xfc6f
#define WMT_EVENT_CODE			0xe4
#define WMT_DIR_CMD			0x01
#define WMT_DIR_EVT			0x02
#define WMT_OP_FUNC_CTRL		0x06

/* H4 packet types */
#define H4_CMD				0x01
#define H4_ACL				0x02
#define H4_SCO				0x03
#define H4_EVT				0x04

struct btmtk_connsys {
	struct device *dev;
	struct mtk_connsys *cs;
	void __iomem *conn;
	void __iomem *bgf;
	void __iomem *beif;		/* EMI ring */
	size_t beif_buf_size;
	u32 h2c_off, c2h_off;

	struct hci_dev *hdev;
	struct mutex tx_lock;		/* serializes H2C pushes */

	struct task_struct *rx_thread;
	struct sk_buff *rx_skb;		/* HCI frame being reassembled */
	u32 rx_want;			/* bytes still needed for rx_skb */
	u8 rx_hdr;			/* header bytes gathered so far */
	u8 rx_tmp[BEIF_TEMP_BUF_SIZE];	/* drained chunk (rx thread only) */

	/* Synchronous WMT exchange, serviced from the rx path */
	struct completion wmt_done;
	bool wmt_pending;
	u8 wmt_op;			/* opcode awaited */
	int wmt_status;			/* <0 error, else event status byte */
};

static void __iomem *bt_io(struct btmtk_connsys *bt, u32 addr)
{
	if (addr >= BGF_WIN_BASE && addr < BGF_WIN_BASE + BGF_WIN_SIZE)
		return bt->bgf + (addr - BGF_WIN_BASE);
	return bt->conn + (addr - CONN_WIN_BASE);
}

static u32 bt_rd(struct btmtk_connsys *bt, u32 addr)
{
	return readl(bt_io(bt, addr));
}

static void bt_wr(struct btmtk_connsys *bt, u32 addr, u32 val)
{
	writel(val, bt_io(bt, addr));
}

static void bt_rmw(struct btmtk_connsys *bt, u32 addr, u32 mask, u32 val)
{
	void __iomem *p = bt_io(bt, addr);

	writel((readl(p) & ~mask) | (val & mask), p);
}

static void bt_set(struct btmtk_connsys *bt, u32 addr, u32 bits)
{
	bt_rmw(bt, addr, bits, bits);
}

static void bt_clr(struct btmtk_connsys *bt, u32 addr, u32 bits)
{
	bt_rmw(bt, addr, bits, 0);
}

static int bt_poll(struct btmtk_connsys *bt, u32 addr, u32 mask, u32 expect,
		   unsigned int delay_us, unsigned int timeout_us)
{
	void __iomem *p = bt_io(bt, addr);
	u32 val;

	return readl_poll_timeout(p, val, (val & mask) == expect, delay_us, timeout_us);
}

/* Wake conn_infra and wait until it reports ready. Leaves force-on asserted. */
static int bgf_conninfra_wakeup(struct btmtk_connsys *bt)
{
	int ret;

	bt_set(bt, CONN_INFRA_WAKEUP_BT, BIT(0));
	udelay(200);

	ret = bt_poll(bt, CONN_INFRA_WAKEUP_BT, ~0, BIT(0), 1000, 10000);
	if (ret) {
		dev_err(bt->dev, "conn_infra force-on not acked\n");
		return ret;
	}

	ret = bt_poll(bt, CONN_INFRA_CFG_VERSION, ~0, CONN_INFRA_CFG_ID, 1000, 20000);
	if (ret) {
		dev_err(bt->dev, "conn_infra cfg version 0x%08x\n",
			bt_rd(bt, CONN_INFRA_CFG_VERSION));
		return ret;
	}

	ret = bt_poll(bt, CONN_INFRA_CFG_PWRCTRL1, CONN_INFRA_RDY_B,
		      CONN_INFRA_RDY_B, 1000, 20000);
	if (ret)
		dev_err(bt->dev, "conn_infra not ready\n");

	return ret;
}

static void bgf_conninfra_release(struct btmtk_connsys *bt)
{
	bt_clr(bt, CONN_INFRA_WAKEUP_BT, BIT(0));
}

/* Disable one GALS sleep-protect channel and wait for the ack to clear. */
static int bgf_gals_unprotect(struct btmtk_connsys *bt, u32 ctl, u32 status,
			      u32 en_bit, u32 ack_bit)
{
	bt_clr(bt, ctl, en_bit);
	return bt_poll(bt, status, ack_bit, 0, 500, 50000);
}

/*
 * Bring the BGF subsystem out of reset and wait for the firmware MCU to reach
 * its ROM idle loop. conn_infra must already be awake (force-on asserted).
 */
static int bgf_power_on(struct btmtk_connsys *bt)
{
	u32 ver, pwr;
	int ret;

	/* Hold the BGF CPU in reset while powering the subsystem */
	bt_clr(bt, RGU_BGFSYS_CPU_SW_RST, BGF_CPU_SW_RST_B);

	/* Power on the BGFSYS-ON domain (key 0x4254 'BT', BIT7 power control) */
	pwr = bt_rd(bt, RGU_BGFSYS_ON_TOP_PWR_CTL) & ~(GENMASK(31, 16) | BIT(7));
	bt_wr(bt, RGU_BGFSYS_ON_TOP_PWR_CTL, pwr | 0x42540080);

	ret = bt_poll(bt, CONNSYS_PWR_STATES, BGFSYS_ON_ISO_EN,
		      BGFSYS_ON_ISO_EN_VAL, 500, 50000);
	if (ret) {
		dev_err(bt->dev, "BGFSYS iso-enable timeout (0x%08x)\n",
			bt_rd(bt, CONNSYS_PWR_STATES));
		return ret;
	}

	/* Enable the BT function */
	bt_set(bt, CFG_BT_PWRCTLCR0, BT_FUNC_EN_B);

	ret = bt_poll(bt, RGU_BGFSYS_OFF_TOP_PWR_ACK, BGF_OFF_PWR_ACK_B,
		      BGF_OFF_PWR_ACK_B, 500, 50000);
	if (!ret)
		ret = bt_poll(bt, RGU_BGFSYS_OFF_TOP_PWR_ACK, BGF_OFF_PWR_ACK_S,
			      BGF_OFF_PWR_ACK_S, 500, 50000);
	if (ret) {
		dev_err(bt->dev, "BGFSYS off-power not acked (0x%08x)\n",
			bt_rd(bt, RGU_BGFSYS_OFF_TOP_PWR_ACK));
		return ret;
	}

	/* Connect BGF to the conn_infra bus: drop the GALS sleep-protection */
	ret = bgf_gals_unprotect(bt, CONN2BT_GALS_SLP_CTL, CONN2BT_GALS_SLP_STATUS,
				 GALS_SLP_PROT_RX_EN_B, GALS_SLP_PROT_RX_ACK_B);
	if (!ret)
		ret = bgf_gals_unprotect(bt, CONN2BT_GALS_SLP_CTL, CONN2BT_GALS_SLP_STATUS,
					 GALS_SLP_PROT_TX_EN_B, GALS_SLP_PROT_TX_ACK_B);
	if (!ret)
		ret = bgf_gals_unprotect(bt, BT2CONN_GALS_SLP_CTL, BT2CONN_GALS_SLP_STATUS,
					 GALS_SLP_PROT_RX_EN_B, GALS_SLP_PROT_RX_ACK_B);
	if (!ret)
		ret = bgf_gals_unprotect(bt, BT2CONN_GALS_SLP_CTL, BT2CONN_GALS_SLP_STATUS,
					 GALS_SLP_PROT_TX_EN_B, GALS_SLP_PROT_TX_ACK_B);
	if (ret) {
		dev_err(bt->dev, "GALS sleep-protect handshake timeout\n");
		return ret;
	}

	/* First access to the BGF D-die: the IP version must read back */
	udelay(400);
	ver = bt_rd(bt, BGF_IP_VERSION);
	if (ver != BGF_IP_VER_ID) {
		u32 alt = bt_rd(bt, BGF_IP_VERSION_ALT);

		dev_err(bt->dev, "BGF IP version 0x%08x (alt 0x%08x), expected 0x%08x\n",
			ver, alt, BGF_IP_VER_ID);
		if (alt != BGF_IP_VER_ID)
			return -ENODEV;
	}

	/* Clock / AHB setup before releasing the CPU */
	bt_clr(bt, BGF_MCCR, BGF_MCCR_AHB_AUTO_DIS_B);
	bt_wr(bt, BGF_MCCR_SET, BGF_MCCR_AHB_STOP);
	bt_rmw(bt, BGF_MCUSYS_DLY_CHAIN_CTL, GENMASK(19, 16), 0x80000);
	bt_set(bt, BGF_HCLK_FRZ_WIN_EN, BGF_HCLK_SWITCH_FRZ_WIN_B);

	/* Debug-select (preserving bits 31:22), then wait for conn_infra restore */
	bt_rmw(bt, CR_HOSTCSR2BGF_ON_DBG_SEL, GENMASK(21, 0), 0x300001);
	ret = bt_poll(bt, BGF_MONFLG_ON_OUT, BGF_CONNINFRA_READY_B,
		      BGF_CONNINFRA_READY_B, 500, 50000);
	if (ret) {
		dev_err(bt->dev, "BGF conn_infra restore timeout\n");
		return ret;
	}

	/* Clear the fw-own IRQ latches */
	bt_wr(bt, BGF_IRQ_STAT, BGF_IRQ_FW_OWN_B);
	bt_wr(bt, BGF_IRQ_STAT2, BGF_IRQ_FW_OWN_B);

	/* Release the BGF CPU (N10 core) from reset */
	bt_set(bt, RGU_BGFSYS_CPU_SW_RST, BGF_CPU_SW_RST_B);

	bt_rmw(bt, CR_HOSTCSR2BGF_ON_DBG_SEL, GENMASK(21, 0), 0);
	bt_wr(bt, BGF_MCU_PC_DBG_SEL, 0xc0010d2a);

	/* Wait for the firmware ROM to reach its idle loop */
	ret = bt_poll(bt, BGF_MCU_CFG_SW_DBG_CTL, ~0, BGF_ROM_IDLE, 5000, 500000);
	if (ret) {
		dev_err(bt->dev, "BGF MCU not alive: SW_DBG_CTL 0x%08x, PC 0x%08x\n",
			bt_rd(bt, BGF_MCU_CFG_SW_DBG_CTL), bt_rd(bt, BGF_MCU_PC));
		return ret;
	}

	dev_info(bt->dev, "BGF MCU alive (IP 0x%08x, PC 0x%08x)\n",
		 ver, bt_rd(bt, BGF_MCU_PC));
	return 0;
}

/* (Re)initialize the BEIF ring header the firmware expects, verify EMI write. */
static int beif_reset(struct btmtk_connsys *bt)
{
	u32 key, g0;
	size_t bufsz;
	int i;

	bufsz = (BEIF_EMI_SIZE - BEIF_BUF_OFF - 2 * 32) / 2;
	bt->beif_buf_size = bufsz;
	bt->h2c_off = BEIF_BUF_OFF;
	bt->c2h_off = BEIF_BUF_OFF + bufsz + 32;

	writel(BEIF_HEADER_KEY, bt->beif + BEIF_HDR_KEY);
	writel(0, bt->beif + BEIF_HDR_VERSION);
	writel(BEIF_HOST_CTRL_OFF, bt->beif + BEIF_HDR_HOST_CTRL_OFF);
	writel(BEIF_FW_CTRL_OFF, bt->beif + BEIF_HDR_FW_CTRL_OFF);
	writel(bt->h2c_off, bt->beif + BEIF_HDR_H2C_BUF_OFF);
	writel(bufsz, bt->beif + BEIF_HDR_H2C_BUF_SIZE);
	writel(bt->c2h_off, bt->beif + BEIF_HDR_C2H_BUF_OFF);
	writel(bufsz, bt->beif + BEIF_HDR_C2H_BUF_SIZE);

	/* Zero the control blocks (tx_push/rx_pop), set the guard words */
	memset_io(bt->beif + BEIF_HOST_CTRL_OFF, 0, 32);
	memset_io(bt->beif + BEIF_FW_CTRL_OFF, 0, 32);
	writel(BEIF_GUARD, bt->beif + BEIF_HOST_CTRL_OFF + BEIF_CTRL_GUARDP);
	writel(BEIF_GUARD, bt->beif + BEIF_FW_CTRL_OFF + BEIF_CTRL_GUARDP);

	/* Guard bands after each buffer */
	for (i = 0; i < 8; i++) {
		writel(BEIF_GUARD, bt->beif + bt->h2c_off + bufsz + 4 * i);
		writel(BEIF_GUARD, bt->beif + bt->c2h_off + bufsz + 4 * i);
	}

	/* Confirm the AP really writes EMI (EMI MPU could block it) */
	key = readl(bt->beif + BEIF_HDR_KEY);
	g0 = readl(bt->beif + BEIF_HOST_CTRL_OFF + BEIF_CTRL_GUARDP);
	if (key != BEIF_HEADER_KEY || g0 != BEIF_GUARD) {
		dev_err(bt->dev, "BEIF EMI not writable: key 0x%08x guard 0x%08x\n",
			key, g0);
		return -EIO;
	}

	dev_dbg(bt->dev, "BEIF ring ready, %zu B/dir\n", bufsz);
	return 0;
}

/* Ring the TX doorbell so the firmware consumes the H2C ring. */
static void beif_notify_fw(struct btmtk_connsys *bt)
{
	bt_wr(bt, BGF_MCUCIRQ, BGF_BTIF0_WAKEUP_OUT);
}

/*
 * Push an H4 frame (one type byte + payload) into the H2C ring and ring the
 * doorbell. The ring keeps one byte free to tell full from empty apart.
 */
static int beif_send(struct btmtk_connsys *bt, u8 type, const u8 *data, u32 len)
{
	u32 total = len + 1;
	u32 sz = bt->beif_buf_size;
	void __iomem *buf = bt->beif + bt->h2c_off;
	u32 push, pop, space, first;
	int retry;

	mutex_lock(&bt->tx_lock);

	for (retry = 0; retry < 100; retry++) {
		push = readl(bt->beif + BEIF_HOST_CTRL_OFF + BEIF_CTRL_TX_PUSH);
		pop = readl(bt->beif + BEIF_FW_CTRL_OFF + BEIF_CTRL_RX_POP);
		space = push >= pop ? sz - (push - pop) - 1 : pop - push - 1;
		if (space >= total)
			break;
		udelay(100);
	}
	if (space < total) {
		mutex_unlock(&bt->tx_lock);
		return -EAGAIN;
	}

	/* type byte */
	writeb(type, buf + push);
	push = (push + 1) % sz;

	/* payload, split across the ring wrap if needed */
	first = min(len, sz - push);
	memcpy_toio(buf + push, data, first);
	if (first < len)
		memcpy_toio(buf, data + first, len - first);
	push = (push + len) % sz;

	writel(push, bt->beif + BEIF_HOST_CTRL_OFF + BEIF_CTRL_TX_PUSH);
	beif_notify_fw(bt);

	mutex_unlock(&bt->tx_lock);
	return 0;
}

static int btmtk_wmt_event(struct btmtk_connsys *bt, const u8 *evt, u32 len);

/* Feed received bytes through an H4 reassembler, dispatch complete frames. */
static void btmtk_rx_bytes(struct btmtk_connsys *bt, const u8 *buf, u32 len)
{
	static const struct { u8 hlen, loff, lsize; } h4[] = {
		[H4_ACL] = { 4, 2, 2 },
		[H4_SCO] = { 3, 2, 1 },
		[H4_EVT] = { 2, 1, 1 },
	};

	while (len) {
		u8 type;
		u32 n;

		if (!bt->rx_skb) {
			type = *buf++;
			len--;
			if (type >= ARRAY_SIZE(h4) || !h4[type].hlen) {
				dev_warn_ratelimited(bt->dev, "bad H4 type 0x%02x\n",
						     type);
				continue;
			}
			bt->rx_skb = bt_skb_alloc(HCI_MAX_FRAME_SIZE, GFP_ATOMIC);
			if (!bt->rx_skb)
				return;
			hci_skb_pkt_type(bt->rx_skb) = type;
			bt->rx_hdr = 0;
			bt->rx_want = h4[type].hlen;	/* header first */
		}

		type = hci_skb_pkt_type(bt->rx_skb);
		n = min(bt->rx_want, len);
		skb_put_data(bt->rx_skb, buf, n);
		buf += n;
		len -= n;
		bt->rx_want -= n;
		if (bt->rx_want)
			return;

		if (bt->rx_hdr < h4[type].hlen) {
			/* Header complete: read the payload length field */
			const u8 *h = bt->rx_skb->data + h4[type].loff;
			u32 plen = h4[type].lsize == 2 ? get_unaligned_le16(h) : *h;

			bt->rx_hdr = h4[type].hlen;
			bt->rx_want = plen;
			if (plen)
				continue;
		}

		/* Full frame */
		if (type == H4_EVT && bt->rx_skb->data[0] == WMT_EVENT_CODE) {
			btmtk_wmt_event(bt, bt->rx_skb->data, bt->rx_skb->len);
			kfree_skb(bt->rx_skb);
		} else {
			hci_recv_frame(bt->hdev, bt->rx_skb);
		}
		bt->rx_skb = NULL;
	}
}

/* Drain whatever the firmware has placed in the C2H ring. */
static void beif_rx_drain(struct btmtk_connsys *bt)
{
	u32 sz = bt->beif_buf_size;
	void __iomem *buf = bt->beif + bt->c2h_off;
	int loops;

	for (loops = 0; loops < 16; loops++) {
		u32 pop = readl(bt->beif + BEIF_HOST_CTRL_OFF + BEIF_CTRL_RX_POP);
		u32 push = readl(bt->beif + BEIF_FW_CTRL_OFF + BEIF_CTRL_TX_PUSH);
		u32 n;

		if (push == pop)
			return;
		if (push > pop)
			n = min(push - pop, (u32)BEIF_TEMP_BUF_SIZE);
		else
			n = min(sz - pop, (u32)BEIF_TEMP_BUF_SIZE);

		memcpy_fromio(bt->rx_tmp, buf + pop, n);
		btmtk_rx_bytes(bt, bt->rx_tmp, n);

		pop = (pop + n) % sz;
		writel(pop, bt->beif + BEIF_HOST_CTRL_OFF + BEIF_CTRL_RX_POP);
	}
}

static int btmtk_rx_thread(void *data)
{
	struct btmtk_connsys *bt = data;

	while (!kthread_should_stop()) {
		beif_rx_drain(bt);
		usleep_range(1000, 2000);
	}
	return 0;
}

/* Record the status of the WMT event we are waiting for and wake the sender. */
static int btmtk_wmt_event(struct btmtk_connsys *bt, const u8 *evt, u32 len)
{
	/* evt: [code=0xE4][hci_len][dir][op][param_len16][status...] */
	if (!bt->wmt_pending || len < 7)
		return -EINVAL;
	if (evt[2] != WMT_DIR_EVT || evt[3] != bt->wmt_op)
		return -EINVAL;

	bt->wmt_status = evt[6];	/* first payload byte: 0 = success */
	bt->wmt_pending = false;
	complete(&bt->wmt_done);
	return 0;
}

/* Send one WMT command and wait for its event. */
static int btmtk_wmt_cmd(struct btmtk_connsys *bt, u8 op, const u8 *param, u16 plen)
{
	u8 buf[16];
	int ret;

	if (plen + 6 > sizeof(buf))
		return -EINVAL;

	/* HCI vendor command 0xFC6F wrapping the WMT header + param */
	buf[0] = WMT_HCI_OPCODE & 0xff;		/* opcode LSB */
	buf[1] = WMT_HCI_OPCODE >> 8;		/* opcode MSB */
	buf[2] = 4 + plen;			/* HCI parameter length */
	buf[3] = WMT_DIR_CMD;
	buf[4] = op;
	buf[5] = plen & 0xff;
	buf[6] = plen >> 8;
	memcpy(buf + 7, param, plen);

	bt->wmt_op = op;
	bt->wmt_status = -ETIMEDOUT;
	bt->wmt_pending = true;
	reinit_completion(&bt->wmt_done);

	ret = beif_send(bt, H4_CMD, buf, 7 + plen);
	if (ret) {
		bt->wmt_pending = false;
		return ret;
	}

	if (!wait_for_completion_timeout(&bt->wmt_done, msecs_to_jiffies(2000))) {
		bt->wmt_pending = false;
		dev_err(bt->dev, "WMT op 0x%02x timed out\n", op);
		return -ETIMEDOUT;
	}
	if (bt->wmt_status) {
		dev_err(bt->dev, "WMT op 0x%02x failed, status %d\n", op, bt->wmt_status);
		return -EIO;
	}
	return 0;
}

static int btmtk_hci_open(struct hci_dev *hdev)
{
	struct btmtk_connsys *bt = hci_get_drvdata(hdev);
	int ret;

	/* Keep conn_infra awake for the doorbell writes while the HCI is up */
	ret = bgf_conninfra_wakeup(bt);
	if (ret)
		return ret;

	ret = beif_reset(bt);
	if (ret)
		goto err_release;

	bt->rx_skb = NULL;
	bt->rx_thread = kthread_run(btmtk_rx_thread, bt, "btmtk_connsys_rx");
	if (IS_ERR(bt->rx_thread)) {
		ret = PTR_ERR(bt->rx_thread);
		goto err_release;
	}

	return 0;

err_release:
	bgf_conninfra_release(bt);
	return ret;
}

static int btmtk_hci_close(struct hci_dev *hdev)
{
	struct btmtk_connsys *bt = hci_get_drvdata(hdev);

	if (!IS_ERR_OR_NULL(bt->rx_thread))
		kthread_stop(bt->rx_thread);
	bt->rx_thread = NULL;

	kfree_skb(bt->rx_skb);
	bt->rx_skb = NULL;

	bgf_conninfra_release(bt);
	return 0;
}

static int btmtk_hci_flush(struct hci_dev *hdev)
{
	return 0;
}

static int btmtk_hci_setup(struct hci_dev *hdev)
{
	struct btmtk_connsys *bt = hci_get_drvdata(hdev);
	static const u8 func_on[] = { 0x00, 0x01 };	/* subsys 0 (BT), on */

	/* Enable the Bluetooth function in the firmware before normal traffic */
	return btmtk_wmt_cmd(bt, WMT_OP_FUNC_CTRL, func_on, sizeof(func_on));
}

/* Program the public BD address via the MediaTek vendor command. */
static int btmtk_hci_set_bdaddr(struct hci_dev *hdev, const bdaddr_t *bdaddr)
{
	struct sk_buff *skb;

	skb = __hci_cmd_sync(hdev, 0xfc1a, sizeof(*bdaddr), bdaddr, HCI_CMD_TIMEOUT);
	if (IS_ERR(skb))
		return PTR_ERR(skb);

	kfree_skb(skb);
	return 0;
}

static int btmtk_hci_send(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct btmtk_connsys *bt = hci_get_drvdata(hdev);
	int ret;

	ret = beif_send(bt, hci_skb_pkt_type(skb), skb->data, skb->len);
	if (ret) {
		hdev->stat.err_tx++;
		return ret;
	}

	switch (hci_skb_pkt_type(skb)) {
	case H4_CMD:
		hdev->stat.cmd_tx++;
		break;
	case H4_ACL:
		hdev->stat.acl_tx++;
		break;
	case H4_SCO:
		hdev->stat.sco_tx++;
		break;
	}

	kfree_skb(skb);
	return 0;
}

static int btmtk_connsys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct btmtk_connsys *bt;
	phys_addr_t emi;
	int ret;

	bt = devm_kzalloc(dev, sizeof(*bt), GFP_KERNEL);
	if (!bt)
		return -ENOMEM;

	bt->dev = dev;
	platform_set_drvdata(pdev, bt);
	mutex_init(&bt->tx_lock);
	init_completion(&bt->wmt_done);

	bt->cs = devm_mtk_connsys_get(dev);
	if (IS_ERR(bt->cs))
		return dev_err_probe(dev, PTR_ERR(bt->cs), "cannot get connsys\n");

	/* Shared with the connsys and WiFi drivers, so don't request the regions */
	bt->conn = devm_ioremap(dev, CONN_WIN_BASE, CONN_WIN_SIZE);
	bt->bgf = devm_ioremap(dev, BGF_WIN_BASE, BGF_WIN_SIZE);
	if (!bt->conn || !bt->bgf)
		return -ENOMEM;

	emi = mtk_connsys_emi_base(bt->cs);
	if (!emi)
		return -ENODEV;

	bt->beif = devm_ioremap(dev, emi + BEIF_EMI_OFFSET, BEIF_EMI_SIZE);
	if (!bt->beif)
		return -ENOMEM;

	ret = mtk_connsys_power_on(bt->cs, MTK_CONNSYS_BT);
	if (ret)
		return dev_err_probe(dev, ret, "connsys power on failed\n");

	ret = bgf_conninfra_wakeup(bt);
	if (ret)
		goto err_power_off;

	ret = bgf_power_on(bt);
	bgf_conninfra_release(bt);
	if (ret)
		goto err_power_off;

	bt->hdev = hci_alloc_dev();
	if (!bt->hdev) {
		ret = -ENOMEM;
		goto err_power_off;
	}

	bt->hdev->bus = HCI_UART;
	bt->hdev->manufacturer = 70;	/* MediaTek */
	hci_set_drvdata(bt->hdev, bt);
	SET_HCIDEV_DEV(bt->hdev, dev);
	bt->hdev->open = btmtk_hci_open;
	bt->hdev->close = btmtk_hci_close;
	bt->hdev->flush = btmtk_hci_flush;
	bt->hdev->setup = btmtk_hci_setup;
	bt->hdev->send = btmtk_hci_send;
	bt->hdev->set_bdaddr = btmtk_hci_set_bdaddr;
	hci_set_quirk(bt->hdev, HCI_QUIRK_NON_PERSISTENT_SETUP);
	/* The firmware has no valid address of its own; take it from the
	 * "local-bd-address" devicetree property and program it.
	 */
	hci_set_quirk(bt->hdev, HCI_QUIRK_USE_BDADDR_PROPERTY);
	/* The firmware advertises an extended-features page 2 whose declared
	 * features (e.g. the synchronization train) it does not implement.
	 */
	hci_set_quirk(bt->hdev, HCI_QUIRK_BROKEN_LOCAL_EXT_FEATURES_PAGE_2);

	ret = hci_register_dev(bt->hdev);
	if (ret) {
		dev_err(dev, "hci_register_dev failed: %d\n", ret);
		goto err_free_hdev;
	}

	dev_info(dev, "BGF Bluetooth ready (%s)\n", dev_name(&bt->hdev->dev));
	return 0;

err_free_hdev:
	hci_free_dev(bt->hdev);
	bt->hdev = NULL;
err_power_off:
	mtk_connsys_power_off(bt->cs, MTK_CONNSYS_BT);
	return ret;
}

static void btmtk_connsys_remove(struct platform_device *pdev)
{
	struct btmtk_connsys *bt = platform_get_drvdata(pdev);

	if (bt->hdev) {
		hci_unregister_dev(bt->hdev);
		hci_free_dev(bt->hdev);
	}
	mtk_connsys_power_off(bt->cs, MTK_CONNSYS_BT);
}

static const struct of_device_id btmtk_connsys_of_match[] = {
	{ .compatible = "mediatek,mt6878-bluetooth" },
	{ }
};
MODULE_DEVICE_TABLE(of, btmtk_connsys_of_match);

static struct platform_driver btmtk_connsys_driver = {
	.probe = btmtk_connsys_probe,
	.remove = btmtk_connsys_remove,
	.driver = {
		.name = "btmtk_connsys",
		.of_match_table = btmtk_connsys_of_match,
	},
};
module_platform_driver(btmtk_connsys_driver);

MODULE_DESCRIPTION("MediaTek MT6878 connsys Bluetooth driver");
MODULE_LICENSE("GPL");
