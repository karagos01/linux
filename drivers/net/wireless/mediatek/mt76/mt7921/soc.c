// SPDX-License-Identifier: BSD-3-Clause-Clear
/*
 * MediaTek MT7921 compatible WiFi integrated in the connectivity subsystem
 * (connsys) of SoCs like the MT6878.
 *
 * The WiFi subsystem is a CONNAC2 MT7961 attached to the AP through AXI. The
 * bootloader loads its firmware into the connsys memory, so the driver only
 * has to power the subsystem on, pass the board calibration data (NVRAM) to
 * the ROM code and start the firmware.
 */

#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/soc/mediatek/mtk-connsys.h>

#include "mt7921.h"
#include "../mt76_connac2_mac.h"
#include "../dma.h"
#include "mcu.h"
#include "soc.h"

#define MT7921_SOC_NVRAM		"mediatek/mt6878-wifi-nvram.bin"

/* conn_infra registers, as offsets in the connsys register space */
#define SOC_WFSYS_ON_TOP_PWR_CTL	0x000010
#define SOC_WFSYS_PWR_ON		BIT(7)
#define SOC_WFSYS_PWR_KEY		(0x5746 << 16)
#define SOC_WFSYS_CPU_SW_RST_B		0x000120
#define SOC_SEMA_M0_SW_RST_B		0x000158
#define SOC_WF_SLP_CTRL			0x001440
#define SOC_WF_SLP_PROT_SW_EN		BIT(0)
#define SOC_WF_SLP_STATUS		0x001444
#define SOC_WFDMA2CONN_SLP_PROT_RDY	BIT(25)
#define SOC_CONN2WF_SLP_PROT_RDY	BIT(29)
#define SOC_WF2CONN_SLP_PROT_HW_EN	BIT(30)
#define SOC_WF2CONN_SLP_PROT_RDY	BIT(31)
#define SOC_CONN_INFRA_PWRCTRL1		0x001210
#define SOC_CONN_INFRA_RDY		BIT(16)
#define SOC_HOST_CSR_IRQ_EN		0x001600
#define SOC_WB_SLP_TOP_CK_1		0x003124
#define SOC_CONN_HW_VER			0x011000
#define SOC_CONN_HW_VER_MT6878		0x02050401
#define SOC_EMI_CTL_WF			0x011114
#define SOC_CKGEN_COEX_0_SET		0x012064
#define SOC_CKGEN_COEX_0_CLR		0x012068
#define SOC_CKGEN_COEX_1_SET		0x012074
#define SOC_CKGEN_COEX_1_CLR		0x012078
#define SOC_COEX_0_CKEN			0x01010101
#define SOC_COEX_1_CKEN			0x00000101
#define SOC_CCIF4_PCCIF_RCHNUM		0x03d010
#define SOC_CCIF4_PCCIF_ACK		0x03d014
#define SOC_SW_INT_FW_LOG		BIT(0)
#define SOC_SW_INT_SUBSYS_RESET		BIT(1)
#define SOC_SW_INT_WHOLE_RESET		BIT(2)
#define SOC_CONN_INFRA_WAKEUP_WF	0x0601a4
#define SOC_CONNSYS_PWR_STATES		0x060a10
#define SOC_WFSYS_ON			BIT(30)
#define SOC_MCU_AWAKE			BIT(4)
#define SOC_WF_ON_MONFLG_EN		0x060b00
#define SOC_WF_ON_MONFLG_SEL		0x060b04
#define SOC_WF_ON_MONFLG_OUT		0x060b10
#define SOC_WF_ON_MONFLG_EINT_SEL	0x060b14
#define SOC_ROM_IDLE			0x1d1e
#define SOC_WF_SLPPROT_STATUS		0x4c300c
#define SOC_WF_SLPPROT_SRC1		BIT(23)
#define SOC_WF_SLPPROT_SRC2		BIT(21)
#define SOC_WF_IP_VER			0x4b0010
#define SOC_WF_IP_VER_MT6878		0x02040300
#define SOC_WF_HW_VER			0x4f0000


#define SOC_REG_SIZE			0x700000
#define SOC_REG_INVALID			U32_MAX

/* Release manifest of the firmware loaded by the bootloader */
#define SOC_EMI_FW_VER_OFFSET		0xb00024
#define SOC_EMI_FW_VER_LEN		128

/* ROM code commands */
#define MCU_CMD_SOC_PHY_ACTION		0x08
#define MCU_CMD_SOC_FW_IMAGE_START	0x42
#define MCU_CE_CMD_SOC_SET_NVRAM	0x48
#define MCU_CE_CMD_SOC_BASIC_CONFIG	0x02
#define MCU_CE_CMD_SOC_SET_DBDC_PARMS	0x28
#define MCU_CE_CMD_SOC_CUSTOMER_CFG	0x70
#define MCU_CE_CMD_SOC_SET_PWR_LIMIT	0x49

#define PHY_ACTION_MAGIC		0x556789aa
#define PHY_ACTION_VERSION		1
#define PHY_ACTION_FORCE_CAL_REQ	0x01

enum {
	PHY_ACTION_TAG_NVRAM,
	PHY_ACTION_TAG_CAL,
	PHY_ACTION_TAG_COM_FEM,
	PHY_ACTION_TAG_LAA,
};

/* NVRAM: settings in the first 256 bytes, then TLVs {id, le16 len, data} */
#define NVRAM_MAC_OFFSET		4
#define NVRAM_TLV_OFFSET		256
#define NVRAM_TLV_HDR_LEN		3
#define NVRAM_TAG_CTRL			0
#define NVRAM_TAG_2G4_COMMON		7
#define NVRAM_TAG_5G_COMMON		8
#define NVRAM_TAG_SYSTEM		9
#define NVRAM_TAG_6G_COMMON		21
#define NVRAM_TAG_VALID			BIT(7)
#define NVRAM_PHY_ACTION_MAX		1024
#define NVRAM_FRAGMENT_LEN		1500

/* The firmware crashes when the NVRAM fragments come too fast */
#define NVRAM_FRAGMENT_DELAY_MS		20

static struct mt7921_soc_dev *to_soc_dev(struct mt792x_dev *dev)
{
	return mt7921_soc_dev(dev);
}

struct mt7921_soc_reg_map {
	u32 phys;
	u32 maps;
	u32 size;
};

/* WFSYS address space as seen from the AP */
static const struct mt7921_soc_reg_map mt7921_soc_fixed_map[] = {
	{ 0x7c000000, 0x000000, 0x100000 }, /* CONN_INFRA */
	{ 0x18000000, 0x000000, 0x700000 }, /* CONN_INFRA, AP physical addresses */
	{ 0x830c0000, 0x400000, 0x001000 }, /* WF_MCU_BUS_CR_REMAP */
	{ 0x54000000, 0x402000, 0x001000 }, /* WFDMA PCIE0 MCU DMA0 */
	{ 0x55000000, 0x403000, 0x001000 }, /* WFDMA PCIE0 MCU DMA1 */
	{ 0x56000000, 0x404000, 0x001000 }, /* WFDMA reserved */
	{ 0x57000000, 0x405000, 0x001000 }, /* WFDMA MCU wrap CR */
	{ 0x58000000, 0x406000, 0x001000 }, /* WFDMA PCIE1 MCU DMA0 (MEM_DMA) */
	{ 0x59000000, 0x407000, 0x001000 }, /* WFDMA PCIE1 MCU DMA1 */
	{ 0x820c0000, 0x408000, 0x004000 }, /* WF_UMAC_TOP (PLE) */
	{ 0x820c8000, 0x40c000, 0x002000 }, /* WF_UMAC_TOP (PSE) */
	{ 0x820cc000, 0x40e000, 0x002000 }, /* WF_UMAC_TOP (PP), WF_MDP_TOP */
	{ 0x83000000, 0x410000, 0x010000 }, /* WF_PHY_MAP3 */
	{ 0x820e0000, 0x420000, 0x000400 }, /* WF_LMAC_TOP BN0 (WF_CFG) */
	{ 0x820e1000, 0x420400, 0x000200 }, /* WF_LMAC_TOP BN0 (WF_TRB) */
	{ 0x820e2000, 0x420800, 0x000400 }, /* WF_LMAC_TOP BN0 (WF_AGG) */
	{ 0x820e3000, 0x420c00, 0x000400 }, /* WF_LMAC_TOP BN0 (WF_ARB) */
	{ 0x820e4000, 0x421000, 0x000400 }, /* WF_LMAC_TOP BN0 (WF_TMAC) */
	{ 0x820e5000, 0x421400, 0x000800 }, /* WF_LMAC_TOP BN0 (WF_RMAC) */
	{ 0x820ce000, 0x421c00, 0x000200 }, /* WF_LMAC_TOP (WF_SEC) */
	{ 0x820e7000, 0x421e00, 0x000200 }, /* WF_LMAC_TOP BN0 (WF_DMA) */
	{ 0x820cf000, 0x422000, 0x001000 }, /* WF_LMAC_TOP (WF_PF) */
	{ 0x820e9000, 0x423400, 0x000200 }, /* WF_LMAC_TOP BN0 (WF_WTBLOFF) */
	{ 0x820ea000, 0x424000, 0x000200 }, /* WF_LMAC_TOP BN0 (WF_ETBF) */
	{ 0x820eb000, 0x424200, 0x000400 }, /* WF_LMAC_TOP BN0 (WF_LPON) */
	{ 0x820ec000, 0x424600, 0x000200 }, /* WF_LMAC_TOP BN0 (WF_INT) */
	{ 0x820ed000, 0x424800, 0x000800 }, /* WF_LMAC_TOP BN0 (WF_MIB) */
	{ 0x820ca000, 0x426000, 0x002000 }, /* WF_LMAC_TOP BN0 (WF_MUCOP) */
	{ 0x820d0000, 0x430000, 0x010000 }, /* WF_LMAC_TOP (WF_WTBLON) */
	{ 0x83080000, 0x450000, 0x010000 }, /* WF_PHY_MAP1 */
	{ 0x83090000, 0x460000, 0x010000 }, /* WF_PHY_MAP2 */
	{ 0xe0200000, 0x470000, 0x010000 }, /* WF_UMAC_SYSRAM */
	{ 0x00400000, 0x480000, 0x010000 }, /* WF_MCU_SYSRAM */
	{ 0x00410000, 0x490000, 0x010000 }, /* WF_MCU_SYSRAM (configure register) */
	{ 0x820f0000, 0x4a0000, 0x000400 }, /* WF_LMAC_TOP BN1 (WF_CFG) */
	{ 0x820f1000, 0x4a0600, 0x000200 }, /* WF_LMAC_TOP BN1 (WF_TRB) */
	{ 0x820f2000, 0x4a0800, 0x000400 }, /* WF_LMAC_TOP BN1 (WF_AGG) */
	{ 0x820f3000, 0x4a0c00, 0x000400 }, /* WF_LMAC_TOP BN1 (WF_ARB) */
	{ 0x820f4000, 0x4a1000, 0x000400 }, /* WF_LMAC_TOP BN1 (WF_TMAC) */
	{ 0x820f5000, 0x4a1400, 0x000800 }, /* WF_LMAC_TOP BN1 (WF_RMAC) */
	{ 0x820f7000, 0x4a1e00, 0x000200 }, /* WF_LMAC_TOP BN1 (WF_DMA) */
	{ 0x820f9000, 0x4a3400, 0x000200 }, /* WF_LMAC_TOP BN1 (WF_WTBLOFF) */
	{ 0x820fa000, 0x4a4000, 0x000200 }, /* WF_LMAC_TOP BN1 (WF_ETBF) */
	{ 0x820fb000, 0x4a4200, 0x000400 }, /* WF_LMAC_TOP BN1 (WF_LPON) */
	{ 0x820fc000, 0x4a4600, 0x000200 }, /* WF_LMAC_TOP BN1 (WF_INT) */
	{ 0x820fd000, 0x4a4800, 0x000800 }, /* WF_LMAC_TOP BN1 (WF_MIB) */
	{ 0x820c4000, 0x4a8000, 0x004000 }, /* WF_UMAC_TOP (UWTBL) */
	{ 0x820b0000, 0x4ae000, 0x001000 }, /* WFSYS_ON */
	{ 0x80020000, 0x4b0000, 0x010000 }, /* WF_TOP_MISC_OFF */
	{ 0x81020000, 0x4c0000, 0x010000 }, /* WF_TOP_MISC_ON */
	{ 0x80010000, 0x4d4000, 0x001000 }, /* WF_AXIDMA */
	{ 0x83010000, 0x4e0000, 0x010000 }, /* WF_PHY_MAP4 */
	{ 0x88000000, 0x4f0000, 0x010000 }, /* WF_MCU_CFG_LS */
};

/*
 * The common code uses the PCIe BAR layout of the discrete chips for a few
 * registers below 0x100000: translate them back to WFSYS addresses. Other
 * addresses in that range are connsys offsets.
 */
static const struct mt7921_soc_reg_map mt7921_pcie_map[] = {
	{ 0x54000000, 0x02000, 0x01000 }, /* WFDMA PCIE0 MCU DMA0 */
	{ 0x55000000, 0x03000, 0x01000 }, /* WFDMA PCIE0 MCU DMA1 */
	{ 0x58000000, 0x06000, 0x01000 }, /* WFDMA PCIE1 MCU DMA0 (MEM_DMA) */
	{ 0x59000000, 0x07000, 0x01000 }, /* WFDMA PCIE1 MCU DMA1 */
	{ 0x7c020000, 0xd0000, 0x10000 }, /* CONN_INFRA, WFDMA */
	{ 0x7c060000, 0xe0000, 0x10000 }, /* CONN_INFRA, conn_host_csr_top */
	{ 0x7c000000, 0xf0000, 0x10000 }, /* CONN_INFRA */
};

static u32 mt7921_soc_map(const struct mt7921_soc_reg_map *map, int n, u32 addr,
			  bool reverse)
{
	int i;

	for (i = 0; i < n; i++) {
		u32 from = reverse ? map[i].maps : map[i].phys;
		u32 to = reverse ? map[i].phys : map[i].maps;

		if (addr >= from && addr - from < map[i].size)
			return to + addr - from;
	}

	return SOC_REG_INVALID;
}

static u32 mt7921_soc_reg_addr(struct mt792x_dev *dev, u32 addr)
{
	u32 ofs;

	if (addr < 0x100000) {
		/* PCIe MAC and remap window don't exist here */
		if ((addr >= 0x10000 && addr < 0x20000) ||
		    (addr >= 0x40000 && addr < 0x50000))
			return SOC_REG_INVALID;

		ofs = mt7921_soc_map(mt7921_pcie_map, ARRAY_SIZE(mt7921_pcie_map),
				     addr, true);
		if (ofs == SOC_REG_INVALID)
			return addr;
		addr = ofs;
	}

	ofs = mt7921_soc_map(mt7921_soc_fixed_map, ARRAY_SIZE(mt7921_soc_fixed_map),
			     addr, false);
	if (ofs == SOC_REG_INVALID)
		dev_err_ratelimited(dev->mt76.dev,
				    "Access unsupported address %08x\n", addr);

	return ofs;
}

static u32 mt7921_soc_rr(struct mt76_dev *mdev, u32 offset)
{
	struct mt792x_dev *dev = container_of(mdev, struct mt792x_dev, mt76);
	u32 addr = mt7921_soc_reg_addr(dev, offset);

	if (addr == SOC_REG_INVALID)
		return 0;

	return dev->bus_ops->rr(mdev, addr);
}

/*
 * The firmware does not let the host control RX header translation per
 * station: keep it off, mac80211 gets 802.11 frames.
 */
static u32 mt7921_soc_reg_fixup(u32 offset, u32 val)
{
	if (offset == MT_MDP_DCR0)
		val &= ~MT_MDP_DCR0_RX_HDR_TRANS_EN;

	return val;
}

static void mt7921_soc_wr(struct mt76_dev *mdev, u32 offset, u32 val)
{
	struct mt792x_dev *dev = container_of(mdev, struct mt792x_dev, mt76);
	u32 addr = mt7921_soc_reg_addr(dev, offset);

	val = mt7921_soc_reg_fixup(offset, val);

	if (addr != SOC_REG_INVALID)
		dev->bus_ops->wr(mdev, addr, val);
}

static u32 mt7921_soc_rmw(struct mt76_dev *mdev, u32 offset, u32 mask, u32 val)
{
	struct mt792x_dev *dev = container_of(mdev, struct mt792x_dev, mt76);
	u32 addr = mt7921_soc_reg_addr(dev, offset);

	if (addr == SOC_REG_INVALID)
		return 0;

	if (offset == MT_MDP_DCR0) {
		u32 cur = dev->bus_ops->rr(mdev, addr);

		val = mt7921_soc_reg_fixup(offset, (cur & ~mask) | val);
		dev->bus_ops->wr(mdev, addr, val);
		return val;
	}

	return dev->bus_ops->rmw(mdev, addr, mask, val);
}

/* Direct accesses to the connsys register space */
static u32 soc_rr(struct mt792x_dev *dev, u32 ofs)
{
	return readl(dev->mt76.mmio.regs + ofs);
}

static void soc_wr(struct mt792x_dev *dev, u32 ofs, u32 val)
{
	writel(val, dev->mt76.mmio.regs + ofs);
}

static void soc_rmw(struct mt792x_dev *dev, u32 ofs, u32 mask, u32 val)
{
	soc_wr(dev, ofs, (soc_rr(dev, ofs) & ~mask) | val);
}

static int soc_poll(struct mt792x_dev *dev, u32 ofs, u32 mask, u32 val,
		    unsigned long delay_us, unsigned long timeout_us)
{
	u32 cur;

	return readl_poll_timeout(dev->mt76.mmio.regs + ofs, cur,
				  (cur & mask) == val, delay_us, timeout_us);
}

/* conn_infra may sleep, it has to be woken up to access it */
static int mt7921_soc_conninfra_wakeup(struct mt792x_dev *dev)
{
	int i, ret;

	for (i = 0; i < 10; i++) {
		soc_rmw(dev, SOC_CONN_INFRA_WAKEUP_WF, BIT(0), BIT(0));
		udelay(200);
		if (soc_rr(dev, SOC_CONN_INFRA_WAKEUP_WF) & BIT(0))
			break;
	}

	ret = soc_poll(dev, SOC_CONN_HW_VER, ~0, SOC_CONN_HW_VER_MT6878, 1000, 11000);
	if (ret) {
		dev_err(dev->mt76.dev, "conn_infra wakeup failed\n");
		return ret;
	}

	ret = soc_poll(dev, SOC_CONN_INFRA_PWRCTRL1, SOC_CONN_INFRA_RDY,
		       SOC_CONN_INFRA_RDY, 500, 5500);
	if (ret)
		dev_err(dev->mt76.dev, "conn_infra restore timeout\n");

	return ret;
}

static void mt7921_soc_conninfra_sleep(struct mt792x_dev *dev)
{
	soc_rmw(dev, SOC_CONN_INFRA_WAKEUP_WF, BIT(0), 0);
}

static int mt7921_soc_wfsys_on(struct mt792x_dev *dev)
{
	int ret;

	ret = mt7921_soc_conninfra_wakeup(dev);
	if (ret)
		return ret;

	soc_wr(dev, SOC_CKGEN_COEX_0_SET, SOC_COEX_0_CKEN);
	soc_wr(dev, SOC_CKGEN_COEX_1_SET, SOC_COEX_1_CKEN);

	/* Keep the WiFi MCU in reset while powering WFSYS on */
	soc_rmw(dev, SOC_WFSYS_CPU_SW_RST_B, BIT(0), 0);
	soc_rmw(dev, SOC_WF_SLP_CTRL, SOC_WF_SLP_PROT_SW_EN, 0);
	soc_rmw(dev, SOC_WFSYS_ON_TOP_PWR_CTL, GENMASK(31, 16) | SOC_WFSYS_PWR_ON,
		SOC_WFSYS_PWR_KEY | SOC_WFSYS_PWR_ON);

	ret = soc_poll(dev, SOC_CONNSYS_PWR_STATES, SOC_WFSYS_ON, SOC_WFSYS_ON,
		       500, 5500);
	if (ret) {
		dev_err(dev->mt76.dev, "WFSYS power on timeout\n");
		goto out;
	}

	ret = soc_poll(dev, SOC_WF_SLP_STATUS,
		       SOC_WF2CONN_SLP_PROT_RDY | SOC_CONN2WF_SLP_PROT_RDY, 0,
		       500, 50000);
	if (!ret)
		ret = soc_poll(dev, SOC_WF_SLPPROT_STATUS,
			       SOC_WF_SLPPROT_SRC1 | SOC_WF_SLPPROT_SRC2, 0,
			       500, 50000);
	if (ret) {
		dev_err(dev->mt76.dev, "WFSYS sleep protection timeout\n");
		goto out;
	}

	ret = soc_poll(dev, SOC_WF_IP_VER, ~0, SOC_WF_IP_VER_MT6878, 500, 5500);
	if (ret) {
		dev_err(dev->mt76.dev, "Unexpected WFSYS version 0x%08x\n",
			soc_rr(dev, SOC_WF_IP_VER));
		goto out;
	}

	/* Let the ROM code report its state in the mailbox */
	soc_rmw(dev, SOC_WF_ON_MONFLG_EN, BIT(0), BIT(0));
	soc_rmw(dev, SOC_WF_ON_MONFLG_SEL, GENMASK(4, 0), 0xc);

	soc_rmw(dev, SOC_WFSYS_CPU_SW_RST_B, BIT(0), BIT(0));

	ret = soc_poll(dev, SOC_WF_ON_MONFLG_OUT, ~0, SOC_ROM_IDLE, 1000, 1000000);
	if (ret)
		dev_err(dev->mt76.dev, "WiFi ROM code not ready (0x%08x)\n",
			soc_rr(dev, SOC_WF_ON_MONFLG_OUT));

out:
	mt7921_soc_conninfra_sleep(dev);

	return ret;
}

static void mt7921_soc_wfsys_off(struct mt792x_dev *dev)
{
	if (mt7921_soc_conninfra_wakeup(dev))
		return;

	soc_rmw(dev, SOC_WF_SLP_CTRL, SOC_WF_SLP_PROT_SW_EN, SOC_WF_SLP_PROT_SW_EN);
	if (soc_poll(dev, SOC_WF_SLP_STATUS, SOC_WFDMA2CONN_SLP_PROT_RDY,
		     SOC_WFDMA2CONN_SLP_PROT_RDY, 500, 50000))
		dev_warn(dev->mt76.dev, "WFDMA sleep protection timeout\n");
	if (soc_poll(dev, SOC_WF_SLP_STATUS,
		     SOC_WF2CONN_SLP_PROT_RDY | SOC_WF2CONN_SLP_PROT_HW_EN,
		     SOC_WF2CONN_SLP_PROT_RDY | SOC_WF2CONN_SLP_PROT_HW_EN,
		     500, 50000))
		dev_warn(dev->mt76.dev, "WFSYS sleep protection timeout\n");

	soc_rmw(dev, SOC_WFSYS_ON_TOP_PWR_CTL, GENMASK(31, 16) | SOC_WFSYS_PWR_ON,
		SOC_WFSYS_PWR_KEY);
	if (soc_poll(dev, SOC_CONNSYS_PWR_STATES, SOC_WFSYS_ON, 0, 500, 5500))
		dev_warn(dev->mt76.dev, "WFSYS power off timeout\n");

	/* Release the HW semaphores the WiFi MCU may hold */
	soc_rmw(dev, SOC_SEMA_M0_SW_RST_B, BIT(0), 0);
	soc_rmw(dev, SOC_SEMA_M0_SW_RST_B, BIT(0), BIT(0));

	soc_rmw(dev, SOC_CCIF4_PCCIF_ACK, 0xff, 0xff);

	/* A-die top clock request from the WiFi MCU */
	soc_rmw(dev, SOC_WB_SLP_TOP_CK_1, BIT(0), 0);
	if (soc_poll(dev, SOC_WB_SLP_TOP_CK_1, BIT(1), 0, 500, 50000))
		dev_warn(dev->mt76.dev, "A-die clock release timeout\n");

	/* Drop the EMI and infra requests of WFSYS */
	soc_rmw(dev, SOC_EMI_CTL_WF, BIT(0), BIT(0));
	soc_rmw(dev, SOC_EMI_CTL_WF, BIT(0), 0);
	soc_rmw(dev, SOC_EMI_CTL_WF, BIT(5), BIT(5));
	soc_rmw(dev, SOC_EMI_CTL_WF, BIT(5), 0);

	soc_wr(dev, SOC_CKGEN_COEX_0_CLR, SOC_COEX_0_CKEN);
	soc_wr(dev, SOC_CKGEN_COEX_1_CLR, SOC_COEX_1_CKEN);

	mt7921_soc_conninfra_sleep(dev);
}

static int mt7921_soc_power_on(struct mt792x_dev *dev)
{
	struct mt7921_soc_dev *sdev = to_soc_dev(dev);
	int ret;

	ret = mtk_connsys_power_on(sdev->cs, MTK_CONNSYS_WIFI);
	if (ret)
		return ret;

	ret = mt7921_soc_wfsys_on(dev);
	if (ret)
		mtk_connsys_power_off(sdev->cs, MTK_CONNSYS_WIFI);

	return ret;
}

static void mt7921_soc_power_off(struct mt792x_dev *dev)
{
	mt7921_soc_wfsys_off(dev);
	mtk_connsys_power_off(to_soc_dev(dev)->cs, MTK_CONNSYS_WIFI);
}

static int mt7921_soc_drv_own(struct mt792x_dev *dev)
{
	int i;

	/* Make sure the WFSYS interrupts reach the host */
	if (!mt7921_soc_conninfra_wakeup(dev))
		soc_wr(dev, SOC_HOST_CSR_IRQ_EN, 0x3ff);
	mt7921_soc_conninfra_sleep(dev);

	/* Wake the MCU up through its EINT if it sleeps */
	soc_rmw(dev, SOC_WF_ON_MONFLG_EN, BIT(0), BIT(0));
	soc_rmw(dev, SOC_WF_ON_MONFLG_SEL, GENMASK(4, 0), 0x6);
	soc_rmw(dev, SOC_WF_ON_MONFLG_EINT_SEL, GENMASK(2, 0), 0x1);

	for (i = 0; i < 3; i++) {
		bool awake = soc_rr(dev, SOC_CONNSYS_PWR_STATES) & SOC_MCU_AWAKE;
		u32 val;

		mt76_wr(dev, MT_CONN_ON_LPCTL, PCIE_LPCR_HOST_CLR_OWN);
		if (awake)
			break;

		udelay(156);
		val = soc_rr(dev, SOC_WF_ON_MONFLG_OUT);
		if (!(val & BIT(0)) || !(val & BIT(10)))
			break;
	}

	return __mt792xe_mcu_drv_pmctrl(dev);
}

static int mt7921_soc_drv_pmctrl(struct mt792x_dev *dev)
{
	struct mt76_phy *mphy = &dev->mt76.phy;
	struct mt76_connac_pm *pm = &dev->pm;
	int err;

	err = mt7921_soc_drv_own(dev);
	if (err < 0)
		return err;

	mt792x_wpdma_reinit_cond(dev);
	clear_bit(MT76_STATE_PM, &mphy->state);

	pm->stats.last_wake_event = jiffies;
	pm->stats.doze_time += pm->stats.last_wake_event -
			       pm->stats.last_doze_event;

	return 0;
}

/*
 * The firmware relies on the DMA scheduler quotas, without them the commands
 * can overrun it.
 */
/*
 * The WiFi MCU notifies the host through a CCIF channel, and waits for the
 * notification to be acknowledged before sending another one.
 */
static irqreturn_t mt7921_soc_ccif_irq(int irq, void *data)
{
	struct mt792x_dev *dev = data;
	u32 status;

	if (mt7921_soc_conninfra_wakeup(dev))
		return IRQ_NONE;

	status = soc_rr(dev, SOC_CCIF4_PCCIF_RCHNUM);
	soc_wr(dev, SOC_CCIF4_PCCIF_ACK, status);
	mt7921_soc_conninfra_sleep(dev);

	if (!status)
		return IRQ_NONE;

	if (status & (SOC_SW_INT_SUBSYS_RESET | SOC_SW_INT_WHOLE_RESET)) {
		dev_err(dev->mt76.dev, "Firmware requested a reset (0x%x)\n", status);
		mt792x_reset(&dev->mt76);
	}

	return IRQ_HANDLED;
}

static void mt7921_soc_dma_prefetch(struct mt792x_dev *dev)
{
#define PREFETCH(base, depth)	((base) << 16 | (depth))
	mt76_wr(dev, MT_WFDMA0_RX_RING0_EXT_CTRL, PREFETCH(0x000, 0x4));
	mt76_wr(dev, MT_WFDMA0_RX_RING1_EXT_CTRL, PREFETCH(0x040, 0x4));
	mt76_wr(dev, MT_WFDMA0_RX_RING2_EXT_CTRL, PREFETCH(0x080, 0x4));
	mt76_wr(dev, MT_WFDMA0_RX_RING3_EXT_CTRL, PREFETCH(0x0c0, 0x4));
	mt76_wr(dev, MT_WFDMA0_TX_RING0_EXT_CTRL, PREFETCH(0x200, 0x4));
	mt76_wr(dev, MT_WFDMA0_TX_RING1_EXT_CTRL, PREFETCH(0x240, 0x4));
	mt76_wr(dev, MT_WFDMA0_TX_RING2_EXT_CTRL, PREFETCH(0x280, 0x4));
	mt76_wr(dev, MT_WFDMA0_TX_RING15_EXT_CTRL, PREFETCH(0x2c0, 0x4));
	mt76_wr(dev, MT_WFDMA0_TX_RING16_EXT_CTRL, PREFETCH(0x300, 0x4));
#undef PREFETCH

	mt76_wr(dev, MT_WFDMA0_RST_DRX_PTR, ~0);
}

static int mt7921_soc_dma_init(struct mt792x_dev *dev)
{
	/* The DMA rings are accessed directly, with WFSYS offsets */
	static const struct mt792x_dma_layout layout = {
		.tx_data0 = mt792x_dma_ring(MT7921_TXQ_BAND0, MT7921_TX_RING_SIZE, 0x24300),
		.tx_mcu = mt792x_dma_ring(MT7902_TXQ_MCU_WM, MT7921_TX_MCU_RING_SIZE, 0x24300),
		.tx_fwdl = mt792x_dma_ring(MT7921_TXQ_FWDL, MT7921_TX_FWDL_RING_SIZE, 0x24300),
		.rx_data = mt792x_dma_ring(0, MT7921_RX_RING_SIZE, 0x24500),
		/* Events and TX free notifications */
		.rx_mcu = mt792x_dma_ring(2, MT7902_RX_MCU_RING_SIZE, 0x24500),
	};
	int ret;

	ret = mt792x_dma_alloc_queues(dev, &layout);
	if (ret)
		return ret;

	/* More TX free notifications, the data ring is only cleaned up on them */
	ret = mt76_queue_alloc(dev, &dev->mt76.q_rx[MT_RXQ_MCU_WA], 3,
			       MT7902_RX_MCU_RING_SIZE, MT_RX_BUF_SIZE, 0x24500);
	if (ret)
		return ret;

	ret = mt76_init_queues(dev, mt792x_poll_rx);
	if (ret < 0)
		return ret;

	netif_napi_add_tx(dev->mt76.tx_napi_dev, &dev->mt76.tx_napi,
			  mt792x_poll_tx);
	napi_enable(&dev->mt76.tx_napi);

	return mt792x_dma_enable(dev);
}

static int mt7921_soc_mcu_parse_response(struct mt76_dev *mdev, int cmd,
					 struct sk_buff *skb, int seq)
{
	struct mt76_connac2_mcu_rxd *rxd;

	if (cmd != MCU_CMD(SOC_PHY_ACTION) &&
	    cmd != MCU_CMD(SOC_FW_IMAGE_START))
		return mt7921_mcu_parse_response(mdev, cmd, skb, seq);

	if (!skb) {
		dev_err(mdev->dev, "Message %08x (seq %d) timeout\n", cmd, seq);
		return -ETIMEDOUT;
	}

	rxd = (struct mt76_connac2_mcu_rxd *)skb->data;
	if (seq != rxd->seq)
		return -EAGAIN;

	/* ROM code events have a shorter header */
	skb_pull(skb, sizeof(*rxd) - 4);

	return 0;
}

/*
 * The firmware only knows the first version of the domain information, with
 * the channels described as ranges: convert the channel list to it.
 */
static void mt7921_soc_domain_info_v1(struct mt792x_dev *dev, struct sk_buff *skb)
{
	struct {
		u8 alpha2[4];
		u8 bw_2g;
		u8 bw_5g;
		u8 bw_6g;
		u8 pad;
		u8 n_2ch;
		u8 n_5ch;
		u8 n_6ch;
		u8 pad2;
	} __packed *v2 = (void *)skb->data;
	struct {
		__le16 hw_value;
		__le16 pad;
		__le32 flags;
	} __packed *chan = (void *)(v2 + 1);
	struct {
		u8 reg_class;
		u8 band;
		u8 span;
		u8 first;
		u8 num;
		u8 rsv[3];
	} __packed *sb;
	struct {
		__le16 country;
		__le16 passive_scan;
		u8 subband[7][8];
		u8 bw_2g;
		u8 bw_5g;
		u8 rsv[2];
	} __packed v1 = {
		/* "00" is the world domain, known as 0 to the firmware */
		.country = cpu_to_le16(v2->alpha2[0] == '0' ? 0 :
				       v2->alpha2[0] << 8 | v2->alpha2[1]),
		.bw_2g = v2->bw_2g,
		.bw_5g = v2->bw_5g,
	};
	typeof(v1) passive;
	int i, n = 0, nch = v2->n_2ch + v2->n_5ch;

	if (skb->len < sizeof(*v2) + nch * sizeof(*chan))
		return;

	for (i = 0, sb = NULL; i < nch; i++) {
		u8 ch = le16_to_cpu(chan[i].hw_value);
		bool is_2g = i < v2->n_2ch;
		u8 span = is_2g ? 1 : 4;

		if (sb && sb->band == (is_2g ? 1 : 2) &&
		    ch == sb->first + sb->num * span) {
			sb->num++;
			continue;
		}

		if (n == ARRAY_SIZE(v1.subband))
			break;

		sb = (void *)v1.subband[n++];
		sb->band = is_2g ? 1 : 2;
		sb->span = span;
		sb->first = ch;
		sb->num = 1;
		if (is_2g)
			sb->reg_class = 81;
		else if (ch < 52)
			sb->reg_class = 115;
		else if (ch < 100)
			sb->reg_class = 118;
		else if (ch < 149)
			sb->reg_class = 121;
		else
			sb->reg_class = 125;
	}

	/* No channel only allowing passive scans */
	passive = v1;
	passive.passive_scan = cpu_to_le16(1);
	memset(passive.subband, 0, sizeof(passive.subband));
	mt7921_soc_legacy_send(dev, MCU_CE_CMD_SET_CHAN_DOMAIN, &passive, sizeof(passive));

	skb_trim(skb, 0);
	skb_put_data(skb, &v1, sizeof(v1));
}

static int mt7921_soc_mcu_send_message(struct mt76_dev *mdev, struct sk_buff *skb,
				       int cmd, int *seq)
{
	struct mt792x_dev *dev = container_of(mdev, struct mt792x_dev, mt76);
	int ret;

	if (cmd == MCU_CE_CMD(SET_CHAN_DOMAIN))
		mt7921_soc_domain_info_v1(dev, skb);

	if (mt7921_soc_legacy_cmd(dev, skb, cmd, seq, &ret)) {
		if (ret)
			dev_warn(mdev->dev, "Command %08x translation failed: %d\n", cmd, ret);
		return 0;
	}

	ret = mt76_connac2_mcu_fill_message(mdev, skb, cmd, seq);
	if (ret)
		return ret;

	/* The RF calibration takes a while */
	if (cmd == MCU_CMD(SOC_PHY_ACTION))
		mdev->mcu.timeout = 5 * HZ;
	else
		mdev->mcu.timeout = 3 * HZ;

	return mt76_tx_queue_skb_raw(dev, mdev->q_mcu[MT_MCUQ_WM], skb, 0);
}

/* Collects the NVRAM TLVs the ROM code needs for the RF calibration */
static int mt7921_soc_nvram_parse(struct mt7921_soc_dev *sdev)
{
	static const u8 rf_tags[] = {
		NVRAM_TAG_2G4_COMMON, NVRAM_TAG_5G_COMMON,
		NVRAM_TAG_6G_COMMON, NVRAM_TAG_SYSTEM,
	};
	const u8 *data = sdev->nvram->data;
	size_t size = sdev->nvram->size;
	u32 ofs = NVRAM_TLV_OFFSET, end;
	int len = 0;

	if (size < NVRAM_TLV_OFFSET + 6 || data[ofs] != NVRAM_TAG_CTRL)
		return -EINVAL;

	end = min_t(size_t, size, NVRAM_TLV_OFFSET + get_unaligned_le16(data + ofs + 4));

	sdev->nvram_rf = devm_kzalloc(sdev->dev.mt76.dev, NVRAM_PHY_ACTION_MAX, GFP_KERNEL);
	if (!sdev->nvram_rf)
		return -ENOMEM;

	while (ofs + NVRAM_TLV_HDR_LEN <= end) {
		u8 tag = data[ofs];
		u16 tlv_len = get_unaligned_le16(data + ofs + 1);
		u32 n = NVRAM_TLV_HDR_LEN + tlv_len;

		if (!tag && !tlv_len)
			break;
		if (ofs + n > size)
			return -EINVAL;

		if ((tag & NVRAM_TAG_VALID) || memchr(rf_tags, tag, sizeof(rf_tags))) {
			if (len + n > NVRAM_PHY_ACTION_MAX)
				return -E2BIG;
			memcpy(sdev->nvram_rf + len, data + ofs, n);
			len += n;
		}

		ofs += n;
	}

	sdev->nvram_rf_len = ALIGN(len, 4);

	return 0;
}

struct mt7921_soc_phy_tlv {
	__le16 tag;
	__le16 len;
} __packed;

struct mt7921_soc_phy_cal {
	u8 cmd;
	u8 save_result;
	u8 skip_cal;
	u8 rsv;
} __packed;

/* Front end module description, all zero: no external PA/LNA control */
struct mt7921_soc_phy_fem {
	__le32 fem_id;
	__le32 pin_count;
	u8 pins[32][3];
} __packed;

struct mt7921_soc_phy_laa {
	__le32 chip_id;
	__le32 pin_count;
	u8 pins[8][4];
} __packed;

static void *mt7921_soc_phy_tlv_put(u8 **buf, u16 tag, int len)
{
	struct mt7921_soc_phy_tlv *tlv = (struct mt7921_soc_phy_tlv *)*buf;

	tlv->tag = cpu_to_le16(tag);
	tlv->len = cpu_to_le16(len);
	*buf += sizeof(*tlv) + len;

	return tlv + 1;
}

/* Passes the board NVRAM to the ROM code, which calibrates the RF */
static int mt7921_soc_phy_action(struct mt792x_dev *dev)
{
	struct mt7921_soc_dev *sdev = to_soc_dev(dev);
	struct {
		__le32 magic;
		u8 tag_num;
		u8 version;
		__le16 len;
	} __packed *hdr;
	struct {
		struct mt7921_soc_phy_tlv tlv;
		u8 event;
		u8 status;
		u8 rsv[2];
		__le32 emi_addr;
		__le32 emi_len;
		__le32 temperature;
	} __packed *res;
	struct mt7921_soc_phy_cal *cal;
	struct sk_buff *skb;
	int len, ret;
	u8 *buf, *p;

	len = sizeof(*hdr) +
	      sizeof(struct mt7921_soc_phy_tlv) * 4 +
	      sizeof(*cal) + sdev->nvram_rf_len +
	      sizeof(struct mt7921_soc_phy_fem) +
	      sizeof(struct mt7921_soc_phy_laa);

	buf = kzalloc(len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	hdr = (void *)buf;
	hdr->magic = cpu_to_le32(PHY_ACTION_MAGIC);
	hdr->tag_num = 4;
	hdr->version = PHY_ACTION_VERSION;
	hdr->len = cpu_to_le16(len - sizeof(*hdr));

	p = buf + sizeof(*hdr);
	cal = mt7921_soc_phy_tlv_put(&p, PHY_ACTION_TAG_CAL, sizeof(*cal));
	cal->cmd = PHY_ACTION_FORCE_CAL_REQ;
	cal->save_result = 1;
	memcpy(mt7921_soc_phy_tlv_put(&p, PHY_ACTION_TAG_NVRAM, sdev->nvram_rf_len),
	       sdev->nvram_rf, sdev->nvram_rf_len);
	mt7921_soc_phy_tlv_put(&p, PHY_ACTION_TAG_COM_FEM, sizeof(struct mt7921_soc_phy_fem));
	mt7921_soc_phy_tlv_put(&p, PHY_ACTION_TAG_LAA, sizeof(struct mt7921_soc_phy_laa));

	ret = mt76_mcu_send_and_get_msg(&dev->mt76, MCU_CMD(SOC_PHY_ACTION),
					buf, len, true, &skb);
	kfree(buf);
	if (ret)
		return ret;

	if (skb->len < sizeof(*hdr) + sizeof(*res)) {
		ret = -EINVAL;
		goto out;
	}

	res = (void *)(skb->data + sizeof(*hdr));
	if (le16_to_cpu(res->tlv.tag) != PHY_ACTION_TAG_CAL || res->status) {
		dev_err(dev->mt76.dev, "RF calibration failed: tag %d status %d\n",
			le16_to_cpu(res->tlv.tag), res->status);
		ret = -EIO;
	}

out:
	dev_kfree_skb(skb);
	return ret;
}

/* Sends the NVRAM to the running firmware, one TLV per message */
static int mt7921_soc_nvram_send(struct mt792x_dev *dev)
{
	struct mt7921_soc_dev *sdev = to_soc_dev(dev);
	const u8 *data = sdev->nvram->data;
	size_t size = sdev->nvram->size;
	u32 start = 0, ofs = NVRAM_TLV_OFFSET;
	u8 *buf;
	int ret = 0;

	buf = kmalloc(NVRAM_FRAGMENT_LEN, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	/* The first message also carries the settings before the TLVs */
	while (ofs + NVRAM_TLV_HDR_LEN <= size) {
		u16 tlv_len = get_unaligned_le16(data + ofs + 1);

		if (!data[ofs] && !tlv_len)
			break;

		ofs += NVRAM_TLV_HDR_LEN + tlv_len;
		if (ofs > size || ofs - start > NVRAM_FRAGMENT_LEN) {
			ret = -EINVAL;
			break;
		}

		memset(buf, 0, NVRAM_FRAGMENT_LEN);
		memcpy(buf, data + start, ofs - start);
		ret = mt76_mcu_send_msg(&dev->mt76, MCU_CE_CMD(SOC_SET_NVRAM),
					buf, NVRAM_FRAGMENT_LEN, false);
		if (ret)
			break;

		/* The firmware crashes if the next fragment comes too early */
		msleep(NVRAM_FRAGMENT_DELAY_MS);

		start = ofs;
	}

	kfree(buf);

	return ret;
}

static void mt7921_soc_print_fw_version(struct mt792x_dev *dev)
{
	phys_addr_t emi = mtk_connsys_emi_base(to_soc_dev(dev)->cs);
	char ver[SOC_EMI_FW_VER_LEN + 1] = {};
	void *p;
	u32 ofs;

	p = memremap(emi + SOC_EMI_FW_VER_OFFSET, sizeof(u32), MEMREMAP_WC);
	if (!p)
		return;
	ofs = readl(p) & 0xffffff;
	memunmap(p);

	p = memremap(emi + ofs, SOC_EMI_FW_VER_LEN, MEMREMAP_WC);
	if (!p)
		return;
	memcpy_fromio(ver, p, SOC_EMI_FW_VER_LEN);
	memunmap(p);

	dev_info(dev->mt76.dev, "Firmware: %s\n", strim(ver));
}

static int mt7921_soc_load_firmware(struct mt792x_dev *dev)
{
	struct {
		__le32 override;
		__le32 addr;
	} __packed req = {};
	struct sk_buff *skb;
	int ret;

	ret = mt7921_soc_phy_action(dev);
	if (ret)
		return ret;

	/* Start the firmware the bootloader loaded */
	ret = mt76_mcu_send_and_get_msg(&dev->mt76, MCU_CMD(SOC_FW_IMAGE_START),
					&req, sizeof(req), true, &skb);
	if (ret)
		return ret;

	if (skb->len < 1 || skb->data[0]) {
		dev_err(dev->mt76.dev, "Firmware start failed: %d\n",
			skb->len ? skb->data[0] : -1);
		ret = -EIO;
	}
	dev_kfree_skb(skb);
	if (ret)
		return ret;

	if (!mt76_poll_msec(dev, MT_CONN_ON_MISC, MT_TOP_MISC2_FW_N9_RDY,
			    MT_TOP_MISC2_FW_N9_RDY, 1500)) {
		dev_err(dev->mt76.dev, "Timeout for initializing firmware\n");
		return -EIO;
	}

	return 0;
}

static int mt7921_soc_basic_config(struct mt792x_dev *dev)
{
	struct {
		u8 native_80211;
		u8 assert_path;
		u8 debug_level;
		u8 rsv;
		__le16 rx_csum;
		__le16 tx_csum;
		u8 segment;
		u8 rsv2[3];
	} __packed basic = {
		/* RX checksums are checked, frames are sent with theirs */
		.rx_csum = cpu_to_le16(0x7),
	};
	struct {
		u8 dbdc_en;
		u8 wmm_band_bitmap;
		u8 update_next_ch_req;
		u8 pad0;
		u8 cmd_ver;
		u8 dbdc_aa_mode;
		__le16 cmd_len;
		u8 rsv[28];
	} __packed dbdc = {
		.cmd_ver = 1,
		.cmd_len = cpu_to_le16(sizeof(dbdc)),
	};
	int ret;

	ret = mt76_mcu_send_msg(&dev->mt76, MCU_CE_CMD(SOC_BASIC_CONFIG), &basic,
				sizeof(basic), false);
	if (ret)
		return ret;

	return mt76_mcu_send_msg(&dev->mt76, MCU_CE_CMD(SOC_SET_DBDC_PARMS), &dbdc,
				 sizeof(dbdc), false);
}

/* Thermal protection and power level settings the firmware relies on */
static int mt7921_soc_thermal_config(struct mt792x_dev *dev)
{
	static const u8 prot[][20] = {
		{ 0x02, 0x00, 0x02, 0x01, 0x78, 0, 0, 0, 0x6e, 0, 0, 0, 0x1e },
		{ 0x02, 0x01, 0x02, 0x01, 0x78, 0, 0, 0, 0x6e, 0, 0, 0, 0x1e },
		{ 0x03, 0x00, 0x01, 0x01 },
		{ 0x04, 0x00, 0x00, 0x64 },
		{ 0x04, 0x00, 0x01, 0x5f },
		{ 0x04, 0x00, 0x02, 0x5a },
		{ 0x04, 0x00, 0x03, 0x55 },
		{ 0x02, 0x00, 0x01, 0x01, 0x6e, 0, 0, 0, 0x64, 0, 0, 0, 0x1e },
		{ 0x03, 0x01, 0x01, 0x01 },
		{ 0x04, 0x01, 0x00, 0x64 },
		{ 0x04, 0x01, 0x01, 0x5f },
		{ 0x04, 0x01, 0x02, 0x5a },
		{ 0x04, 0x01, 0x03, 0x55 },
		{ 0x02, 0x01, 0x01, 0x01, 0x6e, 0, 0, 0, 0x64, 0, 0, 0, 0x1e },
	};
	struct {
		__le32 version;
		__le32 type;
		u8 item_num;
		u8 rsv;
		__le16 len;
		struct {
			u8 type;
			u8 string_len;
			u8 value_len;
			u8 rsv;
			char string[32];
			char value[32];
		} __packed item;
		u8 pad[272 - 68];
	} __packed cfg = {
		.type = cpu_to_le32(1),
		.item_num = 1,
		.len = cpu_to_le16(sizeof(cfg.item)),
		.item = {
			.type = 3,
			.string_len = 16,
			.value_len = 32,
			.string = "PowerLevelNotify",
			.value = "0",
		},
	};
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(prot); i++) {
		ret = mt76_mcu_send_msg(&dev->mt76, MCU_EXT_CMD(THERMAL_PROT), prot[i],
					prot[i][0] == 0x02 ? 20 : 4, false);
		if (ret)
			return ret;
	}

	return mt76_mcu_send_msg(&dev->mt76, MCU_CE_CMD(SOC_CUSTOMER_CFG), &cfg,
				 sizeof(cfg), false);
}

/*
 * Per channel TX power limit tables. The firmware does not accept the
 * SET_RATE_TX_POWER command, it wants these legacy tables instead. They are
 * all sent unlimited (0x3f), the regulatory limits are applied by mac80211.
 */
static int mt7921_soc_power_limit(struct mt792x_dev *dev)
{
	static const u8 ch_2g5g[] = {
		1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
		36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64,
		100, 102, 104, 106, 108, 110, 112, 114, 116, 118, 120, 122, 124,
		126, 128, 130, 132, 134, 136, 138, 140, 142, 144,
		149, 151, 153, 155, 157, 159, 161, 163, 165,
	};
	static const struct {
		u8 type;
		u8 n_pwr;	/* power values per channel */
		u8 first, last;	/* 6 GHz channels, odd only */
	} tbl[] = {
		{ 0x00, 9 },		/* legacy, HT and VHT */
		{ 0x02, 18 },		/* HE */
		{ 0x04, 21, 1, 93 },	/* 6 GHz HE */
		{ 0x05, 21, 95, 185 },
		{ 0x06, 21, 187, 233 },
		{ 0x11, 9, 1, 93 },	/* 6 GHz legacy */
		{ 0x12, 9, 95, 185 },
		{ 0x13, 9, 187, 233 },
		{ 0x07 },		/* end of tables */
	};
	struct {
		__le16 country;
		u8 category;
		u8 num;
		u8 keep;
		u8 band_idx;
		u8 limit_type;
		u8 version;
		u8 data[1464];
	} __packed *req;
	int i, ret = 0;

	req = kmalloc(sizeof(*req), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(tbl); i++) {
		int size = tbl[i].n_pwr + 3, n = 0, ch;
		u8 *e = req->data;

		memset(req, 0, sizeof(*req));
		req->category = 4;
		req->keep = i > 0;
		req->limit_type = tbl[i].type;
		req->version = !tbl[i].first;

		for (ch = 0; tbl[i].n_pwr && !tbl[i].first && ch < ARRAY_SIZE(ch_2g5g); ch++) {
			e[0] = ch_2g5g[ch];
			memset(e + 1, 0x3f, tbl[i].n_pwr);
			e += size;
			n++;
		}
		/* 6 GHz: channel 2 then every odd one */
		for (ch = tbl[i].first; tbl[i].first && ch <= tbl[i].last;
		     ch = ch == 1 ? 2 : (ch == 2 ? 3 : ch + 2)) {
			e[0] = ch;
			memset(e + 1, 0x3f, tbl[i].n_pwr);
			e += size;
			n++;
		}
		req->num = n;

		ret = mt76_mcu_send_msg(&dev->mt76, MCU_CE_CMD(SOC_SET_PWR_LIMIT), req,
					sizeof(*req), false);
		if (ret)
			break;

		/* Unanswered, and the firmware crashes when they come too fast */
		usleep_range(2000, 3000);
	}

	kfree(req);
	return ret;
}

static int mt7921_soc_run_firmware(struct mt792x_dev *dev)
{
	int err;

	err = mt7921_run_firmware(dev);
	if (err)
		return err;

	/* The firmware expects the NVRAM once it reported its capabilities */
	err = mt7921_soc_nvram_send(dev);
	if (err)
		return err;

	/* Settings the firmware does not default to */
	err = mt7921_soc_basic_config(dev);
	if (err)
		return err;

	err = mt7921_soc_thermal_config(dev);
	if (err)
		return err;

	err = mt7921_soc_power_limit(dev);
	if (err)
		return err;

	/* There is no MAC address in the efuse */
	ether_addr_copy(dev->mphy.macaddr, to_soc_dev(dev)->nvram->data + NVRAM_MAC_OFFSET);

	return 0;
}

static int mt7921_soc_mcu_init(struct mt792x_dev *dev)
{
	static const struct mt76_mcu_ops mt7921_soc_mcu_ops = {
		.headroom = sizeof(struct mt76_connac2_mcu_txd),
		.mcu_skb_send_msg = mt7921_soc_mcu_send_message,
		.mcu_parse_response = mt7921_soc_mcu_parse_response,
	};
	int err;

	dev->mt76.mcu_ops = &mt7921_soc_mcu_ops;

	err = mt7921_soc_drv_own(dev);
	if (err)
		return err;

	return mt7921_soc_run_firmware(dev);
}

static int mt7921_soc_wfsys_reset(struct mt792x_dev *dev)
{
	mt7921_soc_wfsys_off(dev);

	return mt7921_soc_wfsys_on(dev);
}

static int mt7921_soc_init_reset(struct mt792x_dev *dev)
{
	int ret;

	ret = mt7921_soc_wfsys_reset(dev);
	if (ret)
		return ret;

	return mt792x_wpdma_reset(dev, false);
}

static int mt7921_soc_mac_reset(struct mt792x_dev *dev)
{
	int i, err;

	mt7921_soc_drv_pmctrl(dev);

	mt76_connac_free_pending_tx_skbs(&dev->pm, NULL);

	mt76_wr(dev, dev->irq_map->host_irq_enable, 0);

	set_bit(MT76_MCU_RESET, &dev->mphy.state);
	wake_up(&dev->mt76.mcu.wait);
	skb_queue_purge(&dev->mt76.mcu.res_q);

	mt76_txq_schedule_all(&dev->mphy);

	mt76_worker_disable(&dev->mt76.tx_worker);
	mt76_for_each_q_rx(&dev->mt76, i) {
		napi_disable(&dev->mt76.napi[i]);
	}
	napi_disable(&dev->mt76.tx_napi);

	mt76_connac2_tx_token_put(&dev->mt76);
	idr_init(&dev->mt76.token);

	mt7921_soc_wfsys_reset(dev);
	mt792x_wpdma_reset(dev, false);

	mt76_for_each_q_rx(&dev->mt76, i) {
		napi_enable(&dev->mt76.napi[i]);
	}

	local_bh_disable();
	mt76_for_each_q_rx(&dev->mt76, i) {
		napi_schedule(&dev->mt76.napi[i]);
	}
	local_bh_enable();

	dev->fw_assert = false;
	clear_bit(MT76_MCU_RESET, &dev->mphy.state);

	mt76_wr(dev, dev->irq_map->host_irq_enable,
		dev->irq_map->tx.all_complete_mask |
		dev->irq_map->rx.data_complete_mask |
		dev->irq_map->rx.wm_complete_mask |
		dev->irq_map->rx.wm2_complete_mask | MT_INT_MCU_CMD);

	err = mt7921_soc_drv_own(dev);
	if (err)
		goto out;

	err = mt7921_soc_run_firmware(dev);
	if (err)
		goto out;

	err = mt7921_mcu_set_eeprom(dev);
	if (err)
		goto out;

	err = mt7921_mac_init(dev);
	if (err)
		goto out;

	err = __mt7921_start(&dev->phy);
out:
	napi_enable(&dev->mt76.tx_napi);
	local_bh_disable();
	napi_schedule(&dev->mt76.tx_napi);
	local_bh_enable();

	mt76_worker_enable(&dev->mt76.tx_worker);

	return err;
}

static void mt7921_soc_unregister_device(struct mt792x_dev *dev)
{
	struct mt76_connac_pm *pm = &dev->pm;
	struct ieee80211_hw *hw = mt76_hw(dev);
	int i;

	if (dev->phy.chip_cap & MT792x_CHIP_CAP_WF_RF_PIN_CTRL_EVT_EN)
		wiphy_rfkill_stop_polling(hw->wiphy);

	cancel_work_sync(&dev->init_work);
	mt76_unregister_device(&dev->mt76);
	mt76_for_each_q_rx(&dev->mt76, i)
		napi_disable(&dev->mt76.napi[i]);
	cancel_delayed_work_sync(&pm->ps_work);
	cancel_work_sync(&pm->wake_work);
	cancel_work_sync(&dev->reset_work);

	mt76_connac2_tx_token_put(&dev->mt76);
	mt7921_soc_drv_own(dev);
	mt792x_dma_cleanup(dev);
	mt7921_soc_power_off(dev);
	skb_queue_purge(&dev->mt76.mcu.res_q);

	tasklet_disable(&dev->mt76.irq_tasklet);
}

/* Legacy firmware events are handled before the common code sees them */
static void mt7921_soc_queue_rx_skb(struct mt76_dev *mdev, enum mt76_rxq_id q,
				    struct sk_buff *skb, u32 *info)
{
	if (q == MT_RXQ_MCU && skb->len >= sizeof(struct mt76_connac2_mcu_rxd)) {
		__le32 *rxd = (__le32 *)skb->data;
		u32 type = le32_get_bits(rxd[0], MT_RXD0_PKT_TYPE);

		if (type == PKT_TYPE_RX_EVENT &&
		    mt7921_soc_legacy_event(mdev, q, skb, info))
			return;
	}

	mt7921_queue_rx_skb(mdev, q, skb, info);
}

/* TX status is reported by the firmware in TX_DONE events */
static int mt7921_soc_tx_prepare_skb(struct mt76_dev *mdev, void *txwi_ptr,
				     enum mt76_txq_id qid, struct mt76_wcid *wcid,
				     struct ieee80211_sta *sta,
				     struct mt76_tx_info *tx_info)
{
	__le32 *txwi = txwi_ptr;
	int ret;

	ret = mt7921e_tx_prepare_skb(mdev, txwi_ptr, qid, wcid, sta, tx_info);
	if (ret)
		return ret;

	if (txwi[5] & cpu_to_le32(MT_TXD5_TX_STATUS_HOST)) {
		txwi[5] &= ~cpu_to_le32(MT_TXD5_TX_STATUS_HOST);
		txwi[5] |= cpu_to_le32(MT_TXD5_TX_STATUS_MCU);
	}

	return 0;
}

/* Block ack sessions are negotiated by the firmware itself */
static int mt7921_soc_ampdu_action(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
				   struct ieee80211_ampdu_params *params)
{
	if (params->action == IEEE80211_AMPDU_TX_START)
		return -EOPNOTSUPP;

	return 0;
}

static int mt7921_soc_add_interface(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	struct mt7921_soc_dev *sdev = mt7921_soc_dev(mt792x_hw_dev(hw));
	int ret;

	/* The legacy firmware needs the network type to activate a BSS */
	sdev->adding_ap = vif->type == NL80211_IFTYPE_AP;
	ret = mt7921_ops.add_interface(hw, vif);
	sdev->adding_ap = false;

	return ret;
}

static int mt7921_soc_start_ap(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			       struct ieee80211_bss_conf *link_conf)
{
	struct mt792x_dev *dev = mt792x_hw_dev(hw);
	int ret;

	/* Like a connection, an access point starts with a channel request */
	mt792x_mutex_acquire(dev);
	ret = mt7921_soc_ap_channel(dev, vif);
	mt792x_mutex_release(dev);
	if (ret)
		return ret;

	return mt7921_ops.start_ap(hw, vif, link_conf);
}

static int mt7921_soc_probe(struct platform_device *pdev)
{
	static const struct mt76_driver_ops drv_ops = {
		/* txwi_size = txd size + txp size */
		.txwi_size = MT_TXD_SIZE + sizeof(struct mt76_connac_hw_txp),
		.drv_flags = MT_DRV_TXWI_NO_FREE | MT_DRV_HW_MGMT_TXQ |
			     MT_DRV_AMSDU_OFFLOAD,
		.survey_flags = SURVEY_INFO_TIME_TX |
				SURVEY_INFO_TIME_RX |
				SURVEY_INFO_TIME_BSS_RX,
		.token_size = MT7921_TOKEN_SIZE,
		.tx_prepare_skb = mt7921_soc_tx_prepare_skb,
		.tx_complete_skb = mt76_connac_tx_complete_skb,
		.rx_check = mt7921_rx_check,
		.rx_skb = mt7921_soc_queue_rx_skb,
		.rx_poll_complete = mt792x_rx_poll_complete,
		.sta_add = mt7921_mac_sta_add,
		.sta_event = mt7921_mac_sta_event,
		.sta_remove = mt7921_mac_sta_remove,
		.update_survey = mt792x_update_channel,
		.set_channel = mt7921_set_channel,
	};
	static const struct mt792x_hif_ops mt7921_soc_ops = {
		.init_reset = mt7921_soc_init_reset,
		.reset = mt7921_soc_mac_reset,
		.mcu_init = mt7921_soc_mcu_init,
		.drv_own = mt7921_soc_drv_pmctrl,
		.fw_own = mt792xe_mcu_fw_pmctrl,
		.load_firmware = mt7921_soc_load_firmware,
		.dma_prefetch = mt7921_soc_dma_prefetch,
	};
	static const struct mt792x_irq_map irq_map = {
		/* also written directly by the common code */
		.host_irq_enable = 0x24204,
		.tx = {
			.all_complete_mask = MT_INT_TX_DONE_ALL,
			.mcu_complete_mask = MT_INT_TX_DONE_MCU,
		},
		.rx = {
			.data_complete_mask = HOST_RX_DONE_INT_ENA0,
			.wm_complete_mask = HOST_RX_DONE_INT_ENA2,
			.wm2_complete_mask = HOST_RX_DONE_INT_ENA3,
		},
	};
	struct device *pdev_dev = &pdev->dev;
	struct mt7921_soc_dev *sdev;
	struct ieee80211_ops *ops;
	struct mt76_bus_ops *bus_ops;
	struct mtk_connsys *cs;
	struct mt792x_dev *dev;
	struct mt76_dev *mdev;
	const char *nvram_name;
	void __iomem *regs;
	struct resource *res;
	u8 features;
	int irq, ccif_irq, ret;

	cs = devm_mtk_connsys_get(pdev_dev);
	if (IS_ERR(cs))
		return dev_err_probe(pdev_dev, PTR_ERR(cs), "Cannot get connsys\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ccif_irq = platform_get_irq(pdev, 1);
	if (ccif_irq < 0)
		return ccif_irq;

	/* Shared with the connsys driver, so don't request it */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res || resource_size(res) < SOC_REG_SIZE)
		return -EINVAL;

	regs = devm_ioremap(pdev_dev, res->start, resource_size(res));
	if (!regs)
		return -ENOMEM;

	/* The DMA descriptors have no room for higher address bits */
	ret = dma_set_mask_and_coherent(pdev_dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	/* The firmware is not available as a file */
	ops = mt792x_get_mac80211_ops(pdev_dev, &mt7921_ops, NULL, &features);
	if (!ops)
		return -ENOMEM;

	/* The firmware manages the channel, connections need channel requests */
	features |= MT792x_FW_CAP_CNM;
	ops->remain_on_channel = mt7921_ops.remain_on_channel;
	ops->cancel_remain_on_channel = mt7921_ops.cancel_remain_on_channel;
	ops->add_chanctx = mt7921_ops.add_chanctx;
	ops->remove_chanctx = mt7921_ops.remove_chanctx;
	ops->change_chanctx = mt7921_ops.change_chanctx;
	ops->switch_vif_chanctx = mt7921_ops.switch_vif_chanctx;
	ops->assign_vif_chanctx = mt7921_ops.assign_vif_chanctx;
	ops->unassign_vif_chanctx = mt7921_ops.unassign_vif_chanctx;
	ops->mgd_prepare_tx = mt7921_ops.mgd_prepare_tx;
	ops->mgd_complete_tx = mt7921_ops.mgd_complete_tx;
	ops->ampdu_action = mt7921_soc_ampdu_action;
	ops->add_interface = mt7921_soc_add_interface;
	ops->start_ap = mt7921_soc_start_ap;

	mdev = mt76_alloc_device(pdev_dev, sizeof(*sdev), ops, &drv_ops);
	if (!mdev)
		return -ENOMEM;

	dev = container_of(mdev, struct mt792x_dev, mt76);
	sdev = to_soc_dev(dev);
	sdev->cs = cs;
	init_completion(&sdev->ap_grant);
	platform_set_drvdata(pdev, mdev);

	nvram_name = MT7921_SOC_NVRAM;
	of_property_read_string(pdev_dev->of_node, "firmware-name", &nvram_name);
	ret = request_firmware(&sdev->nvram, nvram_name, pdev_dev);
	if (ret)
		goto err_free_dev;

	ret = mt7921_soc_nvram_parse(sdev);
	if (ret) {
		dev_err(pdev_dev, "Invalid NVRAM %s\n", nvram_name);
		goto err_free_nvram;
	}

	dev->fw_features = features;
	dev->hif_ops = &mt7921_soc_ops;
	dev->irq_map = &irq_map;
	mt76_mmio_init(&dev->mt76, regs);
	tasklet_init(&mdev->irq_tasklet, mt792x_irq_tasklet, (unsigned long)dev);

	dev->phy.dev = dev;
	dev->phy.mt76 = &dev->mt76.phy;
	dev->mt76.phy.priv = &dev->phy;
	dev->bus_ops = dev->mt76.bus;
	bus_ops = devm_kmemdup(dev->mt76.dev, dev->bus_ops, sizeof(*bus_ops),
			       GFP_KERNEL);
	if (!bus_ops) {
		ret = -ENOMEM;
		goto err_free_nvram;
	}

	bus_ops->rr = mt7921_soc_rr;
	bus_ops->wr = mt7921_soc_wr;
	bus_ops->rmw = mt7921_soc_rmw;
	dev->mt76.bus = bus_ops;

	ret = mt7921_soc_power_on(dev);
	if (ret)
		goto err_free_nvram;

	mt7921_soc_print_fw_version(dev);

	ret = mt7921_soc_drv_own(dev);
	if (ret)
		goto err_power_off;

	mdev->rev = (0x7961 << 16) | (soc_rr(dev, SOC_WF_HW_VER) & 0xff);
	dev_info(mdev->dev, "ASIC revision: %04x\n", mdev->rev);

	mt76_wr(dev, irq_map.host_irq_enable, 0);

	ret = devm_request_irq(pdev_dev, irq, mt792x_irq_handler, 0,
			       KBUILD_MODNAME, dev);
	if (ret)
		goto err_power_off;

	ret = devm_request_threaded_irq(pdev_dev, ccif_irq, NULL, mt7921_soc_ccif_irq,
					IRQF_ONESHOT, "mt7921-soc-ccif", dev);
	if (ret)
		goto err_free_irq;

	ret = mt7921_soc_dma_init(dev);
	if (ret)
		goto err_free_ccif_irq;

	/*
	 * The firmware keeps the WTBL entries matching BSS indexes for the
	 * BIP keys: stations start at 4, entry 0 is the global one.
	 */
	dev->mt76.wcid_mask[0] |= GENMASK(3, 1);

	ret = mt7921_register_device(dev);
	if (ret)
		goto err_free_ccif_irq;

	/* No RX header translation, see mt7921_soc_reg_fixup() */
	__clear_bit(IEEE80211_HW_SUPPORTS_RX_DECAP_OFFLOAD, mt76_hw(dev)->flags);

	/* Runtime power management is not validated yet */
	dev->pm.enable_user = false;
	dev->pm.enable = false;
	dev->pm.ds_enable_user = false;
	dev->pm.ds_enable = false;

	return 0;

err_free_ccif_irq:
	devm_free_irq(pdev_dev, ccif_irq, dev);
err_free_irq:
	devm_free_irq(pdev_dev, irq, dev);
err_power_off:
	mt7921_soc_power_off(dev);
err_free_nvram:
	release_firmware(sdev->nvram);
err_free_dev:
	mt76_free_device(&dev->mt76);

	return ret;
}

static void mt7921_soc_remove(struct platform_device *pdev)
{
	struct mt76_dev *mdev = platform_get_drvdata(pdev);
	struct mt792x_dev *dev = container_of(mdev, struct mt792x_dev, mt76);
	const struct firmware *nvram = to_soc_dev(dev)->nvram;

	mt7921_soc_unregister_device(dev);
	set_bit(MT76_REMOVED, &mdev->phy.state);
	devm_free_irq(&pdev->dev, platform_get_irq(pdev, 0), dev);
	mt76_free_device(&dev->mt76);
	release_firmware(nvram);
}

static const struct of_device_id mt7921_soc_of_match[] = {
	{ .compatible = "mediatek,mt6878-wifi" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt7921_soc_of_match);

static struct platform_driver mt7921_soc_driver = {
	.probe = mt7921_soc_probe,
	.remove = mt7921_soc_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = mt7921_soc_of_match,
	},
};
module_platform_driver(mt7921_soc_driver);

MODULE_FIRMWARE(MT7921_SOC_NVRAM);
MODULE_DESCRIPTION("MediaTek MT7921 compatible connsys integrated wireless driver");
MODULE_LICENSE("Dual BSD/GPL");
