// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6878 modem (MD1) control and communication (CCCI)
 *
 * The bootloader loads the modem firmware into reserved memory and leaves a
 * list of tags describing the memory layout in the "ccci tag" region. This
 * driver releases the modem from reset through the secure monitor and talks
 * to it through the CCIF mailbox: control messages and the boot handshake
 * go through the CCIF SRAM, everything else through ring buffers in the
 * non-cacheable AP/MD shared memory.
 *
 * The logical channels the modem uses at boot (file system, RPC) and the
 * AT command channel are exported as character devices. Packets of the file
 * system and RPC channels keep their 16 byte CCCI header, as the userspace
 * servers need its reserved field to address their replies.
 *
 * The protocol follows the MediaTek downstream ECCCI driver.
 */

#include <linux/arm-smccc.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/if_arp.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/random.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>
#include <linux/timekeeping.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <asm/arch_timer.h>

/* Secure monitor calls */
#define MTK_SIP_CCCI_CONTROL		MTK_SIP_SMC_CMD(0x505)
#define MD_POWER_CONFIG			6
#define  MD_KERNEL_BOOT_UP		0
#define  MD_CHECK_FLAG			2
#define  MD_CHECK_DONE			3
#define  MD_BOOT_STATUS			4
#define MD_FLIGHT_MODE_SET		7
#define CCIF_CLK_REQUEST		14
#define CCIF_CLK_RELEASE		15

/*
 * DVFSRC: the modem's vcore/DDR requests go through it; the stock kernel
 * initializes and starts it in the secure monitor at boot, mainline does not
 */
#define MTK_SIP_DVFSRC_CONTROL		MTK_SIP_SMC_CMD(0x506)
#define  DVFSRC_INIT			0
#define  DVFSRC_START			1

/* CCIF registers */
#define CCIF_BUSY			0x04
#define CCIF_TCHNUM			0x0c
#define CCIF_RCHNUM			0x10
#define CCIF_ACK			0x14
#define CCIF_CHDATA			0x100
#define CCIF_SRAM_SIZE			0x200

/* CCIF SRAM layout, offsets from CCIF_CHDATA */
#define SRAM_DL_HEADER			0x000
#define SRAM_MD_RT_DATA			0x010
#define SRAM_UP_HEADER			0x0f0
#define SRAM_AP_RT_DATA			0x100
#define SRAM_DBG_MAGIC			(CCIF_SRAM_SIZE - 12)

#define CCIF_CH_SRAM			15
/* exception handshake */
#define CCIF_D2H_EX_INIT		16
#define CCIF_D2H_EX_INIT_DONE		17
#define CCIF_D2H_EX_CLEARQ_DONE		18
#define CCIF_D2H_EX_ALLQ_RESET		19
#define CCIF_H2D_EX_ACK			16
#define CCIF_H2D_EX_CLEARQ_ACK		17
#define CCIF_H2D_FORCE_ASSERT		21
#define CCIF_NUM_QUEUES			16

/* MD power domain (SPM), bus protection and clock mode */
#define SPM_MD_PWR_CON			0xe00
#define  PWR_ON				BIT(2)
#define  PWR_ACK			BIT(30)
#define SPM_MD_EXT_BUCK_ISO		0xf24
#define  MD_EXT_BUCK_ISO		GENMASK(1, 0)
#define TOPCKGEN_MD_CLK_MODE		0x000
#define  MD_CLK_MODE_OFF		(BIT(8) | BIT(9))

struct ccci_bus_prot {
	u8 reg;				/* index into mtk_ccci.bus_prot_base */
	u16 set, clr, sta;
	u32 mask;
};

static const struct ccci_bus_prot md_bus_prot[] = {
	{ 0, 0xc54, 0xc58, 0xc5c, BIT(9) },
	{ 0, 0xc44, 0xc48, 0xc4c, BIT(11) },
	{ 1, 0x084, 0x088, 0x08c, BIT(6) | BIT(7) },
};

/* DPMAIF (data path), only set up far enough for the modem to boot */
#define INFRA_AO_DPMAIF_CTRL		0x208
#define  DPMAIF_IP_BUSY_EN		BIT(15)
#define INFRA_AO_DPMAIF_RST_SET		0x130
#define INFRA_AO_DPMAIF_RST_CLR		0x134
#define  DPMAIF_AO_RST			BIT(0)
#define INFRACFG_DPMAIF_PD_RST		BIT(22)
#define DPMAIF_AO_UL_INIT_SET		0x000
#define DPMAIF_AO_UL_AP_L1TIMR0		0x08c
#define DPMAIF_AO_DL_INIT_SET		0x400
#define DPMAIF_PD_DL_RESERVE_RW		0x1a8
#define DPMAIF_PD_MISC_CG_EN		0x468
#define DPMAIF_PD_MISC_OVERWRITE_CFG	0x490
#define DPMAIF_PD_MISC_MEM_CLR		0x494
#define DPMAIF_PD_REMAP_BANK(n)		(0xe0c + (n) * 4)
#define INFRA_AO_DPMAIF_CFG		0xc00

/* AO UL interrupt masks and arbitration */
#define DPMAIF_AO_UL_CHNL_ARB0		0x01c
#define DPMAIF_AO_UL_AP_L2TIMR0		0x080
#define DPMAIF_AO_UL_AP_L2TIMCR0	0x084
#define DPMAIF_AO_UL_AP_L2TIMSR0	0x088
#define DPMAIF_AO_UL_APDL_L2TIMR0	0x090
#define DPMAIF_AO_UL_APDL_L2TIMCR0	0x094
#define DPMAIF_AO_UL_APDL_L2TIMSR0	0x098
#define DPMAIF_AO_UL_IP_BUSY_MASK	0x09c
#define DPMAIF_UL_INTR_MASK		0x01ff800f
#define DPMAIF_DL_INTR_MASK		0x41

/* PD DL: BAT and PIT */
#define DPMAIF_PD_BAT_INIT		0x100
#define DPMAIF_PD_BAT_ADD		0x104
#define DPMAIF_PD_BAT_INIT_CON0		0x108
#define DPMAIF_PD_BAT_INIT_CON1		0x10c
#define  BAT_CON1_EN			BIT(16)
#define  BAT_CON1_CACHE_EN		BIT(22)
#define DPMAIF_PD_PIT_INIT		0x120
#define DPMAIF_PD_PIT_INIT_CON0		0x128
#define DPMAIF_PD_PIT_INIT_CON1		0x12c
#define DPMAIF_PD_PIT_INIT_CON3		0x134
#define  DPMAIF_INIT_BUSY		BIT(31)
#define  DPMAIF_INIT_ALLSET		BIT(0)
#define  DPMAIF_INIT_FRG		BIT(16)

#define DPMAIF_PD_PIT_ADD		0x124
#define  DPMAIF_ADD_BUSY		BIT(31)

/* PD UL: DRB add per queue */
#define DPMAIF_PD_UL_ADD_DESC(q)	(0x0b0 + (q) * 4)

/* PD misc */
#define DPMAIF_PD_UL_L2TISAR0		0x400
#define DPMAIF_PD_APDL_L2TISAR0		0x450
#define DPMAIF_PD_AP_IP_BUSY		0x460
#define  DPMAIF_UL_INT_QDONE		GENMASK(3, 0)
#define  DPMAIF_UL_INT_MD_PWR_NOTRDY	GENMASK(18, 15)
#define  DPMAIF_DL_INT_QDONE		BIT(0)

/* AO DL SRAM: packet info and thresholds */
#define DPMAIF_DL_PKTINFO_CON0		0xc00
#define DPMAIF_DL_PKTINFO_CON1		0xc04
#define DPMAIF_DL_PKTINFO_CON2		0xc08
#define DPMAIF_DL_RDY_CHK_THRES		0xc0c
#define DPMAIF_DL_RDY_CHK_FRG_THRES	0xc10

#define DPMAIF_DL_PIT_STA3		0xc60
#define  DPMAIF_DL_PIT_WRIDX		GENMASK(17, 0)

/* AO UL SRAM: per queue DRB rings, read index (in words) in bits 31:16 */
#define DPMAIF_UL_CHNL_CON0(q)		(0xd10 + (q) * 0x10)
#define DPMAIF_UL_CHNL_CON1(q)		(0xd14 + (q) * 0x10)
#define DPMAIF_UL_CHNL_STA(q)		(0xde0 + (q) * 4)

/*
 * PIT entries (16 bytes): word 0 bit 0 tells a message (channel, drop bit)
 * from a data entry (buffer type and id, length), word 1/2 hold the data
 * address, word 3 the sequence number and the high bits of the buffer id.
 * DRB entries (8 bytes) are the same for the uplink: a message entry with
 * the channel and the packet length, then the data entries.
 */
#define PIT_MSG				BIT(0)
#define PIT_CONT			BIT(1)
#define PIT_FRAG			BIT(2)
#define PIT_BID				GENMASK(15, 3)
#define PIT_LEN				GENMASK(31, 16)
#define PIT_MSG_CHANNEL			GENMASK(23, 16)
#define PIT_MSG_DROP			BIT(31)
#define PIT_SEQ				GENMASK(7, 0)
#define PIT_H_BID			GENMASK(10, 8)
#define PIT_SEQ_MAX			0xfe
#define DRB_MSG				BIT(0)
#define DRB_CONT			BIT(2)
#define DRB_ADDR_EXT			GENMASK(15, 8)
#define DRB_LEN				GENMASK(31, 16)
#define DRB_MSG_CHANNEL			GENMASK(23, 16)

#define CCMNI_NUM			8
#define CCMNI_IF_MTU			1500

#define DPMAIF_PIT_CNT			32768
#define DPMAIF_PIT_SIZE			16
#define DPMAIF_BAT_CNT			16384
#define DPMAIF_BAT_FILL			2000
#define DPMAIF_BAT_BUF_SIZE		2048
#define DPMAIF_TXQ_NUM			4
#define DPMAIF_DRB_CNT			2048

#define INFRACFG_RST_SET		0xf50
#define INFRACFG_RST_CLR		0xf54
#define INFRACFG_CCIF0_RST		BIT(26)

/* Logical channels */
#define CCCI_CONTROL_RX			0
#define CCCI_CONTROL_TX			1
#define CCCI_SYSTEM_RX			2
#define CCCI_SYSTEM_TX			3
#define CCCI_FS_TX			15
#define CCCI_RPC_RX			32
#define CCCI_RPC_TX			33
#define CCCI_MAX_CH			512

#define MD_INIT_START_BOOT		0
#define MD_INIT_CHK_ID			0x5555ffff
#define MD_EX				4
#define MD_EX_REC_OK			6
#define MD_EX_PASS			8
#define CCCI_CTRL_MAGIC			0xffffffff

/* Shared memory users */
#define SMEM_RAW_CCB_CTRL		4
#define SMEM_RAW_DHL			5
#define SMEM_CCISM_SCP			16
#define SMEM_RAW_NETD			7
#define SMEM_RAW_USB			8
#define SMEM_RAW_AUDIO			9
#define SMEM_RAW_LWA			11
#define SMEM_RAW_MDCCCI_DBG		12
#define SMEM_RAW_MDSS_DBG		13
#define SMEM_RAW_RUNTIME_DATA		14
#define SMEM_CCISM_MCU			19
#define SMEM_CCISM_MCU_EXP		20
#define SMEM_RAW_MD_CONSYS		22
#define SMEM_RAW_PHY_CAP		23
#define SMEM_RAW_USIP			24
#define SMEM_RAW_AMMS_POS		29
#define SMEM_MD_WIFI_PROXY		31
#define SMEM_MD_NVRAM_CACHE		32
#define SMEM_SECURITY			34
#define SMEM_MD_DRDI			41
#define SMEM_MD_POST_DUMP		46
#define SMEM_CCB_DHL			1
#define SMEM_MAX			64

#define SMEM_ATTR_PADDING		BIT(2)
#define MD_BANK4_MD_VIEW		0x40000000

/* Runtime features */
#define FEATURE_COUNT			64
#define MD_FEATURE_QUERY_PATTERN	0x49434343
#define AP_FEATURE_QUERY_PATTERN	0x43434349
#define FEATURE_NOT_EXIST		0
#define FEATURE_NOT_SUPPORT		1
#define FEATURE_MUST_SUPPORT		2
#define FEATURE_OPTIONAL_SUPPORT	3
#define FEATURE_BACKWARD_COMPAT		4
#define MULTI_MD_MPU_SUPPORT		2
#define RUNTIME_AP_SIZE			0x800
#define RUNTIME_MD_SIZE			0x800
#define EE_SMEM_SIZE			0x10000
#define CCMNI_MTU			3584

enum {
	BOOT_INFO = 0,
	EXCEPTION_SHARE_MEMORY = 1,
	CCIF_SHARE_MEMORY = 2,
	SMART_LOGGING_SHARE_MEMORY = 3,
	MD1MD3_SHARE_MEMORY = 4,
	MISC_INFO_HIF_DMA_REMAP = 5,
	MISC_INFO_RTC_32K_LESS = 6,
	MISC_INFO_RANDOM_SEED_NUM = 7,
	MISC_INFO_GPS_COCLOCK = 8,
	MISC_INFO_SBP_ID = 9,
	MISC_INFO_CCCI = 10,
	MISC_INFO_CLIB_TIME = 11,
	MISC_INFO_C2K = 12,
	MD_IMAGE_START_MEMORY = 13,
	CCISM_SHARE_MEMORY = 14,
	CCB_SHARE_MEMORY = 15,
	DHL_RAW_SHARE_MEMORY = 16,
	DT_NETD_SHARE_MEMORY = 17,
	DT_USB_SHARE_MEMORY = 18,
	EE_AFTER_EPOF = 19,
	AP_CCMNI_MTU = 20,
	MISC_INFO_CUSTOMER_VAL = 21,
	MISC_INFO_C2K_MEID = 24,
	LWA_SHARE_MEMORY = 25,
	AUDIO_RAW_SHARE_MEMORY = 26,
	MULTI_MD_MPU = 27,
	CCISM_SHARE_MEMORY_EXP = 28,
	MD_PHY_CAPTURE = 29,
	MD_CONSYS_SHARE_MEMORY = 30,
	MD_USIP_SHARE_MEMORY = 31,
	MD_MTEE_SHARE_MEMORY_ENABLE = 32,
	MD_POS_SHARE_MEMORY = 33,
	UDC_RAW_SHARE_MEMORY = 34,
	MD_WIFI_PROXY_SHARE_MEMORY = 35,
	NVRAM_CACHE_SHARE_MEMORY = 36,
	SECURITY_SHARE_MEMORY = 37,
	MD_MEM_AP_VIEW_INF = 38,
	AMMS_DRDI_COPY = 45,
	MD_POST_DUMP = 55,
};

/* Runtime RAT set: NR + LTE FDD/TDD + WCDMA + GSM, as the stock setup */
#define MD_RAT_NLWG			0x79
/* booting_start_id: modem wait time 15 (as set by the stock userspace) */
#define MD_BOOT_START_ID		0xf0000

/*
 * CCB gear 1: the first 2 MB of the CCB region are the CCB buffers, the
 * remaining 20 MB the DHL/MDM logging buffers
 */
#define CCB_DATA_REGION_SIZE		0x200000
#define CCB_DATA_SIZE			0x17c000

/*
 * CCB buffers per user, from the stock ccb_configs table: page and buffer
 * size of the downlink and uplink half. The buffers add up to CCB_DATA_SIZE.
 */
#define CCB_CTRL_ENTRY			0x40
#define CCB_CTRL_MAGIC			0xeeff0011

static const struct {
	u32 dl_page, ul_page, dl_size, ul_size;
} ccb_configs[] = {
	{ 0x400, 0x400, 0x8000, 0x8000 },
	{ 0x5000, 0x80, 0x1e000, 0x80 },
	{ 0x200, 0x400, 0x9000, 0x8000 },
	{ 0x200, 0x80, 0x8000, 0x80 },
	{ 0x200, 0x80, 0x8000, 0x80 },
	{ 0x200, 0x80, 0x20000, 0x80 },
	{ 0x80, 0x80, 0x80, 0x80 },
	{ 0x80, 0x80, 0x80, 0x80 },
	{ 0x80, 0x80, 0x80, 0x80 },
	{ 0x80, 0x80, 0x80, 0x80 },
	{ 0x80, 0x80, 0x80, 0x80 },
	{ 0x80, 0x80, 0x80, 0x80 },
	{ 0x80, 0x80, 0x80, 0x80 },
	{ 0x80, 0x80, 0x80, 0x80 },
	{ 0x80, 0x80, 0x80, 0x80 },
	{ 0x80, 0x80, 0x80, 0x80 },
	{ 0x80, 0x80, 0x80, 0x80 },
	{ 0x80, 0x80, 0x280, 0x80 },
	{ 0x200, 0x400, 0x4000, 0x4000 },
	{ 0x10400, 0x10400, 0x82000, 0x82000 },
};

struct ccci_header {
	__le32 data[2];
	__le16 channel;
	__le16 seq;			/* bit 15: assert bit */
	__le32 reserved;
} __packed;

#define CCCI_SEQ_ASSERT			BIT(15)

struct md_query_ap_feature {
	u32 head_pattern;
	u8 feature_set[FEATURE_COUNT];	/* support_mask:4, version:4 */
	u32 tail_pattern;
	u8 reserved[152];
} __packed;

struct ap_query_md_feature_v2_1 {
	u32 head_pattern;
	u8 feature_set[FEATURE_COUNT];
	u32 share_memory_support;
	u32 ap_runtime_data_addr;
	u32 ap_runtime_data_size;
	u32 md_runtime_data_addr;
	u32 md_runtime_data_size;
	u32 noncached_mpu_start_addr;
	u32 noncached_mpu_total_size;
	u32 cached_mpu_start_addr;
	u32 cached_mpu_total_size;
	u32 reserve_addr[12];
	u32 tail_pattern;
} __packed;

struct ccci_runtime_feature {
	u8 feature_id;
	u8 support_info;
	u8 reserved[2];
	u32 data_len;
} __packed;

/* Bootloader tags */
struct ccci_tag {
	char name[64];
	u32 data_offset;
	u32 data_size;
	u32 next_tag_offset;
} __packed;

struct ccci_tag_hdr_tbl {
	u64 base_addr;
	u32 size;
	s8 md_id;
	s8 errno;
	s8 md_type;
	s8 ver;
	u32 reserved[2];
} __packed;

struct ccci_tag_smem_region {
	u64 ap_phy;
	u64 ap_vir;
	s32 id;
	u32 ap_offset;
	u32 size;
	u32 align;
	u32 flags;
	u32 md_offset;
} __packed;

/* Ring buffers in the shared memory */
#define RBF_HEADER			0xee0000ee
#define RBF_FOOTER			0xff0000ff
#define PKG_HEADER			0xaabbaabb
#define PKG_FOOTER			0xccddeeff
#define RBF_CTL_LEN			40

struct ccci_ringbuf_ctl {
	u32 rx_read;
	u32 rx_write;
	u32 rx_length;
	u32 tx_read;
	u32 tx_write;
	u32 tx_length;
} __packed;

static const u32 rx_queue_size[CCIF_NUM_QUEUES] = {
	80 << 10, 80 << 10, 40 << 10, 80 << 10, 20 << 10, 20 << 10, 48 << 10, 0,
	8 << 10, 16 << 10,
};

static const u32 tx_queue_size[CCIF_NUM_QUEUES] = {
	128 << 10, 40 << 10, 8 << 10, 40 << 10, 20 << 10, 20 << 10, 48 << 10, 0,
	8 << 10, 16 << 10,
};

static const u32 exp_queue_size[CCIF_NUM_QUEUES] = {
	12 << 10, 32 << 10, 8 << 10, 0, 0, 0, 8 << 10,
};

struct ccci_ring {
	struct ccci_ringbuf_ctl __iomem *ctl;
	u8 __iomem *rx_buf;
	u8 __iomem *tx_buf;
	spinlock_t tx_lock;
};

enum md_state {
	MD_STATE_OFF,
	MD_STATE_WAIT_HS1,
	MD_STATE_WAIT_HS2,
	MD_STATE_READY,
	MD_STATE_EXCEPTION,
};

static const char * const md_state_names[] = {
	[MD_STATE_OFF] = "off",
	[MD_STATE_WAIT_HS1] = "wait_hs1",
	[MD_STATE_WAIT_HS2] = "wait_hs2",
	[MD_STATE_READY] = "ready",
	[MD_STATE_EXCEPTION] = "exception",
};

struct mtk_ccci;

struct ccci_port {
	const char *name;
	u16 rx_ch;
	u16 tx_ch;
	u8 txq;
	bool user_header;

	struct mtk_ccci *ccci;
	struct miscdevice misc;
	struct sk_buff_head rx_list;
	wait_queue_head_t rx_wq;
	struct mutex tx_mutex;
};

#define CCCI_PORT_RX_MAX		256

static const struct ccci_port ccci_port_cfg[] = {
	{ .name = "ccci_fs",     .rx_ch = 14,  .tx_ch = 15,  .txq = 4, .user_header = true },
	{ .name = "ccci_rpc",    .rx_ch = 32,  .tx_ch = 33,  .txq = 1, .user_header = true },
	{ .name = "ccci_ipc",    .rx_ch = 34,  .tx_ch = 36,  .txq = 1, .user_header = true },
	{ .name = "ccci_at",     .rx_ch = 10,  .tx_ch = 12,  .txq = 5 },
	{ .name = "ccci_at2",    .rx_ch = 38,  .tx_ch = 40,  .txq = 1 },
	{ .name = "ccci_mipc0",  .rx_ch = 214, .tx_ch = 215, .txq = 5 },
	{ .name = "ccci_mipc1",  .rx_ch = 216, .tx_ch = 217, .txq = 5 },
};

struct mtk_ccci {
	struct device *dev;
	void __iomem *ap_ccif;
	void __iomem *md_ccif;
	void __iomem *infracfg;
	void __iomem *spm;
	void __iomem *topckgen;
	void __iomem *bus_prot_base[2];
	void __iomem *dpmaif_ao;
	void __iomem *dpmaif_pd;
	struct {
		void *cpu;
		dma_addr_t dma;
		size_t size;
	} dpmaif_pit, dpmaif_bat, dpmaif_frg, dpmaif_bat_buf, dpmaif_frg_buf,
	  dpmaif_drb[DPMAIF_TXQ_NUM];

	/* data path: one RX queue, uplink on TX queue 0 */
	int irq_dpmaif;
	bool dpmaif_up;
	struct net_device *napi_dev;
	struct napi_struct napi;
	struct net_device *ccmni[CCMNI_NUM];
	u16 *bat_buf_id;		/* BAT entry -> buffer */
	u16 *frg_buf_id;
	u32 bat_wr, frg_wr, pit_rd;
	u8 pit_seq;
	struct sk_buff *rx_skb;		/* packet being assembled */
	int rx_channel;
	bool rx_drop;
	spinlock_t tx_lock;
	u32 drb_wr, drb_rel;
	struct {
		struct sk_buff *skb;
		dma_addr_t dma;
		u32 len;
	} *drb_skb;
	void *tag_mem;
	size_t tag_size;
	void __iomem *nc_smem;
	phys_addr_t nc_smem_phys;
	size_t nc_smem_size;
	void __iomem *c_smem;
	phys_addr_t c_smem_phys;
	size_t c_smem_size;

	/* From the bootloader tags */
	phys_addr_t md_mem_base;
	u32 md_mem_size;
	u32 md_cap;
	u32 md_img_size;
	u32 c_smem_md_offset;
	u32 nc_smem_used;		/* end of the last non-cached region */
	struct {
		u32 ap_offset;		/* in nc_smem or c_smem */
		u32 md_addr;
		u32 size;
		bool cached;
		bool valid;
	} smem[SMEM_MAX];

	struct ccci_ring *rings;	/* in use: normal or exception */
	struct ccci_ring ring[CCIF_NUM_QUEUES];
	struct ccci_ring exp_ring[CCIF_NUM_QUEUES];
	u32 ccism_size;
	u32 ccism_exp_size;

	int irq;
	int irq_exp;
	bool irqs_on;
	bool dvfsrc_started;
	struct work_struct rx_work;
	unsigned long pending_ch;

	struct mutex lock;		/* held during a start */
	enum md_state state;
	wait_queue_head_t state_wq;
	u32 last_rx_len;
	u16 tx_seq[CCCI_MAX_CH];
	u16 rx_seq[CCCI_MAX_CH];

	struct ccci_port ports[ARRAY_SIZE(ccci_port_cfg)];
};

static void ccci_set_state(struct mtk_ccci *ccci, enum md_state state)
{
	ccci->state = state;
	dev_info(ccci->dev, "modem state: %s\n", md_state_names[state]);
	sysfs_notify(&ccci->dev->kobj, NULL, "state");
	wake_up_all(&ccci->state_wq);
}

static unsigned long ccci_smc(unsigned long req, unsigned long sub,
			      struct arm_smccc_res *res)
{
	arm_smccc_smc(MTK_SIP_CCCI_CONTROL, req, sub, 0, 0, 0, 0, 0, res);
	return res->a0;
}

static void __iomem *ccci_smem_vaddr(struct mtk_ccci *ccci, int id)
{
	if (!ccci->smem[id].valid || ccci->smem[id].cached)
		return NULL;
	return ccci->nc_smem + ccci->smem[id].ap_offset;
}

/* Bootloader tags */

static const void *ccci_find_tag(struct mtk_ccci *ccci, const char *name,
				 u32 *size)
{
	u32 off = 0;
	int i;

	for (i = 0; i < 64 && off + sizeof(struct ccci_tag) <= ccci->tag_size; i++) {
		const struct ccci_tag *tag = ccci->tag_mem + off;

		if (!strncmp(tag->name, name, sizeof(tag->name))) {
			if (tag->data_offset + tag->data_size > ccci->tag_size)
				return NULL;
			*size = tag->data_size;
			return ccci->tag_mem + tag->data_offset;
		}
		if (!tag->next_tag_offset || tag->next_tag_offset <= off)
			break;
		off = tag->next_tag_offset;
	}

	return NULL;
}

static int ccci_parse_smem_layout(struct mtk_ccci *ccci, const char *tag,
				  const char *num_tag, bool cached)
{
	const struct ccci_tag_smem_region *r;
	const u32 *num;
	u32 size, n, i;
	phys_addr_t base = cached ? ccci->c_smem_phys : ccci->nc_smem_phys;
	size_t total = cached ? ccci->c_smem_size : ccci->nc_smem_size;

	num = ccci_find_tag(ccci, num_tag, &size);
	r = ccci_find_tag(ccci, tag, &size);
	if (!num || !r)
		return -ENOENT;

	n = min_t(u32, *num, size / sizeof(*r));
	for (i = 0; i < n; i++, r++) {
		if (r->flags & SMEM_ATTR_PADDING || r->id < 0 || r->id >= SMEM_MAX)
			continue;
		if (r->ap_phy != base + r->ap_offset ||
		    r->ap_offset + r->size > total) {
			dev_err(ccci->dev, "smem region %d out of range\n", r->id);
			return -EINVAL;
		}
		ccci->smem[r->id].ap_offset = r->ap_offset;
		ccci->smem[r->id].md_addr = MD_BANK4_MD_VIEW + r->md_offset;
		ccci->smem[r->id].size = r->size;
		ccci->smem[r->id].cached = cached;
		ccci->smem[r->id].valid = true;
		if (!cached)
			ccci->nc_smem_used = max(ccci->nc_smem_used,
						 r->ap_offset + r->size);
	}

	return 0;
}

static int ccci_parse_tags(struct mtk_ccci *ccci)
{
	const struct ccci_tag_hdr_tbl *hdr;
	const u32 *val;
	const char *s;
	u32 size;
	int ret;

	hdr = ccci_find_tag(ccci, "hdr_tbl_inf", &size);
	if (!hdr || size < sizeof(*hdr)) {
		dev_err(ccci->dev, "no modem image info from the bootloader\n");
		return -ENODEV;
	}
	if (hdr->errno) {
		dev_err(ccci->dev, "bootloader failed to load the modem: %d\n",
			hdr->errno);
		return -ENODEV;
	}
	ccci->md_mem_base = hdr->base_addr;
	ccci->md_mem_size = hdr->size;

	val = ccci_find_tag(ccci, "md_capability", &size);
	ccci->md_cap = val ? *val : 0;
	val = ccci_find_tag(ccci, "md1img", &size);
	ccci->md_img_size = val ? *val : 0;
	val = ccci_find_tag(ccci, "md1_smem_cahce_offset", &size);
	ccci->c_smem_md_offset = val ? *val : 0x8000000;

	ret = ccci_parse_smem_layout(ccci, "nc_smem_layout", "nc_smem_layout_num", false);
	if (ret)
		return ret;
	ret = ccci_parse_smem_layout(ccci, "c_smem_layout", "c_smem_layout_num", true);
	if (ret)
		return ret;

	s = ccci_find_tag(ccci, "md_build_ver", &size);
	dev_info(ccci->dev, "modem %.*s at %pa, size %#x\n",
		 s ? (int)strnlen(s, size) : 1, s ?: "?", &ccci->md_mem_base,
		 ccci->md_mem_size);

	return 0;
}

/* Ring buffers */

static int ccci_ring_init(struct mtk_ccci *ccci, struct ccci_ring *rings,
			  int smem_id, const u32 *rx_size, const u32 *tx_size,
			  u32 *total)
{
	u8 __iomem *buf = ccci_smem_vaddr(ccci, smem_id);
	u32 off = 0;
	int i;

	if (!buf)
		return -ENOENT;

	memset_io(buf, 0, ccci->smem[smem_id].size);
	for (i = 0; i < CCIF_NUM_QUEUES; i++) {
		u32 len = RBF_CTL_LEN + rx_size[i] + tx_size[i];
		u8 __iomem *b = buf + off;

		if (off + len > ccci->smem[smem_id].size)
			return -ENOSPC;

		writel(RBF_HEADER, b);
		writel(RBF_HEADER, b + 4);
		writel(RBF_FOOTER, b + len - 8);
		writel(RBF_FOOTER, b + len - 4);
		rings[i].ctl = (void __iomem *)(b + 8);
		writel(rx_size[i], &rings[i].ctl->rx_length);
		writel(tx_size[i], &rings[i].ctl->tx_length);
		rings[i].rx_buf = b + 8 + sizeof(struct ccci_ringbuf_ctl);
		rings[i].tx_buf = rings[i].rx_buf + rx_size[i];
		spin_lock_init(&rings[i].tx_lock);
		off += len;
	}
	*total = off;

	return 0;
}

static void ring_copy_from(u8 *dst, const u8 __iomem *buf, u32 pos, u32 len,
			   u32 size)
{
	u32 n = min(len, size - pos);

	memcpy_fromio(dst, buf + pos, n);
	if (n < len)
		memcpy_fromio(dst + n, buf, len - n);
}

static void ring_copy_to(u8 __iomem *buf, u32 pos, const u8 *src, u32 len,
			 u32 size)
{
	u32 n = min(len, size - pos);

	memcpy_toio(buf + pos, src, n);
	if (n < len)
		memcpy_toio(buf, src + n, len - n);
}

/* Returns the length of the next packet (CCCI header included) or < 0 */
static int ccci_ring_peek(struct ccci_ring *ring)
{
	u32 rd = readl(&ring->ctl->rx_read);
	u32 wr = readl(&ring->ctl->rx_write);
	u32 size = readl(&ring->ctl->rx_length);
	u32 hdr[2], ftr[2], avail, pkg, fpos;

	if (!size || rd >= size || wr >= size)
		return -EINVAL;
	avail = wr >= rd ? wr - rd : wr + size - rd;
	if (avail < 8 + 8 + sizeof(struct ccci_header))
		return -EAGAIN;

	ring_copy_from((u8 *)hdr, ring->rx_buf, rd, 8, size);
	if (hdr[0] != PKG_HEADER)
		return -EBADMSG;
	pkg = ALIGN(8 + hdr[1] + 8, 8);
	if (hdr[1] < sizeof(struct ccci_header) || pkg > avail)
		return -EBADMSG;
	fpos = (rd + pkg - 8) % size;
	ring_copy_from((u8 *)ftr, ring->rx_buf, fpos, 8, size);
	if (ftr[0] != PKG_FOOTER || ftr[1] != PKG_FOOTER)
		return -EBADMSG;

	return hdr[1];
}

static void ccci_ring_read(struct ccci_ring *ring, u8 *dst, u32 len)
{
	u32 rd = readl(&ring->ctl->rx_read);
	u32 size = readl(&ring->ctl->rx_length);

	ring_copy_from(dst, ring->rx_buf, (rd + 8) % size, len, size);
	rd = (ALIGN(rd + 8 + len + 8, 8)) % size;
	mb();
	writel(rd, &ring->ctl->rx_read);
}

static int ccci_ring_write(struct ccci_ring *ring, const u8 *data, u32 len)
{
	u32 rd = readl(&ring->ctl->tx_read);
	u32 wr = readl(&ring->ctl->tx_write);
	u32 size = readl(&ring->ctl->tx_length);
	u32 hdr[2] = { PKG_HEADER, len };
	u32 ftr[2] = { PKG_FOOTER, PKG_FOOTER };
	u32 need = ALIGN(8 + len + 8, 8), space;

	if (!size || rd >= size || wr >= size)
		return -EINVAL;
	space = rd > wr ? rd - wr - 1 : size - wr + rd - 1;
	if (need >= space)
		return -EBUSY;

	ring_copy_to(ring->tx_buf, wr, (u8 *)hdr, 8, size);
	ring_copy_to(ring->tx_buf, (wr + 8) % size, data, len, size);
	ring_copy_to(ring->tx_buf, (wr + need - 8) % size, (u8 *)ftr, 8, size);
	mb();
	writel((wr + need) % size, &ring->ctl->tx_write);
	mb();

	return 0;
}

static void ccci_ccif_kick(struct mtk_ccci *ccci, int ch)
{
	if (readl(ccci->ap_ccif + CCIF_BUSY) & BIT(ch))
		return;
	writel(BIT(ch), ccci->ap_ccif + CCIF_BUSY);
	writel(ch, ccci->ap_ccif + CCIF_TCHNUM);
}

static int ccci_send(struct mtk_ccci *ccci, int qno, struct ccci_header *hdr,
		     const void *payload, u32 payload_len, bool blocking)
{
	struct ccci_ring *ring = &ccci->rings[qno];
	u32 len = sizeof(*hdr) + payload_len;
	u16 ch = le16_to_cpu(hdr->channel);
	unsigned long flags;
	u8 *pkt;
	int ret, tries = 0;
	u16 seq;

	pkt = kmalloc(len, GFP_KERNEL);
	if (!pkt)
		return -ENOMEM;

	do {
		spin_lock_irqsave(&ring->tx_lock, flags);
		seq = ccci->tx_seq[ch] & 0x7fff;
		/*
		 * The modem checks the order of packets with the assert bit
		 * set; FS and RPC replies are only ordered during the boot.
		 */
		if (ccci->state == MD_STATE_WAIT_HS2 ||
		    (ch != CCCI_FS_TX && ch != CCCI_RPC_TX))
			seq |= CCCI_SEQ_ASSERT;
		hdr->seq = cpu_to_le16(seq);
		memcpy(pkt, hdr, sizeof(*hdr));
		memcpy(pkt + sizeof(*hdr), payload, payload_len);
		ret = ccci_ring_write(ring, pkt, len);
		if (!ret) {
			ccci->tx_seq[ch]++;
			ccci_ccif_kick(ccci, qno);
		}
		spin_unlock_irqrestore(&ring->tx_lock, flags);
		if (ret != -EBUSY || !blocking)
			break;
		msleep(5);
	} while (++tries < 400);

	kfree(pkt);
	return ret;
}

/* Boot handshake */

static void rt_append(u8 __iomem **p, u8 id, u8 support, const void *data,
		      u32 len)
{
	struct ccci_runtime_feature f = {
		.feature_id = id,
		.support_info = support,
		.data_len = len,
	};

	memcpy_toio(*p, &f, sizeof(f));
	*p += sizeof(f);
	if (len) {
		memcpy_toio(*p, data, len);
		*p += len;
	}
}

static u8 ccci_ap_feature(struct mtk_ccci *ccci, int id)
{
	switch (id) {
	case BOOT_INFO:
	case EXCEPTION_SHARE_MEMORY:
	case CCIF_SHARE_MEMORY:
	case CCISM_SHARE_MEMORY_EXP:
	case MD_CONSYS_SHARE_MEMORY:
	case CCB_SHARE_MEMORY:
	case DHL_RAW_SHARE_MEMORY:
	case LWA_SHARE_MEMORY:
	case DT_NETD_SHARE_MEMORY:
	case DT_USB_SHARE_MEMORY:
	case AUDIO_RAW_SHARE_MEMORY:
	case MISC_INFO_HIF_DMA_REMAP:
	case MULTI_MD_MPU:
	case MISC_INFO_RTC_32K_LESS:
	case MISC_INFO_RANDOM_SEED_NUM:
	case MISC_INFO_GPS_COCLOCK:
	case MISC_INFO_SBP_ID:
	case MISC_INFO_CCCI:
	case MISC_INFO_CLIB_TIME:
	case MISC_INFO_C2K:
	case EE_AFTER_EPOF:
	case AP_CCMNI_MTU:
	case NVRAM_CACHE_SHARE_MEMORY:
	case AMMS_DRDI_COPY:
		return FEATURE_MUST_SUPPORT;
	case MD_IMAGE_START_MEMORY:
	case MD_USIP_SHARE_MEMORY:
	case MD_MTEE_SHARE_MEMORY_ENABLE:
	case MD_WIFI_PROXY_SHARE_MEMORY:
	case MD_MEM_AP_VIEW_INF:
	case MD_POST_DUMP:
		return FEATURE_OPTIONAL_SUPPORT;
	/* there is no SCP firmware to talk to */
	case CCISM_SHARE_MEMORY:
	case MD_PHY_CAPTURE:
	case MD1MD3_SHARE_MEMORY:
	case UDC_RAW_SHARE_MEMORY:
	case MD_POS_SHARE_MEMORY:
	case MISC_INFO_CUSTOMER_VAL:
	case MISC_INFO_C2K_MEID:
	case SMART_LOGGING_SHARE_MEMORY:
	case SECURITY_SHARE_MEMORY:
		return FEATURE_NOT_SUPPORT;
	default:
		return FEATURE_NOT_EXIST;
	}
}

static void ccci_smem_info(struct mtk_ccci *ccci, int smem_id, u32 *out)
{
	if (ccci->smem[smem_id].valid) {
		out[0] = ccci->smem[smem_id].md_addr;
		out[1] = ccci->smem[smem_id].size;
	} else {
		/* like the stock driver: empty region at the end of the others */
		out[0] = MD_BANK4_MD_VIEW + ccci->nc_smem_used;
		out[1] = 0;
	}
}

static int ccci_prepare_runtime_data(struct mtk_ccci *ccci,
				     const struct md_query_ap_feature *md)
{
	u8 __iomem *start = ccci_smem_vaddr(ccci, SMEM_RAW_RUNTIME_DATA);
	u8 __iomem *p = start;
	bool hires_time = false;
	int i;

	if (!p)
		return -ENOENT;
	if (md->head_pattern != MD_FEATURE_QUERY_PATTERN ||
	    md->tail_pattern != MD_FEATURE_QUERY_PATTERN) {
		dev_err(ccci->dev, "bad modem feature query %#x/%#x\n",
			md->head_pattern, md->tail_pattern);
		return -EINVAL;
	}

	for (i = 0; i < FEATURE_COUNT; i++) {
		u8 md_mask = md->feature_set[i] & 0xf;
		u8 md_ver = md->feature_set[i] >> 4;
		u8 ap_mask = ccci_ap_feature(ccci, i);
		u8 ap_ver = 0, support;
		u32 d[8] = { };
		u32 len = 0;

		if (i == MISC_INFO_CLIB_TIME && md_ver == 1) {
			ap_ver = 1;
			hires_time = true;
		}

		if (md_mask == FEATURE_MUST_SUPPORT && ap_mask < FEATURE_MUST_SUPPORT) {
			dev_err(ccci->dev, "modem requires unsupported feature %d\n", i);
			return -EOPNOTSUPP;
		}

		switch (md_mask) {
		case FEATURE_NOT_EXIST:
		case FEATURE_MUST_SUPPORT:
			support = md->feature_set[i];
			break;
		case FEATURE_OPTIONAL_SUPPORT:
			if (md_ver == ap_ver && ap_mask >= FEATURE_MUST_SUPPORT)
				support = FEATURE_MUST_SUPPORT | ap_ver << 4;
			else
				support = FEATURE_NOT_SUPPORT | ap_ver << 4;
			break;
		case FEATURE_BACKWARD_COMPAT:
			if (md_ver >= ap_ver)
				support = FEATURE_MUST_SUPPORT | ap_ver << 4;
			else
				support = FEATURE_NOT_SUPPORT | ap_ver << 4;
			break;
		default:
			/* what the modem does not support is reported as 0 */
			support = 0;
			break;
		}

		if ((support & 0xf) != FEATURE_MUST_SUPPORT) {
			rt_append(&p, i, support, NULL, 0);
			continue;
		}

		switch (i) {
		case BOOT_INFO:
			/* control channel, normal boot, "phone" (not EVB) */
			d[0] = CCCI_CONTROL_RX;
			d[1] = MD_BOOT_START_ID;
			d[2] = BIT(1);
			len = 16;
			break;
		case EXCEPTION_SHARE_MEMORY:
			ccci_smem_info(ccci, SMEM_RAW_MDCCCI_DBG, d);
			d[1] = EE_SMEM_SIZE;
			len = 8;
			break;
		case CCIF_SHARE_MEMORY:
			d[0] = ccci->smem[SMEM_CCISM_MCU].md_addr;
			d[1] = ccci->ccism_size;
			len = 8;
			break;
		case CCISM_SHARE_MEMORY_EXP:
			ccci_smem_info(ccci, SMEM_CCISM_MCU_EXP, d);
			len = 8;
			break;
		case CCB_SHARE_MEMORY:
			ccci_smem_info(ccci, SMEM_RAW_CCB_CTRL, d);
			ccci_smem_info(ccci, SMEM_CCB_DHL, d + 2);
			d[3] = CCB_DATA_SIZE;
			len = 16;
			break;
		case DHL_RAW_SHARE_MEMORY:
			ccci_smem_info(ccci, SMEM_CCB_DHL, d);
			d[0] += CCB_DATA_REGION_SIZE;
			d[1] -= CCB_DATA_REGION_SIZE;
			len = 8;
			break;
		case CCISM_SHARE_MEMORY:
			ccci_smem_info(ccci, SMEM_CCISM_SCP, d);
			len = 8;
			break;
		case LWA_SHARE_MEMORY:
			ccci_smem_info(ccci, SMEM_RAW_LWA, d);
			len = 8;
			break;
		case DT_NETD_SHARE_MEMORY:
			ccci_smem_info(ccci, SMEM_RAW_NETD, d);
			len = 8;
			break;
		case DT_USB_SHARE_MEMORY:
			ccci_smem_info(ccci, SMEM_RAW_USB, d);
			len = 8;
			break;
		case AUDIO_RAW_SHARE_MEMORY:
			ccci_smem_info(ccci, SMEM_RAW_AUDIO, d);
			len = 8;
			break;
		case MD_CONSYS_SHARE_MEMORY:
			ccci_smem_info(ccci, SMEM_RAW_MD_CONSYS, d);
			len = 8;
			break;
		case MD_USIP_SHARE_MEMORY:
			ccci_smem_info(ccci, SMEM_RAW_USIP, d);
			len = 8;
			break;
		case MD_WIFI_PROXY_SHARE_MEMORY:
			ccci_smem_info(ccci, SMEM_MD_WIFI_PROXY, d);
			len = 8;
			break;
		case NVRAM_CACHE_SHARE_MEMORY:
			ccci_smem_info(ccci, SMEM_MD_NVRAM_CACHE, d);
			len = 8;
			break;
		case AMMS_DRDI_COPY:
			ccci_smem_info(ccci, SMEM_MD_DRDI, d);
			len = 8;
			break;
		case MD_POST_DUMP:
			ccci_smem_info(ccci, SMEM_MD_POST_DUMP, d);
			len = 8;
			break;
		case MD_IMAGE_START_MEMORY:
			d[0] = lower_32_bits(ccci->md_mem_base);
			d[1] = ccci->md_img_size;
			len = 8;
			break;
		case MISC_INFO_HIF_DMA_REMAP:
		case MISC_INFO_RTC_32K_LESS:
		case MISC_INFO_GPS_COCLOCK:
		case EE_AFTER_EPOF:
			len = 16;
			break;
		case MISC_INFO_RANDOM_SEED_NUM:
			d[0] = get_random_u32();
			len = 16;
			break;
		case MISC_INFO_SBP_ID:
			d[0] = 0;
			d[1] = MD_RAT_NLWG;
			len = 16;
			break;
		case MISC_INFO_CCCI:
			/* sequence check, AP polls the modem status */
			d[0] = BIT(0) | BIT(1);
			len = 16;
			break;
		case MISC_INFO_CLIB_TIME: {
			struct timespec64 t;
			u64 us, cnt;

			ktime_get_real_ts64(&t);
			d[0] = lower_32_bits(t.tv_sec);
			d[1] = upper_32_bits(t.tv_sec);
			if (hires_time) {
				us = t.tv_nsec / NSEC_PER_USEC;
				cnt = __arch_counter_get_cntvct();
				d[4] = lower_32_bits(us);
				d[5] = upper_32_bits(us);
				d[6] = lower_32_bits(cnt);
				d[7] = upper_32_bits(cnt);
				len = 32;
			} else {
				len = 16;
			}
			break;
		}
		case MISC_INFO_C2K:
			len = 16;
			break;
		case AP_CCMNI_MTU:
			d[0] = CCMNI_MTU - sizeof(struct ccci_header);
			len = 4;
			break;
		case MD_MTEE_SHARE_MEMORY_ENABLE:
			len = 4;
			break;
		case MD_MEM_AP_VIEW_INF: {
			/* bank 0 (modem image), bank 4 non-cached and cached */
			u32 t[12] = {
				0, ccci->md_mem_size,
				lower_32_bits(ccci->md_mem_base),
				upper_32_bits(ccci->md_mem_base),
				MD_BANK4_MD_VIEW, ccci->nc_smem_size,
				lower_32_bits(ccci->nc_smem_phys),
				upper_32_bits(ccci->nc_smem_phys),
				MD_BANK4_MD_VIEW + ccci->c_smem_md_offset,
				ccci->c_smem_size,
				lower_32_bits(ccci->c_smem_phys),
				upper_32_bits(ccci->c_smem_phys),
			};

			rt_append(&p, i, support, t, sizeof(t));
			continue;
		}
		case MULTI_MD_MPU:
		default:
			break;
		}
		rt_append(&p, i, support, d, len);
	}

	if (p - start > RUNTIME_AP_SIZE) {
		dev_err(ccci->dev, "runtime data too long: %td\n", p - start);
		return -E2BIG;
	}
	dev_dbg(ccci->dev, "runtime data: %td bytes\n", p - start);

	return 0;
}

/* The CCIF SRAM only takes 32 bit accesses */
static void ccif_sram_write(void __iomem *dst, const void *src, size_t len)
{
	const u32 *s = src;
	size_t i;

	for (i = 0; i < len / 4; i++)
		writel(s[i], dst + i * 4);
}

static void ccif_sram_read(void *dst, const void __iomem *src, size_t len)
{
	u32 *d = dst;
	size_t i;

	for (i = 0; i < len / 4; i++)
		d[i] = readl(src + i * 4);
}

static void ccci_send_runtime_data(struct mtk_ccci *ccci)
{
	void __iomem *sram = ccci->ap_ccif + CCIF_CHDATA;
	struct ap_query_md_feature_v2_1 ap = { };
	struct ccci_header hdr = { };

	ap.head_pattern = AP_FEATURE_QUERY_PATTERN;
	ap.tail_pattern = AP_FEATURE_QUERY_PATTERN;
	/* feature 1 version 1: the MPU size covers all the AP/MD memory */
	ap.feature_set[1] = 1 << 4;
	ap.share_memory_support = MULTI_MD_MPU_SUPPORT;
	ap.ap_runtime_data_addr = ccci->smem[SMEM_RAW_RUNTIME_DATA].md_addr;
	ap.ap_runtime_data_size = RUNTIME_AP_SIZE;
	ap.md_runtime_data_addr = ap.ap_runtime_data_addr + RUNTIME_AP_SIZE;
	ap.md_runtime_data_size = RUNTIME_MD_SIZE;
	ap.noncached_mpu_start_addr = MD_BANK4_MD_VIEW;
	ap.noncached_mpu_total_size = ccci->nc_smem_used;
	ap.cached_mpu_start_addr = MD_BANK4_MD_VIEW + ccci->c_smem_md_offset;
	ap.cached_mpu_total_size = ccci->c_smem_size;

	hdr.data[1] = cpu_to_le32(sizeof(hdr) + sizeof(ap));
	hdr.channel = cpu_to_le16(CCCI_CONTROL_TX);
	hdr.reserved = cpu_to_le32(MD_INIT_CHK_ID);

	ccif_sram_write(sram + SRAM_UP_HEADER, &hdr, sizeof(hdr));
	ccif_sram_write(sram + SRAM_AP_RT_DATA, &ap, sizeof(ap));
	mb();
	ccci_ccif_kick(ccci, CCIF_CH_SRAM);
}

static void ccci_handle_hs1(struct mtk_ccci *ccci)
{
	void __iomem *sram = ccci->ap_ccif + CCIF_CHDATA;
	struct md_query_ap_feature md;
	struct ccci_header hdr;
	int ret;

	ccif_sram_read(&hdr, sram + SRAM_DL_HEADER, sizeof(hdr));
	ccif_sram_read(&md, sram + SRAM_MD_RT_DATA, sizeof(md));

	if (le16_to_cpu(hdr.channel) != CCCI_CONTROL_RX ||
	    le32_to_cpu(hdr.data[1]) != MD_INIT_START_BOOT ||
	    le32_to_cpu(hdr.reserved) != MD_INIT_CHK_ID) {
		dev_warn(ccci->dev, "unexpected SRAM message %08x %08x %08x %08x\n",
			 le32_to_cpu(hdr.data[0]), le32_to_cpu(hdr.data[1]),
			 le32_to_cpu(hdr.channel) | le16_to_cpu(hdr.seq) << 16,
			 le32_to_cpu(hdr.reserved));
		return;
	}
	if (ccci->state != MD_STATE_WAIT_HS1) {
		dev_warn(ccci->dev, "handshake 1 in state %s\n",
			 md_state_names[ccci->state]);
		return;
	}

	dev_info(ccci->dev, "handshake 1\n");
	ccci_set_state(ccci, MD_STATE_WAIT_HS2);
	ret = ccci_prepare_runtime_data(ccci, &md);
	if (ret) {
		ccci_set_state(ccci, MD_STATE_EXCEPTION);
		return;
	}
	ccci_send_runtime_data(ccci);
}

static void ccci_handle_control(struct mtk_ccci *ccci, struct ccci_header *hdr)
{
	u32 msg = le32_to_cpu(hdr->data[1]);

	if (le32_to_cpu(hdr->data[0]) != CCCI_CTRL_MAGIC)
		goto unknown;

	switch (msg) {
	case MD_INIT_START_BOOT:
		if (le32_to_cpu(hdr->reserved) == MD_INIT_CHK_ID)
			goto unknown;
		if (ccci->state == MD_STATE_WAIT_HS2) {
			dev_info(ccci->dev, "handshake 2\n");
			ccci_set_state(ccci, MD_STATE_READY);
		}
		return;
	case MD_EX:
	case MD_EX_REC_OK:
	case MD_EX_PASS:
		dev_err(ccci->dev, "modem exception (%u, %#x)\n", msg,
			le32_to_cpu(hdr->reserved));
		if (msg == MD_EX_REC_OK)
			print_hex_dump(KERN_ERR, "md ex: ", DUMP_PREFIX_OFFSET, 16, 1,
				       hdr + 1, min_t(u32, ccci->last_rx_len -
						      sizeof(*hdr), 1024), true);
		ccci_set_state(ccci, MD_STATE_EXCEPTION);
		return;
	}

unknown:
	dev_info(ccci->dev, "control message %08x %08x %08x\n",
		 le32_to_cpu(hdr->data[0]), msg, le32_to_cpu(hdr->reserved));
}

static void ccci_handle_system(struct mtk_ccci *ccci, struct ccci_header *hdr)
{
	dev_info(ccci->dev, "system message %#x %#x\n",
		 le32_to_cpu(hdr->data[1]), le32_to_cpu(hdr->reserved));
}

/* RX */

static struct ccci_port *ccci_port_by_rx(struct mtk_ccci *ccci, u16 ch)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ccci->ports); i++)
		if (ccci->ports[i].rx_ch == ch)
			return &ccci->ports[i];
	return NULL;
}

static void ccci_dispatch(struct mtk_ccci *ccci, struct sk_buff *skb)
{
	struct ccci_header *hdr = (void *)skb->data;
	u16 ch = le16_to_cpu(hdr->channel);
	u16 seq = le16_to_cpu(hdr->seq);
	struct ccci_port *port;

	if (ch < CCCI_MAX_CH) {
		if (seq & CCCI_SEQ_ASSERT && ccci->rx_seq[ch] &&
		    ((seq - ccci->rx_seq[ch]) & 0x7fff) != 1)
			dev_warn(ccci->dev, "ch %u: sequence %u after %u\n", ch,
				 seq & 0x7fff, ccci->rx_seq[ch]);
		ccci->rx_seq[ch] = seq & 0x7fff;
	}

	ccci->last_rx_len = skb->len;
	switch (ch) {
	case CCCI_CONTROL_RX:
		ccci_handle_control(ccci, hdr);
		break;
	case CCCI_SYSTEM_RX:
		ccci_handle_system(ccci, hdr);
		break;
	default:
		port = ccci_port_by_rx(ccci, ch);
		if (!port) {
			dev_info_ratelimited(ccci->dev, "dropping packet on channel %u\n", ch);
			break;
		}
		if (!port->user_header)
			skb_pull(skb, sizeof(*hdr));
		if (skb_queue_len(&port->rx_list) >= CCCI_PORT_RX_MAX)
			kfree_skb(skb_dequeue(&port->rx_list));
		skb_queue_tail(&port->rx_list, skb);
		wake_up_interruptible(&port->rx_wq);
		return;
	}
	kfree_skb(skb);
}

static void ccci_rx_queue(struct mtk_ccci *ccci, int qno)
{
	struct ccci_ring *ring = &ccci->rings[qno];
	struct sk_buff *skb;
	int len, budget = 64;

	while (budget--) {
		len = ccci_ring_peek(ring);
		if (len == -EAGAIN)
			return;
		if (len < 0) {
			dev_err_ratelimited(ccci->dev, "q%d: corrupted ring (%d)\n",
					    qno, len);
			return;
		}
		skb = alloc_skb(len, GFP_KERNEL);
		if (!skb)
			return;
		ccci_ring_read(ring, skb_put(skb, len), len);
		ccci_dispatch(ccci, skb);
	}
	/* more to do */
	set_bit(qno, &ccci->pending_ch);
	queue_work(system_highpri_wq, &ccci->rx_work);
}

static void ccci_rx_work(struct work_struct *work)
{
	struct mtk_ccci *ccci = container_of(work, struct mtk_ccci, rx_work);
	int i;

	if (test_and_clear_bit(CCIF_D2H_EX_INIT, &ccci->pending_ch)) {
		dev_err(ccci->dev, "modem exception, handshaking\n");
		ccci_set_state(ccci, MD_STATE_EXCEPTION);
		ccci_ccif_kick(ccci, CCIF_H2D_EX_ACK);
	}
	if (test_and_clear_bit(CCIF_D2H_EX_CLEARQ_DONE, &ccci->pending_ch)) {
		for (i = 0; i < CCIF_NUM_QUEUES; i++) {
			writel(0, &ccci->exp_ring[i].ctl->rx_read);
			writel(0, &ccci->exp_ring[i].ctl->rx_write);
			writel(0, &ccci->exp_ring[i].ctl->tx_read);
			writel(0, &ccci->exp_ring[i].ctl->tx_write);
		}
		ccci->rings = ccci->exp_ring;
		ccci_ccif_kick(ccci, CCIF_H2D_EX_CLEARQ_ACK);
	}
	clear_bit(CCIF_D2H_EX_INIT_DONE, &ccci->pending_ch);
	clear_bit(CCIF_D2H_EX_ALLQ_RESET, &ccci->pending_ch);

	if (test_and_clear_bit(CCIF_CH_SRAM, &ccci->pending_ch))
		ccci_handle_hs1(ccci);

	for (i = 0; i < CCIF_NUM_QUEUES; i++)
		if (i != CCIF_CH_SRAM && test_and_clear_bit(i, &ccci->pending_ch))
			ccci_rx_queue(ccci, i);
}

static irqreturn_t ccci_ccif_irq(int irq, void *data)
{
	struct mtk_ccci *ccci = data;
	u32 ch = readl(ccci->ap_ccif + CCIF_RCHNUM);
	unsigned long bits = ch & 0xffff;
	int i;

	writel(bits, ccci->ap_ccif + CCIF_ACK);
	trace_printk("ccif irq %#x\n", ch);
	for_each_set_bit(i, &bits, CCIF_NUM_QUEUES)
		set_bit(i, &ccci->pending_ch);
	queue_work(system_highpri_wq, &ccci->rx_work);

	return IRQ_HANDLED;
}

static irqreturn_t ccci_ccif_exp_irq(int irq, void *data)
{
	struct mtk_ccci *ccci = data;
	u32 ch = readl(ccci->ap_ccif + CCIF_RCHNUM);
	unsigned long bits = ch & GENMASK(23, 16);
	int i;

	writel(bits, ccci->ap_ccif + CCIF_ACK);
	dev_info(ccci->dev, "exception interrupt: %#x\n", ch);
	for_each_set_bit(i, &bits, 24)
		set_bit(i, &ccci->pending_ch);
	queue_work(system_highpri_wq, &ccci->rx_work);

	return IRQ_HANDLED;
}

/* Character devices */

static int ccci_port_open(struct inode *inode, struct file *file)
{
	struct ccci_port *port = container_of(file->private_data,
					      struct ccci_port, misc);

	file->private_data = port;
	return nonseekable_open(inode, file);
}

static ssize_t ccci_port_read(struct file *file, char __user *buf,
			      size_t count, loff_t *ppos)
{
	struct ccci_port *port = file->private_data;
	struct sk_buff *skb;
	size_t len;
	int ret;

	for (;;) {
		skb = skb_dequeue(&port->rx_list);
		if (skb)
			break;
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ret = wait_event_interruptible(port->rx_wq,
					       !skb_queue_empty(&port->rx_list));
		if (ret)
			return ret;
	}

	len = min(skb->len, count);
	if (len < skb->len && port->user_header) {
		skb_queue_head(&port->rx_list, skb);
		return -EMSGSIZE;
	}
	if (copy_to_user(buf, skb->data, len)) {
		skb_queue_head(&port->rx_list, skb);
		return -EFAULT;
	}
	/* stream ports hand out the rest on the next read */
	if (len < skb->len) {
		skb_pull(skb, len);
		skb_queue_head(&port->rx_list, skb);
	} else {
		kfree_skb(skb);
	}

	return len;
}

static ssize_t ccci_port_write(struct file *file, const char __user *buf,
			       size_t count, loff_t *ppos)
{
	struct ccci_port *port = file->private_data;
	struct mtk_ccci *ccci = port->ccci;
	struct ccci_header hdr = { };
	size_t off = 0, len = count;
	void *data;
	int ret;

	if (ccci->state != MD_STATE_READY &&
	    !(port->user_header && (ccci->state == MD_STATE_WAIT_HS1 ||
				    ccci->state == MD_STATE_WAIT_HS2)))
		return -ENODEV;

	if (port->user_header) {
		if (count < sizeof(hdr))
			return -EINVAL;
		if (copy_from_user(&hdr, buf, sizeof(hdr)))
			return -EFAULT;
		off = sizeof(hdr);
		len = count - off;
	} else {
		hdr.data[0] = 0;
		hdr.reserved = 0;
	}
	/* packets can't be split, streams can: write what fits */
	if (len > CCMNI_MTU - sizeof(hdr)) {
		if (port->user_header)
			return -EMSGSIZE;
		len = CCMNI_MTU - sizeof(hdr);
	}
	hdr.channel = cpu_to_le16(port->tx_ch);
	hdr.data[1] = cpu_to_le32(sizeof(hdr) + len);

	data = memdup_user(buf + off, len);
	if (IS_ERR(data))
		return PTR_ERR(data);

	mutex_lock(&port->tx_mutex);
	ret = ccci_send(ccci, port->txq, &hdr, data, len,
			!(file->f_flags & O_NONBLOCK));
	mutex_unlock(&port->tx_mutex);
	kfree(data);

	return ret ? ret : off + len;
}

static __poll_t ccci_port_poll(struct file *file, poll_table *wait)
{
	struct ccci_port *port = file->private_data;
	__poll_t mask = 0;

	poll_wait(file, &port->rx_wq, wait);
	if (!skb_queue_empty(&port->rx_list))
		mask |= EPOLLIN | EPOLLRDNORM;
	if (port->ccci->state == MD_STATE_READY)
		mask |= EPOLLOUT | EPOLLWRNORM;
	return mask;
}

/*
 * The RPC server maps the DRDI region: it copies the RF parameter sets of
 * this board variant from the modem image there when the modem asks for it.
 */
static int ccci_port_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct ccci_port *port = file->private_data;
	struct mtk_ccci *ccci = port->ccci;
	unsigned long size = vma->vm_end - vma->vm_start;

	if (port->rx_ch != CCCI_RPC_RX || !ccci->smem[SMEM_MD_DRDI].valid ||
	    ccci->smem[SMEM_MD_DRDI].cached || vma->vm_pgoff ||
	    size > PAGE_ALIGN(ccci->smem[SMEM_MD_DRDI].size))
		return -EINVAL;

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	return remap_pfn_range(vma, vma->vm_start,
			       PHYS_PFN(ccci->nc_smem_phys +
					ccci->smem[SMEM_MD_DRDI].ap_offset),
			       size, vma->vm_page_prot);
}

static const struct file_operations ccci_port_fops = {
	.owner = THIS_MODULE,
	.open = ccci_port_open,
	.read = ccci_port_read,
	.write = ccci_port_write,
	.poll = ccci_port_poll,
	.mmap = ccci_port_mmap,
};

/* Modem start */

static void ccci_ccif_reset(struct mtk_ccci *ccci)
{
	struct ccci_ringbuf_ctl __iomem *ctl;
	int i;

	writel(INFRACFG_CCIF0_RST, ccci->infracfg + INFRACFG_RST_SET);
	writel(INFRACFG_CCIF0_RST, ccci->infracfg + INFRACFG_RST_CLR);

	for (i = 0; i < CCIF_SRAM_SIZE; i += 4) {
		writel(0, ccci->ap_ccif + CCIF_CHDATA + i);
		writel(0, ccci->md_ccif + CCIF_CHDATA + i);
	}

	/* tell the modem where to leave its boot trace */
	writel(0x7274626e, ccci->ap_ccif + CCIF_CHDATA + SRAM_DBG_MAGIC);
	writel(ccci->smem[SMEM_RAW_MDSS_DBG].md_addr,
	       ccci->ap_ccif + CCIF_CHDATA + SRAM_DBG_MAGIC + 4);
	writel(ccci->smem[SMEM_RAW_MDSS_DBG].size,
	       ccci->ap_ccif + CCIF_CHDATA + SRAM_DBG_MAGIC + 8);

	for (i = 0; i < CCIF_NUM_QUEUES; i++) {
		ctl = ccci->ring[i].ctl;
		writel(0, &ctl->rx_read);
		writel(0, &ctl->rx_write);
		writel(0, &ctl->tx_read);
		writel(0, &ctl->tx_write);
	}
	ccci->rings = ccci->ring;
	memset(ccci->tx_seq, 0, sizeof(ccci->tx_seq));
	memset(ccci->rx_seq, 0, sizeof(ccci->rx_seq));
	ccci->pending_ch = 0;
}

/* The CCB buffer layout, which the stock userspace sets up from ccb_configs */
static void ccci_fill_ccb_ctrl(struct mtk_ccci *ccci)
{
	void __iomem *p = ccci_smem_vaddr(ccci, SMEM_RAW_CCB_CTRL);
	int i;

	if (!p || ccci->smem[SMEM_RAW_CCB_CTRL].size <
		  ARRAY_SIZE(ccb_configs) * CCB_CTRL_ENTRY)
		return;
	for (i = 0; i < ARRAY_SIZE(ccb_configs); i++, p += CCB_CTRL_ENTRY) {
		writel(ccb_configs[i].dl_page, p + 0x14);
		writel(ccb_configs[i].dl_size, p + 0x18);
		writel(CCB_CTRL_MAGIC, p + 0x1c);
		writel(ccb_configs[i].ul_page, p + 0x34);
		writel(ccb_configs[i].ul_size, p + 0x38);
		writel(CCB_CTRL_MAGIC, p + 0x3c);
	}
}

static void ccci_clear_smem(struct mtk_ccci *ccci)
{
	static const u8 keep[] = {
		SMEM_SECURITY, SMEM_CCISM_MCU, SMEM_CCISM_MCU_EXP, SMEM_MD_DRDI,
		SMEM_RAW_PHY_CAP, SMEM_RAW_AMMS_POS,
	};
	int i, j;

	for (i = 0; i < SMEM_MAX; i++) {
		void __iomem *p = ccci_smem_vaddr(ccci, i);

		if (!p || !ccci->smem[i].size)
			continue;
		for (j = 0; j < ARRAY_SIZE(keep); j++)
			if (keep[j] == i)
				break;
		if (j == ARRAY_SIZE(keep))
			memset_io(p, 0, ccci->smem[i].size);
	}

	/* cached: the NVRAM cache and the CCB/DHL logging buffers */
	for (i = 0; i < SMEM_MAX; i++) {
		if (!ccci->smem[i].valid || !ccci->smem[i].cached ||
		    (i != SMEM_MD_NVRAM_CACHE && i != SMEM_CCB_DHL))
			continue;
		memset_io(ccci->c_smem + ccci->smem[i].ap_offset, 0,
			  ccci->smem[i].size);
	}

	ccci_fill_ccb_ctrl(ccci);
}

static void dpmaif_rmw(void __iomem *reg, u32 clr, u32 set)
{
	writel((readl(reg) & ~clr) | set, reg);
}

static int dpmaif_wait_idle(void __iomem *reg)
{
	u32 val;

	return readl_poll_timeout_atomic(reg, val, !(val & DPMAIF_INIT_BUSY),
					 1, 100000);
}

static int dpmaif_init_cmd(void __iomem *reg, u32 cmd)
{
	int ret = dpmaif_wait_idle(reg);

	if (ret)
		return ret;
	writel(DPMAIF_INIT_BUSY | cmd, reg);
	return dpmaif_wait_idle(reg);
}

#define DPMAIF_ALLOC(c, field, sz)						\
	({								\
		(c)->field.size = (sz);					\
		(c)->field.cpu = dmam_alloc_coherent((c)->dev, (sz),	\
						     &(c)->field.dma,	\
						     GFP_KERNEL);	\
		(c)->field.cpu ? 0 : -ENOMEM;				\
	})

static int ccci_dpmaif_alloc(struct mtk_ccci *ccci)
{
	int q, ret;

	ret = DPMAIF_ALLOC(ccci, dpmaif_pit, DPMAIF_PIT_CNT * DPMAIF_PIT_SIZE);
	ret = ret ?: DPMAIF_ALLOC(ccci, dpmaif_bat, DPMAIF_BAT_CNT * 8);
	ret = ret ?: DPMAIF_ALLOC(ccci, dpmaif_frg, DPMAIF_BAT_CNT * 8);
	ret = ret ?: DPMAIF_ALLOC(ccci, dpmaif_bat_buf,
				  DPMAIF_BAT_FILL * DPMAIF_BAT_BUF_SIZE);
	ret = ret ?: DPMAIF_ALLOC(ccci, dpmaif_frg_buf,
				  DPMAIF_BAT_FILL * DPMAIF_BAT_BUF_SIZE);
	for (q = 0; q < DPMAIF_TXQ_NUM && !ret; q++)
		ret = DPMAIF_ALLOC(ccci, dpmaif_drb[q], DPMAIF_DRB_CNT * 8);

	return ret;
}

/* Fill a BAT with buffers: 8 byte entries, 40 bit addresses */
static void dpmaif_fill_bat(u32 *bat, dma_addr_t buf)
{
	int i;

	for (i = 0; i < DPMAIF_BAT_FILL; i++) {
		dma_addr_t a = buf + i * DPMAIF_BAT_BUF_SIZE;

		bat[2 * i] = lower_32_bits(a);
		bat[2 * i + 1] = upper_32_bits(a) & 0xff;
	}
	wmb();
}

static void dpmaif_set_bat(struct mtk_ccci *ccci, dma_addr_t addr)
{
	void __iomem *pd = ccci->dpmaif_pd;

	writel(lower_32_bits(addr), pd + DPMAIF_PD_BAT_INIT_CON0);
	dpmaif_rmw(pd + DPMAIF_PD_BAT_INIT_CON1,
		   GENMASK(31, 24) | BAT_CON1_EN | GENMASK(15, 0),
		   (upper_32_bits(addr) & 0xff) << 24 | DPMAIF_BAT_CNT);
}

/* Data path queues, set up as the stock driver does before the boot */
static int ccci_dpmaif_queues_init(struct mtk_ccci *ccci)
{
	void __iomem *pd = ccci->dpmaif_pd;
	void __iomem *ao = ccci->dpmaif_ao;
	dma_addr_t a;
	u32 val;
	int q, ret;

	/* interrupts: clear, unmask the used ones, mask the rest */
	writel(~0, pd + DPMAIF_PD_UL_L2TISAR0);
	writel(DPMAIF_UL_INTR_MASK, ao + DPMAIF_AO_UL_AP_L2TIMCR0);
	writel(~DPMAIF_UL_INTR_MASK, ao + DPMAIF_AO_UL_AP_L2TIMSR0);
	ret = readl_poll_timeout_atomic(ao + DPMAIF_AO_UL_AP_L2TIMR0, val,
			(val & DPMAIF_UL_INTR_MASK) != DPMAIF_UL_INTR_MASK,
			1, 100000);
	if (ret)
		return ret;
	writel(~0, pd + DPMAIF_PD_APDL_L2TISAR0);
	writel(DPMAIF_DL_INTR_MASK, ao + DPMAIF_AO_UL_APDL_L2TIMCR0);
	writel(~DPMAIF_DL_INTR_MASK, ao + DPMAIF_AO_UL_APDL_L2TIMSR0);
	ret = readl_poll_timeout_atomic(ao + DPMAIF_AO_UL_APDL_L2TIMR0, val,
			(val & DPMAIF_DL_INTR_MASK) != DPMAIF_DL_INTR_MASK,
			1, 100000);
	if (ret)
		return ret;
	writel(~0, pd + DPMAIF_PD_AP_IP_BUSY);
	writel(0, ao + DPMAIF_AO_UL_IP_BUSY_MASK);

	/* RX: packet info: 64 byte remain, 1 BID per packet, 64 PIT check */
	dpmaif_rmw(pd + DPMAIF_DL_PKTINFO_CON0, GENMASK(31, 0), 0x00010440);
	writel(3 * 1024 + 8, pd + DPMAIF_DL_PKTINFO_CON1);
	/* 124 PIT check, 1664 byte BAT buffers, no reserved length */
	dpmaif_rmw(pd + DPMAIF_DL_PKTINFO_CON2, GENMASK(31, 24) | GENMASK(16, 0),
		   0x7c000d00);
	/* checksum, 64 byte alignment, 62 BAT check, PIT burst */
	dpmaif_rmw(pd + DPMAIF_DL_RDY_CHK_THRES,
		   BIT(31) | GENMASK(23, 22) | GENMASK(21, 16) | BIT(13),
		   BIT(31) | BIT(23) | 0x3e << 16 | BIT(13));
	dpmaif_rmw(pd + DPMAIF_PD_BAT_INIT_CON1, 0, BAT_CON1_CACHE_EN);
	/* fragment BATs: enabled, 1920 byte buffers, 62 check */
	writel(BIT(28) | 15 << 8 | 62, pd + DPMAIF_DL_RDY_CHK_FRG_THRES);

	a = ccci->dpmaif_pit.dma;
	writel(lower_32_bits(a), pd + DPMAIF_PD_PIT_INIT_CON0);
	writel((upper_32_bits(a) & 0xff) << 24 | DPMAIF_PIT_CNT,
	       pd + DPMAIF_PD_PIT_INIT_CON1);
	dpmaif_rmw(pd + DPMAIF_PD_PIT_INIT_CON3, 0, BIT(0));
	ret = dpmaif_init_cmd(pd + DPMAIF_PD_PIT_INIT, DPMAIF_INIT_ALLSET);
	if (ret)
		return ret;

	dpmaif_set_bat(ccci, ccci->dpmaif_bat.dma);
	ret = dpmaif_init_cmd(pd + DPMAIF_PD_BAT_INIT, DPMAIF_INIT_ALLSET);
	if (ret)
		return ret;
	dpmaif_set_bat(ccci, ccci->dpmaif_frg.dma);
	ret = dpmaif_init_cmd(pd + DPMAIF_PD_BAT_INIT,
			      DPMAIF_INIT_FRG | DPMAIF_INIT_ALLSET);
	if (ret)
		return ret;

	/* hand the buffers to the hardware */
	dpmaif_fill_bat(ccci->dpmaif_frg.cpu, ccci->dpmaif_frg_buf.dma);
	dpmaif_fill_bat(ccci->dpmaif_bat.cpu, ccci->dpmaif_bat_buf.dma);
	ret = dpmaif_wait_idle(pd + DPMAIF_PD_BAT_ADD);
	if (ret)
		return ret;
	writel(DPMAIF_INIT_BUSY | DPMAIF_INIT_FRG | DPMAIF_BAT_FILL,
	       pd + DPMAIF_PD_BAT_ADD);
	ret = dpmaif_wait_idle(pd + DPMAIF_PD_BAT_ADD);
	if (ret)
		return ret;
	writel(DPMAIF_INIT_BUSY | DPMAIF_BAT_FILL, pd + DPMAIF_PD_BAT_ADD);

	dpmaif_rmw(pd + DPMAIF_PD_BAT_INIT_CON1, 0, BAT_CON1_EN);
	ret = dpmaif_init_cmd(pd + DPMAIF_PD_BAT_INIT, DPMAIF_INIT_FRG);
	if (ret)
		return ret;
	dpmaif_rmw(pd + DPMAIF_PD_BAT_INIT_CON1, 0, BAT_CON1_EN);
	ret = dpmaif_init_cmd(pd + DPMAIF_PD_BAT_INIT, 0);
	if (ret)
		return ret;

	/* TX: DRB rings (size in words), ready and arbitration enabled */
	for (q = 0; q < DPMAIF_TXQ_NUM; q++) {
		a = ccci->dpmaif_drb[q].dma;
		dpmaif_rmw(pd + DPMAIF_UL_CHNL_CON1(q), GENMASK(15, 0),
			   DPMAIF_DRB_CNT * 8 / 4);
		writel(lower_32_bits(a), pd + DPMAIF_UL_CHNL_CON0(q));
		dpmaif_rmw(pd + DPMAIF_UL_CHNL_CON1(q), GENMASK(31, 24),
			   (upper_32_bits(a) & 0xff) << 24);
		dpmaif_rmw(ao + DPMAIF_AO_UL_CHNL_ARB0, 0, BIT(q) | BIT(q + 8));
	}

	return 0;
}

static int ccci_dpmaif_init(struct mtk_ccci *ccci)
{
	void __iomem *infra_ao = ccci->bus_prot_base[0];
	u64 base = ccci->md_mem_base;
	u32 val;
	int i, ret;

	/* reset the power-down and always-on parts */
	writel(readl(infra_ao + INFRA_AO_DPMAIF_CTRL) & ~DPMAIF_IP_BUSY_EN,
	       infra_ao + INFRA_AO_DPMAIF_CTRL);
	udelay(500);
	writel(INFRACFG_DPMAIF_PD_RST, ccci->infracfg + INFRACFG_RST_SET);
	udelay(500);
	writel(DPMAIF_AO_RST, infra_ao + INFRA_AO_DPMAIF_RST_SET);
	udelay(500);
	writel(DPMAIF_AO_RST, infra_ao + INFRA_AO_DPMAIF_RST_CLR);
	udelay(500);
	writel(INFRACFG_DPMAIF_PD_RST, ccci->infracfg + INFRACFG_RST_CLR);

	dpmaif_rmw(infra_ao + INFRA_AO_DPMAIF_CFG, BIT(13), BIT(12));
	writel(readl(infra_ao + INFRA_AO_DPMAIF_CTRL) | DPMAIF_IP_BUSY_EN,
	       infra_ao + INFRA_AO_DPMAIF_CTRL);

	/* clear the SRAM */
	writel(readl(ccci->dpmaif_pd + DPMAIF_PD_MISC_MEM_CLR) | BIT(0),
	       ccci->dpmaif_pd + DPMAIF_PD_MISC_MEM_CLR);
	ret = readl_poll_timeout(ccci->dpmaif_pd + DPMAIF_PD_MISC_MEM_CLR, val,
				 !(val & BIT(0)), 10, 100000);
	if (ret)
		return ret;

	writel(0x7f, ccci->dpmaif_pd + DPMAIF_PD_MISC_CG_EN);

	/*
	 * Map the modem banks (32 MB windows) to the modem memory: each
	 * register holds three 10 bit fields, bit 0 of a field enables it.
	 * Bank 0 and 1 (8 windows each) are mapped, bank 4 is not.
	 */
	for (i = 0; i < 8; i++) {
		u32 en = i < 5 ? 0x100401 : i == 5 ? 0x1 : 0;

		val = ((base >> 24) & 0x3fe) +
		      (((base + SZ_32M) >> 14) & (0x3fe << 10)) +
		      (((base + 2 * SZ_32M) >> 4) & (0x3fe << 20));
		writel(val | en, ccci->dpmaif_pd + DPMAIF_PD_REMAP_BANK(i));
		base += 3 * SZ_32M;
	}

	writel(BIT(9) | BIT(10) | BIT(15) | BIT(16),
	       ccci->dpmaif_ao + DPMAIF_AO_UL_AP_L1TIMR0);
	writel(0xff, ccci->dpmaif_pd + DPMAIF_PD_DL_RESERVE_RW);

	ret = ccci_dpmaif_queues_init(ccci);
	if (ret)
		return ret;

	/* sync the defaults to the SRAM, UL and DL configured */
	writel(readl(ccci->dpmaif_pd + DPMAIF_PD_MISC_OVERWRITE_CFG) | BIT(0),
	       ccci->dpmaif_pd + DPMAIF_PD_MISC_OVERWRITE_CFG);
	ret = readl_poll_timeout(ccci->dpmaif_pd + DPMAIF_PD_MISC_OVERWRITE_CFG,
				 val, !(val & BIT(0)), 10, 100000);
	if (ret)
		return ret;
	writel(BIT(0), ccci->dpmaif_ao + DPMAIF_AO_UL_INIT_SET);
	writel(BIT(0), ccci->dpmaif_ao + DPMAIF_AO_DL_INIT_SET);

	val = readl(ccci->dpmaif_pd + DPMAIF_PD_AP_IP_BUSY);
	if (val)
		writel(val, ccci->dpmaif_pd + DPMAIF_PD_AP_IP_BUSY);

	return 0;
}

/* Data path: DPMAIF queues and the ccmni network interfaces */

struct ccmni {
	struct mtk_ccci *ccci;
	int channel;
};

static int dpmaif_add(void __iomem *reg, u32 val)
{
	u32 sta;
	int ret;

	ret = readl_poll_timeout_atomic(reg, sta, !(sta & DPMAIF_ADD_BUSY),
					0, 10000);
	if (ret)
		return ret;
	writel(DPMAIF_ADD_BUSY | val, reg);
	return 0;
}

/*
 * Give the buffer of a used BAT entry back to the hardware at the write
 * index. The buffers are fixed: the entry count is no multiple of the
 * buffer count, so remember which buffer each entry holds.
 */
static void dpmaif_bat_return(struct mtk_ccci *ccci, bool frg, u32 entry)
{
	u16 *buf_id = frg ? ccci->frg_buf_id : ccci->bat_buf_id;
	u32 *bat = frg ? ccci->dpmaif_frg.cpu : ccci->dpmaif_bat.cpu;
	u32 *wr = frg ? &ccci->frg_wr : &ccci->bat_wr;
	dma_addr_t a = (frg ? ccci->dpmaif_frg_buf.dma : ccci->dpmaif_bat_buf.dma) +
		       buf_id[entry] * DPMAIF_BAT_BUF_SIZE;

	bat[2 * *wr] = lower_32_bits(a);
	bat[2 * *wr + 1] = upper_32_bits(a) & 0xff;
	buf_id[*wr] = buf_id[entry];
	*wr = (*wr + 1) % DPMAIF_BAT_CNT;
}

static void dpmaif_rx_deliver(struct mtk_ccci *ccci)
{
	struct sk_buff *skb = ccci->rx_skb;
	struct net_device *ndev;

	ccci->rx_skb = NULL;
	if (ccci->rx_channel >= CCMNI_NUM || ccci->rx_drop) {
		dev_kfree_skb_any(skb);
		return;
	}
	ndev = ccci->ccmni[ccci->rx_channel];
	switch (skb->data[0] & 0xf0) {
	case 0x40:
		skb->protocol = htons(ETH_P_IP);
		break;
	case 0x60:
		skb->protocol = htons(ETH_P_IPV6);
		break;
	default:
		ndev->stats.rx_errors++;
		dev_kfree_skb_any(skb);
		return;
	}
	skb->dev = ndev;
	skb_reset_mac_header(skb);
	skb_reset_network_header(skb);
	ndev->stats.rx_packets++;
	ndev->stats.rx_bytes += skb->len;
	napi_gro_receive(&ccci->napi, skb);
}

/* One data PIT entry: copy its buffer into the packet being assembled */
static int dpmaif_rx_data(struct mtk_ccci *ccci, const u32 *pit)
{
	bool frg = pit[0] & PIT_FRAG;
	u32 len = FIELD_GET(PIT_LEN, pit[0]);
	u32 entry = FIELD_GET(PIT_BID, pit[0]) |
		    FIELD_GET(PIT_H_BID, pit[3]) << 13;
	dma_addr_t a = pit[1] | (dma_addr_t)(pit[2] & 0xff) << 32;
	typeof(ccci->dpmaif_bat_buf) *buf = frg ? &ccci->dpmaif_frg_buf :
						   &ccci->dpmaif_bat_buf;
	struct sk_buff *skb = ccci->rx_skb;

	if (entry >= DPMAIF_BAT_CNT || a < buf->dma ||
	    a - buf->dma + len > buf->size)
		return -EINVAL;

	if (!frg) {
		dev_kfree_skb_any(skb);
		skb = napi_alloc_skb(&ccci->napi, len);
		ccci->rx_skb = skb;
	} else if (skb && skb_tailroom(skb) < len &&
		   pskb_expand_head(skb, 0, len - skb_tailroom(skb), GFP_ATOMIC)) {
		dev_kfree_skb_any(skb);
		skb = ccci->rx_skb = NULL;
	}
	if (skb)
		skb_put_data(skb, buf->cpu + (a - buf->dma), len);

	dpmaif_bat_return(ccci, frg, entry);
	return 0;
}

static void dpmaif_tx_release(struct mtk_ccci *ccci)
{
	u32 rd = (readl(ccci->dpmaif_pd + DPMAIF_UL_CHNL_STA(0)) >> 16) / 2;
	bool released = false;
	int i;

	spin_lock(&ccci->tx_lock);
	while (ccci->drb_rel != rd && ccci->drb_rel != ccci->drb_wr) {
		typeof(*ccci->drb_skb) *e = &ccci->drb_skb[ccci->drb_rel];

		if (e->skb) {
			dma_unmap_single(ccci->dev, e->dma, e->len, DMA_TO_DEVICE);
			dev_consume_skb_any(e->skb);
			e->skb = NULL;
		}
		ccci->drb_rel = (ccci->drb_rel + 1) % DPMAIF_DRB_CNT;
		released = true;
	}
	spin_unlock(&ccci->tx_lock);

	if (released)
		for (i = 0; i < CCMNI_NUM; i++)
			if (netif_queue_stopped(ccci->ccmni[i]))
				netif_wake_queue(ccci->ccmni[i]);
}

static int dpmaif_poll(struct napi_struct *napi, int budget)
{
	struct mtk_ccci *ccci = container_of(napi, struct mtk_ccci, napi);
	void __iomem *pd = ccci->dpmaif_pd;
	u32 bat_wr = ccci->bat_wr, frg_wr = ccci->frg_wr;
	u32 wr, pits = 0;
	int done = 0;

	dpmaif_tx_release(ccci);

	wr = readl(pd + DPMAIF_DL_PIT_STA3) & DPMAIF_DL_PIT_WRIDX;
	while (ccci->pit_rd != wr && done < budget) {
		const u32 *pit = ccci->dpmaif_pit.cpu +
				 ccci->pit_rd * DPMAIF_PIT_SIZE;

		dma_rmb();
		if (FIELD_GET(PIT_SEQ, pit[3]) != ccci->pit_seq) {
			dev_err_ratelimited(ccci->dev, "PIT %u: sequence %lu, expected %u\n",
					    ccci->pit_rd, FIELD_GET(PIT_SEQ, pit[3]),
					    ccci->pit_seq);
			break;
		}
		ccci->pit_seq = ccci->pit_seq == PIT_SEQ_MAX ? 0 : ccci->pit_seq + 1;

		if (pit[0] & PIT_MSG) {
			ccci->rx_channel = FIELD_GET(PIT_MSG_CHANNEL, pit[0]);
			ccci->rx_drop = pit[0] & PIT_MSG_DROP;
		} else {
			if (dpmaif_rx_data(ccci, pit))
				dev_err_ratelimited(ccci->dev, "PIT %u: bad buffer %08x %08x %08x\n",
						    ccci->pit_rd, pit[0], pit[1], pit[2]);
			if (!(pit[0] & PIT_CONT) && ccci->rx_skb) {
				dpmaif_rx_deliver(ccci);
				done++;
			}
		}
		ccci->pit_rd = (ccci->pit_rd + 1) % DPMAIF_PIT_CNT;
		pits++;
	}

	/* hand the buffers and the PIT entries back */
	dma_wmb();
	if (ccci->bat_wr != bat_wr)
		dpmaif_add(pd + DPMAIF_PD_BAT_ADD,
			   (ccci->bat_wr - bat_wr) % DPMAIF_BAT_CNT);
	if (ccci->frg_wr != frg_wr)
		dpmaif_add(pd + DPMAIF_PD_BAT_ADD, DPMAIF_INIT_FRG |
			   (ccci->frg_wr - frg_wr) % DPMAIF_BAT_CNT);
	if (pits)
		dpmaif_add(pd + DPMAIF_PD_PIT_ADD, pits);

	if (done < budget && napi_complete_done(napi, done)) {
		writel(DPMAIF_DL_INT_QDONE,
		       ccci->dpmaif_ao + DPMAIF_AO_UL_APDL_L2TIMCR0);
		/* the irq handler cleared the status: catch what came since */
		if ((readl(pd + DPMAIF_DL_PIT_STA3) & DPMAIF_DL_PIT_WRIDX) !=
		    ccci->pit_rd)
			napi_schedule(napi);
	}
	return done;
}

static irqreturn_t dpmaif_irq(int irq, void *data)
{
	struct mtk_ccci *ccci = data;
	void __iomem *pd = ccci->dpmaif_pd;
	void __iomem *ao = ccci->dpmaif_ao;
	u32 ul, dl, busy;

	busy = readl(pd + DPMAIF_PD_AP_IP_BUSY);
	if (busy)
		writel(busy, pd + DPMAIF_PD_AP_IP_BUSY);

	ul = readl(pd + DPMAIF_PD_UL_L2TISAR0) &
	     ~readl(ao + DPMAIF_AO_UL_AP_L2TIMR0);
	if (ul)
		writel(ul, pd + DPMAIF_PD_UL_L2TISAR0);
	/*
	 * The modem side is asleep: the hardware waits for it, mask this
	 * until a queue completes again
	 */
	if (ul & DPMAIF_UL_INT_QDONE)
		writel(DPMAIF_UL_INT_MD_PWR_NOTRDY, ao + DPMAIF_AO_UL_AP_L2TIMCR0);
	else if (ul & DPMAIF_UL_INT_MD_PWR_NOTRDY)
		writel(ul & DPMAIF_UL_INT_MD_PWR_NOTRDY, ao + DPMAIF_AO_UL_AP_L2TIMSR0);
	ul &= ~DPMAIF_UL_INT_MD_PWR_NOTRDY;
	dl = readl(pd + DPMAIF_PD_APDL_L2TISAR0) &
	     ~readl(ao + DPMAIF_AO_UL_APDL_L2TIMR0);
	if (dl)
		writel(dl, pd + DPMAIF_PD_APDL_L2TISAR0);
	if ((ul & ~DPMAIF_UL_INT_QDONE) || (dl & ~DPMAIF_DL_INT_QDONE))
		dev_err_ratelimited(ccci->dev, "DPMAIF error interrupt: ul %#x dl %#x\n",
				    ul, dl);

	if (dl & DPMAIF_DL_INT_QDONE)
		writel(DPMAIF_DL_INT_QDONE, ao + DPMAIF_AO_UL_APDL_L2TIMSR0);
	if ((dl & DPMAIF_DL_INT_QDONE) || (ul & DPMAIF_UL_INT_QDONE))
		napi_schedule(&ccci->napi);

	return IRQ_HANDLED;
}

static netdev_tx_t ccmni_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct ccmni *priv = netdev_priv(ndev);
	struct mtk_ccci *ccci = priv->ccci;
	u32 *drb = ccci->dpmaif_drb[0].cpu;
	u32 w, len = skb->len;
	dma_addr_t a;

	if (!ccci->dpmaif_up || skb_linearize(skb))
		goto drop;

	spin_lock_bh(&ccci->tx_lock);
	if ((ccci->drb_rel - ccci->drb_wr - 1) % DPMAIF_DRB_CNT < 2) {
		netif_stop_queue(ndev);
		spin_unlock_bh(&ccci->tx_lock);
		return NETDEV_TX_BUSY;
	}
	a = dma_map_single(ccci->dev, skb->data, len, DMA_TO_DEVICE);
	if (dma_mapping_error(ccci->dev, a)) {
		spin_unlock_bh(&ccci->tx_lock);
		goto drop;
	}

	w = ccci->drb_wr;
	drb[2 * w] = DRB_MSG | DRB_CONT | FIELD_PREP(DRB_LEN, len);
	drb[2 * w + 1] = FIELD_PREP(DRB_MSG_CHANNEL, priv->channel);
	ccci->drb_skb[w].skb = NULL;
	w = (w + 1) % DPMAIF_DRB_CNT;
	drb[2 * w] = FIELD_PREP(DRB_ADDR_EXT, upper_32_bits(a)) |
		     FIELD_PREP(DRB_LEN, len);
	drb[2 * w + 1] = lower_32_bits(a);
	ccci->drb_skb[w].skb = skb;
	ccci->drb_skb[w].dma = a;
	ccci->drb_skb[w].len = len;
	ccci->drb_wr = (w + 1) % DPMAIF_DRB_CNT;

	dma_wmb();
	/* two entries of two words */
	if (dpmaif_add(ccci->dpmaif_pd + DPMAIF_PD_UL_ADD_DESC(0), 4))
		dev_err_ratelimited(ccci->dev, "DRB add timed out\n");
	spin_unlock_bh(&ccci->tx_lock);

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += len;
	return NETDEV_TX_OK;

drop:
	ndev->stats.tx_dropped++;
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

static const struct net_device_ops ccmni_ops = {
	.ndo_start_xmit = ccmni_xmit,
};

static void ccmni_setup(struct net_device *ndev)
{
	ndev->netdev_ops = &ccmni_ops;
	ndev->type = ARPHRD_RAWIP;
	ndev->flags = IFF_POINTOPOINT | IFF_NOARP;
	ndev->mtu = CCMNI_IF_MTU;
	ndev->min_mtu = ETH_MIN_MTU;
	ndev->max_mtu = CCMNI_IF_MTU;
	ndev->hard_header_len = 0;
	ndev->addr_len = 0;
	ndev->tx_queue_len = 1000;
}

/* After the DPMAIF hardware init: reset the software state, start RX/TX */
static void ccci_dpmaif_start(struct mtk_ccci *ccci)
{
	int i;

	for (i = 0; i < DPMAIF_BAT_FILL; i++)
		ccci->bat_buf_id[i] = ccci->frg_buf_id[i] = i;
	ccci->bat_wr = ccci->frg_wr = DPMAIF_BAT_FILL;
	ccci->pit_rd = 0;
	ccci->pit_seq = 0;
	ccci->drb_wr = ccci->drb_rel = 0;
	ccci->rx_channel = CCMNI_NUM;
	dev_info(ccci->dev, "DPMAIF up: arb %#x ul mask %#x dl mask %#x\n",
		 readl(ccci->dpmaif_ao + DPMAIF_AO_UL_CHNL_ARB0),
		 readl(ccci->dpmaif_ao + DPMAIF_AO_UL_AP_L2TIMR0),
		 readl(ccci->dpmaif_ao + DPMAIF_AO_UL_APDL_L2TIMR0));
	if (!ccci->dpmaif_up) {
		napi_enable(&ccci->napi);
		enable_irq(ccci->irq_dpmaif);
		ccci->dpmaif_up = true;
	}
}

static int ccci_net_init(struct mtk_ccci *ccci)
{
	int i, ret;

	ccci->bat_buf_id = devm_kcalloc(ccci->dev, DPMAIF_BAT_CNT,
					sizeof(u16), GFP_KERNEL);
	ccci->frg_buf_id = devm_kcalloc(ccci->dev, DPMAIF_BAT_CNT,
					sizeof(u16), GFP_KERNEL);
	ccci->drb_skb = devm_kcalloc(ccci->dev, DPMAIF_DRB_CNT,
				     sizeof(*ccci->drb_skb), GFP_KERNEL);
	if (!ccci->bat_buf_id || !ccci->frg_buf_id || !ccci->drb_skb)
		return -ENOMEM;
	spin_lock_init(&ccci->tx_lock);

	ccci->napi_dev = alloc_netdev_dummy(0);
	if (!ccci->napi_dev)
		return -ENOMEM;
	netif_napi_add(ccci->napi_dev, &ccci->napi, dpmaif_poll);

	for (i = 0; i < CCMNI_NUM; i++) {
		struct net_device *ndev;
		struct ccmni *priv;

		ndev = alloc_netdev(sizeof(*priv), "ccmni%d", NET_NAME_ENUM,
				    ccmni_setup);
		if (!ndev) {
			ret = -ENOMEM;
			goto err;
		}
		priv = netdev_priv(ndev);
		priv->ccci = ccci;
		priv->channel = i;
		SET_NETDEV_DEV(ndev, ccci->dev);
		ret = register_netdev(ndev);
		if (ret) {
			free_netdev(ndev);
			goto err;
		}
		ccci->ccmni[i] = ndev;
	}
	return 0;

err:
	while (--i >= 0) {
		unregister_netdev(ccci->ccmni[i]);
		free_netdev(ccci->ccmni[i]);
	}
	netif_napi_del(&ccci->napi);
	free_netdev(ccci->napi_dev);
	return ret;
}

static void ccci_net_exit(struct mtk_ccci *ccci)
{
	int i;

	for (i = 0; i < CCMNI_NUM; i++) {
		unregister_netdev(ccci->ccmni[i]);
		free_netdev(ccci->ccmni[i]);
	}
	if (ccci->dpmaif_up) {
		disable_irq(ccci->irq_dpmaif);
		napi_disable(&ccci->napi);
	}
	netif_napi_del(&ccci->napi);
	free_netdev(ccci->napi_dev);
	dev_kfree_skb_any(ccci->rx_skb);
	for (i = 0; i < DPMAIF_DRB_CNT; i++)
		if (ccci->drb_skb[i].skb) {
			dma_unmap_single(ccci->dev, ccci->drb_skb[i].dma,
					 ccci->drb_skb[i].len, DMA_TO_DEVICE);
			dev_kfree_skb_any(ccci->drb_skb[i].skb);
		}
}

static int ccci_md_power_off(struct mtk_ccci *ccci)
{
	struct arm_smccc_res res;
	u32 val;
	int i, ret;

	ccci_smc(MD_FLIGHT_MODE_SET, 1, &res);

	for (i = 0; i < ARRAY_SIZE(md_bus_prot); i++) {
		const struct ccci_bus_prot *bp = &md_bus_prot[i];
		void __iomem *base = ccci->bus_prot_base[bp->reg];

		writel(bp->mask, base + bp->set);
		ret = readl_poll_timeout(base + bp->sta, val,
					 (val & bp->mask) == bp->mask, 10, 100000);
		if (ret) {
			dev_err(ccci->dev, "bus protection %d failed\n", i);
			return ret;
		}
	}

	val = readl(ccci->spm + SPM_MD_PWR_CON);
	writel(val & ~PWR_ON, ccci->spm + SPM_MD_PWR_CON);
	ret = readl_poll_timeout(ccci->spm + SPM_MD_PWR_CON, val, !(val & PWR_ACK),
				 10, 100000);
	if (ret) {
		dev_err(ccci->dev, "MD power off timed out\n");
		return ret;
	}
	val = readl(ccci->spm + SPM_MD_EXT_BUCK_ISO);
	writel(val | MD_EXT_BUCK_ISO, ccci->spm + SPM_MD_EXT_BUCK_ISO);
	mdelay(4);

	val = readl(ccci->topckgen + TOPCKGEN_MD_CLK_MODE);
	writel(val | MD_CLK_MODE_OFF, ccci->topckgen + TOPCKGEN_MD_CLK_MODE);

	return 0;
}

static int ccci_md_power_on(struct mtk_ccci *ccci)
{
	struct arm_smccc_res res;
	u32 val;
	int i, ret;

	val = readl(ccci->topckgen + TOPCKGEN_MD_CLK_MODE);
	writel(val & ~MD_CLK_MODE_OFF, ccci->topckgen + TOPCKGEN_MD_CLK_MODE);

	ccci_smc(MD_FLIGHT_MODE_SET, 0, &res);

	val = readl(ccci->spm + SPM_MD_EXT_BUCK_ISO);
	writel(val & ~MD_EXT_BUCK_ISO, ccci->spm + SPM_MD_EXT_BUCK_ISO);
	val = readl(ccci->spm + SPM_MD_PWR_CON);
	writel(val | PWR_ON, ccci->spm + SPM_MD_PWR_CON);
	ret = readl_poll_timeout(ccci->spm + SPM_MD_PWR_CON, val, val & PWR_ACK,
				 10, 100000);
	if (ret) {
		dev_err(ccci->dev, "MD power on timed out\n");
		return ret;
	}

	for (i = ARRAY_SIZE(md_bus_prot) - 1; i >= 0; i--) {
		const struct ccci_bus_prot *bp = &md_bus_prot[i];
		void __iomem *base = ccci->bus_prot_base[bp->reg];

		writel(bp->mask, base + bp->clr);
		ret = readl_poll_timeout(base + bp->sta, val, !(val & bp->mask),
					 10, 100000);
		if (ret) {
			dev_err(ccci->dev, "bus protection %d release failed\n", i);
			return ret;
		}
	}

	return 0;
}

static void ccci_irqs(struct mtk_ccci *ccci, bool on)
{
	if (ccci->irqs_on == on)
		return;
	if (on) {
		enable_irq(ccci->irq);
		enable_irq(ccci->irq_exp);
	} else {
		disable_irq(ccci->irq);
		disable_irq(ccci->irq_exp);
	}
	ccci->irqs_on = on;
}

static int ccci_start(struct mtk_ccci *ccci)
{
	struct arm_smccc_res res;
	unsigned long rom;
	int i, ret;

	/* one start at a time; a running modem gets power cycled */
	if (!mutex_trylock(&ccci->lock))
		return -EBUSY;
	ccci_irqs(ccci, false);
	flush_work(&ccci->rx_work);

	if (!ccci->dvfsrc_started) {
		arm_smccc_smc(MTK_SIP_DVFSRC_CONTROL, DVFSRC_INIT, 0, 0, 0, 0, 0,
			      0, &res);
		dev_info(ccci->dev, "DVFSRC init: %lx\n", res.a0);
		arm_smccc_smc(MTK_SIP_DVFSRC_CONTROL, DVFSRC_START, 0, 0, 0, 0, 0,
			      0, &res);
		dev_info(ccci->dev, "DVFSRC start: %lx\n", res.a0);
		ccci->dvfsrc_started = true;
	}

	ccci_smc(CCIF_CLK_REQUEST, 0, &res);
	ccci_clear_smem(ccci);
	ccci_ccif_reset(ccci);
	ccci_set_state(ccci, MD_STATE_WAIT_HS1);
	ccci_irqs(ccci, true);

	/* the bootloader has powered the modem; wait for its boot ROM */
	for (i = 0; i < 100; i++) {
		rom = ccci_smc(MD_POWER_CONFIG, MD_CHECK_DONE, &res);
		if (!rom)
			break;
		msleep(20);
	}
	ccci_smc(MD_POWER_CONFIG, MD_CHECK_FLAG, &res);
	dev_info(ccci->dev, "boot ROM %s, flags %lx %lx %lx %lx\n",
		 rom ? "failed" : "passed", res.a0, res.a1, res.a2, res.a3);
	ccci_smc(MD_POWER_CONFIG, MD_BOOT_STATUS, &res);
	dev_info(ccci->dev, "boot status %lx %lx %lx\n", res.a0, res.a1, res.a2);

	/* power cycle it as the downstream driver does on the first start */
	if (ccci_md_power_off(ccci) || ccci_md_power_on(ccci)) {
		ccci_set_state(ccci, MD_STATE_EXCEPTION);
		ret = -EIO;
		goto out;
	}
	ccci_smc(MD_POWER_CONFIG, MD_KERNEL_BOOT_UP, &res);
	dev_info(ccci->dev, "released the modem: %lx %lx %lx %lx\n",
		 res.a0, res.a1, res.a2, res.a3);

	/* the data path, set up as downstream does once the modem runs */
	if (ccci_dpmaif_init(ccci))
		dev_err(ccci->dev, "DPMAIF init failed\n");
	else
		ccci_dpmaif_start(ccci);

	if (!wait_event_timeout(ccci->state_wq, ccci->state != MD_STATE_WAIT_HS1,
				30 * HZ)) {
		dev_err(ccci->dev, "no handshake from the modem\n");
		ccci_smc(MD_POWER_CONFIG, MD_BOOT_STATUS, &res);
		dev_err(ccci->dev, "boot status %lx %lx %lx\n", res.a0, res.a1,
			res.a2);
		ccci_irqs(ccci, false);
		ccci_set_state(ccci, MD_STATE_OFF);
		ret = -ETIMEDOUT;
		goto out;
	}
	/* the modem loads its NVRAM through the FS channel before HS2 */
	wait_event_timeout(ccci->state_wq, ccci->state != MD_STATE_WAIT_HS2,
			   120 * HZ);
	ret = ccci->state == MD_STATE_READY ? 0 : -EIO;
out:
	ccci_smc(CCIF_CLK_RELEASE, 0, &res);
	mutex_unlock(&ccci->lock);
	return ret;
}

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct mtk_ccci *ccci = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s\n", md_state_names[ccci->state]);
}

static ssize_t state_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct mtk_ccci *ccci = dev_get_drvdata(dev);
	int ret;

	if (sysfs_streq(buf, "assert")) {
		ccci_ccif_kick(ccci, CCIF_H2D_FORCE_ASSERT);
		return count;
	}
	if (!sysfs_streq(buf, "start"))
		return -EINVAL;
	ret = ccci_start(ccci);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(state);

static ssize_t smem_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct mtk_ccci *ccci = dev_get_drvdata(dev);
	int i, n = 0;

	for (i = 0; i < SMEM_MAX; i++)
		if (ccci->smem[i].valid)
			n += sysfs_emit_at(buf, n, "%2d %c md %#010x size %#x\n", i,
					   ccci->smem[i].cached ? 'c' : 'n',
					   ccci->smem[i].md_addr, ccci->smem[i].size);
	return n;
}
static DEVICE_ATTR_RO(smem);

/*
 * Dump MD debug registers to the kernel log, like the downstream
 * internal_md_dump_debug_register(): enable the debug bus timeout and freeze
 * the PC monitor while reading. Takes "<addr> <len>" inside MD_DBG_BASE,
 * the default is the PC monitor.
 */
#define MD_DBG_BASE		0x0d100000
#define MD_DBG_SIZE		0x60000
#define MD_PCMON_BASE		0x0d11c000
#define MD_PCMON_SIZE		0x21b0

static ssize_t pcmon_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	unsigned long addr = MD_PCMON_BASE, size = MD_PCMON_SIZE;
	struct arm_smccc_res res;
	void __iomem *base;
	u32 line[8];
	int off, i;

	if (sscanf(buf, "%lx %lx", &addr, &size) == 1)
		return -EINVAL;
	addr &= ~3UL;
	if (addr < MD_DBG_BASE || !size ||
	    addr + size > MD_DBG_BASE + MD_DBG_SIZE)
		return -EINVAL;
	base = ioremap(addr, size);
	if (!base)
		return -ENOMEM;
	arm_smccc_smc(MTK_SIP_CCCI_CONTROL, 0, 0, 0x8001e848, 0, 0, 0, 0, &res);
	dev_info(dev, "pcmon: dbg timeout smc %ld\n", (long)res.a0);
	arm_smccc_smc(MTK_SIP_CCCI_CONTROL, 0, 1, 0x2222, 0, 0, 0, 0, &res);
	dev_info(dev, "pcmon: freeze smc %ld\n", (long)res.a0);
	for (off = 0; off < size; off += sizeof(line)) {
		bool zero = true;

		for (i = 0; i < ARRAY_SIZE(line); i++) {
			line[i] = off + i * 4 < size ?
				  readl(base + off + i * 4) : 0;
			zero &= !line[i];
		}
		if (!zero)
			dev_info(dev, "pcmon %#010lx: %*ph\n", addr + off,
				 (int)sizeof(line), line);
	}
	arm_smccc_smc(MTK_SIP_CCCI_CONTROL, 0, 1, 0x1111, 0, 0, 0, 0, &res);
	iounmap(base);
	return count;
}
static DEVICE_ATTR_WO(pcmon);

static struct attribute *ccci_attrs[] = {
	&dev_attr_state.attr,
	&dev_attr_smem.attr,
	&dev_attr_pcmon.attr,
	NULL
};
ATTRIBUTE_GROUPS(ccci);

static void *ccci_map_region(struct mtk_ccci *ccci, int idx, phys_addr_t *phys,
			     size_t *size, bool io)
{
	struct reserved_mem *rmem;
	struct device_node *np;

	np = of_parse_phandle(ccci->dev->of_node, "memory-region", idx);
	if (!np)
		return ERR_PTR(-ENODEV);
	rmem = of_reserved_mem_lookup(np);
	of_node_put(np);
	if (!rmem)
		return ERR_PTR(-ENODEV);
	*phys = rmem->base;
	*size = rmem->size;
	if (io)
		return devm_ioremap_wc(ccci->dev, rmem->base, rmem->size);
	return devm_memremap(ccci->dev, rmem->base, rmem->size, MEMREMAP_WB);
}

/* Registers owned by the clock and power domain drivers */
static void __iomem *ccci_ioremap_shared(struct platform_device *pdev,
					 const char *name)
{
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!res)
		return NULL;
	return devm_ioremap(&pdev->dev, res->start, resource_size(res));
}

static int mtk_ccci_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_ccci *ccci;
	phys_addr_t phys;
	int i, ret;

	ccci = devm_kzalloc(dev, sizeof(*ccci), GFP_KERNEL);
	if (!ccci)
		return -ENOMEM;
	ccci->dev = dev;
	mutex_init(&ccci->lock);
	init_waitqueue_head(&ccci->state_wq);
	INIT_WORK(&ccci->rx_work, ccci_rx_work);
	platform_set_drvdata(pdev, ccci);

	ccci->ap_ccif = devm_platform_ioremap_resource_byname(pdev, "ap-ccif");
	if (IS_ERR(ccci->ap_ccif))
		return PTR_ERR(ccci->ap_ccif);
	ccci->md_ccif = devm_platform_ioremap_resource_byname(pdev, "md-ccif");
	if (IS_ERR(ccci->md_ccif))
		return PTR_ERR(ccci->md_ccif);
	ccci->infracfg = devm_platform_ioremap_resource_byname(pdev, "infracfg");
	if (IS_ERR(ccci->infracfg))
		return PTR_ERR(ccci->infracfg);

	ccci->spm = ccci_ioremap_shared(pdev, "spm");
	ccci->topckgen = ccci_ioremap_shared(pdev, "topckgen");
	ccci->bus_prot_base[0] = ccci_ioremap_shared(pdev, "infracfg-ao");
	ccci->bus_prot_base[1] = ccci_ioremap_shared(pdev, "nemicfg");
	ccci->dpmaif_ao = ccci_ioremap_shared(pdev, "dpmaif-ao");
	ccci->dpmaif_pd = ccci_ioremap_shared(pdev, "dpmaif-pd");
	if (!ccci->spm || !ccci->topckgen || !ccci->bus_prot_base[0] ||
	    !ccci->bus_prot_base[1] || !ccci->dpmaif_ao || !ccci->dpmaif_pd)
		return dev_err_probe(dev, -ENOMEM, "failed to map the power registers\n");

	ccci->tag_mem = ccci_map_region(ccci, 0, &phys, &ccci->tag_size, false);
	if (IS_ERR_OR_NULL(ccci->tag_mem))
		return dev_err_probe(dev, -ENOMEM, "no tag memory\n");
	ccci->nc_smem = ccci_map_region(ccci, 1, &ccci->nc_smem_phys,
					&ccci->nc_smem_size, true);
	if (IS_ERR_OR_NULL(ccci->nc_smem))
		return dev_err_probe(dev, -ENOMEM, "no shared memory\n");
	ccci->c_smem = ccci_map_region(ccci, 2, &ccci->c_smem_phys,
				       &ccci->c_smem_size, true);
	if (IS_ERR_OR_NULL(ccci->c_smem))
		return dev_err_probe(dev, -ENOMEM, "no cached shared memory\n");

	ret = ccci_parse_tags(ccci);
	if (ret)
		return ret;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(40));
	if (ret)
		return ret;
	ret = ccci_dpmaif_alloc(ccci);
	if (ret)
		return dev_err_probe(dev, ret, "no memory for the data path\n");

	ret = ccci_ring_init(ccci, ccci->ring, SMEM_CCISM_MCU, rx_queue_size,
			     tx_queue_size, &ccci->ccism_size);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set up the rings\n");
	ret = ccci_ring_init(ccci, ccci->exp_ring, SMEM_CCISM_MCU_EXP,
			     exp_queue_size, exp_queue_size, &ccci->ccism_exp_size);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set up the exception rings\n");

	ccci->irq = platform_get_irq(pdev, 0);
	if (ccci->irq < 0)
		return ccci->irq;
	ccci->irq_exp = platform_get_irq(pdev, 1);
	if (ccci->irq_exp < 0)
		return ccci->irq_exp;
	ret = devm_request_irq(dev, ccci->irq, ccci_ccif_irq, IRQF_NO_AUTOEN,
			       "ccif", ccci);
	if (ret)
		return ret;
	ret = devm_request_irq(dev, ccci->irq_exp, ccci_ccif_exp_irq,
			       IRQF_NO_AUTOEN, "ccif-exp", ccci);
	if (ret)
		return ret;
	ccci->irq_dpmaif = platform_get_irq(pdev, 2);
	if (ccci->irq_dpmaif < 0)
		return ccci->irq_dpmaif;
	ret = devm_request_irq(dev, ccci->irq_dpmaif, dpmaif_irq,
			       IRQF_NO_AUTOEN, "dpmaif", ccci);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(ccci->ports); i++) {
		struct ccci_port *port = &ccci->ports[i];

		*port = ccci_port_cfg[i];
		port->ccci = ccci;
		skb_queue_head_init(&port->rx_list);
		init_waitqueue_head(&port->rx_wq);
		mutex_init(&port->tx_mutex);
		port->misc.minor = MISC_DYNAMIC_MINOR;
		port->misc.name = port->name;
		port->misc.fops = &ccci_port_fops;
		port->misc.parent = dev;
		ret = misc_register(&port->misc);
		if (ret) {
			while (--i >= 0)
				misc_deregister(&ccci->ports[i].misc);
			return ret;
		}
	}

	ret = ccci_net_init(ccci);
	if (ret) {
		for (i = 0; i < ARRAY_SIZE(ccci->ports); i++)
			misc_deregister(&ccci->ports[i].misc);
		return dev_err_probe(dev, ret, "failed to set up the network interfaces\n");
	}

	return 0;
}

static void mtk_ccci_remove(struct platform_device *pdev)
{
	struct mtk_ccci *ccci = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < ARRAY_SIZE(ccci->ports); i++) {
		misc_deregister(&ccci->ports[i].misc);
		skb_queue_purge(&ccci->ports[i].rx_list);
	}
	cancel_work_sync(&ccci->rx_work);
	ccci_net_exit(ccci);
}

static const struct of_device_id mtk_ccci_of_match[] = {
	{ .compatible = "mediatek,mt6878-ccci" },
	{ }
};
MODULE_DEVICE_TABLE(of, mtk_ccci_of_match);

static struct platform_driver mtk_ccci_driver = {
	.probe = mtk_ccci_probe,
	.remove = mtk_ccci_remove,
	.driver = {
		.name = "mtk-ccci",
		.of_match_table = mtk_ccci_of_match,
		.dev_groups = ccci_groups,
	},
};
module_platform_driver(mtk_ccci_driver);

MODULE_DESCRIPTION("MediaTek MT6878 modem control and communication");
MODULE_LICENSE("GPL");
