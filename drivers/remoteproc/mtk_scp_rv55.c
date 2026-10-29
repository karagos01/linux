// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6878 SCP (Andes RV55 "tinysys") bring-up.
 *
 * The SCP is a RISC-V companion processor whose firmware is an MTK tinysys
 * container (not ELF), staged into reserved DRAM by the bootloader, which also
 * writes a region-info descriptor at SRAM+0x4. Unlike the Cortex-M SCPs handled
 * by mtk_scp.c, the reset/boot registers here are secure-only: the core is
 * released through the resident TF-A via an SiP SMC.
 *
 * This driver boots the RV55 (reset-release via the SMC above), speaks the
 * tinysys sensor-hub protocol over the hardware mailbox (CTRL/NOTIFY packets
 * with a CRC-8 header), hands the SCP the shared-DRAM ring layout and a host
 * timebase, enumerates the sensor list, and exposes the physical sensors
 * (accelerometer, gyroscope, magnetometer, ambient-light, proximity, SAR) plus
 * the SCP-fused virtual sensors (gravity, linear acceleration, orientation, the
 * rotation-vector quaternions, the step counter, the uncalibrated accel/gyro/
 * magnetometer, the device-orientation quadrant, and the significant-motion,
 * tilt and wake-gesture event detectors) as standard
 * IIO devices. They share one SCP DATA ring; the drain routes each
 * record to its sensor by type. Sensors are enabled on demand (when an IIO
 * reader or capture buffer needs them) and disabled again when idle; with no
 * active sensor the SCP sleeps, and the enable path wakes it first through the
 * infracfg_ao awake handshake, since a sleeping SCP otherwise stops acking
 * mailbox traffic. The latest sample is available through the sysfs *_raw
 * attributes and the full timestamped stream through each device's kfifo capture
 * buffer (/dev/iio:deviceN). The generic rpmsg layer is not wired up yet.
 */
#include <linux/arm-smccc.h>
#include <linux/atomic.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/rtc.h>
#include <linux/sched/clock.h>
#include <linux/spinlock.h>
#include <linux/time.h>
#include <linux/timekeeping.h>
#include <linux/workqueue.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/kfifo_buf.h>
#include <asm/arch_timer.h>

/* region-info descriptor the bootloader leaves at SCP SRAM + 0x4 */
#define SCP_REGION_INFO_OFFSET		0x4

/* SiP service in the resident TF-A: MTK_SIP_TINYSYS_SCP_CONTROL (fn 0x528) */
#define MTK_SIP_TINYSYS_SCP_CONTROL	0xC2000528UL
#define SCP_KERNEL_OP_RESET_RELEASE	3

/* mailbox window: set-irq (AP->SCP trigger) and clr-irq (SCP->AP status/ack) */
#define SCP_MBOX_SET_IRQ		0x100
#define SCP_MBOX_CLR_IRQ		0x10c

#define SCP_NR_MBOX			5

/*
 * Pins are laid out within a mailbox by the firmware's send-then-recv order;
 * for this SoC every pin's slot byte offset works out to pin_index * 8. All
 * SCP<->sensorhub traffic uses mailbox 1. Values verified on-device (the ready
 * + logger recv bits latch as 0x02400000).
 */
#define SCP_SENSOR_MBOX			1
#define SCP_PIN_OFF(bit)		((bit) * 8)

/* SCP->AP recv pins on mailbox 1 */
#define SCP_READY_BIT			25	/* IPI_IN_SCP_READY_1, payload = image size */
#define SCP_SENSOR_NOTIFY_BIT		27	/* IPI_IN_SENSOR_NOTIFY */

#define SCP_SENSOR_CTRL_ACK_BIT		26	/* IPI_IN_SENSOR_CTRL (ack) */

/* AP->SCP send pins on mailbox 1 */
#define SCP_SENSOR_CTRL_OUT_BIT		5	/* IPI_OUT_SENSOR_CTRL */
#define SCP_SENSOR_NOTIFY_OUT_BIT	13	/* IPI_OUT_SENSOR_NOTIFY */

/* sensor_comm notify commands */
#define SCP_SENSOR_NOTIFY_DATA		0
#define SCP_SENSOR_NOTIFY_READY		4
#define SCP_SENSOR_NOTIFY_LIST		5

/* sensor_comm ctrl commands (enum sensor_comm_ctrl_cmd) */
#define SCP_SENSOR_CTRL_DISABLE		0
#define SCP_SENSOR_CTRL_ENABLE		1
#define SCP_SENSOR_CTRL_FLUSH		2
#define SCP_SENSOR_CTRL_CONFIG		4	/* set-delay/batch */
#define SCP_SENSOR_CTRL_TIMESYNC	10
#define SCP_SENSOR_CTRL_SHARE_MEMORY	11

/* DATA-ring record action: 0 = real sample, nonzero = meta (flush-complete...) */
#define SCP_SENSOR_ACTION_DATA		0

/* Android/MTK sensor type IDs (hf_sensor_type.h) */
#define SCP_SENSOR_TYPE_ACCEL		1
#define SCP_SENSOR_TYPE_MAGN		2
#define SCP_SENSOR_TYPE_ORIENTATION	3	/* fusion: azimuth/pitch/roll */
#define SCP_SENSOR_TYPE_GYRO		4
#define SCP_SENSOR_TYPE_UNCAL_MAGN	14	/* fusion: magnetometer + hard-iron bias */
#define SCP_SENSOR_TYPE_UNCAL_GYRO	16	/* fusion: gyroscope + drift bias */
#define SCP_SENSOR_TYPE_UNCAL_ACCEL	35	/* fusion: accelerometer + bias */
#define SCP_SENSOR_TYPE_ALS		5
#define SCP_SENSOR_TYPE_PS		8
#define SCP_SENSOR_TYPE_GRAVITY		9	/* fusion: gravity vector */
#define SCP_SENSOR_TYPE_LINEAR_ACCEL	10	/* fusion: linear acceleration */
#define SCP_SENSOR_TYPE_ROTATION	11	/* fusion: rotation vector (quaternion) */
#define SCP_SENSOR_TYPE_GAME_ROTATION	15	/* fusion: game rotation (no magnetometer) */
#define SCP_SENSOR_TYPE_SIGNIFICANT	17	/* fusion: significant-motion event */
#define SCP_SENSOR_TYPE_STEP_COUNT	19	/* fusion: cumulative step count */
#define SCP_SENSOR_TYPE_GEOMAG_ROTATION	20	/* fusion: geomagnetic rotation (no gyro) */
#define SCP_SENSOR_TYPE_TILT		22	/* fusion: tilt-detector event */
#define SCP_SENSOR_TYPE_WAKE		23	/* fusion: wake-gesture event */
#define SCP_SENSOR_TYPE_DEV_ORIENT	27	/* fusion: device-orientation quadrant */
#define SCP_SENSOR_TYPE_SAR		72

/*
 * SCP "awake" doorbell: two registers in the infracfg_ao syscon, reached
 * through the mediatek,infracfg phandle. To talk to the SCP while it may be in
 * deep idle, the AP writes an awake opcode to INFRA_IRQ_SET, polls that register
 * until the opcode bit self-clears (the SCP has woken and acknowledged), then
 * writes INFRA_IRQ_CLEAR. This mirrors the vendor scp_awake_lock()/unlock()
 * (scp_awake.c). Offsets are within infracfg_ao.
 */
#define SCP_AWAKE_IRQ_SET		0xb14		/* INFRA_IRQ_SET */
#define SCP_AWAKE_IRQ_CLEAR		0xb18		/* INFRA_IRQ_CLEAR */
#define SCP_AWAKE_TAG			0xa0
#define SCP_AWAKE_LOCK			0		/* opcode: hold awake */
#define SCP_AWAKE_UNLOCK		1		/* opcode: release */
#define SCP_AWAKE_POLL_US		2000		/* ~2 ms handshake budget */

/*
 * Steady sampling rate at which each physical sensor is enabled once the sensor
 * hub is up; the latest DATA-ring sample per sensor is cached for sysfs reads and
 * the whole stream is pushed to that sensor's capture buffer. The per-sensor gain
 * (physical = raw / gain) is read back from the sensor list, with a fallback.
 * Keeping these sensors streaming also holds the SCP out of the deep idle state
 * in which it stops acking mailbox traffic.
 */
#define SCP_SENSOR_PERIOD_NS		100000000LL	/* 10 Hz */
#define SCP_FUSION_SETTLE_MS		500	/* fused-filter convergence before a one-shot read */

/* share-memory payload types */
#define SCP_SHM_PAYLOAD_DATA		0
#define SCP_SHM_PAYLOAD_LIST		2

/*
 * Sensor ring buffers inside the SCP shared-DRAM window (DRAM_RESV base
 * 0x8e000000). These sub-block addresses reproduce the firmware's fixed
 * reserved-memory layout (secure-dump 0xe28000, then VOW, then these), so
 * they do not collide with regions the SCP uses on its own.
 */
#define SCP_SHM_LIST_BASE		0x8f1a6700
#define SCP_SHM_LIST_SIZE		0x1000
#define SCP_SHM_DATA_BASE		0x8eef2700
#define SCP_SHM_DATA_SIZE		0x100000
/*
 * Ring element sizes differ per payload type and the SCP firmware asserts
 * (share_memory.c) that each region's header item_size matches what it
 * expects: sizeof(struct share_mem_data)=36 for DATA, sizeof(struct
 * share_mem_info)=40 for LIST.
 */
#define SCP_SHM_DATA_ITEM		36	/* share_mem_data */
#define SCP_SHM_LIST_ITEM		40	/* share_mem_info */
#define SCP_SHM_HDR_SIZE		16	/* rp, wp, buffer_size, item_size */

#define SCP_READY_TIMEOUT_MS		10000
#define SCP_REPLY_TIMEOUT_MS		200

/* sensor_comm ctrl header: 5 CRC'd bytes, payload (if any) follows at offset 8 */
struct scp_sensor_ctrl {
	u8 sequence;
	u8 sensor_type;
	u8 command;
	u8 length;
	u8 crc8;
	u8 pad[3];
} __packed;

/* sensor_comm ack (reply to a ctrl) */
struct scp_sensor_ack {
	u8 sequence;
	u8 sensor_type;
	u8 command;
	s8 ret_val;
	u8 crc8;
	u8 pad[3];
} __packed;

/*
 * SHARE_MEMORY ctrl payload: up to four {type, phys-base} ring descriptors.
 * The SCP firmware reads the base_info entries packed immediately after the
 * available_num byte (base_info[0] at offset 1, not 4), with each entry's
 * payload_base at an *unaligned* offset (5, 13, ...) — the firmware loads it
 * byte-wise. A padding byte after available_num shifts every field and makes
 * the firmware read payload_base as 0, tripping an assert in its
 * share_memory.c. So the whole payload must be truly packed.
 */
struct scp_share_mem {
	u8 available_num;
	struct {
		u8 payload_type;
		u8 pad[3];
		u32 payload_base;
	} __packed base_info[4];
	u8 tail[3];	/* pad to 36 bytes: firmware asserts ctrl->length == 36 */
} __packed;

/* sensor_comm timesync payload (ctrl cmd TIMESYNC) */
struct scp_timesync {
	s64 host_timestamp;
	s64 host_archcounter;
	s64 sched_clock;
	s32 usecond;
	s32 second;
	s32 minute;
	s32 hour;
	s32 day;
	s32 month;
} __packed;

/* ENABLE batch payload (sensor_comm_batch): sample period and max report latency */
struct scp_sensor_batch {
	s64 delay;
	s64 latency;
} __packed;

/* one DATA-ring record (share_mem_data), 36 bytes */
struct scp_sensor_data {
	u8 sensor_type;
	u8 action;
	u8 accuracy;
	u8 pad;
	s64 timestamp;
	s32 value[6];
} __packed;

/* ring header at the start of each share-memory region (share_mem_base) */
struct scp_ring_hdr {
	u32 rp;
	u32 wp;
	u32 buffer_size;
	u32 item_size;
} __packed;

/* one sensor-list record (share_mem_info) */
struct scp_sensor_info {
	u8 sensor_type;
	u8 pad[3];
	u32 gain;
	u8 name[16];
	u8 vendor[16];
} __packed;

/*
 * sensor_comm notify packet (28 bytes): a small header plus five data words.
 * The 5-byte header is covered by a CRC-8 the SCP checks, so the layout and
 * checksum must match the firmware exactly.
 */
struct scp_sensor_notify {
	u8 sequence;
	u8 sensor_type;
	u8 command;
	u8 length;
	u8 crc8;
	u8 pad[3];
	s32 value[5];
} __packed;

/* CRC-8, polynomial 0x8c (reflected), init 0 — the SCP's tiny_crc8 */
static u8 scp_crc8(const u8 *p, size_t len)
{
	static const u8 tbl[32] = {
		0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
		0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
		0x00, 0x9d, 0x23, 0xbe, 0x46, 0xdb, 0x65, 0xf8,
		0x8c, 0x11, 0xaf, 0x32, 0xca, 0x57, 0xe9, 0x74,
	};
	u8 crc = 0;

	while (len--) {
		crc ^= *p++;
		crc = tbl[crc & 0x0f] ^ tbl[16 + ((crc >> 4) & 0x0f)];
	}
	return crc;
}

struct scp_region_info {
	u32 ap_loader_start;
	u32 ap_loader_size;
	u32 ap_firmware_start;
	u32 ap_firmware_size;
	u32 ap_dram_start;
	u32 ap_dram_size;
	u32 ap_dram_backup_start;
	u32 struct_size;
};

struct mt6878_scp;
struct scp_sensor;

struct scp_mbox {
	struct mt6878_scp *scp;
	void __iomem *base;
	int id;
};

struct mt6878_scp {
	struct device *dev;
	void __iomem *sram;
	struct scp_mbox mbox[SCP_NR_MBOX];
	atomic_t ready;
	atomic_t sensor_ready;
	u32 image_size;
	u8 seq;

	/* sensor hub bring-up */
	struct work_struct sensor_init;
	struct work_struct sensorhub;
	struct mutex ctrl_lock;		/* serializes CTRL commands + awake */
	struct completion ctrl_ack;
	struct completion notify_reply;
	s8 ack_ret;
	u32 notify_wp;
	void __iomem *list_ring;
	void __iomem *data_ring;
	struct regmap *infracfg;	/* infracfg_ao: SCP awake doorbell */

	/* live DATA-ring drain */
	struct work_struct drain;
	u32 data_wp;

	/* IIO devices, one per physical sensor (indexed by scp_sensor_table) */
	struct scp_sensor *sensors;

	struct delayed_work report;
};

/* static description of one physical sensor exposed as an IIO device */
struct scp_sensor_desc {
	u8 type;				/* SCP sensor_type */
	const char *name;			/* IIO device name */
	const struct iio_chan_spec *channels;
	int num_channels;
	int nvals;				/* data channels (1 or 3) */
	u32 default_gain;			/* fallback until the list is read */
	u32 unit_div;				/* extra scale divisor for IIO units */
	s64 period_ns;
	bool on_change;				/* flush on enable to prime a value */
	bool fusion;				/* SCP-fused virtual sensor */
	bool event;				/* delivers IIO events, not samples */
};

/* per-sensor runtime state; iio_priv() holds a pointer to one of these */
struct scp_sensor {
	struct mt6878_scp *scp;
	struct iio_dev *indio;
	const struct scp_sensor_desc *desc;
	spinlock_t lock;
	s32 val[6];			/* up to 6 (uncal: x,y,z + bias x,y,z) */
	u8 accuracy;
	s64 ts;
	u32 gain;
	bool valid;
	struct mutex en_lock;		/* guards users/enable transitions */
	unsigned int users;		/* on-demand enable refcount */
	bool ev_on;			/* event delivery armed (event sensors) */
};

static struct scp_sensor *iio_to_sensor(struct iio_dev *indio)
{
	return *(struct scp_sensor **)iio_priv(indio);
}

static int scp_sensor_get(struct scp_sensor *s);
static void scp_sensor_put(struct scp_sensor *s);

/* A modified X/Y/Z tri(IIO_ACCEL/ANGL_VEL/MAGN) or a single-value channel. */
#define SCP_AXIS_CHAN(_type, _mod, _idx) {			\
	.type = _type,						\
	.modified = 1,						\
	.channel2 = IIO_MOD_##_mod,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |	\
				    BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _idx,					\
	.scan_type = { .sign = 's', .realbits = 32,		\
		       .storagebits = 32, .endianness = IIO_CPU }, \
}
#define SCP_1VAL_CHAN(_type) {					\
	.type = _type,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |	\
				    BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = 0,					\
	.scan_type = { .sign = 's', .realbits = 32,		\
		       .storagebits = 32, .endianness = IIO_CPU }, \
}

#define SCP_3AXIS_CHANNELS(_name, _type)			\
static const struct iio_chan_spec _name[] = {			\
	SCP_AXIS_CHAN(_type, X, 0),				\
	SCP_AXIS_CHAN(_type, Y, 1),				\
	SCP_AXIS_CHAN(_type, Z, 2),				\
	IIO_CHAN_SOFT_TIMESTAMP(3),				\
}
/*
 * Uncalibrated tri-axis channel: besides the raw reading it carries the SCP's
 * live bias estimate (ring value[3..5]) through the standard calibbias attr.
 */
#define SCP_UNCAL_AXIS_CHAN(_type, _mod, _idx) {		\
	.type = _type,						\
	.modified = 1,						\
	.channel2 = IIO_MOD_##_mod,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			      BIT(IIO_CHAN_INFO_CALIBBIAS),	\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |	\
				    BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _idx,					\
	.scan_type = { .sign = 's', .realbits = 32,		\
		       .storagebits = 32, .endianness = IIO_CPU }, \
}
#define SCP_UNCAL_3AXIS_CHANNELS(_name, _type)			\
static const struct iio_chan_spec _name[] = {			\
	SCP_UNCAL_AXIS_CHAN(_type, X, 0),			\
	SCP_UNCAL_AXIS_CHAN(_type, Y, 1),			\
	SCP_UNCAL_AXIS_CHAN(_type, Z, 2),			\
	IIO_CHAN_SOFT_TIMESTAMP(3),				\
}
#define SCP_1VAL_CHANNELS(_name, _type)				\
static const struct iio_chan_spec _name[] = {			\
	SCP_1VAL_CHAN(_type),					\
	IIO_CHAN_SOFT_TIMESTAMP(1),				\
}

SCP_3AXIS_CHANNELS(scp_accel_channels, IIO_ACCEL);
SCP_3AXIS_CHANNELS(scp_gyro_channels, IIO_ANGL_VEL);
SCP_3AXIS_CHANNELS(scp_magn_channels, IIO_MAGN);
SCP_1VAL_CHANNELS(scp_light_channels, IIO_LIGHT);
SCP_1VAL_CHANNELS(scp_prox_channels, IIO_PROXIMITY);
SCP_1VAL_CHANNELS(scp_sar_channels, IIO_PROXIMITY);
SCP_3AXIS_CHANNELS(scp_gravity_channels, IIO_GRAVITY);
SCP_3AXIS_CHANNELS(scp_linaccel_channels, IIO_ACCEL);
SCP_UNCAL_3AXIS_CHANNELS(scp_uncal_accel_channels, IIO_ACCEL);
SCP_UNCAL_3AXIS_CHANNELS(scp_uncal_gyro_channels, IIO_ANGL_VEL);
SCP_UNCAL_3AXIS_CHANNELS(scp_uncal_magn_channels, IIO_MAGN);

/* orientation = Euler angles (azimuth->yaw, pitch, roll), in degrees */
static const struct iio_chan_spec scp_orient_channels[] = {
	SCP_AXIS_CHAN(IIO_ROT, YAW, 0),
	SCP_AXIS_CHAN(IIO_ROT, PITCH, 1),
	SCP_AXIS_CHAN(IIO_ROT, ROLL, 2),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

/*
 * device orientation = the discrete screen-rotation quadrant (0..3), as a single
 * raw value. No SCALE is advertised: this is an enum index, not an angle in
 * degrees, so exposing a scale would mislead a reader into reading it as rotation.
 */
static const struct iio_chan_spec scp_devorient_channels[] = {
	{
		.type = IIO_ROT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index = 0,
		.scan_type = { .sign = 'u', .realbits = 32,
			       .storagebits = 32, .endianness = IIO_CPU },
	},
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

/* step counter = a single processed, cumulative count (no scale) */
static const struct iio_chan_spec scp_steps_channels[] = {
	{
		.type = IIO_STEPS,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
		.scan_index = 0,
		.scan_type = { .sign = 'u', .realbits = 32,
			       .storagebits = 32, .endianness = IIO_CPU },
	},
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

/*
 * rotation-vector family = one unit quaternion in a single channel (repeat 4).
 * Read back through read_raw_multi; the four components are reordered to the
 * IIO w,x,y,z convention from the ring's Android x,y,z,w order.
 */
static const struct iio_chan_spec scp_rotvec_channels[] = {
	{
		.type = IIO_ROT,
		.modified = 1,
		.channel2 = IIO_MOD_QUATERNION,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
					    BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_index = 0,
		.scan_type = { .sign = 's', .realbits = 32, .storagebits = 32,
			       .repeat = 4, .endianness = IIO_CPU },
	},
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

/*
 * Event-only sensors: no sample channel, just an IIO event delivered when the
 * SCP reports a detection. significant-motion follows the cros_ec convention
 * (activity leaving "still"); tilt has no canonical IIO type, so it is a
 * generic activity gesture.
 */
static const struct iio_event_spec scp_sigmotion_event = {
	.type = IIO_EV_TYPE_CHANGE,
	.dir = IIO_EV_DIR_FALLING,
	.mask_separate = BIT(IIO_EV_INFO_ENABLE),
};
static const struct iio_chan_spec scp_sigmotion_channels[] = {
	{
		.type = IIO_ACTIVITY,
		.modified = 1,
		.channel2 = IIO_MOD_STILL,
		.event_spec = &scp_sigmotion_event,
		.num_event_specs = 1,
		.scan_index = -1,
	},
};
static const struct iio_event_spec scp_tilt_event = {
	.type = IIO_EV_TYPE_GESTURE,
	.dir = IIO_EV_DIR_NONE,
	.mask_separate = BIT(IIO_EV_INFO_ENABLE),
};
static const struct iio_chan_spec scp_tilt_channels[] = {
	{
		.type = IIO_ACTIVITY,
		.event_spec = &scp_tilt_event,
		.num_event_specs = 1,
		.scan_index = -1,
	},
};
/* wake-gesture: a one-shot trigger like significant-motion, as a rising gesture */
static const struct iio_event_spec scp_wake_event = {
	.type = IIO_EV_TYPE_GESTURE,
	.dir = IIO_EV_DIR_RISING,
	.mask_separate = BIT(IIO_EV_INFO_ENABLE),
};
static const struct iio_chan_spec scp_wake_channels[] = {
	{
		.type = IIO_ACTIVITY,
		.event_spec = &scp_wake_event,
		.num_event_specs = 1,
		.scan_index = -1,
	},
};

/*
 * _udiv is an extra divisor applied on top of the vendor gain so the IIO scale
 * yields the unit the IIO ABI mandates for that channel type. The SCP reports
 * accel in m/s^2, gyro in rad/s and light in lux already (raw/gain), which is
 * what IIO_ACCEL/ANGL_VEL/LIGHT expect, so _udiv is 1. The magnetometer is
 * reported in microtesla but IIO_MAGN is in Gauss (1 Gauss = 100 uT), so it
 * needs an extra /100. Proximity is dimensionless.
 */
#define SCP_SENSOR_FULL(_t, _name, _ch, _nv, _gain, _udiv, _oc, _fus) {	\
	.type = _t, .name = _name, .channels = _ch,		\
	.num_channels = ARRAY_SIZE(_ch), .nvals = _nv,		\
	.default_gain = _gain, .unit_div = _udiv,		\
	.period_ns = SCP_SENSOR_PERIOD_NS,			\
	.on_change = _oc, .fusion = _fus,			\
}
#define SCP_SENSOR_F(_t, _name, _ch, _nv, _gain, _udiv, _oc)	\
	SCP_SENSOR_FULL(_t, _name, _ch, _nv, _gain, _udiv, _oc, false)
#define SCP_SENSOR(_t, _name, _ch, _nv, _gain, _udiv)		\
	SCP_SENSOR_FULL(_t, _name, _ch, _nv, _gain, _udiv, false, false)
/* SCP-fused virtual sensor: needs a standalone CONFIG to start/stop its pipeline */
#define SCP_FUSION(_t, _name, _ch, _nv, _gain, _udiv)		\
	SCP_SENSOR_FULL(_t, _name, _ch, _nv, _gain, _udiv, false, true)
/* fused event sensor: no sample channel, delivers an IIO event on detection */
#define SCP_EVENT(_t, _name, _ch) {				\
	.type = _t, .name = _name, .channels = _ch,		\
	.num_channels = ARRAY_SIZE(_ch), .nvals = 0,		\
	.default_gain = 1, .unit_div = 1,			\
	.period_ns = SCP_SENSOR_PERIOD_NS,			\
	.fusion = true, .event = true,				\
}

static const struct scp_sensor_desc scp_sensor_table[] = {
	SCP_SENSOR(SCP_SENSOR_TYPE_ACCEL,  "mtk-scp-accel",     scp_accel_channels, 3, 1000,    1),
	SCP_SENSOR(SCP_SENSOR_TYPE_GYRO,   "mtk-scp-gyro",      scp_gyro_channels,  3, 1000000, 1),
	SCP_SENSOR(SCP_SENSOR_TYPE_MAGN,   "mtk-scp-magn",      scp_magn_channels,  3, 1000,    100),
	SCP_SENSOR(SCP_SENSOR_TYPE_ALS,    "mtk-scp-als",       scp_light_channels, 1, 1,       1),
	SCP_SENSOR(SCP_SENSOR_TYPE_PS,     "mtk-scp-proximity", scp_prox_channels,  1, 1,       1),
	SCP_SENSOR_F(SCP_SENSOR_TYPE_SAR,  "mtk-scp-sar",       scp_sar_channels,   1, 1, 1, true),
	SCP_FUSION(SCP_SENSOR_TYPE_GRAVITY,      "mtk-scp-gravity",      scp_gravity_channels,  3, 1000000, 1),
	SCP_FUSION(SCP_SENSOR_TYPE_LINEAR_ACCEL, "mtk-scp-linear-accel", scp_linaccel_channels, 3, 1000000, 1),
	SCP_FUSION(SCP_SENSOR_TYPE_ORIENTATION,  "mtk-scp-orientation",  scp_orient_channels,   3, 1000000, 1),
	SCP_FUSION(SCP_SENSOR_TYPE_ROTATION,        "mtk-scp-rotation-vector",   scp_rotvec_channels, 4, 1000000, 1),
	SCP_FUSION(SCP_SENSOR_TYPE_GAME_ROTATION,   "mtk-scp-game-rotation",     scp_rotvec_channels, 4, 1000000, 1),
	SCP_FUSION(SCP_SENSOR_TYPE_GEOMAG_ROTATION, "mtk-scp-geomag-rotation",   scp_rotvec_channels, 4, 1000000, 1),
	SCP_FUSION(SCP_SENSOR_TYPE_UNCAL_ACCEL, "mtk-scp-uncal-accel", scp_uncal_accel_channels, 3, 1000,    1),
	SCP_FUSION(SCP_SENSOR_TYPE_UNCAL_GYRO,  "mtk-scp-uncal-gyro",  scp_uncal_gyro_channels,  3, 1000000, 1),
	SCP_FUSION(SCP_SENSOR_TYPE_UNCAL_MAGN,  "mtk-scp-uncal-magn",  scp_uncal_magn_channels,  3, 1000,    100),
	/* step counter: fused, and on-change so enable flushes the current total */
	SCP_SENSOR_FULL(SCP_SENSOR_TYPE_STEP_COUNT, "mtk-scp-step-counter", scp_steps_channels, 1, 1, 1, true, true),
	/* device orientation: fused, on-change (enable flushes the current quadrant) */
	SCP_SENSOR_FULL(SCP_SENSOR_TYPE_DEV_ORIENT, "mtk-scp-device-orientation", scp_devorient_channels, 1, 1, 1, true, true),
	SCP_EVENT(SCP_SENSOR_TYPE_SIGNIFICANT, "mtk-scp-significant-motion", scp_sigmotion_channels),
	SCP_EVENT(SCP_SENSOR_TYPE_TILT,        "mtk-scp-tilt",              scp_tilt_channels),
	SCP_EVENT(SCP_SENSOR_TYPE_WAKE,        "mtk-scp-wake-gesture",      scp_wake_channels),
};
#define SCP_NR_SENSORS ARRAY_SIZE(scp_sensor_table)

/*
 * A quaternion channel returns all four components from one read; every other
 * channel returns a single value. read_raw_multi shadows read_raw in the IIO
 * core, so all channels are served here.
 */
static int scp_read_raw_multi(struct iio_dev *indio,
			      struct iio_chan_spec const *chan,
			      int size, int *vals, int *val_len, long mask)
{
	struct scp_sensor *s = iio_to_sensor(indio);
	bool quat = chan->type == IIO_ROT &&
		    chan->channel2 == IIO_MOD_QUATERNION;
	unsigned long flags;
	int ret, tries, nv;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
	case IIO_CHAN_INFO_PROCESSED:	/* step counter reports a processed count */
	case IIO_CHAN_INFO_CALIBBIAS: {	/* uncalibrated sensor's bias estimate */
		/* the bias triple sits at ring value[3..5], behind the reading */
		int base = mask == IIO_CHAN_INFO_CALIBBIAS ? 3 : 0;

		nv = quat ? 4 : 1;
		if (size < nv)
			return -EINVAL;
		/*
		 * On-demand: enable the sensor for the duration of the read (a
		 * no-op if a buffer already holds it on), wait briefly for the
		 * first cached sample, then release it so an idle SCP can sleep.
		 * A fused sensor emits meaningless zeros until its filter has
		 * converged (a fraction of a second), so give it a settle delay
		 * first; the loop then returns the latest, converged sample.
		 */
		ret = scp_sensor_get(s);
		if (ret)
			return ret;
		if (s->desc->fusion)
			msleep(SCP_FUSION_SETTLE_MS);
		for (tries = 0; tries < 6; tries++) {
			spin_lock_irqsave(&s->lock, flags);
			if (s->valid) {
				if (quat) {
					/* ring order x,y,z,w -> IIO w,x,y,z */
					vals[0] = s->val[3];
					vals[1] = s->val[0];
					vals[2] = s->val[1];
					vals[3] = s->val[2];
				} else {
					vals[0] = s->val[base + chan->scan_index];
				}
				spin_unlock_irqrestore(&s->lock, flags);
				scp_sensor_put(s);
				*val_len = nv;
				return quat ? IIO_VAL_INT_MULTIPLE : IIO_VAL_INT;
			}
			spin_unlock_irqrestore(&s->lock, flags);
			msleep(60);
		}
		scp_sensor_put(s);
		return -EAGAIN;
	}
	case IIO_CHAN_INFO_SCALE:
		/* raw * scale = IIO-standard unit (m/s^2, rad/s, Gauss, lux, deg) */
		if (size < 2)
			return -EINVAL;
		vals[0] = 1;
		vals[1] = s->gain * s->desc->unit_div;
		*val_len = 2;
		return IIO_VAL_FRACTIONAL;
	case IIO_CHAN_INFO_SAMP_FREQ:
		vals[0] = NSEC_PER_SEC / s->desc->period_ns;
		*val_len = 1;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

/* event sensors arm/disarm through the on-demand enable path */
static int scp_read_event_config(struct iio_dev *indio,
				 const struct iio_chan_spec *chan,
				 enum iio_event_type type,
				 enum iio_event_direction dir)
{
	return iio_to_sensor(indio)->ev_on;
}

static int scp_write_event_config(struct iio_dev *indio,
				  const struct iio_chan_spec *chan,
				  enum iio_event_type type,
				  enum iio_event_direction dir, bool state)
{
	struct scp_sensor *s = iio_to_sensor(indio);
	int ret;

	if (state == s->ev_on)
		return 0;
	if (state) {
		ret = scp_sensor_get(s);
		if (ret)
			return ret;
	} else {
		scp_sensor_put(s);
	}
	s->ev_on = state;
	return 0;
}

static const struct iio_info scp_sensor_iio_info = {
	.read_raw_multi = scp_read_raw_multi,
	.read_event_config = scp_read_event_config,
	.write_event_config = scp_write_event_config,
};

static struct scp_sensor *scp_find_sensor(struct mt6878_scp *scp, u8 type)
{
	int i;

	for (i = 0; i < SCP_NR_SENSORS; i++)
		if (scp->sensors[i].desc->type == type)
			return &scp->sensors[i];
	return NULL;
}

/* Write a message into a send pin's slot and ring the SCP's doorbell. */
static void scp_mbox_send(struct scp_mbox *mb, unsigned int bit,
			  const void *data, size_t len)
{
	const u32 *w = data;
	size_t i;

	for (i = 0; i < DIV_ROUND_UP(len, 4); i++)
		writel(w[i], mb->base + SCP_PIN_OFF(bit) + i * 4);
	/* flush the slot before the trigger the SCP will react to */
	wmb();
	writel(BIT(bit), mb->base + SCP_MBOX_SET_IRQ);
}

static void scp_send_sensor_ready(struct mt6878_scp *scp)
{
	struct scp_sensor_notify n = {
		.sequence = ++scp->seq,
		.sensor_type = 0,
		.command = SCP_SENSOR_NOTIFY_READY,
		.length = 0,
	};

	n.crc8 = scp_crc8((u8 *)&n, 4);
	scp_mbox_send(&scp->mbox[SCP_SENSOR_MBOX], SCP_SENSOR_NOTIFY_OUT_BIT,
		      &n, sizeof(n));
	dev_info(scp->dev, "sent sensorhub host-ready (seq %u)\n", n.sequence);
}

static void scp_sensor_init_work(struct work_struct *work)
{
	struct mt6878_scp *scp =
		container_of(work, struct mt6878_scp, sensor_init);

	scp_send_sensor_ready(scp);
}

/*
 * Ring the SCP awake doorbell with one opcode (LOCK/UNLOCK) and wait for the
 * SCP to consume it: the opcode bit in INFRA_IRQ_SET self-clears once the core
 * has woken and acknowledged, after which we deassert via INFRA_IRQ_CLEAR.
 * Returns 0 on ack, -ETIMEDOUT if the bit never cleared (SCP did not wake).
 */
static int scp_awake_ring(struct mt6878_scp *scp, u32 opcode)
{
	u32 bit = BIT(opcode);
	u32 v;
	int ret;

	if (!scp->infracfg)
		return -ENODEV;

	regmap_write(scp->infracfg, SCP_AWAKE_IRQ_SET, SCP_AWAKE_TAG | bit);
	ret = regmap_read_poll_timeout_atomic(scp->infracfg, SCP_AWAKE_IRQ_SET,
					      v, !(v & bit), 1,
					      SCP_AWAKE_POLL_US);
	regmap_write(scp->infracfg, SCP_AWAKE_IRQ_CLEAR, SCP_AWAKE_TAG | bit);
	return ret;
}

static int scp_awake_lock(struct mt6878_scp *scp)
{
	return scp_awake_ring(scp, SCP_AWAKE_LOCK);
}

static int scp_awake_unlock(struct mt6878_scp *scp)
{
	return scp_awake_ring(scp, SCP_AWAKE_UNLOCK);
}

static int scp_ctrl_send(struct mt6878_scp *scp, struct scp_sensor_ctrl *c,
			 size_t len)
{
	int ret = 0;

	/*
	 * Serialize CTRL traffic (one ack completion, one CTRL-out mailbox slot)
	 * and bracket it with the awake handshake so a sleeping SCP is woken for
	 * the whole command-and-ack, not just the instant the slot is written.
	 */
	mutex_lock(&scp->ctrl_lock);
	c->sequence = ++scp->seq;
	c->crc8 = scp_crc8((u8 *)c, 4);

	reinit_completion(&scp->ctrl_ack);
	scp_awake_lock(scp);
	scp_mbox_send(&scp->mbox[SCP_SENSOR_MBOX], SCP_SENSOR_CTRL_OUT_BIT,
		      c, len);
	if (!wait_for_completion_timeout(&scp->ctrl_ack,
			msecs_to_jiffies(SCP_REPLY_TIMEOUT_MS))) {
		dev_err(scp->dev, "ctrl cmd %u: no ack\n", c->command);
		ret = -ETIMEDOUT;
	} else if (scp->ack_ret < 0) {
		dev_err(scp->dev, "ctrl cmd %u rejected (%d)\n",
			c->command, scp->ack_ret);
		ret = -EREMOTEIO;
	}
	scp_awake_unlock(scp);
	mutex_unlock(&scp->ctrl_lock);
	return ret;
}

/* Write a fresh ring header (rp/wp/buffer_size/item_size) into a region. */
static void scp_ring_init(void __iomem *ring, u32 region_size, u32 item_size)
{
	u32 bufsz = ((region_size - SCP_SHM_HDR_SIZE) / item_size) * item_size;

	writel(0, ring + 0);			/* rp */
	writel(0, ring + 4);			/* wp */
	writel(bufsz, ring + 8);		/* buffer_size */
	writel(item_size, ring + 12);		/* item_size */
	wmb();
}

/* Tell the SCP where the sensor rings live (prerequisite for list/samples). */
static int scp_config_share_mem(struct mt6878_scp *scp)
{
	struct {
		struct scp_sensor_ctrl c;
		struct scp_share_mem shm;
	} __packed pkt = {};

	pkt.c.sensor_type = 0;
	pkt.c.command = SCP_SENSOR_CTRL_SHARE_MEMORY;
	pkt.c.length = sizeof(pkt.shm);
	pkt.shm.available_num = 2;
	pkt.shm.base_info[0].payload_type = SCP_SHM_PAYLOAD_DATA;
	pkt.shm.base_info[0].payload_base = SCP_SHM_DATA_BASE;
	pkt.shm.base_info[1].payload_type = SCP_SHM_PAYLOAD_LIST;
	pkt.shm.base_info[1].payload_base = SCP_SHM_LIST_BASE;

	return scp_ctrl_send(scp, &pkt.c, sizeof(pkt));
}

/* Hand the SCP a host timebase (boot ns, arch counter, sched_clock, wall). */
static int scp_send_timesync(struct mt6878_scp *scp)
{
	struct {
		struct scp_sensor_ctrl c;
		struct scp_timesync t;
	} __packed pkt = {};
	struct timespec64 rt;
	struct rtc_time tm;
	unsigned long flags;

	pkt.c.sensor_type = 0;
	pkt.c.command = SCP_SENSOR_CTRL_TIMESYNC;
	pkt.c.length = sizeof(pkt.t);

	local_irq_save(flags);
	pkt.t.host_timestamp = ktime_get_boottime_ns();
	pkt.t.host_archcounter = __arch_counter_get_cntvct();
	pkt.t.sched_clock = sched_clock();
	ktime_get_real_ts64(&rt);
	local_irq_restore(flags);

	rt.tv_sec -= (s64)sys_tz.tz_minuteswest * 60;
	rtc_time64_to_tm(rt.tv_sec, &tm);
	pkt.t.usecond = rt.tv_nsec / 1000;
	pkt.t.second = tm.tm_sec;
	pkt.t.minute = tm.tm_min;
	pkt.t.hour = tm.tm_hour;
	pkt.t.day = tm.tm_mday;
	pkt.t.month = tm.tm_mon + 1;

	return scp_ctrl_send(scp, &pkt.c, sizeof(pkt));
}

/* Ask the SCP for its sensor list and read the records back from the ring. */
static int scp_query_sensor_list(struct mt6878_scp *scp)
{
	struct scp_sensor *s;
	u32 wp, rp;
	int count = 0, try;

	for (try = 0; try < 3; try++) {
		struct scp_sensor_notify n = {
			.command = SCP_SENSOR_NOTIFY_LIST,
		};

		writel(0, scp->list_ring + 0);	/* reset read pointer */
		wmb();

		n.sequence = ++scp->seq;
		n.crc8 = scp_crc8((u8 *)&n, 4);
		reinit_completion(&scp->notify_reply);
		scp_mbox_send(&scp->mbox[SCP_SENSOR_MBOX],
			      SCP_SENSOR_NOTIFY_OUT_BIT, &n, sizeof(n));
		if (wait_for_completion_timeout(&scp->notify_reply,
				msecs_to_jiffies(1000)))
			break;
		dev_warn(scp->dev, "sensor list: no reply (try %d)\n", try + 1);
	}
	if (try == 3)
		return -ETIMEDOUT;

	wp = scp->notify_wp;
	for (rp = 0; rp + SCP_SHM_LIST_ITEM <= wp &&
	     SCP_SHM_HDR_SIZE + rp + SCP_SHM_LIST_ITEM <= SCP_SHM_LIST_SIZE;
	     rp += SCP_SHM_LIST_ITEM) {
		struct scp_sensor_info rec;

		memcpy_fromio(&rec, scp->list_ring + SCP_SHM_HDR_SIZE + rp,
			      sizeof(rec));
		if (!rec.sensor_type || !rec.gain)
			continue;
		dev_info(scp->dev,
			 "sensor type %2u gain %u name '%.16s' vendor '%.16s'\n",
			 rec.sensor_type, rec.gain, rec.name, rec.vendor);
		s = scp_find_sensor(scp, rec.sensor_type);
		if (s && rec.gain)
			s->gain = rec.gain;
		count++;
	}
	dev_info(scp->dev, "sensor hub lists %d sensor(s) (wp=%u)\n", count, wp);
	return 0;
}

/*
 * Enable a sensor at a given sample period / max report latency (nanoseconds).
 * Waits for the SCP's ack: besides confirming the command this serializes the
 * per-sensor enables at bring-up, since they all go through the one CTRL-out pin
 * and a fresh send would otherwise clobber the slot before the SCP reads it.
 */
static int scp_sensor_enable(struct mt6878_scp *scp, u8 type,
			     s64 delay_ns, s64 latency_ns)
{
	struct {
		struct scp_sensor_ctrl c;
		struct scp_sensor_batch b;
	} __packed pkt = {};

	pkt.c.sensor_type = type;
	pkt.c.command = SCP_SENSOR_CTRL_ENABLE;
	pkt.c.length = sizeof(pkt.b);
	pkt.b.delay = delay_ns;
	pkt.b.latency = latency_ns;

	return scp_ctrl_send(scp, &pkt.c, sizeof(pkt));
}

/*
 * Send a standalone CONFIG (set-delay/batch) command. Physical sensors stream
 * from ENABLE alone, but the SCP's fused virtual sensors (gravity, linear-accel,
 * ...) only run their pipeline once a CONFIG has set a sample period: ENABLE
 * merely marks them active. A CONFIG with delay == 0 stops that pipeline again,
 * which a bare DISABLE does not do (the fusion task keeps emitting otherwise).
 */
static int scp_sensor_config(struct mt6878_scp *scp, u8 type,
			     s64 delay_ns, s64 latency_ns)
{
	struct {
		struct scp_sensor_ctrl c;
		struct scp_sensor_batch b;
	} __packed pkt = {};

	pkt.c.sensor_type = type;
	pkt.c.command = SCP_SENSOR_CTRL_CONFIG;
	pkt.c.length = sizeof(pkt.b);
	pkt.b.delay = delay_ns;
	pkt.b.latency = latency_ns;

	return scp_ctrl_send(scp, &pkt.c, sizeof(pkt));
}

/*
 * Flush a sensor: ask the hub to push any buffered samples into the DATA ring
 * now. Issued for on-change sensors (e.g. the aw963xx SAR) when they are
 * enabled; the hub answers with a flush-complete meta event (action != 0, which
 * the drain ignores) and, if it has one, the current sample. On-change sensors
 * sitting at their baseline have nothing to report yet, so their IIO *_raw reads
 * stay -EAGAIN until the first real change event.
 */
static int scp_sensor_flush(struct mt6878_scp *scp, u8 type)
{
	struct scp_sensor_ctrl c = {
		.sensor_type = type,
		.command = SCP_SENSOR_CTRL_FLUSH,
	};

	return scp_ctrl_send(scp, &c, sizeof(c));
}

/*
 * Best-effort disable: the SCP acts on the command (the stream stops) but does
 * not reliably ack a DISABLE issued while samples are still in flight, so fire
 * it without waiting rather than log a spurious timeout. Still take the CTRL
 * lock and the awake handshake so it doesn't clobber a concurrent command and
 * so a drowsy SCP is woken to see it.
 */
static void scp_sensor_disable(struct mt6878_scp *scp, u8 type)
{
	struct scp_sensor_ctrl c = {
		.sensor_type = type,
		.command = SCP_SENSOR_CTRL_DISABLE,
	};

	mutex_lock(&scp->ctrl_lock);
	c.sequence = ++scp->seq;
	c.crc8 = scp_crc8((u8 *)&c, 4);
	scp_awake_lock(scp);
	scp_mbox_send(&scp->mbox[SCP_SENSOR_MBOX], SCP_SENSOR_CTRL_OUT_BIT,
		      &c, sizeof(c));
	scp_awake_unlock(scp);
	mutex_unlock(&scp->ctrl_lock);
}

/*
 * On-demand enable: bring a sensor online for the first user and take it offline
 * when the last one leaves. The enable CTRL command wakes a sleeping SCP (via
 * scp_ctrl_send's awake handshake); once a sensor is streaming its own traffic
 * keeps the SCP awake, and once the last sensor stops the SCP is free to sleep.
 * The on-change SAR is flushed on enable so its first reading is primed.
 */
static int scp_sensor_get(struct scp_sensor *s)
{
	struct mt6878_scp *scp = s->scp;
	int ret = 0;

	mutex_lock(&s->en_lock);
	if (s->users == 0) {
		ret = scp_sensor_enable(scp, s->desc->type,
					s->desc->period_ns, 0);
		/*
		 * Fused sensors need a standalone CONFIG to start their pipeline;
		 * the SCP's fusion manager brings up its own physical inputs
		 * (accel/gyro/mag), so we do not enable those from here.
		 */
		if (!ret && s->desc->fusion)
			scp_sensor_config(scp, s->desc->type,
					  s->desc->period_ns, 0);
		if (!ret && s->desc->on_change)
			scp_sensor_flush(scp, s->desc->type);
	}
	if (!ret)
		s->users++;
	mutex_unlock(&s->en_lock);
	return ret;
}

static void scp_sensor_put(struct scp_sensor *s)
{
	unsigned long flags;

	mutex_lock(&s->en_lock);
	if (!WARN_ON(s->users == 0) && --s->users == 0) {
		/* stop a fused sensor's pipeline first; DISABLE alone won't */
		if (s->desc->fusion)
			scp_sensor_config(s->scp, s->desc->type, 0, 0);
		scp_sensor_disable(s->scp, s->desc->type);
		spin_lock_irqsave(&s->lock, flags);
		s->valid = false;	/* cached sample is stale once stopped */
		spin_unlock_irqrestore(&s->lock, flags);
	}
	mutex_unlock(&s->en_lock);
}

/* IIO buffer open/close maps straight onto the on-demand refcount. */
static int scp_buffer_postenable(struct iio_dev *indio)
{
	return scp_sensor_get(iio_to_sensor(indio));
}

static int scp_buffer_predisable(struct iio_dev *indio)
{
	scp_sensor_put(iio_to_sensor(indio));
	return 0;
}

static const struct iio_buffer_setup_ops scp_buffer_ops = {
	.postenable = scp_buffer_postenable,
	.predisable = scp_buffer_predisable,
};

/*
 * Drain the DATA ring up to the SCP's reported write pointer. rp/wp are byte
 * offsets into the circular buffer (past the 16-byte header); the firmware
 * never splits a record across the wrap, so each item can be read whole.
 */
static void scp_drain_data_work(struct work_struct *work)
{
	struct mt6878_scp *scp = container_of(work, struct mt6878_scp, drain);
	u32 bufsz = readl(scp->data_ring + 8);
	u32 wp = scp->data_wp;
	u32 rp = readl(scp->data_ring + 0);

	if (!bufsz || wp >= bufsz)
		return;

	while (rp != wp) {
		struct scp_sensor_data rec;
		struct scp_sensor *s;
		unsigned long flags;
		bool first;
		int i, nv;

		memcpy_fromio(&rec, scp->data_ring + SCP_SHM_HDR_SIZE + rp,
			      sizeof(rec));
		rp = (rp + SCP_SHM_DATA_ITEM) % bufsz;

		/*
		 * Only action 0 carries an actual sample. Other actions are meta
		 * events (action 1 = flush-complete, etc.) whose value[] is not a
		 * reading, so skip them rather than cache their payload.
		 */
		if (rec.action != SCP_SENSOR_ACTION_DATA)
			continue;

		/* route the record to its sensor by type (ignore unknown) */
		s = scp_find_sensor(scp, rec.sensor_type);
		if (!s)
			continue;

		/* event sensors carry a detection, not a sample: push an event */
		if (s->desc->event) {
			const struct iio_chan_spec *c = &s->desc->channels[0];
			u64 code = c->modified ?
				IIO_MOD_EVENT_CODE(c->type, 0, c->channel2,
						   c->event_spec[0].type,
						   c->event_spec[0].dir) :
				IIO_UNMOD_EVENT_CODE(c->type, 0,
						     c->event_spec[0].type,
						     c->event_spec[0].dir);

			iio_push_event(s->indio, code, rec.timestamp);
			continue;
		}
		nv = s->desc->nvals;

		/*
		 * Cache the whole record (all six value words) so sysfs reads
		 * can serve both the reading (value[0..2]) and, for uncalibrated
		 * sensors, the bias estimate (value[3..5]) via calibbias.
		 */
		spin_lock_irqsave(&s->lock, flags);
		for (i = 0; i < 6; i++)
			s->val[i] = rec.value[i];
		s->accuracy = rec.accuracy;
		s->ts = rec.timestamp;
		first = !s->valid;
		s->valid = true;
		spin_unlock_irqrestore(&s->lock, flags);

		if (first)
			dev_dbg(scp->dev, "%s first sample (v0=%d acc %u)\n",
				s->desc->name, rec.value[0], rec.accuracy);

		/* and push it to this sensor's capture buffer if one is open */
		if (iio_buffer_enabled(s->indio)) {
			struct {
				s32 chan[4];
				s64 ts __aligned(8);
			} sample;

			if (nv == 4) {
				/* quaternion: ring x,y,z,w -> IIO w,x,y,z */
				sample.chan[0] = rec.value[3];
				sample.chan[1] = rec.value[0];
				sample.chan[2] = rec.value[1];
				sample.chan[3] = rec.value[2];
			} else {
				for (i = 0; i < nv; i++)
					sample.chan[i] = rec.value[i];
			}
			iio_push_to_buffers_with_timestamp(s->indio, &sample,
							   rec.timestamp);
		}
	}

	/* publish the advanced read pointer so the SCP can reuse the space */
	writel(rp, scp->data_ring + 0);
	wmb();
}

static void scp_sensorhub_work(struct work_struct *work)
{
	struct mt6878_scp *scp =
		container_of(work, struct mt6878_scp, sensorhub);

	scp->list_ring = ioremap_wc(SCP_SHM_LIST_BASE, SCP_SHM_LIST_SIZE);
	scp->data_ring = ioremap_wc(SCP_SHM_DATA_BASE, SCP_SHM_DATA_SIZE);
	if (!scp->list_ring || !scp->data_ring) {
		dev_err(scp->dev, "cannot map sensor rings\n");
		return;
	}

	scp_ring_init(scp->list_ring, SCP_SHM_LIST_SIZE, SCP_SHM_LIST_ITEM);
	scp_ring_init(scp->data_ring, SCP_SHM_DATA_SIZE, SCP_SHM_DATA_ITEM);

	if (scp_config_share_mem(scp))
		return;
	dev_info(scp->dev, "sensor share-memory configured\n");

	if (scp_send_timesync(scp))
		dev_warn(scp->dev, "timesync failed\n");
	else
		dev_info(scp->dev, "timesync sent\n");

	scp_query_sensor_list(scp);

	/*
	 * Sensors are enabled on demand (first IIO reader or buffer open) rather
	 * than kept running: each enable wakes the SCP through the awake handshake
	 * in scp_ctrl_send, and with no sensor active the SCP is free to sleep.
	 */
	dev_info(scp->dev, "sensor hub ready; sensors enabled on demand\n");
}

static irqreturn_t scp_mbox_isr(int irq, void *data)
{
	struct scp_mbox *mb = data;
	struct mt6878_scp *scp = mb->scp;
	u32 status = readl(mb->base + SCP_MBOX_CLR_IRQ);

	if (!status)
		return IRQ_NONE;

	if (mb->id == SCP_SENSOR_MBOX && (status & BIT(SCP_READY_BIT))) {
		scp->image_size = readl(mb->base + SCP_PIN_OFF(SCP_READY_BIT));
		if (atomic_xchg(&scp->ready, 1) == 0) {
			dev_info(scp->dev, "SCP ready: image size %#x\n",
				 scp->image_size);
			/* the SCP is up; greet its sensor hub from a thread */
			schedule_work(&scp->sensor_init);
		}
	}

	if (mb->id == SCP_SENSOR_MBOX && (status & BIT(SCP_SENSOR_CTRL_ACK_BIT))) {
		struct scp_sensor_ack a;
		u32 *w = (u32 *)&a;
		unsigned int i;

		for (i = 0; i < sizeof(a) / 4; i++)
			w[i] = readl(mb->base +
				     SCP_PIN_OFF(SCP_SENSOR_CTRL_ACK_BIT) + i * 4);
		scp->ack_ret = a.ret_val;
		complete(&scp->ctrl_ack);
	}

	if (mb->id == SCP_SENSOR_MBOX && (status & BIT(SCP_SENSOR_NOTIFY_BIT))) {
		struct scp_sensor_notify n;
		u32 *w = (u32 *)&n;
		unsigned int i;

		for (i = 0; i < sizeof(n) / 4; i++)
			w[i] = readl(mb->base +
				     SCP_PIN_OFF(SCP_SENSOR_NOTIFY_BIT) + i * 4);

		switch (n.command) {
		case SCP_SENSOR_NOTIFY_READY:
			if (atomic_xchg(&scp->sensor_ready, 1) == 0) {
				dev_info(scp->dev, "sensor hub ready\n");
				schedule_work(&scp->sensorhub);
			}
			break;
		case SCP_SENSOR_NOTIFY_LIST:
			scp->notify_wp = n.value[0];
			complete(&scp->notify_reply);
			break;
		case SCP_SENSOR_NOTIFY_DATA:
			/* value[0] = ring write position; drain outside IRQ */
			scp->data_wp = n.value[0];
			schedule_work(&scp->drain);
			break;
		default:
			dev_dbg(scp->dev, "sensor NOTIFY cmd %u val0 %d\n",
				n.command, n.value[0]);
			break;
		}
	}

	/*
	 * The clr-irq register is level-reflecting write-1-to-clear: ack every
	 * pending bit, including pins we do not consume yet, or the source stays
	 * asserted and storms.
	 */
	writel(status, mb->base + SCP_MBOX_CLR_IRQ);
	return IRQ_HANDLED;
}

static irqreturn_t scp_ipc_isr(int irq, void *data)
{
	struct mt6878_scp *scp = data;

	/* IPC doorbells are not wired to a decoder yet; mask on first fire. */
	disable_irq_nosync(irq);
	dev_info(scp->dev, "SCP IPC doorbell IRQ %d (masked)\n", irq);
	return IRQ_HANDLED;
}

static void mt6878_scp_report(struct work_struct *work)
{
	struct mt6878_scp *scp =
		container_of(to_delayed_work(work), struct mt6878_scp, report);

	if (!atomic_read(&scp->ready))
		dev_warn(scp->dev, "SCP did not signal ready within %d ms\n",
			 SCP_READY_TIMEOUT_MS);
	else if (atomic_read(&scp->sensor_ready))
		dev_info(scp->dev, "SCP ready (image %#x), sensor hub ready\n",
			 scp->image_size);
	else
		dev_info(scp->dev,
			 "SCP ready (image %#x), sensor hub did not answer\n",
			 scp->image_size);
}

static int mt6878_scp_request_mbox(struct platform_device *pdev,
				   struct mt6878_scp *scp)
{
	struct device *dev = &pdev->dev;
	char name[8];
	int i, irq, ret;

	for (i = 0; i < SCP_NR_MBOX; i++) {
		scp->mbox[i].scp = scp;
		scp->mbox[i].id = i;

		snprintf(name, sizeof(name), "mbox%d", i);
		scp->mbox[i].base =
			devm_platform_ioremap_resource_byname(pdev, name);
		if (IS_ERR(scp->mbox[i].base))
			return dev_err_probe(dev, PTR_ERR(scp->mbox[i].base),
					     "cannot map %s\n", name);

		irq = platform_get_irq_byname(pdev, name);
		if (irq < 0)
			return irq;
		ret = devm_request_irq(dev, irq, scp_mbox_isr, 0,
				       dev_name(dev), &scp->mbox[i]);
		if (ret)
			return dev_err_probe(dev, ret, "request %s irq\n", name);
	}

	/* IPC doorbells (optional, masked on first fire) */
	for (i = 0; i < 2; i++) {
		snprintf(name, sizeof(name), "ipc%d", i);
		irq = platform_get_irq_byname(pdev, name);
		if (irq < 0)
			continue;
		ret = devm_request_irq(dev, irq, scp_ipc_isr, 0,
				       dev_name(dev), scp);
		if (ret)
			dev_warn(dev, "cannot request %s irq: %d\n", name, ret);
	}

	return 0;
}

static int mt6878_scp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct scp_region_info ri;
	struct arm_smccc_res res;
	struct mt6878_scp *scp;
	int ret, i;

	scp = devm_kzalloc(dev, sizeof(*scp), GFP_KERNEL);
	if (!scp)
		return -ENOMEM;
	scp->dev = dev;
	atomic_set(&scp->ready, 0);
	atomic_set(&scp->sensor_ready, 0);
	mutex_init(&scp->ctrl_lock);
	init_completion(&scp->ctrl_ack);
	init_completion(&scp->notify_reply);
	INIT_WORK(&scp->sensor_init, scp_sensor_init_work);
	INIT_WORK(&scp->sensorhub, scp_sensorhub_work);
	INIT_WORK(&scp->drain, scp_drain_data_work);
	INIT_DELAYED_WORK(&scp->report, mt6878_scp_report);
	platform_set_drvdata(pdev, scp);

	scp->sram = devm_platform_ioremap_resource_byname(pdev, "sram");
	if (IS_ERR(scp->sram))
		return dev_err_probe(dev, PTR_ERR(scp->sram),
				     "cannot map SCP SRAM\n");

	/* infracfg_ao holds the SCP awake doorbell, reached via a syscon phandle */
	scp->infracfg = syscon_regmap_lookup_by_phandle(dev->of_node,
							"mediatek,infracfg");
	if (IS_ERR(scp->infracfg)) {
		dev_warn(dev, "no mediatek,infracfg; SCP awake handshake off\n");
		scp->infracfg = NULL;
	}

	memcpy_fromio(&ri, scp->sram + SCP_REGION_INFO_OFFSET, sizeof(ri));
	dev_info(dev,
		 "region_info: loader=%#x/%#x fw=%#x/%#x dram=%#x/%#x ssz=%#x\n",
		 ri.ap_loader_start, ri.ap_loader_size, ri.ap_firmware_start,
		 ri.ap_firmware_size, ri.ap_dram_start, ri.ap_dram_size,
		 ri.struct_size);
	if (ri.struct_size < 0x20 || !ri.ap_firmware_start) {
		dev_err(dev,
			"region_info invalid; bootloader did not stage SCP firmware\n");
		return -ENODEV;
	}

	ret = mt6878_scp_request_mbox(pdev, scp);
	if (ret)
		return ret;

	/*
	 * Expose each physical sensor as its own IIO device with a kfifo capture
	 * buffer. They are registered up front; the *_raw attributes return -EAGAIN
	 * and the buffers stay empty until the sensor hub comes up and the first
	 * DATA-ring record for that sensor has been cached.
	 */
	scp->sensors = devm_kcalloc(dev, SCP_NR_SENSORS, sizeof(*scp->sensors),
				    GFP_KERNEL);
	if (!scp->sensors)
		return -ENOMEM;
	for (i = 0; i < SCP_NR_SENSORS; i++) {
		struct scp_sensor *s = &scp->sensors[i];
		struct iio_dev *indio;

		s->scp = scp;
		s->desc = &scp_sensor_table[i];
		s->gain = s->desc->default_gain;
		spin_lock_init(&s->lock);
		mutex_init(&s->en_lock);

		indio = devm_iio_device_alloc(dev, sizeof(s));
		if (!indio)
			return -ENOMEM;
		*(struct scp_sensor **)iio_priv(indio) = s;
		indio->name = s->desc->name;
		indio->modes = INDIO_DIRECT_MODE;
		indio->info = &scp_sensor_iio_info;
		indio->channels = s->desc->channels;
		indio->num_channels = s->desc->num_channels;
		s->indio = indio;

		/* event sensors have no sample channel, so no capture buffer */
		if (!s->desc->event) {
			ret = devm_iio_kfifo_buffer_setup(dev, indio,
							  &scp_buffer_ops);
			if (ret)
				return dev_err_probe(dev, ret,
						     "cannot set up %s buffer\n",
						     s->desc->name);
		}
		ret = devm_iio_device_register(dev, indio);
		if (ret)
			return dev_err_probe(dev, ret, "cannot register %s\n",
					     s->desc->name);
	}

	/*
	 * Release the RV55 from reset. Firmware and the region-info descriptor
	 * were placed by the bootloader; the resident TF-A programs the
	 * DRAM-reserve GPRs (these are secure-only) and deasserts core0. The SCP
	 * then raises its ready IPI on the mailbox.
	 */
	dev_info(dev, "releasing SCP (SMC %#lx op %d)\n",
		 MTK_SIP_TINYSYS_SCP_CONTROL, SCP_KERNEL_OP_RESET_RELEASE);
	arm_smccc_smc(MTK_SIP_TINYSYS_SCP_CONTROL, SCP_KERNEL_OP_RESET_RELEASE,
		      0, 0, 0, 0, 0, 0, &res);
	dev_info(dev, "reset-release SMC returned %#lx\n", res.a0);
	if (res.a0)
		dev_warn(dev, "reset-release SMC reported error %#lx\n", res.a0);

	schedule_delayed_work(&scp->report,
			      msecs_to_jiffies(SCP_READY_TIMEOUT_MS));
	return 0;
}

static void mt6878_scp_remove(struct platform_device *pdev)
{
	struct mt6878_scp *scp = platform_get_drvdata(pdev);
	int i;

	if (atomic_read(&scp->sensor_ready))
		for (i = 0; i < SCP_NR_SENSORS; i++)
			if (scp->sensors[i].users)
				scp_sensor_disable(scp,
						   scp->sensors[i].desc->type);
	cancel_work_sync(&scp->sensor_init);
	cancel_work_sync(&scp->sensorhub);
	cancel_work_sync(&scp->drain);
	cancel_delayed_work_sync(&scp->report);
	if (scp->list_ring)
		iounmap(scp->list_ring);
	if (scp->data_ring)
		iounmap(scp->data_ring);
}

static const struct of_device_id mt6878_scp_of_match[] = {
	{ .compatible = "mediatek,mt6878-scp" },
	{}
};
MODULE_DEVICE_TABLE(of, mt6878_scp_of_match);

static struct platform_driver mt6878_scp_driver = {
	.probe = mt6878_scp_probe,
	.remove = mt6878_scp_remove,
	.driver = {
		.name = "mt6878-scp-rv55",
		.of_match_table = mt6878_scp_of_match,
	},
};
module_platform_driver(mt6878_scp_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek MT6878 SCP (RV55 tinysys) bring-up");
