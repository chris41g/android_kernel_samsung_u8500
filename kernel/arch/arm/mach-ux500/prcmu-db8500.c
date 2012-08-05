/*
 * Copyright (C) STMicroelectronics 2009
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License Terms: GNU General Public License v2
 * Author: Kumar Sanghvi <kumar.sanghvi@stericsson.com>
 * Author: Sundar Iyer <sundar.iyer@stericsson.com>
 * Author: Mattias Nilsson <mattias.i.nilsson@stericsson.com>
 *
 * U8500 PRCM Unit interface driver
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/bitops.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

/*
 * NOTE! Temporary until all users of set_hwacc() are using the regulator
 * framework API
 */
#include <linux/regulator/consumer.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include "prcmu-regs-db8500.h"
#include "prcmu-debug.h"
#include <mach/prcmu.h>
#include <mach/db8500-regs.h>

#define PRCMU_I2C_TIMEOUT	0x0F000000

/* Offset for the firmware version within the TCPM */
#define PRCMU_FW_VERSION_OFFSET 0xA4

/* PRCMU project numbers, defined by PRCMU FW */
#define PRCMU_PROJECT_ID_8500V1_0 1
#define PRCMU_PROJECT_ID_8500V2_0 2

/* Index of different voltages to be used when accessing AVSData */
#define PRCM_AVS_BASE		0x2FC
#define PRCM_AVS_VBB_RET	(PRCM_AVS_BASE + 0x0)
#define PRCM_AVS_VBB_MAX_OPP	(PRCM_AVS_BASE + 0x1)
#define PRCM_AVS_VBB_100_OPP	(PRCM_AVS_BASE + 0x2)
#define PRCM_AVS_VBB_50_OPP	(PRCM_AVS_BASE + 0x3)
#define PRCM_AVS_VARM_MAX_OPP	(PRCM_AVS_BASE + 0x4)
#define PRCM_AVS_VARM_100_OPP	(PRCM_AVS_BASE + 0x5)
#define PRCM_AVS_VARM_50_OPP	(PRCM_AVS_BASE + 0x6)
#define PRCM_AVS_VARM_RET	(PRCM_AVS_BASE + 0x7)
#define PRCM_AVS_VAPE_100_OPP	(PRCM_AVS_BASE + 0x8)
#define PRCM_AVS_VAPE_50_OPP	(PRCM_AVS_BASE + 0x9)
#define PRCM_AVS_VMOD_100_OPP	(PRCM_AVS_BASE + 0xA)
#define PRCM_AVS_VMOD_50_OPP	(PRCM_AVS_BASE + 0xB)
#define PRCM_AVS_VSAFE		(PRCM_AVS_BASE + 0xC)

#define PRCM_AVS_VOLTAGE		0
#define PRCM_AVS_VOLTAGE_MASK		0x3f
#define PRCM_AVS_ISSLOWSTARTUP		6
#define PRCM_AVS_ISSLOWSTARTUP_MASK	(1 << PRCM_AVS_ISSLOWSTARTUP)
#define PRCM_AVS_ISMODEENABLE		7
#define PRCM_AVS_ISMODEENABLE_MASK	(1 << PRCM_AVS_ISMODEENABLE)

#define PRCM_BOOT_STATUS	0xFFF
#define PRCM_ROMCODE_A2P	0xFFE
#define PRCM_ROMCODE_P2A	0xFFD
#define PRCM_XP70_CUR_PWR_STATE 0xFFC      /* 4 BYTES */

#define PRCM_SW_RST_REASON 0xFF8 /* 2 bytes */

#define _PRCM_MBOX_HEADER		0xFE8 /* 16 bytes */
#define PRCM_MBOX_HEADER_REQ_MB0	(_PRCM_MBOX_HEADER + 0x0)
#define PRCM_MBOX_HEADER_REQ_MB1	(_PRCM_MBOX_HEADER + 0x1)
#define PRCM_MBOX_HEADER_REQ_MB2	(_PRCM_MBOX_HEADER + 0x2)
#define PRCM_MBOX_HEADER_REQ_MB3	(_PRCM_MBOX_HEADER + 0x3)
#define PRCM_MBOX_HEADER_REQ_MB4	(_PRCM_MBOX_HEADER + 0x4)
#define PRCM_MBOX_HEADER_REQ_MB5	(_PRCM_MBOX_HEADER + 0x5)
#define PRCM_MBOX_HEADER_ACK_MB0	(_PRCM_MBOX_HEADER + 0x8)

/* Req Mailboxes */
#define PRCM_REQ_MB0 0xFDC /* 12 bytes  */
#define PRCM_REQ_MB1 0xFD0 /* 12 bytes  */
#define PRCM_REQ_MB2 0xFC0 /* 16 bytes  */
#define PRCM_REQ_MB3 0xE4C /* 372 bytes  */
#define PRCM_REQ_MB4 0xE48 /* 4 bytes  */
#define PRCM_REQ_MB5 0xE44 /* 4 bytes  */

/* Ack Mailboxes */
#define PRCM_ACK_MB0 0xE08 /* 52 bytes  */
#define PRCM_ACK_MB1 0xE04 /* 4 bytes */
#define PRCM_ACK_MB2 0xE00 /* 4 bytes */
#define PRCM_ACK_MB3 0xDFC /* 4 bytes */
#define PRCM_ACK_MB4 0xDF8 /* 4 bytes */
#define PRCM_ACK_MB5 0xDF4 /* 4 bytes */

/* Mailbox 0 headers */
#define MB0H_POWER_STATE_TRANS		0
#define MB0H_CONFIG_WAKEUPS_EXE		1
#define MB0H_READ_WAKEUP_ACK		3
#define MB0H_CONFIG_WAKEUPS_SLEEP	4

#define MB0H_WAKEUP_EXE 2
#define MB0H_WAKEUP_SLEEP 5

/* Mailbox 0 REQs */
#define PRCM_REQ_MB0_AP_POWER_STATE	(PRCM_REQ_MB0 + 0x0)
#define PRCM_REQ_MB0_AP_PLL_STATE	(PRCM_REQ_MB0 + 0x1)
#define PRCM_REQ_MB0_ULP_CLOCK_STATE	(PRCM_REQ_MB0 + 0x2)
#define PRCM_REQ_MB0_DO_NOT_WFI		(PRCM_REQ_MB0 + 0x3)
#define PRCM_REQ_MB0_WAKEUP_8500	(PRCM_REQ_MB0 + 0x4)
#define PRCM_REQ_MB0_WAKEUP_4500	(PRCM_REQ_MB0 + 0x8)

/* Mailbox 0 ACKs */
#define PRCM_ACK_MB0_AP_PWRSTTR_STATUS	(PRCM_ACK_MB0 + 0x0)
#define PRCM_ACK_MB0_READ_POINTER	(PRCM_ACK_MB0 + 0x1)
#define PRCM_ACK_MB0_WAKEUP_0_8500	(PRCM_ACK_MB0 + 0x4)
#define PRCM_ACK_MB0_WAKEUP_0_4500	(PRCM_ACK_MB0 + 0x8)
#define PRCM_ACK_MB0_WAKEUP_1_8500	(PRCM_ACK_MB0 + 0x1C)
#define PRCM_ACK_MB0_WAKEUP_1_4500	(PRCM_ACK_MB0 + 0x20)
#define PRCM_ACK_MB0_EVENT_4500_NUMBERS	20

/* Mailbox 1 headers */
#define MB1H_ARM_APE_OPP 0x0
#define MB1H_RESET_MODEM 0x2
#define MB1H_REQUEST_APE_OPP_100_VOLT 0x3
#define MB1H_RELEASE_APE_OPP_100_VOLT 0x4
#define MB1H_RELEASE_USB_WAKEUP 0x5
#define MB1H_PLL_ON_OFF 0x6

/* Mailbox 1 Requests */
#define PRCM_REQ_MB1_ARM_OPP			(PRCM_REQ_MB1 + 0x0)
#define PRCM_REQ_MB1_APE_OPP			(PRCM_REQ_MB1 + 0x1)
#define PRCM_REQ_MB1_APE_OPP_100_RESTORE	(PRCM_REQ_MB1 + 0x4)
#define PRCM_REQ_MB1_ARM_OPP_100_RESTORE	(PRCM_REQ_MB1 + 0x8)
#define PRCM_REQ_MB1_PLL_ON_OFF			(PRCM_REQ_MB1 + 0x4)
#define PLL_SOC1_OFF	0x4
#define PLL_SOC1_ON	0x8

/* Mailbox 1 ACKs */
#define PRCM_ACK_MB1_CURRENT_ARM_OPP	(PRCM_ACK_MB1 + 0x0)
#define PRCM_ACK_MB1_CURRENT_APE_OPP	(PRCM_ACK_MB1 + 0x1)
#define PRCM_ACK_MB1_APE_VOLTAGE_STATUS	(PRCM_ACK_MB1 + 0x2)
#define PRCM_ACK_MB1_DVFS_STATUS	(PRCM_ACK_MB1 + 0x3)

/* Mailbox 2 headers */
#define MB2H_DPS	0x0
#define MB2H_AUTO_PWR	0x1

/* Mailbox 2 REQs */
#define PRCM_REQ_MB2_SVA_MMDSP		(PRCM_REQ_MB2 + 0x0)
#define PRCM_REQ_MB2_SVA_PIPE		(PRCM_REQ_MB2 + 0x1)
#define PRCM_REQ_MB2_SIA_MMDSP		(PRCM_REQ_MB2 + 0x2)
#define PRCM_REQ_MB2_SIA_PIPE		(PRCM_REQ_MB2 + 0x3)
#define PRCM_REQ_MB2_SGA		(PRCM_REQ_MB2 + 0x4)
#define PRCM_REQ_MB2_B2R2_MCDE		(PRCM_REQ_MB2 + 0x5)
#define PRCM_REQ_MB2_ESRAM12		(PRCM_REQ_MB2 + 0x6)
#define PRCM_REQ_MB2_ESRAM34		(PRCM_REQ_MB2 + 0x7)
#define PRCM_REQ_MB2_AUTO_PM_SLEEP	(PRCM_REQ_MB2 + 0x8)
#define PRCM_REQ_MB2_AUTO_PM_IDLE	(PRCM_REQ_MB2 + 0xC)

/* Mailbox 2 ACKs */
#define PRCM_ACK_MB2_DPS_STATUS (PRCM_ACK_MB2 + 0x0)
#define HWACC_PWR_ST_OK 0xFE

/* Mailbox 3 headers */
#define MB3H_ANC	0x0
#define MB3H_SIDETONE	0x1
#define MB3H_SYSCLK	0xE

/* Mailbox 3 Requests */
#define PRCM_REQ_MB3_ANC_FIR_COEFF	(PRCM_REQ_MB3 + 0x0)
#define PRCM_REQ_MB3_ANC_IIR_COEFF	(PRCM_REQ_MB3 + 0x20)
#define PRCM_REQ_MB3_ANC_SHIFTER	(PRCM_REQ_MB3 + 0x60)
#define PRCM_REQ_MB3_ANC_WARP		(PRCM_REQ_MB3 + 0x64)
#define PRCM_REQ_MB3_SIDETONE_FIR_GAIN	(PRCM_REQ_MB3 + 0x68)
#define PRCM_REQ_MB3_SIDETONE_FIR_COEFF	(PRCM_REQ_MB3 + 0x6C)
#define PRCM_REQ_MB3_SYSCLK_MGT		(PRCM_REQ_MB3 + 0x16C)

/* Mailbox 4 headers */
#define MB4H_DDR_INIT	0x0
#define MB4H_MEM_ST	0x1
#define MB4H_HOTDOG	0x12
#define MB4H_HOTMON	0x13
#define MB4H_HOT_PERIOD	0x14

/* Mailbox 4 Requests */
#define PRCM_REQ_MB4_DDR_ST_AP_SLEEP_IDLE	(PRCM_REQ_MB4 + 0x0)
#define PRCM_REQ_MB4_DDR_ST_AP_DEEP_IDLE	(PRCM_REQ_MB4 + 0x1)
#define PRCM_REQ_MB4_ESRAM0_ST			(PRCM_REQ_MB4 + 0x3)
#define PRCM_REQ_MB4_HOTDOG_THRESHOLD		(PRCM_REQ_MB4 + 0x0)
#define PRCM_REQ_MB4_HOTMON_LOW			(PRCM_REQ_MB4 + 0x0)
#define PRCM_REQ_MB4_HOTMON_HIGH		(PRCM_REQ_MB4 + 0x1)
#define PRCM_REQ_MB4_HOTMON_CONFIG		(PRCM_REQ_MB4 + 0x2)
#define PRCM_REQ_MB4_HOT_PERIOD			(PRCM_REQ_MB4 + 0x0)
#define HOTMON_CONFIG_LOW			BIT(0)
#define HOTMON_CONFIG_HIGH			BIT(1)

/* Mailbox 5 Requests */
#define PRCM_REQ_MB5_I2C_SLAVE_OP	(PRCM_REQ_MB5 + 0x0)
#define PRCM_REQ_MB5_I2C_HW_BITS	(PRCM_REQ_MB5 + 0x1)
#define PRCM_REQ_MB5_I2C_REG		(PRCM_REQ_MB5 + 0x2)
#define PRCM_REQ_MB5_I2C_VAL		(PRCM_REQ_MB5 + 0x3)
#define PRCMU_I2C_WRITE(slave) \
	(((slave) << 1) | (cpu_is_u8500v2() ? BIT(6) : 0))
#define PRCMU_I2C_READ(slave) \
	(((slave) << 1) | BIT(0) | (cpu_is_u8500v2() ? BIT(6) : 0))
#define PRCMU_I2C_STOP_EN		BIT(3)

/* Mailbox 5 ACKs */
#define PRCM_ACK_MB5_I2C_STATUS	(PRCM_ACK_MB5 + 0x1)
#define PRCM_ACK_MB5_I2C_VAL	(PRCM_ACK_MB5 + 0x3)
#define I2C_WR_OK 0x1
#define I2C_RD_OK 0x2

#define NUM_MB 8
#define MBOX_BIT BIT
#define ALL_MBOX_BITS (MBOX_BIT(NUM_MB) - 1)

/*
 * Wakeups/IRQs
 */

#define WAKEUP_BIT_RTC BIT(0)
#define WAKEUP_BIT_RTT0 BIT(1)
#define WAKEUP_BIT_RTT1 BIT(2)
#define WAKEUP_BIT_HSI0 BIT(3)
#define WAKEUP_BIT_HSI1 BIT(4)
#define WAKEUP_BIT_CA_WAKE BIT(5)
#define WAKEUP_BIT_USB BIT(6)
#define WAKEUP_BIT_ABB BIT(7)
#define WAKEUP_BIT_ABB_FIFO BIT(8)
#define WAKEUP_BIT_SYSCLK_OK BIT(9)
#define WAKEUP_BIT_CA_SLEEP BIT(10)
#define WAKEUP_BIT_AC_WAKE_ACK BIT(11)
#define WAKEUP_BIT_SIDE_TONE_OK BIT(12)
#define WAKEUP_BIT_ANC_OK BIT(13)
#define WAKEUP_BIT_SW_ERROR BIT(14)
#define WAKEUP_BIT_AC_SLEEP_ACK BIT(15)
#define WAKEUP_BIT_ARM BIT(17)
#define WAKEUP_BIT_HOTMON_LOW BIT(18)
#define WAKEUP_BIT_HOTMON_HIGH BIT(19)
#define WAKEUP_BIT_MODEM_SW_RESET_REQ BIT(20)
#define WAKEUP_BIT_GPIO0 BIT(23)
#define WAKEUP_BIT_GPIO1 BIT(24)
#define WAKEUP_BIT_GPIO2 BIT(25)
#define WAKEUP_BIT_GPIO3 BIT(26)
#define WAKEUP_BIT_GPIO4 BIT(27)
#define WAKEUP_BIT_GPIO5 BIT(28)
#define WAKEUP_BIT_GPIO6 BIT(29)
#define WAKEUP_BIT_GPIO7 BIT(30)
#define WAKEUP_BIT_GPIO8 BIT(31)

void log_this(u8 pc, char* a, u32 extra1, char* b, u32 extra2);

/*
 * This vector maps irq numbers to the bits in the bit field used in
 * communication with the PRCMU firmware.
 *
 * The reason for having this is to keep the irq numbers contiguous even though
 * the bits in the bit field are not. (The bits also have a tendency to move
 * around, to further complicate matters.)
 */
#define IRQ_INDEX(_name) ((IRQ_PRCMU_##_name) - IRQ_PRCMU_BASE)
#define IRQ_ENTRY(_name)[IRQ_INDEX(_name)] = (WAKEUP_BIT_##_name)
static u32 prcmu_irq_bit[NUM_PRCMU_WAKEUPS] = {
	IRQ_ENTRY(RTC),
	IRQ_ENTRY(RTT0),
	IRQ_ENTRY(RTT1),
	IRQ_ENTRY(HSI0),
	IRQ_ENTRY(HSI1),
	IRQ_ENTRY(CA_WAKE),
	IRQ_ENTRY(USB),
	IRQ_ENTRY(ABB),
	IRQ_ENTRY(ABB_FIFO),
	IRQ_ENTRY(CA_SLEEP),
	IRQ_ENTRY(ARM),
	IRQ_ENTRY(HOTMON_LOW),
	IRQ_ENTRY(HOTMON_HIGH),
	IRQ_ENTRY(MODEM_SW_RESET_REQ),
	IRQ_ENTRY(GPIO0),
	IRQ_ENTRY(GPIO1),
	IRQ_ENTRY(GPIO2),
	IRQ_ENTRY(GPIO3),
	IRQ_ENTRY(GPIO4),
	IRQ_ENTRY(GPIO5),
	IRQ_ENTRY(GPIO6),
	IRQ_ENTRY(GPIO7),
	IRQ_ENTRY(GPIO8)
};

#define VALID_WAKEUPS (BIT(NUM_PRCMU_WAKEUP_INDICES) - 1)
#define WAKEUP_ENTRY(_name)[PRCMU_WAKEUP_INDEX_##_name] = (WAKEUP_BIT_##_name)
static u32 prcmu_wakeup_bit[NUM_PRCMU_WAKEUP_INDICES] = {
	WAKEUP_ENTRY(RTC),
	WAKEUP_ENTRY(RTT0),
	WAKEUP_ENTRY(RTT1),
	WAKEUP_ENTRY(HSI0),
	WAKEUP_ENTRY(HSI1),
	WAKEUP_ENTRY(USB),
	WAKEUP_ENTRY(ABB),
	WAKEUP_ENTRY(ABB_FIFO),
	WAKEUP_ENTRY(ARM)
};

/*
 * mb0_transfer - state needed for mailbox 0 communication.
 * @lock:		The transaction lock.
 * @dbb_events_lock:	A lock used to handle concurrent access to (parts of)
 *			the request data.
 * @mask_work:		Work structure used for (un)masking wakeup interrupts.
 * @req:		Request data that need to persist between requests.
 */
static struct {
	spinlock_t lock;
	spinlock_t dbb_irqs_lock;
	struct work_struct mask_work;
	struct mutex ac_wake_lock;
	struct completion ac_wake_work;
	struct {
		u32 dbb_irqs;
		u32 dbb_wakeups;
		u32 abb_events;
	} req;
} mb0_transfer;

/*
 * mb1_transfer - state needed for mailbox 1 communication.
 * @lock:	The transaction lock.
 * @work:	The transaction completion structure.
 * @ape_opp:	 The current APE OPP.
 * @ack:	Reply ("acknowledge") data.
 */
static struct {
	struct mutex lock;
	struct completion work;
	u8 ape_opp;
	struct {
		u8 header;
		u8 arm_opp;
		u8 ape_opp;
		u8 ape_voltage_status;
	} ack;
} mb1_transfer;

/*
 * mb2_transfer - state needed for mailbox 2 communication.
 * @lock:            The transaction lock.
 * @work:            The transaction completion structure.
 * @auto_pm_lock:    The autonomous power management configuration lock.
 * @auto_pm_enabled: A flag indicating whether autonomous PM is enabled.
 * @req:             Request data that need to persist between requests.
 * @ack:             Reply ("acknowledge") data.
 */
static struct {
	struct mutex lock;
	struct completion work;
	spinlock_t auto_pm_lock;
	bool auto_pm_enabled;
	struct {
		u8 status;
	} ack;
} mb2_transfer;

/*
 * mb3_transfer - state needed for mailbox 3 communication.
 * @lock:		The request lock.
 * @sysclk_lock:	A lock used to handle concurrent sysclk requests.
 * @sysclk_work:	Work structure used for sysclk requests.
 */
static struct {
	spinlock_t lock;
	struct mutex sysclk_lock;
	struct completion sysclk_work;
} mb3_transfer;

/*
 * mb4_transfer - state needed for mailbox 4 communication.
 * @lock:	The transaction lock.
 * @work:	The transaction completion structure.
 */
static struct {
	struct mutex lock;
	struct completion work;
} mb4_transfer;

/*
 * mb5_transfer - state needed for mailbox 5 communication.
 * @lock:	The transaction lock.
 * @work:	The transaction completion structure.
 * @ack:	Reply ("acknowledge") data.
 */
static struct {
	struct mutex lock;
	struct completion work;
	struct {
		u8 status;
		u8 value;
	} ack;
} mb5_transfer;

static atomic_t ac_wake_req_state = ATOMIC_INIT(0);

/* Spinlocks */
static DEFINE_SPINLOCK(clkout_lock);
static DEFINE_SPINLOCK(gpiocr_lock);

/* Global var to runtime determine TCDM base for v2 or v1 */
static __iomem void *tcdm_base;

struct clk_mgt {
	unsigned int offset;
	u32 pllsw;
};

static DEFINE_SPINLOCK(clk_mgt_lock);

#define CLK_MGT_ENTRY(_name)[PRCMU_##_name] = { (PRCM_##_name##_MGT), 0 }
struct clk_mgt clk_mgt[PRCMU_NUM_REG_CLOCKS] = {
	CLK_MGT_ENTRY(SGACLK),
	CLK_MGT_ENTRY(UARTCLK),
	CLK_MGT_ENTRY(MSP02CLK),
	CLK_MGT_ENTRY(MSP1CLK),
	CLK_MGT_ENTRY(I2CCLK),
	CLK_MGT_ENTRY(SDMMCCLK),
	CLK_MGT_ENTRY(SLIMCLK),
	CLK_MGT_ENTRY(PER1CLK),
	CLK_MGT_ENTRY(PER2CLK),
	CLK_MGT_ENTRY(PER3CLK),
	CLK_MGT_ENTRY(PER5CLK),
	CLK_MGT_ENTRY(PER6CLK),
	CLK_MGT_ENTRY(PER7CLK),
	CLK_MGT_ENTRY(LCDCLK),
	CLK_MGT_ENTRY(BMLCLK),
	CLK_MGT_ENTRY(HSITXCLK),
	CLK_MGT_ENTRY(HSIRXCLK),
	CLK_MGT_ENTRY(HDMICLK),
	CLK_MGT_ENTRY(APEATCLK),
	CLK_MGT_ENTRY(APETRACECLK),
	CLK_MGT_ENTRY(MCDECLK),
	CLK_MGT_ENTRY(IPI2CCLK),
	CLK_MGT_ENTRY(DSIALTCLK),
	CLK_MGT_ENTRY(DMACLK),
	CLK_MGT_ENTRY(B2R2CLK),
	CLK_MGT_ENTRY(TVCLK),
	CLK_MGT_ENTRY(SSPCLK),
	CLK_MGT_ENTRY(RNGCLK),
	CLK_MGT_ENTRY(UICCCLK),
};

/*
 * NOTE! Temporary until all users of set_hwacc() are using the regulator
 * framework API
 */
static struct regulator *hwacc_regulator[NUM_HW_ACC];
static struct regulator *hwacc_ret_regulator[NUM_HW_ACC];

static bool hwacc_enabled[NUM_HW_ACC];
static bool hwacc_ret_enabled[NUM_HW_ACC];

static const char *hwacc_regulator_name[NUM_HW_ACC] = {
	[HW_ACC_SVAMMDSP]	= "hwacc-sva-mmdsp",
	[HW_ACC_SVAPIPE]	= "hwacc-sva-pipe",
	[HW_ACC_SIAMMDSP]	= "hwacc-sia-mmdsp",
	[HW_ACC_SIAPIPE]	= "hwacc-sia-pipe",
	[HW_ACC_SGA]		= "hwacc-sga",
	[HW_ACC_B2R2]		= "hwacc-b2r2",
	[HW_ACC_MCDE]		= "hwacc-mcde",
	[HW_ACC_ESRAM1]		= "hwacc-esram1",
	[HW_ACC_ESRAM2]		= "hwacc-esram2",
	[HW_ACC_ESRAM3]		= "hwacc-esram3",
	[HW_ACC_ESRAM4]		= "hwacc-esram4",
};

static const char *hwacc_ret_regulator_name[NUM_HW_ACC] = {
	[HW_ACC_SVAMMDSP]	= "hwacc-sva-mmdsp-ret",
	[HW_ACC_SIAMMDSP]	= "hwacc-sia-mmdsp-ret",
	[HW_ACC_ESRAM1]		= "hwacc-esram1-ret",
	[HW_ACC_ESRAM2]		= "hwacc-esram2-ret",
	[HW_ACC_ESRAM3]		= "hwacc-esram3-ret",
	[HW_ACC_ESRAM4]		= "hwacc-esram4-ret",
};

/*
* Used by MCDE to setup all necessary PRCMU registers
*/
#define PRCMU_RESET_DSIPLL		0x00004000
#define PRCMU_UNCLAMP_DSIPLL		0x00400800

#define PRCMU_CLK_PLL_DIV_SHIFT		0
#define PRCMU_CLK_PLL_SW_SHIFT		5
#define PRCMU_CLK_EN			BIT(8)
#define PRCMU_CLK_38			(1 << 9)
#define PRCMU_CLK_38_SRC		(1 << 10)
#define PRCMU_CLK_38_DIV		(1 << 11)

/* PLLDIV=12, PLLSW=4 (PLLDDR) */
#define PRCMU_DSI_CLOCK_SETTING		0x0000008C

#if defined(CONFIG_MACH_JANICE) || defined(CONFIG_MACH_CODINA)
#define PRCMU_DPI_CLOCK_SETTING		((4 << PRCMU_CLK_PLL_SW_SHIFT) | \
					  (8 << PRCMU_CLK_PLL_DIV_SHIFT))
#else
/* DPI 80000000 Hz */
#define PRCMU_DPI_CLOCK_SETTING		((4 << PRCMU_CLK_PLL_SW_SHIFT) | \
					  (5 << PRCMU_CLK_PLL_DIV_SHIFT))
#endif
#define PRCMU_DSI_LP_CLOCK_SETTING	0x00000E00

/* D=101, N=1, R=4, SELDIV2=0 */
#define PRCMU_PLLDSI_FREQ_SETTING	0x00040165

#define PRCMU_ENABLE_PLLDSI		0x00000001
#define PRCMU_DISABLE_PLLDSI		0x00000000
#define PRCMU_RELEASE_RESET_DSS		0x0000400C
#define PRCMU_DSI_PLLOUT_SEL_SETTING	0x00000202
/* ESC clk, div0=1, div1=1, div2=3 */
#define PRCMU_ENABLE_ESCAPE_CLOCK_DIV	0x07030101
#define PRCMU_DISABLE_ESCAPE_CLOCK_DIV	0x00030101
#define PRCMU_DSI_RESET_SW		0x00000007

#define PRCMU_PLLDSI_LOCKP_LOCKED	0x3

static struct {
	u8 project_number;
	u8 api_version;
	u8 func_version;
	u8 errata;
} prcmu_version;

struct wake_lock prcmu_uart_wake_lock;
extern void ux500_ci_dbg_console(void);

static int request_timclk(bool enable);

int prcmu_enable_dsipll(void)
{
	int i;
	unsigned int plldsifreq;

	/* Clear DSIPLL_RESETN */
	writel(PRCMU_RESET_DSIPLL, (_PRCMU_BASE + PRCM_APE_RESETN_CLR));
	/* Unclamp DSIPLL in/out */
	writel(PRCMU_UNCLAMP_DSIPLL, (_PRCMU_BASE + PRCM_MMIP_LS_CLAMP_CLR));

	plldsifreq = PRCMU_PLLDSI_FREQ_SETTING;
	/* Set DSI PLL FREQ */
	writel(plldsifreq, (_PRCMU_BASE + PRCM_PLLDSI_FREQ));
	writel(PRCMU_DSI_PLLOUT_SEL_SETTING,
		(_PRCMU_BASE + PRCM_DSI_PLLOUT_SEL));
	/* Enable Escape clocks */
	writel(PRCMU_ENABLE_ESCAPE_CLOCK_DIV,
					(_PRCMU_BASE + PRCM_DSITVCLK_DIV));

	/* Start DSI PLL */
	writel(PRCMU_ENABLE_PLLDSI, (_PRCMU_BASE + PRCM_PLLDSI_ENABLE));
	/* Reset DSI PLL */
	writel(PRCMU_DSI_RESET_SW, (_PRCMU_BASE + PRCM_DSI_SW_RESET));
	for (i = 0; i < 10; i++) {
		if ((readl(_PRCMU_BASE + PRCM_PLLDSI_LOCKP) &
			PRCMU_PLLDSI_LOCKP_LOCKED)
					== PRCMU_PLLDSI_LOCKP_LOCKED)
			break;
		udelay(100);
	}
	/* Set DSIPLL_RESETN */
	writel(PRCMU_RESET_DSIPLL, (_PRCMU_BASE + PRCM_APE_RESETN_SET));
	return 0;
}

int prcmu_disable_dsipll(void)
{
	/* Disable dsi pll */
	writel(PRCMU_DISABLE_PLLDSI, (_PRCMU_BASE + PRCM_PLLDSI_ENABLE));
	/* Disable  escapeclock */
	writel(PRCMU_DISABLE_ESCAPE_CLOCK_DIV,
					(_PRCMU_BASE + PRCM_DSITVCLK_DIV));
	return 0;
}

int prcmu_set_display_clocks(void)
{
	unsigned long flags;
	unsigned int dsiclk;
	u32 reg;

	dsiclk = PRCMU_DSI_CLOCK_SETTING;

	spin_lock_irqsave(&clk_mgt_lock, flags);

	/* Grab the HW semaphore. */
	while ((readl(_PRCMU_BASE + PRCM_SEM) & PRCM_SEM_PRCM_SEM) != 0)
		cpu_relax();

	writel(dsiclk, (_PRCMU_BASE + PRCM_HDMICLK_MGT));
	writel(PRCMU_DSI_LP_CLOCK_SETTING, (_PRCMU_BASE + PRCM_TVCLK_MGT));
#ifdef CONFIG_MCDE_DISPLAY_DPI_MAIN
	/* Keep the LCDCLK boot state to preserve the boot splash screen. */
	reg = readl(_PRCMU_BASE + PRCM_LCDCLK_MGT) & PRCMU_CLK_EN;
	writel(PRCMU_DPI_CLOCK_SETTING | reg, (_PRCMU_BASE + PRCM_LCDCLK_MGT));
#else
	writel(PRCMU_DPI_CLOCK_SETTING, (_PRCMU_BASE + PRCM_LCDCLK_MGT));
#endif

	/* Release the HW semaphore. */
	writel(0, (_PRCMU_BASE + PRCM_SEM));

	spin_unlock_irqrestore(&clk_mgt_lock, flags);

	return 0;
}

/**
 * prcmu_enable_spi2 - Enables pin muxing for SPI2 on OtherAlternateC1.
 */
void prcmu_enable_spi2(void)
{
	u32 reg;
	unsigned long flags;

	spin_lock_irqsave(&gpiocr_lock, flags);
	reg = readl(_PRCMU_BASE + PRCM_GPIOCR);
	writel(reg | PRCM_GPIOCR_SPI2_SELECT, _PRCMU_BASE + PRCM_GPIOCR);
	spin_unlock_irqrestore(&gpiocr_lock, flags);
}

/**
 * prcmu_disable_spi2 - Disables pin muxing for SPI2 on OtherAlternateC1.
 */
void prcmu_disable_spi2(void)
{
	u32 reg;
	unsigned long flags;

	spin_lock_irqsave(&gpiocr_lock, flags);
	reg = readl(_PRCMU_BASE + PRCM_GPIOCR);
	writel(reg & ~PRCM_GPIOCR_SPI2_SELECT, _PRCMU_BASE + PRCM_GPIOCR);
	spin_unlock_irqrestore(&gpiocr_lock, flags);
}

bool prcmu_has_arm_maxopp(void)
{
	return (readb(tcdm_base + PRCM_AVS_VARM_MAX_OPP) &
		PRCM_AVS_ISMODEENABLE_MASK) == PRCM_AVS_ISMODEENABLE_MASK;
}

/**
 * prcmu_get_boot_status - PRCMU boot status checking
 * Returns: the current PRCMU boot status
 */
int prcmu_get_boot_status(void)
{
	return readb(tcdm_base + PRCM_BOOT_STATUS);
}

/**
 * prcmu_set_rc_a2p - This function is used to run few power state sequences
 * @val: Value to be set, i.e. transition requested
 * Returns: 0 on success, -EINVAL on invalid argument
 *
 * This function is used to run the following power state sequences -
 * any state to ApReset,  ApDeepSleep to ApExecute, ApExecute to ApDeepSleep
 */
int prcmu_set_rc_a2p(enum romcode_write val)
{
	if (val < RDY_2_DS || val > RDY_2_XP70_RST)
		return -EINVAL;
	writeb(val, (tcdm_base + PRCM_ROMCODE_A2P));
	return 0;
}

/**
 * prcmu_get_rc_p2a - This function is used to get power state sequences
 * Returns: the power transition that has last happened
 *
 * This function can return the following transitions-
 * any state to ApReset,  ApDeepSleep to ApExecute, ApExecute to ApDeepSleep
 */
enum romcode_read prcmu_get_rc_p2a(void)
{
	return readb(tcdm_base + PRCM_ROMCODE_P2A);
}

/**
 * prcmu_get_current_mode - Return the current XP70 power mode
 * Returns: Returns the current AP(ARM) power mode: init,
 * apBoot, apExecute, apDeepSleep, apSleep, apIdle, apReset
 */
enum ap_pwrst prcmu_get_xp70_current_state(void)
{
	return readb(tcdm_base + PRCM_XP70_CUR_PWR_STATE);
}

/**
 * prcmu_config_clkout - Configure one of the programmable clock outputs.
 * @clkout:	The CLKOUT number (0 or 1).
 * @source:	The clock to be used (one of the PRCMU_CLKSRC_*).
 * @div:	The divider to be applied.
 *
 * Configures one of the programmable clock outputs (CLKOUTs).
 * @div should be in the range [1,63] to request a configuration, or 0 to
 * inform that the configuration is no longer requested.
 */
int prcmu_config_clkout(u8 clkout, u8 source, u8 div)
{
	static int requests[2];
	int r = 0;
	unsigned long flags;
	u32 val;
	u32 bits;
	u32 mask;
	u32 div_mask;

	BUG_ON(clkout > 1);
	BUG_ON(div > 63);
	BUG_ON((clkout == 0) && (source > PRCMU_CLKSRC_CLK009));

	if (!div && !requests[clkout])
		return -EINVAL;

	switch (clkout) {
	case 0:
		div_mask = PRCM_CLKOCR_CLKODIV0_MASK;
		mask = (PRCM_CLKOCR_CLKODIV0_MASK | PRCM_CLKOCR_CLKOSEL0_MASK);
		bits = ((source << PRCM_CLKOCR_CLKOSEL0_SHIFT) |
			(div << PRCM_CLKOCR_CLKODIV0_SHIFT));
		break;
	case 1:
		div_mask = PRCM_CLKOCR_CLKODIV1_MASK;
		mask = (PRCM_CLKOCR_CLKODIV1_MASK | PRCM_CLKOCR_CLKOSEL1_MASK |
			PRCM_CLKOCR_CLK1TYPE);
		bits = ((source << PRCM_CLKOCR_CLKOSEL1_SHIFT) |
			(div << PRCM_CLKOCR_CLKODIV1_SHIFT));
		break;
	}
	bits &= mask;

	spin_lock_irqsave(&clkout_lock, flags);

	val = readl(_PRCMU_BASE + PRCM_CLKOCR);
	if (val & div_mask) {
		if (div) {
			if ((val & mask) != bits) {
				r = -EBUSY;
				goto unlock_and_return;
			}
		} else {
			if ((val & mask & ~div_mask) != bits) {
				r = -EINVAL;
				goto unlock_and_return;
			}
		}
	}
	writel((bits | (val & ~mask)), (_PRCMU_BASE + PRCM_CLKOCR));
	requests[clkout] += (div ? 1 : -1);

unlock_and_return:
	spin_unlock_irqrestore(&clkout_lock, flags);

	return r;
}

int prcmu_set_power_state(u8 state, bool keep_ulp_clk, bool keep_ap_pll)
{
	unsigned long flags;

	BUG_ON((state < PRCMU_AP_SLEEP) || (PRCMU_AP_DEEP_IDLE < state));

	spin_lock_irqsave(&mb0_transfer.lock, flags);

	while (readl(_PRCMU_BASE + PRCM_MBOX_CPU_VAL) & MBOX_BIT(0))
		cpu_relax();

	writeb(MB0H_POWER_STATE_TRANS, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB0));
	writeb(state, (tcdm_base + PRCM_REQ_MB0_AP_POWER_STATE));
	writeb((keep_ap_pll ? 1 : 0), (tcdm_base + PRCM_REQ_MB0_AP_PLL_STATE));
	writeb((keep_ulp_clk ? 1 : 0),
		(tcdm_base + PRCM_REQ_MB0_ULP_CLOCK_STATE));
	writeb(0, (tcdm_base + PRCM_REQ_MB0_DO_NOT_WFI));
	log_this(100, "state", state, NULL, 0);
	writel(MBOX_BIT(0), (_PRCMU_BASE + PRCM_MBOX_CPU_SET));

	spin_unlock_irqrestore(&mb0_transfer.lock, flags);

	return 0;
}

/* This function should only be called while mb0_transfer.lock is held. */
static void config_wakeups(void)
{
	const u8 header[2] = {
		MB0H_CONFIG_WAKEUPS_EXE,
		MB0H_CONFIG_WAKEUPS_SLEEP
	};
	static u32 last_dbb_events;
	static u32 last_abb_events;
	u32 dbb_events;
	u32 abb_events;
	unsigned int i;

	dbb_events = mb0_transfer.req.dbb_irqs | mb0_transfer.req.dbb_wakeups;
	dbb_events |= (WAKEUP_BIT_AC_WAKE_ACK | WAKEUP_BIT_AC_SLEEP_ACK);

	abb_events = mb0_transfer.req.abb_events;

	if ((dbb_events == last_dbb_events) && (abb_events == last_abb_events))
		return;

	for (i = 0; i < 2; i++) {
		while (readl(_PRCMU_BASE + PRCM_MBOX_CPU_VAL) & MBOX_BIT(0))
			cpu_relax();
		writel(dbb_events, (tcdm_base + PRCM_REQ_MB0_WAKEUP_8500));
		writel(abb_events, (tcdm_base + PRCM_REQ_MB0_WAKEUP_4500));
		writeb(header[i], (tcdm_base + PRCM_MBOX_HEADER_REQ_MB0));
		if (i == 0)
			log_this(110, "dbb", dbb_events, "abb", abb_events);
		else
			log_this(111, "ddb", dbb_events, "abb", abb_events);
		writel(MBOX_BIT(0), (_PRCMU_BASE + PRCM_MBOX_CPU_SET));
	}
	last_dbb_events = dbb_events;
	last_abb_events = abb_events;
}

void prcmu_enable_wakeups(u32 wakeups)
{
	unsigned long flags;
	u32 bits;
	int i;

	BUG_ON(wakeups != (wakeups & VALID_WAKEUPS));

	for (i = 0, bits = 0; i < NUM_PRCMU_WAKEUP_INDICES; i++) {
		if (wakeups & BIT(i))
			bits |= prcmu_wakeup_bit[i];
	}

	spin_lock_irqsave(&mb0_transfer.lock, flags);

	mb0_transfer.req.dbb_wakeups = bits;
	config_wakeups();

	spin_unlock_irqrestore(&mb0_transfer.lock, flags);
}

void prcmu_config_abb_event_readout(u32 abb_events)
{
	unsigned long flags;

	spin_lock_irqsave(&mb0_transfer.lock, flags);

	mb0_transfer.req.abb_events = abb_events;
	config_wakeups();

	spin_unlock_irqrestore(&mb0_transfer.lock, flags);
}

void prcmu_get_abb_event_buffer(void __iomem **buf)
{
	if (readb(tcdm_base + PRCM_ACK_MB0_READ_POINTER) & 1)
		*buf = (tcdm_base + PRCM_ACK_MB0_WAKEUP_1_4500);
	else
		*buf = (tcdm_base + PRCM_ACK_MB0_WAKEUP_0_4500);
}

/**
 * prcmu_set_arm_opp - set the appropriate ARM OPP
 * @opp: The new ARM operating point to which transition is to be made
 * Returns: 0 on success, non-zero on failure
 *
 * This function sets the the operating point of the ARM.
 */
int prcmu_set_arm_opp(u8 opp)
{
	int r;

	if (opp < ARM_NO_CHANGE || opp > ARM_EXTCLK)
		return -EINVAL;

	r = 0;

	mutex_lock(&mb1_transfer.lock);

	while (readl(_PRCMU_BASE + PRCM_MBOX_CPU_VAL) & MBOX_BIT(1))
		cpu_relax();

	writeb(MB1H_ARM_APE_OPP, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB1));
	writeb(opp, (tcdm_base + PRCM_REQ_MB1_ARM_OPP));
	writeb(APE_NO_CHANGE, (tcdm_base + PRCM_REQ_MB1_APE_OPP));

	log_this(120, "OPP", opp, NULL, 0);
	writel(MBOX_BIT(1), (_PRCMU_BASE + PRCM_MBOX_CPU_SET));
	wait_for_completion(&mb1_transfer.work);

	if ((mb1_transfer.ack.header != MB1H_ARM_APE_OPP) ||
		(mb1_transfer.ack.arm_opp != opp))
		r = -EIO;

	mutex_unlock(&mb1_transfer.lock);

	return r;
}

/**
 * prcmu_get_arm_opp - get the current ARM OPP
 *
 * Returns: the current ARM OPP
 */
int prcmu_get_arm_opp(void)
{
	return readb(tcdm_base + PRCM_ACK_MB1_CURRENT_ARM_OPP);
}

/**
 * prcmu_get_ddr_opp - get the current DDR OPP
 *
 * Returns: the current DDR OPP
 */
int prcmu_get_ddr_opp(void)
{
	return readb(_PRCMU_BASE + PRCM_DDR_SUBSYS_APE_MINBW);
}

/**
 * set_ddr_opp - set the appropriate DDR OPP
 * @opp: The new DDR operating point to which transition is to be made
 * Returns: 0 on success, non-zero on failure
 *
 * This function sets the operating point of the DDR.
 */
int prcmu_set_ddr_opp(u8 opp)
{
	if (opp < DDR_100_OPP || opp > DDR_25_OPP)
		return -EINVAL;
	/* Changing the DDR OPP can hang the hardware pre-v21 */
	if (cpu_is_u8500v20_or_later() && !cpu_is_u8500v20())
		writeb(opp, (_PRCMU_BASE + PRCM_DDR_SUBSYS_APE_MINBW));

	return 0;
}

/* Divide the frequency of certain clocks by 2 for APE_50_PARTLY_25_OPP. */
static void request_even_slower_clocks(bool enable)
{
	const u8 clock_reg[] = {
		PRCM_ACLK_MGT,
		PRCM_DMACLK_MGT
	};
	unsigned long flags;
	unsigned int i;

	spin_lock_irqsave(&clk_mgt_lock, flags);

	/* Grab the HW semaphore. */
	while ((readl(_PRCMU_BASE + PRCM_SEM) & PRCM_SEM_PRCM_SEM) != 0)
		cpu_relax();

	for (i = 0; i < ARRAY_SIZE(clock_reg); i++) {
		u32 val;
		u32 div;

		val = readl(_PRCMU_BASE + clock_reg[i]);
		div = (val & PRCM_CLK_MGT_CLKPLLDIV_MASK);
		if (enable) {
			if ((div <= 1) || (div > 15)) {
				pr_err("prcmu: Bad clock divider %d in %s\n",
					div, __func__);
				goto unlock_and_return;
			}
			div <<= 1;
		} else {
			if (div <= 2)
				goto unlock_and_return;
			div >>= 1;
		}
		val = ((val & ~PRCM_CLK_MGT_CLKPLLDIV_MASK) |
			(div & PRCM_CLK_MGT_CLKPLLDIV_MASK));
		writel(val, (_PRCMU_BASE + clock_reg[i]));
	}

unlock_and_return:
	/* Release the HW semaphore. */
	writel(0, (_PRCMU_BASE + PRCM_SEM));

	spin_unlock_irqrestore(&clk_mgt_lock, flags);
}

/**
 * set_ape_opp - set the appropriate APE OPP
 * @opp: The new APE operating point to which transition is to be made
 * Returns: 0 on success, non-zero on failure
 *
 * This function sets the operating point of the APE.
 */
int prcmu_set_ape_opp(u8 opp)
{
	int r = 0;

	if (opp == mb1_transfer.ape_opp)
		return 0;

	mutex_lock(&mb1_transfer.lock);

	if (mb1_transfer.ape_opp == APE_50_PARTLY_25_OPP)
		request_even_slower_clocks(false);

	if ((opp != APE_100_OPP) && (mb1_transfer.ape_opp != APE_100_OPP))
		goto skip_message;

	while (readl(_PRCMU_BASE + PRCM_MBOX_CPU_VAL) & MBOX_BIT(1))
		cpu_relax();

	writeb(MB1H_ARM_APE_OPP, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB1));
	writeb(ARM_NO_CHANGE, (tcdm_base + PRCM_REQ_MB1_ARM_OPP));
	writeb(((opp == APE_50_PARTLY_25_OPP) ? APE_50_OPP : opp),
		(tcdm_base + PRCM_REQ_MB1_APE_OPP));

	log_this(130, "OPP", opp, NULL, 0);
	writel(MBOX_BIT(1), (_PRCMU_BASE + PRCM_MBOX_CPU_SET));
	wait_for_completion(&mb1_transfer.work);

	if ((mb1_transfer.ack.header != MB1H_ARM_APE_OPP) ||
		(mb1_transfer.ack.ape_opp != opp))
		r = -EIO;

skip_message:
	if ((!r && (opp == APE_50_PARTLY_25_OPP)) ||
		(r && (mb1_transfer.ape_opp == APE_50_PARTLY_25_OPP)))
		request_even_slower_clocks(true);
	if (!r)
		mb1_transfer.ape_opp = opp;

	mutex_unlock(&mb1_transfer.lock);

	return r;
}

/**
 * prcmu_get_ape_opp - get the current APE OPP
 *
 * Returns: the current APE OPP
 */
int prcmu_get_ape_opp(void)
{
	return readb(tcdm_base + PRCM_ACK_MB1_CURRENT_APE_OPP);
}

/**
 * prcmu_request_ape_opp_100_voltage - Request APE OPP 100% voltage
 * @enable: true to request the higher voltage, false to drop a request.
 *
 * Calls to this function to enable and disable requests must be balanced.
 */
int prcmu_request_ape_opp_100_voltage(bool enable)
{
	int r = 0;
	u8 header;
	static unsigned int requests;

	mutex_lock(&mb1_transfer.lock);

	if (enable) {
		if (0 != requests++)
			goto unlock_and_return;
		header = MB1H_REQUEST_APE_OPP_100_VOLT;
	} else {
		if (requests == 0) {
			r = -EIO;
			goto unlock_and_return;
		} else if (1 != requests--) {
			goto unlock_and_return;
		}
		header = MB1H_RELEASE_APE_OPP_100_VOLT;
	}

	while (readl(_PRCMU_BASE + PRCM_MBOX_CPU_VAL) & MBOX_BIT(1))
		cpu_relax();

	writeb(header, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB1));

	log_this(140, "enable", enable, NULL, 0);
	writel(MBOX_BIT(1), (_PRCMU_BASE + PRCM_MBOX_CPU_SET));
	wait_for_completion(&mb1_transfer.work);

	if ((mb1_transfer.ack.header != header) ||
		((mb1_transfer.ack.ape_voltage_status & BIT(0)) != 0))
		r = -EIO;

unlock_and_return:
	mutex_unlock(&mb1_transfer.lock);

	return r;
}

/**
 * prcmu_release_usb_wakeup_state - release the state required by a USB wakeup
 *
 * This function releases the power state requirements of a USB wakeup.
 */
int prcmu_release_usb_wakeup_state(void)
{
	int r = 0;

	mutex_lock(&mb1_transfer.lock);

	while (readl(_PRCMU_BASE + PRCM_MBOX_CPU_VAL) & MBOX_BIT(1))
		cpu_relax();

	writeb(MB1H_RELEASE_USB_WAKEUP,
		(tcdm_base + PRCM_MBOX_HEADER_REQ_MB1));

	log_this(150, NULL, 0, NULL, 0);
	writel(MBOX_BIT(1), (_PRCMU_BASE + PRCM_MBOX_CPU_SET));
	wait_for_completion(&mb1_transfer.work);

	if ((mb1_transfer.ack.header != MB1H_RELEASE_USB_WAKEUP) ||
		((mb1_transfer.ack.ape_voltage_status & BIT(0)) != 0))
		r = -EIO;

	mutex_unlock(&mb1_transfer.lock);

	return r;
}

static int request_pll(u8 clock, bool enable)
{
	int r = 0;

	if (clock == PRCMU_PLLSOC1)
		clock = (enable ? PLL_SOC1_ON : PLL_SOC1_OFF);
	else
		return -EINVAL;

	mutex_lock(&mb1_transfer.lock);

	while (readl(_PRCMU_BASE + PRCM_MBOX_CPU_VAL) & MBOX_BIT(1))
		cpu_relax();

	writeb(MB1H_PLL_ON_OFF, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB1));
	writeb(clock, (tcdm_base + PRCM_REQ_MB1_PLL_ON_OFF));

	log_this(160, "clock", clock, "enable", enable);
	writel(MBOX_BIT(1), (_PRCMU_BASE + PRCM_MBOX_CPU_SET));
	wait_for_completion(&mb1_transfer.work);

	if (mb1_transfer.ack.header != MB1H_PLL_ON_OFF)
		r = -EIO;

	mutex_unlock(&mb1_transfer.lock);

	return r;
}

/**
 * prcmu_set_hwacc - set the power state of a h/w accelerator
 * @hwacc_dev: The hardware accelerator (enum hw_acc_dev).
 * @state: The new power state (enum hw_acc_state).
 *
 * This function sets the power state of a hardware accelerator.
 * This function should not be called from interrupt context.
 *
 * NOTE! Deprecated, to be removed when all users switched over to use the
 * regulator framework API.
 */
int prcmu_set_hwacc(u16 hwacc_dev, u8 state)
{
	int r = 0;
	bool ram_retention = false;
	bool enable, enable_ret;

	/* check argument */
	BUG_ON(hwacc_dev >= NUM_HW_ACC);

	/* get state of switches */
	enable = hwacc_enabled[hwacc_dev];
	enable_ret = hwacc_ret_enabled[hwacc_dev];

	/* set flag if retention is possible */
	switch (hwacc_dev) {
	case HW_ACC_SVAMMDSP:
	case HW_ACC_SIAMMDSP:
	case HW_ACC_ESRAM1:
	case HW_ACC_ESRAM2:
	case HW_ACC_ESRAM3:
	case HW_ACC_ESRAM4:
		ram_retention = true;
		break;
	}

	/* check argument */
	BUG_ON(state > HW_ON);
	BUG_ON(state == HW_OFF_RAMRET && !ram_retention);

	/* modify enable flags */
	switch (state) {
	case HW_OFF:
		enable_ret = false;
		enable = false;
		break;
	case HW_ON:
		enable = true;
		break;
	case HW_OFF_RAMRET:
		enable_ret = true;
		enable = false;
		break;
	}

	/* get regulator (lazy) */
	if (hwacc_regulator[hwacc_dev] == NULL) {
		hwacc_regulator[hwacc_dev] = regulator_get(NULL,
			hwacc_regulator_name[hwacc_dev]);
		if (IS_ERR(hwacc_regulator[hwacc_dev])) {
			pr_err("prcmu: failed to get supply %s\n",
				hwacc_regulator_name[hwacc_dev]);
			r = PTR_ERR(hwacc_regulator[hwacc_dev]);
			goto out;
		}
	}

	if (ram_retention) {
		if (hwacc_ret_regulator[hwacc_dev] == NULL) {
			hwacc_ret_regulator[hwacc_dev] = regulator_get(NULL,
				hwacc_ret_regulator_name[hwacc_dev]);
			if (IS_ERR(hwacc_ret_regulator[hwacc_dev])) {
				pr_err("prcmu: failed to get supply %s\n",
					hwacc_ret_regulator_name[hwacc_dev]);
				r = PTR_ERR(hwacc_ret_regulator[hwacc_dev]);
				goto out;
			}
		}
	}

	/* set regulators */
	if (ram_retention) {
		if (enable_ret && !hwacc_ret_enabled[hwacc_dev]) {
			r = regulator_enable(hwacc_ret_regulator[hwacc_dev]);
			if (r < 0) {
				pr_err("prcmu_set_hwacc: ret enable failed\n");
				goto out;
			}
			hwacc_ret_enabled[hwacc_dev] = true;
		}
	}

	if (enable && !hwacc_enabled[hwacc_dev]) {
		r = regulator_enable(hwacc_regulator[hwacc_dev]);
		if (r < 0) {
			pr_err("prcmu_set_hwacc: enable failed\n");
			goto out;
		}
		hwacc_enabled[hwacc_dev] = true;
	}

	if (!enable && hwacc_enabled[hwacc_dev]) {
		r = regulator_disable(hwacc_regulator[hwacc_dev]);
		if (r < 0) {
			pr_err("prcmu_set_hwacc: disable failed\n");
			goto out;
		}
		hwacc_enabled[hwacc_dev] = false;
	}

	if (ram_retention) {
		if (!enable_ret && hwacc_ret_enabled[hwacc_dev]) {
			r = regulator_disable(hwacc_ret_regulator[hwacc_dev]);
			if (r < 0) {
				pr_err("prcmu_set_hwacc: ret disable failed\n");
				goto out;
			}
			hwacc_ret_enabled[hwacc_dev] = false;
		}
	}

out:
	return r;
}
EXPORT_SYMBOL(prcmu_set_hwacc);

/**
 * prcmu_set_epod - set the state of a EPOD (power domain)
 * @epod_id: The EPOD to set
 * @epod_state: The new EPOD state
 *
 * This function sets the state of a EPOD (power domain). It may not be called
 * from interrupt context.
 */
int prcmu_set_epod(u16 epod_id, u8 epod_state)
{
	int r = 0;
	bool ram_retention = false;
	int i;

	/* check argument */
	BUG_ON(epod_id >= NUM_EPOD_ID);

	/* set flag if retention is possible */
	switch (epod_id) {
	case EPOD_ID_SVAMMDSP:
	case EPOD_ID_SIAMMDSP:
	case EPOD_ID_ESRAM12:
	case EPOD_ID_ESRAM34:
		ram_retention = true;
		break;
	}

	/* check argument */
	BUG_ON(epod_state > EPOD_STATE_ON);
	BUG_ON(epod_state == EPOD_STATE_RAMRET && !ram_retention);

	/* get lock */
	mutex_lock(&mb2_transfer.lock);

	/* wait for mailbox */
	while (readl(_PRCMU_BASE + PRCM_MBOX_CPU_VAL) & MBOX_BIT(2))
		cpu_relax();

	/* fill in mailbox */
	for (i = 0; i < NUM_EPOD_ID; i++)
		writeb(EPOD_STATE_NO_CHANGE, (tcdm_base + PRCM_REQ_MB2 + i));
	writeb(epod_state, (tcdm_base + PRCM_REQ_MB2 + epod_id));

	writeb(MB2H_DPS, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB2));

	log_this(170, "epod_id", epod_id, "state", epod_state);
	writel(MBOX_BIT(2), (_PRCMU_BASE + PRCM_MBOX_CPU_SET));
	/*
	 * The current firmware version does not handle errors correctly,
	 * and we cannot recover if there is an error.
	 * This is expected to change when the firmware is updated.
	 */
	if (!wait_for_completion_timeout(&mb2_transfer.work,
			msecs_to_jiffies(20000))) {
		pr_err("prcmu: %s timed out (20 s) waiting for a reply.\n",
			__func__);
		r = -EIO;
		goto unlock_and_return;
	}
	log_this(171, "epod_id", epod_id, "done", epod_state);
	if (mb2_transfer.ack.status != HWACC_PWR_ST_OK)
		r = -EIO;

unlock_and_return:
	mutex_unlock(&mb2_transfer.lock);
	return r;
}

#ifdef CONFIG_SAMSUNG_PANIC_DISPLAY_DEVICES
/**
 * prcmu_set_epod - set the state of a EPOD (power domain) called in Kernel Panic state.
 * @epod_id: The EPOD to set
 * @epod_state: The new EPOD state
 *
 * This function sets the state of a EPOD (power domain). It may not be called
 * from interrupt context.
 */
int prcmu_panic_set_epod(u16 epod_id, u8 epod_state)
{
	int r = 0;
	bool ram_retention = false;
	int i;
	u32 timeout = PRCMU_I2C_TIMEOUT;
	u8 status;

	/* check argument */
	BUG_ON(epod_id >= NUM_EPOD_ID);

	/* set flag if retention is possible */
	switch (epod_id) {
	case EPOD_ID_SVAMMDSP:
	case EPOD_ID_SIAMMDSP:
	case EPOD_ID_ESRAM12:
	case EPOD_ID_ESRAM34:
		ram_retention = true;
		break;
	}

	/* check argument */
	BUG_ON(epod_state > EPOD_STATE_ON);
	BUG_ON(epod_state == EPOD_STATE_RAMRET && !ram_retention);

	if (readl(_PRCMU_BASE + PRCM_ARM_IT1_VAL) & MBOX_BIT(2)) {
		/* clear mailbox 2 ack irq */
		writel(MBOX_BIT(2), _PRCMU_BASE + PRCM_ARM_IT1_CLR);
	}

	while ((readl(_PRCMU_BASE + PRCM_MBOX_CPU_VAL) & MBOX_BIT(2)) && timeout--)
		cpu_relax();

	if ( timeout == 0 ){
		pr_emerg("%s: timed out waiting for MBOX2 to be free.\n",__func__);
		return -EIO;
	}

	/* fill in mailbox */
	for (i = 0; i < NUM_EPOD_ID; i++)
		writeb(EPOD_STATE_NO_CHANGE, (tcdm_base + PRCM_REQ_MB2 + i));
	writeb(epod_state, (tcdm_base + PRCM_REQ_MB2 + epod_id));

	writeb(MB2H_DPS, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB2));

	writel(MBOX_BIT(2), (_PRCMU_BASE + PRCM_MBOX_CPU_SET));

	/*
	 * The current firmware version does not handle errors correctly,
	 * and we cannot recover if there is an error.
	 * This is expected to change when the firmware is updated.
	 */
	timeout = PRCMU_I2C_TIMEOUT;

	while (!(readl(_PRCMU_BASE + PRCM_ARM_IT1_VAL) & MBOX_BIT(2)) && timeout--)
		cpu_relax();

	if (!timeout) {
		pr_emerg("%s timed out waiting for a reply.\n",__func__);
		return -EIO;
	}

	status = readb(tcdm_base + PRCM_ACK_MB2_DPS_STATUS);

	/* clear mailbox 2 ack irq */
	writel(MBOX_BIT(2), _PRCMU_BASE + PRCM_ARM_IT1_CLR);

	r = ((status == HWACC_PWR_ST_OK) ? 0 : -EIO);

	return r;
}
#endif

/**
 * prcmu_configure_auto_pm - Configure autonomous power management.
 * @sleep: Configuration for ApSleep.
 * @idle:  Configuration for ApIdle.
 */
void prcmu_configure_auto_pm(struct prcmu_auto_pm_config *sleep,
	struct prcmu_auto_pm_config *idle)
{
	u32 sleep_cfg;
	u32 idle_cfg;
	unsigned long flags;

	BUG_ON((sleep == NULL) || (idle == NULL));

	sleep_cfg = (sleep->sva_auto_pm_enable & 0xF);
	sleep_cfg = ((sleep_cfg << 4) | (sleep->sia_auto_pm_enable & 0xF));
	sleep_cfg = ((sleep_cfg << 8) | (sleep->sva_power_on & 0xFF));
	sleep_cfg = ((sleep_cfg << 8) | (sleep->sia_power_on & 0xFF));
	sleep_cfg = ((sleep_cfg << 4) | (sleep->sva_policy & 0xF));
	sleep_cfg = ((sleep_cfg << 4) | (sleep->sia_policy & 0xF));

	idle_cfg = (idle->sva_auto_pm_enable & 0xF);
	idle_cfg = ((idle_cfg << 4) | (idle->sia_auto_pm_enable & 0xF));
	idle_cfg = ((idle_cfg << 8) | (idle->sva_power_on & 0xFF));
	idle_cfg = ((idle_cfg << 8) | (idle->sia_power_on & 0xFF));
	idle_cfg = ((idle_cfg << 4) | (idle->sva_policy & 0xF));
	idle_cfg = ((idle_cfg << 4) | (idle->sia_policy & 0xF));

	spin_lock_irqsave(&mb2_transfer.auto_pm_lock, flags);

	/*
	 * The autonomous power management configuration is done through
	 * fields in mailbox 2, but these fields are only used as shared
	 * variables - i.e. there is no need to send a message.
	 */
	log_this(112, "sleep_cfg", sleep_cfg, NULL, 0);
	writel(sleep_cfg, (tcdm_base + PRCM_REQ_MB2_AUTO_PM_SLEEP));
	log_this(112, "idle_cfg", idle_cfg, NULL, 0);
	writel(idle_cfg, (tcdm_base + PRCM_REQ_MB2_AUTO_PM_IDLE));

	mb2_transfer.auto_pm_enabled =
		((sleep->sva_auto_pm_enable == PRCMU_AUTO_PM_ON) ||
		 (sleep->sia_auto_pm_enable == PRCMU_AUTO_PM_ON) ||
		 (idle->sva_auto_pm_enable == PRCMU_AUTO_PM_ON) ||
		 (idle->sia_auto_pm_enable == PRCMU_AUTO_PM_ON));
	log_this(112, "enabled", mb2_transfer.auto_pm_enabled, NULL, 0);
	spin_unlock_irqrestore(&mb2_transfer.auto_pm_lock, flags);
}
EXPORT_SYMBOL(prcmu_configure_auto_pm);

bool prcmu_is_auto_pm_enabled(void)
{
	return mb2_transfer.auto_pm_enabled;
}

#ifdef CONFIG_SAMSUNG_PANIC_DISPLAY_DEVICES
static int request_panic_pll(u8 clock, bool enable)
{
	int r = 0;
	u32 timeout = PRCMU_I2C_TIMEOUT;
	u8 status;

	if (clock == PRCMU_PLLSOC1)
		clock = (enable ? PLL_SOC1_ON : PLL_SOC1_OFF);
	else
		return -EINVAL;

	if (readl(_PRCMU_BASE + PRCM_ARM_IT1_VAL) & MBOX_BIT(1)) {
		/* clear mailbox 1 ack irq */
		writel(MBOX_BIT(1), _PRCMU_BASE + PRCM_ARM_IT1_CLR);
	}

	while ((readl(_PRCMU_BASE + PRCM_MBOX_CPU_VAL) & MBOX_BIT(1)) && timeout--)
		cpu_relax();

	if ( 0 == timeout ){
		pr_emerg("%s: timed out waiting for MBOX1 to be free.\n",__func__);
		return -EIO;
	}

	writeb(MB1H_PLL_ON_OFF, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB1));
	writeb(clock, (tcdm_base + PRCM_REQ_MB1_PLL_ON_OFF));

	writel(MBOX_BIT(1), (_PRCMU_BASE + PRCM_MBOX_CPU_SET));

	timeout = PRCMU_I2C_TIMEOUT;

	while (!(readl(_PRCMU_BASE + PRCM_ARM_IT1_VAL) & MBOX_BIT(1)) && timeout--)
		cpu_relax();

	if (0 == timeout) {
		pr_emerg("%s timed out waiting for a reply.\n",__func__);
		return -EIO;
	}

	status = readb(tcdm_base + PRCM_MBOX_HEADER_REQ_MB1);

	/* clear mailbox 1 ack irq */
	writel(MBOX_BIT(1), _PRCMU_BASE + PRCM_ARM_IT1_CLR);

	r = ((status == MB1H_PLL_ON_OFF) ? 0 : -EIO);

	return r;
}

static int request_panic_sysclk(bool enable)
{
	int r;
	u32 timeout = PRCMU_I2C_TIMEOUT;

	r = 0;

	if (readl(_PRCMU_BASE + PRCM_ARM_IT1_VAL) & MBOX_BIT(3)) {
		/* clear mailbox 3 ack irq */
		writel(MBOX_BIT(3), _PRCMU_BASE + PRCM_ARM_IT1_CLR);
	}

	while ((readl(_PRCMU_BASE + PRCM_MBOX_CPU_VAL) & MBOX_BIT(3)) && timeout--)
		cpu_relax();

	if ( 0 == timeout ){
		pr_emerg("%s: timed out waiting for MBOX3 to be free.\n",__func__);
		return -EIO;
	}

	writeb((enable ? 1 : 0), (tcdm_base + PRCM_REQ_MB3_SYSCLK_MGT));

	writeb(MB3H_SYSCLK, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB3));
	writel(MBOX_BIT(3), (_PRCMU_BASE + PRCM_MBOX_CPU_SET));

	/*
	 * The firmware only sends an ACK if we want to enable the
	 * SysClk, and it succeeds.
	 */
	if (enable){
		u32 ev;
		u8 header;

		timeout = PRCMU_I2C_TIMEOUT;
		
		while ((readl(_PRCMU_BASE + PRCM_MBOX_CPU_VAL) & MBOX_BIT(3)) 
				&& timeout--)
			cpu_relax();

		if ( 0 == timeout ){
			pr_emerg("%s: timed out waiting for MBOX3.\n",__func__);
			return -EIO;
		}
		
		timeout = PRCMU_I2C_TIMEOUT;
		
		while ((readl(_PRCMU_BASE + PRCM_MBOX_CPU_VAL) & MBOX_BIT(0)) 
				&& timeout--)
			cpu_relax();

		if ( timeout == 0){
			pr_err("%s timed out waiting for MBOX0.\n",
				__func__);
			return -EIO;
		}

		header = readb(tcdm_base + PRCM_MBOX_HEADER_ACK_MB0);
		switch (header) {
		case MB0H_WAKEUP_EXE:
		case MB0H_WAKEUP_SLEEP:
			if (readb(tcdm_base + PRCM_ACK_MB0_READ_POINTER) & 1)
				ev = readl(tcdm_base + PRCM_ACK_MB0_WAKEUP_1_8500);
			else
				ev = readl(tcdm_base + PRCM_ACK_MB0_WAKEUP_0_8500);
		
			if ( !(ev & WAKEUP_BIT_SYSCLK_OK))
				r = -EIO;
		}

		writel(MBOX_BIT(3), (_PRCMU_BASE + PRCM_ARM_IT1_CLR));
		writel(MBOX_BIT(0), (_PRCMU_BASE + PRCM_ARM_IT1_CLR));
	}

	return r;
}

static int request_panic_reg_clock(u8 clock, bool enable)
{
	u32 val;

	val = readl(_PRCMU_BASE + clk_mgt[clock].offset);
	if (enable) {
		val |= (PRCM_CLK_MGT_CLKEN | clk_mgt[clock].pllsw);
	} else {
		clk_mgt[clock].pllsw = (val & PRCM_CLK_MGT_CLKPLLSW_MASK);
		val &= ~(PRCM_CLK_MGT_CLKEN | PRCM_CLK_MGT_CLKPLLSW_MASK);
	}
	writel(val, (_PRCMU_BASE + clk_mgt[clock].offset));

	return 0;
}

static int request_panic_sga_clock(u8 clock, bool enable)
{
	u32 val;
	int ret;

	if (enable && cpu_is_u8500v20()) {
		val = readl(_PRCMU_BASE + PRCM_CGATING_BYPASS);
		writel(val | PRCM_CGATING_BYPASS_ICN2,
				_PRCMU_BASE + PRCM_CGATING_BYPASS);
	}

	ret = request_panic_reg_clock(clock, enable);

	if (!ret && !enable && cpu_is_u8500v20()) {
		val = readl(_PRCMU_BASE + PRCM_CGATING_BYPASS);
		writel(val & ~PRCM_CGATING_BYPASS_ICN2,
				_PRCMU_BASE + PRCM_CGATING_BYPASS);
	}

	return ret;
}

/**
 * prcmu_request_clock() - Request for a clock to be enabled or disabled.
 * @clock:      The clock for which the request is made.
 * @enable:     Whether the clock should be enabled (true) or disabled (false).
 *
 * This function should only be used by the clock implementation.
 * Do not use it from any other place!
 */
int prcmu_panic_request_clock(u8 clock, bool enable)
{
	if (clock == PRCMU_SGACLK)
		return request_panic_sga_clock(clock, enable);
	else if (clock < PRCMU_NUM_REG_CLOCKS)
		return request_panic_reg_clock(clock, enable);
	else if (clock == PRCMU_TIMCLK)
		return request_timclk(enable);
	else if (clock == PRCMU_SYSCLK)
		return request_panic_sysclk(enable);
	else if (clock == PRCMU_PLLSOC1)
		return request_panic_pll(clock, enable);
	else
		return -EINVAL;
}
#endif

static int request_sysclk(bool enable)
{
	int r;
	unsigned long flags;

	r = 0;

	mutex_lock(&mb3_transfer.sysclk_lock);

	spin_lock_irqsave(&mb3_transfer.lock, flags);

	while (readl(_PRCMU_BASE + PRCM_MBOX_CPU_VAL) & MBOX_BIT(3))
		cpu_relax();

	writeb((enable ? 1 : 0), (tcdm_base + PRCM_REQ_MB3_SYSCLK_MGT));

	writeb(MB3H_SYSCLK, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB3));
	log_this(180, "enable", enable, NULL, 0);
	writel(MBOX_BIT(3), (_PRCMU_BASE + PRCM_MBOX_CPU_SET));

	spin_unlock_irqrestore(&mb3_transfer.lock, flags);

	/*
	 * The firmware only sends an ACK if we want to enable the
	 * SysClk, and it succeeds.
	 */
	if (enable && !wait_for_completion_timeout(&mb3_transfer.sysclk_work,
			msecs_to_jiffies(20000))) {
		pr_err("prcmu: %s timed out (20 s) waiting for a reply.\n",
			__func__);
		r = -EIO;
	}

	mutex_unlock(&mb3_transfer.sysclk_lock);

	return r;
}

static int request_timclk(bool enable)
{
	u32 val = (PRCM_TCR_DOZE_MODE | PRCM_TCR_TENSEL_MASK);

	if (!enable)
		val |= PRCM_TCR_STOP_TIMERS;
	writel(val, (_PRCMU_BASE + PRCM_TCR));

	return 0;
}

static int request_reg_clock(u8 clock, bool enable)
{
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&clk_mgt_lock, flags);

	/* Grab the HW semaphore. */
	while ((readl(_PRCMU_BASE + PRCM_SEM) & PRCM_SEM_PRCM_SEM) != 0)
		cpu_relax();

	val = readl(_PRCMU_BASE + clk_mgt[clock].offset);
	if (enable) {
		val |= (PRCM_CLK_MGT_CLKEN | clk_mgt[clock].pllsw);
	} else {
		clk_mgt[clock].pllsw = (val & PRCM_CLK_MGT_CLKPLLSW_MASK);
		val &= ~(PRCM_CLK_MGT_CLKEN | PRCM_CLK_MGT_CLKPLLSW_MASK);
	}
	log_this(181, "clock", clock, "enable", enable);
	writel(val, (_PRCMU_BASE + clk_mgt[clock].offset));

	/* Release the HW semaphore. */
	writel(0, (_PRCMU_BASE + PRCM_SEM));
	log_this(182, "written", val, "status", readl(_PRCMU_BASE + clk_mgt[clock].offset));
	spin_unlock_irqrestore(&clk_mgt_lock, flags);

	return 0;
}

static int request_sga_clock(u8 clock, bool enable)
{
	u32 val;
	int ret;

	if (enable && cpu_is_u8500v20()) {
		val = readl(_PRCMU_BASE + PRCM_CGATING_BYPASS);
		writel(val | PRCM_CGATING_BYPASS_ICN2,
				_PRCMU_BASE + PRCM_CGATING_BYPASS);
	}

	ret = request_reg_clock(clock, enable);

	if (!ret && !enable && cpu_is_u8500v20()) {
		val = readl(_PRCMU_BASE + PRCM_CGATING_BYPASS);
		writel(val & ~PRCM_CGATING_BYPASS_ICN2,
				_PRCMU_BASE + PRCM_CGATING_BYPASS);
	}

	return ret;
}

/*
 * This is a workaround for prcmu firmware bug. While settings EPODs, prcmu
 * firmware enables/disables B2R2 clock. However, this is done without taking
 * PRCM_SEM. Concurrent accesses from APE side to change state of EPODs and
 * B2R2 clock can result in B2R2 clock in unwanted state.
 */
static int request_b2r2_clock(u8 clock, bool enable)
{
	int ret;
	mutex_lock(&mb2_transfer.lock);
	ret = request_reg_clock(clock, enable);
	mutex_unlock(&mb2_transfer.lock);
	return ret;
}

/**
 * prcmu_request_clock() - Request for a clock to be enabled or disabled.
 * @clock:      The clock for which the request is made.
 * @enable:     Whether the clock should be enabled (true) or disabled (false).
 *
 * This function should only be used by the clock implementation.
 * Do not use it from any other place!
 */
int prcmu_request_clock(u8 clock, bool enable)
{
	if (clock == PRCMU_SGACLK)
		return request_sga_clock(clock, enable);
	else if (clock == PRCMU_B2R2CLK)
		return request_b2r2_clock(clock, enable);
	else if (clock < PRCMU_NUM_REG_CLOCKS)
		return request_reg_clock(clock, enable);
	else if (clock == PRCMU_TIMCLK)
		return request_timclk(enable);
	else if (clock == PRCMU_SYSCLK)
		return request_sysclk(enable);
	else if (clock == PRCMU_PLLSOC1)
		return request_pll(clock, enable);
	else
		return -EINVAL;
}

int prcmu_config_esram0_deep_sleep(u8 state)
{
	if ((state > ESRAM0_DEEP_SLEEP_STATE_RET) ||
	    (state < ESRAM0_DEEP_SLEEP_STATE_OFF))
		return -EINVAL;

	mutex_lock(&mb4_transfer.lock);

	while (readl(_PRCMU_BASE + PRCM_MBOX_CPU_VAL) & MBOX_BIT(4))
		cpu_relax();

	writeb(MB4H_MEM_ST, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB4));
	writeb(((DDR_PWR_STATE_OFFHIGHLAT << 4) | DDR_PWR_STATE_ON),
	       (tcdm_base + PRCM_REQ_MB4_DDR_ST_AP_SLEEP_IDLE));
	writeb(DDR_PWR_STATE_ON,
	       (tcdm_base + PRCM_REQ_MB4_DDR_ST_AP_DEEP_IDLE));
	writeb(state, (tcdm_base + PRCM_REQ_MB4_ESRAM0_ST));

	log_this(190, "state", state, NULL, 0);
	writel(MBOX_BIT(4), (_PRCMU_BASE + PRCM_MBOX_CPU_SET));
	wait_for_completion(&mb4_transfer.work);

	mutex_unlock(&mb4_transfer.lock);

	return 0;
}

int prcmu_config_hotdog(u8 threshold)
{
	mutex_lock(&mb4_transfer.lock);

	while (readl(_PRCMU_BASE + PRCM_MBOX_CPU_VAL) & MBOX_BIT(4))
		cpu_relax();

	writeb(threshold, (tcdm_base + PRCM_REQ_MB4_HOTDOG_THRESHOLD));
	writeb(MB4H_HOTDOG, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB4));

	log_this(200, "threshold", threshold, NULL, 0);
	writel(MBOX_BIT(4), (_PRCMU_BASE + PRCM_MBOX_CPU_SET));
	wait_for_completion(&mb4_transfer.work);

	mutex_unlock(&mb4_transfer.lock);

	return 0;
}

int prcmu_config_hotmon(u8 low, u8 high)
{
	mutex_lock(&mb4_transfer.lock);

	while (readl(_PRCMU_BASE + PRCM_MBOX_CPU_VAL) & MBOX_BIT(4))
		cpu_relax();

	writeb(low, (tcdm_base + PRCM_REQ_MB4_HOTMON_LOW));
	writeb(high, (tcdm_base + PRCM_REQ_MB4_HOTMON_HIGH));
	writeb((HOTMON_CONFIG_LOW | HOTMON_CONFIG_HIGH),
		(tcdm_base + PRCM_REQ_MB4_HOTMON_CONFIG));
	writeb(MB4H_HOTMON, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB4));

	log_this(210, "low", low, "high", high);
	writel(MBOX_BIT(4), (_PRCMU_BASE + PRCM_MBOX_CPU_SET));
	wait_for_completion(&mb4_transfer.work);

	mutex_unlock(&mb4_transfer.lock);

	return 0;
}

static int config_hot_period(u16 val)
{
	mutex_lock(&mb4_transfer.lock);

	while (readl(_PRCMU_BASE + PRCM_MBOX_CPU_VAL) & MBOX_BIT(4))
		cpu_relax();

	writew(val, (tcdm_base + PRCM_REQ_MB4_HOT_PERIOD));
	writeb(MB4H_HOT_PERIOD, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB4));

	log_this(220, "val", val, NULL, 0);
	writel(MBOX_BIT(4), (_PRCMU_BASE + PRCM_MBOX_CPU_SET));
	wait_for_completion(&mb4_transfer.work);

	mutex_unlock(&mb4_transfer.lock);

	return 0;
}

int prcmu_start_temp_sense(u16 cycles32k)
{
	if (cycles32k == 0xFFFF)
		return -EINVAL;

	return config_hot_period(cycles32k);
}

int prcmu_stop_temp_sense(void)
{
	return config_hot_period(0xFFFF);
}

/**
 * prcmu_set_clock_divider() - Configure the clock divider.
 * @clock:	The clock for which the request is made.
 * @divider:	The clock divider. (< 32)
 *
 * This function should only be used by the clock implementation.
 * Do not use it from any other place!
 */
int prcmu_set_clock_divider(u8 clock, u8 divider)
{
	u32 val;
	unsigned long flags;

	if ((clock >= PRCMU_NUM_REG_CLOCKS) || (divider < 1) || (31 < divider))
		return -EINVAL;

	spin_lock_irqsave(&clk_mgt_lock, flags);

	/* Grab the HW semaphore. */
	while ((readl(_PRCMU_BASE + PRCM_SEM) & PRCM_SEM_PRCM_SEM) != 0)
		cpu_relax();

	val = readl(_PRCMU_BASE + clk_mgt[clock].offset);
	val &= ~(PRCM_CLK_MGT_CLKPLLDIV_MASK);
	val |= (u32)divider;
	writel(val, (_PRCMU_BASE + clk_mgt[clock].offset));

	/* Release the HW semaphore. */
	writel(0, (_PRCMU_BASE + PRCM_SEM));

	spin_unlock_irqrestore(&clk_mgt_lock, flags);

	return 0;
}

/**
 * prcmu_abb_read() - Read register value(s) from the ABB.
 * @slave:	The I2C slave address.
 * @reg:	The (start) register address.
 * @value:	The read out value(s).
 * @size:	The number of registers to read.
 *
 * Reads register value(s) from the ABB.
 * @size has to be 1 for the current firmware version.
 */
int prcmu_abb_read(u8 slave, u8 reg, u8 *value, u8 size)
{
	int r;

	if (size != 1)
		return -EINVAL;

	mutex_lock(&mb5_transfer.lock);

	while (readl(_PRCMU_BASE + PRCM_MBOX_CPU_VAL) & MBOX_BIT(5))
		cpu_relax();

	writeb(PRCMU_I2C_READ(slave), (tcdm_base + PRCM_REQ_MB5_I2C_SLAVE_OP));
	writeb(PRCMU_I2C_STOP_EN, (tcdm_base + PRCM_REQ_MB5_I2C_HW_BITS));
	writeb(reg, (tcdm_base + PRCM_REQ_MB5_I2C_REG));
	writeb(0, (tcdm_base + PRCM_REQ_MB5_I2C_VAL));

	log_this(230, "reg", slave << 8 | reg, NULL, 0);
	writel(MBOX_BIT(5), (_PRCMU_BASE + PRCM_MBOX_CPU_SET));

	if (!wait_for_completion_timeout(&mb5_transfer.work,
				msecs_to_jiffies(20000))) {
		pr_err("prcmu: %s timed out (20 s) waiting for a reply.\n",
			__func__);
		r = -EIO;
	} else {
		r = ((mb5_transfer.ack.status == I2C_RD_OK) ? 0 : -EIO);
	}

	if (!r)
		*value = mb5_transfer.ack.value;

	log_this(231, "value", *value, NULL, 0);
	mutex_unlock(&mb5_transfer.lock);

	return r;
}

/**
 * prcmu_abb_write() - Write register value(s) to the ABB.
 * @slave:	The I2C slave address.
 * @reg:	The (start) register address.
 * @value:	The value(s) to write.
 * @size:	The number of registers to write.
 *
 * Reads register value(s) from the ABB.
 * @size has to be 1 for the current firmware version.
 */
int prcmu_abb_write(u8 slave, u8 reg, u8 *value, u8 size)
{
	int r;

	if (size != 1)
		return -EINVAL;

	mutex_lock(&mb5_transfer.lock);

	while (readl(_PRCMU_BASE + PRCM_MBOX_CPU_VAL) & MBOX_BIT(5))
		cpu_relax();

	writeb(PRCMU_I2C_WRITE(slave), (tcdm_base + PRCM_REQ_MB5_I2C_SLAVE_OP));
	writeb(PRCMU_I2C_STOP_EN, (tcdm_base + PRCM_REQ_MB5_I2C_HW_BITS));
	writeb(reg, (tcdm_base + PRCM_REQ_MB5_I2C_REG));
	writeb(*value, (tcdm_base + PRCM_REQ_MB5_I2C_VAL));

	log_this(240, "reg", slave << 8 | reg, "value", *value);
	writel(MBOX_BIT(5), (_PRCMU_BASE + PRCM_MBOX_CPU_SET));

	if (!wait_for_completion_timeout(&mb5_transfer.work,
				msecs_to_jiffies(20000))) {
		pr_err("prcmu: %s timed out (20 s) waiting for a reply.\n",
			__func__);
		r = -EIO;
	} else {
		r = ((mb5_transfer.ack.status == I2C_WR_OK) ? 0 : -EIO);
	}

	mutex_unlock(&mb5_transfer.lock);

	return r;
}

struct log_entry {
	u64 cpu_clk;
	u32 extra1;
	u32 extra2;
	u8 pc;
	char* a;
	char* b;
};

/* log on non-cacheable buffer */
#ifdef CONFIG_SAMSUNG_LOG_BUF
#include <mach/board-sec-u8500.h>
static DEFINE_SPINLOCK(log_lock);
#define A_LOG_SIZE (6398)
static int log_idx = -1;
static int log_active = 1;
static struct log_entry *a_log;

void* log_buf_prcmu;
EXPORT_SYMBOL(log_buf_prcmu);
const int log_buf_prcmu_entry_size = sizeof(struct log_entry);
EXPORT_SYMBOL(log_buf_prcmu_entry_size);
const int log_buf_prcmu_entry_count = A_LOG_SIZE; 
EXPORT_SYMBOL(log_buf_prcmu_entry_count);
#endif

void log_stop(void)
{
#ifdef CONFIG_SAMSUNG_LOG_BUF
        log_active = 0;
#endif
}
EXPORT_SYMBOL(log_stop);

void log_this(u8 pc, char* a, u32 extra1, char* b, u32 extra2)
{
#ifdef CONFIG_SAMSUNG_LOG_BUF
	u64 cpu_clk = cpu_clock(0);
	unsigned long flags;

	if (!log_active)
		return;

	spin_lock_irqsave(&log_lock, flags);

	if (a_log) {
		log_idx++;

		if ((unsigned int)log_idx >= A_LOG_SIZE)
			log_idx = 0;

		a_log[log_idx].cpu_clk = cpu_clk;
		a_log[log_idx].pc = pc;
		a_log[log_idx].a = a;
		a_log[log_idx].b = b;
		a_log[log_idx].extra1 = extra1;
		a_log[log_idx].extra2 = extra2;

	} else if (log_buf_prcmu) {
		a_log = (struct log_entry *)log_buf_prcmu;
	}

	spin_unlock_irqrestore(&log_lock, flags);
#endif
}
EXPORT_SYMBOL(log_this);

static char* convert_pc_to_text(u8 pc)
{
	switch (pc) {
		case 10: return "READ_MAILBOX_0";
		case 11: return "READ_MAILBOX_0*";
		case 40: return "GOP_CA_WAKE_ACK_BIT";
		case 20: return "PRCM_HOSTACCESS SET";
		case 30: return "MB0H_READ_WAKEUP_ACK";
		case 60: return "PRCM_HOSTACCESS CLR";
		case 80: return "AUD SHARED RD PTR";
		case 81: return "AUD SHARED WR PTR";
		case 82: return "COM SHARED RD PTR";
		case 83: return "COM SHARED WD PTR";
		case 100: return "POWER_STATE_TRANS";
		case 110: return "CONFIG_WAKEUPS_EXE";
		case 111: return "CONFIG_WAKEUPS_SLEEP";
		case 112: return "AUTO_PM";
		case 120: return "ARM OPP";
		case 130: return "APE OPP";
		case 140: return "APE_OPP_100_VOLT";
		case 150: return "RELEASE_USB_WAKEUP";
		case 160: return "PLL_ON_OFF";
		case 170: return "DPS_REQ";
		case 171: return "DPS_RES";
		case 180: return "SYSCLK";
		case 181: return "REGCLK_REQ";
		case 182: return "REGCLK_RES";
		case 190: return "ESRAM MEM STATE";
		case 200: return "HOTDOG";
		case 210: return "HOTMON";
		case 220: return "HOT PERIOD";
		case 230: return "ABB READ";
		case 231: return "ABB READ RESULT";
		case 240: return "ABB WRITE";
		case 248: return "CA MSG PEND 0";
		case 249: return "CA MSG PEND 1";
		case 250: return "RESET_MODEM";
		case 251: return "AC COM PEND NOTIF";
		case 252: return "AC AUD PEND NOTIF";
		case 253: return "CA COM READ NOTIF";
		case 254: return "CA AUD READ NOTIF";
		default: return "UNKNOWN";
	}
}

static void *c;
static u32 __iomem *p;

static void prcmu_stats_prints(struct seq_file *m)
{
	u32 __iomem *q;
	int i;
	u64 last_clk = 0;

	q = tcdm_base;

	seq_printf(m, "PRCMU DATA MEMORY 0x801B8000...0x801B8FFF:\n");

	for (i = 0; i < 1024; i += 4) {
		if (0x801B8000 + i*4 == 0x801B8E00)
			seq_printf(m, "ackMb0 start at 0x801B8E08\n");
		seq_printf(m, "%x: 0x%.8x 0x%.8x 0x%.8x 0x%.8x\n",
			0x801B8000 + i*4, readl(q++), readl(q++),
			readl(q++), readl(q++));
	}

#ifdef CONFIG_SAMSUNG_LOG_BUF
	if (a_log) {
		seq_printf(m, "\nlog_idx: %d\n", log_idx);

		for (i = 0; i < A_LOG_SIZE; i++) {
			char e[100];

			if (a_log[i].pc == 10) {
				char* c = e;
				char* d = "ERROR";
				u32 ev = a_log[i].extra1;
				u8 header = a_log[i].extra2;

				if (ev & WAKEUP_BIT_RTC)
					c = "RTC";

				if (ev & WAKEUP_BIT_RTT0)
					c = "RTT0";

				if (ev & WAKEUP_BIT_RTT1)
					c = "RTT1";

				if (ev & WAKEUP_BIT_HSI0)
				c = "HSI0";

				if (ev & WAKEUP_BIT_HSI1)
					c = "HSI1";

				if (ev & WAKEUP_BIT_CA_WAKE)
					c = "CA_WAKE";

				if (ev & WAKEUP_BIT_USB)
					c = "USB";

				if (ev & WAKEUP_BIT_ABB)
					c = "ABB";

				if (ev & WAKEUP_BIT_ABB_FIFO)
				c = "ABB_FIFO";

				if (ev & WAKEUP_BIT_SYSCLK_OK)
					c = "SYSCLK_OK";

				if (ev & WAKEUP_BIT_CA_SLEEP)
					c = "CA_SLEEP";

				if (ev & WAKEUP_BIT_AC_WAKE_ACK)
					c = "AC_WAKE_ACK";

				if (ev & WAKEUP_BIT_SIDE_TONE_OK)
					c = "SIDE_TONE_OK";

				if (ev & WAKEUP_BIT_ANC_OK)
					c = "ANC_OK";

				if (ev & WAKEUP_BIT_SW_ERROR)
					c = "SW_ERROR";

				if (ev & WAKEUP_BIT_AC_SLEEP_ACK)
					c = "AC_SLEEP_ACK";

				if (ev & WAKEUP_BIT_ARM)
					c = "ARM";

				if (ev & WAKEUP_BIT_HOTMON_LOW)
					c = "HOTMON_LOW";
	
				if (ev & WAKEUP_BIT_HOTMON_HIGH)
					c = "HOTMON_HIGH";

				if (ev & WAKEUP_BIT_MODEM_SW_RESET_REQ)
					c = "MODEM_SW_RESET_REQ";

				if (ev & WAKEUP_BIT_GPIO0)
					c = "GPIO0";

				if (ev & WAKEUP_BIT_GPIO1)
					c = "GPIO1";

				if (ev & WAKEUP_BIT_GPIO2)
					c = "GPIO2";

				if (ev & WAKEUP_BIT_GPIO3)
					c = "GPIO3";

				if (ev & WAKEUP_BIT_GPIO4)
					c = "GPIO4";

				if (ev & WAKEUP_BIT_GPIO5)
					c = "GPIO5) ";

				if (ev & WAKEUP_BIT_GPIO6)
					c = "GPIO6";

				if (ev & WAKEUP_BIT_GPIO7)
					c = "GPIO7";

				if (ev & WAKEUP_BIT_GPIO8)
					c = "GPIO8";

				/* If more than one bit was set, print that */
				if (c == e)
					sprintf(c, "%.8x", ev);

				if (header == MB0H_WAKEUP_EXE)
					d = "MB0H_WAKEUP_EXE";
				else if (header == MB0H_WAKEUP_SLEEP)
					d = "MB0H_WAKEUP_SLEEP";

				seq_printf(m, "%.4u: %.18lld %.18lld %-20s %15s: %-20s %15s: %s\n",
						i,
						a_log[i].cpu_clk,
						a_log[i].cpu_clk - last_clk,
						convert_pc_to_text(a_log[i].pc),
						a_log[i].a ? a_log[i].a : "",
						c,
						a_log[i].b ? a_log[i].b : "",
						d);
			}
			else
			{
				seq_printf(m, "%.4u: %.18lld %.18lld %-20s %15s: 0x%-18x %15s: 0x%x\n",
						i,
						a_log[i].cpu_clk,
						a_log[i].cpu_clk - last_clk,
						convert_pc_to_text(a_log[i].pc),
						a_log[i].a ? a_log[i].a : "",
						a_log[i].extra1,
						a_log[i].b ? a_log[i].b : "",
						a_log[i].extra2);
			}

			if (i == log_idx)
				seq_puts(m, "---\n");

			last_clk = a_log[i].cpu_clk;

		}
	}
#endif

	seq_printf(m, "\nPRCMU REGISTERS\n");

	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ARMCLKFIX_MGT", p[0/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ACLK_MGT", p[4/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_SVAMMDSPCLK_MGT", p[8/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_SIAMMDSPCLK_MGT", p[12/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_SGACLK_MGT", p[20/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_UARTCLK_MGT", p[24/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_MSP02CLK_MGT", p[28/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_I2CCLK_MGT", p[32/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_SDMMCCLK_MGT", p[36/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_SLIMCLK_MGT", p[40/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PER1CLK_MGT", p[44/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PER2CLK_MGT", p[48/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PER3CLK_MGT", p[52/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PER5CLK_MGT", p[56/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PER6CLK_MGT", p[60/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PER7CLK_MGT", p[64/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_LCDCLK_MGT", p[68/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_SPARE1CLK_MGT", p[72/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_BMLCLK_MGT", p[76/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_HSITXCLK_MGT", p[80/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_HSIRXCLK_MGT", p[84/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_HDMICLK_MGT", p[88/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_APEATCLK_MGT", p[92/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_APETRACECLK_MGT", p[96/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_MCDECLK_MGT", p[100/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_IPI2CCLK_MGT", p[104/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_DSIALTCLK_MGT", p[108/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_SPARE2CLK_MGT", p[112/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_DMACLK_MGT", p[116/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_B2R2CLK_MGT", p[120/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_TVCLK_MGT", p[124/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PLLSOC0_FREQ", p[128/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PLLSOC1_FREQ", p[132/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PLLARM_FREQ", p[136/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PLLDDR_FREQ", p[140/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PLLSOC0_ENABLE", p[144/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PLLSOC1_ENABLE", p[148/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PLLARM_ENABLE", p[152/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PLLDDR_ENABLE", p[156/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PLLSOC0_LOCKP", p[160/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PLLSOC1_LOCKP", p[164/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PLLARM_LOCKP", p[168/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PLLDDR_LOCKP", p[172/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_XP70CLK_MGT", p[176/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_TIMER_0_REF", p[180/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_TIMER_0_DOWNCOUNT", p[184/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_TIMER_0_MODE", p[188/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_TIMER_1_REF", p[192/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_TIMER_1_DOWNCOUNT", p[196/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_TIMER_1_MODE", p[200/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_TIMER_2_REF", p[204/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_TIMER_2_DOWNCOUNT", p[208/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_TIMER_2_MODE", p[212/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_CLK009_MGT", p[228/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_MODECLK", p[232/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ARM_IT_SET", p[240/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_4500_CLK_REQ", p[248/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_MBOX_CPU_VAL", p[252/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PLL32K_ENABLE", p[268/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PLL32K_LOCKP", p[272/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ARM_CHGCLKREQ", p[276/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ARM_PLLDIVPS", p[280/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ARMITMSK31TO0", p[284/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ARMITMSK63TO32", p[288/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ARMITMSK95TO64", p[292/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ARMITMSK127TO96", p[296/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ARMSTANDBY_STATUS", p[304/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_CGATING_BYPASS", p[308/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_GPIOCR", p[312/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_LEMI", p[316/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_COMPCR", p[320/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_COMPSTA", p[324/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ITSTATUS0", p[328/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ITSTATUS1", p[336/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ITSTATUS2", p[344/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ITSTATUS3", p[352/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ITSTATUS4", p[360/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_LINE_VALUE", p[368/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_HOLD_EVT", p[372/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_EDGE_SENS_L", p[376/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_EDGE_SENS_H", p[380/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_DEBUG_CTRL_VAL", p[400/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_DEBUG_NOPWRDOWN_VAL", p[404/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_DEBUG_CTRL_ACK", p[408/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_A9PL_FORCE_CLKEN", p[412/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_TPIU_FLUSHIN_REQ", p[416/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_TPIU_FLUSHIN_ACK", p[420/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_STP_FLUSHIN_REQ", p[424/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_STP_FLUSHIN_ACK", p[428/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_HWI2C_DIV", p[432/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_HWI2C_GO", p[436/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_HWI2C_CMD", p[440/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_HWI2C_DATA123", p[444/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_HWI2C_SR", p[448/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_REMAPCR", p[452/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_TCR", p[456/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_CLKOCR", p[460/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ITSTATUS_DBG", p[464/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_LINE_VALUE_DBG", p[472/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_DBG_HOLD", p[476/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_EDGE_SENS_DBG", p[480/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_APE_RESETN_SET", p[484/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_APE_RESETN_VAL", p[492/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_A9_RESETN_SET", p[496/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_A9_RESETN_VAL", p[504/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_MOD_RESETN_SET", p[508/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_MOD_RESETN_VAL", p[516/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_GPIO_RESETN_SET", p[520/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_GPIO_RESETN_VAL", p[528/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_4500_RESETN_SET", p[532/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_4500_RESETN_VAL", p[540/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_HSI_SOFTRST", p[548/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_APE_SOFTRST", p[552/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_SWD_RST_TEMPO", p[568/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_RST_4500_TEMPO", p[572/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_SVAMMDSP_IT", p[576/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_SIAMMDSP_IT", p[584/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_POWER_STATE_SET", p[596/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_POWER_STATE_VAL", p[604/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ARMITVALUE31TO0", p[608/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ARMITVALUE63TO32", p[612/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ARMITVALUE95TO64", p[616/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ARMITVALUE127TO96", p[620/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_REDUN_LOAD", p[624/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_REDUN_STATUS", p[628/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_UNIPROCLK_MGT", p[632/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_UICCCLK_MGT", p[636/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_SSPCLK_MGT", p[640/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_RNGCLK_MGT", p[644/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_MSP1CLK_MGT", p[648/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_DAP_RESETN_SET", p[672/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_DAP_RESETN_VAL", p[680/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_SRAM_DEDCSTOV", p[768/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_SRAM_LS_SLEEP", p[772/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_SRAM_A9", p[776/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ARM_LS_CLAMP", p[780/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_IOCR", p[784/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_MODEM_SYSCLKOK", p[788/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_SYSCLKOK_DELAY", p[792/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_SYSCLKSTATUS", p[796/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_DSI_SW_RESET", p[804/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_A9_MASK_REQ", p[808/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_A9_MASK_ACK", p[812/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_HOSTACCESS_REQ", p[820/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_TIMER_3_REF", p[824/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_TIMER_3_DOWNCOUNT", p[828/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_TIMER_3_MODE", p[832/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PMB_SENS_CTRL", p[836/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PMB_REF_COUNTER", p[840/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PMB_SENSOR_STATUS", p[844/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_APE_EPOD_CFG", p[1028/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_DDR_EPOD_CFG", p[1032/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_EPOD_C_SET", p[1040/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_EPOD_C_VAL", p[1048/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_EPOD_VOK", p[1052/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_MMIP_LS_CLAMP_SET", p[1056/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_MMIP_LS_CLAMP_VAL", p[1064/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_VSAFE_LS_CLAMP_SET", p[1068/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_VSAFE_LS_CLAMP_VAL", p[1076/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_DDRSUBSYS_APE_MINBW", p[1080/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_DDRSUBSYS_STATUS", p[1084/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_DDRSUBSYS_CONTROL", p[1088/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_DDRSUBSYS_HIGH_LEAK_COND", p[1092/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_DDRSUBSYS_CONFIG", p[1096/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_DDRSUBSYS_CONFIG_ACK", p[1100/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_TIMER_4_REF", p[1104/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_TIMER_4_DOWNCOUNT", p[1108/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_TIMER_4_MODE", p[1112/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_TIMER_5_REF", p[1116/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_TIMER_5_DOWNCOUNT", p[1120/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_TIMER_5_MODE", p[1124/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_APE_MEM_REQ", p[1136/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_DBG_FORCES_APE_MEM_REQ", p[1140/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_APE_MEM_WFX_EN", p[1144/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_APE_MEM_LATENCY", p[1148/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_APE_MEM_ACK", p[1152/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ITSTATUS5", p[1156/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ARM_IT1_SET", p[1168/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ARM_IT1_VAL", p[1172/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_MOD_PWR_OK", p[1176/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_MOD_AUXCLKOK", p[1180/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_MOD_AWAKE_STATUS", p[1184/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_MOD_SWRESET_IRQ_ACK", p[1188/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_MOD_SWRESET_ACK", p[1192/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_DBG_PWRCTL", p[1196/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_HWOBS_H", p[1200/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_HWOBS_L", p[1204/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PLLDSI_FREQ", p[1280/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PLLDSI_ENABLE", p[1284/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PLLDSI_LOCKP", p[1288/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_RNG_ENABLE", p[1292/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_YYCLKEN0_MGT_SET", p[1296/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_YYCLKEN1_MGT_SET", p[1300/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_YYCLKEN0_MGT_VAL", p[1312/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_YYCLKEN1_MGT_VAL", p[1316/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_XP70CLK_MGT2", p[1320/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_DSITVCLK_DIV", p[1324/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_DSI_PLLOUT_SEL", p[1328/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_DSI_GLITCHFREE_EN", p[1332/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_CLKACTIV", p[1336/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_SIA_MMDSP_MEM_MGT", p[1340/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_SVA_MMDSP_MEM_MGT", p[1344/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_SXAMMDSP_FORCE_CLKEN", p[1348/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_UICC_NANDTREE", p[1392/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_GPIOCR2", p[1396/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_MDM_ACWAKE", p[1400/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_MOD_MEM_REQ", p[1444/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_MOD_MEM_ACK", p[1448/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ARM_PLLDIVPS_REQ", p[1456/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ARM_PLLDIVPS_ACK", p[1460/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_SRPTIMER_VAL", p[1488/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_SECNONSEWM", p[4096/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ESRAM0_INITN", p[4100/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ARMITMSKSEC_31TO0", p[4104/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ARMITMSKSEC_63TO32", p[4108/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ARMITMSKSEC_95TO64", p[4112/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ARMITMSKSEC_127TO96", p[4116/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ARMIT_MASKXP70_IT", p[4120/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ESRAM0_EPOD_CFG", p[4124/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ESRAM0_EPOD_C_VAL", p[4128/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ESRAM0_EPOD_C_SET", p[4132/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ESRAM0_EPOD_VOK", p[4140/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_ESRAM0_LS_SLEEP", p[4144/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_SECURE_ONGOING", p[4148/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_I2C_SECURE", p[4152/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_RESET_STATUS", p[4156/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PERIPH4_RESETN_SET", p[4160/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PERIPH4_RESETN_VAL", p[4168/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_SPAREOUT_SEC", p[4172/4]);
	seq_printf(m, "%-30s0x%.8x\n", "PRCM_PIPELINEDCR", p[4312/4]);

	seq_puts(m, "\n");

	for (i = 0; i < 100; i++) {
		seq_printf(m, "cpu0: PC: 0x%.8x 0x%.8lx\n", readl(c), virt_to_phys((void *) readl(c)));
		udelay(10);
	}

	for (i = 0; i < 100; i++) {
		seq_printf(m, "cpu1: PC: 0x%.8x 0x%.8lx\n", readl(c+0x2000), virt_to_phys((void *) readl(c+0x2000)));
		udelay(10);
	}
}

static int prcmu_stats_show(struct seq_file *m, void *unused)
{
#ifdef CONFIG_SAMSUNG_LOG_BUF
	log_active = 0;
#endif

	seq_puts(m, "=== PRCMU DUMP START ===\n");

	prcmu_stats_prints(m);

	seq_puts(m, "=== PRCMU DUMP DONE ===\n");

#ifdef CONFIG_SAMSUNG_LOG_BUF
	log_active = 1;
#endif

	return 0;
}

static int prcmu_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, prcmu_stats_show, NULL);
}

static const struct file_operations prcmu_stats_fops = {
	.owner = THIS_MODULE,
	.open = prcmu_stats_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


#ifdef CONFIG_SAMSUNG_PANIC_DISPLAY_DEVICES
/**
 * prcmu_panic_abb_read() - Read register value(s) from the ABB under Kernel Panic.
 * @slave:	The I2C slave address.
 * @reg:	The (start) register address.
 * @value:	The read out value(s).
 * @size:	The number of registers to read.
 *
 * Reads register value(s) from the ABB.
 * @size has to be 1 for the current firmware version.
 */
int prcmu_panic_abb_read(u8 slave, u8 reg, u8 *value, u8 size)
{
	int r;
	u32 timeout = PRCMU_I2C_TIMEOUT;
	u8 status;

	if (size != 1)
		return -EINVAL;

	if (readl(_PRCMU_BASE + PRCM_ARM_IT1_VAL) & MBOX_BIT(5)) {
		/* clear mailbox 5 ack irq */
		writel(MBOX_BIT(5), _PRCMU_BASE + PRCM_ARM_IT1_CLR);
	}

	while ((readl(_PRCMU_BASE + PRCM_MBOX_CPU_VAL) & MBOX_BIT(5)) && timeout--)
		cpu_relax();

	if ( timeout == 0 ){
		pr_emerg("%s: timed out waiting for MBOX5 to be free.\n",__func__);
		return -EIO;
	}

	writeb(PRCMU_I2C_READ(slave), (tcdm_base + PRCM_REQ_MB5_I2C_SLAVE_OP));
	writeb(PRCMU_I2C_STOP_EN, (tcdm_base + PRCM_REQ_MB5_I2C_HW_BITS));
	writeb(reg, (tcdm_base + PRCM_REQ_MB5_I2C_REG));
	writeb(0, (tcdm_base + PRCM_REQ_MB5_I2C_VAL));

	writel(MBOX_BIT(5), (_PRCMU_BASE + PRCM_MBOX_CPU_SET));

	timeout = PRCMU_I2C_TIMEOUT;

	while (!(readl(_PRCMU_BASE + PRCM_ARM_IT1_VAL) & MBOX_BIT(5)) && timeout--)
		cpu_relax();

	if (!timeout) {
		pr_emerg("%s timed out waiting for a reply.\n",__func__);
		return -EIO;
	}

	status = readb(tcdm_base + PRCM_ACK_MB5_I2C_STATUS);
	*value = readb(tcdm_base + PRCM_ACK_MB5_I2C_VAL);

	/* clear mailbox 5 ack irq */
	writel(MBOX_BIT(5), _PRCMU_BASE + PRCM_ARM_IT1_CLR);

	r = ((status == I2C_RD_OK) ? 0 : -EIO);

	if (r) {
		pr_emerg("%s I2C RD NAK.\n",__func__);
	}

	return r;
}

/**
 * prcmu_panic_abb_write() - Write register value(s) to the ABB under Kernel Panic.
 * @slave:	The I2C slave address.
 * @reg:	The (start) register address.
 * @value:	The value(s) to write.
 * @size:	The number of registers to write.
 *
 * Reads register value(s) from the ABB.
 * @size has to be 1 for the current firmware version.
 */
int prcmu_panic_abb_write(u8 slave, u8 reg, u8 *value, u8 size)
{
	int r;
	u32 timeout = PRCMU_I2C_TIMEOUT;
	u8 status;

	if (size != 1)
		return -EINVAL;

	if (readl(_PRCMU_BASE + PRCM_ARM_IT1_VAL) & MBOX_BIT(5)) {
		/* clear mailbox 5 ack irq */
		writel(MBOX_BIT(5), _PRCMU_BASE + PRCM_ARM_IT1_CLR);
	}

	while ((readl(_PRCMU_BASE + PRCM_MBOX_CPU_VAL) & MBOX_BIT(5)) && timeout--)
		cpu_relax();

	if ( timeout == 0 ){
		pr_emerg("%s: timed out waiting for MBOX5 to be free.\n",__func__);
		return -EIO;
	}

	writeb(PRCMU_I2C_WRITE(slave), (tcdm_base + PRCM_REQ_MB5_I2C_SLAVE_OP));
	writeb(PRCMU_I2C_STOP_EN, (tcdm_base + PRCM_REQ_MB5_I2C_HW_BITS));
	writeb(reg, (tcdm_base + PRCM_REQ_MB5_I2C_REG));
	writeb(*value, (tcdm_base + PRCM_REQ_MB5_I2C_VAL));

	writel(MBOX_BIT(5), (_PRCMU_BASE + PRCM_MBOX_CPU_SET));

	timeout = PRCMU_I2C_TIMEOUT;

	while (!(readl(_PRCMU_BASE + PRCM_ARM_IT1_VAL) & MBOX_BIT(5)) && timeout--)
		cpu_relax();

	if (!timeout) {
		pr_emerg("%s: timed out waiting for a reply.\n",__func__);
		return -EIO;
	}

	status = readb(tcdm_base + PRCM_ACK_MB5_I2C_STATUS);

	/* clear mailbox 5 ack irq */
	writel(MBOX_BIT(5), _PRCMU_BASE + PRCM_ARM_IT1_CLR);

	r = ((status == I2C_RD_OK) ? 0 : -EIO);

	return r;
}
#endif

/**
 * prcmu_ac_wake_req - should be called whenever ARM wants to wakeup Modem
 */
int prcmu_ac_wake_req(void)
{
	u32 val;
	u32 status;

	mutex_lock(&mb0_transfer.ac_wake_lock);

	val = readl(_PRCMU_BASE + PRCM_HOSTACCESS_REQ);

	if (val & PRCM_HOSTACCESS_REQ_HOSTACCESS_REQ)
		goto unlock_and_return;

	atomic_set(&ac_wake_req_state, 1);

retry:
	log_this(20, NULL, 0, NULL, 0);
	/*
	 * Force Modem Wake-up before hostaccess_req ping-pong.
	 * It prevents Modem to enter in Sleep while acking the hostaccess
	 * request. The 31us delay has been calculated by HWI.
	 */
	val |= PRCM_HOSTACCESS_REQ_WAKE_REQ;
	writel(val, (_PRCMU_BASE + PRCM_HOSTACCESS_REQ));

	udelay(31);

	writel((val | PRCM_HOSTACCESS_REQ_HOSTACCESS_REQ),
		(_PRCMU_BASE + PRCM_HOSTACCESS_REQ));
/*+ debug patch for sysclk status */
	if (!wait_for_completion_timeout(&mb0_transfer.ac_wake_work,
			msecs_to_jiffies(4000))) {
		u8 data;
		prcmu_abb_read(0x4, 0xa, &data, 1);
		pr_info("prcmu : 0x%02x@0x040A\n", data);

		prcmu_abb_read(0x1, 0x42, &data, 1);
		pr_info("prcmu : 0x%02x@0x0142\n", data);

		prcmu_abb_read(0x2, 0xd, &data, 1);
		pr_info("prcmu : 0x%02x@0x020D\n", data);
		/*- debug patch for sysclk status */

		pr_crit("prcmu: %s timed out (5 s) waiting for a reply.\n",
			__func__);
		mutex_unlock(&mb0_transfer.ac_wake_lock);
		return -EFAULT;
	}
/*- debug patch for sysclk status */
	/*
	 * The modem can generate an AC_WAKE_ACK, and then still go to sleep.
	 * As a workaround, we wait, and then check that the modem is indeed
	 * awake (in terms of the value of the PRCM_MOD_AWAKE_STATUS
	 * register, which may not be the whole truth).
	 */
	udelay(400);
	status = (readl(_PRCMU_BASE + PRCM_MOD_AWAKE_STATUS) & BITS(0, 2));
	if (status != (PRCM_MOD_AWAKE_STATUS_PRCM_MOD_AAPD_AWAKE |
			PRCM_MOD_AWAKE_STATUS_PRCM_MOD_COREPD_AWAKE)) {
		pr_err("prcmu: %s received ack, but modem not awake (0x%X).\n",
			__func__, status);
		udelay(1200);

		val &= ~PRCM_HOSTACCESS_REQ_WAKE_REQ;

		writel(val, (_PRCMU_BASE + PRCM_HOSTACCESS_REQ));
		if (wait_for_completion_timeout(&mb0_transfer.ac_wake_work,
				msecs_to_jiffies(5000)))
			goto retry;
		pr_crit("prcmu: %s timed out (5 s) waiting for AC_SLEEP_ACK.\n",
				__func__);
		mutex_unlock(&mb0_transfer.ac_wake_lock);
		return -EFAULT;
	}

unlock_and_return:
	mutex_unlock(&mb0_transfer.ac_wake_lock);
	return 0;
}

/**
 * prcmu_ac_sleep_req - called when ARM no longer needs to talk to modem
 */
void prcmu_ac_sleep_req()
{
	u32 val;

	mutex_lock(&mb0_transfer.ac_wake_lock);

	val = readl(_PRCMU_BASE + PRCM_HOSTACCESS_REQ);
	if (!(val & PRCM_HOSTACCESS_REQ_HOSTACCESS_REQ))
		goto unlock_and_return;

	log_this(60, NULL, 0, NULL, 0);

	val &= ~(PRCM_HOSTACCESS_REQ_HOSTACCESS_REQ |
			PRCM_HOSTACCESS_REQ_WAKE_REQ);
	writel(val, (_PRCMU_BASE + PRCM_HOSTACCESS_REQ));

	if (!wait_for_completion_timeout(&mb0_transfer.ac_wake_work,
			msecs_to_jiffies(5000))) {

		panic("prcmu: %s timed out (5 s) waiting for a reply.\n",
			__func__);
	}

	atomic_set(&ac_wake_req_state, 0);

unlock_and_return:
	mutex_unlock(&mb0_transfer.ac_wake_lock);
}

bool prcmu_is_ac_wake_requested(void)
{
	return (atomic_read(&ac_wake_req_state) != 0);
}

/*
 * it's really redundant job to see this value for ApSleep
 * however, there's a suspicious problem(ER416165) which seems to be related to ApSleep.
 * @dedicated usage only on cpuidle
 */
bool prcmu_is_mcdeclk_on(void)
{
	volatile unsigned int reg_value;
	reg_value = readl(_PRCMU_BASE + PRCM_MCDECLK_MGT);
	return (reg_value & 0x100 /* MCDECLKEN */);
}
EXPORT_SYMBOL(prcmu_is_mcdeclk_on);

/**
 * prcmu_system_reset - System reset
 *
 * Saves the reset reason code and then sets the APE_SOFTRST register which
 * fires interrupt to fw
 */
void prcmu_system_reset(u16 reset_code)
{
	local_irq_disable();

	writew_relaxed(reset_code, (tcdm_base + PRCM_SW_RST_REASON));
#ifdef CONFIG_SAMSUNG_LOG_BUF
	__write_log((_PRCMU_BASE + PRCM_APE_SOFTRST));
#endif
	writel_relaxed(1, (_PRCMU_BASE + PRCM_APE_SOFTRST));
	smp_wmb();
}

/**
 * prcmu_get_reset_code - Retrieve SW reset reason code
 *
 * Retrieves the reset reason code stored by prcmu_system_reset() before
 * last restart.
 */
u16 prcmu_get_reset_code(void)
{
	return readw(tcdm_base + PRCM_SW_RST_REASON);
}

/**
 * prcmu_reset_modem - ask the PRCMU to reset modem
 */
void prcmu_modem_reset(void)
{
	mutex_lock(&mb1_transfer.lock);

	while (readl(_PRCMU_BASE + PRCM_MBOX_CPU_VAL) & MBOX_BIT(1))
		cpu_relax();

	writeb(MB1H_RESET_MODEM, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB1));
	log_this(250, NULL, 0, NULL, 0);
	writel(MBOX_BIT(1), (_PRCMU_BASE + PRCM_MBOX_CPU_SET));
	wait_for_completion(&mb1_transfer.work);

	/*
	 * No need to check return from PRCMU as modem should go in reset state
	 * This state is already managed by upper layer
	 */

	mutex_unlock(&mb1_transfer.lock);
}

static void ack_dbb_wakeup(void)
{
	unsigned long flags;

	spin_lock_irqsave(&mb0_transfer.lock, flags);

	while (readl(_PRCMU_BASE + PRCM_MBOX_CPU_VAL) & MBOX_BIT(0))
		cpu_relax();

	writeb(MB0H_READ_WAKEUP_ACK, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB0));
	log_this(30, NULL, 0, NULL, 0);
	writel(MBOX_BIT(0), (_PRCMU_BASE + PRCM_MBOX_CPU_SET));

	spin_unlock_irqrestore(&mb0_transfer.lock, flags);
}

static inline void print_unknown_header_warning(u8 n, u8 header)
{
	pr_warning("prcmu: Unknown message header (%d) in mailbox %d.\n",
		header, n);
}

void uart_wakeunlock(void)
{
	pr_info("UART wakelock is unlocked.\n");
	wake_unlock(&prcmu_uart_wake_lock);
}
EXPORT_SYMBOL(uart_wakeunlock);

static bool read_mailbox_0(void)
{
	bool r = false;
	u32 ev;
	unsigned int n;
	u8 header;

	header = readb(tcdm_base + PRCM_MBOX_HEADER_ACK_MB0);
	switch (header) {
	case MB0H_WAKEUP_EXE:
	case MB0H_WAKEUP_SLEEP:
		if (readb(tcdm_base + PRCM_ACK_MB0_READ_POINTER) & 1)
			ev = readl(tcdm_base + PRCM_ACK_MB0_WAKEUP_1_8500);
		else
			ev = readl(tcdm_base + PRCM_ACK_MB0_WAKEUP_0_8500);

		if (ev & (WAKEUP_BIT_AC_WAKE_ACK | WAKEUP_BIT_AC_SLEEP_ACK))
			complete(&mb0_transfer.ac_wake_work);
		if (ev & WAKEUP_BIT_SYSCLK_OK)
			complete(&mb3_transfer.sysclk_work);

		log_this(10, "ev", ev, "header", header);
//+ WAKEUP CHECK
		if ((header == MB0H_WAKEUP_SLEEP) && (ev) && (ev != 1)) {
			pr_info("Wakeup Status: 0x%08x\n", ev);
			if(ev == 0x00800000) {
				pr_info("Wakeup Status: UART\n");
				wake_lock_timeout(&prcmu_uart_wake_lock, 20*HZ);
				ux500_ci_dbg_console();
			}
		}
//- WAKEUP CHECK
		ev &= mb0_transfer.req.dbb_irqs;

		for (n = 0; n < NUM_PRCMU_WAKEUPS; n++) {
			if (ev & prcmu_irq_bit[n]) {
				log_this(11, "IRQ", IRQ_PRCMU_BASE + n, NULL, 0);
				generic_handle_irq(IRQ_PRCMU_BASE + n);
			}
		}
		r = true;
		break;
	default:
		print_unknown_header_warning(0, header);
		break;
	}
	writel(MBOX_BIT(0), (_PRCMU_BASE + PRCM_ARM_IT1_CLR));

	if (r) {
		unsigned long flags;

		spin_lock_irqsave(&mb0_transfer.lock, flags);

		/* Do not send the ack if MB0 is busy */
		if (!(readl(_PRCMU_BASE + PRCM_MBOX_CPU_VAL) & MBOX_BIT(0))) {
			/* Send ack */
			writeb(MB0H_READ_WAKEUP_ACK, (tcdm_base + PRCM_MBOX_HEADER_REQ_MB0));
			writel(MBOX_BIT(0), (_PRCMU_BASE + PRCM_MBOX_CPU_SET));
			r = false;
		}

		spin_unlock_irqrestore(&mb0_transfer.lock, flags);
	}
	return r;
}

static bool read_mailbox_1(void)
{
	mb1_transfer.ack.header = readb(tcdm_base + PRCM_MBOX_HEADER_REQ_MB1);
	mb1_transfer.ack.arm_opp = readb(tcdm_base +
		PRCM_ACK_MB1_CURRENT_ARM_OPP);
	mb1_transfer.ack.ape_opp = readb(tcdm_base +
		PRCM_ACK_MB1_CURRENT_APE_OPP);
	mb1_transfer.ack.ape_voltage_status = readb(tcdm_base +
		PRCM_ACK_MB1_APE_VOLTAGE_STATUS);
	writel(MBOX_BIT(1), (_PRCMU_BASE + PRCM_ARM_IT1_CLR));
	complete(&mb1_transfer.work);
	return false;
}

static bool read_mailbox_2(void)
{
	mb2_transfer.ack.status = readb(tcdm_base + PRCM_ACK_MB2_DPS_STATUS);
	writel(MBOX_BIT(2), (_PRCMU_BASE + PRCM_ARM_IT1_CLR));
	complete(&mb2_transfer.work);
	return false;
}

static bool read_mailbox_3(void)
{
	writel(MBOX_BIT(3), (_PRCMU_BASE + PRCM_ARM_IT1_CLR));
	return false;
}

static bool read_mailbox_4(void)
{
	u8 header;
	bool do_complete = true;

	header = readb(tcdm_base + PRCM_MBOX_HEADER_REQ_MB4);
	switch (header) {
	case MB4H_MEM_ST:
	case MB4H_HOTDOG:
	case MB4H_HOTMON:
	case MB4H_HOT_PERIOD:
		break;
	default:
		print_unknown_header_warning(4, header);
		do_complete = false;
		break;
	}

	writel(MBOX_BIT(4), (_PRCMU_BASE + PRCM_ARM_IT1_CLR));

	if (do_complete)
		complete(&mb4_transfer.work);

	return false;
}

static bool read_mailbox_5(void)
{
	mb5_transfer.ack.status = readb(tcdm_base + PRCM_ACK_MB5_I2C_STATUS);
	mb5_transfer.ack.value = readb(tcdm_base + PRCM_ACK_MB5_I2C_VAL);
	writel(MBOX_BIT(5), (_PRCMU_BASE + PRCM_ARM_IT1_CLR));
	complete(&mb5_transfer.work);
	return false;
}

static bool read_mailbox_6(void)
{
	writel(MBOX_BIT(6), (_PRCMU_BASE + PRCM_ARM_IT1_CLR));
	return false;
}

static bool read_mailbox_7(void)
{
	writel(MBOX_BIT(7), (_PRCMU_BASE + PRCM_ARM_IT1_CLR));
	return false;
}

static bool (* const read_mailbox[NUM_MB])(void) = {
	read_mailbox_0,
	read_mailbox_1,
	read_mailbox_2,
	read_mailbox_3,
	read_mailbox_4,
	read_mailbox_5,
	read_mailbox_6,
	read_mailbox_7
};

static irqreturn_t prcmu_irq_handler(int irq, void *data)
{
	u32 bits;
	u8 n;
	irqreturn_t r;

	bits = (readl(_PRCMU_BASE + PRCM_ARM_IT1_VAL) & ALL_MBOX_BITS);
	if (unlikely(!bits))
		return IRQ_NONE;

	r = IRQ_HANDLED;
	for (n = 0; bits; n++) {
		if (bits & MBOX_BIT(n)) {
			bits -= MBOX_BIT(n);
			if (read_mailbox[n]())
				r = IRQ_WAKE_THREAD;
		}
	}
	return r;
}

static irqreturn_t prcmu_irq_thread_fn(int irq, void *data)
{
	ack_dbb_wakeup();
	return IRQ_HANDLED;
}

static void prcmu_mask_work(struct work_struct *work)
{
	unsigned long flags;

	spin_lock_irqsave(&mb0_transfer.lock, flags);

	config_wakeups();

	spin_unlock_irqrestore(&mb0_transfer.lock, flags);
}

static void prcmu_irq_mask(unsigned int irq)
{
	unsigned long flags;

	spin_lock_irqsave(&mb0_transfer.dbb_irqs_lock, flags);

	mb0_transfer.req.dbb_irqs &= ~prcmu_irq_bit[irq - IRQ_PRCMU_BASE];

	spin_unlock_irqrestore(&mb0_transfer.dbb_irqs_lock, flags);

	if (irq != IRQ_PRCMU_CA_SLEEP)
		schedule_work(&mb0_transfer.mask_work);
}

static void prcmu_irq_unmask(unsigned int irq)
{
	unsigned long flags;

	spin_lock_irqsave(&mb0_transfer.dbb_irqs_lock, flags);

	mb0_transfer.req.dbb_irqs |= prcmu_irq_bit[irq - IRQ_PRCMU_BASE];

	spin_unlock_irqrestore(&mb0_transfer.dbb_irqs_lock, flags);

	if (irq != IRQ_PRCMU_CA_SLEEP)
		schedule_work(&mb0_transfer.mask_work);
}

static void noop(unsigned int irq)
{
}

static struct irq_chip prcmu_irq_chip = {
	.name		= "prcmu",
	.disable	= prcmu_irq_mask,
	.ack		= noop,
	.mask		= prcmu_irq_mask,
	.unmask		= prcmu_irq_unmask,
};

void __init prcmu_early_init(void)
{
	unsigned int i;

	if (cpu_is_u8500v1()) {
		tcdm_base = __io_address(U8500_PRCMU_TCDM_BASE_V1);
	} else if (cpu_is_u8500v2()) {
		void *tcpm_base = ioremap_nocache(U8500_PRCMU_TCPM_BASE, SZ_4K);

		if (tcpm_base != NULL) {
			int version;
			version = readl(tcpm_base + PRCMU_FW_VERSION_OFFSET);
			prcmu_version.project_number = version & 0xFF;
			prcmu_version.api_version = (version >> 8) & 0xFF;
			prcmu_version.func_version = (version >> 16) & 0xFF;
			prcmu_version.errata = (version >> 24) & 0xFF;
			pr_info("PRCMU firmware version %d.%d.%d\n",
				(version >> 8) & 0xFF, (version >> 16) & 0xFF,
				(version >> 24) & 0xFF);
			iounmap(tcpm_base);
		}

		tcdm_base = __io_address(U8500_PRCMU_TCDM_BASE);
	} else {
		pr_err("prcmu: Unsupported chip version\n");
		BUG();
	}

	spin_lock_init(&mb0_transfer.lock);
	spin_lock_init(&mb0_transfer.dbb_irqs_lock);
	mutex_init(&mb0_transfer.ac_wake_lock);
	init_completion(&mb0_transfer.ac_wake_work);
	mutex_init(&mb1_transfer.lock);
	init_completion(&mb1_transfer.work);
	mb1_transfer.ape_opp = APE_NO_CHANGE;
	mutex_init(&mb2_transfer.lock);
	init_completion(&mb2_transfer.work);
	spin_lock_init(&mb2_transfer.auto_pm_lock);
	spin_lock_init(&mb3_transfer.lock);
	mutex_init(&mb3_transfer.sysclk_lock);
	init_completion(&mb3_transfer.sysclk_work);
	mutex_init(&mb4_transfer.lock);
	init_completion(&mb4_transfer.work);
	mutex_init(&mb5_transfer.lock);
	init_completion(&mb5_transfer.work);

	INIT_WORK(&mb0_transfer.mask_work, prcmu_mask_work);

	/* Initalize irqs. */
	for (i = 0; i < NUM_PRCMU_WAKEUPS; i++) {
		unsigned int irq;

		irq = IRQ_PRCMU_BASE + i;
		set_irq_chip(irq, &prcmu_irq_chip);
		set_irq_flags(irq, IRQF_VALID);
		set_irq_handler(irq, handle_simple_irq);
	}

	wake_lock_init(&prcmu_uart_wake_lock, WAKE_LOCK_SUSPEND, "prcmu_uart_wake_lock");
}

static void __init init_prcm_registers(void)
{
	u32 val;

	val = readl(_PRCMU_BASE + PRCM_A9PL_FORCE_CLKEN);
	val &= ~(PRCM_A9PL_FORCE_CLKEN_PRCM_A9PL_FORCE_CLKEN |
		PRCM_A9PL_FORCE_CLKEN_PRCM_A9AXI_FORCE_CLKEN);
	writel(val, (_PRCMU_BASE + PRCM_A9PL_FORCE_CLKEN));
}

/**
 * prcmu_fw_init - arch init call for the Linux PRCMU fw init logic
 *
 */
int __init prcmu_init(void)
{
	int err = 0;

	if (ux500_is_svp())
		return -ENODEV;

	init_prcm_registers();

	/* Clean up the mailbox interrupts after pre-kernel code. */
	writel(ALL_MBOX_BITS, (_PRCMU_BASE + PRCM_ARM_IT1_CLR));

	err = request_threaded_irq(IRQ_DB8500_PRCMU1, prcmu_irq_handler,
		prcmu_irq_thread_fn, IRQF_NO_SUSPEND, "prcmu", NULL);
	if (err < 0) {
		pr_err("prcmu: Failed to allocate IRQ_DB8500_PRCMU1.\n");
		err = -EBUSY;
		goto no_irq_return;
	}

	prcmu_config_esram0_deep_sleep(ESRAM0_DEEP_SLEEP_STATE_RET);

	if (prcmu_debug_init())
		pr_err("prcmu: Failed to initialize debugfs\n");

	c = ioremap_nocache(0x801A8084, SZ_16K);
	p = ioremap_nocache(0x80157000, SZ_8K);

	proc_create("prcmu", S_IRUGO, NULL, &prcmu_stats_fops);

no_irq_return:
	return err;
}

arch_initcall(prcmu_init);
