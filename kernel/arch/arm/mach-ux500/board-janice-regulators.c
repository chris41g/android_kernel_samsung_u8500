/*
 * Copyright (C) STMicroelectronics 2009
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License Terms: GNU General Public License v2
 * Authors: Sundar Iyer <sundar.iyer@stericsson.com> for ST-Ericsson
 *          Bengt Jonsson <bengt.g.jonsson@stericsson.com> for ST-Ericsson
 *
 * Board specific file for regulator machine initialization
 *
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/amba/bus.h>
#include <linux/regulator/machine.h>
#include <linux/mpu.h>
#include <mach/bma023.h>
#include <mach/gp2a.h>
#include <mach/mpu60x0.h>
#include "board-janice-regulators.h"

/*
 * AB8500 Regulator Configuration
 */

/* ab8500 regulator register initialization */
struct ab8500_regulator_reg_init
		janice_ab8500_regulator_reg_init[AB8500_NUM_REGULATOR_REGISTERS] = {
	/*
	 * VanaRequestCtrl          = HP/LP depending on VxRequest
	 * VpllRequestCtrl          = HP/LP depending on VxRequest
	 * VextSupply1RequestCtrl   = HP/LP depending on VxRequest
	 */
	INIT_REGULATOR_REGISTER(AB8500_REGUREQUESTCTRL2,       0x00),
	/*
	 * VextSupply2RequestCtrl   = HP/LP depending on VxRequest
	 * VextSupply3RequestCtrl   = HP/LP depending on VxRequest
	 * Vaux1RequestCtrl         = HP/LP depending on VxRequest
	 * Vaux2RequestCtrl         = HP/LP depending on VxRequest
	 */
	INIT_REGULATOR_REGISTER(AB8500_REGUREQUESTCTRL3,       0x00),
	/*
	 * Vaux3RequestCtrl         = HP/LP depending on VxRequest
	 * SwHPReq                  = Control through SWValid disabled
	 */
	INIT_REGULATOR_REGISTER(AB8500_REGUREQUESTCTRL4,       0x00),
	/*
	 * Vsmps1SysClkReq1HPValid  = enabled
	 * Vsmps2SysClkReq1HPValid  = enabled
	 * Vsmps3SysClkReq1HPValid  = enabled
	 * VanaSysClkReq1HPValid    = disabled
	 * VpllSysClkReq1HPValid    = enabled
	 * Vaux1SysClkReq1HPValid   = disabled
	 * Vaux2SysClkReq1HPValid   = disabled
	 * Vaux3SysClkReq1HPValid   = disabled
	 */
	INIT_REGULATOR_REGISTER(AB8500_REGUSYSCLKREQ1HPVALID1, 0x17),
	/*
	 * VextSupply1SysClkReq1HPValid = disabled
	 * VextSupply2SysClkReq1HPValid = disabled
	 * VextSupply3SysClkReq1HPValid = disabled
	 * VarmSysClkReq1HPValid	= disabled
	 * VapeSysClkReq1HPValid	= disabled
	 */
	INIT_REGULATOR_REGISTER(AB8500_REGUSYSCLKREQ1HPVALID2, 0x00),
	/*
	 * VanaHwHPReq1Valid        = disabled
	 * Vaux1HwHPreq1Valid       = disabled
	 * Vaux2HwHPReq1Valid       = disabled
	 * Vaux3HwHPReqValid        = disabled
	 */
	INIT_REGULATOR_REGISTER(AB8500_REGUHWHPREQ1VALID1,     0x00),
	/*
	 * VextSupply1HwHPReq1Valid = disabled
	 * VextSupply2HwHPReq1Valid = disabled
	 * VextSupply3HwHPReq1Valid = disabled
	 */
	INIT_REGULATOR_REGISTER(AB8500_REGUHWHPREQ1VALID2,     0x00),
	/*
	 * VanaHwHPReq2Valid        = disabled
	 * Vaux1HwHPReq2Valid       = disabled
	 * Vaux2HwHPReq2Valid       = disabled
	 * Vaux3HwHPReq2Valid       = disabled
	 */
	INIT_REGULATOR_REGISTER(AB8500_REGUHWHPREQ2VALID1,     0x00),
	/*
	 * VextSupply1HwHPReq2Valid = disabled
	 * VextSupply2HwHPReq2Valid = disabled
	 * VextSupply3HwHPReq2Valid = disabled
	 */
	INIT_REGULATOR_REGISTER(AB8500_REGUHWHPREQ2VALID2,     0x00),
	/*
	 * VanaSwHPReqValid         = disabled
	 * Vaux1SwHPReqValid        = disabled
	 */
	INIT_REGULATOR_REGISTER(AB8500_REGUSWHPREQVALID1,      0x00),
	/*
	 * Vaux2SwHPReqValid        = disabled
	 * Vaux3SwHPReqValid        = disabled
	 * VextSupply1SwHPReqValid  = disabled
	 * VextSupply2SwHPReqValid  = disabled
	 * VextSupply3SwHPReqValid  = disabled
	 */
	INIT_REGULATOR_REGISTER(AB8500_REGUSWHPREQVALID2,      0x00),
	/*
	 * SysClkReq2Valid1         = disabled
	 * SysClkReq3Valid1         = disabled
	 * SysClkReq4Valid1         = disabled
	 * SysClkReq5Valid1         = disabled
	 * SysClkReq6Valid1         = disabled
	 * SysClkReq7Valid1         = disabled
	 * SysClkReq8Valid1         = disabled
	 */
	INIT_REGULATOR_REGISTER(AB8500_REGUSYSCLKREQVALID1,    0x00),
	/*
	 * SysClkReq2Valid2         = disabled
	 * SysClkReq3Valid2         = disabled
	 * SysClkReq4Valid2         = disabled
	 * SysClkReq5Valid2         = disabled
	 * SysClkReq6Valid2         = disabled
	 * SysClkReq7Valid2         = disabled
	 * SysClkReq8Valid2         = disabled
	 */
	INIT_REGULATOR_REGISTER(AB8500_REGUSYSCLKREQVALID2,    0x00),
	/*
	 * VTVoutEna                = disabled
	 * Vintcore12Ena            = disabled
	 * Vintcore12Sel            = 1.25 V
	 * Vintcore12LP             = inactive (HP)
	 * VTVoutLP                 = inactive (HP)
	 */
	INIT_REGULATOR_REGISTER(AB8500_REGUMISC1,              0x10),
	/*
	 * VaudioEna                = disabled
	 * VdmicEna                 = disabled
	 * Vamic1Ena                = disabled
	 * Vamic2Ena                = disabled
	 */
	INIT_REGULATOR_REGISTER(AB8500_VAUDIOSUPPLY,           0x00),
	/*
	 * Vamic1_dzout             = high-Z when Vamic1 is disabled
	 * Vamic2_dzout             = high-Z when Vamic2 is disabled
	 */
	INIT_REGULATOR_REGISTER(AB8500_REGUCTRL1VAMIC,         0x00),
	/*
	 * Vsmps1Regu               = HW control
	 * Vsmps1SelCtrl            = Vsmps1 voltage defined by Vsmsp1Sel2
	 */
	INIT_REGULATOR_REGISTER(AB8500_VSMPS1REGU,             0x06),
	/*
	 * Vsmps2Regu               = HW control
	 * Vsmps2SelCtrl            = Vsmps2 voltage defined by Vsmsp2Sel2
	 */
	INIT_REGULATOR_REGISTER(AB8500_VSMPS2REGU,             0x06),
	/*
	 * VPll                     = Hw controlled
	 * VanaRegu                 = force off
	 */
	INIT_REGULATOR_REGISTER(AB8500_VPLLVANAREGU,           0x02),
	/*
	 * VrefDDREna               = disabled
	 * VrefDDRSleepMode         = inactive (no pulldown)
	 */
	INIT_REGULATOR_REGISTER(AB8500_VREFDDR,                0x00),
	/*
	 * No external regaultors connected to AB8500 in Godin
	 */
	INIT_REGULATOR_REGISTER(AB8500_EXTSUPPLYREGU,          0x00),
	/*
	 * Vaux1Regu                = force HP
	 * Vaux2Regu                = force HP
	 */
	INIT_REGULATOR_REGISTER(AB8500_VAUX12REGU,             0x05),
	/*
	 * Vrf1Regu                 = HW control
	 * Vaux3Regu                = force off
	 */
	INIT_REGULATOR_REGISTER(AB8500_VRF1VAUX3REGU,          0x08),
	/*
	 * Vsmps1                   = 1.2V
	 */
	INIT_REGULATOR_REGISTER(AB8500_VSMPS1SEL1,             0x28),
	/*
	 * Vaux1Sel                 = 3.0 V
	 */
	INIT_REGULATOR_REGISTER(AB8500_VAUX1SEL,               0x0E),
	/*
	 * Vaux2Sel                 = 1.8 V
	 */
	INIT_REGULATOR_REGISTER(AB8500_VAUX2SEL,               0x05),
	/*
	 * Vaux3Sel                 = 2.91 V
	 */
	INIT_REGULATOR_REGISTER(AB8500_VRF1VAUX3SEL,           0x07),
	/*
	 * VextSupply12LP           = disabled (no LP)
	 */
	INIT_REGULATOR_REGISTER(AB8500_REGUCTRL2SPARE,         0x00),
	/*
	 * Vaux1Disch               = short discharge time
	 * Vaux2Disch               = short discharge time
	 * Vaux3Disch               = short discharge time
	 * Vintcore12Disch          = short discharge time
	 * VTVoutDisch              = short discharge time
	 * VaudioDisch              = short discharge time
	 */
	INIT_REGULATOR_REGISTER(AB8500_REGUCTRLDISCH,          0x00),
	/*
	 * VanaDisch                = short discharge time
	 * VdmicPullDownEna         = pulldown disabled when Vdmic is disabled
	 * VdmicDisch               = short discharge time
	 */
	INIT_REGULATOR_REGISTER(AB8500_REGUCTRLDISCH2,         0x00),
};

/* vana regulator configuration, for analogue part of displays */
static struct regulator_consumer_supply ab8500_vana_consumers[] = {
	{
		.dev_name = "mcde",
		.supply = "v-ana",
	},
	{
		.dev_name = "mmio_camera",
		.supply = "v-ana",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.0",
		.supply = "ana",
	},
#endif
};

/* vaux1 regulator configuration */
static struct regulator_consumer_supply ab8500_vaux1_consumers[] = {
#if defined(CONFIG_ACCEL_MPU60X0)
	{
		.dev_name = MPU60X0_I2C_DEVICE_NAME,
		// NB: "v-accelerometer" is already taken
		.supply = MPU60X0_REGULATOR,
	},
#endif
#if defined(CONFIG_MPU_SENSORS_MPU3050)
	{
	        /* connected to i2c bus number 2, slave address 0x68*/
		.dev_name = "8-0068",
		.supply = "v_mpu_vdd",
	},
#endif
#if defined(CONFIG_PROXIMITY_GP2A)
	{
		.dev_name = GP2A_I2C_DEVICE_NAME,
		.supply = "v-prox-vcc",
	},
#endif
#if defined(CONFIG_ACCEL_BMA022)
	{
		.dev_name = "bma022",
		.supply = "v-accel-vdd",
	},
#endif
	{
		.dev_name = "mmio_camera",
		.supply = "v-mmio-camera",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.1",
		.supply = "aux1",
	},
#endif
#if 0
	{
		.dev_name = "3-005c",
		.supply = "avdd",
	},
	{
		.dev_name = "3-005d",
		.supply = "avdd",
	},
	{
		.dev_name = "3-004b",
		.supply = "vdd",
	},
#endif
};

/* vaux2 regulator configuration */
static struct regulator_consumer_supply ab8500_vaux2_consumers[] = {
#if defined(CONFIG_MPU_SENSORS_MPU3050)
	{
	        /* connected to i2c bus number 2, slave address 0x68*/
		.dev_name = "8-0068",
		.supply = "v_mpu_vlogic",
	},
#endif
#if defined(CONFIG_ACCEL_BMA022)
	{
		.dev_name = "bma022",
		.supply = "v-accel_vio",
	},
#endif
#if defined(CONFIG_PROXIMITY_GP2A)
	{
		.dev_name = GP2A_I2C_DEVICE_NAME,
		.supply = "v-prox_vio",
	},
#endif
#if defined(CONFIG_COMPASS_MMC3280)
	{
		.dev_name = "mmc3280",
		.supply = "v-compass",
	},
#endif
	/* Not sure if this is required maybe specific to MOP500 */
	{
		.dev_name = "ab8500-codec.0",
		.supply = "vcc-avswitch",
	},
	{
	       .dev_name = "ab8500-acc-det.0",
	       .supply = "vcc-avswitch"
	},

#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.2",
		.supply = "aux2",
	},
#endif
};

/* vaux3 regulator configuration */
static struct regulator_consumer_supply ab8500_vaux3_consumers[] = {
	{
		.dev_name = "sdi0",
		.supply = "v-MMC-SD"
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.3",
		.supply = "aux3",
	},
#endif
};

/* vtvout regulator configuration, supply for tvout, gpadc, TVOUT LDO */
static struct regulator_consumer_supply ab8500_vtvout_consumers[] = {
	{
		.supply = "ab8500-gpadc",
	},
	{
		.dev_name = "ab8500-charger.0",
		.supply = "vddadc"
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.4",
		.supply = "tvout",
	},
#endif
};
/* vaudio regulator configuration, supply for ab8500-vaudio, VAUDIO LDO */
static struct regulator_consumer_supply ab8500_vaudio_consumers[] = {
	{
		.supply = "v-audio",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.5",
		.supply = "audio",
	},
#endif
};

/* vamic1 regulator configuration */
static struct regulator_consumer_supply ab8500_vamic1_consumers[] = {
	{
		.supply = "v-amic1",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.6",
		.supply = "anamic1",
	},
#endif
};

/* supply for v-amic2, VAMIC2 LDO, reuse constants for AMIC1 */
static struct regulator_consumer_supply ab8500_vamic2_consumers[] = {
	{
		.supply = "v-amic2",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.7",
		.supply = "anamic2",
	},
#endif
};

/* supply for v-dmic, VDMIC LDO */
static struct regulator_consumer_supply ab8500_vdmic_consumers[] = {
	{
		.supply = "v-dmic",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.8",
		.supply = "dmic",
	},
#endif
};

/* supply for v-intcore12, VINTCORE12 LDO */
static struct regulator_consumer_supply ab8500_vintcore_consumers[] = {
	{
		.supply = "v-intcore",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.9",
		.supply = "intcore",
	},
#endif
	{
		.dev_name = "ab8500-usb.0",
		.supply = "v-intcore",
	},

};

/*
 * AB8500 regulators
 */
struct regulator_init_data janice_regulators[AB8500_NUM_REGULATORS] = {
	/* supplies to the Sensors 3V */
	/* 'REGULATOR_CHANGE_MODE' ops added to support LP mode */
	[AB8500_LDO_AUX1] = {
		.constraints = {
			.name = "ab8500-vaux1",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS |
					REGULATOR_CHANGE_MODE,
			.valid_modes_mask = REGULATOR_MODE_NORMAL |
					REGULATOR_MODE_IDLE,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8500_vaux1_consumers),
		.consumer_supplies = ab8500_vaux1_consumers,
	},
	/* supplies to the Sensors IO 1V8  */
	/* 'REGULATOR_CHANGE_MODE' ops added to support LP mode */
	[AB8500_LDO_AUX2] = {
		.constraints = {
			.name = "ab8500-vaux2",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS |
					REGULATOR_CHANGE_MODE,
			.valid_modes_mask = REGULATOR_MODE_NORMAL |
					REGULATOR_MODE_IDLE,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8500_vaux2_consumers),
		.consumer_supplies = ab8500_vaux2_consumers,
	},
	/* supply for VAUX3, supplies to SDcard slots */
	[AB8500_LDO_AUX3] = {
		.constraints = {
			.name = "ab8500-vaux3",
			.min_uV = 1200000,
			.max_uV = 2910000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					REGULATOR_CHANGE_STATUS |
					REGULATOR_CHANGE_MODE,
			.valid_modes_mask = REGULATOR_MODE_NORMAL |
					REGULATOR_MODE_IDLE,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8500_vaux3_consumers),
		.consumer_supplies = ab8500_vaux3_consumers,
	},
	/* supply for v-intcore12, VINTCORE12 LDO */
	[AB8500_LDO_INTCORE] = {
		.constraints = {
			.name = "ab8500-vintcore",
			.min_uV = 1250000,
			.max_uV = 1350000,
			.input_uV = 1800000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
					REGULATOR_CHANGE_STATUS |
					REGULATOR_CHANGE_MODE |
					REGULATOR_CHANGE_DRMS,
			.valid_modes_mask = REGULATOR_MODE_NORMAL |
					REGULATOR_MODE_IDLE,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8500_vintcore_consumers),
		.consumer_supplies = ab8500_vintcore_consumers,
	},
	/* supply for tvout, gpadc, TVOUT LDO */
	[AB8500_LDO_TVOUT] = {
		.constraints = {
			.name = "ab8500-vtvout",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8500_vtvout_consumers),
		.consumer_supplies = ab8500_vtvout_consumers,
	},
	/* supply for ab8500-vaudio, VAUDIO LDO */
	[AB8500_LDO_AUDIO] = {
		.constraints = {
			.name = "ab8500-vaudio",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8500_vaudio_consumers),
		.consumer_supplies = ab8500_vaudio_consumers,
	},
	/* supply for v-anamic1 VAMic1-LDO */
	[AB8500_LDO_ANAMIC1] = {
		.constraints = {
			.name = "ab8500-vamic1",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8500_vamic1_consumers),
		.consumer_supplies = ab8500_vamic1_consumers,
	},
	/* supply for v-amic2, VAMIC2 LDO, reuse constants for AMIC1 */
	[AB8500_LDO_ANAMIC2] = {
		.constraints = {
			.name = "ab8500-vamic2",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8500_vamic2_consumers),
		.consumer_supplies = ab8500_vamic2_consumers,
	},
	/* supply for v-dmic, VDMIC LDO */
	[AB8500_LDO_DMIC] = {
		.constraints = {
			.name = "ab8500-vdmic",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8500_vdmic_consumers),
		.consumer_supplies = ab8500_vdmic_consumers,
	},
	/* supply for U8500 CSI/DSI, VANA LDO */
	[AB8500_LDO_ANA] = {
		.constraints = {
			.name = "ab8500-vana",
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(ab8500_vana_consumers),
		.consumer_supplies = ab8500_vana_consumers,
	},
};

/*
 * Power State Regulator Configuration
 */
#define U8500_VAPE_REGULATOR_MIN_VOLTAGE	1800000
#define U8500_VAPE_REGULATOR_MAX_VOLTAGE	2000000

/* vape regulator configuration */
static struct regulator_consumer_supply u8500_vape_consumers[] = {
	{
		.supply = "v-ape",
	},
	{
		.dev_name = "nmk-i2c.0",
		.supply = "v-i2c",
	},
	{
		.dev_name = "nmk-i2c.1",
		.supply = "v-i2c",
	},
	{
		.dev_name = "nmk-i2c.2",
		.supply = "v-i2c",
	},
	{
		.dev_name = "nmk-i2c.3",
		.supply = "v-i2c",
	},
	{
		.dev_name = "sdi0",
		.supply = "v-mmc",
	},
	{
		.dev_name = "sdi1",
		.supply = "v-mmc",
	},
	{
		.dev_name = "sdi2",
		.supply = "v-mmc",
	},
	{
		.dev_name = "sdi4",
		.supply = "v-mmc",
	},
	{
		.dev_name = "dma40.0",
		.supply = "v-dma",
	},
	{
		.dev_name = "ab8500-usb.0",
		.supply = "v-ape",
	},
	{
		.dev_name = "uart0",
		.supply = "v-uart",
	},
	{
		.dev_name = "uart1",
		.supply = "v-uart",
	},
	{
		.dev_name = "uart2",
		.supply = "v-uart",
	},
	{
		.dev_name = "ste_hsi.0",
		.supply = "v-hsi",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.10",
		.supply = "test",
	},
#endif
};

struct regulator_init_data janice_u8500_vape_regulator = {
	.constraints = {
		.name = "u8500-vape",
		.min_uV = U8500_VAPE_REGULATOR_MIN_VOLTAGE,
		.max_uV = U8500_VAPE_REGULATOR_MAX_VOLTAGE,
		.input_uV = 1, /* notional, for set_mode* */
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_DRMS |
			REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL |
			REGULATOR_MODE_IDLE,
	},
	.consumer_supplies = u8500_vape_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_vape_consumers),
};

/* varm regulator_configuration */
static struct regulator_consumer_supply u8500_varm_consumers[] = {
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.11",
		.supply = "test",
	},
#endif
};

struct regulator_init_data janice_u8500_varm_regulator = {
	.constraints = {
		.name = "u8500-varm",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_varm_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_varm_consumers),
};

/* vmodem regulator configuration */
static struct regulator_consumer_supply u8500_vmodem_consumers[] = {
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.12",
		.supply = "test",
	},
#endif
};

struct regulator_init_data janice_u8500_vmodem_regulator = {
	.constraints = {
		.name = "u8500-vmodem",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_vmodem_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_vmodem_consumers),
};

/* vpll regulator configuration */
static struct regulator_consumer_supply u8500_vpll_consumers[] = {
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.13",
		.supply = "test",
	},
#endif
};

struct regulator_init_data janice_u8500_vpll_regulator = {
	.constraints = {
		.name = "u8500-vpll",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_vpll_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_vpll_consumers),
};

/* vsmps1 regulator configuration */
static struct regulator_consumer_supply u8500_vsmps1_consumers[] = {
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.14",
		.supply = "test",
	},
#endif
};

struct regulator_init_data janice_u8500_vsmps1_regulator = {
	.constraints = {
		.name = "u8500-vsmps1",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_vsmps1_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_vsmps1_consumers),
};

/* vsmsp2 regulator configuration */
static struct regulator_consumer_supply u8500_vsmps2_consumers[] = {
	{
		.dev_name = "ab8500-usb.0",
		.supply = "musb_1v8",
	},
	{
		.dev_name = "0-0070",
		.supply = "hdmi_1v8",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.15",
		.supply = "test",
	},
#endif
};

struct regulator_init_data janice_u8500_vsmps2_regulator = {
	.constraints = {
		.name = "u8500-vsmps2",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = 1, //TODO: Check why this is needed
/*		.valid_modes_mask = REGULATOR_MODE_NORMAL |
			REGULATOR_MODE_IDLE, */
	},
	.consumer_supplies = u8500_vsmps2_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_vsmps2_consumers),
};

/* vsmps3 regulator configuration */
static struct regulator_consumer_supply u8500_vsmps3_consumers[] = {
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.16",
		.supply = "test",
	},
#endif
};

struct regulator_init_data janice_u8500_vsmps3_regulator = {
	.constraints = {
		.name = "u8500-vsmps3",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_vsmps3_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_vsmps3_consumers),
};

/* vrf1 regulator configuration */
static struct regulator_consumer_supply u8500_vrf1_consumers[] = {
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.17",
		.supply = "test",
	},
#endif
};

struct regulator_init_data janice_u8500_vrf1_regulator = {
	.constraints = {
		.name = "u8500-vrf1",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_vrf1_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_vrf1_consumers),
};

/*
 * Power Domain Switch Configuration
 */

/* SVA MMDSP regulator switch */
static struct regulator_consumer_supply u8500_svammdsp_consumers[] = {
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-sva-mmdsp",
	},
	{
		.dev_name = "cm_control",
		.supply = "sva-mmdsp",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.18",
		.supply = "test",
	},
#endif
};

struct regulator_init_data janice_u8500_svammdsp_regulator = {
	/* dependency to u8500-vape is handled outside regulator framework */
	.constraints = {
		.name = "u8500-sva-mmdsp",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_svammdsp_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_svammdsp_consumers),
};

/* SVA MMDSP retention regulator switch */
static struct regulator_consumer_supply u8500_svammdspret_consumers[] = {
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-sva-mmdsp-ret",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.19",
		.supply = "test",
	},
#endif
};

struct regulator_init_data janice_u8500_svammdspret_regulator = {
	.constraints = {
		.name = "u8500-sva-mmdsp-ret",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_svammdspret_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_svammdspret_consumers),
};

/* SVA pipe regulator switch */
static struct regulator_consumer_supply u8500_svapipe_consumers[] = {
	/* Add SVA pipe device supply here */
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-sva-pipe",
	},
	{
		.dev_name = "cm_control",
		.supply = "sva-pipe",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.20",
		.supply = "test",
	},
#endif
};

struct regulator_init_data janice_u8500_svapipe_regulator = {
	/* dependency to u8500-vape is handled outside regulator framework */
	.constraints = {
		.name = "u8500-sva-pipe",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_svapipe_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_svapipe_consumers),
};

/* SIA MMDSP regulator switch */
static struct regulator_consumer_supply u8500_siammdsp_consumers[] = {
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-sia-mmdsp",
	},
	{
		.dev_name = "cm_control",
		.supply = "sia-mmdsp",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.21",
		.supply = "test",
	},
#endif
};

struct regulator_init_data janice_u8500_siammdsp_regulator = {
	/* dependency to u8500-vape is handled outside regulator framework */
	.constraints = {
		.name = "u8500-sia-mmdsp",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_siammdsp_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_siammdsp_consumers),
};

/* SIA MMDSP retention regulator switch */
static struct regulator_consumer_supply u8500_siammdspret_consumers[] = {
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-sia-mmdsp-ret",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.22",
		.supply = "test",
	},
#endif
};

struct regulator_init_data janice_u8500_siammdspret_regulator = {
	.constraints = {
		.name = "u8500-sia-mmdsp-ret",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_siammdspret_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_siammdspret_consumers),
};

/* SIA pipe regulator switch */
static struct regulator_consumer_supply u8500_siapipe_consumers[] = {
	/* Add SIA pipe device supply here */
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-sia-pipe",
	},
	{
		.dev_name = "cm_control",
		.supply = "sia-pipe",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.23",
		.supply = "test",
	},
#endif
};

struct regulator_init_data janice_u8500_siapipe_regulator = {
	/* dependency to u8500-vape is handled outside regulator framework */
	.constraints = {
		.name = "u8500-sia-pipe",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_siapipe_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_siapipe_consumers),
};

/* SGA regulator switch */
static struct regulator_consumer_supply u8500_sga_consumers[] = {
	/* Add SGA device supply here */
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-sga",
	},
	{
		/*
		 * The Mali driver doesn't have access to the device when
		 * requesting the SGA regulator. Therefore only supply name.
		 */
		.supply = "v-mali",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.24",
		.supply = "test",
	},
#endif
};

struct regulator_init_data janice_u8500_sga_regulator = {
	.supply_regulator = "u8500-vape",
	.constraints = {
		.name = "u8500-sga",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_sga_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_sga_consumers),
};

/* B2R2-MCDE regulator switch */
static struct regulator_consumer_supply u8500_b2r2_mcde_consumers[] = {
	{
		.dev_name = "b2r2_bus",
		.supply = "vsupply",
	},
	{
		.dev_name = "mcde",
		.supply = "vsupply",
	},
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-b2r2",
	},
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-mcde",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.25",
		.supply = "test",
	},
#endif
};

struct regulator_init_data janice_u8500_b2r2_mcde_regulator = {
	.supply_regulator = "u8500-vape",
	.constraints = {
		.name = "u8500-b2r2-mcde",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_b2r2_mcde_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_b2r2_mcde_consumers),
};

/* ESRAM1 and 2 regulator switch */
static struct regulator_consumer_supply u8500_esram12_consumers[] = {
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-esram1",
	},
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-esram2",
	},
	{
		.dev_name = "cm_control",
		.supply = "esram12",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.26",
		.supply = "test",
	},
#endif
};

struct regulator_init_data janice_u8500_esram12_regulator = {
	/*
	 * esram12 is set in retention and supplied by Vsafe when Vape is off,
	 * no need to hold Vape
	 */
	.constraints = {
		.name = "u8500-esram12",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_esram12_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_esram12_consumers),
};

/* ESRAM1 and 2 retention regulator switch */
static struct regulator_consumer_supply u8500_esram12ret_consumers[] = {
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-esram1-ret",
	},
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-esram2-ret",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.27",
		.supply = "test",
	},
#endif
};

struct regulator_init_data janice_u8500_esram12ret_regulator = {
	.constraints = {
		.name = "u8500-esram12-ret",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_esram12ret_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_esram12ret_consumers),
};

/* ESRAM3 and 4 regulator switch */
static struct regulator_consumer_supply u8500_esram34_consumers[] = {
	{
		.dev_name = "mcde",
		.supply = "v-esram34",
	},
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-esram3",
	},
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-esram4",
	},
	{
		.dev_name = "cm_control",
		.supply = "esram34",
	},
	{
		.dev_name = "dma40.0",
		.supply = "lcla_esram",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.28",
		.supply = "test",
	},
#endif
};

struct regulator_init_data janice_u8500_esram34_regulator = {
	/*
	 * esram34 is set in retention and supplied by Vsafe when Vape is off,
	 * no need to hold Vape
	 */
	.constraints = {
		.name = "u8500-esram34",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_esram34_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_esram34_consumers),
};

/* ESRAM3 and 4 retention regulator switch */
static struct regulator_consumer_supply u8500_esram34ret_consumers[] = {
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-esram3-ret",
	},
	{
		/* NOTE! This is a temporary supply for prcmu_set_hwacc */
		.supply = "hwacc-esram4-ret",
	},
#ifdef CONFIG_U8500_REGULATOR_DEBUG
	{
		.dev_name = "reg-virt-consumer.29",
		.supply = "test",
	},
#endif
};

struct regulator_init_data janice_u8500_esram34ret_regulator = {
	.constraints = {
		.name = "u8500-esram34-ret",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = u8500_esram34ret_consumers,
	.num_consumer_supplies = ARRAY_SIZE(u8500_esram34ret_consumers),
};
