/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License Terms: GNU General Public License v2
 *
 * Authors: Sundar Iyer <sundar.iyer@stericsson.com> for ST-Ericsson
 *          Bengt Jonsson <bengt.g.jonsson@stericsson.com> for ST-Ericsson
 */

#ifndef __LINUX_MFD_AB8500_REGULATOR_H
#define __LINUX_MFD_AB8500_REGULATOR_H

#include <linux/regulator/driver.h>

/* AB8500 regulators */
enum ab8500_regulator_id {
	AB8500_LDO_AUX1,
	AB8500_LDO_AUX2,
	AB8500_LDO_AUX3,
	AB8500_LDO_INTCORE,
	AB8500_LDO_TVOUT,
	AB8500_LDO_AUDIO,
	AB8500_LDO_ANAMIC1,
	AB8500_LDO_ANAMIC2,
	AB8500_LDO_DMIC,
	AB8500_LDO_ANA,
	AB8500_NUM_REGULATORS,
};

/* AB8500 register initialization */
struct ab8500_regulator_reg_init {
	int id;
	u8 value;
};

#define INIT_REGULATOR_REGISTER(_id, _value)	\
	{					\
		.id = _id,			\
		.value = _value,		\
	}

/* AB8500 registers */
enum ab8500_regulator_reg {
	AB8500_REGUREQUESTCTRL2,
	AB8500_REGUREQUESTCTRL3,
	AB8500_REGUREQUESTCTRL4,
	AB8500_REGUSYSCLKREQ1HPVALID1,
	AB8500_REGUSYSCLKREQ1HPVALID2,
	AB8500_REGUHWHPREQ1VALID1,
	AB8500_REGUHWHPREQ1VALID2,
	AB8500_REGUHWHPREQ2VALID1,
	AB8500_REGUHWHPREQ2VALID2,
	AB8500_REGUSWHPREQVALID1,
	AB8500_REGUSWHPREQVALID2,
	AB8500_REGUSYSCLKREQVALID1,
	AB8500_REGUSYSCLKREQVALID2,
	AB8500_REGUMISC1,
	AB8500_VAUDIOSUPPLY,
	AB8500_REGUCTRL1VAMIC,
	AB8500_VPLLVANAREGU,
	AB8500_VREFDDR,
	AB8500_EXTSUPPLYREGU,
	AB8500_VAUX12REGU,
	AB8500_VRF1VAUX3REGU,
	AB8500_VSMPS1SEL2,
	AB8500_VSMPS1SEL3,
	AB8500_VSMPS2SEL1,
	AB8500_VSMPS2SEL2,
	AB8500_VSMPS2SEL3,
	AB8500_VAUX1SEL,
	AB8500_VAUX2SEL,
	AB8500_VRF1VAUX3SEL,
	AB8500_REGUCTRL2SPARE,
	AB8500_REGUCTRLDISCH,
	AB8500_REGUCTRLDISCH2,
	AB8500_VSMPS1REGU,
	AB8500_VSMPS2REGU,
	AB8500_VSMPS1SEL1,
	AB8500_NUM_REGULATOR_REGISTERS,
};

/**
 * struct ab8500_regulator_info - ab8500 regulator information
 * @dev: device pointer
 * @desc: regulator description
 * @regulator_dev: regulator device
 * @is_enabled: status of regulator (on/off)
 * @max_uV: maximum voltage (for variable voltage supplies)
 * @min_uV: minimum voltage (for variable voltage supplies)
 * @fixed_uV: typical voltage (for fixed voltage supplies)
 * @load_lp_uA: maximum load in idle (low power) mode
 * @update_bank: bank to control on/off
 * @update_reg: register to control on/off
 * @update_mask: mask to enable/disable and set mode of regulator
 * @update_val: bits holding the regulator current mode
 * @update_val_idle: bits to enable the regulator in idle (low power) mode
 * @update_val_normal: bits to enable the regulator in normal (high power) mode
 * @voltage_bank: bank to control regulator voltage
 * @voltage_reg: register to control regulator voltage
 * @voltage_mask: mask to control regulator voltage
 * @voltages: supported voltage table
 * @voltages_len: number of supported voltages for the regulator
 * @delay: startup delay in ms
 */
struct ab8500_regulator_info {
	struct device		*dev;
	struct regulator_desc	desc;
	struct regulator_dev	*regulator;
	bool is_enabled;
	int max_uV;
	int min_uV;
	int fixed_uV;
	int load_lp_uA;
	u8 update_bank;
	u8 update_reg;
	u8 update_mask;
	u8 update_val;
	u8 update_val_idle;
	u8 update_val_normal;
	u8 voltage_bank;
	u8 voltage_reg;
	u8 voltage_mask;
	int const *voltages;
	int voltages_len;
	unsigned int delay;
};

#ifdef CONFIG_SAMSUNG_PANIC_DISPLAY_DEVICES
void ab8500_panic_regulator_volt_mode(struct regulator_ops *ops);

void ab8500_panic_regulator_mode(struct regulator_ops *ops);

void ab8500_panic_regulator(struct regulator_ops *ops);
#endif

#endif
