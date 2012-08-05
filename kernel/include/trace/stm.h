/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * ST-Ericsson STM Trace driver
 *
 * Author: Pierre Peiffer <pierre.peiffer@stericsson.com> for ST-Ericsson.
 * License terms:  GNU General Public License (GPL), version 2.
 */

#ifndef STM_H
#define STM_H

#define STM_DEV_NAME "stm"
#define STM_NUMBER_OF_CHANNEL   256

/* One single channel mapping */
struct stm_channel {
	union {
		uint8_t  no_stamp8;
		uint16_t no_stamp16;
		uint32_t no_stamp32;
		uint64_t no_stamp64;
	};
	union {
		uint8_t  stamp8;
		uint16_t stamp16;
		uint32_t stamp32;
		uint64_t stamp64;
	};
};

/* Mapping of all channels */
struct stm_channels {
	struct stm_channel channel[STM_NUMBER_OF_CHANNEL];
};

/* Possible clock setting */
enum clock_div {
	STM_CLOCK_DIV2 = 0,
	STM_CLOCK_DIV4,
	STM_CLOCK_DIV6,
	STM_CLOCK_DIV8,
	STM_CLOCK_DIV10,
	STM_CLOCK_DIV12,
	STM_CLOCK_DIV14,
	STM_CLOCK_DIV16,
};

/* Possible enabler */
#define STM_ENABLE_ARM		BIT(0)
#define STM_ENABLE_SVA		BIT(2)
#define STM_ENABLE_SIA		BIT(3)
#define STM_ENABLE_SIA_XP70	BIT(4)
#define STM_ENABLE_PRCMU	BIT(5)
#define STM_ENABLE_MCSBAG	BIT(9)

/* ioctl commands */
#define STM_DISABLE               _IO('t', 0)
#define STM_SET_CLOCK_DIV         _IOW('t', 1, enum clock_div)
#define STM_GET_CTRL_REG          _IOR('t', 2, int)
#define STM_ENABLE_SRC            _IOW('t', 3, int)
#define STM_ENABLE_MIPI34_MODEM   _IOW('t', 4, int)
#define STM_DISABLE_MIPI34_MODEM  _IOW('t', 5, int)
#define STM_GET_FREE_CHANNEL      _IOR('t', 6, int)
#define STM_RELEASE_CHANNEL       _IO('t', 7)

#ifdef __KERNEL__

struct stm_platform_data {
	void (*ste_enable_modem_on_mipi34)(void);
	void (*ste_disable_modem_on_mipi34)(void);
	int (*ste_gpio_enable_mipi34) (void);
	int (*ste_gpio_disable_mipi34) (void);
	int (*ste_gpio_enable_ape_modem_mipi60) (void);
	int (*ste_gpio_disable_ape_modem_mipi60) (void);
	int (*ste_gpio_enable_stm_ape_microsd)(void);
	int (*ste_gpio_disable_stm_ape_microsd)(void);
	int (*ste_gpio_enable_stm_modem_microsd)(void);
	int (*ste_gpio_disable_stm_modem_microsd)(void);
	int (*ste_enable_power_for_microsd)(void);
	int (*ste_disable_power_for_microsd)(void);
	const u32 regs_phys_base;
	const u32 channels_phys_base;
	const u32 periph_id;
	const u32 cell_id;
	s16 channels_reserved[4];
};

#ifdef CONFIG_STM_TRACE
/* Disbale all cores */
void stm_disable_src(void);
/* Set clock speed */
int stm_set_ckdiv(unsigned int v);
/* Return the control register */
unsigned int stm_get_cr(void);
/* Enable the trace */
int stm_enable_src(unsigned int v);

#define DECLLLTFUN(size) \
	void stm_trace_##size(unsigned char channel, uint##size##_t data); \
	void stm_tracet_##size(unsigned char channel, uint##size##_t data)
#else
static inline void stm_disable_src(void) {};
static inline int stm_set_ckdiv(unsigned int v) { return 0; };
static inline unsigned int stm_get_cr(void) { return 0; };
static inline int stm_enable_src(unsigned int v) { return 0; };

#define DECLLLTFUN(size) \
	static inline void stm_trace_##size(unsigned char channel,  \
					    uint##size##_t data) {};\
	static inline void stm_tracet_##size(unsigned char channel, \
					     uint##size##_t data) {}
#endif /* CONFIG_STM_TRACE */

/* Provides stm_trace_XX() and stm_tracet_XX() trace API */
DECLLLTFUN(8);
DECLLLTFUN(16);
DECLLLTFUN(32);
DECLLLTFUN(64);

#endif /* __KERNEL__ */

#endif /* STM_H */
