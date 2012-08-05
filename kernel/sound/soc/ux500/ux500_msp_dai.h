/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Ola Lilja <ola.o.lilja@stericsson.com>,
 *         Roger Nilsson <roger.xr.nilsson@stericsson.com>
 *         for ST-Ericsson.
 *
 * License terms:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#ifndef UX500_msp_dai_H
#define UX500_msp_dai_H

#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/i2s/i2s.h>
#include <mach/msp.h>

#define UX500_NBR_OF_DAI	4

#define UX500_I2S_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |	\
			SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000)

#define UX500_I2S_FORMATS (SNDRV_PCM_FMTBIT_S16_LE)

#define FRAME_PER_SINGLE_SLOT_8_KHZ		31
#define FRAME_PER_SINGLE_SLOT_16_KHZ	124
#define FRAME_PER_SINGLE_SLOT_44_1_KHZ	63
#define FRAME_PER_SINGLE_SLOT_48_KHZ	49
#define FRAME_PER_2_SLOTS				31
#define FRAME_PER_8_SLOTS				138
#define FRAME_PER_16_SLOTS				277
#ifndef CONFIG_SND_SOC_UX500_AB5500
#define UX500_MSP_INTERNAL_CLOCK_FREQ 40000000
#else
#define UX500_MSP_INTERNAL_CLOCK_FREQ 13000000
#endif
#define UX500_MSP_MIN_CHANNELS		1
#define UX500_MSP_MAX_CHANNELS		8

#define PLAYBACK_CONFIGURED		1
#define CAPTURE_CONFIGURED		2

enum ux500_msp_clock_id {
	UX500_MSP_MASTER_CLOCK,
};

struct ux500_platform_drvdata {
	struct i2s_device *i2s;
	unsigned int fmt;
	unsigned int tx_mask;
	unsigned int rx_mask;
	int slots;
	int slot_width;
	bool playback_active;
	bool capture_active;
	u8 configured;
	int data_delay;
	unsigned int master_clk;
};

extern struct snd_soc_dai ux500_msp_dai[UX500_NBR_OF_DAI];

bool ux500_msp_dai_i2s_get_underrun_status(int dai_idx);
dma_addr_t ux500_msp_dai_i2s_get_pointer(int dai_idx, int stream_id);
int ux500_msp_dai_i2s_configure_sg(dma_addr_t dma_addr,
				int sg_len,
				int sg_size,
				int dai_idx,
				int stream_id);
int ux500_msp_dai_i2s_send_data(void *data, size_t bytes, int dai_idx);
int ux500_msp_dai_i2s_receive_data(void *data, size_t bytes, int dai_idx);

int ux500_msp_dai_set_data_delay(struct snd_soc_dai *dai, int delay);

#endif
