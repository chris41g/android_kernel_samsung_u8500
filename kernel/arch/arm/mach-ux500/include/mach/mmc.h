/*
 * Overview:
 *  	 SD/EMMC driver for u8500 platform
 *
 * Copyright (C) 2009 ST-Ericsson SA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/*-------------------------------------------------------------------------
 * SDI controller configuration
 * Kernel entry point for sdmmc/emmc chip
 *-------------------------------------------------------------------------
 * <VERSION>v1.0.0
 *-------------------------------------------------------------------------
 */
/*------------------------------i-----------------------------------------*/

#ifndef MACH_MMC_H
#define MACH_MMC_H

#include <linux/dmaengine.h>

/*
 * SDI Power register offset
 */
#define MMCIPOWER		0x000
/*
 * SDI Power register bits
 */
#define MCI_PWR_OFF		0x00
#define MCI_PWR_UP		0x02
#define MCI_PWR_ON		0x03
#define MCI_STATE_ENABLE	0x38
#define MCI_OPEN_DRAIN		(1 << 6)
#define MCI_FBCLK_ENABLE	(1 << 7)
#define MCI_POWER_IOS_MASK	\
	(MCI_PWR_ON | MCI_DIREN_CMD | MCI_DIREN_DAT0 | MCI_DIREN_DAT2 | \
	 MCI_DIREN_DAT31 | MCI_DIREN_DAT74 | MCI_OPEN_DRAIN)
#define MCI_DIREN_CMD		(1<<3)
#define MCI_DIREN_DAT0		(1<<4)
#define MCI_DIREN_DAT2		(1<<2)
#define MCI_DIREN_DAT31		(1<<5)
#define MCI_DIREN_DAT74		(1<<8)
#define MCI_DIREN_1BIT		(MCI_DIREN_CMD|MCI_DIREN_DAT0)
#define MCI_DIREN_4BIT		(MCI_DIREN_CMD|MCI_DIREN_DAT0|MCI_DIREN_DAT31)
/* #define MCI_DIREN_8BIT	(MCI_DIREN_CMD|MCI_DIREN_DAT0|MCI_DIREN_DAT31)*/
#define MCI_DIREN_BIT		(MCI_DIREN_CMD|MCI_DIREN_DAT0|MCI_DIREN_DAT31|\
				 MCI_DIREN_DAT2|MCI_DIREN_DAT74)
/*
 * SDI Clock register offset
 */
#define MMCICLOCK		0x004
/*
 * SDI Power register bits
 */
#define MCI_CLK_ENABLE		(1 << 8)
#define MCI_CLK_PWRSAVE		(1 << 9)
#define MCI_CLK_BYPASS		(1 << 10)
#define MCI_BUS_WIDTH_1		(0 << 11)
#define MCI_BUS_WIDTH_4		(1 << 11)
#define MCI_BUS_WIDTH_8		(2 << 11)
#define MCI_HWFC_EN		(1 << 14)
#define MCI_NEG_EDGE            (1 << 13)
/*
 * SDI Arguement register offset
 */
#define MMCIARGUMENT		0x008
/*
 * SDI Command register offset
 */
#define MMCICOMMAND		0x00c
/*
 * SDI command register bits
 */
#define MCI_CPSM_RESPONSE	(1 << 6)
#define MCI_CPSM_LONGRSP	(1 << 7)
#define MCI_CPSM_INTERRUPT	(1 << 8)
#define MCI_CPSM_PENDING	(1 << 9)
#define MCI_CPSM_ENABLE		(1 << 10)

/*
 * SDI RespCMD register offset
 */
#define MMCIRESPCMD		0x010
/*
 * SDI Response0 register offset
 */
#define MMCIRESPONSE0		0x014
/*
 * SDI Response1 register offset
 */
#define MMCIRESPONSE1		0x018
/*
 * SDI Response2 register offset
 */
#define MMCIRESPONSE2		0x01c
/*
 * SDI Response3 register offset
 */
#define MMCIRESPONSE3		0x020
/*
 * SDI Datatimer register offset
 */
#define MMCIDATATIMER		0x024
/*
 * SDI DataLength register offset
 */
#define MMCIDATALENGTH		0x028
/*
 * SDI Data control register offset
 */
#define MMCIDATACTRL		0x02c
/*
 * SDI Data control register bits
 */
#define MCI_DPSM_ENABLE		(1 << 0)
#define MCI_DPSM_DIRECTION	(1 << 1)
#define MCI_DPSM_MODE		(1 << 2)
#define MCI_DPSM_DMAENABLE	(1 << 3)
#define MCI_DPSM_DMAreqctl	(1 << 12)
#define MCI_SDIO_ENABLE		(1 << 11)

/*
 * SDI Data Count register offset
 */
#define MMCIDATACNT		0x030
/*
 * SDI Status register offset
 */
#define MMCISTATUS		0x034
/*
 * SDI Status register bits
 */
#define MCI_CMDCRCFAIL		(1 << 0)
#define MCI_DATACRCFAIL		(1 << 1)
#define MCI_CMDTIMEOUT		(1 << 2)
#define MCI_DATATIMEOUT		(1 << 3)
#define MCI_TXUNDERRUN		(1 << 4)
#define MCI_RXOVERRUN		(1 << 5)
#define MCI_CMDRESPEND		(1 << 6)
#define MCI_CMDSENT		(1 << 7)
#define MCI_DATAEND		(1 << 8)
#define MCI_STBITERR	 	(1 << 9)
#define MCI_DATABLOCKEND	(1 << 10)
#define MCI_CMDACTIVE		(1 << 11)
#define MCI_TXACTIVE		(1 << 12)
#define MCI_RXACTIVE		(1 << 13)
#define MCI_TXFIFOHALFEMPTY	(1 << 14)
#define MCI_RXFIFOHALFFULL	(1 << 15)
#define MCI_TXFIFOFULL		(1 << 16)
#define MCI_RXFIFOFULL		(1 << 17)
#define MCI_TXFIFOEMPTY		(1 << 18)
#define MCI_RXFIFOEMPTY		(1 << 19)
#define MCI_TXDATAAVLBL		(1 << 20)
#define MCI_RXDATAAVLBL		(1 << 21)
#define MCI_SDIOIT		(1 << 22)
/*
 * SDI Clear register offset
 */
#define MMCICLEAR		0x038
#define MCI_CMDCRCFAILCLR	(1 << 0)
#define MCI_DATACRCFAILCLR	(1 << 1)
#define MCI_CMDTIMEOUTCLR	(1 << 2)
#define MCI_DATATIMEOUTCLR	(1 << 3)
#define MCI_TXUNDERRUNCLR	(1 << 4)
#define MCI_RXOVERRUNCLR	(1 << 5)
#define MCI_CMDRESPENDCLR	(1 << 6)
#define MCI_CMDSENTCLR		(1 << 7)
#define MCI_DATAENDCLR		(1 << 8)
#define MCI_DATABLOCKENDCLR	(1 << 10)
#define MCI_SDIOITCLR		(1 << 22)
/*
 * SDI Mask register offset
 */
#define MMCIMASK0		0x03c
/*
 * SDI Mask register bits
 */
#define MCI_CMDCRCFAILMASK	(1 << 0)
#define MCI_DATACRCFAILMASK	(1 << 1)
#define MCI_CMDTIMEOUTMASK	(1 << 2)
#define MCI_DATATIMEOUTMASK	(1 << 3)
#define MCI_TXUNDERRUNMASK	(1 << 4)
#define MCI_RXOVERRUNMASK	(1 << 5)
#define MCI_CMDRESPENDMASK	(1 << 6)
#define MCI_CMDSENTMASK		(1 << 7)
#define MCI_DATAENDMASK		(1 << 8)
#define MCI_DATABLOCKENDMASK	(1 << 10)
#define MCI_CMDACTIVEMASK	(1 << 11)
#define MCI_TXACTIVEMASK	(1 << 12)
#define MCI_RXACTIVEMASK	(1 << 13)
#define MCI_TXFIFOHALFEMPTYMASK	(1 << 14)
#define MCI_RXFIFOHALFFULLMASK	(1 << 15)
#define MCI_TXFIFOFULLMASK	(1 << 16)
#define MCI_RXFIFOFULLMASK	(1 << 17)
#define MCI_TXFIFOEMPTYMASK	(1 << 18)
#define MCI_RXFIFOEMPTYMASK	(1 << 19)
#define MCI_TXDATAAVLBLMASK	(1 << 20)
#define MCI_RXDATAAVLBLMASK	(1 << 21)
#define MCI_SDIOITMASK		(1 << 22)

#define MMCIMASK1		0x040
#define MMCIFIFOCNT		0x048
#define MMCIFIFO		0x080	/* to 0x0bc */

#define MCI_DATA_ERR	\
			(MCI_RXOVERRUN | MCI_TXUNDERRUN | MCI_DATATIMEOUT | \
			 MCI_DATACRCFAIL | MCI_STBITERR)
#define MCI_IRQENABLE	\
			(MCI_CMDCRCFAIL | MCI_DATACRCFAIL | MCI_CMDTIMEOUT | \
			 MCI_DATATIMEOUT | MCI_TXUNDERRUN | MCI_RXOVERRUN | \
			 MCI_CMDRESPEND | MCI_CMDSENT | MCI_DATABLOCKEND)
#define MCI_DATA_IRQ (MCI_DATA_ERR | MCI_DATAEND)
#define MCI_XFER_IRQ_MASK \
			(MCI_TXFIFOEMPTY | MCI_TXFIFOHALFEMPTY | \
			 MCI_RXFIFOHALFFULL | MCI_RXDATAAVLBL)
#define MCI_CMD_IRQ \
			(MCI_CMDCRCFAIL | MCI_CMDTIMEOUT | MCI_CMDRESPEND | \
			 MCI_CMDSENT)
#define MCI_XFER_IRQ \
			(MCI_TXFIFOHALFEMPTY | MCI_RXFIFOHALFFULL)

/*
 * The size of the FIFO in bytes.
 */
#define MCI_FIFOSIZE	16
#define MCI_FIFOHALFSIZE (MCI_FIFOSIZE / 2)
#define NR_SG		16
#define MMC_MAX_REQ_SIZE	SZ_4M
#define MMC_MAX_SEG_SIZE	SZ_64K
#define MMC_HOST_BLK_SIZE	512
#define MMC_MAX_PHY_SEGMENTS	128
#define MMC_MAX_HW_SEGMENTS		128

#define OCR_AVAIL	(MMC_VDD_17_18 | MMC_VDD_18_19 | \
			/*MMC_VDD_28_29 |*/ \
			MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_32_33 | \
			 MMC_VDD_33_34)
#define INVALID_PIPEID	(-1)

/**
 * struct u8500_mmci_host - host device structure
 * @base:	pointer to the device baseaddress
 * @mrq:	pointer to the request structure
 * @cmd:        pointer to the command structure
 * @data:       pointer to the data structure
 * @mmc:        pointer to the mmc_host structure
 * @clk:        pointer to the clock structure
 * @data_xfered: variable which updates the data_transfered
 * @lock:      spinlock variable
 * @sg_ptr:    scatter list pointer
 * @mclk:      master clock
 * @cclk:      card clock
 * @card_detect_intr_value: callback for the carddetection
 * @oldstat: card detection value
 * @dmach_mmc2mem: dma pipeid card to memory
 * @dmach_mem2mmc: dma pipeid from memory to card
 * @dma_fifo_addr: dma fifo address
 * @dma_fifo_dev_type_rx: rx channel number
 * @dma_fifo_dev_type_tx: tx channel number
 * @level_shifter: variable for checking level shifter
 * @dma: ponter to dma_addr_t structure
 * @caps: host capabilities
 * @bus_resume_flags: Type of MMC bus resume requested.
 * @sg_len: scatter gather length
 * @sg_off: offset address
 * @size: data size
 * @buffer: buffer used in polling mode
 * @dma_buffer: buffer used in dma mode
 * @dma_done: variable for dma completion
 * @devicemode: variable for device mode
 * @is_sdio: variable for sdio
 * @board: pointer to the board structure
 * @regulator: pointer to the regulator structure
 * @sdio_setirq: set irq status for SDIO
 * @sdio_irqstatus: current irq status for SDIO
 * @aligned_blksz: aligned block size value for SDIO
 * @aligned_size: aligned size value for SDIO
 * @reg_context: array to store register context
 * @host_reg: pointer to the PL180 regulator
 *
 * host controller Internal device structure
 */
struct u8500_mmci_host {
	void __iomem *base;
	struct mmc_request *mrq;
	struct mmc_command *cmd;
	struct mmc_data *data;
	struct mmc_host *mmc;
	struct clk *clk;
#ifdef CONFIG_REGULATOR
	struct regulator *regulator;
#endif
	unsigned int data_xfered;
	spinlock_t lock;
	unsigned int mclk;
	unsigned int cclk;
	int (*card_detect_intr_value) (void);
	unsigned int oldstat;
	struct dma_chan *dmach_mmc2mem;
	struct dma_chan *dmach_mem2mmc;
	struct stedma40_chan_cfg *dma_mem2mmc;
	struct stedma40_chan_cfg *dma_mmc2mem;
	dma_filter_fn dma_filter;
	unsigned int level_shifter;
	dma_addr_t dma;
	unsigned long	caps;		/* Host capabilities */
	unsigned int bus_resume_flags;
	unsigned int sg_len;
	/* pio stuff */
	struct scatterlist *sg_ptr;
	unsigned int sg_off;
	unsigned int size;
	unsigned int *buffer;
	unsigned int *dma_buffer;
	void *dma_done;	/* completion data */
	int devicemode;
	unsigned int is_sdio;
	int sdio_setirq;
	int sdio_irqstatus;
	int aligned_blksz;
	int aligned_size;
	struct mmc_board *board;
	unsigned long reg_context[2];
	struct regulator *host_reg;
};

/* Define the current mode  */
#define MCI_DMAMODE 		0x01
#define MCI_POLLINGMODE		0x02
#define MCI_INTERRUPTMODE	0x03
#define MCI_ALLINTERRUPTS	(0x007FFFFF)
#define MMCCLRSTATICFLAGS	(0x000007FF)
#define MCI_MAXVOLTTRIAL	(200)	/* 200 times */
#define MAX_FREQ (24000000)
#define MAX_DATA (64*512)
#define MMC_HOST_CLK_MAX	100000000
#define MMC_CLK_DIV 0xFF
/*
 * different card states
 */
enum card_state {
	CARD_STATE_EMPTY = -1,
	CARD_STATE_IDLE,
	CARD_STATE_READY,
	CARD_STATE_IDENT,
	CARD_STATE_STBY,
	CARD_STATE_TRAN,
	CARD_STATE_DATA,
	CARD_STATE_RCV,
	CARD_STATE_PRG,
	CARD_STATE_DIS,
};
/*
 * struct mmc_board -  mmc board dependent  structure
 * @init:	function pointer for mmc board related initialization
 * @exit:	function pointer for mmc board related de-initialization
 *
 * SDMMC platfoem dependent structure
 */
struct mmc_board {
	int (*init) (struct amba_device *dev);
	void (*exit) (struct amba_device *dev);
	int (*set_power) (struct device *dev, int power_on);
	int (*card_detect_intr_conf) (int enable_or_disable);
	int (*card_detect)(void (*callback)(void *parameter), void *);
	int (*card_detect_intr_value) (void);
	struct stedma40_chan_cfg *dma_mem2mmc;
	struct stedma40_chan_cfg *dma_mmc2mem;
	dma_filter_fn dma_filter;
	unsigned int level_shifter;
	unsigned long	caps;	/* Host capabilities */
	unsigned int bus_resume_flags;
	int is_sdio;		/* To check if the bus is SD/MMC or sdio */
#ifdef CONFIG_REGULATOR
	const char *supply;
	int min_supply_voltage;
	int max_supply_voltage;
#endif
};

#endif
