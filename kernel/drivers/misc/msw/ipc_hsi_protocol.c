/*---------------------------------------------------------------------------*/
/* Copyright ST Ericsson, 2009.                                              */
/* This program is free software; you can redistribute it and/or modify it   */
/* under the terms of the GNU General Public License as published by the     */
/* Free Software Foundation; either version 2.1 of the License, or	     */
/* (at your option) any later version.                                       */
/*                                                                           */
/* This program is distributed in the hope that it will be useful, but 	     */
/* WITHOUT ANY WARRANTY; without even the implied warranty of 		     */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 		     */
/* See the GNU General Public License for more details.   		     */
/*                                                                           */
/* You should have received a copy of the GNU General Public License         */
/* along with this program. If not, see <http://www.gnu.org/licenses/>.      */
/*---------------------------------------------------------------------------*/

/** @file  ipc_hsi_protocol.c
 * @brief This file contains HSI driver protocol interface handling.
 */
#include <linux/kernel.h>
#include <linux/smp_lock.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/types.h>
#include <linux/completion.h>
#include <linux/hsi-legacy.h>
#include "ipc_protocol_if.h"
#include "ipc_hsi_protocol.h"

#define NAME "IPC_HSI"
#define IPC_HSI_DEBUG 0
#define dbg_printk(format, arg...)	(IPC_HSI_DEBUG & 1) ? \
		(printk(KERN_ALERT NAME ": " format , ## arg)) : \
					({do {} while (0); })

#define HSICTRL 0x80157324
/** HSI Power management configuration*/
#define NO_SLEEP_NO_DVFS     0x0
#define SLEEP_NO_DVFS        0x1
#define DVFS_NO_SLEEP        0x2
#define SLEEP_DVFS           0x3
/** HSI version */
#define NDK_SSI_VERSION      0x0000
#define NDK_SSI_CONFIGURATION NO_SLEEP_NO_DVFS
/** boot state defines */
#define MODEM_BOOT_INFO_REQ_ID 0x1
#define MODEM_BOOT_INFO_RESP_ID 0x2
#define BOOT_INFO_REQ   0x10000000
#define BOOT_INFO_RESP  0x20000000
/** Modem States */
#define MODEM_SSI_BOOT_INIT                       0
#define MODEM_SSI_BOOT_INFO_SYNC                  1
#define MODEM_SSI_BOOT_DONE                       2

u8 current_boot_state;
u8 current_boot_state_audio;
u32 modem_response;
u32 host_request,boot_info_resp;
u32 modem_response_audio;
u32 rcv_start;

static struct t_l1_desc dev_desc[MAX_CHANNELS_MONITORED];

static int do_modem_boot(void);

static inline int align_4byte(int len)
{
	return 4*(int)((len+3)/4);
}

/**
 * hsi_msg_send - UL Message Transmission Function
 *
 * @addr:  message address
 * @l2_header: l2 header
 * This function sends the message to the media available(HSI/SSI).
 * After successful transmission of the message it calls layer-2
 * provided deallocation function.
*/
int hsi_msg_send(void *addr, u8 l2_header, u32 length)
{
	u32 st;
	struct t_data_buf *p_data;
	struct t_l1_desc *tx_params;
	u32 hsimode;
	p_data = (struct t_data_buf *)addr;
	hsimode = 0;
	dbg_printk("host_msg_send++\n");

	if (l2_header < 2)
		tx_params = &dev_desc[3];
	else
		tx_params = &dev_desc[2];

	down(&tx_params->tx_mutex);

	if (dlp_ssi_get_boot_state() != MODEM_SSI_BOOT_DONE) {
		printk(KERN_ALERT NAME ":Modem not Initialized\n");
		up(&tx_params->tx_mutex);
		return -1;
	}

	st = ((l2_header & 0xFF) << 24) |
		 ((tx_params->tx_msg.mapid & 0xF) << 20) |
		 (length & 0xFFFFF);

	*(tx_params->tx_msg.st) = st;

	init_completion(&tx_params->tx_complete);
	/** set interrupt mode */
	hsimode = HSI_INTERRUPT_MODE;
	hsi_ioctl(tx_params->tx_dev, HSI_IOCTL_SET_CURRMODE, &hsimode);
	/** send start */
	hsi_write(tx_params->tx_dev, &st, 32, 4);
	/** wait for start finish */
	wait_for_completion(&tx_params->tx_complete);

	init_completion(&tx_params->tx_complete);
	/** set interrupt mode */
	hsimode = HSI_INTERRUPT_MODE;
	hsi_ioctl(tx_params->tx_dev, HSI_IOCTL_SET_CURRMODE, &hsimode);
	/** write data */
	hsi_write(tx_params->tx_dev, p_data->log_address, \
			32, align_4byte(length));

	wait_for_completion(&tx_params->tx_complete);

	tx_params->tx_msg.mapid = (tx_params->tx_msg.mapid + 1) & 0xF;

	dbg_printk("%d byte data transmitted\n", length);

	up(&tx_params->tx_mutex);

	return 0;
}
/**
 * tx_common_data_handler()  common data tx completion callback
 *
 * This function gets called after any tx completion on common
 * control channel.
 */
static void tx_common_data_handler(void)
{
	struct t_l1_desc *tx_params = &dev_desc[3];

	dbg_printk("tx complete on data channel\n");
	complete(&tx_params->tx_complete);
}

static void tx_audio_data_handler(void)
{
	struct t_l1_desc *tx_params = &dev_desc[2];
	dbg_printk("tx complete on audio channel\n");
	complete(&tx_params->tx_complete);
}

/**
 * rx_common_data_handler() - Common channel Rx Message handler
 *
 * This function handles messages received on common hsi channel.
 * This callback is registerd while registering client to hsi driver.
 */
static void rx_common_data_handler(void)
{
	struct t_data_buf *msgptr;
	u32 pdu_length;
	u8 l2_header;
	struct t_l1_desc *rx_params;
	u32 st;
	u8 mapid;
	u32 hsimode = 2;

	rx_params = &dev_desc[3];

	/** start_transmission handler*/
	if (rx_params->rx_state == L1_RX_WAITING_ST_STATE) {
		rx_params->rx_state = L1_RX_ST_RECEIVED_STATE;

		if (hsimode != HSI_DMA_MODE)
			st = rcv_start;
		else
			st = *(rx_params->rx_msg.rx_st.log_address);

		pdu_length = st & 0xFFFFF;
		mapid = (st >> 20) & 0xF;
		if ((pdu_length <= 0) || \
			(mapid != rx_params->rx_msg.mapid) || \
			((st >> 24) > 1)) {
			printk(KERN_ALERT "\n False Message..Start searching again");
			BUG();
		}

		/** assign correct pointer to the msgptr*/
		msgptr = &rx_params->rx_msg.msgptr;

		l3alloc(pdu_length, st>>24, (void *)msgptr);

		rx_params->rx_state = L1_RX_BUSY_STATE;
		rx_params->rx_msg.length = pdu_length;

		hsimode=HSI_DMA_MODE;
		hsi_ioctl(rx_params->rx_dev, HSI_IOCTL_SET_CURRMODE, \
					&hsimode);
		mdelay(1);
		hsi_read(rx_params->rx_dev, (hsimode == HSI_DMA_MODE) ? \
					(void *)msgptr->phys_address : \
					(void *)msgptr->log_address, \
					32, align_4byte(pdu_length));
	}
	/** data intr/dma eot handler callback*/
	else if (rx_params->rx_state == L1_RX_BUSY_STATE) {

		msgptr = &rx_params->rx_msg.msgptr;
		pdu_length = rx_params->rx_msg.length;

		if (hsimode != HSI_DMA_MODE)
			l2_header = ((rcv_start)>>24) & 0xFF;
		else
			l2_header = (*(rx_params->rx_msg.rx_st.log_address) >> \
					24) & 0xFF;

		l3receive(msgptr, pdu_length, l2_header);

		/** changing map-id for next message*/
		rx_params->rx_msg.mapid = (rx_params->rx_msg.mapid+1) & 0xF;

		/** set the state*/
		rx_params->rx_state = L1_RX_WAITING_ST_STATE;
		/** start searching for start again */
		rcv_start = 0;
		hsimode=HSI_DMA_MODE;
		hsi_ioctl(rx_params->rx_dev, HSI_IOCTL_SET_CURRMODE, & \
			hsimode);
		hsi_read(rx_params->rx_dev, (hsimode != HSI_DMA_MODE) ? \
				(void *)&rcv_start : \
				(void *)rx_params->rx_msg.rx_st.phys_address,
				32, 4);
	}
	/** error case */
	else {
		printk(KERN_ALERT "Entering while(1)\n");
		BUG();
	}
}

/**
 * rx_audio_data_handler - Audio data channel callback handler
 *
 * This function handles events on audio data channel.
 */
static void rx_audio_data_handler(void)
{
	struct t_data_buf *msgptr;
	u32 pdu_length;
	u8 l2_header;
	struct t_l1_desc *rx_params;
	u32 st;
	u8 mapid;
	u32 hsimode = 2;

	dbg_printk("rx_audio_data_handler++ \n");

	rx_params = &dev_desc[2];

	/** start_transmission handler*/
	if (rx_params->rx_state == L1_RX_WAITING_ST_STATE) {
		rx_params->rx_state = L1_RX_ST_RECEIVED_STATE;

		if (hsimode != HSI_DMA_MODE)
			st = rcv_start;
		else
			st = *(rx_params->rx_msg.rx_st.log_address);


		pdu_length = st & 0xFFFFF;
		/*message ID*/
		mapid = (st >> 20) & 0xF;

		if ((pdu_length <= 0) || \
			(mapid != rx_params->rx_msg.mapid) || \
			((st >> 24) != 2)) {
			printk(KERN_ALERT "False Alert");
			BUG();
		}

		/** assign correct pointer to the msgptr*/
		msgptr = &rx_params->rx_msg.msgptr;

		l3alloc(align_4byte(pdu_length), st>>24, (void *)msgptr);

		rx_params->rx_state = L1_RX_BUSY_STATE;
		rx_params->rx_msg.length = pdu_length;

		hsimode = HSI_DMA_MODE;
		hsi_ioctl(rx_params->rx_dev, HSI_IOCTL_SET_CURRMODE, &hsimode);
		hsi_read(rx_params->rx_dev, \
			(hsimode == HSI_DMA_MODE) ? \
				(void *)msgptr->phys_address : \
				(void *)msgptr->log_address, \
				32, align_4byte(pdu_length));
	}
	/** data intr/dma eot handler callback*/
	else if (rx_params->rx_state == L1_RX_BUSY_STATE) {

		msgptr = &rx_params->rx_msg.msgptr;
		pdu_length = rx_params->rx_msg.length;
		if (hsimode != HSI_DMA_MODE)
			l2_header = ((rcv_start) >> 24) & 0xFF;
		else
			l2_header = (*(rx_params->rx_msg.rx_st.log_address) >> \
					24) & 0xFF;

		l3receive(msgptr, align_4byte(pdu_length), l2_header);

		/** changing map-id for next message*/
		rx_params->rx_msg.mapid = (rx_params->rx_msg.mapid + 1) & 0xF;

		/** set the state*/
		rx_params->rx_state = L1_RX_WAITING_ST_STATE;
		/** start searching for start again */
		rcv_start = 0;
		hsimode = HSI_DMA_MODE;
		hsi_ioctl(rx_params->rx_dev, HSI_IOCTL_SET_CURRMODE, &hsimode);
		hsi_read(rx_params->rx_dev, \
			(hsimode != HSI_DMA_MODE) ? \
				(void *)&rcv_start : \
				(void *)rx_params->rx_msg.rx_st.phys_address, \
				32, 4);
	} else {
		printk("Entering while(1)\n");
		BUG();
	}
	dbg_printk("rx_audio_data_handler-- \n");
}

void state_change_audio(u8 state)
{
u32 hsimode = HSI_DMA_MODE;
static struct t_l1_desc *rx_params;
rx_params = &dev_desc[2]; /** Common data channel */

if(state==MODEM_SSI_BOOT_DONE){

			rx_params->rx_state = L1_RX_WAITING_ST_STATE;
			hsimode = HSI_DMA_MODE;
			hsi_ioctl(rx_params->rx_dev, HSI_IOCTL_SET_CURRMODE, \
				&hsimode);
			hsi_read(rx_params->rx_dev, \
				(hsimode != HSI_DMA_MODE) ? \
				(void *)&rcv_start : \
				(void *)rx_params->rx_msg.rx_st.phys_address, \
				32, 4);
		  }

}

/**
 * rx_audio_control_handler - Audio control channel callback handler
 *
 * This function handles events on audio control channel.
 */
static void rx_audio_control_handler(void)
{
	int err=0;
	static struct t_l1_desc *ch1;
	u32 hsimode = HSI_INTERRUPT_MODE;
	u32 rx_command;
	u8  command_ID,modem_ssi_version,modem_config_info;

	dbg_printk("rx_common_control_handler Read buff %x\n", modem_response);


	ch1 = &dev_desc[1];

	rx_command = modem_response;

	command_ID = (rx_command >> 28);

	switch (command_ID) {

	case MODEM_BOOT_INFO_REQ_ID:


	     modem_ssi_version = (u8)rx_command;
	             if (modem_ssi_version != NDK_SSI_VERSION) {
	             dbg_printk(KERN_ALERT NAME ":Unsup SSI version\n");
	     BUG();
	     }

	     host_request = BOOT_INFO_RESP;



	     modem_config_info = (rx_command & \
				  0x0000ff00) >> 8;

	     boot_info_resp = BOOT_INFO_RESP|
				(NDK_SSI_CONFIGURATION & \
				 modem_config_info) << 8 |
				 modem_ssi_version;



	     hsi_ioctl(ch1->tx_dev, HSI_IOCTL_SET_CURRMODE, &hsimode);

	     err = hsi_write(ch1->tx_dev, &boot_info_resp, 32, 4);

	     printk(KERN_ALERT NAME ":Audio:SYNC ->DONE!\n");

	     current_boot_state_audio = MODEM_SSI_BOOT_DONE;

	     /** Start common data channel receive */
	     state_change_audio(current_boot_state_audio);

	break;

	case MODEM_BOOT_INFO_RESP_ID:
	     modem_ssi_version = (u8)rx_command;
		     if (modem_ssi_version != NDK_SSI_VERSION) {
		     dbg_printk(KERN_ALERT NAME ":Unsup SSI version\n");
		     BUG();
	     }

	     printk(KERN_ALERT NAME ":Audio:SYNC ->DONE!\n");

	     current_boot_state_audio = MODEM_SSI_BOOT_DONE;

	     /** Start common data channel receive */
	     state_change_audio(current_boot_state_audio);

	break;

	default:

	     dbg_printk(" Error Corrupted response\n");



	break;

	}


}


void state_change_common(u8 state)
{
u32 hsimode = HSI_DMA_MODE;
static struct t_l1_desc *rx_params;

rx_params = &dev_desc[3]; /** Common data channel */

if(state==MODEM_SSI_BOOT_DONE){

			rx_params->rx_state = L1_RX_WAITING_ST_STATE;
			hsimode = HSI_DMA_MODE;
			hsi_ioctl(rx_params->rx_dev, HSI_IOCTL_SET_CURRMODE, \
				&hsimode);
			hsi_read(rx_params->rx_dev, \
				(hsimode != HSI_DMA_MODE) ? \
				(void *)&rcv_start : \
				(void *)rx_params->rx_msg.rx_st.phys_address, \
				32, 4);
		  }

}

/**
 * rx_common_control_handler - Common control channel callback handler
 * This function handles events on common control channel.
 */
static void rx_common_control_handler(void)
{
	int err=0;
	static struct t_l1_desc *ch0;
	u32 hsimode = HSI_INTERRUPT_MODE;
	u32 rx_command;
	u8  command_ID,modem_ssi_version,modem_config_info;

	dbg_printk("rx_common_control_handler Read buff %x\n", modem_response);


	ch0 = &dev_desc[0];

	rx_command = modem_response;

	command_ID = (rx_command >> 28);

	switch (command_ID) {

	case MODEM_BOOT_INFO_REQ_ID:


	     modem_ssi_version = (u8)rx_command;
	             if (modem_ssi_version != NDK_SSI_VERSION) {
	             dbg_printk(KERN_ALERT NAME ":Unsup SSI version\n");
	     BUG();
	     }

	     host_request = BOOT_INFO_RESP;



	     modem_config_info = (rx_command & \
				  0x0000ff00) >> 8;

	     boot_info_resp = BOOT_INFO_RESP|
				(NDK_SSI_CONFIGURATION & \
				 modem_config_info) << 8 |
				 modem_ssi_version;


	     hsi_ioctl(ch0->tx_dev, HSI_IOCTL_SET_CURRMODE, &hsimode);

	     err = hsi_write(ch0->tx_dev, &boot_info_resp, 32, 4);

	     printk(KERN_ALERT NAME ":Common:SYNC ->DONE!\n");

	     current_boot_state = MODEM_SSI_BOOT_DONE;

	     /** Start common data channel receive */
	     state_change_common(current_boot_state);

	break;

	case MODEM_BOOT_INFO_RESP_ID:
	     modem_ssi_version = (u8)rx_command;
		     if (modem_ssi_version != NDK_SSI_VERSION) {
		     dbg_printk(KERN_ALERT NAME ":Unsup SSI version\n");
		     BUG();
	     }

	     printk(KERN_ALERT NAME ":Common:SYNC ->DONE!\n");

	     current_boot_state = MODEM_SSI_BOOT_DONE;

	     /** Start common data channel receive */
	     state_change_common(current_boot_state);

	break;

	default:

	     dbg_printk(" Error Corrupted response\n");



	break;

	}


}



static void tx_common_control_handler(void)
{
	dbg_printk("tx_common_control_handler\n");
}

static void tx_audio_control_handler(void)
{
	dbg_printk("tx_common_audio_control_handler\n");
}

u8 dlp_ssi_get_boot_state(void)
{
	return current_boot_state;
}



static void read_callbk(struct hsi_device *dev)
{
	u8 ch = 0;
	dbg_printk("read_callbk\n");

	ch = (u8)dev->chid;

	switch (ch) {
	case CHANNEL_COMMON_CONTROL:
		rx_common_control_handler();
		break;
	case CHANNEL_AUDIO_CONTROL:
		rx_audio_control_handler();
		break;
	case CHANNEL_AUDIO_DATA:
		rx_audio_data_handler();
		break;
	case CHANNEL_COMMON_DATA:
		rx_common_data_handler();
		break;
	default:
		BUG();
	};
}

static void write_callbk(struct hsi_device *dev)
{
	int ch = 0;
	dbg_printk("write_callbk\n");

	ch = dev->chid;
	switch (ch) {
	case CHANNEL_COMMON_CONTROL:
		tx_common_control_handler();
		break;
	case CHANNEL_AUDIO_CONTROL:
		tx_audio_control_handler();
		break;
	case CHANNEL_AUDIO_DATA:
		tx_audio_data_handler();
		break;
	case CHANNEL_COMMON_DATA:
		tx_common_data_handler();
		break;
	default:
		BUG();
	}
}

static int __init hsi_l2mux_drv_probe(struct hsi_device *dev)
{
	int err = 0;
	static int chid = -1, no_chn;

	if (!dev) {
		printk(KERN_ALERT "SSI device not populated \n");
		return -ENODEV;
	}
	dbg_printk("hsi_l2mux_drv_probe++\n");
	dbg_printk("ctrlrid %d chid %d\n", dev->ctrlrid, dev->chid);

	switch (dev->chid) {
	case CHANNEL_COMMON_CONTROL:
		no_chn++; chid = 0;
		break;
	case CHANNEL_AUDIO_CONTROL:
		no_chn++; chid = 1;
		break;
	case CHANNEL_AUDIO_DATA:
		no_chn++; chid = 2;
		break;
	case CHANNEL_COMMON_DATA:
		no_chn++; chid = 3;
		break;
	default:
		break;
	};

	if (no_chn >= 0) {
		if (dev->ctrlrid == HSIT_CTRLR_ID) {
			dev_desc[chid].tx_dev = dev;
			hsi_dev_set_cb(dev_desc[chid].tx_dev, \
				NULL, &write_callbk);
		} else {
			dev_desc[chid].rx_dev = dev;
			hsi_dev_set_cb(dev_desc[chid].rx_dev, \
				&read_callbk, NULL);
		}
	}

	return err;
}

static int __exit hsi_l2mux_drv_remove(struct hsi_device *dev)
{
	u8 chid = dev->chid;
	dbg_printk("hsi_l2mux_drv_remove++\n");
	dbg_printk("ctrlrid %d chid %d\n", dev->ctrlrid, dev->chid);

	if (dev->ctrlrid == HSIT_CTRLR_ID)
		dev_desc[chid].tx_dev = NULL;
	else
		dev_desc[chid].rx_dev = NULL;
	return 0;
}

/**
 * channel_exception_handler -
 *
 * @ctrlr_id: controller id
 * @event: exception type
 * @arg:
 *
 * NOTE: an actual protocol driver is expected to do proper
 * exception handling. This could include flushing its buffers,
 * cancelling read, informing the TX through a message to
 * resend/start sending data
 *
 * Apart from informing the protocol driver the lowlevel HSI driver
 * does not perform any operations on receiving an exception,
 * except when it receives :
 * 		HSI_EXCEP_PIPEBUF_OVERRUN, where it flushes HSIR
 * 		pipeline buffer
 * 		HSI_RXCHANNELS_OVERRUN , where it flushes the HSIR channel
 * 		buffer
 */
static void channel_exception_handler(u32 ctrlr_id, u32 event, void *arg)
{
	u8 chid;
	/** exceptions are received only on HSIR */
	if (ctrlr_id != HSIR_CTRLR_ID) {
		printk(KERN_ALERT "HSI test protocol driver:exception on \
			wrong controller\n");
		return;
	}

	switch (event) {
	case HSI_EXCEP_TIMEOUT:
	printk(KERN_ALERT "protocol driver:TIMEOUT \n");
		break;
	case HSI_EXCEP_PIPEBUF_OVERRUN:
	/** HSI has flushed pipeline buffer...
	protocol driver MUST take appropriate action */
	printk(KERN_ALERT "protocol driver:PIPEBUF OVERRUN \n");
		break;
	case HSI_EXCEP_BREAK_DETECTED:
	printk(KERN_ALERT "protocol driver:BREAK DETECT \n");
		break;
	case HSI_EXCEP_PARITY_ERROR:
	printk(KERN_ALERT "protocol driver: PARITY ERROR \n");
		break;
	case HSI_RXCHANNELS_OVERRUN:
	/** HSI has flushed corresponding channel buffer...
	protocol driver MUST take appropriate action */
		chid = *(u8 *)arg;
	printk(KERN_ALERT "protocol driver: Ch %d OVERRUN \n", chid);
		break;
	}
}

static struct hsi_device_driver ipc_driver = {
	/** tx and rx controller */
	.ctrl_mask = 0x3,
	/** channel 3 and 0*/
	.ch_mask = 0xf,
	/** all exceptions */
	.excep_mask = 0x1F,
	/** exception handler*/
	.excep_event = channel_exception_handler,
	/**probe function*/
	.probe = hsi_l2mux_drv_probe,
	/**remove function*/
	.remove = __exit_p(hsi_l2mux_drv_remove),
	.driver = {
		   .name = "HSI_IPC_DRIVER",
		  },
};
/**
 * dlp_protocol_init() - HSI Protocol Init
 *
 * This function performs following tasks.
 * Registers to hsi bus driver and waits till all the channels are probed.
 * It opens common data and common control channel.
 */
int dlp_protocol_init(void)
{
	int err = 0, no_ch;
	u32 framelen = 32, watermark = 0x01;

	/*HSI out*/
	#if (CONFIG_U8500_HSI_MODEM_DIRECTION)
	*((unsigned int *)((void __iomem *)ioremap_nocache(HSICTRL, 32))) = 2;
	#endif

	err = register_hsi_driver(&ipc_driver);
	if (err < 0) {
		printk(KERN_ALERT "hsi protocol driver registration failed\n");
		return err;
	}
	for (no_ch = 0; no_ch < MAX_CHANNELS_MONITORED; no_ch++) {
		err = hsi_open(dev_desc[no_ch].tx_dev);
		if (err < 0) {
			printk(KERN_ALERT NAME ":Can't open HSIT-%d\n", no_ch);
			return err;
		}
		err = hsi_open(dev_desc[no_ch].rx_dev);
		if (err < 0) {
			printk(KERN_ALERT NAME ":Can't open HSIR-%d\n", no_ch);
			return err;
		}
		err = hsi_ioctl(dev_desc[no_ch].tx_dev, \
					HSI_IOCTL_SET_WATERMARK, \
					(void *)&watermark);
		if (err < 0) {
			printk(KERN_ALERT NAME ":Cannot set frame-length\n");
			return err;
		}
		err = hsi_ioctl(dev_desc[no_ch].rx_dev, \
					HSI_IOCTL_SET_WATERMARK, \
					(void *)&watermark);
		if (err < 0) {
			printk(KERN_ALERT NAME ":Cannot set frame-length\n");
			return err;
		}
		err = hsi_ioctl(dev_desc[no_ch].tx_dev, \
					HSI_IOCTL_SET_FRAMELEN, \
					(void *)&framelen);
		if (err < 0) {
			printk(KERN_ALERT NAME ":Cannot set frame-length\n");
			return err;
		}
		err = hsi_ioctl(dev_desc[no_ch].rx_dev, \
					HSI_IOCTL_SET_FRAMELEN, \
					(void *)&framelen);
		if (err < 0) {
			printk(KERN_ALERT NAME ":Cannot set frame-length\n");
			return err;
		}
	}

	dev_desc[2].tx_msg.st = kmalloc(4, GFP_KERNEL | GFP_ATOMIC);
	if (dev_desc[2].tx_msg.st == NULL) {
		printk(KERN_ALERT NAME ":Cannot allocate memory for ST\n");
		return -ENOMEM;
	}

	dev_desc[2].rx_msg.rx_st.log_address = dma_alloc_coherent(NULL, 4, \
				 &dev_desc[2].rx_msg.rx_st.phys_address,
				GFP_DMA | GFP_KERNEL);
	if (dev_desc[2].rx_msg.rx_st.log_address == NULL) {
		printk(KERN_ALERT NAME ":Failed to alloc memory\n");
		BUG();
	}

	dev_desc[3].tx_msg.st = kmalloc(4, GFP_KERNEL | GFP_ATOMIC);
	if (dev_desc[3].tx_msg.st == NULL) {
		printk(KERN_ALERT NAME ":Cannot allocate memory for ST\n");
		return -ENOMEM;
	}

	dev_desc[3].rx_msg.rx_st.log_address = dma_alloc_coherent(NULL, 4, \
				&dev_desc[3].rx_msg.rx_st.phys_address, \
				GFP_DMA | GFP_KERNEL);
	if (dev_desc[3].rx_msg.rx_st.log_address == NULL) {
		printk(KERN_ALERT NAME ":Failed to alloc memory\n");
		BUG();
	}

	return err;
}
/**
 * dlp_protocol_deinit() - HSI protocol Uninstall
 *
 * This function will close all the hsi channels opened earlier and
 * unregisters HSI protocol driver client.
 */
int dlp_protocol_deinit(void)
{
	int no_ch;

	for (no_ch = 0; no_ch < MAX_CHANNELS_MONITORED; no_ch++) {
		hsi_close(dev_desc[no_ch].tx_dev);
		kfree(dev_desc[no_ch].tx_msg.st);
	}

	unregister_hsi_driver(&ipc_driver);

	printk(KERN_ALERT NAME ":unregister_hsi_driver done\n");
	return 0;
}


static int do_modem_boot(void)
{
	int err = 0;
	struct t_l1_desc *rx_params, *rx_params_audio;
	u32 hsimode = 1;

	rx_params = &dev_desc[0];
	rx_params_audio = &dev_desc[1];

	printk(KERN_ALERT "Trying to boot modem\n");


	current_boot_state = MODEM_SSI_BOOT_INIT;
	current_boot_state_audio = MODEM_SSI_BOOT_INIT;

	host_request = BOOT_INFO_REQ;
	hsimode = HSI_INTERRUPT_MODE;

	printk(KERN_ALERT NAME ":Common: INIT->SYNC!\n");
	hsi_ioctl(dev_desc[0].tx_dev, HSI_IOCTL_SET_CURRMODE, &hsimode);
	err = hsi_write(dev_desc[0].tx_dev, &host_request, 32, 4);


	hsi_ioctl(rx_params->rx_dev, HSI_IOCTL_SET_CURRMODE, &hsimode);
	err = hsi_read(rx_params->rx_dev, &modem_response, 32, 4);

	printk(KERN_ALERT NAME ":Audio: INIT->SYNC!\n");
	err = hsi_write(dev_desc[1].tx_dev, &host_request, 32, 4);
	err = hsi_read(rx_params_audio->rx_dev, &modem_response_audio, 32, 4);

	return err;
}




/**
 * dlp_protocol_start() - HSI Protocol Start
 *
 * This function starts HSI protocol by sending boot request on common
 * control channel.
 */
int dlp_protocol_start(u8 l2_header)
{
	int err = 0;

	sema_init(&dev_desc[3].tx_mutex, 1);
	sema_init(&dev_desc[3].rx_mutex, 1);
	sema_init(&dev_desc[2].tx_mutex, 1);
	sema_init(&dev_desc[2].rx_mutex, 1);

	*(dev_desc[3].tx_msg.st) 	= 0;
	dev_desc[3].tx_msg.mapid 	= 0;
	dev_desc[3].tx_msg.length 	= 0;
	dev_desc[3].rx_msg.mapid 	= 0;
	dev_desc[3].rx_msg.length 	= 0;

	*(dev_desc[2].tx_msg.st)	= 0;
	dev_desc[2].tx_msg.mapid	= 0;
	dev_desc[2].tx_msg.length	= 0;
	dev_desc[2].rx_msg.mapid	= 0;
	dev_desc[2].rx_msg.length	= 0;

	/*delayed modem boot*/
	do_modem_boot();

	printk(KERN_ALERT NAME ":protocol started\n");
	return err;
}

int dlp_protocol_stop(u8 l2_header)
{
	return 0;
}

