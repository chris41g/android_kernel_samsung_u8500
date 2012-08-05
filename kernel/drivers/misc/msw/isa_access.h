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

/** @file  isa_access.h
  * @brief This file contains data structures for ISA access driver.
  *
  */
#ifndef __MODEM_ISA_H
#define __MODEM_ISA_H

struct t_data_buf {
	dma_addr_t phys_address;
	u32 *log_address;
};

struct t_queue_element {
	struct list_head entry;
	u32 offset;
	u32 size;
	u32 no;
};

struct t_message_queue {
	void *log_address;
	dma_addr_t phy_address;
	u32 size;
	u32 available_size;
	u32 readptr;
	u32 writeptr;
	u32 no;
	u8 direction;
	spinlock_t read_lock;
	spinlock_t write_lock;
	spinlock_t size_lock;
	spinlock_t update_lock;
	struct semaphore tx_mutex;
	atomic_t q_rp;
	atomic_t q_wp;
	wait_queue_head_t wq_readable;
	wait_queue_head_t wq_writable;
	struct list_head msg_list;
};

struct t_isadev_context {
	struct t_message_queue ul_queue;
	struct t_message_queue dl_queue;
	struct delayed_work work;
	struct workqueue_struct *work_queue;
	u8 device_id;
};

struct t_isa_driver_context {
	u8 modemState;
	u8 linkState;
	atomic_t isOpen[4];
	struct t_isadev_context *p_isadev[4];
	struct cdev cdev;/* Char device structure */
	spinlock_t op_lock;
};

enum t_queue_operation {
	READ_COMPLETE,
	WRITE_COMPLETE
};

#endif/*__MODEM_ISA_H*/
