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

/** @file  isa_access.c
  * @brief This file contains character driver registrations, character driver
  * interface to user space and buffer management.
  *
  */
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/ioport.h>
#include <linux/pci.h>
#include <linux/smp_lock.h>
#include <linux/poll.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <asm/atomic.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <mach/isa_ioctl.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include "ipc_protocol_if.h"
#include "isa_access.h"

#define ISA_ACCESS_DRIVER_DESC "isa_access"
#define ISA_ACCESS_DRIVER_AUTHOR "ST_Ericsson"
#define NAME "IPC_HSI"
#define ISA_DEVICES 4
#define CREATE_QUEUES_BOOT_TIME 1
/**debug functionality*/
#define ISA_DEBUG 1
#define dbg_printk(format, arg...)	(ISA_DEBUG & 1) ? \
	(printk(KERN_ALERT NAME ": " format , ## arg)) : \
	({do {} while (0); })

/**global data*/
/**
This variable is exported to user as module_param to specify
major number at load time
*/
static int major;
module_param(major, int, 0);
MODULE_PARM_DESC(major, "Major device number");
/**global fops mutex*/
static DEFINE_MUTEX(isa_lock);
/**global driver context pointer */
struct t_isa_driver_context *p_isa_context_hsi;

static inline int align_4byte(int len)
{
	return 4*((len+3)/4);
}
/**
 * create_queue() - To create FIFO for Tx and Rx message buffering.
 * @q: message queue.
 * @n_bytes: queue size in bytes.
 *
 * This function creates a FIFO buffer of n_bytes size using
 * dma_alloc_coherent(). It also initializes all queue handling
 * locks, queue management pointers. It also initializes message list
 * which occupies this queue.
 *
 * It return -ENOMEM in case of no memory.
 */
static int create_queue(struct t_message_queue *q, u32 n_bytes)
{
	q->log_address = dma_alloc_coherent(NULL, n_bytes, &q->phy_address, \
						GFP_DMA|GFP_KERNEL);
	if (q->log_address == NULL) {
		printk(KERN_ALERT NAME ":Failed to alloc memory\n");
		return -ENOMEM;
	}
	q->size = n_bytes;
	q->available_size = q->size;
	q->readptr = 0;
	q->writeptr = 0;
	q->no = 0;
	spin_lock_init(&q->size_lock);
	spin_lock_init(&q->read_lock);
	spin_lock_init(&q->write_lock);
	spin_lock_init(&q->update_lock);
	sema_init(&q->tx_mutex, 1);
	INIT_LIST_HEAD(&q->msg_list);
	init_waitqueue_head(&q->wq_readable);
	init_waitqueue_head(&q->wq_writable);
	atomic_set(&q->q_wp, 1);
	atomic_set(&q->q_rp, 0);
	return 0;
}
/**
 * delete_queue() - To delete FIFO and assiciated memory.
 * @q: message queue
 *
 * This function deletes FIFO created using create_queue() function.
 * It resets queue management pointers.
 */
static void delete_queue(struct t_message_queue *q)
{
	dma_free_coherent(NULL, q->size, q->log_address, q->phy_address);
	q->size = 0;
	q->available_size = 0;
	q->readptr = 0;
	q->writeptr = 0;
}
/**
 * allocate_buffer() - To allocate memory inside FIFO
 *
 * @q: message queue
 * @n_bytes: size in bytes
 *
 * This function tries to allocate n_bytes of size in FIFO q.
 * It returns negative number when no memory can be allocated
 * currently.
 */
static int allocate_buffer(struct t_message_queue *q, u32 n_bytes)
{
	int readptr, writeptr = -1;
	int available_size;
	unsigned long flag;

	dbg_printk("allocate_buffer++\n");

	if (q == NULL) {
		printk(KERN_ALERT NAME ":q->NULL!\n");
		return -ENOMEM;
	}

	spin_lock_irqsave(&q->update_lock, flag);
	readptr = q->readptr;
	available_size = q->available_size;
	spin_unlock_irqrestore(&q->update_lock, flag);

	dbg_printk("curr readptr %d writeptr %d available size %d\n", readptr, \
					q->writeptr, \
					available_size);

	if (q->writeptr == readptr) {
		if (available_size == q->size) {
			if (q->writeptr + n_bytes < q->size)
				writeptr = q->writeptr;
			else
				writeptr = 0;
		} else
			atomic_set(&q->q_wp, 0);
	} else {
		if (q->writeptr < readptr) {
			if (q->writeptr + n_bytes < readptr)
				writeptr = q->writeptr;
			else
				atomic_set(&q->q_wp, 0);
		} else {
			if (q->writeptr + n_bytes < q->size)
				writeptr = q->writeptr;
			else {
				if (n_bytes < readptr)
					writeptr = 0;
				else
					atomic_set(&q->q_wp, 0);
			}
		}
	}

	dbg_printk("allocated buffer at offset %d of size %d!\n", \
					writeptr, n_bytes);
	dbg_printk("allocate_buffer--\n");
	return writeptr;
}
/**
 * update_queue() - To update FIFO message list and available size.
 *
 * @q: message queue
 * @op_type: update type (operation completion type)
 * @size: size in bytes
 *
 * This function updates message list associated with message queue q and also
 * updates available size of FIFO.
 * If op_type is READ_COMPLETE, then, message is deleted from the list and
 * available size is increased by size of message. If the message list is empty,
 * then, event is generated for waiting process to indicate memory availability
 * inside queue.
 * If op_type is WRITE_COMPLETE, then, message is added to the list and
 * available size is reduced by message size. If the message list is empty then,
 * event is generated for waiting process to indicate new message availability
 * inside queue.
 * The message list is FIFO style and message is always added to tail and
 * removed from
 * head.
 */
static void update_queue(struct t_message_queue *q, \
				enum t_queue_operation op_type, \
				u32 size)
{
	struct t_queue_element *new_msg = NULL, *old_msg = NULL;
	struct list_head *msg;
	dbg_printk("update_queue++\n");

	if (op_type == READ_COMPLETE) {
		dbg_printk("READ_COMPLETE\n");
		list_for_each(msg, &q->msg_list) {
			old_msg = list_entry(msg, \
				struct t_queue_element, entry);
			if (old_msg == NULL) {
				printk(KERN_ALERT NAME ":no message found\n");
				/*while (1);*/
				BUG();
			}

			break;
		}
		list_del(msg);

		if (old_msg != NULL) {
			/** to keep dma buffer alignment */
			size = align_4byte(old_msg->size);
			q->readptr = old_msg->offset + size;
			q->available_size = q->available_size + size;
		}

		if (q->direction == 0) {
			if (list_empty(&q->msg_list))
				atomic_set(&q->q_rp, 0);
			} else {
			if (atomic_read(&q->q_wp) == 0) {
				atomic_set(&q->q_wp, 1);
				wake_up_interruptible(&q->wq_writable);
			}
		}
		dbg_printk("%s:msg_deleted %d:%d:%d\n", \
			(q->direction == 1) ? "UL" : "DL", old_msg->no, \
						old_msg->offset, \
						old_msg->size);
		dbg_printk("%s:updating readptr to %d\n", \
			(q->direction == 1) ? "UL" : "DL", q->readptr);
		if (old_msg != NULL)
			kfree(old_msg);
	}

	if (op_type == WRITE_COMPLETE) {
		dbg_printk("WRITE_COMPLETE\n");
		new_msg = kmalloc(sizeof(struct t_queue_element), \
					GFP_KERNEL|GFP_ATOMIC);
		if (new_msg == NULL) {
			printk(KERN_ALERT NAME ":memory overflow\n");
			/*while (1);*/
			BUG();
		}

		new_msg->offset = q->writeptr;
		new_msg->size = size;
		new_msg->no = q->no++;

		dbg_printk("%s:msg_added %d:%d:%d\n", \
			(q->direction == 1) ? "UL" : "DL", new_msg->no, \
					new_msg->offset, \
					new_msg->size);
		/** alignment needed to keep 4-byte boundry of dma buffer */
		size = align_4byte(size);

		q->writeptr = q->writeptr + size;
		q->available_size = q->available_size - size;

		if (list_empty(&q->msg_list)) {
			list_add_tail(&new_msg->entry, &q->msg_list);
			if (q->direction == 0) {
				atomic_set(&q->q_rp, 1);
				wake_up_interruptible(&q->wq_readable);
			}
		} else {
			list_add_tail(&new_msg->entry, &q->msg_list);
		}
	}
	dbg_printk("update_queue--\n");
}
/**
 * get_new_msg() - retrieve new message from message list
 *
 * @q:  message queue
 * @msg: message element in list
 *
 * This function will retrieve most recent message from the corresponding
 * queue list. New message is always retrieved from head side.
 * It returns new message no, offset if FIFO and size.
 */
static int get_new_msg(struct t_message_queue *q, struct t_dlp_message *msg)
{
	struct t_queue_element *new_msg = NULL;
	struct list_head *msg_list;
	unsigned long flag;

	dbg_printk("get_new_message++\n");

	spin_lock_irqsave(&q->update_lock, flag);
	list_for_each(msg_list, &q->msg_list) {
		new_msg = list_entry(msg_list, struct t_queue_element, entry);
		if (new_msg == NULL) {
			spin_unlock_irqrestore(&q->update_lock, flag);
			printk(KERN_ALERT NAME ":no message found\n");
			return -1;
		}
		break;
	}
	spin_unlock_irqrestore(&q->update_lock, flag);

	msg->offset = new_msg->offset;
	msg->size = new_msg->size;

	dbg_printk("%s:Message No :%d\n", (q->direction == 1) ? "UL" : "DL", \
					new_msg->no);
	dbg_printk("%s:Offset :%d Size :%d\n", \
					(q->direction == 1) ? "UL" : "DL", \
					msg->offset, \
					msg->size);
	dbg_printk("get_new_message--\n");
	if (msg->size > 0)
		return 0;
	else
		return -1;
}
/**
 * get_queue_address() - To get logical and physical address from offset
 *
 * @q: message queue
 * @ptr: message pointer pointer
 * @offset: message offset
 *
 * This function returns logical and physical address corresponding to
 * given offset in queue.
 */
static void get_queue_address(struct t_message_queue *q, void *ptr, u32 offset)
{
	struct t_data_buf *addr;
	addr = (struct t_data_buf *)ptr;
	addr->log_address = q->log_address + offset;
	addr->phys_address = q->phy_address + offset;
}
/**
 * l3alloc() - Memory allocation callback
 *
 * @n_bytes: Required size in bytes
 * @l2_msg_header: L2 Message Header
 * @addr: address of allocated memory
 *
 * This function allocates required memory in bytes and return corresponding
 * memory address.
 */
int l3alloc(u32 n_bytes, u8 l2_msg_header, void *addr)
{
	int offset;
	unsigned long flag;
	struct t_message_queue *q;
	struct t_isadev_context *l3dev;

	l3dev = p_isa_context_hsi->p_isadev[l2_msg_header];
	q = &l3dev->dl_queue;

	offset = allocate_buffer(q, align_4byte(n_bytes));
	if (offset < 0) {
		printk(KERN_ALERT ":%s:Waiting for WP\n", \
			(q->direction == 1) ? "UL" : "DL");
		printk(KERN_ALERT "Error:Buffer Full\n");
		/*while (1)*/
		BUG();
	}

	get_queue_address(q, addr, offset);
	spin_lock_irqsave(&q->update_lock, flag);
	q->writeptr = offset;
	spin_unlock_irqrestore(&q->update_lock, flag);

	return 0;
}
EXPORT_SYMBOL(l3alloc);

/**
 * sec_receive() - Rx Completion callback
 *
 * @p_data: message pointer
 * @n_bytes: message size
 *
 * This function is a callback to indicate SEC message reception is complete.
 * It calls update_queue() to update the message list to indicate arrival
 * of new message.
 */
static void sec_receive(void *p_data, u32 n_bytes)
{
	struct t_message_queue *q;
	unsigned long flag;
	struct t_isadev_context *secdev = p_isa_context_hsi->p_isadev[3];
	q = &secdev->dl_queue;
	dbg_printk("rpc_receive++\n");
	spin_lock_irqsave(&q->update_lock, flag);
	update_queue(q, WRITE_COMPLETE, n_bytes);
	spin_unlock_irqrestore(&q->update_lock, flag);
	dbg_printk("rpc_receive--\n");
}
/**
 * rpc_receive() - Rx Completion callback
 *
 * @p_data: message pointer
 * @n_bytes: message size
 *
 * This function is a callback to indicate RPC message reception is complete.
 * It calls update_queue() to update the message list to indicate arrival
 * of new message.
 */
static void rpc_receive(void *p_data, u32 n_bytes)
{
	struct t_message_queue *q;
	unsigned long flag;
	struct t_isadev_context *rpcdev = p_isa_context_hsi->p_isadev[1];
	q = &rpcdev->dl_queue;
	dbg_printk("rpc_receive++\n");
	spin_lock_irqsave(&q->update_lock, flag);
	update_queue(q, WRITE_COMPLETE, n_bytes);
	spin_unlock_irqrestore(&q->update_lock, flag);
	dbg_printk("rpc_receive--\n");
}
/**
 * isi_receive() - Rx Completion callback
 *
 * @p_data: message pointer
 * @n_bytes: message size
 *
 * This function is a callback to indicate ISI message reception is complete.
 * It calls update_queue() to update the message list to indicate arrival
 * of new message.
 */
static void isi_receive(void *p_data, u32 n_bytes)
{
	struct t_message_queue *q;
	unsigned long flag;
	struct t_isadev_context *isidev = p_isa_context_hsi->p_isadev[0];
	dbg_printk("isi_receive++\n");
	q = &isidev->dl_queue;
	spin_lock_irqsave(&q->update_lock, flag);
	update_queue(q, WRITE_COMPLETE, n_bytes);
	spin_unlock_irqrestore(&q->update_lock, flag);
	dbg_printk("isi_receive--\n");
}

/**
 * audio_receive() - Rx Completion callback
 *
 * @p_data: message pointer
 * @n_bytes: message size
 *
 * This function is a callback to indicate ISI message reception is complete.
 * It calls update_queue() to update the message list to indicate arrival
 * of new message.
 */
static void audio_receive(void *p_data, u32 n_bytes)
{
	struct t_message_queue *q;
	unsigned long flag;
	struct t_isadev_context *audiodev = p_isa_context_hsi->p_isadev[2];
	dbg_printk("isi_receive++\n");
	q = &audiodev->dl_queue;
	spin_lock_irqsave(&q->update_lock, flag);
	update_queue(q, WRITE_COMPLETE, n_bytes);
	spin_unlock_irqrestore(&q->update_lock, flag);
	dbg_printk("isi_receive--\n");
}

/**
 * l3receive() - Rx Completion callback
 *
 * @msg_ptr: message pointer
 * @length: message size
 * @l2_msg_header: L2 message header
 *
 * This function is a callback to indicate RPC message reception is complete.
 * It calls update_queue() to update the message list to indicate arrival
 * of new message.
 */
void l3receive(void *msg_ptr, u32 length, u8 l2_msg_header)
{
	struct t_isadev_context *l3dev;

	l3dev = p_isa_context_hsi->p_isadev[l2_msg_header];

	if (l3dev->device_id == 0)
		isi_receive(msg_ptr, length);
	if (l3dev->device_id == 1)
		rpc_receive(msg_ptr, length);
	if (l3dev->device_id == 2)
		audio_receive(msg_ptr, length);
	if (l3dev->device_id == 3)
		sec_receive(msg_ptr, length);


}
EXPORT_SYMBOL(l3receive);
/**
 * isa_select() - Select Interface
 *
 * @filp: file descriptor pointer
 * @wait:  poll_table_struct pointer
 *
 * This function is used to perform non-blocking read operations. It allows
 * a process to determine whether it can read from one or more open files
 * without blocking. These calls can also block a process until any of a
 * given set of file descriptors becomes available for reading.
 * If a file is ready to read, POLLIN | POLLRDNORM bitmask is returned.
 * The driver method is called whenever the user-space program performs a
 * select system call involving a file descriptor associated with the driver.
 */
static unsigned int isa_select(struct file *filp, \
		struct poll_table_struct *wait)
{
	struct t_isadev_context *p_isadev;
	struct t_message_queue *q;
	unsigned int mask = 0;
	u32 m = iminor(filp->f_path.dentry->d_inode);
	dbg_printk("isa_select++\n");
	p_isadev = (struct t_isadev_context *)filp->private_data;

	if (p_isadev->device_id != m)
		return -1;

	q = &p_isadev->dl_queue;

	poll_wait(filp, &q->wq_readable, wait);

	if (atomic_read(&q->q_rp) == 1)
		mask = POLLIN | POLLRDNORM;

	dbg_printk("isa_select--\n");
	return mask;
}
/**
 * isa_ioctl() - To handle different ioctl commands supported by driver.
 *
 * @inode: structure is used by the kernel internally to represent files
 * @filp: file descriptor pointer
 * @cmd: ioctl command
 * @arg: input param
 *
 * IOCTL support removed. Use read/write instead.
 */
static int isa_ioctl(struct inode *inode, struct file *filp,
				unsigned cmd, unsigned long arg)
{
	int err = 0;

	switch (cmd) {
		/** DLP_IOC_ALLOCATE_BUFFER - allocate uplink buffer */
		case DLP_IOC_ALLOCATE_BUFFER: {
			dbg_printk(" DLP_IOC_ALLOCATE_BUFFER++\n");
			dbg_printk(" DLP_IOC_ALLOCATE_BUFFER--\n");
			break;
		}
		/** DLP_IOC_PUT_MESSAGE - This ioctl is used to signal
		that message is copied to the uplink buffer and ready for
		transmission. */
		case DLP_IOC_PUT_MESSAGE: {
			dbg_printk(" DLP_IOC_PUT_MESSAGE++\n");
			dbg_printk(" DLP_IOC_PUT_MESSAGE--\n");
			break;
		}
		/** DLP_IOC_GET_MESSAGE - This ioctl is used to poll
		any downlink message */
		case DLP_IOC_GET_MESSAGE: {
			dbg_printk(" DLP_IOC_GET_MESSAGE++\n");
			dbg_printk(" DLP_IOC_GET_MESSAGE--\n");
			break;
		}
		/** DLP_IOC_DEALLOCATE_BUFFER- This ioctl is used to \
		deallocate buffer reserved for received downlink message */
		case DLP_IOC_DEALLOCATE_BUFFER: {
			dbg_printk(" DLP_IOC_DEALLOCATE_BUFFER++\n");
			dbg_printk(" DLP_IOC_DEALLOCATE_BUFFER--\n");
			break;
		}
		default: {
			dbg_printk("Unknown IOCTL\n");
			err = -1;
			break;
		}
	};
	return err;
}
/**
 * isa_mmap() - Maps kernel queue memory to user space.
 *
 * @filp: file descriptor pointer
 * @vma: virtual area memory structure.
 *
 * This function maps kernel FIFO into user space. This function
 * shall be called twice to map both uplink and downlink buffers.
 */
static int isa_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct t_isadev_context *p_isadev;
	int err = 0;
	u8 direction;
	u32 m = iminor(filp->f_path.dentry->d_inode);
	dbg_printk("isa_mmap %d++\n", m);

	p_isadev = (struct t_isadev_context *)filp->private_data;
	direction = vma->vm_pgoff;
	vma->vm_pgoff = 0;

	switch (direction) {
		case MMAP_DLQUEUE: {
			dbg_printk(" MMAP_DLQUEUE\n");
			vma->vm_flags |= VM_RESERVED;
			vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
			if (remap_pfn_range(vma, vma->vm_start, \
				p_isadev->dl_queue.phy_address >> PAGE_SHIFT,\
					vma->vm_end-vma->vm_start, \
					vma->vm_page_prot)) {
					err =  -EAGAIN;
				}
		}
		break;
		case MMAP_ULQUEUE: {
			dbg_printk(" MMAP_ULQUEUE\n");
			vma->vm_flags |= VM_RESERVED;
			vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
			if (remap_pfn_range(vma, vma->vm_start, \
				p_isadev->ul_queue.phy_address >> PAGE_SHIFT,
					vma->vm_end-vma->vm_start, \
					vma->vm_page_prot)) {
					err =  -EAGAIN;
			}
		}
		break;
		default: {
			printk(KERN_ALERT NAME ": unknown mmap called\n");
			err = -EAGAIN;
		}
	};
	dbg_printk(" isa_mmap--\n");
	return err;
}
/**
 * isa_read() - Read from device
 *
 * @filp: file descriptor
 * @buf: user buffer pointer
 * @len: size of requested data transfer
 * @ppos: not used
 *
 * This function is called whenever user calls read() system call.
 * It reads a oldest message from queue and copies it into user buffer and
 * returns its size.
 * If there is no message present in queue, then it blocks until new data is
 * available.
 */
static ssize_t isa_read(struct file *filp, char __user *buf,
				size_t len, loff_t *ppos)
{
	int err = 0;
	struct t_isadev_context *p_isadev;
	struct t_message_queue *q;
	struct t_dlp_message message;
	void *addr;
	u32 size;
	unsigned long flag;
	dbg_printk("isa_read++\n");

	if (len <= 0 || buf == NULL)
		return -EFAULT;

	p_isadev = (struct t_isadev_context *)filp->private_data;
	q = &p_isadev->dl_queue;

	spin_lock_irqsave(&q->update_lock, flag);
	if (list_empty(&q->msg_list)) {
		spin_unlock_irqrestore(&q->update_lock, flag);
		dbg_printk("%s:Waiting for RP\n", \
				(q->direction == 1) ? "UL" : "DL");
		if (wait_event_interruptible(q->wq_readable, \
			atomic_read(&q->q_rp) == 1)) {
			return -ERESTARTSYS;
		}
	} else
		spin_unlock_irqrestore(&q->update_lock, flag);

	err = get_new_msg(q, &message);
	if (err < 0) {
		printk(KERN_ALERT "get_new_msg Failed\n");
		return -EAGAIN;
	}
	size = message.size;
	addr = q->log_address + message.offset;

	if (copy_to_user(buf, addr, size)) {
		printk(KERN_ALERT NAME ":copy_to_user failed\n");
		return -EFAULT;
	}
	spin_lock_irqsave(&q->update_lock, flag);
	update_queue(q, READ_COMPLETE, message.size);
	spin_unlock_irqrestore(&q->update_lock, flag);
	dbg_printk("isa_read-- %d\n", size);
	return size;
}
/**
 * isa_write() - Write to device
 *
 * @filp: file descriptor
 * @buf: user buffer pointer
 * @len: size of requested data transfer
 * @ppos: not used
 *
 * This function is called whenever user calls write() system call.
 * It checks if there is space available in queue, and copies the message
 * inside queue. If there is no space, it blocks until space becomes available.
 * It also schedules transfer thread to transmit the newly added message.
 */
static ssize_t isa_write(struct file *filp, const char __user *buf,
				 size_t len, loff_t *ppos)
{
	struct t_isadev_context *p_isadev;
	struct t_message_queue *q;
	int err = 0;
	struct t_data_buf addr;

	dbg_printk("isa_write++\n");

	if (len <= 0 || (len > 8*1024))
		return -EFAULT;

	p_isadev = (struct t_isadev_context *)filp->private_data;
	q = &p_isadev->ul_queue;
	dbg_printk("%s\n", (p_isadev->device_id == 0) ? "ISI" : "RPC");

	down(&q->tx_mutex);

	/** protection when close happens*/
	if (atomic_dec_and_test(&p_isa_context_hsi->isOpen[p_isadev->device_id]))
		return -EFAULT;

	if (copy_from_user((void *)(q->log_address), buf, len)) {
		printk(KERN_ALERT NAME ":copy_from_user failed\n");
		up(&q->tx_mutex);
		return -EFAULT;
	}

	addr.log_address = q->log_address;
	addr.phys_address = q->phy_address;

	err = hsi_msg_send((void *)&addr, p_isadev->device_id, \
				align_4byte(len));
	if (err < 0)
		return err;

	dbg_printk("isa_write--\n");
	up(&q->tx_mutex);
	return len;
}
/**
 * isa_close() - Close device file
 *
 * @inode: structure is used by the kernel internally to represent files
 * @filp:  device file descriptor
 *
 * This function deletes structues associated with this file, deletes
 * queues, flushes and destroys workqueus and closes this file.
 * It also unregisters itself from l2mux driver.
 */
static int isa_close(struct inode *inode, struct file *filp)
{
	struct t_isadev_context *p_isadev;
	u8 m;
	struct t_message_queue *q;
	mutex_lock(&isa_lock);
	m = iminor(filp->f_path.dentry->d_inode);
	dbg_printk("isa_close %d", m);

	if (atomic_dec_and_test(&p_isa_context_hsi->isOpen[m])) {
		atomic_inc(&p_isa_context_hsi->isOpen[m]);
		printk(KERN_ALERT NAME ":Device not opened yet\n");
		mutex_unlock(&isa_lock);
		return -ENODEV;
	}
	atomic_set(&p_isa_context_hsi->isOpen[m], 1);

	p_isadev = (struct t_isadev_context *)filp->private_data;
	dbg_printk("p_isadev->device_id %d", p_isadev->device_id);
	q = &p_isadev->ul_queue;
	/**unblock any write thread*/
	up(&q->tx_mutex);

	dbg_printk("Closed %d device\n", m);
	mutex_unlock(&isa_lock);
	return 0;
}
/**
 * isa_open() -  Open device file
 *
 * @inode: structure is used by the kernel internally to represent files
 * @filp:  device file descriptor
 *
 * This function performs initialization tasks needed to open HSI channel.
 * Following tasks are performed.
 * -return if device is already opened
 * -create uplink FIFO
 * -create downlink FIFO
 * -init delayed workqueue thread
 * -register to l2mux driver
 */
static int isa_open(struct inode *inode, struct file *filp)
{
	int err = 0;
	u8 m;
	struct t_isadev_context *p_isadev;
	dbg_printk("isa_open++\n");

	mutex_lock(&isa_lock);
	m = iminor(inode);

	if ((m != ISI_MESSAGING) && \
		(m != RPC_MESSAGING) && \
		(m != AUDIO_MESSAGING) && \
		(m != SEC_MESSAGING)) {
		printk(KERN_ALERT NAME ":No such device present\n");
		mutex_unlock(&isa_lock);
		return -ENODEV;
	}
	if (!atomic_dec_and_test(&p_isa_context_hsi->isOpen[m])) {
		atomic_inc(&p_isa_context_hsi->isOpen[m]);
		printk(KERN_ALERT NAME ":Device already opened\n");
		mutex_unlock(&isa_lock);
		return -EBUSY;
	}

	p_isadev = p_isa_context_hsi->p_isadev[m];
	filp->private_data = p_isadev;

	mutex_unlock(&isa_lock);
	dbg_printk("isa_open--\n");
	return err;
}

const struct file_operations hsi_fops = { \
	.owner	= THIS_MODULE,
	.open 	= isa_open,
	.release = isa_close,
	.ioctl 	= isa_ioctl,
	.mmap 	= isa_mmap,
	.read 	= isa_read,
	.write 	= isa_write,
	.poll	= isa_select,
};
/**
 * isa_init() - module insertion function
 *
 * This function registers module as a character driver using
 * register_chrdev_region()
 * or alloc_chrdev_region. It adds this driver to system using cdev_add() call.
 * Major number is dynamically allocated using alloc_chrdev_region()
 * by default or left to user to specify it during load time.
 * For this variable major is used as module_param
 *
 * Nodes to be created using
 * mknod /dev/isi c $major 0
 * mknod /dev/rpc c $major 1
 * mknod /dev/audio c $major 2
 * mknod /dev/secure c $major 3
 */
static int __init isa_init(void)
{
	int err = 0;
	dev_t dev_id;
	int retval, no_dev;
	struct t_isadev_context *p_isadev;

	p_isa_context_hsi = kzalloc(sizeof(struct t_isa_driver_context), \
				GFP_KERNEL);
	if (p_isa_context_hsi == NULL) {
		printk(KERN_ALERT NAME ":Failed to alloc memory\n");
		return -ENOMEM;
	}

	if (major) {
		dev_id = MKDEV(major, 0);
		retval = register_chrdev_region(dev_id, ISA_DEVICES, NAME);
	} else {
		retval = alloc_chrdev_region(&dev_id, 0, ISA_DEVICES, NAME);
		major = MAJOR(dev_id);
	}

	printk(KERN_ALERT NAME ": major %d\n", major);

	cdev_init(&p_isa_context_hsi->cdev, &hsi_fops);
	p_isa_context_hsi->cdev.owner = THIS_MODULE;
	retval = cdev_add(&p_isa_context_hsi->cdev, dev_id, ISA_DEVICES);
	if (retval) {
		printk(KERN_ALERT NAME ":Failed to add char device\n");
		return retval;
	}

	for (no_dev = 0; no_dev < ISA_DEVICES; no_dev++)
		atomic_set(&p_isa_context_hsi->isOpen[no_dev], 1);

	for (no_dev = 0; no_dev < ISA_DEVICES; no_dev++) {

		p_isadev = kzalloc(sizeof(struct t_isadev_context), GFP_KERNEL);
		if (p_isadev == NULL) {
			printk(KERN_ALERT NAME ":Failed to alloc memory\n");
			return -ENOMEM;
		}

		p_isadev->device_id = no_dev;

		retval = create_queue(&p_isadev->ul_queue, 8*1024);
		if (retval < 0) {
			printk(KERN_ALERT NAME ":create_queue ul_q failed\n");
			kfree(p_isadev);
			return retval;
		}
		p_isadev->ul_queue.direction = 1;

		retval = create_queue(&p_isadev->dl_queue, COMMON_BUFFER_SIZE);
		if (retval < 0) {
			printk(KERN_ALERT NAME ":create_queue dl_q failed\n");
			delete_queue(&p_isadev->ul_queue);
			kfree(p_isadev);
			return retval;
		}
		p_isadev->dl_queue.direction = 0;

		p_isa_context_hsi->p_isadev[p_isadev->device_id] = p_isadev;
	}

	err = dlp_protocol_init();
	if (err < 0)
		return err;

	/** protocol start*/
	dlp_protocol_start(0);

	printk(KERN_ALERT NAME ": Driver added\n");

	return retval;
}
/**
 * isa_exit() - module removal function
 *
 * This function removes this driver from system.
 */
static void __exit isa_exit(void)
{
	int no_dev;
	struct t_isadev_context *p_isadev;
	dev_t dev_id = MKDEV(major, 0);

	for (no_dev = 0; no_dev < ISA_DEVICES; no_dev++) {
		p_isadev = p_isa_context_hsi->p_isadev[no_dev];
		delete_queue(&p_isadev->ul_queue);
		delete_queue(&p_isadev->dl_queue);
		kfree(p_isadev);
	}

	cdev_del(&p_isa_context_hsi->cdev);
	unregister_chrdev_region(dev_id, ISA_DEVICES);
	kfree(p_isa_context_hsi);

	printk(KERN_ALERT NAME ": Driver removed\n");
}

module_init(isa_init);
module_exit(isa_exit);

MODULE_AUTHOR(ISA_ACCESS_DRIVER_AUTHOR);
MODULE_DESCRIPTION(ISA_ACCESS_DRIVER_DESC);
MODULE_LICENSE("GPL");
