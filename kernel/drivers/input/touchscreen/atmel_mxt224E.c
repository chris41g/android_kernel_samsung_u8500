/*
*  atmel_maxtouch.c - Atmel maXTouch Touchscreen Controller
*
*  Version 0.2a
*
*  An early alpha version of the maXTouch Linux driver.
*
*
*  Copyright (C) 2010 Iiro Valkonen <iiro.valkonen@atmel.com>
*  Copyright (C) 2009 Ulf Samuelsson <ulf.samuelsson@atmel.com>
*  Copyright (C) 2009 Raphael Derosso Pereira <raphaelpereira@gmail.com>
*
*  This program is free software; you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation; either version 2 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program; if not, write to the Free Software
*  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#define	DEBUG_INFO      1
#define DEBUG_VERBOSE   2
#define	DEBUG_MESSAGES  5
#define	DEBUG_RAW       8
#define	DEBUG_TRACE     10

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>

#include <linux/init.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/random.h>
#include <linux/major.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/rcupdate.h>
#include <linux/smp_lock.h>

#include <linux/delay.h>
#include <linux/atmel_mxt224E.h>

/*#include <mach/gpio-n1.h>*/
#include <linux/reboot.h>
#include <linux/regulator/consumer.h>


/*
* This is a driver for the Atmel maXTouch Object Protocol
*
* When the driver is loaded, mxt_init is called.
* mxt_driver registers the "mxt_driver" structure in the i2c subsystem
* The mxt_idtable.name string allows the board support to associate
* the driver with its own data.
*
* The i2c subsystem will call the mxt_driver.probe == mxt_probe
* to detect the device.
* mxt_probe will reset the maXTouch device, and then
* determine the capabilities of the I2C peripheral in the
* host processor (needs to support BYTE transfers)
*
* If OK; mxt_probe will try to identify which maXTouch device it is
* by calling mxt_identify.
*
* If a known device is found, a linux input device is initialized
* the "mxt" device data structure is allocated
* as well as an input device structure "mxt->input"
* "mxt->client" is provided as a parameter to mxt_probe.
*
* mxt_read_object_table is called to determine which objects
* are present in the device, and to determine their adresses
*
*
* Addressing an object:
*
* The object is located at a 16 address in the object address space
*
* The object address can vary between revisions of the firmware
*
* The address is provided through an object descriptor table in the beginning
* of the object address space.
* It is assumed that an object type is only listed once in this table,
* Each object type can have several instances, and the number of
* instances is available in the object table
*
* The base address of the first instance of an object is stored in
* "mxt->object_table[object_type].chip_addr",
* This is indexed by the object type and allows direct access to the
* first instance of an object.
*
* Each instance of an object is assigned a "Report Id" uniquely identifying
* this instance. Information about this instance is available in the
* "mxt->report_id" variable, which is a table indexed by the "Report Id".
*
* The maXTouch object protocol supports adding a checksum to messages.
* By setting the most significant bit of the maXTouch address
* an 8 bit checksum is added to all writes.
*
*
* How to use driver.
* -----------------
* Example:
* In arch/avr32/boards/atstk1000/atstk1002.c
* an "i2c_board_info" descriptor is declared.
* This contains info about which driver ("mXT224"),
* which i2c address and which pin for CHG interrupts are used.
*
* In the "atstk1002_init" routine, "i2c_register_board_info" is invoked
* with this information. Also, the I/O pins are configured, and the I2C
* controller registered is on the application processor.
*
*/

#if defined(CONFIG_MACH_N1) || defined(CONFIG_MACH_JANICE)
/*#define MXT_FIRMUP_ENABLE */		/* for auto firmware upgrade *//*TODO*/
/*#define MXT_TUNNING_ENABLE */		/* for TS tunning using console  */
#define MXT_FACTORY_TEST  /*for TS factory test */
#define MXT_SLEEP_POWEROFF
#define MXT_ESD_WORKAROUND
#define MXT_DRIVER_FILTER
#define MXT_THREADED_IRQ
#define ENABLE_NOISE_TEST_MODE	1
#endif

/* TSP Key */
#define NUMOFKEYS       2

#define KEY_PRESS       1
#define KEY_RELEASE     0

int tsp_keycodes[NUMOFKEYS] = {
#if 0
	KEY_MENU,
		KEY_HOME,
		KEY_BACK,
		KEY_SEARCH
#else
		KEY_MENU,
		KEY_BACK,
#endif
};

char *tsp_keyname[NUMOFKEYS] = {
#if 0
	"Menu",
		"Home",
		"Back",
		"Search"
#else
		"Menu",
		"Back",
#endif
};

static u16 tsp_keystatus;


int mxt_debug = 1;
static int debug = DEBUG_INFO;

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Activate debugging output");

#if defined(MXT_FACTORY_TEST) || defined(MXT_FIRMUP_ENABLE)
unsigned int mxt_firm_status_data;
unsigned int mxt_firm_normal_status_ack;
#endif
#if ENABLE_NOISE_TEST_MODE
/*botton_right, botton_left, center, top_right, top_left */
u16 test_node[5] = {325, 45, 700, 1356, 1140};

unsigned int return_refer_0, return_refer_1, return_refer_2, return_refer_3, return_refer_4;
unsigned int return_delta_0, return_delta_1, return_delta_2, return_delta_3, return_delta_4;
#endif


#ifdef CONFIG_HAS_EARLYSUSPEND
static struct mxt_data *s_mxt;
static void mxt_early_suspend(struct early_suspend *h);
static void mxt_late_resume(struct early_suspend *h);
#endif


#define I2C_RETRY_COUNT 10


/* Returns the start address of object in mXT memory. */
#define	MXT_BASE_ADDR(object_type) \
mxt_get_object_address(object_type, 0, mxt->object_table, mxt->device_info.num_objs)

/* If report_id (rid) == 0, then "mxt->report_id[rid].object" will be 0. */
#define	REPORT_ID_TO_OBJECT(rid) \
(((rid) == 0xff) ? 0 : mxt->rid_map[rid].object)

#define	REPORT_ID_TO_OBJECT_NAME(rid) \
object_type_name[REPORT_ID_TO_OBJECT(rid)]

#define	T6_REG(x) (MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6) + (x))
#define	T37_REG(x) (MXT_BASE_ADDR(MXT_DEBUG_DIAGNOSTICS_T37) +  (x))

#if defined(CONFIG_MACH_N1) || defined(CONFIG_MACH_JANICE)
#if 1 /* TO_CHK :ADD TRACKING_ID */
#define REPORT_MT(touch_number, x, y, amplitude, size)		\
	do {														\
	input_report_abs(mxt->input, ABS_MT_TRACKING_ID, touch_number); \
	input_report_abs(mxt->input, ABS_MT_POSITION_X, x);             \
	input_report_abs(mxt->input, ABS_MT_POSITION_Y, y);             \
	input_report_abs(mxt->input, ABS_MT_TOUCH_MAJOR, amplitude);    \
	input_report_abs(mxt->input, ABS_MT_WIDTH_MAJOR, size);         \
	input_mt_sync(mxt->input);                                      \
	} while (0)
#else  /* TO_CHK : S1, P1 */
#define REPORT_MT(x, y, amplitude, size)                            \
	do {                                                                \
	input_report_abs(mxt->input, ABS_MT_POSITION_X, x);             \
	input_report_abs(mxt->input, ABS_MT_POSITION_Y, y);             \
	input_report_abs(mxt->input, ABS_MT_TOUCH_MAJOR, amplitude);    \
	input_report_abs(mxt->input, ABS_MT_WIDTH_MAJOR, size); \
	input_mt_sync(mxt->input);                                      \
	} while (0)
#endif
#else/*original Atmel */
#define REPORT_MT(touch_number, size, x, y)                         \
	do {                                                                \
	input_report_abs(mxt->input, ABS_MT_TRACKING_ID, touch_number); \
	input_report_abs(mxt->input, ABS_MT_TOUCH_MAJOR, size);         \
	input_report_abs(mxt->input, ABS_MT_POSITION_X, x);             \
	input_report_abs(mxt->input, ABS_MT_POSITION_Y, y);             \
	input_mt_sync(mxt->input);                                      \
	} while (0)
#endif


const u8 *maxtouch_family = "maXTouch";
const u8 *mxt224_variant  = "mXT224E";

u8	*object_type_name[MXT_MAX_OBJECT_TYPES]	= {
		[37]	= "DEBUG_DIAGNOSTICS_T37",
		[5]	    = "GEN_MESSAGEPROCESSOR_T5",
		[6]	    = "GEN_COMMANDPROCESSOR_T6",
		[38]	= "USER_DATA_T38",
		[7]	    = "GEN_POWERCONFIG_T7",
		[8]	    = "GEN_ACQUIRECONFIG_T8",
		[9]	    = "TOUCH_MULTITOUCHSCREEN_T9",
		[15]	= "TOUCH_KEYARRAY_T15",
		[18]	= "SPT_COMMSCONFIG_T18",
		[24]	= "PROCI_ONETOUCHGESTUREPROCESSOR_T24",
		[25]	= "SPT_SELFTEST_T25",
		[40]	= "PROCI_GRIPSUPPRESSION_T40",
		[42]	= "PROCI_TOUCHSUPPRESSION_T42",
		[46]	= "SPT_CTECONFIG_T46",
		[47]	= "PROCI_STYLUS_T47",
		[48]	= "PROCG_NOISESUPPRESSION_T48",
		/*	[19]	= "T19 - Obsolete", */
		/*	[22]	= "PROCG_NOISESUPPRESSION_T22", */
		/*	[23]	= "T23 - Obsolete", */
		/*	[24]	= "", */
		/*	[25]	= "", */
		/*	[26]	= "T26 - Obsolete", */
		/*	[27]	= "PROCI_TWOTOUCHGESTUREPROCESSOR_T27", */
		/*	[28]	= "SPT_CTECONFIG_T28", */
		/*	[37]	= "", */
		/*	[38]	= "", */
		/*	[40]	= "", */
		/*	[42]	= "", */
		/*	[46]    = "", */
		/*	[47]	= "", */
		/*	[48]	= "", */
};

#if 1/* _SUPPORT_MULTITOUCH_ */
struct multi_touch_info {
	uint16_t size;
	int16_t pressure;
	int16_t x;
	int16_t y;
};

static struct multi_touch_info mtouch_info[MXT_MAX_NUM_TOUCHES];
#endif

/*
* declaration of external functions
*/
extern int mxt_config_settings(struct mxt_data *mxt);
extern int mxt_get_object_values(struct mxt_data *mxt, int obj_type);
extern int mxt_copy_object(struct mxt_data *mxt, u8 *buf, int obj_type);
extern int mxt_load_firmware(struct device *dev, const char *fn);
extern int mxt_power_config(struct mxt_data *mxt);

int	backup_to_nv(struct mxt_data *mxt);
int	mxt_reset_chip(struct mxt_data *mxt, u8 mode);
#ifdef MXT_ESD_WORKAROUND
void mxt_force_reset(struct mxt_data *mxt);
#endif

#if defined(MXT_DRIVER_FILTER)
static void equalize_coordinate(bool detect, u8 id, u16 *px, u16 *py)
{
	static int tcount[MXT_MAX_NUM_TOUCHES] = { 0, };
	static u16 pre_x[MXT_MAX_NUM_TOUCHES][4] = {{0}, };
	static u16 pre_y[MXT_MAX_NUM_TOUCHES][4] = {{0}, };
	int coff[4] = {0,};
	int distance = 0;

	if (detect) {
		tcount[id] = 0;
	}

	pre_x[id][tcount[id]%4] = *px;
	pre_y[id][tcount[id]%4] = *py;

	if (tcount[id] > 3)	{
		distance = abs(pre_x[id][(tcount[id]-1)%4] - *px) + abs(pre_y[id][(tcount[id]-1)%4] - *py);

		coff[0] = (u8)(4 + distance/5);
		if (coff[0] < 8) {
			coff[0] = max(4, coff[0]);
			coff[1] = min((10 - coff[0]), (coff[0]>>1)+1);
			coff[2] = min((10 - coff[0] - coff[1]), (coff[1]>>1)+1);
			coff[3] = 10 - coff[0] - coff[1] - coff[2];

			printk(KERN_DEBUG "[TSP] %d, %d, %d, %d \n", coff[0], coff[1], coff[2], coff[3]);
			*px = (u16)((*px*(coff[0]) + pre_x[id][(tcount[id]-1)%4]*(coff[1])
				+ pre_x[id][(tcount[id]-2)%4]*(coff[2]) + pre_x[id][(tcount[id]-3)%4]*(coff[3]))/10);
			*py = (u16)((*py*(coff[0]) + pre_y[id][(tcount[id]-1)%4]*(coff[1])
				+ pre_y[id][(tcount[id]-2)%4]*(coff[2]) + pre_y[id][(tcount[id]-3)%4]*(coff[3]))/10);
		} else {
			*px = (u16)((*px*4 + pre_x[id][(tcount[id]-1)%4])/5);
			*py = (u16)((*py*4 + pre_y[id][(tcount[id]-1)%4])/5);
		}
	}
	tcount[id]++;
}
#endif  /* MXT_DRIVER_FILTER */

/* Calculates the 24-bit CRC sum. */

static u32 CRC_24(u32 crc, u8 byte1, u8 byte2)
{
	static const u32 crcpoly = 0x80001B;
	u32 result;
	u16 data_word;

	data_word = (u16) ((u16) (byte2 << 8u) | byte1);
	result = ((crc << 1u) ^ (u32) data_word);
	if (result & 0x1000000)
		result ^= crcpoly;
	return result;
}

/* Returns object address in mXT chip, or zero if object is not found */
u16 mxt_get_object_address(uint8_t object_type,
					   uint8_t instance,
					   struct mxt_object *object_table,
					   int max_objs)
{
	uint8_t object_table_index = 0;
	uint8_t address_found = 0;
	uint16_t address = 0;

	struct mxt_object obj;

	while ((object_table_index < max_objs) && !address_found) {
		obj = object_table[object_table_index];
		if (obj.type == object_type) {
			address_found = 1;
			/* Are there enough instances defined in the FW? */
			if (obj.instances >= instance) {
				address = obj.chip_addr +
					(obj.size + 1) * instance;
			} else {
				return 0;
			}
		}
		object_table_index++;
	}

	return address;
}

/* Returns object size in mXT chip, or zero if object is not found */
u16 mxt_get_object_size(uint8_t object_type, struct mxt_object *object_table, int max_objs)
{
	uint8_t object_table_index = 0;
	struct mxt_object obj;

	while (object_table_index < max_objs) {
		obj = object_table[object_table_index];
		if (obj.type == object_type) {
			return obj.size;
		}
		object_table_index++;
	}
	return 0;
}

/*
* Reads one byte from given address from mXT chip (which requires
* writing the 16-bit address pointer first).
*/

int mxt_read_byte(struct i2c_client *client, u16 addr, u8 *value)
{
	struct i2c_adapter *adapter = client->adapter;
	struct i2c_msg msg[2];
	__le16 le_addr = cpu_to_le16(addr);
	struct mxt_data *mxt;

	mxt = i2c_get_clientdata(client);


	msg[0].addr  = client->addr;
	msg[0].flags = 0x00;
	msg[0].len   = 2;
	msg[0].buf   = (u8 *) &le_addr;

	msg[1].addr  = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = 1;
	msg[1].buf   = (u8 *) value;
	if  (i2c_transfer(adapter, msg, 2) == 2) {
		mxt->last_read_addr = addr;
		return 0;
	} else {
	/*
	* In case the transfer failed, set last read addr to invalid
	* address, so that the next reads won't get confused.
	*/
		mxt->last_read_addr = -1;
		return -EIO;
	}
}

/*
* Reads a block of bytes from given address from mXT chip. If we are
* reading from message window, and previous read was from message window,
* there's no need to write the address pointer: the mXT chip will
* automatically set the address pointer back to message window start.
*/

int mxt_read_block(struct i2c_client *client,
				   u16 addr,
				   u16 length,
				   u8 *value)
{
	struct i2c_adapter *adapter = client->adapter;
	struct i2c_msg msg[2];
	__le16	le_addr;
	struct mxt_data *mxt;

	mxt = i2c_get_clientdata(client);

	if (mxt != NULL) {
		if ((mxt->last_read_addr == addr) &&
			(addr == mxt->msg_proc_addr)) {
			if  (i2c_master_recv(client, value, length) == length)
				return 0;
			else
				{
				printk(KERN_INFO "[tsp] %s, %d, i2c_master_recv err\n",
					__func__, __LINE__);
				return -EIO;
				}
		} else {
			mxt->last_read_addr = addr;
		}
	}

	le_addr = cpu_to_le16(addr);
	msg[0].addr  = client->addr;
	msg[0].flags = 0x00;
	msg[0].len   = 2;
	msg[0].buf   = (u8 *) &le_addr;

	msg[1].addr  = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = length;
	msg[1].buf   = (u8 *) value;
	if  (i2c_transfer(adapter, msg, 2) == 2)
		return 0;
	else
		return -EIO;

}

/* Reads a block of bytes from current address from mXT chip. */

int mxt_read_block_wo_addr(struct i2c_client *client,
						   u16 length,
						   u8 *value)
{


	if  (i2c_master_recv(client, value, length) == length) {
		printk(KERN_INFO "read ok\n");
		return length;
	} else {
		printk(KERN_WARNING "read failed\n");
		return -EIO;
	}

}


/* Writes one byte to given address in mXT chip. */

int mxt_write_byte(struct i2c_client *client, u16 addr, u8 value)
{
	struct {
		__le16 le_addr;
		u8 data;

	} i2c_byte_transfer;

	struct mxt_data *mxt;

	mxt = i2c_get_clientdata(client);
	if (mxt != NULL)
		mxt->last_read_addr = -1;

	i2c_byte_transfer.le_addr = cpu_to_le16(addr);
	i2c_byte_transfer.data = value;


	if  (i2c_master_send(client, (u8 *) &i2c_byte_transfer, 3) == 3)
		return 0;
	else
		return -EIO;
}


/* Writes a block of bytes (max 256) to given address in mXT chip. */

int mxt_write_block(struct i2c_client *client,
					u16 addr,
					u16 length,
					u8 *value)
{
	int i;
	struct {
		__le16	le_addr;
		u8	data[256];

	} i2c_block_transfer;

	struct mxt_data *mxt;

	if (length > 256)
		return -EINVAL;

	mxt = i2c_get_clientdata(client);
	if (mxt != NULL)
		mxt->last_read_addr = -1;



	for (i = 0; i < length; i++)
		i2c_block_transfer.data[i] = *value++;


	i2c_block_transfer.le_addr = cpu_to_le16(addr);

	i = i2c_master_send(client, (u8 *) &i2c_block_transfer, length + 2);

	if (i == (length + 2))
		return length;
	else
		return -EIO;
}

/* TODO: make all other access block until the read has been done? Otherwise
an arriving message for example could set the ap to message window, and then
the read would be done from wrong address! */

/* Writes the address pointer (to set up following reads). */

int mxt_write_ap(struct i2c_client *client, u16 ap)
{

	__le16	le_ap = cpu_to_le16(ap);
	struct mxt_data *mxt;

	mxt = i2c_get_clientdata(client);
	if (mxt != NULL)
		mxt->last_read_addr = -1;

	printk(KERN_INFO "Address pointer set to %d\n", ap);

	if (i2c_master_send(client, (u8 *) &le_ap, 2) == 2)
		return 0;
	else
		return -EIO;
}

/* Calculates the CRC value for mXT infoblock. */
static int calculate_infoblock_crc(u32 *crc_result, struct mxt_data *mxt)
{
	u32 crc = 0;
	u16 crc_area_size;
	u8 *mem;
	int i;

	int error;
	struct i2c_client *client;

	client = mxt->client;

	crc_area_size = MXT_ID_BLOCK_SIZE +
		mxt->device_info.num_objs * MXT_OBJECT_TABLE_ELEMENT_SIZE;

	mem = kmalloc(crc_area_size, GFP_KERNEL);

	if (mem == NULL) {
		dev_err(&client->dev, "Error allocating memory\n");
		return -ENOMEM;
	}

	error = mxt_read_block(client, 0, crc_area_size, mem);
	if (error < 0) {
		kfree(mem);
		return error;
	}

	for (i = 0; i < (crc_area_size - 1); i = i + 2)
		crc = CRC_24(crc, *(mem + i), *(mem + i + 1));

	/* If uneven size, pad with zero */
	if (crc_area_size & 0x0001)
		crc = CRC_24(crc, *(mem + i), 0);

	kfree(mem);

	/* Return only 24 bits of CRC. */
	*crc_result = (crc & 0x00FFFFFF);
	return 1;

}

#if defined(CONFIG_MACH_N1) || defined(CONFIG_MACH_JANICE)
void process_T9_message(u8 *message, struct mxt_data *mxt)
{
	struct	input_dev *input;
	u8  status;
	u16 xpos = 0xFFFF;
	u16 ypos = 0xFFFF;
	u8 report_id;
	u8 touch_id;  /* to identify each touches. starts from 0 to 15 */
	u8 pressed_or_released = 0;
	static int prev_touch_id = -1;
	int i;
	u16 tmp;

	//printk("[TSP] %s\n", __func__);

	input = mxt->input;
	status = message[MXT_MSG_T9_STATUS];
	report_id = message[0];
	touch_id = report_id - 2;

	if (touch_id >= MXT_MAX_NUM_TOUCHES) {
		kloge("[TSP] Invalid touch_id (toud_id=%d)", touch_id);
		return;
	}

	/* calculate positions of x, y */
	ypos = message[2];
	ypos = ypos << 4;
	ypos = ypos | (message[4] >> 4);
	ypos >>= 2;

	xpos = message[3];
	xpos = xpos << 4;
	xpos = xpos | (message[4] & 0x0F);
	xpos >>= 2;

#ifdef CONFIG_MACH_JANICE
	// X, Y change and invert values
	tmp = xpos;
	xpos = ypos;
	ypos = tmp;

	xpos = 480 - xpos;
	ypos = 800 - ypos;
#endif

	if (status & MXT_MSGB_T9_DETECT) {  /* case 1: detected */


		mtouch_info[touch_id].pressure = message[MXT_MSG_T9_TCHAMPLITUDE];  /* touch amplitude */
		mtouch_info[touch_id].x = (int16_t)xpos;
		mtouch_info[touch_id].y = (int16_t)ypos;

		if (status & MXT_MSGB_T9_PRESS) {
			pressed_or_released = 1;  /* pressed */
			#if defined(MXT_DRIVER_FILTER)
			equalize_coordinate(1, touch_id, &mtouch_info[touch_id].x, &mtouch_info[touch_id].y);
			#endif
		} else if (status & MXT_MSGB_T9_MOVE) {
			#if defined(MXT_DRIVER_FILTER)
			equalize_coordinate(0, touch_id, &mtouch_info[touch_id].x, &mtouch_info[touch_id].y);
			#endif
		}
	} else if (status & MXT_MSGB_T9_RELEASE) {  /* case 2: released */
		pressed_or_released = 1;
		mtouch_info[touch_id].pressure = 0;
	} else if (status & MXT_MSGB_T9_SUPPRESS) {  /* case 3: suppressed */
	/*
	* Atmel's recommendation:
	* In the case of supression, mxt224E chip doesn't make a release event.
	* So we need to release them forcibly.
	*/
		pressed_or_released = 1;
		mtouch_info[touch_id].pressure = 0;
	} else {
		kloge("[TSP] Unknown status (0x%x)", status);
	}

	/*only get size , id would use TRACKING_ID*/
	mtouch_info[touch_id].size = message[MXT_MSG_T9_TCHAREA];  /* TO_CHK: what's the meaning of this? */

	if (prev_touch_id >= touch_id || pressed_or_released) {
		for (i = 0; i < MXT_MAX_NUM_TOUCHES; ++i) {
			if (mtouch_info[i].pressure == -1)
				continue;

			/* ADD TRACKING_ID*/
			REPORT_MT(i, mtouch_info[i].x, mtouch_info[i].y, mtouch_info[i].pressure, mtouch_info[i].size);

			if (mtouch_info[i].pressure == 0)   /* if released */
				mtouch_info[i].pressure = -1;
		}
		input_sync(input);  /* TO_CHK: correct position? */
	}
	prev_touch_id = touch_id;

	//if (mxt_debug && (debug >= DEBUG_MESSAGES)) {
		char msg[64] = {0};
		char info[64] = {0};

		if (status & MXT_MSGB_T9_SUPPRESS) {
			strcpy(msg, "Suppress: ");
		} else {
			if (status & MXT_MSGB_T9_DETECT) {
				strcpy(msg, "Detect(");
				if (status & MXT_MSGB_T9_PRESS)
					strcat(msg, "P");
				if (status & MXT_MSGB_T9_MOVE)
					strcat(msg, "M");
				if (status & MXT_MSGB_T9_AMP)
					strcat(msg, "A");
				if (status & MXT_MSGB_T9_VECTOR)
					strcat(msg, "V");
				strcat(msg, "): ");
			} else if (status & MXT_MSGB_T9_RELEASE) {
				strcpy(msg, "Release: ");
			} else {
				strcpy(msg, "[!] Unknown status: ");
			}
		}
		sprintf(info, "(%d,%d) amp=%d, size=%d", xpos, ypos,
			message[MXT_MSG_T9_TCHAMPLITUDE], message[MXT_MSG_T9_TCHAREA]);
		strcat(msg, info);
		printk(KERN_INFO "[TSP] %s\n", msg);
	//}

	return;
}

void process_T15_message(u8 *message, struct mxt_data *mxt)
{
	struct	input_dev *input;
	u8  status, reported = false;
	int i;

	printk(KERN_INFO "[TSP] %s\n", __func__);

	input = mxt->input;
	status = message[MXT_MSG_T15_STATUS];

	/* whether reportid is thing of QT602240_TOUCH_KEYARRAY */
	/* single key configuration*/

	if (message[MXT_MSG_T15_STATUS] == MXT_MSGB_T15_DETECT) {
		if (message[MXT_MSG_T15_KEYSTATE] == 0x01) {
			input_report_key(mxt->input, KEY_MENU, 1);
			klogi("[TSP] MENU is pressed. Keycode : %d\n", message[MXT_MSG_T15_KEYSTATE]);
			tsp_keystatus = KEY_MENU;
		} else if (message[MXT_MSG_T15_KEYSTATE] == 0x02) {
			input_report_key(mxt->input, KEY_BACK, 1);
			klogi("[TSP] BACK is pressed. Keycode : %d\n", message[MXT_MSG_T15_KEYSTATE]);
			tsp_keystatus = KEY_BACK;
		}
	} else {
		if (tsp_keystatus == KEY_MENU) {
			input_report_key(mxt->input, KEY_MENU, 0);
			klogi("[TSP] MENU is release. Keycode : %d\n", message[MXT_MSG_T15_KEYSTATE]);
		} else if (tsp_keystatus == KEY_BACK) {
			input_report_key(mxt->input, KEY_BACK, 0);
			klogi("[TSP] BACK is release. Keycode : %d\n", message[MXT_MSG_T15_KEYSTATE]);
		}

		tsp_keystatus = 0;
	}

#if 0
	for (i = 0; i < NUMOFKEYS; i++) {
		if (tsp_keystatus[i]) {
			input_report_key(input, tsp_keycodes[i], 0);
			printk(KERN_DEBUG "[TSP] %s key is release. Keycode : %d\n", tsp_keyname[i], tsp_keycodes[i]);
			tsp_keystatus[i] = KEY_RELEASE;
		} else if (message[MXT_MSG_T15_KEYSTATE] & (0x1<<i)) {
			if (message[MXT_MSG_T15_STATUS] & MXT_MSGB_T15_DETECT)	{	/* detect */
				/* set_dvfs_perf_level(); */
				input_report_key(input, tsp_keycodes[i], 1);
				printk(KERN_DEBUG "[TSP] %s key is pressed. Keycode : %d\n", tsp_keyname[i], tsp_keycodes[i]);
				tsp_keystatus[i] = KEY_PRESS;
			}
		}
	}
#endif
	return;
}

#else
void process_T9_message(u8 *message, struct mxt_data *mxt)
{

	struct	input_dev *input;
	u8  status;
	u16 xpos = 0xFFFF;
	u16 ypos = 0xFFFF;
	u8  touch_size = 255;
	u8  touch_number;
	u8  amplitude;
	u8  report_id;

	input = mxt->input;
	status = message[MXT_MSG_T9_STATUS];
	report_id = message[0];

	if (status & MXT_MSGB_T9_SUPPRESS) {
		/* Touch has been suppressed by grip/face */
		/* detection                              */
		if (debug >= DEBUG_TRACE)
			printk(KERN_INFO "SUPRESS ");
	} else {
		xpos = message[MXT_MSG_T9_XPOSMSB] << 4 +
			((message[MXT_MSG_T9_XYPOSLSB] >> 4) & 0xF);
		ypos = message[MXT_MSG_T9_YPOSMSB] << 4 +
			((message[MXT_MSG_T9_XYPOSLSB] >> 0) & 0xF);

		xpos >>= 2;
		ypos >>= 2;

		touch_number = message[MXT_MSG_REPORTID] -
			mxt->rid_map[report_id].first_rid;

		if (status & MXT_MSGB_T9_DETECT) {
		/*
		* TODO: more precise touch size calculation?
		* mXT224 reports the number of touched nodes,
		* so the exact value for touch ellipse major
		* axis length would be 2*sqrt(touch_size/pi)
		* (assuming round touch shape).
			*/
			touch_size = message[MXT_MSG_T9_TCHAREA];
			touch_size = touch_size >> 2;
			if (!touch_size)
				touch_size = 1;
			REPORT_MT(touch_number, touch_size, xpos, ypos);
			if (status & MXT_MSGB_T9_AMP)
				/* Amplitude of touch has changed */
				amplitude = message[MXT_MSG_T9_TCHAMPLITUDE];
		} else if (status & MXT_MSGB_T9_RELEASE) {
			/* The previously reported touch has been removed.*/
			REPORT_MT(touch_number, 0, xpos, ypos);
		}

		input_sync(input);
	}
	if (debug >= DEBUG_TRACE) {
		if (status & MXT_MSGB_T9_SUPPRESS) {
			printk(KERN_INFO "SUPRESS ");
		} else {
			if (status & MXT_MSGB_T9_DETECT) {
				printk(KERN_INFO "DETECT: ");
				if (status & MXT_MSGB_T9_PRESS)
					printk(KERN_INFO "PRESS ");
				if (status & MXT_MSGB_T9_MOVE)
					printk(KERN_INFO "MOVE ");
				if (status & MXT_MSGB_T9_AMP)
					printk(KERN_INFO "AMP ");
				if (status & MXT_MSGB_T9_VECTOR)
					printk(KERN_INFO "VECTOR ");
			} else if (status & MXT_MSGB_T9_RELEASE) {
				printk(KERN_INFO "RELEASE: ");
			}
		}
	}
	if (debug >= DEBUG_MESSAGES)
		printk(KERN_INFO "X=%d, Y=%d, TOUCHSIZE=%d\n",
		xpos, ypos, touch_size);
	return;
}
#endif

int process_message(u8 *message, u8 object, struct mxt_data *mxt)
{

	struct i2c_client *client;

	u8  status;
	u16 xpos = 0xFFFF;
	u16 ypos = 0xFFFF;
	u8  event;
	u8  length;
	u8  report_id;

	client = mxt->client;
	length = mxt->message_size;
	report_id = message[0];

	if (mxt_debug && (debug >= DEBUG_MESSAGES))
		kloge_if("process_message 0: (0x%x) 1:(0x%x) object:(%d)", message[0], message[1], object);

	switch (object) {
	case MXT_GEN_COMMANDPROCESSOR_T6:
		status = message[1];
		if (status & MXT_MSGB_T6_COMSERR) {
			dev_err(&client->dev,
				"maXTouch checksum error\n");
		}
		if (status & MXT_MSGB_T6_CFGERR) {
			dev_err(&client->dev,
				"maXTouch configuration error\n");
		mxt_reset_chip(mxt, RESET_TO_NORMAL);
		msleep(100);
		mxt_config_settings(mxt);
		}
		if (status & MXT_MSGB_T6_CAL) {
			dev_info(&client->dev,
				"maXTouch calibration in progress\n");
		}
		if (status & MXT_MSGB_T6_SIGERR) {
			dev_err(&client->dev,
				"maXTouch acquisition error\n");
#ifdef MXT_ESD_WORKAROUND
			mxt_force_reset(mxt);
#endif
		}
		if (status & MXT_MSGB_T6_OFL) {
			dev_err(&client->dev,
				"maXTouch cycle overflow\n");
		}
		if (status & MXT_MSGB_T6_RESET) {
			dev_info(&client->dev,
				"maXTouch chip reset\n");
		}
		if (status == 0) {
			dev_info(&client->dev,
				"maXTouch status normal\n");
#if defined(MXT_FACTORY_TEST) || defined(MXT_FIRMUP_ENABLE)
			if (mxt_firm_status_data == 1) {	/*firmware started*/
				dev_info(&client->dev,
					"maXTouch mxt_firm_normal_status_ack after firm up\n");
				mxt_firm_normal_status_ack = 1;/*got normal status ack*/
			}
#endif
		}
		break;

	case MXT_TOUCH_MULTITOUCHSCREEN_T9:
		process_T9_message(message, mxt);
		break;

	case MXT_TOUCH_KEYARRAY_T15:
		process_T15_message(message, mxt);
		break;
#if 0  /* note: deprecated in ver0.9 */
	case MXT_SPT_GPIOPWM_T19:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev,
			"Receiving GPIO message\n");
		break;


	case MXT_PROCI_GRIPFACESUPPRESSION_T20:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev,
			"Receiving face suppression msg\n");
		break;
#endif

#if 0	/* CHK IT */
	case MXT_PROCG_NOISESUPPRESSION_T22:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev,
			"Receiving noise suppression msg\n");
		status = message[MXT_MSG_T22_STATUS];
		if (status & MXT_MSGB_T22_FHCHG) {
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev,
				"maXTouch: Freq changed\n");
		}
		if (status & MXT_MSGB_T22_GCAFERR) {
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev,
				"maXTouch: High noise "
				"level\n");
		}
		if (status & MXT_MSGB_T22_FHERR) {
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev,
				"maXTouch: Freq changed - "
				"Noise level too high\n");

		}
		break;
#endif
	case MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev,
			"Receiving one-touch gesture msg\n");

		event = message[MXT_MSG_T24_STATUS] & 0x0F;
		xpos = message[MXT_MSG_T24_XPOSMSB] * 16 +
			((message[MXT_MSG_T24_XYPOSLSB] >> 4) & 0x0F);
		ypos = message[MXT_MSG_T24_YPOSMSB] * 16 +
			((message[MXT_MSG_T24_XYPOSLSB] >> 0) & 0x0F);
		xpos >>= 2;
		ypos >>= 2;

		switch (event) {
		case	MT_GESTURE_RESERVED:
			break;
		case	MT_GESTURE_PRESS:
			break;
		case	MT_GESTURE_RELEASE:
			break;
		case	MT_GESTURE_TAP:
			break;
		case	MT_GESTURE_DOUBLE_TAP:
			break;
		case	MT_GESTURE_FLICK:
			break;
		case	MT_GESTURE_DRAG:
			break;
		case	MT_GESTURE_SHORT_PRESS:
			break;
		case	MT_GESTURE_LONG_PRESS:
			break;
		case	MT_GESTURE_REPEAT_PRESS:
			break;
		case	MT_GESTURE_TAP_AND_PRESS:
			break;
		case	MT_GESTURE_THROW:
			break;
		default:
			break;
		}
		break;

	case MXT_SPT_SELFTEST_T25:
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev,
				"Receiving Self-Test msg\n");

			if (message[MXT_MSG_T25_STATUS] == MXT_MSGR_T25_OK) {
				if (debug >= DEBUG_TRACE)
					dev_info(&client->dev,
					"maXTouch: Self-Test OK\n");

			} else  {
				dev_err(&client->dev,
					"maXTouch: Self-Test Failed [%02x]:"
					"{%02x,%02x,%02x,%02x,%02x}\n",
					message[MXT_MSG_T25_STATUS],
					message[MXT_MSG_T25_STATUS + 0],
					message[MXT_MSG_T25_STATUS + 1],
					message[MXT_MSG_T25_STATUS + 2],
					message[MXT_MSG_T25_STATUS + 3],
					message[MXT_MSG_T25_STATUS + 4]
					);
			}
			break;
#if 0
	case MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27:
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev,
				"Receiving 2-touch gesture message\n");
			break;
#endif
	case MXT_SPT_CTECONFIG_T46:
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev,
				"Receiving CTE message...\n");
			status = message[MXT_MSG_T46_STATUS];
			if (status & MXT_MSGB_T46_CHKERR)
				dev_err(&client->dev,
				"maXTouch: Power-Up CRC failure\n");

			break;
	default:
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev,
				"maXTouch: Unknown message!\n");

			break;
	}

	return 0;
}


#ifdef MXT_THREADED_IRQ
/* Processes messages when the interrupt line (CHG) is asserted. */

static void mxt_threaded_irq_handler(struct mxt_data *mxt)
{
	struct	i2c_client *client;

	u8	*message;
	u16	message_length;
	u16	message_addr;
	u8	report_id;
	u8	object;
	int	error;
	int	i;

	message = NULL;
	client = mxt->client;
	message_addr =
		mxt->msg_proc_addr;
	message_length =
		mxt->message_size;
	if (message_length < 256) {
		message = kmalloc(message_length, GFP_KERNEL);
		if (message == NULL) {
			dev_err(&client->dev, "[TSP] Error allocating memory\n");
			return;
		}
	} else {
		dev_err(&client->dev,
			"[TSP] Message length larger than 256 bytes not supported\n");
	}

	if (debug >= DEBUG_TRACE)
		dev_info(&mxt->client->dev, "[TSP] maXTouch worker active: \n");

	do {
		/* Read next message */
		mxt->message_counter++;
		/* Reread on failure! */
		for (i = 1; i < I2C_RETRY_COUNT; i++) {
			/* note: changed message_length to 8 in ver0.9*/
			error = mxt_read_block(client, message_addr, 8/*message_length*/, message);
			if (error >= 0)
				break;
			mxt->read_fail_counter++;
			printk(KERN_ALERT "[TSP] mXT: message read failed!\n");
			/* Register read failed */
			dev_err(&client->dev,
				"[TSP] Failure reading maxTouch device\n");
		}

		report_id = message[0];
		if (debug >= DEBUG_RAW) {
			printk(KERN_INFO "[TSP] %s message [%08x]:",
				REPORT_ID_TO_OBJECT_NAME(report_id),
				mxt->message_counter
				);
			for (i = 0; i < message_length; i++) {
				printk(KERN_INFO "[TSP] 0x%02x ", message[i]);;
			}
			printk(KERN_INFO "\n");
		}

		if ((report_id != MXT_END_OF_MESSAGES) && (report_id != 0)) {

			for (i = 0; i < message_length; i++)
				mxt->last_message[i] = message[i];

			if (down_interruptible(&mxt->msg_sem)) {
				printk(KERN_WARNING "[TSP] mxt_worker Interrupted "
					"while waiting for msg_sem!\n");
				kfree(message);
				return;
			}
			mxt->new_msgs = 1;
			up(&mxt->msg_sem);
			wake_up_interruptible(&mxt->msg_queue);
			/* Get type of object and process the message */
			object = mxt->rid_map[report_id].object;
			process_message(message, object, mxt);
		}

	} while ((report_id != MXT_END_OF_MESSAGES) && (report_id != 0));
	kfree(message);
}

static irqreturn_t mxt_threaded_irq(int irq, void *_mxt)
{
	struct	mxt_data *mxt = _mxt;
	mxt->irq_counter++;
	mxt_threaded_irq_handler(mxt);
	return IRQ_HANDLED;
}

#else
/* Processes messages when the interrupt line (CHG) is asserted. */

static void mxt_worker(struct work_struct *work)
{
	struct	mxt_data *mxt;
	struct	i2c_client *client;

	u8	*message;
	u16	message_length;
	u16	message_addr;
	u8	report_id;
	u8	object;
	int	error;
	int	i;

	message = NULL;
	mxt = container_of(work, struct mxt_data, dwork.work);
	client = mxt->client;
	message_addr = mxt->msg_proc_addr;
	message_length = mxt->message_size;

	if (message_length < 256) {
		message = kmalloc(message_length, GFP_KERNEL);
		if (message == NULL) {
			dev_err(&client->dev, "[TSP] Error allocating memory\n");
			return;
		}
	} else {
		dev_err(&client->dev,
			"[TSP] Message length larger than 256 bytes not supported\n");
	}

	if (debug >= DEBUG_TRACE)
		dev_info(&mxt->client->dev, "[TSP] maXTouch worker active: \n");

	do {
		/* Read next message */
		mxt->message_counter++;
		/* Reread on failure! */
		for (i = 1; i < I2C_RETRY_COUNT; i++) {
			/* note: changed message_length to 8 in ver0.9 */
			error = mxt_read_block(client, message_addr, 8/*message_length*/, message);
			if (error >= 0)
				break;
			mxt->read_fail_counter++;
			printk(KERN_ALERT "[TSP] mXT: message read failed!\n");
			/* Register read failed */
			dev_err(&client->dev,
				"[TSP] Failure reading maxTouch device\n");
		}

		report_id = message[0];
		if (debug >= DEBUG_RAW) {
			printk(KERN_INFO "%s message [%08x]:",
				REPORT_ID_TO_OBJECT_NAME(report_id),
				mxt->message_counter
				);
			for (i = 0; i < message_length; i++) {
				printk(KERN_INFO "0x%02x ", message[i]);;
			}
			printk(KERN_INFO "\n");
		}

		if ((report_id != MXT_END_OF_MESSAGES) && (report_id != 0)) {

			for (i = 0; i < message_length; i++)
				mxt->last_message[i] = message[i];

			if (down_interruptible(&mxt->msg_sem)) {
				printk(KERN_WARNING "[TSP] mxt_worker Interrupted "
					"while waiting for msg_sem!\n");
				kfree(message);
				return;
			}
			mxt->new_msgs = 1;
			up(&mxt->msg_sem);
			wake_up_interruptible(&mxt->msg_queue);
			/* Get type of object and process the message */
			object = mxt->rid_map[report_id].object;
			process_message(message, object, mxt);
		}

	} while ((report_id != MXT_END_OF_MESSAGES) && (report_id != 0));

	kfree(message);
}


/*
* The maXTouch device will signal the host about a new message by asserting
* the CHG line. This ISR schedules a worker routine to read the message when
* that happens.
*/

static irqreturn_t mxt_irq_handler(int irq, void *_mxt)
{
	struct	mxt_data *mxt = _mxt;
	unsigned long	flags;
	mxt->irq_counter++;
	spin_lock_irqsave(&mxt->lock, flags);

	if (mxt_valid_interrupt()) {
		/* Send the signal only if falling edge generated the irq. */
		cancel_delayed_work(&mxt->dwork);
		schedule_delayed_work(&mxt->dwork, 0);
		mxt->valid_irq_counter++;
	} else {
		mxt->invalid_irq_counter++;
	}
	spin_unlock_irqrestore(&mxt->lock, flags);

	return IRQ_HANDLED;
}
#endif

/******************************************************************************/
/* sysfs entries                                                              */
/******************************************************************************/

int read_object(struct device *dev, u16 object, char *message)
{
	struct i2c_client	*client = to_i2c_client(dev);
	struct mxt_data		*mxt = i2c_get_clientdata(client);
	u16			chip_addr;
	u16			size;
	int			error;
	if (object >= MXT_MAX_OBJECT_TYPES)
		return -EINVAL;

	chip_addr = mxt_get_object_address(object, 0, mxt->object_table,
		mxt->device_info.num_objs);
	if (chip_addr == 0)
		return -EIO;

	size = mxt->object_table[object].size;
	error = mxt_read_block(client, chip_addr, size, message);
	if (error < 0) {
		mxt->read_fail_counter++;
		return error;
	} else
		return size;
}

char	*print_object(char *object, char *bufp, int size)
{
	char *p;
	int  i, max;
	char *header;

	header = "\t\t\t\t";
	p = object;
	bufp += sprintf(bufp, "Object:");
	while (size > 0) {
		bufp += sprintf(bufp, header);
		max = (size > 8) ? 8 : size;
		for (i = 0; i < max; i++)
			bufp += sprintf(bufp, "0x%02x ", *p++);

		bufp += sprintf(bufp, "\n");
		size -= max;
	}
	return bufp;
}


/* Function to write a block of data to any address on touch chip. */

#define I2C_PAYLOAD_SIZE 254

static ssize_t set_config(struct device *dev,
						  struct device_attribute *attr,
						  const char *buf,
						  size_t count)
{
	int i;

	u16 address;
	int whole_blocks;
	int last_block_size;

	struct i2c_client *client  = to_i2c_client(dev);

	address = *((u16 *) buf);
	address = cpu_to_be16(address);
	buf += 2;

	whole_blocks = (count - 2) / I2C_PAYLOAD_SIZE;
	last_block_size = (count - 2) % I2C_PAYLOAD_SIZE;

	for (i = 0; i < whole_blocks; i++) {
		mxt_write_block(client, address, I2C_PAYLOAD_SIZE, (u8 *) buf);
		address += I2C_PAYLOAD_SIZE;
		buf += I2C_PAYLOAD_SIZE;
	}

	mxt_write_block(client, address, last_block_size, (u8 *) buf);

	return count;

}

static ssize_t get_config(struct device *dev,
						  struct device_attribute *attr,
						  char *buf)
{
	int i;
	struct i2c_client *client  = to_i2c_client(dev);
	struct mxt_data *mxt = i2c_get_clientdata(client);

	printk(KERN_WARNING "Reading %d bytes from current ap\n",
		mxt->bytes_to_read);

	i = mxt_read_block_wo_addr(client, mxt->bytes_to_read, (u8 *) buf);

	return (ssize_t) i;

}

/*
* Sets up a read from mXT chip. If we want to read config data from user space
* we need to use this first to tell the address and byte count, then use
* get_config to read the data.
*/

static ssize_t set_ap(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf,
					  size_t count)
{

	int i;
	struct i2c_client *client;
	struct mxt_data *mxt;
	u16 ap;

	client = to_i2c_client(dev);
	mxt = i2c_get_clientdata(client);

	if (count < 3) {
		/* Error, ap needs to be two bytes, plus 1 for size! */
		printk(KERN_INFO "set_ap needs to arguments: address pointer "
			"and data size");
		return -EIO;
	}

	ap = (u16) *((u16 *)buf);
	i = mxt_write_ap(client, ap);
	mxt->bytes_to_read = (u16) *(buf + 2);
	return count;

}


static ssize_t show_deltas(struct device *dev,
						   struct device_attribute *attr,
						   char *buf)
{
	struct i2c_client *client;
	struct mxt_data *mxt;
	s16     *delta;
	s16     size, read_size;
	u16     diagnostics;
	u16     debug_diagnostics;
	char    *bufp;
	int     x, y;
	int     error;
	u16     *val;

	client = to_i2c_client(dev);
	mxt = i2c_get_clientdata(client);

	/* Allocate buffer for delta's */
	size = mxt->device_info.num_nodes * sizeof(__u16);
	if (mxt->delta == NULL) {
		mxt->delta = kzalloc(size, GFP_KERNEL);
		if (!mxt->delta) {
			sprintf(buf, "insufficient memory\n");
			return strlen(buf);
		}
	}

	if (mxt->object_table[MXT_GEN_COMMANDPROCESSOR_T6].type == 0) {
		dev_err(&client->dev, "[TSP] maXTouch: Object T6 not found\n");
		return 0;
	}
	diagnostics =  T6_REG(MXT_ADR_T6_DIAGNOSTICS);
	if (mxt->object_table[MXT_DEBUG_DIAGNOSTICS_T37].type == 0) {
		dev_err(&client->dev, "[TSP] maXTouch: Object T37 not found\n");
		return 0;
	}
	debug_diagnostics = T37_REG(2);

	/* Configure T37 to show deltas */
	error = mxt_write_byte(client, diagnostics, MXT_CMD_T6_DELTAS_MODE);
	if (error)
		return error;

	delta = mxt->delta;

	while (size > 0) {
		read_size = size > 128 ? 128 : size;
		error = mxt_read_block(client,
			debug_diagnostics,
			read_size,
			(__u8 *) delta);
		if (error < 0) {
			mxt->read_fail_counter++;
			dev_err(&client->dev,
				"[TSP] maXTouch: Error reading delta object\n");
		}
		delta += (read_size / 2);
		size -= read_size;
		/* Select next page */
		mxt_write_byte(client, diagnostics, MXT_CMD_T6_PAGE_UP);
	}

	bufp = buf;
	val  = (s16 *) mxt->delta;
	for (x = 0; x < mxt->device_info.x_size; x++) {
		for (y = 0; y < mxt->device_info.y_size; y++)
			bufp += sprintf(bufp, "%05d  ",
			(s16) le16_to_cpu(*val++));
		bufp -= 2;	/* No spaces at the end */
		bufp += sprintf(bufp, "\n");
	}
	bufp += sprintf(bufp, "\n");
	return strlen(buf);
}


static ssize_t show_references(struct device *dev,
							   struct device_attribute *attr,
							   char *buf)
{
	struct i2c_client *client;
	struct mxt_data *mxt;
	s16   *reference;
	s16   size, read_size;
	u16   diagnostics;
	u16   debug_diagnostics;
	char  *bufp;
	int   x, y;
	int   error;
	u16   *val;

	client = to_i2c_client(dev);
	mxt = i2c_get_clientdata(client);
	/* Allocate buffer for reference's */
	size = mxt->device_info.num_nodes * sizeof(u16);
	if (mxt->reference == NULL) {
		mxt->reference = kzalloc(size, GFP_KERNEL);
		if (!mxt->reference) {
			sprintf(buf, "insufficient memory\n");
			return strlen(buf);
		}
	}

	if (mxt->object_table[MXT_GEN_COMMANDPROCESSOR_T6].type == 0) {
		dev_err(&client->dev, "[TSP] maXTouch: Object T6 not found\n");
		return 0;
	}
	diagnostics =  T6_REG(MXT_ADR_T6_DIAGNOSTICS);
	if (mxt->object_table[MXT_DEBUG_DIAGNOSTICS_T37].type == 0) {
		dev_err(&client->dev, "[TSP] maXTouch: Object T37 not found\n");
		return 0;
	}
	debug_diagnostics = T37_REG(2);

	/* Configure T37 to show references */
	mxt_write_byte(client, diagnostics, MXT_CMD_T6_REFERENCES_MODE);
	/* Should check for error */
	reference = mxt->reference;
	while (size > 0) {
		read_size = size > 128 ? 128 : size;
		error = mxt_read_block(client,
			debug_diagnostics,
			read_size,
			(__u8 *) reference);
		if (error < 0) {
			mxt->read_fail_counter++;
			dev_err(&client->dev,
				"[TSP] maXTouch: Error reading reference object\n");
		}
		reference += (read_size / 2);
		size -= read_size;
		/* Select next page */
		mxt_write_byte(client, diagnostics, MXT_CMD_T6_PAGE_UP);
	}

	bufp = buf;
	val  = (u16 *) mxt->reference;

	for (x = 0; x < mxt->device_info.x_size; x++) {
		for (y = 0; y < mxt->device_info.y_size; y++)
			bufp += sprintf(bufp, "%05d  ", le16_to_cpu(*val++));
		bufp -= 2; /* No spaces at the end */
		bufp += sprintf(bufp, "\n");
	}
	bufp += sprintf(bufp, "\n");
	return strlen(buf);
}

static ssize_t show_device_info(struct device *dev,
								struct device_attribute *attr,
								char *buf)
{
	struct i2c_client *client;
	struct mxt_data *mxt;
	char *bufp;

	client = to_i2c_client(dev);
	mxt = i2c_get_clientdata(client);

	bufp = buf;
	bufp += sprintf(bufp,
		"Family:\t\t\t[0x%02x] %s\n",
		mxt->device_info.family_id,
		mxt->device_info.family
		);
	bufp += sprintf(bufp,
		"Variant:\t\t[0x%02x] %s\n",
		mxt->device_info.variant_id,
		mxt->device_info.variant
		);
	bufp += sprintf(bufp,
		"Firmware version:\t[%d.%d], build 0x%02X\n",
		mxt->device_info.major,
		mxt->device_info.minor,
		mxt->device_info.build
		);
	bufp += sprintf(bufp,
		"%d Sensor nodes:\t[X=%d, Y=%d]\n",
		mxt->device_info.num_nodes,
		mxt->device_info.x_size,
		mxt->device_info.y_size
		);
	bufp += sprintf(bufp,
		"Reported resolution:\t[X=%d, Y=%d]\n",
		mxt->max_x_val+1,
		mxt->max_y_val+1
		);
	return strlen(buf);
}

static ssize_t show_stat(struct device *dev,
						 struct device_attribute *attr,
						 char *buf)
{
	struct i2c_client *client;
	struct mxt_data *mxt;
	char *bufp;

	client = to_i2c_client(dev);
	mxt = i2c_get_clientdata(client);

	bufp = buf;
	bufp += sprintf(bufp,
		"Interrupts:\t[VALID=%d ; INVALID=%d]\n",
		mxt->valid_irq_counter,
		mxt->invalid_irq_counter
		);
	bufp += sprintf(bufp, "Messages:\t[%d]\n", mxt->message_counter);
	bufp += sprintf(bufp, "Read Failures:\t[%d]\n", mxt->read_fail_counter);
	return strlen(buf);
}

static ssize_t show_object_info(struct device *dev,
								struct device_attribute *attr,
								char *buf)
{
	struct i2c_client	*client;
	struct mxt_data		*mxt;
	char			*bufp;
	struct mxt_object	*object_table;
	int			i;

	client = to_i2c_client(dev);
	mxt = i2c_get_clientdata(client);
	object_table = mxt->object_table;

	bufp = buf;

	bufp += sprintf(bufp, "maXTouch: %d Objects\n",
		mxt->device_info.num_objs);

	for (i = 0; i < mxt->device_info.num_objs; i++) {
		if (object_table[i].type != 0) {
			bufp += sprintf(bufp,
				"Type:\t\t[%d]: %s\n",
				object_table[i].type,
				object_type_name[object_table[i].type]);
			bufp += sprintf(bufp,
				"Address:\t0x%04X\n",
				object_table[i].chip_addr);
			bufp += sprintf(bufp,
				"Size:\t\t%d Bytes\n",
				object_table[i].size);
			bufp += sprintf(bufp,
				"Instances:\t%d\n",
				object_table[i].instances
				);
			bufp += sprintf(bufp,
				"Report Id's:\t%d\n\n",
				object_table[i].num_report_ids);
		}
	}
	return strlen(buf);
}

static ssize_t show_messages(struct device *dev,
							 struct device_attribute *attr,
							 char *buf)
{
	struct i2c_client *client;
	struct mxt_data   *mxt;
	struct mxt_object *object_table;
	int   i;
	__u8  *message;
	__u16 message_len;
	__u16 message_addr;

	char  *bufp;

	client = to_i2c_client(dev);
	mxt = i2c_get_clientdata(client);
	object_table = mxt->object_table;

	bufp = buf;

	message = kmalloc(mxt->message_size, GFP_KERNEL);
	if (message == NULL) {
		printk(KERN_WARNING "Error allocating memory!\n");
		return -ENOMEM;
	}

	message_addr = mxt->msg_proc_addr;
	message_len = mxt->message_size;
	bufp += sprintf(bufp,
		"Reading Message Window [0x%04x]\n",
		message_addr);

	/* Acquire the lock. */
	if (down_interruptible(&mxt->msg_sem)) {
		printk(KERN_INFO "mxt: Interrupted while waiting for mutex!\n");
		kfree(message);
		return -ERESTARTSYS;
	}

	while (mxt->new_msgs == 0) {
		/* Release the lock. */
		up(&mxt->msg_sem);
		if (wait_event_interruptible(mxt->msg_queue, mxt->new_msgs)) {
			printk(KERN_INFO
				"mxt: Interrupted while waiting for new msg!\n");
			kfree(message);
			return -ERESTARTSYS;
		}

		/* Acquire the lock. */
		if (down_interruptible(&mxt->msg_sem)) {
			printk(KERN_INFO
				"mxt: Interrupted while waiting for mutex!\n");
			kfree(message);
			return -ERESTARTSYS;
		}

	}

	for (i = 0; i < mxt->message_size; i++)
		message[i] = mxt->last_message[i];

	mxt->new_msgs = 0;

	/* Release the lock. */
	up(&mxt->msg_sem);

	for (i = 0; i < message_len; i++)
		bufp += sprintf(bufp, "0x%02x ", message[i]);
	bufp--;
	bufp += sprintf(bufp, "\t%s\n", REPORT_ID_TO_OBJECT_NAME(message[0]));

	kfree(message);
	return strlen(buf);
}


static ssize_t show_report_id(struct device *dev,
							  struct device_attribute *attr,
							  char *buf)
{
	struct i2c_client    *client;
	struct mxt_data      *mxt;
	struct report_id_map *report_id;
	int                  i;
	int                  object;
	char                 *bufp;

	client    = to_i2c_client(dev);
	mxt       = i2c_get_clientdata(client);
	report_id = mxt->rid_map;

	bufp = buf;
	for (i = 0 ; i < mxt->report_id_count ; i++) {
		object = report_id[i].object;
		bufp += sprintf(bufp, "Report Id [%03d], object [%03d], "
			"instance [%03d]:\t%s\n",
			i,
			object,
			report_id[i].instance,
			object_type_name[object]);
	}
	return strlen(buf);
}

static ssize_t set_debug(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int state;

	sscanf(buf, "%d", &state);
	if (state == 0 || state == 1) {
		mxt_debug = state;
		if (state)
			klogi("touch info enabled");
		else
			klogi("touch info disabled");
	} else
		return -EINVAL;

	return count;
}

static ssize_t show_firmware(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt_data *mxt = dev_get_drvdata(dev);
	u8 val[7];

	mxt_read_block(mxt->client, MXT_ADDR_INFO_BLOCK, 7, (u8 *)val);
	mxt->device_info.major = ((val[2] >> 4) & 0x0F);
	mxt->device_info.minor = (val[2] & 0x0F);
	mxt->device_info.build	= val[3];

	return snprintf(buf, PAGE_SIZE,
		"Atmel %s Firmware version [%d.%d] Build %d\n",
		mxt224_variant,
		mxt->device_info.major,
		mxt->device_info.minor,
		mxt->device_info.build);
}

static ssize_t store_firmware(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int state;
	struct mxt_data *mxt = dev_get_drvdata(dev);

	if (sscanf(buf, "%i", &state) != 1 || (state < 0 || state > 1))
		return -EINVAL;

	wake_lock(&mxt->wakelock);  /* prevents the system from entering suspend during updating */
	disable_irq(mxt->client->irq);

	mxt_load_firmware(dev, MXT224E_FIRMWARE);
	msleep(100);
	enable_irq(mxt->client->irq);
	wake_unlock(&mxt->wakelock);

	return count;
}


#ifdef MXT_FIRMUP_ENABLE
static void set_mxt_auto_update_exe(struct device *dev)
{
	int ret;
	struct mxt_data *mxt = dev_get_drvdata(dev);
	klogi("set_mxt_auto_update_exe \n");

	ret = mxt_load_firmware(&mxt->client->dev, MXT224E_FIRMWARE);

	if (ret >= 0) {
		mxt_firm_status_data = 2;	/*firmware update success */
		klogi("[MXT]Reprogram done : Firmware update Success~~~~~~~~~~\n");
	} else {
		mxt_firm_status_data = 3;	/* firmware update Fail */
		klogi("[MXT]Reprogram done : Firmware update Fail~~~~~~~~~~\n");
	}
	kernel_restart(NULL);
}
#endif
#ifdef MXT_FACTORY_TEST
static void set_mxt_update_exe(struct work_struct *work)
{
	struct	mxt_data *mxt;
	int ret, cnt;;
	mxt = container_of(work, struct mxt_data, firmup_dwork.work);
	/*client = mxt->client;*/
	klogi("set_mxt_update_exe \n");


	/*wake_lock(&mxt->wakelock);*/  /* prevents the system from entering suspend during updating */
	disable_irq(mxt->client->irq);
	ret = mxt_load_firmware(&mxt->client->dev, MXT224E_FIRMWARE);
	enable_irq(mxt->client->irq);
	/*wake_unlock(&mxt->wakelock);*/

	if (ret >= 0) {
		for (cnt = 10; cnt > 0; cnt--) {
			if (mxt_firm_normal_status_ack == 1) {
				mxt_firm_status_data = 2;	/* firmware update success */
				klogi("[MXT]Reprogram done : Firmware update Success~~~~~~~~~~\n");
				break;
			} else {
				klogi("[MXT]Reprogram done , but not yet normal status : 3s delay needed \n");
				msleep(3000);/* 3s delay */
			}

		}
		if (cnt == 0) {
			mxt_firm_status_data = 3;	/* firmware update Fail */
			klogi("[MXT]Reprogram done : Firmware update Fail ~~~~~~~~~~\n");
		}
	} else {
		mxt_firm_status_data = 3;	/* firmware update Fail */
		klogi("[MXT]Reprogram done : Firmware update Fail~~~~~~~~~~\n");
	}
	mxt_firm_normal_status_ack = 0;
}
static ssize_t set_mxt_update_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	int count;
	struct mxt_data *mxt = dev_get_drvdata(dev);

	klogi("touch firmware update \n");
	mxt_firm_status_data = 1;	/* start firmware updating */
	cancel_delayed_work(&mxt->firmup_dwork);
	schedule_delayed_work(&mxt->firmup_dwork, 0);

	if (mxt_firm_status_data == 3) {
		count = sprintf(buf, "FAIL\n");
	} else
		count = sprintf(buf, "OK\n");
	return count;

}
/*Current(Part) Version*/
static ssize_t set_mxt_firm_version_read_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	struct mxt_data *mxt = dev_get_drvdata(dev);
	int error, cnt;
	u8 val[7];
	u8 fw_current_version;

	for (cnt = 10; cnt > 0; cnt--) {
		error = mxt_read_block(mxt->client, MXT_ADDR_INFO_BLOCK, 7, (u8 *)val);
		if (error < 0) {
			klogi("Atmel touch version read fail it will try 2s later");
			msleep(2000);
		} else {
			break;
		}
	}
	if (cnt == 0) {
		kloge("set_mxt_firm_version_show failed!!!");
		fw_current_version = 0;
	}

	mxt->device_info.major = ((val[2] >> 4) & 0x0F);
	mxt->device_info.minor = (val[2] & 0x0F);
	mxt->device_info.build	= val[3];
	fw_current_version = val[2];
	klogi("Atmel %s Firmware version [%d.%d](%d) Build %d\n",
		mxt224_variant,
		mxt->device_info.major,
		mxt->device_info.minor,
		fw_current_version,
		mxt->device_info.build);

	return sprintf(buf, "%02d\n", fw_current_version);

}

/* Last(Phone) Version */
extern u8 firmware_latest[];
static ssize_t set_mxt_firm_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 fw_latest_version;
	fw_latest_version = firmware_latest[0];
	klogi("Atmel Last firmware version is %d\n", fw_latest_version);
	return sprintf(buf, "%02d\n", fw_latest_version);

}
static ssize_t set_mxt_firm_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	int count;

	klogi("Enter firmware_status_show by Factory command \n");

	if (mxt_firm_status_data == 1) {
		count = sprintf(buf, "Downloading\n");
	} else if (mxt_firm_status_data == 2) {
		count = sprintf(buf, "PASS\n");
	} else if (mxt_firm_status_data == 3) {
		count = sprintf(buf, "FAIL\n");
	} else
		count = sprintf(buf, "PASS\n");

	return count;

}

extern uint8_t mxt_touch_threshold;
extern int mxt_multitouch_config(struct mxt_data *mxt);
static ssize_t key_threshold_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", mxt_touch_threshold);
}

static ssize_t key_threshold_store(struct device *dev, struct device_attribute *attr,
								   const char *buf, size_t size)
{
	struct mxt_data *mxt = dev_get_drvdata(dev);
	int i;
	if (sscanf(buf, "%d", &i) == 1) {
		wake_lock(&mxt->wakelock);  /* prevents the system from entering suspend during updating */
		disable_irq(mxt->client->irq);   /* disable interrupt */
		mxt_touch_threshold = i;	/*basically,48; */
		mxt_multitouch_config(mxt);
		/* backup to nv memory */
		backup_to_nv(mxt);
		/* forces a reset of the chipset */
		mxt_reset_chip(mxt, RESET_TO_NORMAL);
		msleep(250);  /* 250ms */

		enable_irq(mxt->client->irq);    /* enable interrupt */
		wake_unlock(&mxt->wakelock);
		printk(KERN_INFO "[TSP] threshold is changed to %d\n", i);
	} else
		printk(KERN_INFO "[TSP] threshold write error\n");

	return size;
}
#endif

#if ENABLE_NOISE_TEST_MODE
uint8_t read_uint16_t(u16 Address, u16 *Data, struct mxt_data *mxt)
{
	uint8_t status;
	uint8_t temp[2];

	status = mxt_read_block(mxt->client, Address, 2, temp);
	*Data = ((uint16_t)temp[1] << 8) + (uint16_t)temp[0];

	return status;
}
int  read_dbg_data(u8 dbg_mode , u8 node, u16 *dbg_data, struct mxt_data *mxt)
{
	int  status;
	u8 mode, page, i;
	u8 read_page;
	u16 read_point;
	u16	diagnostics;
	u16 diagnostic_addr;

	diagnostic_addr = MXT_BASE_ADDR(MXT_DEBUG_DIAGNOSTICS_T37);
	diagnostics =  T6_REG(MXT_ADR_T6_DIAGNOSTICS);

	read_page = node / 64;
	node %= 64;
	read_point = (node * 2) + 2;

	/* Page Num Clear */
	mxt_write_byte(mxt->client, diagnostics, MXT_CMD_T6_CTE_MODE);
	msleep(20);
	mxt_write_byte(mxt->client, diagnostics, dbg_mode);
	msleep(20);

	for (i = 0; i < 5; i++) {
		msleep(20);
		status = mxt_read_byte(mxt->client, diagnostic_addr, &mode);
		if (status == 0) {
			if (mode == dbg_mode) {
				break;
			}
		} else {
			printk(KERN_INFO "[TSP] read mode fail \n");
			return status;
		}
	}



	for (page = 0; page < read_page; page++) {
		mxt_write_byte(mxt->client, diagnostics, MXT_CMD_T6_PAGE_UP);
		msleep(10);
	}

	status = read_uint16_t(diagnostic_addr + read_point, dbg_data, mxt);

	msleep(10);

	return status;
}
static ssize_t set_refer0_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int  status;
	u16 qt_refrence = 0;
	struct mxt_data *mxt = dev_get_drvdata(dev);

	status = read_dbg_data(MXT_CMD_T6_REFERENCES_MODE, test_node[0], &qt_refrence, mxt);
	return sprintf(buf, "%u\n", qt_refrence);
}

static ssize_t set_refer1_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int  status;
	u16 qt_refrence = 0;
	struct mxt_data *mxt = dev_get_drvdata(dev);

	status = read_dbg_data(MXT_CMD_T6_REFERENCES_MODE, test_node[1], &qt_refrence, mxt);
	return sprintf(buf, "%u\n", qt_refrence);
}

static ssize_t set_refer2_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int  status;
	u16 qt_refrence = 0;
	struct mxt_data *mxt = dev_get_drvdata(dev);

	status = read_dbg_data(MXT_CMD_T6_REFERENCES_MODE, test_node[2], &qt_refrence, mxt);
	return sprintf(buf, "%u\n", qt_refrence);
}


static ssize_t set_refer3_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int  status;
	u16 qt_refrence = 0;
	struct mxt_data *mxt = dev_get_drvdata(dev);

	status = read_dbg_data(MXT_CMD_T6_REFERENCES_MODE, test_node[3], &qt_refrence, mxt);
	return sprintf(buf, "%u\n", qt_refrence);
}


static ssize_t set_refer4_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int  status;
	u16 qt_refrence = 0;
	struct mxt_data *mxt = dev_get_drvdata(dev);

	status = read_dbg_data(MXT_CMD_T6_REFERENCES_MODE, test_node[4], &qt_refrence, mxt);
	return sprintf(buf, "%u\n", qt_refrence);
}

static ssize_t set_delta0_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int  status;
	u16 qt_delta = 0;
	struct mxt_data *mxt = dev_get_drvdata(dev);

	status = read_dbg_data(MXT_CMD_T6_DELTAS_MODE, test_node[0], &qt_delta, mxt);
	if (qt_delta < 32767) {
		return sprintf(buf, "%u\n", qt_delta);
	   } else	{
		qt_delta = 65535 - qt_delta;
		return sprintf(buf, "-%u\n", qt_delta);
	}
}

static ssize_t set_delta1_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int  status;
	u16 qt_delta = 0;
	struct mxt_data *mxt = dev_get_drvdata(dev);

	status = read_dbg_data(MXT_CMD_T6_DELTAS_MODE, test_node[1], &qt_delta, mxt);
	if (qt_delta < 32767) {
		return sprintf(buf, "%u\n", qt_delta);
	   } else	{
		qt_delta = 65535 - qt_delta;
		return sprintf(buf, "-%u\n", qt_delta);
	}
}

static ssize_t set_delta2_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int  status;
	u16 qt_delta = 0;
	struct mxt_data *mxt = dev_get_drvdata(dev);

	status = read_dbg_data(MXT_CMD_T6_DELTAS_MODE, test_node[2], &qt_delta, mxt);
	if (qt_delta < 32767) {
		return sprintf(buf, "%u\n", qt_delta);
	   } else {
		qt_delta = 65535 - qt_delta;
		return sprintf(buf, "-%u\n", qt_delta);
	}
}

static ssize_t set_delta3_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int  status;
	u16 qt_delta = 0;
	struct mxt_data *mxt = dev_get_drvdata(dev);

	status = read_dbg_data(MXT_CMD_T6_DELTAS_MODE, test_node[3], &qt_delta, mxt);
	if (qt_delta < 32767) {
		return sprintf(buf, "%u\n", qt_delta);
	   } else {
		qt_delta = 65535 - qt_delta;
		return sprintf(buf, "-%u\n", qt_delta);
	}
}

static ssize_t set_delta4_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int  status;
	u16 qt_delta = 0;
	struct mxt_data *mxt = dev_get_drvdata(dev);

	status = read_dbg_data(MXT_CMD_T6_DELTAS_MODE, test_node[4], &qt_delta, mxt);
	if (qt_delta < 32767) {
		return sprintf(buf, "%u\n", qt_delta);
	   } else {
		qt_delta = 65535 - qt_delta;
		return sprintf(buf, "-%u\n", qt_delta);
	}
}

static ssize_t set_threshold_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", mxt_touch_threshold);
}

#endif


static int chk_obj(u8 type)
{
	switch (type) {
		/*	case	MXT_GEN_MESSAGEPROCESSOR_T5:*/
		/*	case	MXT_GEN_COMMANDPROCESSOR_T6:*/
	case	MXT_GEN_POWERCONFIG_T7:
	case	MXT_GEN_ACQUIRECONFIG_T8:
	case	MXT_TOUCH_MULTITOUCHSCREEN_T9:
	case	MXT_TOUCH_KEYARRAY_T15:
		/*	case	MXT_SPT_COMMSCONFIG_T18:*/
		/*	case	MXT_PROCG_NOISESUPPRESSION_T22:*/
		/*	case	MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24:*/
		/*	case	MXT_SPT_SELFTEST_T25:*/
		/*	case	MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27:*/
		/*	case	MXT_SPT_CTECONFIG_T28:*/
		/*	case	MXT_DEBUG_DIAGNOSTICS_T37:*/
	case	MXT_USER_INFO_T38:
		/*	case	MXT_GEN_EXTENSION_T39:*/
	case	MXT_PROCI_GRIPSUPPRESSION_T40:
	case	MXT_PROCI_TOUCHSUPPRESSION_T42:
	case	MXT_SPT_CTECONFIG_T46:
	case	MXT_PROCI_STYLUS_T47:
	case	MXT_PROCG_NOISESUPPRESSION_T48:
		/*	case	MXT_SPT_DIGITIZER_T43: */
		/*	case	MXT_MESSAGECOUNT_T44:*/
		return 0;
	default:
		return -1;
	}
}

static ssize_t show_object(struct device *dev, struct device_attribute *attr, char *buf)
{
	/*	struct qt602240_data *data = dev_get_drvdata(dev); */
	/*	struct qt602240_object *object; */
	struct mxt_data *mxt;
	struct mxt_object	 *object_table;

	int count = 0;
	int i, j;
	u8 val;

	mxt = dev_get_drvdata(dev);
	object_table = mxt->object_table;

	for (i = 0; i < mxt->device_info.num_objs; i++) {
		u8 obj_type = object_table[i].type;

		if (chk_obj(obj_type))
			continue;

		count += sprintf(buf + count, "%s: %d bytes\n",
			object_type_name[obj_type], object_table[i].size);

		for (j = 0; j < object_table[i].size; j++) {
			mxt_read_byte(mxt->client, MXT_BASE_ADDR(obj_type)+(u16)j, &val);
			count += sprintf(buf + count,
				"  Byte %2d: 0x%02x (%d)\n", j, val, val);
		}

		count += sprintf(buf + count, "\n");
	}

#ifdef MXT_TUNNING_ENABLE
	backup_to_nv(mxt);
#endif

	return count;
}

static ssize_t store_object(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/*	struct qt602240_data *data = dev_get_drvdata(dev); */
	/*	struct qt602240_object *object; */
	struct mxt_data *mxt;
	/*	struct mxt_object	*object_table;//TO_CHK: not used now */

	unsigned int type, offset, val;
	u16	chip_addr;
	int ret;

	mxt = dev_get_drvdata(dev);

	if ((sscanf(buf, "%u %u %u", &type, &offset, &val) != 3) || (type >= MXT_MAX_OBJECT_TYPES)) {
		kloge("Invalid values");
		return -EINVAL;
	}

	klogi("Object type: %u, Offset: %u, Value: %u\n", type, offset, val);

	chip_addr = mxt_get_object_address(type, 0, mxt->object_table,
		mxt->device_info.num_objs);
	if (chip_addr == 0) {
		kloge("Invalid object type(%d)!", type);
		return -EIO;
	}

	ret = mxt_write_byte(mxt->client, chip_addr+(u16)offset, (u8)val);
	kloge("store_object result: (%d)\n", ret);
	if (ret < 0) {
		return ret;
	}

	return count;
}

#if 0  /* FOR_TEST */
static ssize_t test_suspend(struct device *dev, struct device_attribute *attr, char *buf)
{
	char *bufp;
	struct early_suspend  *fake;

	bufp = buf;
	bufp += sprintf(bufp, "Running early_suspend function...\n");

	fake = kzalloc(sizeof(struct early_suspend), GFP_KERNEL);
	mxt_early_suspend(fake);
	kfree(fake);

	return strlen(buf);
}

static ssize_t test_resume(struct device *dev, struct device_attribute *attr, char *buf)
{
	char *bufp;
	struct early_suspend  *fake;

	bufp = buf;
	bufp += sprintf(bufp, "Running late_resume function...\n");

	fake = kzalloc(sizeof(struct early_suspend), GFP_KERNEL);
	mxt_late_resume(fake);
	kfree(fake);

	return strlen(buf);
}
#endif



/* Register sysfs files */

static DEVICE_ATTR(deltas,      S_IRUGO, show_deltas,      NULL);
static DEVICE_ATTR(references,  S_IRUGO, show_references,  NULL);
static DEVICE_ATTR(device_info, S_IRUGO, show_device_info, NULL);
static DEVICE_ATTR(object_info, S_IRUGO, show_object_info, NULL);
static DEVICE_ATTR(messages,    S_IRUGO, show_messages,    NULL);
static DEVICE_ATTR(report_id,   S_IRUGO, show_report_id,   NULL);
static DEVICE_ATTR(stat,        S_IRUGO, show_stat,        NULL);
static DEVICE_ATTR(config,      S_IWUSR|S_IRUGO, get_config, set_config);
static DEVICE_ATTR(ap,          S_IWUSR, NULL,             set_ap);
static DEVICE_ATTR(debug, S_IWUSR, NULL, set_debug);
static DEVICE_ATTR(firmware, S_IWUSR|S_IRUGO, show_firmware, store_firmware);
static DEVICE_ATTR(object, S_IWUSR|S_IRUGO, show_object, store_object);
/* static DEVICE_ATTR(suspend, S_IRUGO, test_suspend, NULL); */
/* static DEVICE_ATTR(resume, S_IRUGO, test_resume, NULL);  */
#ifdef MXT_FACTORY_TEST
static DEVICE_ATTR(set_mxt_update, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_mxt_update_show, NULL);
static DEVICE_ATTR(set_mxt_firm_status, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_mxt_firm_status_show, NULL);
static DEVICE_ATTR(key_threshold, S_IRUGO | S_IWUSR, key_threshold_show, key_threshold_store);
static DEVICE_ATTR(set_mxt_firm_version, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_mxt_firm_version_show, NULL);/* PHONE*/
static DEVICE_ATTR(set_mxt_firm_version_read, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_mxt_firm_version_read_show, NULL);/*PART*/
#endif
#if ENABLE_NOISE_TEST_MODE
static DEVICE_ATTR(set_refer0, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_refer0_mode_show, NULL);
static DEVICE_ATTR(set_delta0, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_delta0_mode_show, NULL);
static DEVICE_ATTR(set_refer1, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_refer1_mode_show, NULL);
static DEVICE_ATTR(set_delta1, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_delta1_mode_show, NULL);
static DEVICE_ATTR(set_refer2, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_refer2_mode_show, NULL);
static DEVICE_ATTR(set_delta2, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_delta2_mode_show, NULL);
static DEVICE_ATTR(set_refer3, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_refer3_mode_show, NULL);
static DEVICE_ATTR(set_delta3, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_delta3_mode_show, NULL);
static DEVICE_ATTR(set_refer4, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_refer4_mode_show, NULL);
static DEVICE_ATTR(set_delta4, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_delta4_mode_show, NULL);
static DEVICE_ATTR(set_threshould, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_threshold_mode_show, NULL);
#endif


static struct attribute *maxTouch_attributes[] = {
	&dev_attr_deltas.attr,
		&dev_attr_references.attr,
		&dev_attr_device_info.attr,
		&dev_attr_object_info.attr,
		&dev_attr_messages.attr,
		&dev_attr_report_id.attr,
		&dev_attr_stat.attr,
		&dev_attr_config.attr,
		&dev_attr_ap.attr,
		&dev_attr_debug.attr,
		&dev_attr_firmware.attr,
		&dev_attr_object.attr,
		/*	&dev_attr_suspend.attr, */
		/*	&dev_attr_resume.attr, */
#ifdef MXT_FACTORY_TEST
		&dev_attr_set_mxt_update.attr,
		&dev_attr_set_mxt_firm_status.attr,
		&dev_attr_key_threshold.attr,
		&dev_attr_set_mxt_firm_version.attr,
		&dev_attr_set_mxt_firm_version_read.attr,
#endif
#if ENABLE_NOISE_TEST_MODE
		&dev_attr_set_refer0.attr,
		&dev_attr_set_delta0.attr,
		&dev_attr_set_refer1.attr,
		&dev_attr_set_delta1.attr,
		&dev_attr_set_refer2.attr,
		&dev_attr_set_delta2.attr,
		&dev_attr_set_refer3.attr,
		&dev_attr_set_delta3.attr,
		&dev_attr_set_refer4.attr,
		&dev_attr_set_delta4.attr,
		&dev_attr_set_threshould.attr,
#endif

		NULL,
};

static struct attribute_group maxtouch_attr_group = {
	.attrs = maxTouch_attributes,
};

int backup_to_nv(struct mxt_data *mxt)
{
	/* backs up settings to the non-volatile memory */
	return mxt_write_byte(mxt->client,
		MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6) +
		MXT_ADR_T6_BACKUPNV,
		0x55);
}

int mxt_reset_chip(struct mxt_data *mxt, u8 mode)
{
	u8 data;
	kloge("Reset chip Reset mode (%d)", mode);
	if (mode == RESET_TO_NORMAL)
		data = 0x1;  /* non-zero value */
	else if (mode == RESET_TO_BOOTLOADER)
		data = 0xA5;
	else {
		kloge("Invalid reset mode(%d)", mode);
		return -1;
	}

	/* Any non-zero value written to reset reg will reset the chip */
#ifdef MXT_FIRMUP_ENABLE
	if (mxt_firm_status_data == 1) {	/*start firmware updating : not yet finished*/
		/*There are no MXT_BASE_ADDR , before get object table*/
		/*MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6) +MXT_ADR_T6_RESET is 0x102*/
		return mxt_write_byte(mxt->client,
			0x102,
			data);
	} else {
		return mxt_write_byte(mxt->client,
			MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6) +
			MXT_ADR_T6_RESET,
			data);
	}
#else
	return mxt_write_byte(mxt->client,
		MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6) +
		MXT_ADR_T6_RESET,
		data);
#endif

}

#ifdef MXT_ESD_WORKAROUND
void mxt_force_reset(struct mxt_data *mxt)
{
	int cnt;

	klogi("has been called!");

#ifndef MXT_THREADED_IRQ
	wake_lock(&mxt->wakelock);  /* prevents the system from entering suspend during updating */
	disable_irq(mxt->client->irq);  /* disable interrupt */
#endif
	if (mxt->suspend_hw != NULL)
		mxt->suspend_hw();
	msleep(400);
	if (mxt->resume_hw != NULL)
		mxt->resume_hw();

	for (cnt = 10; cnt > 0; cnt--) {
		if (mxt_reset_chip(mxt, RESET_TO_NORMAL) == 0)  /* soft reset */
			break;
	}
	if (cnt == 0) {
		kloge("mxt_force_reset failed!!!");
		return;
	}
	msleep(250);  /* 200ms */
#ifndef MXT_THREADED_IRQ
	enable_irq(mxt->client->irq);;  /* enable interrupt */
	wake_unlock(&mxt->wakelock);
#endif
}


#endif

/******************************************************************************/
/* Initialization of driver                                                   */
/******************************************************************************/

static int __devinit mxt_identify(struct i2c_client *client,
								  struct mxt_data *mxt)
{
	u8 buf[7];
	int error;
	int identified;

	identified = 0;

retry_i2c:
	/* Read Device info to check if chip is valid */
	error = mxt_read_block(client, MXT_ADDR_INFO_BLOCK, 7, (u8 *)buf);

	if (error < 0) {
		mxt->read_fail_counter++;
		if (mxt->read_fail_counter == 1) {
			klogi("Warning: To wake up touch-ic in deep sleep, retry i2c communication!");
			msleep(30);  /* delay 25ms */
			goto retry_i2c;
		}
		dev_err(&client->dev, "Failure accessing maXTouch device\n");
		return -EIO;
	}

	mxt->device_info.family_id  = buf[0];
	mxt->device_info.variant_id = buf[1];
	mxt->device_info.major	    = ((buf[2] >> 4) & 0x0F);
	mxt->device_info.minor      = (buf[2] & 0x0F);
	mxt->device_info.build	    = buf[3];
	mxt->device_info.x_size	    = buf[4];
	mxt->device_info.y_size	    = buf[5];
	mxt->device_info.num_objs   = buf[6];
	mxt->device_info.num_nodes  = mxt->device_info.x_size *
		mxt->device_info.y_size;

	/* Check Family Info */
	if (mxt->device_info.family_id == MAXTOUCH_FAMILYID) {
		strcpy(mxt->device_info.family, maxtouch_family);
	} else {
		dev_err(&client->dev,
			"maXTouch Family ID [0x%x] not supported\n",
			mxt->device_info.family_id);
		identified = -ENXIO;
	}

	/* Check Variant Info */
	if ((mxt->device_info.variant_id == MXT224_CAL_VARIANTID) ||
		(mxt->device_info.variant_id == MXT224_UNCAL_VARIANTID)) {
		strcpy(mxt->device_info.variant, mxt224_variant);
	} else {
		dev_err(&client->dev,
			"maXTouch Variant ID [0x%x] not supported\n",
			mxt->device_info.variant_id);
		identified = -ENXIO;
	}

	dev_info(
		&client->dev,
		"Atmel %s.%s Firmware version [%d.%d] Build %d\n",
		mxt->device_info.family,
		mxt->device_info.variant,
		mxt->device_info.major,
		mxt->device_info.minor,
		mxt->device_info.build
		);
	dev_info(
		&client->dev,
		"Atmel %s.%s Configuration [X: %d] x [Y: %d]\n",
		mxt->device_info.family,
		mxt->device_info.variant,
		mxt->device_info.x_size,
		mxt->device_info.y_size
		);
	dev_info(
		&client->dev,
		"number of objects: %d\n",
		mxt->device_info.num_objs
		);

	return identified;
}

/*
* Reads the object table from maXTouch chip to get object data like
* address, size, report id.
*/
static int __devinit mxt_read_object_table(struct i2c_client *client,
										   struct mxt_data *mxt)
{
	u16	report_id_count;
	u8	buf[MXT_OBJECT_TABLE_ELEMENT_SIZE];
	u8	object_type;
	u16	object_address;
	u16	object_size;
	u8	object_instances;
	u8	object_report_ids;
	u16	object_info_address;
	u32	crc;
	u32     crc_calculated;
	int	i;
	int	error;

	u8	object_instance;
	u8	object_report_id;
	u8	report_id;
	int     first_report_id;

	struct mxt_object *object_table;

	if (debug >= DEBUG_TRACE)
		printk(KERN_INFO "maXTouch driver get configuration\n");

	object_table = kzalloc(sizeof(struct mxt_object) *
		mxt->device_info.num_objs,
		GFP_KERNEL);
	if (object_table == NULL) {
		printk(KERN_WARNING "maXTouch: Memory allocation failed!\n");
		return -ENOMEM;
	}

	mxt->object_table = object_table;

	if (debug >= DEBUG_TRACE)
		printk(KERN_INFO "maXTouch driver Memory allocated\n");

	object_info_address = MXT_ADDR_OBJECT_TABLE;

	report_id_count = 0;
	for (i = 0; i < mxt->device_info.num_objs; i++) {
		if (debug >= DEBUG_TRACE)
			printk(KERN_INFO "Reading maXTouch at [0x%04x]: ",
			object_info_address);

		error = mxt_read_block(client, object_info_address, MXT_OBJECT_TABLE_ELEMENT_SIZE, (u8 *)buf);

		if (error < 0) {
			mxt->read_fail_counter++;
			dev_err(&client->dev,
				"maXTouch Object %d could not be read\n", i);
			return -EIO;
		}
		object_type		=  buf[0];
		object_address		= (buf[2] << 8) + buf[1];
		object_size		=  buf[3] + 1;
		object_instances	=  buf[4] + 1;
		object_report_ids	=  buf[5];
		if (debug >= DEBUG_TRACE)
			printk(KERN_INFO "Type=%03d, Address=0x%04x, "
			"Size=0x%02x, %d instances, %d report id's\n",
			object_type,
			object_address,
			object_size,
			object_instances,
			object_report_ids
			);

		if (object_type > MXT_MAX_OBJECT_TYPES) {
			/* Unknown object type */
			dev_err(&client->dev,
				"maXTouch object type [%d] not recognized\n",
				object_type);
			return -ENXIO;

		}

		/* Save frequently needed info. */
		if (object_type == MXT_GEN_MESSAGEPROCESSOR_T5) {
			mxt->msg_proc_addr = object_address;
			mxt->message_size = object_size;
#if defined(CONFIG_MACH_N1) || defined(CONFIG_MACH_JANICE)
			printk(KERN_INFO "Saving info for later use: msg_proc_addr=0x%02x, message_size=%d\n",
				object_address, object_size);	/* TO_DEL */
#endif
		}

		object_table[i].type            = object_type;
		object_table[i].chip_addr       = object_address;
		object_table[i].size            = object_size;
		object_table[i].instances       = object_instances;
		object_table[i].num_report_ids  = object_report_ids;
		report_id_count += object_instances * object_report_ids;

		object_info_address += MXT_OBJECT_TABLE_ELEMENT_SIZE;
	}

	mxt->rid_map =
		kzalloc(sizeof(struct report_id_map) * (report_id_count + 1),
		/* allocate for report_id 0, even if not used */
		GFP_KERNEL);
	if (mxt->rid_map == NULL) {
		printk(KERN_WARNING "maXTouch: Can't allocate memory!\n");
		return -ENOMEM;
	}

	mxt->last_message = kzalloc(mxt->message_size, GFP_KERNEL);
	if (mxt->last_message == NULL) {
		printk(KERN_WARNING "maXTouch: Can't allocate memory!\n");
		return -ENOMEM;
	}


	mxt->report_id_count = report_id_count;
	if (report_id_count > 254) {	/* 0 & 255 are reserved */
		dev_err(&client->dev,
			"Too many maXTouch report id's [%d]\n",
			report_id_count);
		return -ENXIO;
	}

	/* Create a mapping from report id to object type */
	report_id = 1; /* Start from 1, 0 is reserved. */

	/* Create table associating report id's with objects & instances */
	for (i = 0; i < mxt->device_info.num_objs; i++) {
		for (object_instance = 0;
		object_instance < object_table[i].instances;
		object_instance++) {
			first_report_id = report_id;
			for (object_report_id = 0;
			object_report_id < object_table[i].num_report_ids;
			object_report_id++) {
				mxt->rid_map[report_id].object =
					object_table[i].type;
				mxt->rid_map[report_id].instance =
					object_instance;
				mxt->rid_map[report_id].first_rid =
					first_report_id;
				report_id++;
			}
		}
	}

	/* Read 3 byte CRC */
	error = mxt_read_block(client, object_info_address, 3, buf);
	if (error < 0) {
		mxt->read_fail_counter++;
		dev_err(&client->dev, "Error reading CRC\n");
	}

	crc = (buf[2] << 16) | (buf[1] << 8) | buf[0];

	calculate_infoblock_crc(&crc_calculated, mxt);

	if (debug >= DEBUG_TRACE) {
		printk(KERN_INFO "Reported info block CRC = 0x%6X\n\n", crc);
		printk(KERN_INFO "Calculated info block CRC = 0x%6X\n\n",
			crc_calculated);
	}

	if (crc == crc_calculated) {
		mxt->info_block_crc = crc;
	} else {
		mxt->info_block_crc = 0;
		printk(KERN_WARNING "maXTouch: info block CRC invalid!\n");
	}


	mxt->delta	= NULL;
	mxt->reference	= NULL;
	mxt->cte	= NULL;

	if (debug >= DEBUG_VERBOSE) {

		dev_info(&client->dev, "maXTouch: %d Objects\n",
			mxt->device_info.num_objs);

		for (i = 0; i < mxt->device_info.num_objs; i++) {
			dev_info(&client->dev, "Type:\t\t\t[%d]: %s\n",
				object_table[i].type,
				object_type_name[object_table[i].type]);
			dev_info(&client->dev, "\tAddress:\t0x%04X\n",
				object_table[i].chip_addr);
			dev_info(&client->dev, "\tSize:\t\t%d Bytes\n",
				object_table[i].size);
			dev_info(&client->dev, "\tInstances:\t%d\n",
				object_table[i].instances);
			dev_info(&client->dev, "\tReport Id's:\t%d\n",
				object_table[i].num_report_ids);
		}
	}
	return 0;
}

u8 mxt_valid_interrupt(void)
{
	/* TO_CHK: how to implement this function? */
	return 1;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mxt_early_suspend(struct early_suspend *h)
{
	int i = 0;
#ifndef MXT_SLEEP_POWEROFF
	u8 cmd_sleep[2] = {0};
	u16 addr;
#endif

	klogi("has been called!");
#if defined(MXT_FACTORY_TEST) || defined(MXT_FIRMUP_ENABLE)
	while (mxt_firm_status_data == 1) {	/*start firmware updating : not yet finished*/
		klogi("mxt firmware is Downloading : mxt suspend must be delayed!");
		msleep(1000);
	}
#endif
	disable_irq(s_mxt->client->irq);

#ifdef MXT_SLEEP_POWEROFF
	if (s_mxt->suspend_hw != NULL)
		s_mxt->suspend_hw();
#else
		/*
		* a setting of zeros to IDLEACQINT and ACTVACQINT
		* forces the chip set to enter Deep Sleep mode.
	*/
	addr = mxt_get_object_address(MXT_GEN_POWERCONFIG_T7, 0, s_mxt->object_table, s_mxt->device_info.num_objs);
	klogi("addr: 0x%02x, buf[0]=0x%x, buf[1]=0x%x", addr, cmd_sleep[0], cmd_sleep[1]);
	mxt_write_block(s_mxt->client, addr, 2, (u8 *)cmd_sleep);
#endif
	for (i = 0; i < MXT_MAX_NUM_TOUCHES ; i++)  /*  _SUPPORT_MULTITOUCH_ */
		mtouch_info[i].pressure = -1;

}

static void mxt_late_resume(struct early_suspend *h)
{
	int cnt;

	klogi("has been called!");
#ifdef MXT_SLEEP_POWEROFF
	if (s_mxt->resume_hw != NULL)
		s_mxt->resume_hw();
#endif
	for (cnt = 10; cnt > 0; cnt--) {
#ifndef MXT_SLEEP_POWEROFF
		if (mxt_power_config(s_mxt) < 0)
			continue;
#endif
		if (mxt_reset_chip(s_mxt, RESET_TO_NORMAL) == 0)  /* soft reset */
			break;
	}
	if (cnt == 0) {
		kloge("mxt_late_resume failed!!!");
		return;
	}
	msleep(250);  /*typical value is 250ms*/
	/* calibrate_chip(); */
	enable_irq(s_mxt->client->irq);
}
#endif


static int __devinit mxt_probe(struct i2c_client *client,
							   const struct i2c_device_id *id)
{
	struct mxt_data          *mxt;
	struct mxt_platform_data *pdata;
	struct input_dev         *input;
	int error;
#if defined(CONFIG_MACH_N1) || defined(CONFIG_MACH_JANICE)
	int i;
	u8 unverified = 0;
#endif
#ifdef MXT_FIRMUP_ENABLE

	/* mXT224E Latest Firmware version [0.4] Build 0x02*/
	u8 last_major = 0x00;
	u8 last_minor = 0x04;
	u8 last_build = 0x02;
#endif


	printk(KERN_INFO "[TSP] mXT224E: mxt_probe\n");


	if (client == NULL)
		printk(KERN_INFO "[TSP] maXTouch: client == NULL\n");
	else if (client->adapter == NULL)
		printk(KERN_INFO "[TSP] maXTouch: client->adapter == NULL\n");
	else if (&client->dev == NULL)
		printk(KERN_INFO "[TSP] maXTouch: client->dev == NULL\n");
	else if (&client->adapter->dev == NULL)
		printk(KERN_INFO "[TSP] maXTouch: client->adapter->dev == NULL\n");
	else if (id == NULL)
		printk(KERN_INFO "[TSP] maXTouch: id == NULL\n");
	else
		goto param_check_ok;
	return	-EINVAL;

param_check_ok:
	if (debug >= DEBUG_INFO) {
		printk(KERN_INFO "[TSP] maXTouch driver\n");
		printk(KERN_INFO "[TSP] \t \"%s\"\n",		client->name);
		printk(KERN_INFO "[TSP] \taddr:\t0x%04x\n",	client->addr);
		printk(KERN_INFO "[TSP] \tirq:\t%d\n",	client->irq);
		printk(KERN_INFO "[TSP] \tflags:\t0x%04x\n",	client->flags);
		printk(KERN_INFO "[TSP] \tadapter:\"%s\"\n",	client->adapter->name);
		printk(KERN_INFO "[TSP] \tdevice:\t\"%s\"\n",	client->dev.init_name);
	}
	if (debug >= DEBUG_TRACE)
		printk(KERN_INFO "Parameters OK\n");;

#if 0  /* Note: This returns "not supported", but touch i2c is working. Neglect this. */
	/* Check if the I2C bus supports BYTE transfer */
	error = i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_BYTE);
	if (!error) {
		dev_err(&client->dev, "%s adapter not supported\n",
			dev_driver_string(&client->adapter->dev));
		return -ENODEV;
	}
	if (debug >= DEBUG_TRACE)
		printk(KERN_INFO "maXTouch driver functionality OK\n");
#endif

#if defined(CONFIG_MACH_N1) || defined(CONFIG_MACH_JANICE)
	/* Allocate structure - we need it to identify device */
	mxt = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	input = input_allocate_device();
	if (!mxt || !input) {
		dev_err(&client->dev, "[TSP] insufficient memory\n");
		error = -ENOMEM;
		goto err_after_kmalloc;
	}
#endif
	/* Initialize Platform data */
	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&client->dev, "[TSP] platform data is required!\n");
		return -EINVAL;
	}
	if (debug >= DEBUG_TRACE)
		printk(KERN_INFO "[TSP] Platform OK: pdata = 0x%08x\n", (unsigned int) pdata);

	mxt->read_fail_counter = 0;
	mxt->message_counter   = 0;

	/*Width: 1280, Height: 800*/
	mxt->max_x_val         = pdata->max_x;
	mxt->max_y_val         = pdata->max_y;

	mxt->numtouch	       = pdata->numtouch;
	mxt->rotate	       = pdata->rotate;

	/* Get data that is defined in board specific code. */
	mxt->init_hw = pdata->init_platform_hw;
	mxt->exit_hw = pdata->exit_platform_hw;
#if defined(CONFIG_MACH_N1) || defined(CONFIG_MACH_JANICE)
	mxt->suspend_hw = pdata->suspend_platform_hw;
	mxt->resume_hw = pdata->resume_platform_hw;
#endif
#ifndef CONFIG_MACH_N1/*move to board-p3*/
	if (mxt->init_hw != NULL)
		mxt->init_hw();
#endif
	if (debug >= DEBUG_TRACE)
		printk(KERN_INFO "[TSP] maXTouch driver identifying chip\n");


	mxt->client = client;
	mxt->input  = input;
	input_set_drvdata(input, mxt);
	input->name = "sec_touchscreen";

#if 0
	mxt->reg_avdd = regulator_get(NULL, "TSP_AVDD");
	if (IS_ERR(mxt->reg_avdd)) {
		error = PTR_ERR(mxt->reg_vdd);
		printk(KERN_ERR, "unable to get regulator TSP_AVDD: %d\n", error);
		goto err_get_regulator;
	}

    mxt->reg_vdd = regulator_get(NULL, "TSP_VDD");
	if (IS_ERR(mxt->reg_vdd)) {
		error = PTR_ERR(mxt->reg_vdd);
		printk(KERN_ERR, "unable to get regulator TSP_VDD: %d\n", error);
		 goto err_get_regulator;
	}

    /* TSP Power on */
    error = regulator_enable(mxt->reg_vdd);
	regulator_set_voltage(mxt->reg_vdd,
		1800000, 1800000);
    printk("enable TSP_VDD: ret=%d\n", error);
	error = regulator_enable(mxt->reg_avdd);
	regulator_set_voltage(mxt->reg_avdd,
		3300000, 3300000);

    printk("enable TSP_AVDD: ret=%d\n", error);
	msleep(700);
#endif

	error = mxt_identify(client, mxt);
	if (error < 0) {
		dev_err(&client->dev, "[TSP] ATMEL Chip could not be identified. error = %d\n", error);
		goto err_get_regulator;
#if	0/* ifdef CONFIG_MACH_N1 */
#ifdef MXT_FIRMUP_ENABLE
		unverified = 1;
#else
		goto back_door_for_fwupdate;
#endif
#else
		return -ENODEV;
#endif
	}
	/* Chip is valid and active. */
	if (debug >= DEBUG_TRACE)
		printk(KERN_INFO "[TSP] maXTouch driver allocating input device\n");

#ifndef MXT_THREADED_IRQ
	INIT_DELAYED_WORK(&mxt->dwork, mxt_worker);
#endif
#ifdef MXT_FACTORY_TEST
	INIT_DELAYED_WORK(&mxt->firmup_dwork, set_mxt_update_exe);
#endif

	if (debug >= DEBUG_TRACE)
		printk(KERN_INFO "[TSP] maXTouch driver init spinlock\n");

	init_waitqueue_head(&mxt->msg_queue);
	init_MUTEX(&mxt->msg_sem);

	spin_lock_init(&mxt->lock);


	if (debug >= DEBUG_TRACE)
		printk(KERN_INFO "[TSP] maXTouch driver creating device name\n");

	snprintf(
		mxt->phys_name,
		sizeof(mxt->phys_name),
		"%s/input0",
		dev_name(&client->dev)
		);
	//input_dev->name = "Atmel maXTouch Touchscreen controller";
	input->phys = mxt->phys_name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	if (debug >= DEBUG_INFO) {
		printk(KERN_INFO "[TSP] maXTouch name: \"%s\"\n", input->name);
		printk(KERN_INFO "[TSP] maXTouch phys: \"%s\"\n", input->phys);
		printk(KERN_INFO "[TSP] maXTouch driver setting abs parameters\n");
	}
#if defined(CONFIG_MACH_N1) || defined(CONFIG_MACH_JANICE)
	__set_bit(BTN_TOUCH, input->keybit);  /* TO_CHK: added here? */
#endif
	__set_bit(EV_ABS, input->evbit);
	/* single touch */
	input_set_abs_params(input, ABS_X, 0, mxt->max_x_val, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, mxt->max_y_val, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, MXT_MAX_REPORTED_PRESSURE, 0, 0);
	input_set_abs_params(input, ABS_TOOL_WIDTH, 0, MXT_MAX_REPORTED_WIDTH, 0, 0);

	/* multi touch */
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, mxt->max_x_val, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, mxt->max_y_val, 0, 0);
#if defined(CONFIG_MACH_N1) || defined(CONFIG_MACH_JANICE)
	/* TO_CHK */
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, MXT_MAX_NUM_TOUCHES-1, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, 30, 0, 0);
#else
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, MXT_MAX_TOUCH_SIZE, 0, 0);
	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, MXT_MAX_NUM_TOUCHES, 0, 0);
#endif
	__set_bit(EV_SYN, input->evbit);
	__set_bit(EV_KEY, input->evbit);

	for (i = 0; i < NUMOFKEYS; i++) {
		__set_bit(tsp_keycodes[i], input->keybit);
	}
	if (debug >= DEBUG_TRACE)
		printk(KERN_INFO "[TSP] maXTouch driver setting client data\n");

	i2c_set_clientdata(client, mxt);
	if (debug >= DEBUG_TRACE)
		printk(KERN_INFO "[TSP] maXTouch driver setting drv data\n");

	input_set_drvdata(input, mxt);

	if (debug >= DEBUG_TRACE)
		printk(KERN_INFO "[TSP] maXTouch driver input register device\n");

	error = input_register_device(mxt->input);
	if (error < 0) {
		dev_err(&client->dev,
			"[TSP] Failed to register input device\n");
		goto err_after_kmalloc;
	}
	if (debug >= DEBUG_TRACE)
		printk(KERN_INFO "[TSP] maXTouch driver allocate interrupt\n");

#ifdef MXT_FIRMUP_ENABLE /*auto firmware upgrade check */
	if ((mxt->device_info.major != last_major) || (mxt->device_info.minor != last_minor) || (mxt->device_info.build != last_build) || unverified) {
		kloge("[TSP] Touch firm up is needed to last version :[%d.%d] , build : [%d] ", last_major, last_minor, last_build);
		mxt_firm_status_data = 1;		/* start firmware updating */
		set_mxt_auto_update_exe(&client->dev);
		/* Need Reset*/
	}
#endif

	/* Allocate the interrupt */
	mxt->irq = client->irq;
	mxt->valid_irq_counter = 0;
	mxt->invalid_irq_counter = 0;
	mxt->irq_counter = 0;
	if (mxt->irq) {
	/* Try to request IRQ with falling edge first. This is
		* not always supported. If it fails, try with any edge. */
#ifdef MXT_THREADED_IRQ
		error = request_threaded_irq(mxt->irq, NULL,  mxt_threaded_irq, IRQF_DISABLED|IRQF_TRIGGER_FALLING, client->dev.driver->name, mxt);
		if (error < 0) {
			error = request_threaded_irq(mxt->irq, NULL, mxt_threaded_irq, IRQF_DISABLED, client->dev.driver->name, mxt);
		}
#else
		error = request_irq(mxt->irq, mxt_irq_handler, IRQF_TRIGGER_FALLING, client->dev.driver->name, mxt);
		if (error < 0) {
			error = request_irq(mxt->irq, mxt_irq_handler, 0, client->dev.driver->name, mxt);
		}
#endif
#if defined(CONFIG_MACH_N1) || defined(CONFIG_MACH_JANICE)
		if (error < 0) {
			dev_err(&client->dev,
				"[TSP] failed to allocate irq %d\n", mxt->irq);
			goto err_after_input_register;
		}
	}
#endif
	if (debug > DEBUG_INFO)
		dev_info(&client->dev, "touchscreen, irq %d\n", mxt->irq);
#if !defined(CONFIG_MACH_N1) && !defined(CONFIG_MACH_JANICE)
	/* move to back_door_for_fwupdate */
	error = sysfs_create_group(&client->dev.kobj, &maxtouch_attr_group);

	if (error)
		goto err_remove_attr_group;
#endif
	error = mxt_read_object_table(client, mxt);
	if (error < 0)
		goto err_after_interrupt_register;

#if defined(CONFIG_MACH_N1) || defined(CONFIG_MACH_JANICE)
#ifndef MXT_TUNNING_ENABLE
	/* pre-set configuration before soft reset */
/*
	u8  udata[8];


	mxt_get_object_values(mxt, MXT_USER_INFO_T38);
	mxt_copy_object(mxt, udata, MXT_USER_INFO_T38);

	printk(KERN_INFO "\t[TSP] udata[0] = :\t\"%d\"\n", udata[0]);
	if (udata[0] != 0xA5) {
		error = mxt_config_settings(mxt);
	}
	if (error < 0)
	goto err_after_interrupt_register;
*/

	/* backup to nv memory */
	backup_to_nv(mxt);
	/* forces a reset of the chipset */
	mxt_reset_chip(mxt, RESET_TO_NORMAL);
	msleep(250); /*mxt1386 need 250ms*/

#endif
	for (i = 0; i < MXT_MAX_NUM_TOUCHES ; i++)  /* _SUPPORT_MULTITOUCH_ */
		mtouch_info[i].pressure = -1;
#endif




#ifndef MXT_THREADED_IRQ
		/* Schedule a worker routine to read any messages that might have
	* been sent before interrupts were enabled. */
	cancel_delayed_work(&mxt->dwork);
	schedule_delayed_work(&mxt->dwork, 0);
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	s_mxt = mxt;
	mxt->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	mxt->early_suspend.suspend = mxt_early_suspend;
	mxt->early_suspend.resume = mxt_late_resume;
	register_early_suspend(&mxt->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

#if defined(CONFIG_MACH_N1) || defined(CONFIG_MACH_JANICE)
back_door_for_fwupdate:
	error = sysfs_create_group(&client->dev.kobj, &maxtouch_attr_group);
	if (error) {
		if (unverified)
			goto err_after_kmalloc;
		else {
			unregister_early_suspend(&mxt->early_suspend);
			goto err_after_interrupt_register;
		}
	}
	wake_lock_init(&mxt->wakelock, WAKE_LOCK_SUSPEND, "touch");
#endif
	printk(KERN_WARNING "[TSP] mxt probe ok\n");
	return 0;

#if defined(CONFIG_MACH_N1) || defined(CONFIG_MACH_JANICE)
	/* sysfs_remove_group(&pdev->dev.kobj, &maxtouch_attr_group); */
err_after_interrupt_register:
	if (mxt->irq)
		free_irq(mxt->irq, mxt);
err_after_input_register:
	input_free_device(input);

err_get_regulator:
	regulator_put(mxt->reg_vdd);
	regulator_put(mxt->reg_avdd);

err_after_kmalloc:
	if (mxt != NULL) {
		kfree(mxt->rid_map);
		kfree(mxt->delta);
		kfree(mxt->reference);
		kfree(mxt->cte);
		kfree(mxt->object_table);
		kfree(mxt->last_message);
		/* if (mxt->exit_hw != NULL) */
		/*	mxt->exit_hw(); */
	}
	kfree(mxt);

	return error;
#else

err_remove_attr_group:
	sysfs_remove_group(&client->dev.kobj, &maxtouch_attr_group);
err_free_irq:
	if (mxt->irq)
		free_irq(mxt->irq, mxt);
err_free_device:
	input_free_device(input);
err_free_mem:
	if (mxt != NULL) {
		kfree(mxt->rid_map);
		kfree(mxt->delta);
		kfree(mxt->reference);
		kfree(mxt->cte);
		kfree(mxt->object_table);
		kfree(mxt->last_message);
		mxt->exit_hw();

	}
	kfree(mxt);
	return error;
#endif
}

static int __devexit mxt_remove(struct i2c_client *client)
{
	struct mxt_data *mxt;

	mxt = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
	wake_lock_destroy(&mxt->wakelock);
	unregister_early_suspend(&mxt->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */
	/* Close down sysfs entries */
	sysfs_remove_group(&client->dev.kobj, &maxtouch_attr_group);

	/* Release IRQ so no queue will be scheduled */
	if (mxt->irq)
		free_irq(mxt->irq, mxt);
#ifndef MXT_THREADED_IRQ
	cancel_delayed_work_sync(&mxt->dwork);
#endif
	input_unregister_device(mxt->input);
	/* Should dealloc deltas, references, CTE structures, if allocated */

	if (mxt != NULL) {
		kfree(mxt->rid_map);
		kfree(mxt->delta);
		kfree(mxt->reference);
		kfree(mxt->cte);
		kfree(mxt->object_table);
		kfree(mxt->last_message);
	}
	kfree(mxt);

	i2c_set_clientdata(client, NULL);
	if (debug >= DEBUG_TRACE)
		dev_info(&client->dev, "Touchscreen unregistered\n");

	return 0;
}

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
static int mxt_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct mxt_data *mxt = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(mxt->irq);

	return 0;
}

static int mxt_resume(struct i2c_client *client)
{
	struct mxt_data *mxt = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(mxt->irq);

	return 0;
}
#else
#define mxt_suspend NULL
#define mxt_resume NULL
#endif

static const struct i2c_device_id mxt_idtable[] = {
	{"mxt_touch", 0,},
	{ }
};

MODULE_DEVICE_TABLE(i2c, mxt_idtable);

static struct i2c_driver mxt_driver = {
	.driver = {
		.name	= "mxt_touch",
		.owner  = THIS_MODULE,
	},

	.id_table	= mxt_idtable,
	.probe		= mxt_probe,
	.remove		= __devexit_p(mxt_remove),
	.suspend	= mxt_suspend,
	.resume		= mxt_resume,

};

static int __init mxt_init(void)
{
	int err;
	err = i2c_add_driver(&mxt_driver);
/*	if (err) {
*		printk(KERN_WARNING "Adding mXT224E driver failed "
*			"(errno = %d)\n", err);
*	} else {
*		printk(KERN_INFO "Successfully added driver %s\n",
*			mxt_driver.driver.name);
*	}
*/
	return err;
}

static void __exit mxt_cleanup(void)
{
	i2c_del_driver(&mxt_driver);
}


module_init(mxt_init);
module_exit(mxt_cleanup);

MODULE_AUTHOR("Samsung");
MODULE_DESCRIPTION("Driver for Atmel mXT224E Touchscreen Controller");

MODULE_LICENSE("GPL");
