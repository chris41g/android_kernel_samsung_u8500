/*----------------------------------------------------------------------------------*/
/*  copyright STMicroelectronics, 2007.                                            */
/*                                                                                  */
/* This program is free software; you can redistribute it and/or modify it under    */
/* the terms of the GNU General Public License as published by the Free             */
/* Software Foundation; either version 2.1 of the License, or (at your option)      */
/* any later version.                                                               */
/*                                                                                  */
/* This program is distributed in the hope that it will be useful, but WITHOUT      */
/* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS    */
/* FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.   */
/*                                                                                  */
/* You should have received a copy of the GNU General Public License                */
/* along with this program. If not, see <http://www.gnu.org/licenses/>.             */
/*----------------------------------------------------------------------------------*/

#ifndef __HSI_TEST_PROT_DRIVER_IF_H__
#define __HSI_TEST_PROT_DRIVER_IF_H__

#define MAX_CHANNELS_MONITORED 4

struct callback_data {
	u8 read_done;
	u8 write_done;
};


int hsi_testprot_drv_open(unsigned char flags);
int hsi_testprot_drv_read(unsigned int ch,void* data,unsigned int datawidth, unsigned int count);
int hsi_testprot_drv_read_cancel(unsigned int ch);
int hsi_testprot_drv_write(unsigned int ch,void* data,unsigned int datawidth, unsigned int count);
int hsi_testprot_drv_write_cancel(unsigned int ch);
int hsi_testprot_drv_close(unsigned int ch);
int hsi_testprot_drv_ioctl(unsigned int ch,unsigned int command,void *arg);


#endif
