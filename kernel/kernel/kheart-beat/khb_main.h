/*****************************************************************************
** kernel/kheart-beat/khb_main.h
******************************************************************************/

#ifndef _KHB_MAIN_H_
#define _KHB_MAIN_H_

#include <linux/module.h> 
#include "khb_lcd.h"

/**** debugging related ***/
//#define khb_dbg_msg(MSG...)            printk(MSG)
#define khb_dbg_msg(MSG...)          
//#define khb_dbg_msg(LEVEL, MSG...)            printk(LEVEL MSG)

#if 0 
#define MSG_LVL_EMERG                           0
#define MSG_LVL_ALERT                           1
#define MSG_LVL_CRIT                            2
#define MSG_LVL_ERR                             3
#define MSG_LVL_WARNING                         4
#define MSG_LVL_NOTICE                          5
#define MSG_LVL_INFO                            6
#define MSG_LVL_DEBUG                           7
#endif

/*** LCD ***/
#define KHB_MIN_BOX_W	3
#define KHB_MIN_BOX_H	3

#define KHB_DEFAULT_FONT_SIZE	0
#define KHB_ENLARGED_FONT		1

#define KHB_DISCARD_PREV_MSG	0
#define KHB_RETAIN_PREV_MSG		1

int kernel_alive_indicator(int x, int y, int rowHeight, int colHeight, int mode);
int display_dbg_msg(char* txtMsg, unsigned char fontSize, unsigned char retainPrevMsg);
extern int uart_switch(int flags);
#endif /* #ifndef _KHB_MAIN_H_  */
