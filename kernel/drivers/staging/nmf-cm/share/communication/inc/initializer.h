/*
 *  Copyright (C) 2012 ST-Ericsson Co.Ltd
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
 */
#ifndef __INC_SHARE_INITIALIZER
#define __INC_SHARE_INITIALIZER

#define NMF_CONSTRUCT_INDEX         0
#define NMF_START_INDEX             1
#define NMF_STOP_INDEX              2
#define NMF_DESTROY_INDEX           3
#define NMF_UPDATE_STACK            4
#define NMF_LOCK_CACHE              5
#define NMF_UNLOCK_CACHE            6
#define NMF_ULP_FORCEWAKEUP         7
#define NMF_ULP_ALLOWSLEEP          8
#define NMF_CONSTRUCT_SYNC_INDEX    9
#define NMF_START_SYNC_INDEX        10
#define NMF_STOP_SYNC_INDEX         11

/*
 * Index of datas in command parameter format
 */
#define INIT_COMPONENT_CMD_HANDLE_INDEX         0
#define INIT_COMPONENT_CMD_THIS_INDEX           2
#define INIT_COMPONENT_CMD_METHOD_INDEX         4
#define INIT_COMPONENT_CMD_SIZE                 6

/*
 * Index of datas in acknowledge parameter format
 */
#define INIT_COMPONENT_ACK_HANDLE_INDEX         0
#define INIT_COMPONENT_ACK_SIZE                 2

#endif /* __INC_SHARE_INITIALIZER */
