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
#ifndef __INC_NMF_FIFO_DESC
#define __INC_NMF_FIFO_DESC

#include <inc/typedef.h>
#include <share/semaphores/inc/semaphores.h>

/*
 * SHOULD be mapped onto a AHB burst (16 bytes=8x16-bit)
 */
typedef struct {
    t_semaphore_id semId;

    t_uint16 elemSize;
    t_uint16 fifoFullValue;
    t_uint16 readIndex;
    t_uint16 writeIndex;
    t_uint16 wrappingValue;

    t_uint32 extendedField; /* in DSP 24 memory when to MPC in Logical Host when to ARM */
} t_nmf_fifo_desc;

#define EXTENDED_FIELD_BCTHIS_OR_TOP        0       //<! This field will be used:
                                                    //<! - as hostBCThis for DSP->HOST binding
                                                    //<! - as TOP else
#define EXTENDED_FIELD_BCDESC               1       //<! This field will be used for:
                                                    //<! - interface method address for ->MPC binding
                                                    //<! - for params size for ->Host binding (today only [0] is used as max size)

#endif /* __INC_NMF_FIFO */
