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
#ifndef __INC_SHARED_SEMAPHORE_H
#define __INC_SHARED_SEMAPHORE_H

#include <share/inc/nmf.h>

typedef t_uint16 t_semaphore_id;

/*
 * HW semaphore allocation
 * -----------------------
 *  We want to optimize interrupt demultiplexing at dsp interrupt handler level
 *  so a good solution would be to have sequentially the semaphores for each neighbors
 *
 * STn8500 :
 * ---------
 * ARM <- SVA  COMS => 0
 * ARM <- SIA  COMS => 1
 * SVA <- ARM  COMS => 2
 * SVA <- SIA  COMS => 3
 * SIA <- ARM  COMS => 4
 * SIA <- SVA  COMS => 5

 * The first neighbor is always the ARM, then the other ones (SVA,SIA)
 */

/*
 * Local semaphore allocation
 * -----------------------
 * 0 : ARM <- DSP
 * 1 : DSP <- ARM
 */

#define NB_USED_HSEM_PER_CORE (NB_CORE_IDS - 1)
#define FIRST_NEIGHBOR_SEMID(coreId) ((coreId)*NB_USED_HSEM_PER_CORE)

#endif /* __INC_SHARED_SEMAPHORE_H */
