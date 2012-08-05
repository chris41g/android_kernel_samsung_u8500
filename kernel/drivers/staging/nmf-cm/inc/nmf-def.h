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
/*!
 * \brief NMF Version.
 *
 * This file contains the NMF Version.
 *
 * \defgroup NMF_VERSION NMF Version
 * \ingroup COMMON
 */

#ifndef __INC_NMF_DEF_H
#define __INC_NMF_DEF_H

/*!
 * \brief Current NMF version number
 *
 * \ingroup NMF_VERSION
 */
#define NMF_VERSION ((2 << 16) | (10 << 8) | (118))

/*!
 * \brief Get NMF major version corresponding to NMF version number
 * \ingroup NMF_VERSION
 */
#define VERSION_MAJOR(version)  (((version) >> 16) & 0xFF)
/*!
 * \brief Get NMF minor version corresponding to NMF version number
 * \ingroup NMF_VERSION
 */
#define VERSION_MINOR(version)  (((version) >> 8) & 0xFF)
/*!
 * \brief Get NMF patch version corresponding to NMF version number
 * \ingroup NMF_VERSION
 */
#define VERSION_PATCH(version)  (((version) >> 0) & 0xFF)

#endif /* __INC_NMF_DEF_H */
