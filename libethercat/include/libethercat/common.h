//! ethercat master common stuff
/*!
 * author: Robert Burger
 *
 * $Id$
 */

/*
 * This file is part of libethercat.
 *
 * libethercat is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * libethercat is distributed in the hope that 
 * it will be useful, but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with libethercat
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __LIBETHERCAT_COMMON_H__
#define __LIBETHERCAT_COMMON_H__

#include "stdint.h"

#define PACKED __attribute__((__packed__))
#define min(a, b)  ((a) < (b) ? (a) : (b))

typedef union ec_data {
    uint8_t     bdata[1]; /* variants for easy data access */
    uint16_t    wdata[1];
    uint32_t    ldata[1];
} ec_data_t;

#endif // __LIBETHERCAT_COMMON_H__

