//! ethercat distributed clocks
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

#ifndef __DC_H__
#define __DC_H__

#include "ec.h"

//! configure slave for distributed clock sync 0 pulse
/*/
 * \param pec ethercat master pointer 
 * \oaran slave slave number
 * \param active dc active flag
 * \param cycle_time cycle time to program to fire sync 0 in [ns]
 * \param cycle_shift shift of first sync 0 start in [ns]
 */
void ec_dc_sync0(ec_t *pec, uint16_t slave, int active, 
        uint32_t cycle_time, uint32_t cycle_shift);

//! configure slave for distributed clock sync 0 and sync 1 pulse
/*/
 * \param pec ethercat master pointer 
 * \oaran slave slave number
 * \param active dc active flag
 * \param cycle_time_0 cycle time to program to fire sync 0 in [ns]
 * \param cycle_time_1 cycle time to program to fire sync 1 in [ns]
 * \param cycle_shift shift of first sync 0 start in [ns]
 */
void ec_dc_sync01(ec_t *pec, uint16_t slave, int active, 
        uint32_t cycle_time_0, uint32_t cycle_time_1, uint32_t cycle_shift);

#endif // __DC_H__

