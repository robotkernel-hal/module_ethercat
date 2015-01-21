//! robotkernel module ethercat
/*!
 * author: Robert Burger
 *
 * $Id$
 */

/*
 * This file is part of robotkernel.
 *
 * robotkernel is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * robotkernel is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with robotkernel.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __MODULE_ETHERCAT_H__
#define __MODULE_ETHERCAT_H__

#include <sys/queue.h>
#include "robotkernel/module_intf.h"
#include "robotkernel/kernel.h"

#define MEM_ADDRESS(x)          ((x) & 0x0000FFFF)
#define MEM_TYPE_SLAVE_MEM      0x00000000
#define MEM_TYPE_SLAVE_EEPROM   0x00010000
#define MEM_TYPE_MASK           0x000F0000

void ethercat_log(robotkernel::loglevel lvl, std::string name, const char *format, ...);

namespace module_ethercat {

}; // namespace module_ethercat

#endif // __MODULE_ETHERCAT_H__

