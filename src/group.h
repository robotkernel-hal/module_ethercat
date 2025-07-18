//! robotkernel module ethercat master process data group
/*!
 * author: Robert Burger <robert.burger@dlr.de>
 */

/*
 * This file is part of module_ethercat.
 *
 * module_ethercat is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 * 
 * module_ethercat is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with module_ethercat; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef MODULE_ETHERCAT__GROUP_H
#define MODULE_ETHERCAT__GROUP_H

#include <list>
#include <string>

#include <libosal/timer.h>

#include "yaml-cpp/yaml.h"
#include "robotkernel/trigger.h"

//! module_ethercat::
namespace module_ethercat {

// forward declarations
class master;

class group : 
    public std::enable_shared_from_this<group>,
    public robotkernel::trigger
{
    public:

        //! group creation
        /*!
         * \param parent pointer to ethercat master class
         * \param index group index
         * \param node configuration node
         */
        group(master *parent, int index, const YAML::Node& node);

        void set_rate(double new_rate) { rate = new_rate; }

        int recv_timeout;
        int _index;
        int divisor;
        int _divisor_cnt;
        bool overlapping;
        bool lrw;
        std::list<int> _slaves;

        osal_timer_t timeout;
        master *parent;
};

//! module_ethercat::
};

#endif // MODULE_ETHERCAT__GROUP_H

