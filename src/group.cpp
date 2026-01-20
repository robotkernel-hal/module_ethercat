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

#include "group.h"
#include "master.h"

#include "robotkernel/helpers.h"

using namespace std;
using namespace robotkernel;
using namespace robotkernel::helpers;
using namespace module_ethercat;

//! group creation
/*!
 * \param parent pointer to ethercat master class
 * \param index group index
 * \param node configuration node
 */
group::group(master *parent, int index, const YAML::Node& node) :
    trigger(parent->name, string_printf("group_%d", index)), parent(parent)
{
    _index         = index;
    divisor        = get_as<int>(node, "divisor");
    _divisor_cnt   = 0;
    recv_timeout   = get_as<int>(node, "recv_timeout", 1000000);
    overlapping    = get_as<bool>(node, "overlapping", true);
    lrw            = get_as<bool>(node, "lrw", true);

    for (YAML::const_iterator it = node["slaves"].begin(); 
            it != node["slaves"].end(); ++it)
        _slaves.push_back(it->as<int>());
}

