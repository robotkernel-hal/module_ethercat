//! robotkernel module ethercat master process data group
/*!
 * author: Robert Burger <robert.burger@dlr.de>
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

#include "group.h"
#include "master.h"

#include "string_util/string_util.h"

using namespace std;
using namespace module_ethercat;
using namespace string_util;

//! group creation
/*!
 * \param parent pointer to ethercat master class
 * \param index group index
 * \param node configuration node
 */
group::group(master *parent, int index, const YAML::Node& node) :
    trigger_device(parent->name, format_string("group_%d.trigger", index)),
    parent(parent)
{
    _index          = index;
    _divisor        = get_as<int>(node, "divisor");
    _divisor_cnt    = 0;
    recv_timeout   = get_as<int>(node, "recv_timeout", 1000000);

    for (YAML::const_iterator it = node["slaves"].begin(); 
            it != node["slaves"].end(); ++it)
        _slaves.push_back(it->as<int>());
}

