//! robotkernel module ethercat slave
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

#include "slave.h"
#include "master.h"

using namespace std;
using namespace robotkernel;
using namespace string_util;
using namespace module_ethercat;

slave::memory_inspection::memory_inspection(std::shared_ptr<slave> slv, const request_type& type)
:   service_provider::memory_inspection::base(slv->master_dev->name, 
        format_string("slave_%d.%s", slv->index, 
            type == request_type_eeprom ? "eeprom" : "memory")), 
    slv(slv), type(type) {

}
                
//! retreave all readable/writeable memory areas
/*!
 * \param areas list of areas
 */
void slave::memory_inspection::get_memory_areas(
        service_provider::memory_inspection::area_list_t& areas) {

    switch (type) {
        default:
            break;
        case request_type_eeprom: {
            service_provider::memory_inspection::memory_t mem = { 0x0, 0x8000 };
            areas.push_back(mem);
            break;
        }
        case request_type_memory: {
            service_provider::memory_inspection::memory_t mem = { 0x0, 0x8000 };
            areas.push_back(mem);
            break;
        }
    }
}

//! read memory
/*!
 * \param address start address
 * \param data read data
 */
void slave::memory_inspection::read_memory(const uint64_t& address, 
        service_provider::memory_inspection::data_t& data) {

    switch (type) {
        default:
            break;
        case request_type_eeprom: {
            ec_eepromread_len(&slv->master_dev->ec, 
                    slv->index, address/2, &data[0], data.size());
            break;
        }
        case request_type_memory: {
            for (unsigned offset = 0; offset < data.size(); offset+=100) {
                uint32_t act_len = min(100, data.size() - offset);
                uint16_t wkc;

                ec_fprd(&slv->master_dev->ec, slv->master_dev->ec.slaves[slv->index].fixed_address, 
                        address + offset, &data[0] + offset, act_len, &wkc);
            }
            break;
        }
    }
}

//! write memory
/*!
 * \param address start address
 * \param data data to write
 */
void slave::memory_inspection::write_memory(const uint64_t& address, 
        const service_provider::memory_inspection::data_t& data) {

    switch (type) {
        default:
            break;
        case request_type_eeprom: {
            ec_eepromwrite_len(&slv->master_dev->ec, slv->index, 
                    address, &data[0], data.size());
            break;
        }
        case request_type_memory: {
            for (unsigned offset = 0; offset < data.size(); offset+=100) {
                uint32_t act_len = min(100, data.size() - offset);
                uint16_t wkc;

                ec_fpwr(&slv->master_dev->ec, slv->master_dev->ec.slaves[slv->index].fixed_address, 
                    address + offset, &data[0] + offset, act_len, &wkc);
            }
            break;
        }
    }
}
