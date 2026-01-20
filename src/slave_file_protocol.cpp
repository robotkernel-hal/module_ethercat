//! robotkernel module ethercat slave
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

#include "slave.h"
#include "master.h"

using namespace std;
using namespace robotkernel;
using namespace robotkernel::helpers;
using namespace module_ethercat;

slave::file_protocol::file_protocol(std::shared_ptr<slave> slv) :
    service_provider_file_protocol::base(slv->master_dev->name, 
            slv->master_dev->use_real_names ?
            string_printf("%s.mailbox", slv->name.c_str()) : 
            string_printf("slave_%d.mailbox", slv->index)), 
    slv(slv) 
{
}

//! read from file
/*!
 * \param info file info structure
 */
void slave::file_protocol::file_read(
        service_provider_file_protocol::file_readwrite_info_t& info) {
    // file data buffer
    uint8_t *buffer = NULL;
    size_t buffer_len = 0;

    // file name truncation
    char file_name[MAX_FILE_NAME_SIZE];
    strncpy(file_name, info.file_name.c_str(), MAX_FILE_NAME_SIZE-1);

    // others
    uint32_t password = info.password;
    const char *error_message = NULL; 

    ec_foe_read(
            &slv->master_dev->ec,   // ethercat master device
            slv->index,             // slave index
            password,               // file password
            file_name,              // file name
            &buffer,                // returns file_data
            &buffer_len,            // returns file_data_len
            &error_message);        // returns error_message

    if (error_message) {
        if (buffer)
            free(buffer);

        std::string msg = string(error_message);
        throw runtime_error(msg);
    }

    if (buffer) {
        info.file_data.resize(buffer_len);
        memcpy(&info.file_data[0], buffer, buffer_len);
    }
}

//! write to file
/*!
 * \param info file info structure
 */
void slave::file_protocol::file_write(
        const service_provider_file_protocol::file_readwrite_info_t& info) {
    slv->master_dev->log(robotkernel::info, "writing file %s\n", info.file_name.c_str());

    // file name truncation
    char file_name[MAX_FILE_NAME_SIZE];
    strncpy(file_name, info.file_name.c_str(), MAX_FILE_NAME_SIZE-1);

    // others
    uint32_t password = info.password;
    const char *error_message = NULL; 
            
    // local copy, cause it's const
    auto file_data = info.file_data;

    ec_foe_write(
            &slv->master_dev->ec,   // ethercat master device
            slv->index,             // slave index
            password,               // file password
            file_name,              // file name
            &file_data[0],          // file_data
            file_data.size(),       // file_data_len
            &error_message);        // returns error_message
    
    if (error_message) {
        std::string msg = string(error_message);
        slv->master_dev->log(robotkernel::error, "writing file failed: %s\n", 
                msg.c_str());

        throw runtime_error(msg);
    }

    slv->master_dev->log(robotkernel::info, "writing file succeeded!\n");
}

