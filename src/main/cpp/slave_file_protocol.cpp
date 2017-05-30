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

slave::file_protocol::file_protocol(std::shared_ptr<slave> slv)
    :   service_provider::file_protocol::base(slv->master_dev->name, 
            format_string("slave_%d.mailbox", slv->index)), slv(slv) 
{
}

        //! file read 
        //        typedef struct file_readwrite_info {
        //                    std::string          password;   //! [in]     file password
        //                                std::string          file_name;  //! [in]     file name
        //                                            std::vector<uint8_t> file_data;  //! [in/out] file data
        //                                                    } file_readwrite_info_t;
        //
//! read from file
/*!
 * \param info file info structure
 */
void slave::file_protocol::file_read(
        service_provider::file_protocol::file_readwrite_info_t& info) {
    // file data buffer
    uint8_t *buffer = NULL;
    ssize_t  buffer_len = 0;

    // file name truncation
    char file_name[MAX_FILE_NAME_SIZE];
    strncpy(file_name, info.file_name.c_str(), MAX_FILE_NAME_SIZE);

    // others
    uint32_t password = 0;
    char *error_message = NULL; 

    ec_foe_read(
            slv->master_dev->_pec,  // ethercat master device
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
        free(error_message);
        throw str_exception(msg.c_str());
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
        const service_provider::file_protocol::file_readwrite_info_t& info) {
    slv->master_dev->log(robotkernel::info, "writing file %s\n", info.file_name.c_str());

    // file name truncation
    char file_name[MAX_FILE_NAME_SIZE];
    strncpy(file_name, info.file_name.c_str(), MAX_FILE_NAME_SIZE);

    // others
    uint32_t password = 0;
    char *error_message = NULL; 
            
    // local copy, cause it's const
    auto file_data = info.file_data;

    ec_foe_write(
            slv->master_dev->_pec,  // ethercat master device
            slv->index,             // slave index
            password,               // file password
            file_name,              // file name
            &file_data[0],          // file_data
            file_data.size(),       // file_data_len
            &error_message);        // returns error_message
    
    if (error_message) {
        std::string msg = string(error_message);
        free(error_message);
        throw str_exception(msg.c_str());
    }

}

