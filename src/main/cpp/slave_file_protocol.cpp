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
            format_string("slave_%d", slv->index)), slv(slv) 
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
//    file_readwrite_info_t *frwi = (file_readwrite_info_t *)ptr;
//    uint32_t password = 0;
//    char file_name[MAX_FILE_NAME_SIZE];
//    strncpy(file_name, frwi->file_name, MAX_FILE_NAME_SIZE-1);
//    if (info.password)
//        password = strtol(frwi.password.c_str(), NULL, 10);
//
//    ret = ec_foe_read(
//            _pec,                   // ethercat master device
//            frwi->slave_id,         // slave index
//            password,               // file password
//            file_name,              // file name
//            &frwi->file_data,       // returns file_data
//            &frwi->file_data_len,   // returns file_data_len
//            &frwi->error_message);  // returns error_message
//
//    break;
}

//! write to file
/*!
 * \param info file info structure
 */
void slave::file_protocol::file_write(
        const service_provider::file_protocol::file_readwrite_info_t& info) {
//        case MOD_REQUEST_FILE_WRITE: {
//            file_readwrite_info_t *frwi = (file_readwrite_info_t *)ptr;
//            uint32_t password = 0;
//            char file_name[MAX_FILE_NAME_SIZE];
//            strncpy(file_name, frwi->file_name, MAX_FILE_NAME_SIZE-1);
//            if (frwi->password)
//                password = strtol(frwi->password, NULL, 10);
//
//            ret = ec_foe_write(
//                    _pec,                   // ethercat master device
//                    frwi->slave_id,         // slave index
//                    password,               // file password
//                    file_name,              // file name
//                    frwi->file_data,        // file_data
//                    frwi->file_data_len,    // file_data_len
//                    &frwi->error_message);  // returns error_message
//
//            break;
//        }
}

