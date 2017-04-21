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

slave::canopen::canopen(std::shared_ptr<slave> slv, const request_type& type) 
:   service_provider::canopen_protocol::base(slv->master_dev->name, 
        format_string("slave_%d.%s", slv->index, 
            type == request_type_eeprom ? "eeprom" : "mailbox")), 
    slv(slv), type(type) {
}
        
//! return a list with all indices of the object dictionary
// derived from service_provider::canopen_protocol::base
/*!
 * \param list returns the list with all indices
 */
void slave::canopen::get_object_dictionary_list(
        service_provider::canopen_protocol::object_dictionary_list_t& list) {

    switch (type) {
        default:
            break;
        case request_type_eeprom: {
            list.push_back(0x1008);     // device name index

            ec_slave_t *ec_slv = &slv->master_dev->_pec->slaves[slv->index];
            ec_eeprom_cat_pdo_t *pdo;

            // inputs and outputs
            TAILQ_FOREACH(pdo, &ec_slv->eeprom.txpdos, qh) {
                list.push_back(pdo->pdo_index);
            }

            TAILQ_FOREACH(pdo, &ec_slv->eeprom.rxpdos, qh) {
                list.push_back(pdo->pdo_index);
            }
            break;
        }
        case request_type_mailbox: {
            uint8_t *buf = NULL;
            size_t len = 0;
            int ret = ec_coe_odlist_read(slv->master_dev->_pec, slv->index, &buf, &len);

            if (ret != 0) {
                throw str_exception("slave %2d: reading CoE object dictionary list "
                        "returned errorcode 0x%X!\n", slv->index, ret);
            }

            // buf is allocated by ec_coe_odlist_read
            if (buf) {
                list.resize(len/2);
                memcpy(&list[0], buf, len);

                free(buf);
            }
            break;
        }
    }
}

//! return a object description of given index
// derived from service_provider::canopen_protocol::base
/*!
 * \param index requested index
 * \param desc returns the object description
 */
void slave::canopen::get_object_description(const uint16_t& index, 
        service_provider::canopen_protocol::object_description_t& desc) {
    
    switch (type) {
        default:
            break;
        case request_type_eeprom: {
            ec_slave_t *ec_slv = &slv->master_dev->_pec->slaves[slv->index];
            ec_eeprom_cat_pdo_t *pdo;

            if (index == 0x1008) {
                desc.data_type         = 0x0009;
                desc.object_code       = 7;
                desc.max_subindices    = 0;
                desc.name              = string("Device Name");
                break;
            }

            struct ec_eeprom_cat_pdo_queue *pdos[] = {
                &ec_slv->eeprom.txpdos,
                &ec_slv->eeprom.rxpdos };

            for (int qi = 0; qi < 2; qi++) {
                TAILQ_FOREACH(pdo, pdos[qi], qh) {
                    if (pdo->pdo_index == index) {
                        desc.data_type         = DEFTYPE_PDOMAPPING;
                        desc.object_code       = pdo->n_entry > 1 ? 9 : 7;
                        desc.max_subindices    = pdo->n_entry;

                        if ((pdo->name_idx > 0) && 
                                (pdo->name_idx <= ec_slv->eeprom.strings_cnt)) {
                            desc.name          = string(ec_slv->eeprom.strings[pdo->name_idx-1]);
                        } 

                        return;
                    }
                }
            }
            break;
        }
        case request_type_mailbox: {
            // get description
            ec_coe_sdo_desc_t obj_desc;
            memset(&obj_desc, 0, sizeof(obj_desc));
            int ret = ec_coe_sdo_desc_read(slv->master_dev->_pec, slv->index, index, &obj_desc);

            if (ret != 0) {
                // decode ret
                throw str_exception("slave %2d: reading CoE object description index 0x%X "
                        "returned errorcode 0x%X!\n", slv->index, index, ret);
            }

            desc.data_type      = obj_desc.data_type;
            desc.object_code    = obj_desc.obj_code;
            desc.max_subindices = obj_desc.max_subindices;

            if (obj_desc.name) {
                desc.name = std::string(obj_desc.name, obj_desc.name_len);
                free(obj_desc.name);  // allocated by ec_coe_sdo_desc_read
            }
            break;
        }
    }
}

//! return a element description of given index and sub index
// derived from service_provider::canopen_protocol::base
/*!
 * \param index requested index
 * \param sub_index requested sub index
 * \param desc returns the object description
 */
void slave::canopen::get_element_description(const uint16_t& index, const uint8_t& sub_index,
        service_provider::canopen_protocol::element_description_t& desc) {
    
    switch (type) {
        default:
            break;
        case request_type_eeprom: {
            ec_slave_t *ec_slv = &slv->master_dev->_pec->slaves[slv->index];
            ec_eeprom_cat_pdo_t *pdo;

            // device name
            if (index == 0x1008) {
                if ((ec_slv->eeprom.strings_cnt > 0) && 
                        (ec_slv->eeprom.general.name_idx <= ec_slv->eeprom.strings_cnt)) {
                    desc.value_info = 0x7F;
                    desc.data_type = 0x0009;
                    desc.bit_length = strlen(ec_slv->eeprom.strings[ec_slv->eeprom.general.name_idx-1]) * 8;
                    desc.obj_access = 7;
                } else {
                    desc.value_info = 0x7F;
                    desc.data_type = 0x0009;
                    desc.bit_length = 7*8;
                    desc.obj_access = 7;
                }

                break;
            }
        
            struct ec_eeprom_cat_pdo_queue *pdos[] = {
                &ec_slv->eeprom.txpdos,
                &ec_slv->eeprom.rxpdos };

            for (int qi = 0; qi < 2; qi++) {
                TAILQ_FOREACH(pdo, pdos[qi], qh) {
                    if (pdo->pdo_index != index)
                        continue;

                    if (sub_index == 0) {
                        desc.value_info        = 0x7F;
                        desc.data_type         = 0x0005; //DEFTYPE_UNSIGNED8;
                        desc.bit_length        = 8;
                        desc.obj_access        = 7;
                        desc.name              = string("SubIndex_0");

                        return;
                    } else if (sub_index <= pdo->n_entry) {
                        ec_eeprom_cat_pdo_entry_t *entry = &pdo->entries[sub_index-1];

                        desc.value_info        = 0x7F;
                        desc.data_type         = entry->data_type;
                        desc.bit_length        = entry->bit_len;
                        desc.obj_access        = 7;

                        if ((entry->entry_name_idx > 0) &&
                                (entry->entry_name_idx <= ec_slv->eeprom.strings_cnt)) {
                            desc.name = string(ec_slv->eeprom.strings[entry->entry_name_idx-1]);
                        }

                        return;
                    }
                }
            }
            break;
        }
        case request_type_mailbox: {
            // get description
            ec_coe_sdo_entry_desc_t entry_desc;
            memset(&entry_desc, 0, sizeof(entry_desc));
            int ret = ec_coe_sdo_entry_desc_read(slv->master_dev->_pec, slv->index, index, 
                    sub_index, 0x7F, &entry_desc);

            if (ret != 0) {
                // decode ret
                throw str_exception("slave %2d: reading CoE element description index 0x%X "
                        "sub index %d returned errorcode 0x%X!\n", slv->index, 
                        index, sub_index, ret);
            }

            desc.value_info    = entry_desc.value_info;
            desc.data_type     = entry_desc.data_type;
            desc.bit_length    = entry_desc.bit_length;
            desc.obj_access    = entry_desc.obj_access;
            desc.unit          = 0;

            if (entry_desc.data) {
                // decode data
                uint8_t *tmp = entry_desc.data;
                if (entry_desc.value_info & EC_COE_SDO_VALUE_INFO_UNIT) {
                    if ((tmp + 2) <= (entry_desc.data + entry_desc.data_len)) {
                        desc.unit = *(uint16_t *)tmp;
                        tmp += 2;
                    }
                }

                if (entry_desc.value_info & EC_COE_SDO_VALUE_INFO_DEFAULT_VALUE) { 
                    size_t bytesize = (entry_desc.bit_length + 7) / 8;

                    if ((tmp + bytesize) <= (entry_desc.data + entry_desc.data_len)) {
                        desc.default_value.resize(bytesize);
                        memcpy(&desc.default_value[0], tmp, bytesize);
                        tmp += bytesize;
                    }
                }

                if (entry_desc.value_info & EC_COE_SDO_VALUE_INFO_MIN_VALUE) {
                    size_t bytesize = (entry_desc.bit_length + 7) / 8;

                    if ((tmp + bytesize) <= (entry_desc.data + entry_desc.data_len)) {
                        desc.min_value.resize(bytesize);
                        memcpy(&desc.min_value[0], tmp, bytesize);
                        tmp += bytesize;
                    }
                }

                if (entry_desc.value_info & EC_COE_SDO_VALUE_INFO_MAX_VALUE) {
                    size_t bytesize = (entry_desc.bit_length + 7) / 8;

                    if ((tmp + bytesize) <= (entry_desc.data + entry_desc.data_len)) {
                        desc.max_value.resize(bytesize);
                        memcpy(&desc.max_value[0], tmp, bytesize);
                        tmp += bytesize;
                    }
                }

                if (tmp < (entry_desc.data + entry_desc.data_len)) {
                    size_t restlen = (entry_desc.data + entry_desc.data_len) - tmp;
                    desc.name = std::string((char *)tmp, restlen);
                }

                free(entry_desc.data);
            }
            break;
        }
    }
}

//! reads one element
// derived from service_provider::canopen_protocol::base
/*!
 * \param index requested index
 * \param sub_index requested sub index
 * \param value returns read value 
 */
void slave::canopen::read_element(const uint16_t& index, const uint8_t& sub_index,
        service_provider::canopen_protocol::element_t& value) {
    switch (type) {
        default:
            break;
        case request_type_eeprom: {
            ec_slave_t *ec_slv = &slv->master_dev->_pec->slaves[slv->index];

            if (index == 0x1008) {
                if ((ec_slv->eeprom.strings_cnt > 0) &&
                        (ec_slv->eeprom.general.name_idx <= ec_slv->eeprom.strings_cnt)) {
                    string tmp = string(ec_slv->eeprom.strings[ec_slv->eeprom.general.name_idx-1]);
                    value.resize(tmp.size());
                    memcpy(&value[0], tmp.c_str(), value.size());
                    break;
                } 
            } else {
                ec_eeprom_cat_pdo_t *pdo;
                struct ec_eeprom_cat_pdo_queue *pdos[] = {
                    &ec_slv->eeprom.txpdos,
                    &ec_slv->eeprom.rxpdos };

                for (int qi = 0; qi < 2; qi++) {
                    TAILQ_FOREACH(pdo, pdos[qi], qh) {
                        if (pdo->pdo_index != index)
                            continue;

                        if (sub_index == 0) {
                            value.resize(1);
                            memcpy(&value[0], &pdo->n_entry, 1);
                            return;
                        } else if (sub_index <= pdo->n_entry) {
                            ec_eeprom_cat_pdo_entry_t *entry = &pdo->entries[sub_index-1];
                            value.resize(2);
                            memcpy(&value[0], &entry->entry_index, 2);
                            return;
                        }
                    } 
                }
            }
            break;
        }
        case request_type_mailbox: {
            uint8_t *buf = NULL; 
            size_t buf_len = 0;
            uint32_t abort_code = 0;

            int ret = ec_coe_sdo_read(slv->master_dev->_pec, slv->index, index, sub_index, 
                    0, &buf, &buf_len, &abort_code);

            if (ret != 0) {
                // decode ret
                throw str_exception("slave %2d: reading CoE element description index 0x%X "
                        "sub index %d returned errorcode 0x%X!\n", slv->index, 
                        index, sub_index, ret);
            }

            if (buf != (uint8_t *)&value[0]) {
                // ec_coe_sdo_read call did allocate buffer
                value.resize(buf_len);    
                memcpy(&value[0], buf, buf_len);

                free(buf);
            }
            break;
        }
    }
}

//! writes one element
// derived from service_provider::canopen_protocol::base
/*!
 * \param index requested index
 * \param sub_index requested sub index
 * \param value value to write
 */
void slave::canopen::write_element(const uint16_t& index, const uint8_t& sub_index,
        const service_provider::canopen_protocol::element_t& value) {
    switch (type) {
        default:
            break;
        case request_type_eeprom:
            throw str_exception("slave %2d: writing canopen value in eeprom is not supported!\n", 
                    slv->index);
        case request_type_mailbox: {
            uint32_t abort_code = 0;

            int ret = ec_coe_sdo_write(slv->master_dev->_pec, slv->index, index, sub_index, 
                    0, (uint8_t *)&value[0], value.size(), &abort_code);

            if (ret != 0) {
                // decode ret
                throw str_exception("slave %2d: writing CoE element value index 0x%X "
                        "sub index %d returned errorcode 0x%X!\n", slv->index, 
                        index, sub_index, ret);
            }
            break;
        }
    }
}
