//! robotkernel module ethercat slave
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

#include "slave.h"
#include "master.h"

using namespace std;
using namespace robotkernel;
using namespace string_util;
using namespace module_ethercat;

slave::canopen::canopen(std::shared_ptr<slave> slv, const request_type& type) :
    service_provider::canopen_protocol::base(slv->master_dev->name, format_string(
                "slave_%d.%s", slv->index, type == request_type_eeprom ? "eeprom" : "mailbox")), 
    slv(slv), type(type) 
{
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

            ec_slave_t *ec_slv = &slv->master_dev->ec.slaves[slv->index];
            ec_eeprom_cat_pdo_t *pdo;

            // inputs and outputs
            TAILQ_FOREACH(pdo, &ec_slv->eeprom.txpdos, qh) {
                list.push_back(pdo->pdo_index);
                
                for (uint8_t sub_index = 0; sub_index < pdo->n_entry; sub_index++) {
                    ec_eeprom_cat_pdo_entry_t *entry = &pdo->entries[sub_index];

                    if (entry->entry_index != 0) {
                        list.push_back(entry->entry_index);
                    }
                }
            }

            TAILQ_FOREACH(pdo, &ec_slv->eeprom.rxpdos, qh) {
                list.push_back(pdo->pdo_index);
                
                for (uint8_t sub_index = 0; sub_index < pdo->n_entry; sub_index++) {
                    ec_eeprom_cat_pdo_entry_t *entry = &pdo->entries[sub_index];

                    if (entry->entry_index != 0) {
                        list.push_back(entry->entry_index);
                    }
                }
            }
            break;
        }
        case request_type_mailbox: {
            uint8_t buf[COE_DATA_MAXLEN];
            size_t len = COE_DATA_MAXLEN;
            int ret = ec_coe_odlist_read(&slv->master_dev->ec, slv->index, buf, &len);

            if (ret == EC_ERROR_MAILBOX_BUFFER_TOO_SMALL) {
                // got bigger length from libethercat
                list.resize(len/2);
                int ret = ec_coe_odlist_read(&slv->master_dev->ec, slv->index, (osal_uint8_t *)&list[0], &len);

                if (ret != 0) {
                    throw str_exception("slave %2d: reading CoE object dictionary list "
                            "returned errorcode 0x%X!\n", slv->index, ret);
                }

                // reduce size to eliminite padding bytes at the end
                list.resize(len/2);
            } else if (ret == 0) {
                list.resize(len/2);
                memcpy(&list[0], buf, len);
            } else {
                throw str_exception("slave %2d: reading CoE object dictionary list "
                        "returned errorcode 0x%X!\n", slv->index, ret);
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
            ec_slave_t *ec_slv = &slv->master_dev->ec.slaves[slv->index];
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
                        desc.object_code       = pdo->n_entry > 0 ? 9 : 7;
                        desc.max_subindices    = pdo->n_entry;

                        if ((pdo->name_idx > 0) && 
                                (pdo->name_idx <= ec_slv->eeprom.strings_cnt)) {
                            desc.name          = string(ec_slv->eeprom.strings[pdo->name_idx-1]);
                        } 

                        return;
                    }

                    for (uint8_t sub_index = 0; sub_index < pdo->n_entry; sub_index++) {
                        ec_eeprom_cat_pdo_entry_t *entry = &pdo->entries[sub_index];

                        if (entry->entry_index == index) {
                            desc.data_type         = entry->data_type;
                            desc.object_code       = OBJCODE_VAR;
                            desc.max_subindices    = 0;

                            if ((entry->entry_name_idx > 0) && 
                                    (entry->entry_name_idx <= ec_slv->eeprom.strings_cnt)) {
                                desc.name          = string(ec_slv->eeprom.strings[entry->entry_name_idx-1]);
                            } 

                            return;
                        }
                    }
                }
            }
            break;
        }
        case request_type_mailbox: {
            // get description
            uint32_t error_code = 0;
            ec_coe_sdo_desc_t obj_desc;
            memset(&obj_desc, 0, sizeof(obj_desc));
            int ret = ec_coe_sdo_desc_read(&slv->master_dev->ec, slv->index, index, &obj_desc, &error_code);

            if (ret != 0) {
                // decode ret
                throw str_exception("slave %2d: reading CoE object description index 0x%X "
                        "returned errorcode 0x%X: %s!\n", slv->index, index, error_code, get_sdo_info_error_string(error_code));
            }

            desc.data_type      = obj_desc.data_type;
            desc.object_code    = obj_desc.obj_code;
            desc.max_subindices = obj_desc.max_subindices;

            if (obj_desc.name) {
                desc.name = std::string(obj_desc.name, obj_desc.name_len);
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
            ec_slave_t *ec_slv = &slv->master_dev->ec.slaves[slv->index];
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
                    if (pdo->pdo_index != index) {
                        for (uint8_t sub_index = 0; sub_index < pdo->n_entry; sub_index++) {
                            ec_eeprom_cat_pdo_entry_t *entry = &pdo->entries[sub_index];

                            if (entry->entry_index == index) {
                                desc.value_info        = 0x7F;
                                desc.data_type         = entry->data_type;
                                desc.bit_length        = entry->bit_len;
                                desc.obj_access        = 7;


                                if ((entry->entry_name_idx > 0) && 
                                        (entry->entry_name_idx <= ec_slv->eeprom.strings_cnt)) {
                                    desc.name          = string(ec_slv->eeprom.strings[entry->entry_name_idx-1]);
                                } 

                                return;
                            }
                        }
                        continue;
                    }

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
                        desc.data_type         = 0x0007; //DETTYPE_UNSIGNED32; entry->data_type;
                        desc.bit_length        = 32;     //entry->bit_len;
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
            uint32_t error_code = 0;
            ec_coe_sdo_entry_desc_t entry_desc;
            memset(&entry_desc, 0, sizeof(entry_desc));
            int ret = ec_coe_sdo_entry_desc_read(&slv->master_dev->ec, slv->index, index, 
                    sub_index, 0x7F, &entry_desc, &error_code);

            if (ret != 0) {
                // decode ret
                throw str_exception("slave %2d: reading CoE element description index 0x%X sub index %d"
                        "returned errorcode 0x%X: %s!\n", slv->index, index, sub_index, error_code, get_sdo_info_error_string(error_code));
            }

            desc.value_info    = entry_desc.value_info;
            desc.data_type     = entry_desc.data_type;
            desc.bit_length    = entry_desc.bit_length;
            desc.obj_access    = entry_desc.obj_access;
            desc.unit          = 0;

            if (entry_desc.data_len > 0) {
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

                    if ((signed)desc.name.length() != std::count_if(desc.name.begin(), desc.name.end(), 
                                [](unsigned char c){ return std::isprint(c); } ))
                        desc.name = format_string("subindex_%d", sub_index); // name is not printable
                }
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
            ec_slave_t *ec_slv = &slv->master_dev->ec.slaves[slv->index];

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
                        if (pdo->pdo_index != index) {
                            for (uint8_t sub_index = 0; sub_index < pdo->n_entry; sub_index++) {
                                ec_eeprom_cat_pdo_entry_t *entry = &pdo->entries[sub_index];

                                if (entry->entry_index == index) {
                                    value.resize((entry->bit_len + 7)/8);
                                    memset(&value[0], 0, (entry->bit_len + 7)/8);
                                    return;
                                }
                            }
                            
                            continue;
                        }

                        if (sub_index == 0) {
                            value.resize(1);
                            memcpy(&value[0], &pdo->n_entry, 1);
                            return;
                        } else if (sub_index <= pdo->n_entry) {
                            ec_eeprom_cat_pdo_entry_t *entry = &pdo->entries[sub_index-1];
                            value.resize(4);
                            uint32_t tmp_val = 0u;
                            tmp_val |= ((uint32_t)entry->entry_index << 16u) & 0xFFFF0000u;
                            tmp_val |= ((uint32_t)entry->sub_index << 8u) & 0x0000FF00u;;
                            tmp_val |= entry->bit_len & 0x000000FFu;
                            memcpy(&value[0], &tmp_val, 4);
                            return;
                        }
                    } 
                }
            }
            break;
        }
        case request_type_mailbox: {
            uint8_t buf[64]; 
            size_t buf_len = 64;
            uint32_t abort_code = 0;

            int ret = ec_coe_sdo_read(&slv->master_dev->ec, slv->index, index, sub_index, 
                    0, &buf[0], &buf_len, &abort_code);

            if (ret != 0) {
                if (ret == EC_ERROR_MAILBOX_ABORT) {
                    throw service_provider::canopen_protocol::sdo_abort_exception(abort_code);
                }

                // decode ret
                throw str_exception("slave %2d: reading CoE element index 0x%X "
                        "sub index %d returned errorcode 0x%X!\n", slv->index, 
                        index, sub_index, ret);
            }

            if (buf_len) {
                value.resize(buf_len);    
                memcpy(&value[0], buf, buf_len);
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

            int ret = ec_coe_sdo_write(&slv->master_dev->ec, slv->index, index, sub_index, 
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

//! pop next emer   gency message, throw exception if non present
/*!
 * \param msg ret   urn emergency message
 */
void slave::canopen::pop_emergency_message(
        service_provider::canopen_protocol::emergency_message_t& msg) {
    ec_coe_emergency_message_t msg_tmp;
    if (ec_coe_emergency_get_next(&slv->master_dev->ec, slv->index, &msg_tmp) == EC_OK) {
        msg.ts.tv_sec = msg_tmp.timestamp.sec;
        msg.ts.tv_nsec = msg_tmp.timestamp.nsec;
        msg.error_code = (uint16_t)msg_tmp.msg[0] | ((uint16_t)msg_tmp.msg[1] << 8);
        msg.error_register = msg_tmp.msg[2]; 
    
        for (unsigned i = 3; i < msg_tmp.msg_len; ++i) {
            msg.data.push_back(msg_tmp.msg[i]);
        }
    }
}

typedef struct data_type_desc {
    std::string data_type;
    int bitsize;
    int signprefix;
} data_type_desc_t;

std::map<uint16_t, data_type_desc_t> data_type_2_desc = {
    { 0x0000, { "null"            , 0  , 0 } },
    { 0x0001, { "bool_t"          , 1  , 0 } },
    { 0x0002, { "int8_t"          , 8  , 1 } },
    { 0x0003, { "int16_t"         , 16 , 1 } },
    { 0x0004, { "int32_t"         , 32 , 1 } },
    { 0x0005, { "uint8_t"         , 8  , 0 } },
    { 0x0006, { "uint16_t"        , 16 , 0 } },
    { 0x0007, { "uint32_t"        , 32 , 0 } },
    { 0x0008, { "float"           , 32 , 0 } },
    { 0x0009, { "string"          , -1 , 0 } },
    { 0x000A, { "string"          , -1 , 0 } },
    { 0x000B, { "string"          , -1 , 0 } },
    { 0x000C, { "time_of_day"     , -1 , 0 } },
    { 0x000D, { "time_difference" , -1 , 0 } },
    { 0x0010, { "int24_t"         , 24 , 1 } },
    { 0x0011, { "double"          , 64 , 0 } },
    { 0x0012, { "int40_t"         , 40 , 1 } },
    { 0x0013, { "int48_t"         , 48 , 1 } },
    { 0x0014, { "int56_t"         , 56 , 1 } },
    { 0x0015, { "int64_t"         , 64 , 1 } },
    { 0x0016, { "uint24_t"        , 24 , 0 } },
    { 0x0018, { "uint40_t"        , 40 , 0 } },
    { 0x0019, { "uint48_t"        , 48 , 0 } },
    { 0x001A, { "uint56_t"        , 56 , 0 } },
    { 0x001B, { "uint64_t"        , 64 , 0 } },
    { 0x001D, { "guid"            , -1 , 0 } },
    { 0x001E, { "uint8_t"         , 8  , 0 } },
    { 0x001F, { "uint16_t"        , 16 , 0 } },
    { 0x0020, { "uint32_t"        , 32 , 0 } },
    { 0x0021, { "pdo_mapping_t"   , -1 , 0 } },
    { 0x0023, { "identity_t"      , -1 , 0 } },
    { 0x0025, { "command_t"       , -1 , 0 } },
    { 0x0027, { "pdocompar_t"     , -1 , 0 } },
    { 0x0028, { "enum_t"          , -1 , 0 } },
    { 0x0029, { "smpar_t"         , -1 , 0 } },
    { 0x002A, { "record_t"        , -1 , 0 } },
    { 0x002B, { "backup_t"        , -1 , 0 } },
    { 0x002C, { "mdp_t"           , -1 , 0 } },
    { 0x002D, { "bitarr8_t"       , -1 , 0 } },
    { 0x002E, { "bitarr16_t"      , -1 , 0 } },
    { 0x002F, { "bitarr32_t"      , -1 , 0 } },
    { 0x0030, { "bit1_t"          , 1  , 0 } },
    { 0x0031, { "bit2_t"          , 2  , 0 } },
    { 0x0032, { "bit3_t"          , 3  , 0 } },
    { 0x0033, { "bit4_t"          , 4  , 0 } },
    { 0x0034, { "bit5_t"          , 5  , 0 } },
    { 0x0035, { "bit6_t"          , 6  , 0 } },
    { 0x0036, { "bit7_t"          , 7  , 0 } },
    { 0x0037, { "bit8_t"          , 8  , 0 } },
    { 0x0260, { "vector/int32_t"  , -1 , 0 } },
    { 0x0261, { "vector/int16_t"  , -1 , 0 } },
    { 0x0262, { "vector/int64_t"  , -1 , 0 } },
    { 0x0263, { "vector/uint64_t" , -1 , 0 } },
    { 0x0281, { "error_handling_t", -1 , 0 } },
    { 0x0282, { "diag_history_t"  , -1 , 0 } },
    { 0x0283, { "sync_status_t"   , -1 , 0 } },
    { 0x0284, { "sync_settings_t" , -1 , 0 } },
    { 0x0285, { "fsoe_frame_t"    , -1 , 0 } },
    { 0x0286, { "fsoe_commpar_t"  , -1 , 0 } } };
 
//! return process data description yaml string 
/*!
 * \param idx pdo index, usually 0x1C12 (RxPDO) or 0x1C13 (TxPDO)
 */
string slave::canopen::get_pdo_description(uint16_t idx) {
    YAML::Emitter out;
    out << YAML::BeginSeq;

    // read mapped pdo count
    service_provider::canopen_protocol::element_t element;
    read_element(idx, 0, element);
    uint8_t entry_cnt = element[0];

    slv->master_dev->log(verbose, "getting pdo description idx 0x%X : reading %d entries\n", 
            idx, entry_cnt);

    int stored_bits = 0;
    int combined_cnt = 0;

    // now read all mapped pdo's to retreave the mapped object lengths
    for (int i = 1; i <= entry_cnt; ++i) {
        // read mapped pdo
        read_element(idx, i, element);
        uint16_t entry_idx = *(uint16_t *)&element[0];

        if (entry_idx == 0)
            continue; // skip this one

        // read mapped element count
        read_element(entry_idx, 0, element);
        uint8_t entry_cnt_2 = element[0];

        slv->master_dev->log(verbose, "  mapped pdo 0x%X, count %d\n", entry_idx, entry_cnt_2);

        for (int entry_sub_idx = 1; entry_sub_idx <= entry_cnt_2; ++entry_sub_idx) {
            // read mapped element 
            read_element(entry_idx, entry_sub_idx, element);
            uint32_t entry = *(uint32_t *)&element[0];

            service_provider::canopen_protocol::element_description_t desc;
            service_provider::canopen_protocol::object_description_t obj_desc;

            uint16_t pdo_entry_id = (entry & 0xFFFF0000) >> 16;
            uint16_t pdo_entry_subid = (entry & 0x0000FF00) >> 8;
                
            desc.name = "";
            desc.data_type = 0;

            if (pdo_entry_id > 0) {
                try {
                    get_element_description(pdo_entry_id, pdo_entry_subid, desc);
                } catch (std::exception& e) {
                    slv->master_dev->log(verbose, "%s\n", e.what());
                }
            
                if ((pdo_entry_subid == 0) || slv->prefer_obj_names) {
                    try {
                        get_object_description(pdo_entry_id, obj_desc);
                        slv->master_dev->log(verbose, "got obj_desc.name %s\n", obj_desc.name.c_str());

                        if (obj_desc.name != "") {
                            desc.name = obj_desc.name;
                        }
                    } catch (std::exception& e) {
                        slv->master_dev->log(verbose, "%s\n", e.what());
                    }
                }
            }

            desc.name.erase(remove_if(desc.name.begin(), desc.name.end(), 
                        [](unsigned char c){ return !std::isprint(c); }), desc.name.end());  

            //if ((signed)desc.name.length() != std::count_if(desc.name.begin(), desc.name.end(), 
            //            [](unsigned char c){ return std::isprint(c); } )) {
            //    desc.name = format_string("no_text_%d", entry_sub_idx); // name is not printable
            //}

            if (desc.name == "") {
                desc.name = format_string("padding_%d", entry_sub_idx);
            }

            auto& _data_type_desc = data_type_2_desc[desc.data_type];
            string data_type = _data_type_desc.data_type;

            if (desc.data_type == 0x0000) {
                stringstream ss;
                ss << "int" << (entry & 0x000000FF) << "_t";
                data_type = ss.str();
            } else {
                if ((_data_type_desc.bitsize >= 0) && ((unsigned)_data_type_desc.bitsize != (entry & 0x000000FFu))) {
                    slv->master_dev->log(warning, "    subindex %d, mappend bitsize %d, datatype bitsize %d mismatch!\n", 
                            entry_sub_idx, (entry & 0x000000FF), _data_type_desc.bitsize);

                    stringstream ss;
                    if (_data_type_desc.signprefix == 1)
                        ss << "int";
                    else 
                        ss << "uint";
                    ss << (entry & 0x000000FF) << "_t";
                    data_type = ss.str();
                }
            }
    
            slv->master_dev->log(verbose, "    subindex %d, entry %08X, name %s\n", entry_sub_idx, entry, desc.name.c_str());

            if (((entry & 0x000000FF) % 8) != 0) {
                stored_bits += (entry & 0x000000FF);
            } else {
                if (stored_bits != 0) {
                    stringstream ss;
                    ss << "uint" << (((stored_bits + 7) / 8) * 8) << "_t";
                    string combined_data_type = ss.str();

                    ss.str("");
                    ss << "combined_" << combined_cnt++;
                    string combined_name = ss.str();

                    out << YAML::BeginMap;
                    out << YAML::Key << combined_data_type << YAML::Value << combined_name;
                    out << YAML::EndMap;

                    stored_bits = 0;
                }

                out << YAML::BeginMap;
                out << YAML::Key << data_type << YAML::Value << desc.name;
                out << YAML::EndMap;
            }
        }                        

        if (stored_bits != 0) {
            stringstream ss;
            ss << "uint" << (((stored_bits + 7) / 8) * 8) << "_t";
            string combined_data_type = ss.str();

            ss.str("");
            ss << "combined_" << combined_cnt++;
            string combined_name = ss.str();

            out << YAML::BeginMap;
            out << YAML::Key << combined_data_type << YAML::Value << combined_name;
            out << YAML::EndMap;

            stored_bits = 0;
        }

    }
                    
    out << YAML::EndSeq;
    return out.c_str();
}

