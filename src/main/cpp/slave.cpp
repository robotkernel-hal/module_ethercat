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
#include "robotkernel/kernel.h"
#include "robotkernel/helpers.h"
#include "string_util/string_util.h"
#include <iomanip>
#include <stdio.h>

using namespace std;
using namespace robotkernel;
using namespace string_util;
using namespace module_ethercat;

pthread_mutex_t slave_lock = PTHREAD_MUTEX_INITIALIZER;

//! forward declaration ethercat state string
extern const string module_ethercat::state_strings[];

void convert_string_to_hex(string input, char **output, size_t *outlen) {
    size_t len = input.length();
    *output = new char[len/2];
    unsigned int tmp;

    for (size_t i = 0; i < len/2; ++i) {
        string sub = input.substr(i*2, 2);
        sscanf(sub.c_str(), "%x", &tmp);
        (*output)[i] = tmp; 
    }

    *outlen = len/2;
}

//! construction
/*!
 * \param node yaml intialization node
 */
slave::coe_init_cmd::coe_init_cmd(const YAML::Node& node) {
    index      = get_as<int>(node, "index");
    subindex   = get_as<int>(node, "subindex", 0);
    ca         = get_as<int>(node, "ca", 0);
    transition = (transition_t)get_as<int>(node, "transition", 0x24);
    data       = NULL;
    value      = "";

    if (node["data"]) 
        convert_string_to_hex(get_as<string>(node, "data"), &data, &datalen);
    if (node["value"])
        value = get_as<string>(node, "value");
}

//! destruction
slave::coe_init_cmd::~coe_init_cmd() {
    if (data) {
        delete[] data;
    }
}

//! construction
/*!
 * \param node yaml intialization node
 */
slave::soe_init_cmd::soe_init_cmd(const YAML::Node& node) {
    idn        = get_as<int>(node, "idn");
    element    = get_as<int>(node, "element") >> 1;
    atn        = get_as<int>(node, "atn");
    transition = (transition_t)get_as<int>(node, "transition");
    string data_string = get_as<string>(node, "data");
    convert_string_to_hex(data_string, &data, &datalen);
}

//! destruction
slave::soe_init_cmd::~soe_init_cmd() {
    if (data) {
        delete[] data;
    }
}

//! default construction
slave::slave_dc::slave_dc() {
    has_dc          = false;

    type            = 0;
    cycle_time_0    = 0;
    cycle_time_1    = 0;
    cycle_shift     = 0;
}

//! construction
/*!
 * \param node yaml intialization node
 */
slave::slave_dc::slave_dc(const YAML::Node& node) {
    has_dc          = true;
    
    type            = get_as<int     >(node, "type");
    cycle_time_0    = get_as<uint32_t>(node, "cycle_time_0", 0);
    cycle_time_1    = get_as<uint32_t>(node, "cycle_time_1", 0);
    cycle_shift     = get_as<uint32_t>(node, "cycle_shift", 0);
}

//! construction
/*!
 * \param node yaml intialization node
 */
slave::sync_manager_settings::sync_manager_settings(const YAML::Node& node) {
    _address = get_as<int     >(node, "address");
    _flags   = get_as<unsigned>(node, "flags");
    _length  = get_as<unsigned>(node, "length");
}

//! construction
/*!
 * \param index slave index
 * \param master_dev master device
 */
slave::slave(int index, master *master_dev) 
    : index(index), master_dev(master_dev) {
                
    master_dev->log(verbose, "default slave index %d created\n", index);
    _init();
};

//! construction
/*!
 * \param node yaml intialization node
 * \param master_dev master device
 */
slave::slave(const YAML::Node& node, master *master_dev)
    :   master_dev(master_dev) {
    name  = get_as<string>(node, "name");
    index = get_as<int>(node, "index");

    // sync manager settings
    if (node["sm"]) {
        master_dev->log(verbose,
                "slave %s parsing sm settings\n", name.c_str());

        for (YAML::const_iterator it = node["sm"].begin();
                it != node["sm"].end(); ++it) {
        
            int sm_nr = it->first.as<int>();
            _sm_map[sm_nr] = new sync_manager_settings(it->second);
        }
    }
    
    if (node["dc"])
        dc = slave_dc(node["dc"]);

    if (node["init_cmds"]) {
        master_dev->log(verbose,
                "slave %s parsing init commands\n", name.c_str());

        // parsing slave configurations
        for (YAML::const_iterator it = node["init_cmds"].begin();
                it != node["init_cmds"].end(); ++it) {
            string type = get_as<string>(*it, "type");

            if (type == "coe")
                coe_init_cmds.push_back(new coe_init_cmd_t(*it));
            else if (type == "soe")
                soe_init_cmds.push_back(new soe_init_cmd_t(*it));
        }
    }

    if (node["mapping"]) {
        const YAML::Node& mapping_node = node["mapping"];
        string type = get_as<string>(mapping_node, "type");

        master_dev->log(verbose, 
                "slave %s parsing mapping type %s\n", 
                name.c_str(), type.c_str());

        if (type == "coe") {
            const YAML::Node& pdos_node = mapping_node["pdos"];

            for (YAML::const_iterator it = pdos_node.begin(); 
                    it != pdos_node.end(); ++it) {
                int value = it->as<int>();
                master_dev->log(verbose, 
                        "slave %s got mapping value 0x%X\n",
                        name.c_str(), value);
        
                if ((value & 0x1A00) == 0x1A00)
                    input_mapping.push_back(value);    //! process data input mapping values
                else if ((value & 0x1600) == 0x1600)
                    output_mapping.push_back(value);   //! process data output mapping values
            }
        }
    }

//    _init();

    master_dev->log(verbose,
            "slave %s index %d created\n", name.c_str(), index);
}

//! destruction
slave::~slave() {
    for (coe_list_t::iterator it = coe_init_cmds.begin();
            it != coe_init_cmds.end(); ++it)
        delete(*it);
    
    for (soe_list_t::iterator it = soe_init_cmds.begin();
            it != soe_init_cmds.end(); ++it)
        delete(*it);
    
}
        
//! initialize common stuff
void slave::_init() {
    kernel& k = *kernel::get_instance();

    if (!_eeprom_mi) 
        _eeprom_mi = make_shared<slave::memory_inspection>(
                shared_from_this(), request_type_eeprom);
    
    if (!_memory_mi) 
        _memory_mi = make_shared<slave::memory_inspection>(
                shared_from_this(), request_type_memory);
    
    k.add_service_requester(_eeprom_mi);
    k.add_service_requester(_memory_mi);

//    k.add_service_requester("memory_inspection", 
//            master_dev->name, format_string("slave_%d.memory", index), index);
//    k.add_service_requester("memory_inspection", 
//            master_dev->name, format_string("slave_%d.eeprom", index), 
//            index | ECAT_SLAVE_ID_EEPROM);

//    if (k.clnt) {
//        stringstream base;
//        base << k.clnt->name << "." << master_dev->name <<
//            ".slave_" << index;
//
//        register_set_ec_state(k.clnt, base.str() + ".set_ec_state");
//        register_get_ec_state(k.clnt, base.str() + ".get_ec_state");
//    }
}

//! prepare state transitions
/*!
 * \param ctx ethercat master device
 * \param transition state transition
 */
bool slave::prepare_state_transition(transition_t transition) {
    int state_from = (transition & 0xF0) >> 4,
        state_to   =  transition & 0x0F;

    if (state_to == 4) {
        // configure distributed clocks if needed 
        if (master_dev->_pec->dc.have_dc && dc.has_dc) {
            if (dc.cycle_time_0 == 0)
                dc.cycle_time_0 = master_dev->_pec->dc.timer_override; 

            if (dc.type == 1) {
                if (dc.cycle_time_1 == 0)
                    dc.cycle_time_1 = master_dev->_pec->dc.timer_override; 

                master_dev->log(verbose, "slave %2d configuring dc sync 01, "
                        "cycle_times %d/%d, cycle_shift %d\n",
                        index, dc.cycle_time_0, dc.cycle_time_1, dc.cycle_shift);

                ec_dc_sync01(master_dev->_pec, index, 1, dc.cycle_time_0, dc.cycle_time_1, dc.cycle_shift);
            } else {
                master_dev->log(verbose, "slave %2d configuring dc sync 0, "
                        "cycle_time %d, cycle_shift %d\n",
                        index, dc.cycle_time_0, dc.cycle_shift);

                ec_dc_sync0(master_dev->_pec, index, 1, dc.cycle_time_0, dc.cycle_shift);
            }
        } else
            ec_dc_sync0(master_dev->_pec, index, 0, 0, 0);
    }

    master_dev->log(verbose,
            "slave %2d prepare state transition from 0x%x/%s to 0x%x/%s\n",
            index, state_from, state_strings[state_from].c_str(),
            state_to, state_strings[state_to].c_str());

    for (coe_list_t::iterator it = coe_init_cmds.begin();
            it != coe_init_cmds.end(); ++it) {

        coe_init_cmd_t *cmd = *it;

        if (cmd->transition == transition) {
            master_dev->log(verbose, "sending coe init "
                    "command slave %d, index %X\n", index, cmd->index);

            if (cmd->data) {
                uint8_t *buf = (uint8_t *)cmd->data;
                size_t buf_len = cmd->datalen;
                uint32_t abort_code = 0;

                int ret = ec_coe_sdo_write(master_dev->_pec, index, cmd->index, 
                        cmd->subindex, cmd->ca, buf, buf_len, &abort_code);
                if (ret != 0) {
                    master_dev->log(info, "writing sdo, %s\n",
                            "todo");//ecx_elist2string(ctx));
                }
            } else {
                // get description
                ec_coe_sdo_entry_desc_t entry_desc;
                entry_desc.data = NULL;
                int ret2 = ec_coe_sdo_entry_desc_read(master_dev->_pec, index, 
                        cmd->index, cmd->subindex, 0x7F, &entry_desc);

                master_dev->log(warning, "desc returned %d, data_len %d\n", ret2, entry_desc.data_len);
                if (ret2 > 0) {
                    py_value    *pval       = eval_full(cmd->value);
                    py_int      *pintval    = dynamic_cast<py_int *>(pval);
                    py_long     *plongval   = dynamic_cast<py_long *>(pval);
                    py_float    *pfloatval  = dynamic_cast<py_float *>(pval);
                    py_special  *pspval     = dynamic_cast<py_special *>(pval);
                 
                    size_t data_len = (entry_desc.bit_length+7)/8;
                    uint8_t *data = new uint8_t[data_len];

                    switch (entry_desc.data_type) {
                        case ECT_BOOLEAN:
                            if (!pspval)
                                break;

                            (*(uint8_t *)data) = (bool)*pspval;
                            break;
                        case ECT_INTEGER8:
                            if (!pintval) 
                                break;

                            (*(int8_t *)data) = (int)*pintval;
                            break;
                        case ECT_INTEGER16:
                            if (!pintval)
                                break;

                            (*(int16_t *)data) = (int)*pintval;
                            break;
                        case ECT_INTEGER32:
                        case ECT_INTEGER24:
                            if (!pintval)
                                break;

                            (*(int32_t *)data) = (int)*pintval;
                            break;
                        case ECT_INTEGER64: {
                            if (!pintval)
                                break;

                            if (plongval) 
                                (*(int64_t *)data) = (int64_t)*plongval;
                            else
                                (*(int64_t *)data) = (int)*pintval;
                            break;
                        }
                        case ECT_UNSIGNED8:
                            if (!pintval)
                                break;

                            (*(uint8_t *)data) = (unsigned int)*pintval;
                            break;
                        case ECT_UNSIGNED16:
                            master_dev->log(warning, "this case unsigned 16\n");
                            if (!pintval)
                                break;

                            (*(uint16_t *)data) = (unsigned int)*pintval;
                            break;
                        case ECT_UNSIGNED32:
                        case ECT_UNSIGNED24:
                            if (!pintval)
                                break;

                            (*(uint32_t *)data) = (unsigned int)*pintval;
                            break;
                        case ECT_UNSIGNED64: {
                            if (!pintval)
                                break;

                            if (plongval) 
                                (*(uint64_t *)data) = (int64_t)*plongval;
                            else
                                (*(uint64_t *)data) = (unsigned int)*pintval;
                            break;
                        }
                        case ECT_REAL32:
                            if (pfloatval)
                                (*(float *)data) = (float)*pfloatval;
                            else if (pintval)
                                (*(float *)data) = (float)*pintval;
                            else if (plongval)
                                (*(float *)data) = (float)*plongval;
                            break;
                        case ECT_REAL64:
                            if (pfloatval) 
                                (*(float *)data) = (float)*pfloatval;
                            else if (pintval)
                                (*(float *)data) = (float)*pintval;
                            else if (plongval)
                                (*(float *)data) = (float)*plongval;
                            break;
                        case ECT_BIT1:
                        case ECT_BIT2:
                        case ECT_BIT3:
                        case ECT_BIT4:
                        case ECT_BIT5:
                        case ECT_BIT6:
                        case ECT_BIT7:
                        case ECT_BIT8:
                        case ECT_VISIBLE_STRING:
                            break;
                        case ECT_OCTET_STRING: {
                            py_list *plist  = dynamic_cast<py_list *>(pval);
                            if (!plist)
                                break;

                            int num = 0;
                            for (py_list_value_t::iterator it = plist->value.begin();
                                    it != plist->value.end(); ++it) {
                                //py_long *plongval2     = dynamic_cast<py_long *>(*it);
                                py_int *pintval2     = dynamic_cast<py_int *>(*it);
                                data[num++] = (int)*pintval2;
                            }
                            break;
                        }
                        default:
                            break;
                    }

                    printf("data: ");
                    for (unsigned int q = 0; q < data_len; ++q)
                        printf("%02X ", data[q]);
                    printf("\n");
                    uint32_t abort_code = 0;
                    int ret = ec_coe_sdo_write(master_dev->_pec, index, cmd->index, 
                            cmd->subindex, cmd->ca, data, data_len, &abort_code);
                    if (ret != 0) {
                        master_dev->log(info, "writing sdo, %s\n",
                                "todo");//ecx_elist2string(ctx));
                    }

                    if (pval)
                        delete pval;
                    if (data)
                        delete data;
                }
            }
        } 
    }
    
    for (soe_list_t::iterator it = soe_init_cmds.begin();
            it != soe_init_cmds.end(); ++it) {

        soe_init_cmd_t *cmd = *it;

        if (cmd->transition == transition) {
            master_dev->log(verbose, "sending soe init "
                    "command slave %d, idn %d, atn %d\n", index, cmd->idn, cmd->atn);

            int wkc = ec_soe_write(master_dev->_pec, index, cmd->atn, cmd->idn, 
                    cmd->element, (uint8_t *)cmd->data, cmd->datalen);
            if (!wkc) {
                master_dev->log(info, "writing sdo, %s\n",
                     "todo");//ecx_elist2string(ctx));
            }
        } 
    }

    return true;
}

//! register interfaces for slave
/*!
 * \param ctx ethercat context
 * \return N/A
 */
void slave::register_interfaces(module_state_t state) {
    uint32_t mbx_sup = master_dev->_pec->slaves[index].eeprom.mbx_supported;
    //uint32_t soe_ch  = master_dev->_pec->slaves[index].eeprom.general.soe_channels;
    kernel& k = *kernel::get_instance();

    switch (state) {
        case module_state_boot: 
//            if (mbx_sup & EC_EEPROM_MBX_FOE) {
//                k.add_service_requester("file_protocol", master_dev->name,
//                        format_string("slave_%d.mailbox", index), index);
//            }
//
//            k.remove_service_requester("process_data_inspection", 
//                    master_dev->name, index);
//            k.remove_service_requester("canopen_protocol", 
//                    master_dev->name, index);
//            k.remove_service_requester("canopen_protocol", 
//                    master_dev->name, index | ECAT_SLAVE_ID_EEPROM);
//
//            for (unsigned atn = 0; atn < soe_ch; ++atn) {
//                k.remove_service_requester("sercos_protocol", 
//                        master_dev->name, ECAT_SLAVE_ID_SUB | (atn << 16) | index);
//                k.remove_service_requester("process_data_inspection", 
//                        master_dev->name, ECAT_SLAVE_ID_SUB | (atn << 16) | index);
//            }
            break;
        case module_state_init:
#define REMOVE_SERVICE_REQUESTER(req) \
            { if (req) { k.remove_service_requester(req); (req).reset(); } }

#define ADD_SERVICE_REQUESTER(req, cls, type) \
            { if (!(req)) { (req) = make_shared<cls>(shared_from_this(), (type)); \
                k.add_service_requester(req); } }

            ADD_SERVICE_REQUESTER(_eeprom_mi, slave::memory_inspection, request_type_eeprom);
            ADD_SERVICE_REQUESTER(_memory_mi, slave::memory_inspection, request_type_memory);
            REMOVE_SERVICE_REQUESTER(_mbx_coe);
            REMOVE_SERVICE_REQUESTER(_eeprom_coe);

//            k.remove_service_requester("process_data_inspection", 
//                    master_dev->name, index);
//            k.remove_service_requester("file_protocol", 
//                    master_dev->name, index);
//
//            if (mbx_sup & EC_EEPROM_MBX_SOE) {
//                for (unsigned atn = 0; atn < soe_ch; ++atn) {
//                    k.remove_service_requester("sercos_protocol", 
//                            master_dev->name, ECAT_SLAVE_ID_SUB | (atn << 16) | index);
//                    k.remove_service_requester("process_data_inspection", 
//                            master_dev->name, ECAT_SLAVE_ID_SUB | (atn << 16) | index);
//                }
//            }
            break;
        case module_state_preop: {
//            k.remove_service_requester("process_data_inspection", 
//                    master_dev->name, index);
//            
//            if (mbx_sup & EC_EEPROM_MBX_FOE) {
//                k.add_service_requester("file_protocol", master_dev->name,
//                    format_string("slave_%d.mailbox", index), index);
//            }
            
            if (mbx_sup & EC_EEPROM_MBX_COE)
                ADD_SERVICE_REQUESTER(_mbx_coe, slave::canopen, request_type_mailbox);

            ADD_SERVICE_REQUESTER(_eeprom_coe, slave::canopen, request_type_eeprom);
            
//            if (mbx_sup & EC_EEPROM_MBX_SOE) {
//                for (unsigned atn = 0; atn < soe_ch; ++atn) {
//                    k.add_service_requester("sercos_protocol", 
//                            master_dev->name, 
//                            format_string("slave_%d.mailbox.atn_%d", index, atn),
//                            ECAT_SLAVE_ID_SUB | (atn << 16) | index);
//                    k.remove_service_requester("process_data_inspection", 
//                            master_dev->name, ECAT_SLAVE_ID_SUB | (atn << 16) | index);
//                }
//            }
            break;
        }
        case module_state_op:
        case module_state_safeop: {
//            k.add_service_requester("process_data_inspection", 
//                    master_dev->name, format_string("slave_%d", index), index);
//            
//            if (mbx_sup & EC_EEPROM_MBX_SOE) {
//                for (unsigned atn = 0; atn < soe_ch; ++atn) {
//                    k.add_service_requester("process_data_inspection", 
//                            master_dev->name, 
//                            format_string("slave_%d.mailbox.atn_%d", index, atn),
//                            ECAT_SLAVE_ID_SUB | (atn << 16) | index);
//                }
//            }
            break;
        }
        default:
            break;
    }
}


slave::sercos::sercos(std::shared_ptr<slave> slv, int atn)
:   service_provider::sercos_protocol::base(slv->master_dev->name, 
        format_string("slave_%d.atn_%d", slv->index, atn)), slv(slv), atn(atn) {
}

//! read sercos id number
/*!
 * \param idn id number to read
 * \param elements elements to read
 * \param data data to read
 */
void slave::sercos::sercos_read_idn(const uint16_t& idn, 
        const service_provider::sercos_protocol::sercos_service_elements_t& elements, 
        service_provider::sercos_protocol::service_data_t& data) {
    uint8_t *buf = NULL; 
    size_t buf_len = 0;
    int ret;
        
    if ((ret = ec_soe_read(slv->master_dev->_pec, slv->index, atn, idn,
                elements >> 1, buf, &buf_len)) != 0) {
        throw str_exception("slave %2d: reading sercos atn %d idn 0x%X "
                "elements 0x%X returned errorcode 0x%X!\n", slv->index, 
                atn, idn, elements, ret);
    }

    
    // todo decode answer

    free(buf);
}

//! write sercos id number
/*!
 * \param idn id number to write
 * \param elements elements to write
 * \param data data to write
 */
void slave::sercos::sercos_write_idn(const uint16_t& idn, 
        const service_provider::sercos_protocol::sercos_service_elements_t& elements, 
        service_provider::sercos_protocol::service_data_t& data) {
        // todo implement
}


//int slave::on_set_ec_state(ln::service_request& req, ln_service_module_ethercat_set_ec_state& svc) {
//    string state_to = string(svc.req.state, svc.req.state_len);
//
//    if (state_to == string("init"))
//        ec_slave_set_state(master_dev->_pec, index, EC_STATE_INIT);
//    else if (state_to == "preop")    
//        ec_slave_set_state(master_dev->_pec, index, EC_STATE_PREOP);
//    else if (state_to == "safeop")    
//        ec_slave_set_state(master_dev->_pec, index, EC_STATE_SAFEOP);
//    else if (state_to == "op")    
//        ec_slave_set_state(master_dev->_pec, index, EC_STATE_OP);
//
//    req.respond();
//    return 0;
//}
//
//int slave::on_get_ec_state(ln::service_request& req, ln_service_module_ethercat_get_ec_state& svc) {
//    ec_state_t state;
//    string state_string;
//    int wkc = ec_slave_get_state(master_dev->_pec,
//            index, &state, NULL);
//
//    if (wkc > 0) {
//        if ((state & 0x000F) == EC_STATE_INIT)
//            state_string = strdup("init");
//        else if ((state & 0x000F) == EC_STATE_PREOP)
//            state_string = strdup("preop");
//        else if ((state & 0x000F) == EC_STATE_SAFEOP)
//            state_string = strdup("safeop");
//        else if ((state & 0x000F) == EC_STATE_OP)
//            state_string = strdup("op");
//        else 
//            state_string = strdup("unknown");
//
//        if ((state & 0x0010) == 0x0010)
//            state_string += " ERROR";
//    } else
//        state_string = "ERROR got no answer on get_state command\n";
//
//    svc.resp.state = strdup(state_string.c_str());
//    svc.resp.state_len = strlen(svc.resp.state);
//
//    req.respond();
//    free(svc.resp.state);
//    return 0;
//}
	    
