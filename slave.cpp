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
#include <iomanip>
#include <stdio.h>

using namespace std;
using namespace robotkernel;
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
    convert_string_to_hex(node["data"].to<string>(), &data, &datalen);
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
    has_dc = false;

    type = 0;
    cycle_time_0 = 0;
    cycle_time_1 = 0;
    cycle_shift  = 0;
}

//! construction
/*!
 * \param node yaml intialization node
 */
slave::slave_dc::slave_dc(const YAML::Node& node) {
    has_dc = true;
    
    type             = node["type"].to<int>();
    cycle_time_0     = node["cycle_time_0"].to<uint32_t>();
    if (type == 1)
        cycle_time_1 = node["cycle_time_1"].to<uint32_t>();
    cycle_shift      = node["cycle_shift"].to<uint32_t>();
}

//! construction
/*!
 * \param node yaml intialization node
 */
slave::sync_manager_settings::sync_manager_settings(const YAML::Node& node) {
    _address = node["address"].to<int>();
    _flags   = node["flags"].to<unsigned>();
    _length  = node["length"].to<unsigned>();
}

//! construction
/*!
 * \param index slave index
 * \param master_dev master device
 */
slave::slave(int index, master *master_dev) 
    : index(index),
    master_dev(master_dev) {
                
    master_dev->log(module_verbose, "default slave index %d created\n", index);
};

//! construction
/*!
 * \param node yaml intialization node
 * \param master_dev master device
 */
slave::slave(const YAML::Node& node, master *master_dev)
    : master_dev(master_dev) {
    name  = node["name"].to<string>();
    index = node["index"].to<int>();

    // sync manager settings
    const YAML::Node *sm_node = node.FindValue("sm");
    if (sm_node) {
        master_dev->log(module_verbose,
                "slave %s parsing sm settings\n", name.c_str());

        for (YAML::Iterator it = sm_node->begin();
                it != sm_node->end(); ++it) {
        
            int sm_nr = it.first().to<int>();
            _sm_map[sm_nr] = new sync_manager_settings(it.second());
        }
    }
    
    if (node.FindValue("dc") != NULL) {
        dc = slave_dc(node["dc"]);
    }

    if (node.FindValue("init_cmds") != NULL) {
        master_dev->log(module_verbose,
                "slave %s parsing init commands\n", name.c_str());

        // parsing slave configurations
        const YAML::Node& init_cmds = node["init_cmds"];
        for (YAML::Iterator it = init_cmds.begin();
                it != init_cmds.end(); ++it) {
            string type = (*it)["type"].to<string>();

            if (type == "coe")
                coe_init_cmds.push_back(new coe_init_cmd_t(*it));
            else if (type == "soe")
                soe_init_cmds.push_back(new soe_init_cmd_t(*it));
        }
    }
	
    kernel& k = *kernel::get_instance();
    if (k.clnt) {
        stringstream base;
        base << k.clnt->name << "." << master_dev->name <<
            ".slave_" << index;

        register_set_ec_state(k.clnt, base.str() + ".set_ec_state");
        register_get_ec_state(k.clnt, base.str() + ".get_ec_state");
    }

    master_dev->log(module_verbose,
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

//! prepare state transitions
/*!
 * \param ctx ethercat master device
 * \param transition state transition
 */
bool slave::prepare_state_transition(transition_t transition) {
    int state_from = (transition & 0xF0) >> 4,
        state_to = transition & 0x0F;

    if (state_to == 4) {
        // configure distributed clocks if needed 
        if (dc.has_dc) {
            if (dc.type == 1) {
                master_dev->log(module_verbose, "slave %2d configuring dc sync 01, "
                        "cycle_times %d/%d, cycle_shift %d\n",
                        index, dc.cycle_time_0, dc.cycle_time_1, dc.cycle_shift);

                ec_dc_sync01(master_dev->_pec, index, 1, dc.cycle_time_0, dc.cycle_time_1, dc.cycle_shift);
            } else {
                master_dev->log(module_verbose, "slave %2d configuring dc sync 0, "
                        "cycle_time %d, cycle_shift %d\n",
                        index, dc.cycle_time_0, dc.cycle_shift);

                ec_dc_sync0(master_dev->_pec, index, 1, dc.cycle_time_0, dc.cycle_shift);
            }
        } else
            ec_dc_sync0(master_dev->_pec, index, 0, 0, 0);
    }

    master_dev->log(module_verbose,
            "slave %2d prepare state transition from 0x%x/%s to 0x%x/%s\n",
            index, state_from, state_strings[state_from].c_str(),
            state_to, state_strings[state_to].c_str());

    for (coe_list_t::iterator it = coe_init_cmds.begin();
            it != coe_init_cmds.end(); ++it) {

        coe_init_cmd_t *cmd = *it;

        if (cmd->transition == transition) {
            master_dev->log(module_verbose, "sending coe init "
                    "command slave %d, index %X\n", index, cmd->index);

            uint8_t *buf = (uint8_t *)cmd->data;
            size_t buf_len = cmd->datalen;
            uint32_t abort_code = 0;

            int wkc = ec_coe_sdo_write(master_dev->_pec, index, cmd->index, 
                    cmd->subindex, cmd->ca, buf, &buf_len, &abort_code);
            if (!wkc) {
                master_dev->log(module_info, "writing sdo, %s\n",
                     "todo");//ecx_elist2string(ctx));
            }
        } 
    }
    
    for (soe_list_t::iterator it = soe_init_cmds.begin();
            it != soe_init_cmds.end(); ++it) {

        soe_init_cmd_t *cmd = *it;

        if (cmd->transition == transition) {
            master_dev->log(module_verbose, "sending soe init "
                    "command slave %d, idn %d, atn %d\n", index, cmd->idn, cmd->atn);

            int wkc = ec_soe_write(master_dev->_pec, index, cmd->atn, cmd->idn, 
                    cmd->element, (uint8_t *)cmd->data, cmd->datalen/2);
            if (!wkc) {
                master_dev->log(module_info, "writing sdo, %s\n",
                     "todo");//ecx_elist2string(ctx));
            }
        } 
    }

    return true;
}
        
//! perform memory request
/*!
 * \param code request code
 * \param memreq memory request structure
 *               address in range 0x00000000 - 0x0000FFFF slave memory
 *                       above    0x00010000              eeprom memory
 */
void slave::memory_request(int code, memory_t *memreq) {
    master_dev->log(module_verbose, "slave %d: incoming memory request\n", index);
    
    switch (code) {
        case MOD_REQUEST_MEMORY_READ: {
            uint16_t address = MEM_ADDRESS(memreq->address);

            switch (memreq->address & MEM_TYPE_MASK) {
                case MEM_TYPE_SLAVE_EEPROM:
                    master_dev->log(module_verbose, "slave %d: reading eeprom address 0x%X\n", 
                            index, address);
                    ec_eepromread_len(master_dev->_pec, index, address, memreq->data, memreq->length);
                    break;
                case MEM_TYPE_SLAVE_MEM:
                    {
                        uint16_t wkc;
                        master_dev->log(module_verbose, "slave %d: reading esc memory address 0x%X\n", 
                                index, address);

                        for (unsigned offset = 0; offset < memreq->length; offset+=100) {
                            uint32_t act_len = min(100, memreq->length - offset);

                            ec_fprd(master_dev->_pec, master_dev->_pec->slaves[index].fixed_address, 
                                    address + offset, memreq->data + offset, act_len, &wkc);
                        }
                    }
                    break;
            }
            break;
        }
        case MOD_REQUEST_MEMORY_WRITE: {
            break;
        }
        case MOD_REQUEST_MEMORY_GET_INFO: {
            break;
        }
    }
}

//! register interfaces for slave
/*!
 * \param ctx ethercat context
 * \return N/A
 */
void slave::register_interfaces() {
    std::stringstream slave_name; 
    slave_name << "slave_" << index;

    ifaces.push_back(robotkernel::kernel::register_interface_cb(master_dev->name.c_str(), 
            "libinterface_canopen_protocol.so", slave_name.str().c_str(), index));
    ifaces.push_back(robotkernel::kernel::register_interface_cb(master_dev->name.c_str(), 
            "libinterface_process_data_inspection.so", slave_name.str().c_str(), index));
    ifaces.push_back(robotkernel::kernel::register_interface_cb(master_dev->name.c_str(),
            "libinterface_memory_inspection.so", slave_name.str().c_str(), index));
    
    if (master_dev->_pec->slaves[index].eeprom.mbx_supported & EC_EEPROM_MBX_SOE) {
        int atn;
        for (atn = 0; atn < master_dev->_pec->slaves[index].eeprom.general.soe_channels; ++atn) {
            std::stringstream atn_name;
            atn_name << "slave_" << index << ".atn_" << atn;
            ifaces.push_back(robotkernel::kernel::register_interface_cb(master_dev->name.c_str(), 
                "libinterface_sercos_protocol.so", atn_name.str().c_str(), (index << 16) | atn));
        }
    }
}

//! unregister interfaces of slave
/*!
 * \return N/A
 */
void slave::unregister_interfaces() {
    for (iface_list_t::iterator it = ifaces.begin(); it != ifaces.end(); ++it)
        kernel::unregister_interface_cb(*it);
}

int slave::on_set_ec_state(ln::service_request& req, ln_service_module_ethercat_set_ec_state& svc) {
    string state_to = string(svc.req.state, svc.req.state_len);

    if (state_to == string("init"))
        ec_slave_set_state(master_dev->_pec, index, EC_STATE_INIT);
    else if (state_to == "preop")    
        ec_slave_set_state(master_dev->_pec, index, EC_STATE_PREOP);
    else if (state_to == "safeop")    
        ec_slave_set_state(master_dev->_pec, index, EC_STATE_SAFEOP);
    else if (state_to == "op")    
        ec_slave_set_state(master_dev->_pec, index, EC_STATE_OP);

    req.respond();
    return 0;
}

int slave::on_get_ec_state(ln::service_request& req, ln_service_module_ethercat_get_ec_state& svc) {
    ec_state_t state;
    string state_string;
    int wkc = ec_slave_get_state(master_dev->_pec,
            index, &state);

    if (wkc > 0) {
        if ((state & 0x000F) == EC_STATE_INIT)
            state_string = strdup("init");
        else if ((state & 0x000F) == EC_STATE_PREOP)
            state_string = strdup("preop");
        else if ((state & 0x000F) == EC_STATE_SAFEOP)
            state_string = strdup("safeop");
        else if ((state & 0x000F) == EC_STATE_OP)
            state_string = strdup("op");
        else 
            state_string = strdup("unknown");

        if ((state & 0x0010) == 0x0010)
            state_string += " ERROR";
    } else
        state_string = "ERROR got no answer on get_state command\n";

    svc.resp.state = strdup(state_string.c_str());
    svc.resp.state_len = strlen(svc.resp.state);

    req.respond();
    free(svc.resp.state);
    return 0;
}

