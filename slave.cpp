//! robotkernel module ethercat slave
/*!
  $Id$
 */

#include "slave.h"
#include "master.h"
#include "robotkernel/kernel.h"
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
    index      = node["index"].to<int>();
    subindex   = node["subindex"].to<int>();
    ca         = node["ca"].to<int>();
    transition = (transition_t)node["transition"].to<int>();
    convert_string_to_hex(node["data"].to<string>(), &data, &datalen);
}

//! destruction
slave::coe_init_cmd::~coe_init_cmd() {
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
                
    _pd_intf = NULL;
    _coe_intf = NULL;

    master_dev->log(module_verbose, "default slave index %d created\n", index);
};

//! construction
/*!
 * \param node yaml intialization node
 * \param master_dev master device
 */
slave::slave(const YAML::Node& node, master *master_dev)
    : master_dev(master_dev) {
    _pd_intf = NULL;
    _coe_intf = NULL;

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

            if (type == "coe") {
                coe_init_cmds.push_back(new coe_init_cmd_t(*it));
            }
        }
    }

    master_dev->log(module_verbose,
            "slave %s index %d created\n", name.c_str(), index);
}

//! destruction
slave::~slave() {
    for (coe_list_t::iterator it = coe_init_cmds.begin();
            it != coe_init_cmds.end(); ++it) {
        delete(*it);
    }
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

    _coe_intf = robotkernel::kernel::register_interface_cb(master_dev->name.c_str(), 
            "libinterface_canopen_protocol.so", slave_name.str().c_str(), index);

    if (master_dev->_pec->slaves[index].eeprom.mbx_supported & EC_EEPROM_MBX_SOE)
        _soe_intf = robotkernel::kernel::register_interface_cb(master_dev->name.c_str(), 
                "libinterface_sercos_protocol.so", slave_name.str().c_str(), index);

    _pd_intf = robotkernel::kernel::register_interface_cb(master_dev->name.c_str(), 
            "libinterface_process_data_inspection.so", slave_name.str().c_str(), index);
    _mem_intf = robotkernel::kernel::register_interface_cb(master_dev->name.c_str(),
            "libinterface_memory_inspection.so", slave_name.str().c_str(), index);
}

//! unregister interfaces of slave
/*!
 * \return N/A
 */
void slave::unregister_interfaces() {
    if (_mem_intf) {
        kernel::unregister_interface_cb(_mem_intf);
        _mem_intf = NULL;
    }

    if (_pd_intf) {
        kernel::unregister_interface_cb(_pd_intf);
        _pd_intf = NULL; 
    }
    
    if (_soe_intf) {
        kernel::unregister_interface_cb(_soe_intf);
        _soe_intf = NULL; 
    }
    
    if (_coe_intf) {
        kernel::unregister_interface_cb(_coe_intf);
        _coe_intf = NULL; 
    }
}

