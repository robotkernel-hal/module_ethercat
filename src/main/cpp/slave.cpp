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
    
    already_added = false;
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
    
    already_added = false;
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
    : service_provider::process_data_inspection::base(master_dev->name, 
        format_string("slave_%d", index)),
    index(index), master_dev(master_dev) {
                
    master_dev->log(verbose, "default slave index %d created\n", index);
};

//! construction
/*!
 * \param node yaml intialization node
 * \param master_dev master device
 */
slave::slave(const YAML::Node& node, master *master_dev)
    : service_provider::process_data_inspection::base(master_dev->name, 
        format_string("slave_%d", get_as<int>(node, "index"))), master_dev(master_dev) {
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
 
// perform robotkernel clean up
void slave::clean_up() {
    kernel& k = *kernel::get_instance();
    k.remove_device(_eeprom_mi);
    k.remove_device(_memory_mi);
    k.remove_device(_eeprom_coe);

    _eeprom_mi = nullptr;
    _memory_mi = nullptr;
    _eeprom_coe = nullptr;
}

//! sending slave init commands
/*!
 */
void slave::add_init_cmds() {
    for (coe_list_t::iterator it = coe_init_cmds.begin();
            it != coe_init_cmds.end(); ++it) {

        coe_init_cmd_t *cmd = *it;

        if (cmd->already_added)
            continue;

        if (!cmd->data) {
            // get description
            ec_coe_sdo_entry_desc_t entry_desc;
            entry_desc.data = NULL;
            int ret2 = ec_coe_sdo_entry_desc_read(master_dev->pec, index, 
                    cmd->index, cmd->subindex, 0x7F, &entry_desc);

            if (ret2 > 0) {
                py_value    *pval       = eval_full(cmd->value);
                py_int      *pintval    = dynamic_cast<py_int *>(pval);
                py_long     *plongval   = dynamic_cast<py_long *>(pval);
                py_float    *pfloatval  = dynamic_cast<py_float *>(pval);
                py_special  *pspval     = dynamic_cast<py_special *>(pval);

                cmd->datalen = (entry_desc.bit_length+7)/8;
                cmd->data = new char[cmd->datalen];

                switch (entry_desc.data_type) {
                    case ECT_BOOLEAN:
                        if (!pspval)
                            break;

                        (*(uint8_t *)cmd->data) = (bool)*pspval;
                        break;
                    case ECT_INTEGER8:
                        if (!pintval) 
                            break;

                        (*(int8_t *)cmd->data) = (int)*pintval;
                        break;
                    case ECT_INTEGER16:
                        if (!pintval)
                            break;

                        (*(int16_t *)cmd->data) = (int)*pintval;
                        break;
                    case ECT_INTEGER32:
                    case ECT_INTEGER24:
                        if (!pintval)
                            break;

                        (*(int32_t *)cmd->data) = (int)*pintval;
                        break;
                    case ECT_INTEGER64: {
                        if (!pintval)
                            break;

                        if (plongval) 
                            (*(int64_t *)cmd->data) = (int64_t)*plongval;
                        else
                            (*(int64_t *)cmd->data) = (int)*pintval;
                        break;
                    }
                    case ECT_UNSIGNED8:
                        if (!pintval)
                            break;

                        (*(uint8_t *)cmd->data) = (unsigned int)*pintval;
                        break;
                    case ECT_UNSIGNED16:
                        master_dev->log(warning, "this case unsigned 16\n");
                        if (!pintval)
                            break;

                        (*(uint16_t *)cmd->data) = (unsigned int)*pintval;
                        break;
                    case ECT_UNSIGNED32:
                    case ECT_UNSIGNED24:
                        if (!pintval)
                            break;

                        (*(uint32_t *)cmd->data) = (unsigned int)*pintval;
                        break;
                    case ECT_UNSIGNED64: {
                        if (!pintval)
                            break;

                        if (plongval) 
                            (*(uint64_t *)cmd->data) = (int64_t)*plongval;
                        else
                            (*(uint64_t *)cmd->data) = (unsigned int)*pintval;
                        break;
                    }
                    case ECT_REAL32:
                        if (pfloatval)
                            (*(float *)cmd->data) = (float)*pfloatval;
                        else if (pintval)
                            (*(float *)cmd->data) = (float)*pintval;
                        else if (plongval)
                            (*(float *)cmd->data) = (float)*plongval;
                        break;
                    case ECT_REAL64:
                        if (pfloatval) 
                            (*(float *)cmd->data) = (float)*pfloatval;
                        else if (pintval)
                            (*(float *)cmd->data) = (float)*pintval;
                        else if (plongval)
                            (*(float *)cmd->data) = (float)*plongval;
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
                            cmd->data[num++] = (int)*pintval2;
                        }
                        break;
                    }
                    default:
                        break;
                }

                if (pval)
                    delete pval;
            }
        }
        
        if (cmd->data) {
            ec_slave_add_init_cmd(master_dev->pec, index, EC_MBX_COE, 
                    (int)cmd->transition, cmd->index, cmd->subindex, 
                    cmd->ca, cmd->data, cmd->datalen);

            cmd->already_added = true;
        }
    } 

    for (soe_list_t::iterator it = soe_init_cmds.begin();
            it != soe_init_cmds.end(); ++it) {

        soe_init_cmd_t *cmd = *it;

        if (cmd->already_added)
            continue;

        ec_slave_add_init_cmd(master_dev->pec, index, EC_MBX_SOE, 
                (int)cmd->transition, cmd->idn, cmd->element, 
                cmd->atn, cmd->data, cmd->datalen);

        cmd->already_added = true;
    }
}

//! prepare state transitions
/*!
 * \param from state coming from
 * \param to state switching to
 */
void slave::pre_state_transition(module_state_t from, module_state_t to) {
    // get transition
    uint32_t transition = GEN_STATE(from, to);

    switch (transition) {
        case op_2_safeop:
        case op_2_preop:
        case op_2_init:
        case op_2_boot:
            // ====> stop sending commands
            if (to == module_state_safeop)
                break;
        case safeop_2_preop:
        case safeop_2_init:
        case safeop_2_boot:
            // ====> stop receiving measurements
            if (to == module_state_preop)
                break;
        case preop_2_init:
        case preop_2_boot:
            // ====> deinit devices
        case init_2_init:
            // ====> re-/open ethercat device
            if (to == module_state_init)
                break;
        case init_2_boot:
            break;
        case boot_2_init:
        case boot_2_preop:
        case boot_2_safeop:
        case boot_2_op:
            // ====> re-/open ethercat device
            if (to == module_state_init)
                break;
        case init_2_op:
        case init_2_safeop:
        case init_2_preop:
        case preop_2_preop:
            // ====> initial devices            
            if (to == module_state_preop)
                break;
        case preop_2_op:
        case preop_2_safeop:
            // ====> sending init commands for safeop
            add_init_cmds();

            // ====> configure distributed clocks if needed 
            if (master_dev->pec->dc.have_dc && dc.has_dc) {
                if (dc.cycle_time_0 == 0)
                    dc.cycle_time_0 = master_dev->pec->dc.timer_override; 

                if (dc.type == 1) {
                    if (dc.cycle_time_1 == 0)
                        dc.cycle_time_1 = master_dev->pec->dc.timer_override; 

                    master_dev->log(verbose, "slave %2d configuring dc sync 01, "
                            "cycle_times %d/%d, cycle_shift %d\n",
                            index, dc.cycle_time_0, dc.cycle_time_1, dc.cycle_shift);

                    ec_dc_sync01(master_dev->pec, index, 1, dc.cycle_time_0, dc.cycle_time_1, dc.cycle_shift);
                } else {
                    master_dev->log(verbose, "slave %2d configuring dc sync 0, "
                            "cycle_time %d, cycle_shift %d\n",
                            index, dc.cycle_time_0, dc.cycle_shift);

                    ec_dc_sync0(master_dev->pec, index, 1, dc.cycle_time_0, dc.cycle_shift);
                }
            } else
                ec_dc_sync0(master_dev->pec, index, 0, 0, 0);

            if (to == module_state_safeop)
                break;
        case safeop_2_op:
        case safeop_2_safeop:
        case op_2_op:
            break;
        default:
            break;
    }
}

//! register interfaces for slave
/*!
 * \param ctx ethercat master device
 * \param transition state transition
 */
void slave::post_state_transition(module_state_t from, module_state_t to) {
    uint32_t mbx_sup = master_dev->pec->slaves[index].eeprom.mbx_supported;
    uint32_t soe_ch  = master_dev->pec->slaves[index].eeprom.general.soe_channels;
    kernel& k = *kernel::get_instance();
    auto *slv = &(master_dev->pec->slaves[index]);

#define REMOVE_SERVICE_COLLECTOR(req) \
            { if (req) { k.remove_device(req); (req) = nullptr; } }

#define ADD_SERVICE_COLLECTOR(req) { \
                k.add_device(req); }

#define ADD_SERVICE_COLLECTOR_CLASS(req, cls, ...) \
            { if (!(req)) { (req) = make_shared<cls>(shared_from_this(), ##__VA_ARGS__); \
                ADD_SERVICE_COLLECTOR(req); } }
    // get transition
    uint32_t transition = GEN_STATE(from, to);

    switch (transition) {
        case op_2_safeop:
        case op_2_preop:
        case op_2_init:
        case op_2_boot:
            // ====> stop sending commands
            if (to == module_state_safeop)
                break;
        case safeop_2_preop:
        case safeop_2_init:
        case safeop_2_boot:
            // ====> stop receiving measurements
            REMOVE_SERVICE_COLLECTOR(shared_from_this()); // process data inspection
            
            if (pdin)  { k.remove_device(pdin); pdin = nullptr; }
            if (pdout) { k.remove_device(pdout); pdout = nullptr; }

            if (to == module_state_preop)
                break;
        case preop_2_init:
        case preop_2_boot:
            // ====> deinit devices
            REMOVE_SERVICE_COLLECTOR(_mbx_foe);
            REMOVE_SERVICE_COLLECTOR(mbx_coe);

            if (mbx_sup & EC_EEPROM_MBX_SOE) {
                for (unsigned atn = 0; atn < _mbx_soe_list.size(); ++atn)
                    REMOVE_SERVICE_COLLECTOR(_mbx_soe_list[atn]);
            }
        case init_2_init:
            // ====> re-/open ethercat device
            ADD_SERVICE_COLLECTOR_CLASS(_eeprom_mi, slave::memory_inspection, request_type_eeprom);
            ADD_SERVICE_COLLECTOR_CLASS(_memory_mi, slave::memory_inspection, request_type_memory);
            ADD_SERVICE_COLLECTOR_CLASS(_eeprom_coe, slave::canopen, request_type_eeprom);

            if (to == module_state_init)
                break;
        case init_2_boot:
            ADD_SERVICE_COLLECTOR_CLASS(_mbx_foe, slave::file_protocol);
            break;
        case boot_2_init:
        case boot_2_preop:
        case boot_2_safeop:
        case boot_2_op:
            // ====> re-/open ethercat device
            REMOVE_SERVICE_COLLECTOR(_mbx_foe);

            if (to == module_state_init)
                break;
        case init_2_op:
        case init_2_safeop:
        case init_2_preop:
        case preop_2_preop:
            // ====> initial devices            
            if (mbx_sup & EC_EEPROM_MBX_FOE)
                ADD_SERVICE_COLLECTOR_CLASS(_mbx_foe, slave::file_protocol);
            
            if (mbx_sup & EC_EEPROM_MBX_COE)
                ADD_SERVICE_COLLECTOR_CLASS(mbx_coe, slave::canopen, request_type_mailbox);

            if (mbx_sup & EC_EEPROM_MBX_SOE) {
                if (_mbx_soe_list.size() != soe_ch)
                    _mbx_soe_list.resize(soe_ch);

                for (unsigned atn = 0; atn < _mbx_soe_list.size(); ++atn)
                    ADD_SERVICE_COLLECTOR_CLASS(_mbx_soe_list[atn], slave::sercos, atn);
            }

            if (to == module_state_preop)
                break;
        case preop_2_op:
        case preop_2_safeop:
            // ====> start receiving measurements
            ADD_SERVICE_COLLECTOR(shared_from_this()); // process data inspection
            
            if (mbx_sup & EC_EEPROM_MBX_SOE) {
                for (unsigned atn = 0; atn < _mbx_soe_list.size(); ++atn) {
                    // register soe process data inspection
//                    ADD_SERVICE_COLLECTOR_CLASS(_mbx_soe_list[atn], slave_servodrive, atn);
                }
            }
    
            if (slv->pdin.len) {
                if (pdin)
                    k.remove_device(pdin);
                
                string pdo_desc = mbx_coe->get_pdo_description(0x1C13);
                pdin = make_shared<robotkernel::triple_buffer>(
                        slv->pdin.len, 
                        master_dev->name, 
                        format_string("slave_%d.inputs", index), 
                        pdo_desc,
                        format_string("%s.group_%d.trigger", master_dev->name.c_str(), slv->assigned_pd_group));;
                k.add_device(pdin);
            }
            
            if (slv->pdout.len) {
                if (pdout)
                    k.remove_device(pdout);

                string pdo_desc = mbx_coe->get_pdo_description(0x1C12);
                pdout = make_shared<robotkernel::triple_buffer>(
                        slv->pdout.len, 
                        master_dev->name, format_string("slave_%d.outputs", index), pdo_desc);
                k.add_device(pdout);
            }

            if (to == module_state_safeop)
                break;
        case safeop_2_op:
        case safeop_2_safeop:
        case op_2_op:
            // ====> do nothing
            break;
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
        
    if ((ret = ec_soe_read(slv->master_dev->pec, slv->index, atn, idn,
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
	    
//! return input process data (measurements)
/*!
 * \param pd return input process data
 */
void slave::get_pdin(service_provider::process_data_inspection::pd_t& pd) {
    ec_slave_t *slv = &master_dev->pec->slaves[index];
    pd.resize(slv->pdin.len);
    memcpy(&pd[0], slv->pdin.pd, slv->pdin.len);
}

//! return output process data (commands)
/*!
 * \param pd return output process data
 */
void slave::get_pdout(service_provider::process_data_inspection::pd_t& pd) {
    ec_slave_t *slv = &master_dev->pec->slaves[index];
    pd.resize(slv->pdout.len);
    memcpy(&pd[0], slv->pdout.pd, slv->pdout.len);
}

//! process data out handler
void slave::pdout_handler() {
    ec_slave_t *slv = &master_dev->pec->slaves[index];
    if (!pdout || (slv->pdout.len == 0))
        return;

    pdout->read(0, slv->pdout.pd, slv->pdout.len);
}

//! process data in handler
void slave::pdin_handler() {
    ec_slave_t *slv = &master_dev->pec->slaves[index];
    if (!pdin || (slv->pdin.len == 0))
        return;

    pdin->write(0, slv->pdin.pd, slv->pdin.len);
    pdin->pd_cookie++;
}

