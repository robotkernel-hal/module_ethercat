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
#include "robotkernel/helpers.h"
#include "robotkernel/robotkernel.h"
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
    size_t len = (input.length() + 1) / 2;

    if (len == 0) {
        *outlen = 0;
        return;
    }

    *output = new char[len];
    unsigned int tmp;

    for (size_t i = 0; i < len; ++i) {
        string sub = input.substr(i*2, 2);
        sscanf(sub.c_str(), "%x", &tmp);
        (*output)[i] = tmp; 
    }

    *outlen = len;
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
    cycle_shift     = get_as<int32_t>(node, "cycle_shift", 0);
}
            
//! emit yaml node
YAML::Node slave::slave_dc::to_yaml() {
    YAML::Node node(YAML::NodeType::Map);
    node["type"]         = type;
    node["cycle_time_0"] = cycle_time_0;
    node["cycle_time_1"] = cycle_time_1;
    node["cycle_shift"]  = cycle_shift;
    return node;
}
            
//! construction
/*!
 * \param[in]   node        YAML initialization node.
 */
slave::slave_eoe::slave_eoe(const YAML::Node& node) {
    has_eoe = true;

    if (node["mac"]) {
        mac.resize(6);
        sscanf(get_as<string>(node, "mac").c_str(), "%hhX:%hhX:%hhX:%hhX:%hhX:%hhX", 
                &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
    }
    
    if (node["ip_address"]) {
        ip_address.resize(4);
        sscanf(get_as<string>(node, "ip_address").c_str(), "%hhu.%hhu.%hhu.%hhu", 
                &ip_address[3], &ip_address[2], &ip_address[1], &ip_address[0]);
    }
    
    if (node["subnet"]) {
        subnet.resize(4);
        sscanf(get_as<string>(node, "subnet").c_str(), "%hhu.%hhu.%hhu.%hhu", 
                &subnet[3], &subnet[2], &subnet[1], &subnet[0]);
    }
    
    if (node["gateway"]) {
        gateway.resize(4);
        sscanf(get_as<string>(node, "gateway").c_str(), "%hhu.%hhu.%hhu.%hhu", 
                &gateway[3], &gateway[2], &gateway[1], &gateway[0]);
    }
    
    if (node["dns"]) {
        dns.resize(4);
        sscanf(get_as<string>(node, "dns").c_str(), "%hhu.%hhu.%hhu.%hhu", 
                &dns[0], &dns[1], &dns[2], &dns[3]);
    }

    if (node["dns_name"]) {
        dns_name = get_as<string>(node, "dns_server");
    }
}

//! emit yaml node
YAML::Node slave::sync_manager_settings::to_yaml() {
    YAML::Node node(YAML::NodeType::Map);
    node["address"] = format_string("0x%X", _address);
    node["flags"]   = format_string("0x%X", _flags);
    node["length"]  = _length;
    return node;
}

//! Emit YAML status of module instance.
/*! 
 * \param[out] out  Emitter output stream.
 * \param[in] sm    Module instance.
 * \return  Output emitter.
 */
YAML::Emitter& operator << (YAML::Emitter& out, slave::sync_manager_settings& sm) {
    out << YAML::BeginMap;
    out << YAML::Key << "address"   << YAML::Value << sm._address;
    out << YAML::Key << "flags"     << YAML::Value << YAML::Hex << sm._flags;
    out << YAML::Key << "length"    << YAML::Value << sm._length;
    out << YAML::EndMap;

    return out;
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
slave::slave(int index, master *master_dev) : 
    key_value_slave(master_dev->name, format_string("slave_%d", index)),
    sm_set_by_user(false), index(index), master_dev(master_dev) 
{
    master_dev->log(verbose, "default slave index %d created\n", index);
    prefer_obj_names = false;
};

//! construction
/*!
 * \param node yaml intialization node
 * \param master_dev master device
 */
slave::slave(int index, const YAML::Node& node, master *master_dev) : 
    key_value_slave(master_dev->name, format_string("slave_%d", index)),
    sm_set_by_user(false), master_dev(master_dev) 
{
    name  = get_as<string>(node, "name");
    this->index = index;

    // sync manager settings
    if (node["sm"]) {
        master_dev->log(verbose,
                "slave %s parsing sm settings\n", name.c_str());

        for (YAML::const_iterator it = node["sm"].begin();
                it != node["sm"].end(); ++it) {
        
            int sm_nr = it->first.as<int>();
            _sm_map[sm_nr] = make_shared<sync_manager_settings_t>(it->second);
        }

        sm_set_by_user = true;
    }
    
    if (node["dc"])
        dc = slave_dc(node["dc"]);
   
    if (node["eoe"]) {
        eoe = slave_eoe(node["eoe"]);
    }

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

    prefer_obj_names = false;

    skip_pdo_description = get_as<bool>(node, "skip_pdo_description", false);

    expected_vendor = get_as<uint32_t>(node, "vendor_id", 0u);
    expected_product = get_as<uint32_t>(node, "product_code", 0u);

    if (node["mapping"]) {
        const YAML::Node& mapping_node = node["mapping"];
        string type = get_as<string>(mapping_node, "type");
        prefer_obj_names = get_as<bool>(mapping_node, "prefer_obj_names", false);

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
        
//! emit yaml node
YAML::Node slave::to_yaml() {
    YAML::Node node;

    node["index"] = index;

    if (name != "") {
        node["name"] = name;
    } else if (master_dev->ec.slaves[index].eeprom.general.name_idx > 0) {
        node["name"] = master_dev->ec.slaves[index].eeprom.strings[
            master_dev->ec.slaves[index].eeprom.general.name_idx - 1];
    } else {
        node["name"] = "no name";
    }

    YAML::Node sms_node(YAML::NodeType::Map);

    for (const auto& kv : _sm_map) {
        if (kv.second->is_set())
            sms_node[kv.first] = kv.second->to_yaml();
    }

    if (sms_node.size() > 0)
        node["sm"] = sms_node;

    if (dc.is_set())
        node["dc"] = dc.to_yaml();

    return node;
}

template <typename T>
key_value_key<T> *create_key(key_value_slave *parent, std::string name, T* val, std::string desc = "", 
        std::string unit = "", std::string default_value = "", std::string format = "") {
    auto *k = new key_value_key<T>(parent, name, val, false);
    k->describe(desc);
    k->unit(unit);
    k->default_value(default_value);
    k->format(format);
    return k;
}

template <typename T>
key_value_key_read_only<T> *create_key_read_only(key_value_slave *parent, std::string name, T* val, std::string desc = "", 
        std::string unit = "", std::string default_value = "", std::string format = "") {
    auto *k = new key_value_key_read_only<T>(parent, name, val, false);
    k->describe(desc);
    k->unit(unit);
    k->default_value(default_value);
    k->format(format);
    return k;
}

void slave::init_key_value() {
    _add_key(create_key<int>     (this, "index", &index, "Position where attached on EtherCAT"));
    _add_key(create_key<string>  (this, "name", &name, "Slave name"));
    _add_key(create_key<bool>    (this, "dc.has_dc", &dc.has_dc, "Slave support for Distributed Clocks"));
    _add_key(create_key<int>     (this, "dc.type", &dc.type, "Distributed Clock type"));
    _add_key(create_key<uint32_t>(this, "dc.cycle_time_0", &dc.cycle_time_0, "Cycle Time Sync0", "ns")); 
    _add_key(create_key<uint32_t>(this, "dc.cycle_time_1", &dc.cycle_time_1, "Cycle Time Sync1", "ns")); 
    _add_key(create_key<int32_t> (this, "dc.cycle_shift", &dc.cycle_shift, "Cyclce Shift"));
            
    for (int i = 0; i < master_dev->ec.slaves[index].sm_ch; ++i) {
        auto prefix = format_string("sync_manager.%d.", i);
        _add_key(create_key<uint16_t>(this, prefix + "address", 
                    &master_dev->ec.slaves[index].sm[i].adr, "Physical start address"));
        _add_key(create_key<uint16_t>(this, prefix + "length", 
                    &master_dev->ec.slaves[index].sm[i].len, "Length"));
        _add_key(create_key<uint32_t>(this, prefix + "flags", 
                    &master_dev->ec.slaves[index].sm[i].flags, "Flags"));
    }

    for (int i = 0; i < master_dev->ec.slaves[index].fmmu_ch; ++i) {
#define _add_key_fmmu(type, mbr, desc)\
        _add_key(create_key<type>(this, format_string("fmmu.%d." # mbr, i), \
                    &master_dev->ec.slaves[index].fmmu[i].mbr, desc))

        _add_key_fmmu(uint32_t, log,            "Logical bus address");
        _add_key_fmmu(uint16_t, log_len,        "Length of logical address area");
        _add_key_fmmu(uint8_t,  log_bit_start, "Start bit at logical bus address");
        _add_key_fmmu(uint8_t,  log_bit_stop,   "Stop bit at logical address plus length");
        _add_key_fmmu(uint16_t, phys,           "Physical (local) address in slave");
        _add_key_fmmu(uint8_t,  phys_bit_start, "Physical start bit at physical address");
        _add_key_fmmu(uint8_t,  type,           "Type, read or write");
        _add_key_fmmu(uint8_t,  active,         "Activation flag");
    }

    _add_key(create_key<uint32_t>(this, "eeprom.vendor_id",
                &master_dev->ec.slaves[index].eeprom.vendor_id, "Vendor ID"));
    _add_key(create_key<uint32_t>(this, "eeprom.product_code",
                &master_dev->ec.slaves[index].eeprom.product_code, "Product Code"));

#define _add_key_general(type, mbr, desc) \
    _add_key(create_key<type>(this, "eeprom.general." # mbr, \
                &master_dev->ec.slaves[index].eeprom.general.mbr, (desc)));
#define _add_key_string(idx, name, desc) \
    if (((idx) > 0) && ((idx) <=master_dev->ec.slaves[index].eeprom.strings_cnt)) \
    _add_key(create_key_read_only<char *>(this, (name), \
                (osal_char_t **)&master_dev->ec.slaves[index].eeprom.strings[(idx) - 1], (desc)));
#define _add_key_general_string(mbr, name, desc) \
    _add_key_string(master_dev->ec.slaves[index].eeprom.general.mbr, "eeprom.general." name, (desc)) 

    _add_key_general(uint8_t, group_idx,        "Group index to strings");
    _add_key_general_string(  group_idx,        "group_name", "Group name");
    _add_key_general(uint8_t, img_idx,          "Image index to strings");
    _add_key_general_string(  img_idx,          "img_name",   "Image name");
    _add_key_general(uint8_t,  order_idx,       "Order index to strings");
    _add_key_general_string(   order_idx,       "order_name", "Order name");
    _add_key_general(uint8_t,  name_idx,        "Name index to strings");
    _add_key_general_string(   name_idx,        "name",       "Name");
    _add_key_general(uint8_t,  physical_layer,  "Physical layer (0 e-bus, 1 ethernet)");
    _add_key_general(uint8_t,  can_open,        "CoE support");
    _add_key_general(uint8_t,  file_access,     "FoE support");
    _add_key_general(uint8_t,  ethernet,        "EoE support");
    _add_key_general(uint8_t,  soe_channels,    "Supported SoE channels");
    _add_key_general(uint8_t,  ds402_channels,  "Supported CoE DS402 channels");
    _add_key_general(uint8_t,  sysman_class,    "Sys Man");
    _add_key_general(uint8_t,  flags,           "EEPROM flags");
    _add_key_general(uint16_t, current_on_ebus, "EBus current in [mA]");


    for (int i = 0; i < master_dev->ec.slaves[index].eeprom.strings_cnt; ++i) {
        auto prefix = format_string("eeprom.strings.%d", i);
        _add_key(create_key_read_only<char *>(this, prefix,
                    (osal_char_t **)&master_dev->ec.slaves[index].eeprom.strings[i], ""));
    }

    for (int i = 0; i < master_dev->ec.slaves[index].eeprom.fmmus_cnt; ++i) {
        auto prefix = format_string("eeprom.fmmu.%d.", i);
        _add_key(create_key_read_only<uint8_t>(this, prefix + "type",
                    &master_dev->ec.slaves[index].eeprom.fmmus[i].type, "FMMU type"));
    }

    for (int i = 0; i < master_dev->ec.slaves[index].eeprom.sms_cnt; ++i) {
        auto prefix = format_string("eeprom.sync_manager.%d.", i);
        _add_key(create_key_read_only<uint16_t>(this, prefix + "adr",
                    &master_dev->ec.slaves[index].eeprom.sms[i].adr, "Physical start address"));
        _add_key(create_key_read_only<uint16_t>(this, prefix + "len",
                    &master_dev->ec.slaves[index].eeprom.sms[i].len, "Length of physical start address"));
        _add_key(create_key_read_only<uint8_t>(this, prefix + "ctrl_reg",
                    &master_dev->ec.slaves[index].eeprom.sms[i].ctrl_reg, "Control register init value"));
        _add_key(create_key_read_only<uint8_t>(this, prefix + "status_reg",
                    &master_dev->ec.slaves[index].eeprom.sms[i].status_reg, "Status register init value"));
        _add_key(create_key_read_only<uint8_t>(this, prefix + "activate",
                    &master_dev->ec.slaves[index].eeprom.sms[i].activate, "Activation flags"));
        _add_key(create_key_read_only<uint8_t>(this, prefix + "pdi_ctrl",
                    &master_dev->ec.slaves[index].eeprom.sms[i].pdi_ctrl, "PDI control register"));
    }

    for (int i = 0; i < master_dev->ec.slaves[index].eeprom.dcs_cnt; ++i) {
        auto prefix = format_string("eeprom.distributed_clocks.%d.", i);
        _add_key(create_key_read_only<uint32_t>(this, prefix + "cycle_time_0",
                    &master_dev->ec.slaves[index].eeprom.dcs[i].cycle_time_0, "Cycle time Sync0"));
        _add_key(create_key_read_only<uint32_t>(this, prefix + "shift_time_0",
                    &master_dev->ec.slaves[index].eeprom.dcs[i].shift_time_0, "Shift time Sync0"));
        _add_key(create_key_read_only<uint32_t>(this, prefix + "shift_time_1",
                    &master_dev->ec.slaves[index].eeprom.dcs[i].shift_time_1, "Shift time Sync1"));
        _add_key(create_key_read_only<int16_t>(this, prefix + "sync_1_cycle_factor",
                    &master_dev->ec.slaves[index].eeprom.dcs[i].sync_1_cycle_factor, "Cycle factor Sync1"));
        _add_key(create_key_read_only<uint16_t>(this, prefix + "assign_active",
                    &master_dev->ec.slaves[index].eeprom.dcs[i].assign_active, "Activation flags"));
        _add_key(create_key_read_only<int16_t>(this, prefix + "sync_0_cycle_factor",
                    &master_dev->ec.slaves[index].eeprom.dcs[i].sync_0_cycle_factor, "Cycle factor Sync0"));
        _add_key(create_key_read_only<uint8_t>(this, prefix + "name_idx",
                    &master_dev->ec.slaves[index].eeprom.dcs[i].name_idx, "Name index in strings"));
        _add_key_string(master_dev->ec.slaves[index].eeprom.dcs[i].name_idx, prefix + "name", "Name"); 
        _add_key(create_key_read_only<uint8_t>(this, prefix + "desc_idx",
                    &master_dev->ec.slaves[index].eeprom.dcs[i].desc_idx, "Description index in strings"));
        _add_key_string(master_dev->ec.slaves[index].eeprom.dcs[i].desc_idx, prefix + "desc", "Description"); 
    }

    ec_eeprom_cat_pdo_t *entry;
    struct ec_eeprom_cat_pdo_queue *pdos[] = { 
        &master_dev->ec.slaves[index].eeprom.txpdos,
        &master_dev->ec.slaves[index].eeprom.rxpdos };

    for (int u = 0; u < 2; ++u) {
        string type = u == 0 ? string("txpdo") : string("rxpdo");

        TAILQ_FOREACH(entry, pdos[u], qh) {
            auto prefix = format_string("eeprom.%s.0x%04X.", type.c_str(), entry->pdo_index);
            _add_key(create_key_read_only<uint8_t>(this, prefix + "n_entry",
                        &entry->n_entry, "Number of PDO entries"));
            _add_key(create_key_read_only<uint8_t>(this, prefix + "sm_nr",
                        &entry->sm_nr, "Assigned sync manager"));
            _add_key(create_key_read_only<uint8_t>(this, prefix + "dc_sync",
                        &entry->dc_sync, "Use distributed clocks"));
            _add_key(create_key_read_only<uint8_t>(this, prefix + "name_idx",
                        &entry->name_idx, "Name index in strings"));
            _add_key_string(entry->name_idx, prefix + "name", "Name"); 
            _add_key(create_key_read_only<uint16_t>(this, prefix + "flags",
                        &entry->flags, "PDO flags"));

            for (int i = 0; i < entry->n_entry; ++i) {
                auto prefix2 = format_string("%s0x%04X.%d.", prefix.c_str(), 
                        entry->entries[i].entry_index, entry->entries[i].sub_index);
                _add_key(create_key_read_only<uint8_t>(this, prefix2 + "entry_name_idx",
                            &entry->entries[i].entry_name_idx, "Name index in strings"));
                _add_key_string(entry->entries[i].entry_name_idx, prefix2 + "entry_name", "Entry name"); 

                _add_key(create_key_read_only<uint8_t>(this, prefix2 + "data_type",
                            &entry->entries[i].data_type, "Data type"));
                _add_key(create_key_read_only<uint8_t>(this, prefix2 + "bit_len",
                            &entry->entries[i].bit_len, "Length in bits"));
                _add_key(create_key_read_only<uint16_t>(this, prefix2 + "flags",
                            &entry->entries[i].flags, "Flags"));
            }
        }
    }
}

// perform robotkernel clean up
void slave::clean_up() {
    robotkernel::remove_device(_eeprom_mi);
    robotkernel::remove_device(_memory_mi);
    robotkernel::remove_device(_eeprom_coe);

    _eeprom_mi = nullptr;
    _memory_mi = nullptr;
    _eeprom_coe = nullptr;
}

//! sending slave init commands
/*!
 */
void slave::add_init_cmds() {
    for (const auto& cmd : coe_init_cmds) {
        if (cmd->already_added)
            continue;

        if (!cmd->data) {
            // get description
            uint32_t error_code = 0;
            ec_coe_sdo_entry_desc_t entry_desc;
            int ret2 = ec_coe_sdo_entry_desc_read(&master_dev->ec, index, 
                    cmd->index, cmd->subindex, 0x7F, &entry_desc, &error_code);

            if (ret2 == 0) {
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
            ec_init_cmd_t& icmd = *master_dev->init_cmds.insert(master_dev->init_cmds.end(), ec_init_cmd_t());
            ec_slave_mailbox_coe_init_cmd_init(&icmd, 
                    (int)cmd->transition, cmd->index, cmd->subindex, 
                    cmd->ca, cmd->data, cmd->datalen);
            ec_slave_add_init_cmd(&master_dev->ec, index, &icmd);

            cmd->already_added = true;
        }
    } 

    for (soe_list_t::iterator it = soe_init_cmds.begin();
            it != soe_init_cmds.end(); ++it) {

        soe_init_cmd_t *cmd = *it;

        if (cmd->already_added)
            continue;

        ec_init_cmd_t& icmd = *master_dev->init_cmds.insert(master_dev->init_cmds.end(), ec_init_cmd_t());
        ec_slave_mailbox_soe_init_cmd_init(&icmd, 
                (int)cmd->transition, cmd->idn, cmd->element, 
                cmd->atn, cmd->data, cmd->datalen);
        ec_slave_add_init_cmd(&master_dev->ec, index, &icmd);

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
            for (const auto& cmd : coe_init_cmds) {
                cmd->already_added = false;
            }
        case preop_2_boot:
            // ====> deinit devices
            for (int i = 0; i < master_dev->ec.slaves[index].sm_ch; ++i) {
                auto prefix = format_string("sync_manager.%d.", i);
                key_map.erase(prefix + "address");
                key_map.erase(prefix + "length");
                key_map.erase(prefix + "flags");    
            }
            
            for (int i = 0; i < master_dev->ec.slaves[index].fmmu_ch; ++i) {
                auto prefix = format_string("fmmu.%d.", i);
                key_map.erase(prefix + "log");
                key_map.erase(prefix + "log_len");
                key_map.erase(prefix + "log_bit_start");
                key_map.erase(prefix + "log_bit_stop");
                key_map.erase(prefix + "phys");
                key_map.erase(prefix + "phys_bit_start");
                key_map.erase(prefix + "type");
                key_map.erase(prefix + "active");
            }

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
            if (true == dc.has_dc) {
                if (dc.cycle_time_0 == 0)
                    dc.cycle_time_0 = master_dev->ec.main_cycle_interval; 

                if (dc.type == 1) {
                    if (dc.cycle_time_1 == 0)
                        dc.cycle_time_1 = master_dev->ec.main_cycle_interval; 

                    master_dev->log(verbose, "slave %2d configuring dc sync 01, "
                            "cycle_times %d/%d, cycle_shift %d\n",
                            index, dc.cycle_time_0, dc.cycle_time_1, dc.cycle_shift);

                    ec_slave_set_dc_config(&master_dev->ec, index, 1, 7, 
                            dc.cycle_time_0, dc.cycle_time_1, dc.cycle_shift);
                } else {
                    master_dev->log(verbose, "slave %2d configuring dc sync 0, "
                            "cycle_time %d, cycle_shift %d\n",
                            index, dc.cycle_time_0, dc.cycle_shift);

                    ec_slave_set_dc_config(&master_dev->ec, index, 1, 3, 
                            dc.cycle_time_0, 0, dc.cycle_shift);
                }
            } else 
                ec_slave_set_dc_config(&master_dev->ec, index, 0, 0, 0, 0, 0); 

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
    uint32_t mbx_sup = master_dev->ec.slaves[index].eeprom.mbx_supported;
    uint32_t soe_ch  = master_dev->ec.slaves[index].eeprom.general.soe_channels;
    auto *slv = &(master_dev->ec.slaves[index]);

#define REMOVE_SERVICE_COLLECTOR(req) \
            { if (req) { robotkernel::remove_device(req); (req) = nullptr; } }

#define ADD_SERVICE_COLLECTOR(req) { \
            robotkernel::add_device(req); }

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
            if (pdin)  { 
                robotkernel::remove_device(pdin_inspection);
                pdin_inspection = nullptr;

                pdin->reset_provider(pdin_provider);
                pdin_provider = nullptr;
                robotkernel::remove_device(pdin); 
                pdin = nullptr; 
            }

            if (pdout) { 
                robotkernel::remove_device(pdout_inspection);
                pdout_inspection = nullptr;

                pdout->reset_consumer(pdout_consumer);
                pdout_consumer = nullptr;
                robotkernel::remove_device(pdout); 
                pdout = nullptr; 
            }

            if (to == module_state_preop)
                break;
        case preop_2_init:
        case preop_2_boot:
            delete_keys();

            // ====> deinit devices
            REMOVE_SERVICE_COLLECTOR(_mbx_foe);
            REMOVE_SERVICE_COLLECTOR(mbx_coe);

            if (mbx_sup & EC_EEPROM_MBX_SOE) {
                for (unsigned atn = 0; atn < _mbx_soe_list.size(); ++atn) {
                    REMOVE_SERVICE_COLLECTOR(_mbx_soe_list[atn]);
                }
            }

            REMOVE_SERVICE_COLLECTOR(_eeprom_mi);
            REMOVE_SERVICE_COLLECTOR(_memory_mi);
            REMOVE_SERVICE_COLLECTOR(_eeprom_coe);
        case init_2_init:
            // ====> re-/open ethercat device
            if (to == module_state_init)
                break;
        case init_2_boot:
            ADD_SERVICE_COLLECTOR_CLASS(_eeprom_mi, slave::memory_inspection, request_type_eeprom);
            ADD_SERVICE_COLLECTOR_CLASS(_memory_mi, slave::memory_inspection, request_type_memory);
            ADD_SERVICE_COLLECTOR_CLASS(_eeprom_coe, slave::canopen, request_type_eeprom);

            ADD_SERVICE_COLLECTOR_CLASS(_mbx_foe, slave::file_protocol);
            break;
        case boot_2_init:
        case boot_2_preop:
        case boot_2_safeop:
        case boot_2_op:
            // ====> re-/open ethercat device
            REMOVE_SERVICE_COLLECTOR(_mbx_foe);

            REMOVE_SERVICE_COLLECTOR(_eeprom_mi);
            REMOVE_SERVICE_COLLECTOR(_memory_mi);
            REMOVE_SERVICE_COLLECTOR(_eeprom_coe);

            if (to == module_state_init)
                break;
        case init_2_op:
        case init_2_safeop:
        case init_2_preop:
        case preop_2_preop: {
            // ====> initial devices            
            ADD_SERVICE_COLLECTOR_CLASS(_eeprom_mi, slave::memory_inspection, request_type_eeprom);
            ADD_SERVICE_COLLECTOR_CLASS(_memory_mi, slave::memory_inspection, request_type_memory);
            ADD_SERVICE_COLLECTOR_CLASS(_eeprom_coe, slave::canopen, request_type_eeprom);

            init_key_value();
            
            robotkernel::add_device(std::static_pointer_cast<
                    service_provider::key_value::base>(shared_from_this()));
            
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
        }
        case preop_2_op:
        case preop_2_safeop:
            // ====> start receiving measurements
            if (slv->pdin.len) {
                if (pdin)
                    robotkernel::remove_device(pdin);
                
                string base_name = master_dev->use_real_names ?
                    format_string("%s.inputs", name.c_str()) :
                    format_string("slave_%d.inputs", index);

                string pdo_desc = "";
                    
                if (mbx_coe && !skip_pdo_description) {
                    try {
                        pdo_desc = mbx_coe->get_pdo_description(0x1C13);
                    } catch (std::exception& e) {
                        master_dev->log(error, e.what());
                    }
                }

                if (pdo_desc == "") {
                    pdo_desc = format_string("- uint8_t[%d]: buf\n", slv->pdin.len);
                }

                pdin = make_shared<robotkernel::triple_buffer>(slv->pdin.len, master_dev->name, base_name, pdo_desc);
                pdin_provider = make_shared<robotkernel::pd_provider>(master_dev->name + "." + base_name);
                pdin->set_provider(pdin_provider);
                robotkernel::add_device(pdin);

                pdin_inspection = make_shared<service_provider::process_data_inspection::pd_inspection>(master_dev->name, base_name, pdin);
                robotkernel::add_device(pdin_inspection);
            }
            
            if (slv->pdout.len) {
                if (pdout)
                    robotkernel::remove_device(pdout);
                
                string base_name = master_dev->use_real_names ?
                    format_string("%s.outputs", name.c_str()) :
                    format_string("slave_%d.outputs", index);

                string pdo_desc = "";
                
                if (mbx_coe && !skip_pdo_description) {
                    try {
                        pdo_desc = mbx_coe->get_pdo_description(0x1C12);
                    } catch (std::exception& e) {
                        master_dev->log(error, e.what());
                    }
                }
                
                if (pdo_desc == "") {
                    pdo_desc = format_string("- uint8_t[%d]: buf\n", slv->pdout.len);
                }

                pdout = make_shared<robotkernel::triple_buffer>(slv->pdout.len, master_dev->name, base_name, pdo_desc);
                pdout_consumer = make_shared<robotkernel::pd_consumer>(master_dev->name + "." + base_name);
                pdout->set_consumer(pdout_consumer);
                robotkernel::add_device(pdout);
                
                pdout_inspection = make_shared<service_provider::process_data_inspection::pd_inspection>(master_dev->name, base_name, pdout);
                robotkernel::add_device(pdout_inspection);
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


static int decode_soe_answer(uint8_t *tmp, service_provider::sercos_protocol::sercos_service_attribute_t attr, 
    std::vector<uint16_t>& value) 
{
    bool is_fix = true;
    size_t elem_size = 1;

    switch (attr.datalength) {
        case service_provider::sercos_protocol::SSA_DATALENGTH_2BYTEFIX:
            elem_size = 2;
            break;
        case service_provider::sercos_protocol::SSA_DATALENGTH_4BYTEFIX:
            elem_size = 4;
            break;
        case service_provider::sercos_protocol::SSA_DATALENGTH_8BYTEFIX:
            elem_size = 8;
            break;
        case service_provider::sercos_protocol::SSA_DATALENGTH_1BYTEVAR:
            is_fix = false;
            elem_size = 1;
            break;
        case service_provider::sercos_protocol::SSA_DATALENGTH_2BYTEVAR:
            is_fix = false;
            elem_size = 2;
            break;
        case service_provider::sercos_protocol::SSA_DATALENGTH_4BYTEVAR:
            is_fix = false;
            elem_size = 4;
            break;
        case service_provider::sercos_protocol::SSA_DATALENGTH_8BYTEVAR:
            is_fix = false;
            elem_size = 8;
            break;
    }

    if (is_fix) {
        value.resize((elem_size+1)/2);
        memcpy(&value[0], tmp, (elem_size));
        return elem_size;
    }
        
    uint16_t array_len = *(uint16_t *)tmp;
    value.resize(4 + (elem_size+1)/2 * array_len);
    memcpy(&value[0], tmp, 4 + elem_size * array_len);
    return (4 + elem_size * array_len);
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
    uint8_t buf[1024]; 
    uint8_t serc_elements = (elements | service_provider::sercos_protocol::SSE_ATTR) >> 1;
    size_t buf_len = 1024;
    int ret;

    if ((ret = ec_soe_read(&slv->master_dev->ec, slv->index, atn, idn,
                &serc_elements, buf, &buf_len)) != 0) {
        throw str_exception("slave %2d: reading sercos atn %d idn 0x%X "
                "elements 0x%X returned errorcode 0x%X!\n", slv->index, 
                atn, idn, serc_elements, ret);
    }

    // todo decode answer
    uint8_t *tmp = buf;
    if (serc_elements & (service_provider::sercos_protocol::SSE_NAME >> 1)) {
        uint16_t name_len = *(uint16_t *)tmp; 
        tmp += 4;
        data.name = string((char *)tmp, (size_t)name_len);
        tmp += name_len;
    }

    if (serc_elements & (service_provider::sercos_protocol::SSE_ATTR >> 1)) {
        data.attr = *(service_provider::sercos_protocol::sercos_service_attribute *)tmp;
        tmp += 4;
    }

    if (serc_elements & (service_provider::sercos_protocol::SSE_UNIT >> 1)) {
        uint16_t unit_len = *(uint16_t *)tmp; 
        tmp += 4;
        data.unit = string((char *)tmp, (size_t)unit_len);
        tmp += unit_len;
    }

    if (serc_elements & (service_provider::sercos_protocol::SSE_MAXVAL >> 1)) {
        tmp += decode_soe_answer(tmp, data.attr, data.min_value);
    }

    if (serc_elements & (service_provider::sercos_protocol::SSE_MINVAL >> 1)) {
        tmp += decode_soe_answer(tmp, data.attr, data.max_value);
    }

    if (serc_elements & (service_provider::sercos_protocol::SSE_DATA >> 1)) {
        tmp += decode_soe_answer(tmp, data.attr, data.value);
    }
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
    uint8_t serc_elements = (elements | service_provider::sercos_protocol::SSE_ATTR) >> 1;
    int ret;

    if ((ret = ec_soe_write(&slv->master_dev->ec, slv->index, atn, idn,
                serc_elements, (uint8_t *)&data.value[0], data.value.size() * 2)) != 0) {
        throw str_exception("slave %2d: writing sercos atn %d idn 0x%X "
                "elements 0x%X returned errorcode 0x%X!\n", slv->index, 
                atn, idn, serc_elements, ret);
    }
}
	    
//! process data out handler
void slave::pdout_handler() {
    ec_slave_t *slv = &master_dev->ec.slaves[index];
    if (!pdout || !pdout_consumer || (slv->pdout.len == 0))
        return;

    pdout->read(pdout_consumer, 0, slv->pdout.pd, slv->pdout.len);
}

//! process data in handler
void slave::pdin_handler() {
    ec_slave_t *slv = &master_dev->ec.slaves[index];
    if (!pdin || !pdin_provider || (slv->pdin.len == 0))
        return;

    pdin->write(pdin_provider, 0, slv->pdin.pd, slv->pdin.len);
}

