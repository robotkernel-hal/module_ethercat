//! robotkernel module ethercat master
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

#include "master.h"
#include "interface_canopen_protocol/module_intf.h"
#include "interface_sercos_protocol/module_intf.h"
#include "interface_file_protocol/module_intf.h"

MODULE_DEF(module_ethercat, module_ethercat::master)

using namespace std;
using namespace robotkernel;
using namespace module_ethercat;

//! ethercat state string
const string module_ethercat::state_strings[] = {
    "Unknown (0)",
    "EtherCAT INIT",
    "EtherCAT PREOP",
    "Unknown (3)",
    "EtherCAT SAFEOP",
    "Unknown (5)",
    "Unknown (6)",
    "Unknown (7)",
    "EtherCAT OP"
};


void log_func(int lvl, void *user, const char *format, ...) {
    master *e = (master *)user;
    va_list ap;
    va_start(ap, format);

    robotkernel::loglevel loglvl = verbose;
    if (lvl < 100)
        loglvl = info;
    if (lvl < 10)
        loglvl = warning;
    if (lvl < 1)
        loglvl = error;

    char buf[1024];
    vsnprintf(buf, sizeof(buf), format, ap);
//    e->log(loglvl, format, ap);
    va_end(ap);

    e->log(loglvl, buf);
}

master::group::group(int index, const YAML::Node& node) {
    _index          = index;
    _divisor        = get_as<int>(node, "divisor");
    _divisor_cnt    = 0;
    _recv_timeout   = get_as<int>(node, "recv_timeout", 1000000);
    _pd_intf        = NULL;

    for (YAML::const_iterator it = node["slaves"].begin(); 
            it != node["slaves"].end(); ++it)
        _slaves.push_back(it->as<int>());
}

//! register interfaces for group
/*!
 * \param ctx ethercat context
 * \return N/A
 */
void master::group::register_interfaces(std::string name, const loglevel& ll) {
    if (_pd_intf)
        return;

    std::stringstream group_name; 
    group_name << "group_" << _index;

    YAML::Node node;
    node["mod_name"] = name;
    node["dev_name"] = group_name.str();
    node["slave_id"] = (signed int)(_index | ECAT_SLAVE_ID_GROUP);
    node["loglevel"] = (string)ll;

    _pd_intf = kernel::register_interface_cb(
            "libinterface_process_data_inspection.so", node);
}

//! unregister interfaces of group
/*!
 * \return N/A
 */
void master::group::unregister_interfaces() {
    if (_pd_intf) {
        kernel::unregister_interface_cb(_pd_intf);
        _pd_intf = NULL; 
    }
}

//! construction
/*!
 * \param node yaml intialization node
 */
master::master(const std::string& name, const YAML::Node& node) 
    : module_base("module_ethercat", name, node), 
      cmd_delay(node),
      runnable(node) {
    _ifname          = get_as<string>(node, "ifname");
    _recv_prio       = get_as<int>(node, "recv_prio");
    _recv_mask       = get_as<int>(node, "recv_mask");
    _log_eeprom_data = get_as<bool>(node, "log_eeprom_data", false);
    _pec             = NULL;
    _trigger_interval= get_as<int>(node, "trigger_interval", 0);
            
    dc_offset_compensation_cycles 
                = get_as<int>(node, "dc_offset_compensation_cycles", 250);
    dc_timer_override 
                = get_as<int>(node, "dc_timer_override", -1);
    dc_offset_compensation_max 
                = get_as<uint64_t>(node, "dc_offset_compensation_max", 100000000);

    ec_log_func_user = this;
    ec_log_func = log_func;

    // group settings
    if (node["groups"]) {
        for (YAML::const_iterator it = node["groups"].begin();
                it != node["groups"].end(); ++it) {
            int g_nr = it->first.as<int>();
            _group_info[g_nr] = new group(g_nr, it->second);
        }
    }

    if (node["slaves"] != NULL) {
        // parsing slave configurations
        const YAML::Node& slaves = node["slaves"];
        for (YAML::const_iterator it = slaves.begin(); it != slaves.end(); ++it) {
            slave *slv = new slave(*it, this);
            _slave_info[slv->index] = slv;
        }
    }

    _dc_mode_string = get_as<string>(node, "dc_mode", "master_clock");
    _dc_sync.first_run = true;
    _dc_sync.last_diff = 0.;

    pthread_mutex_init(&pd_lock, NULL);
    pthread_cond_init(&pd_cond, NULL);
   
    pthread_mutex_init(&async_lock, NULL);
    pthread_cond_init(&async_cond, NULL);

    YAML::Node dc_node;
    dc_node["mod_name"] = name;
    dc_node["dev_name"] = "distributed_clocks";
    dc_node["slave_id"] = ECAT_SLAVE_ID_DC;
    dc_node["loglevel"] = (string)ll;

    log(info, "adding process data inspection for dc info\n");

    dc_pd_intf = kernel::register_interface_cb(
            "libinterface_process_data_inspection.so", dc_node);

    pd_cookie = 0;

    // -----------------------------------------------------------
    // open ethercat interface
    int ret = ec_open(&_pec, _ifname.c_str(), _recv_prio, _recv_mask, _log_eeprom_data);
    if (ret != 0) 
        throw str_exception("ec_open failed: %s!\n", strerror(ret));
        
    // -----------------------------------------------------------
    // setting init commands and distributed clocks
    for (slave_map_t::iterator it = _slave_info.begin(); 
            it != _slave_info.end(); ++it) {
        int slave_nr = it->first;
        slave *slv = it->second;

        if (slave_nr > _pec->slave_cnt) {
            log(error, "setting inits for slave %d, failed. no slave found!\n", slave_nr);
            continue;
        }

        for (slave::coe_list_t::iterator it2 = slv->coe_init_cmds.begin();
                it2 != slv->coe_init_cmds.end(); ++it2) {
            slave::coe_init_cmd_t *cmd = *it2;
            ec_slave_add_init_cmd(_pec, slave_nr, EC_MBX_COE, 
                    (int)cmd->transition, cmd->index, cmd->subindex, 
                    cmd->ca, cmd->data, cmd->datalen);
        }

        if (slv->dc.has_dc) {
            _pec->slaves[slave_nr].dc.use_dc        = 1;
            _pec->slaves[slave_nr].dc.type          = slv->dc.type;
            _pec->slaves[slave_nr].dc.cycle_time_0  = slv->dc.cycle_time_0;
            _pec->slaves[slave_nr].dc.cycle_time_1  = slv->dc.cycle_time_1;
            _pec->slaves[slave_nr].dc.cycle_shift   = slv->dc.cycle_shift;
        } else 
            _pec->slaves[slave_nr].dc.use_dc = 0;
    }

    // -----------------------------------------------------------
    // creating and assigning process data groups
    ec_create_pd_groups(_pec, _group_info.size());
            
    for (group_map_t::iterator it = _group_info.begin(); it != _group_info.end(); ++it) {
        int g_nr = it->first;

        for (std::list<int>::iterator it2 = it->second->_slaves.begin();
                it2 != it->second->_slaves.end(); ++it2) {

            int s_nr = *it2;
            if (_pec->slave_cnt <= s_nr) {
                log(warning, "slave %d not connected to ethercat bus, "
                        "not adding to group %d\n", s_nr, g_nr);
                continue;
            }

            _pec->slaves[s_nr].assigned_pd_group = g_nr;
        }
    }
    
    set_state(module_state_init);
    
    // -----------------------------------------------------------
    // set pdo mapping entries
    for (slave_map_t::iterator it = _slave_info.begin(); 
            it != _slave_info.end(); ++it) {
        int slave_nr = it->first;
        slave *slv = it->second;

        if (_pec->slave_cnt > slave_nr) {
            if (_pec->slaves[slave_nr].eeprom.mbx_supported & 
                    EC_EEPROM_MBX_COE) {              

                // generate input mapping for coe
                int mapping_entries = slv->input_mapping.size();
                if (mapping_entries > 0) {
                    uint16_t mapping[mapping_entries + 1];
                    int mnr = 0;
                    mapping[mnr++] = mapping_entries;
                    for (slave::mapping_t::iterator mit = slv->input_mapping.begin(); 
                            mit != slv->input_mapping.end(); ++mit) {
                        mapping[mnr++] = *mit;
                    }
                
                    ec_slave_add_init_cmd(_pec, slave_nr, EC_MBX_COE, 0x24, 0x1C13, 
                            0, 1, (char *)mapping, 2 * (mapping_entries + 1));
                }
                
                // generate output mapping for coe
                mapping_entries = slv->output_mapping.size();
                if (mapping_entries > 0) {
                    uint16_t mapping[mapping_entries + 1];
                    int mnr = 0;
                    mapping[mnr++] = mapping_entries;
                    for (slave::mapping_t::iterator mit = slv->output_mapping.begin(); 
                            mit != slv->output_mapping.end(); ++mit) {
                        mapping[mnr++] = *mit;
                    }
                
                    ec_slave_add_init_cmd(_pec, slave_nr, EC_MBX_COE, 0x24, 0x1C12, 
                            0, 1, (char *)mapping, 2 * (mapping_entries + 1));
                }
            }
        } else {
            log(error, "setting mapping for slave %d, failed. no slave found!\n", slave_nr);
        }
    }
}

//! destruction 
master::~master() {
    if (_pec)
        ec_close(_pec);

    _pec = NULL;
    
    pthread_mutex_destroy(&async_lock);
    pthread_cond_destroy(&async_cond);
    
    pthread_mutex_destroy(&pd_lock);
    pthread_cond_destroy(&pd_cond);
}

#define TRANSITION_INIT_2_UNKNOWN       0x0001FFFE
#define TRANSITION_INIT_2_ERROR         0x0001FFFF
#define TRANSITION_INIT_2_BOOT          0x00010000
#define TRANSITION_INIT_2_INIT          0x00010001
#define TRANSITION_INIT_2_PREOP         0x00010002
#define TRANSITION_INIT_2_SAFEOP        0x00010003
#define TRANSITION_INIT_2_OP            0x00010004

#define TRANSITION_PREOP_2_UNKNOWN      0x0002FFFE
#define TRANSITION_PREOP_2_ERROR        0x0002FFFF
#define TRANSITION_PREOP_2_BOOT         0x00020000
#define TRANSITION_PREOP_2_INIT         0x00020001
#define TRANSITION_PREOP_2_PREOP        0x00020002
#define TRANSITION_PREOP_2_SAFEOP       0x00020003
#define TRANSITION_PREOP_2_OP           0x00020004

#define TRANSITION_SAFEOP_2_UNKNOWN     0x0003FFFE
#define TRANSITION_SAFEOP_2_ERROR       0x0003FFFF
#define TRANSITION_SAFEOP_2_BOOT        0x00030000
#define TRANSITION_SAFEOP_2_INIT        0x00030001
#define TRANSITION_SAFEOP_2_PREOP       0x00030002
#define TRANSITION_SAFEOP_2_SAFEOP      0x00030003
#define TRANSITION_SAFEOP_2_OP          0x00030004

#define TRANSITION_OP_2_UNKNOWN         0x0004FFFE
#define TRANSITION_OP_2_ERROR           0x0004FFFF
#define TRANSITION_OP_2_BOOT            0x00040000
#define TRANSITION_OP_2_INIT            0x00040001
#define TRANSITION_OP_2_PREOP           0x00040002
#define TRANSITION_OP_2_SAFEOP          0x00040003
#define TRANSITION_OP_2_OP              0x00040004

//! set module state machine to defined state
/*!
 * \param state requested state
 * \return success or failure
 */
int master::set_state(module_state_t new_state) {
    int ret = 0, nr;

    log(info, "setting state from %s to %s\n", 
            state_to_string(state), state_to_string(new_state));

    switch (new_state) {
        case module_state_boot: {
            if (state != module_state_init) {
                ret = -1;
                break;
            }

            ec_set_state(_pec, EC_STATE_BOOT);
            
            for (nr = 0; nr < _pec->slave_cnt; ++nr)
                _slave_info[nr]->register_interfaces(module_state_boot);

            break;
        }
        case module_state_init: {
            stop();
            _pec->tx_sync = 1;

            ec_set_state(_pec, EC_STATE_INIT);

            for (nr = 0; nr < _pec->slave_cnt; ++nr) {
                if (_slave_info.find(nr) == _slave_info.end()) {
                    log(verbose, "slave %d creating empty one\n", nr);

                    slave *slv = new slave(nr, this);
                    _slave_info[nr] = slv;
                }
                
                if (_slave_info[nr]->dc.has_dc)
                    _pec->slaves[nr].dc.use_dc = 1;
                else 
                    _pec->slaves[nr].dc.use_dc = 0;
                
                _slave_info[nr]->register_interfaces(module_state_init);
            }

            break;
        }
        case module_state_preop: {
            stop();
            _pec->tx_sync = 1;
            if (_dc_mode_string == "ref_clock") 
                _pec->dc.mode = ec_dc_info::dc_mode_ref_clock;
            else
                _pec->dc.mode = ec_dc_info::dc_mode_master_clock;
            
            ec_set_state(_pec, EC_STATE_PREOP);

            if (dc_offset_compensation_cycles > 0)
                _pec->dc.offset_compensation = dc_offset_compensation_cycles;
            if (dc_timer_override > 0)
                _pec->dc.timer_override = dc_timer_override;
            if (dc_offset_compensation_max > 0)
                _pec->dc.offset_compensation_max = dc_offset_compensation_max;

            for (nr = 0; nr < _pec->slave_cnt; ++nr) {
                log(verbose, "slave %d: propagation delay %d [ns]\n", 
                        nr, _pec->slaves[nr].pdelay);

                // apply sm and fmmu config
                for (slave::sm_map_t::iterator it = _slave_info[nr]->_sm_map.begin();
                        it != _slave_info[nr]->_sm_map.end(); ++it) {
                    int sm_nr = it->first;

                    if (sm_nr < _pec->slaves[nr].sm_ch) {
                        log(verbose, "slave %d: applying sm%d: adr 0x%X, len %d, flags 0x%X\n",
                                nr, sm_nr, it->second->_address,
                                it->second->_length,
                                it->second->_flags);

                        _pec->slaves[nr].sm[sm_nr].adr = it->second->_address;
                        _pec->slaves[nr].sm[sm_nr].len = it->second->_length;
                        _pec->slaves[nr].sm[sm_nr].flags = it->second->_flags;
                        _pec->slaves[nr].sm_set_by_user = 1;
                    }
                }
            }
            
            for (nr = 0; nr < _pec->slave_cnt; ++nr)
                _slave_info[nr]->register_interfaces(module_state_preop);

            break;
        }
        case module_state_safeop:            
            _dc_sync.first_run = true;

            if (dc_timer_override == -1 && trigger_mod_name != "") {
                double tmp;
                kernel::request_cb(trigger_mod_name.c_str(), 
                        MOD_REQUEST_GET_TRIGGER_INTERVAL, &tmp);

                log(info, "got trigger interval from our trigger module: %+17.13f\n", 
                        tmp);

                dc_timer_override = 
                _pec->dc.timer_override = tmp * 1E9;
            }
            // start cyclic operation via trigger
            _pec->tx_sync = 0;
            state = module_state_safeop;

            ec_set_state(_pec, EC_STATE_SAFEOP);
            
            for (nr = 0; nr < _pec->slave_cnt; ++nr)
                _slave_info[nr]->register_interfaces(module_state_safeop);

            start();
            break;
        case module_state_op:
            start();
            _pec->tx_sync = 0;

            ec_set_state(_pec, EC_STATE_OP);
            
            for (nr = 0; nr < _pec->slave_cnt; ++nr)
                _slave_info[nr]->register_interfaces(module_state_op);

            break;
        default:
            ret = -1;
            break;
    }

    if (ret == 0)
        state = new_state;

    return ret;
}

//! send a request to module
/*!
 * \param reqcode request code
 * \param ptr pointer to request structure
 * \return success or failure
 */
int master::request(int reqcode, void* ptr) {
    int ret = 0;

    switch (reqcode) {
        case MOD_REQUEST_GET_PDIN: {            
            process_data_t *pd = (process_data_t *)ptr;
            pd->pd = NULL;
            pd->len = 0;

            if (pd->slave_id & ECAT_SLAVE_ID_GROUP) {
                // group case
                int g_nr = ECAT_SLAVE_ID_GET_GROUP(pd->slave_id);
                if (g_nr < _pec->pd_group_cnt) {
                    pd->pd = _pec->pd_groups[g_nr].pd + _pec->pd_groups[g_nr].pdout_len;
                    pd->len = _pec->pd_groups[g_nr].pdin_len;
                }
            } else if (pd->slave_id & ECAT_SLAVE_ID_DC) {
                pd->pd = &_pec->dc.dc_time;
                pd->len = (uint8_t *)&_pec->dc.p_de_dc - (uint8_t *)&_pec->dc.dc_time;
            } else {
                int sub_slave_id = ECAT_SLAVE_ID_GET_SUB(pd->slave_id),
                    slave_id = ECAT_SLAVE_ID_GET_SLAVE(pd->slave_id);

                if (slave_id < _pec->slave_cnt) {
                    if (pd->slave_id & ECAT_SLAVE_ID_SUB) {
                        if (_pec->slaves[slave_id].subdev_cnt > (unsigned)sub_slave_id) {
                            pd->pd = _pec->slaves[slave_id].subdevs[sub_slave_id].pdin.pd;
                            pd->len = _pec->slaves[slave_id].subdevs[sub_slave_id].pdin.len;
                        }
                    } else {
                        pd->pd = _pec->slaves[slave_id].pdin.pd;
                        pd->len = _pec->slaves[slave_id].pdin.len;
                    }
                }
            }

            log(verbose, "GET_PDIN: %p/%d\n", pd->pd, pd->len);
            break;
        }
        case MOD_REQUEST_GET_PDOUT: {            
            process_data_t *pd = (process_data_t *)ptr;
            pd->pd = NULL;
            pd->len = 0;
            if (pd->slave_id & ECAT_SLAVE_ID_GROUP) {
                // group case
                int g_nr = ECAT_SLAVE_ID_GET_GROUP(pd->slave_id);
                if (g_nr < _pec->pd_group_cnt) {
                    pd->pd = _pec->pd_groups[g_nr].pd;
                    pd->len = _pec->pd_groups[g_nr].pdout_len;
                }
            } else {
                int sub_slave_id = ECAT_SLAVE_ID_GET_SUB(pd->slave_id),
                    slave_id = ECAT_SLAVE_ID_GET_SLAVE(pd->slave_id);

                if (slave_id < _pec->slave_cnt) {
                    if (pd->slave_id & ECAT_SLAVE_ID_SUB) {
                        if (_pec->slaves[slave_id].subdev_cnt > (unsigned)sub_slave_id) {
                            pd->pd = _pec->slaves[slave_id].subdevs[sub_slave_id].pdout.pd;
                            pd->len = _pec->slaves[slave_id].subdevs[sub_slave_id].pdout.len;
                        }
                    } else {
                        pd->pd = _pec->slaves[slave_id].pdout.pd;
                        pd->len = _pec->slaves[slave_id].pdout.len;
                    }
                }
            }

            log(verbose, "GET_PDOUT: %p/%d\n", pd->pd, pd->len);
            break;
        }
        case MOD_REQUEST_SET_PDOUT:
            ret = set_pdout((set_pd_t *)ptr);
            break;
        case MOD_REQUEST_GET_PD_COOKIE:
            *(uint64_t **)ptr = &pd_cookie;
            break;
        case MOD_REQUEST_FILE_READ: {
            file_readwrite_info_t *frwi = (file_readwrite_info_t *)ptr;
            uint32_t password = 0;
            char file_name[MAX_FILE_NAME_SIZE];
            strncpy(file_name, frwi->file_name, MAX_FILE_NAME_SIZE-1);
            if (frwi->password)
                password = strtol(frwi->password, NULL, 10);

            ret = ec_foe_read(
                    _pec,                   // ethercat master device
                    frwi->slave_id,         // slave index
                    password,               // file password
                    file_name,              // file name
                    &frwi->file_data,       // returns file_data
                    &frwi->file_data_len,   // returns file_data_len
                    &frwi->error_message);  // returns error_message

            break;
        }
        case MOD_REQUEST_FILE_WRITE: {
            file_readwrite_info_t *frwi = (file_readwrite_info_t *)ptr;
            uint32_t password = 0;
            char file_name[MAX_FILE_NAME_SIZE];
            strncpy(file_name, frwi->file_name, MAX_FILE_NAME_SIZE-1);
            if (frwi->password)
                password = strtol(frwi->password, NULL, 10);

            ret = ec_foe_write(
                    _pec,                   // ethercat master device
                    frwi->slave_id,         // slave index
                    password,               // file password
                    file_name,              // file name
                    frwi->file_data,        // file_data
                    frwi->file_data_len,    // file_data_len
                    &frwi->error_message);  // returns error_message

            break;
        }
        case MOD_REQUEST_MEMORY_READ:
        case MOD_REQUEST_MEMORY_WRITE:
        case MOD_REQUEST_MEMORY_GET_INFO: {
            memory_t *memory_req = (memory_t *)ptr;
            slave::mem_type_t type = (slave::mem_type_t)ECAT_SLAVE_ID_GET_MEM_TYPE(memory_req->slave_id);
            int slave_id = ECAT_SLAVE_ID_GET_SLAVE(memory_req->slave_id);

            if (_slave_info.find(slave_id) != _slave_info.end())
                _slave_info[slave_id]->memory_request(reqcode, type, memory_req);
            else 
                ret = -1;

            break;
        }
        case MOD_REQUEST_SET_TRIGGER_CB: {
            set_trigger_cb_t *cb = (set_trigger_cb_t *)ptr;
            if (cb->cb == NULL) {
                log(error, "ERROR could not register, callback is NULL\n");
                break;
            }

            add_trigger_module(*cb);
            break;
        }
        case MOD_REQUEST_UNSET_TRIGGER_CB: {
            set_trigger_cb_t *cb = (set_trigger_cb_t *)ptr;

            if (cb->cb == NULL) {
                log(error, "ERROR could not remove, callback is NULL\n");
                break;
            }

            remove_trigger_module(*cb);
            break;
        }
        case MOD_REQUEST_TRIGGERED_BY: {
            char **mdl_name = (char **)ptr;

            log(info, "our trigger module is %s\n", *mdl_name);
            trigger_mod_name = *mdl_name;
            break;
        }
        case MOD_REQUEST_CANOPEN_OBJECT_DICTIONARY_LIST: {
            canopen_object_dictionary_list *list = (canopen_object_dictionary_list *)ptr;

            if (list->slave_id & ECAT_SLAVE_ID_EEPROM) {
                // search in eeprom entries
                uint16_t slave_id = ECAT_SLAVE_ID_GET_SLAVE(list->slave_id);
                list->indices_cnt = 0;

                if (list->indices)
                    list->indices[list->indices_cnt++] = 0x1008;
                else
                    list->indices_cnt += 1;

                ec_slave_t *slv = &_pec->slaves[slave_id];
                ec_eeprom_cat_pdo_t *pdo;

                // inputs and outputs
                TAILQ_FOREACH(pdo, &slv->eeprom.txpdos, qh) {
                    if (list->indices)
                        list->indices[list->indices_cnt++] = pdo->pdo_index;
                    else list->indices_cnt++;
                }

                TAILQ_FOREACH(pdo, &slv->eeprom.rxpdos, qh) {
                    if (list->indices)
                        list->indices[list->indices_cnt++] = pdo->pdo_index;
                    else list->indices_cnt++;
                }
            } else {
                if (_pec->slaves[list->slave_id].eeprom.mbx_supported & EC_EEPROM_MBX_COE) {
                    uint8_t buf[512];
                    size_t len = sizeof(buf);
                    int ret = ec_coe_odlist_read(_pec, list->slave_id, buf, &len);

                    if (ret <= 0)
                        break;

                    if (list->indices) {
                        memcpy(list->indices, buf, len);
                        list->indices_cnt = len/2;
                    } else
                        list->indices_cnt = len/2;
                }
            }

            break;
        }
        case MOD_REQUEST_CANOPEN_READ_OBJECT_DESC: {
            int ret2;
            canopen_object_description *desc = (canopen_object_description *)ptr;
            ec_eeprom_cat_pdo_t *pdo;

            if (desc->slave_id & ECAT_SLAVE_ID_EEPROM) {
                // search in eeprom entries
                uint16_t slave_id = ECAT_SLAVE_ID_GET_SLAVE(desc->slave_id);
                ec_slave_t *slv = &_pec->slaves[slave_id];

                if (desc->index == 0x1008) {
                    desc->data_type         = 0x0009;
                    desc->object_code       = 7;
                    desc->max_subindices    = 0;
                    desc->name_len          = strlen("Device Name");
                    desc->name              = (char *)malloc(desc->name_len);
                    strncpy(desc->name, "Device Name", desc->name_len);
                    return 0;
                }

                struct ec_eeprom_cat_pdo_queue *pdos[] = {
                    &slv->eeprom.txpdos,
                    &slv->eeprom.rxpdos };

                for (int qi = 0; qi < 2; qi++) {
                    TAILQ_FOREACH(pdo, pdos[qi], qh) {
                        if (pdo->pdo_index == desc->index) {
                            desc->data_type         = DEFTYPE_PDOMAPPING;
                            desc->object_code       = pdo->n_entry > 1 ? 9 : 7;
                            desc->max_subindices    = pdo->n_entry;

                            if ((pdo->name_idx > 0) && 
                                    (pdo->name_idx <= slv->eeprom.strings_cnt)) {
                                desc->name_len      = strlen(slv->eeprom.strings[pdo->name_idx-1]);
                                desc->name          = (char *)malloc(desc->name_len);
                                strncpy(desc->name, slv->eeprom.strings[pdo->name_idx-1], desc->name_len);
                            } else {
                                desc->name_len = 0;
                            }

                            return 0;
                        }
                    }
                }

                ret = -1;
            } else {
                if (_pec->slaves[desc->slave_id].eeprom.mbx_supported & EC_EEPROM_MBX_COE) {
                    // get description
                    ec_coe_sdo_desc_t obj_desc;
                    ret2 = ec_coe_sdo_desc_read(_pec, desc->slave_id, desc->index, &obj_desc);

                    desc->data_type      = obj_desc.data_type;
                    desc->object_code    = obj_desc.obj_code;
                    desc->max_subindices = obj_desc.max_subindices;
                    desc->name_len       = obj_desc.name_len;                
                    desc->name           = obj_desc.name; // allocated by ec_coe_sdo_desc_read, 
                    // freed by caller
                }
            }
                
            break;
        }
        case MOD_REQUEST_CANOPEN_READ_ELEMENT_DESC: {
            int ret2;

            canopen_element_description *desc = (canopen_element_description *)ptr;
            ec_eeprom_cat_pdo_t *pdo;

            if (desc->slave_id & ECAT_SLAVE_ID_EEPROM) {
                // search in eeprom entries
                uint16_t slave_id = ECAT_SLAVE_ID_GET_SLAVE(desc->slave_id);
                ec_slave_t *slv = &_pec->slaves[slave_id];

                // device name
                if (desc->index == 0x1008) {
                    if ((slv->eeprom.strings_cnt > 0) && 
                            (slv->eeprom.general.name_idx <= slv->eeprom.strings_cnt)) {
                        desc->value_info = 0x7F;
                        desc->data_type = 0x0009;
                        desc->bit_length = strlen(slv->eeprom.strings[slv->eeprom.general.name_idx-1]) * 8;
                        desc->obj_access = 7;
                    } else {
                        desc->value_info = 0x7F;
                        desc->data_type = 0x0009;
                        desc->bit_length = 7*8;
                        desc->obj_access = 7;
                    }

                    return 0;
                }
            
                struct ec_eeprom_cat_pdo_queue *pdos[] = {
                    &slv->eeprom.txpdos,
                    &slv->eeprom.rxpdos };

                for (int qi = 0; qi < 2; qi++) {
                    TAILQ_FOREACH(pdo, pdos[qi], qh) {
                        if (pdo->pdo_index != desc->index)
                            continue;

                        if (desc->sub_index == 0) {
                            desc->value_info        = 0x7F;
                            desc->data_type         = 0x0005; //DEFTYPE_UNSIGNED8;
                            desc->bit_length        = 8;
                            desc->obj_access        = 7;
                            desc->name_len          = strlen("SubIndex_0");
                            desc->name              = strdup("SubIndex_0");

                            return 0;
                        } else if (desc->sub_index <= pdo->n_entry) {
                            ec_eeprom_cat_pdo_entry_t *entry = &pdo->entries[desc->sub_index-1];

                            desc->value_info        = 0x7F;
                            desc->data_type         = entry->data_type;
                            desc->bit_length        = entry->bit_len;
                            desc->obj_access        = 7;

                            if ((entry->entry_name_idx > 0) &&
                                    (entry->entry_name_idx <= slv->eeprom.strings_cnt)) {
                                char *tmp = slv->eeprom.strings[entry->entry_name_idx-1];
                                desc->name_len = min(strlen(tmp), CANOPEN_MAXNAME - 1);
                                desc->name = (char *)malloc(desc->name_len + 1);
                                memcpy(desc->name, tmp, desc->name_len);
                                desc->name[desc->name_len] = '\0';
                            } else {
                                desc->name_len = 0;
                                desc->name = NULL;
                            }

                            return 0;
                        }
                    }
                }
                
                ret = -1;
            } else {
                if (_pec->slaves[desc->slave_id].eeprom.mbx_supported & EC_EEPROM_MBX_COE) {
                    // get description
                    ec_coe_sdo_entry_desc_t entry_desc;
                    entry_desc.data = NULL;
                    ret2 = ec_coe_sdo_entry_desc_read(_pec, desc->slave_id, desc->index, 
                            desc->sub_index, 0x7F, &entry_desc);
                    if (ret2 <= 0) {
                        ret = -1;
                        break;
                    }

                    entry_desc.data = (uint8_t *)malloc(entry_desc.data_len);
                    ret2 = ec_coe_sdo_entry_desc_read(_pec, desc->slave_id, desc->index, 
                            desc->sub_index, 0x7F, &entry_desc);
                    if (ret2 <= 0) {
                        ret = -1;
                        break;
                    }

                    desc->value_info    = 0x7F;
                    desc->data_type     = entry_desc.data_type;
                    desc->bit_length    = entry_desc.bit_length;
                    desc->obj_access    = entry_desc.obj_access;
                    desc->name_len      = entry_desc.data_len;                
                    if (desc->name_len > 0) {
                        desc->name          = (char *)malloc(entry_desc.data_len + 1);
                        memcpy(desc->name, &entry_desc.data[0], desc->name_len);
                        desc->name[desc->name_len] = '\0';
                    } else
                        desc->name      = NULL;

                    if (entry_desc.data)
                        free(entry_desc.data);
                }
            }
            break;
        }
        case MOD_REQUEST_CANOPEN_READ_ELEMENT_VALUE: {
            canopen_element_value *value = (canopen_element_value *)ptr;
            size_t size = value->value_len;
            uint32_t abort_code = 0;

//            log(verbose, "CANOPEN_READ_ELEMENT slave %d: index 0x%X, "
//                    "sub_index %d, want to read %d bytes\n", value->slave_id, value->index,
//                    value->sub_index, value->value_len);

            if (value->slave_id & ECAT_SLAVE_ID_EEPROM) {
                // search in eeprom entries
                uint16_t slave_id = ECAT_SLAVE_ID_GET_SLAVE(value->slave_id);
                ec_slave_t *slv = &_pec->slaves[slave_id];
                memset(value->value, 0, value->value_len);

                if (value->index == 0x1008) {
                    if ((slv->eeprom.strings_cnt > 0) &&
                            (slv->eeprom.general.name_idx <= slv->eeprom.strings_cnt)) {
                        value->value_len = strlen(slv->eeprom.strings[slv->eeprom.general.name_idx-1]);
                        memcpy(value->value, slv->eeprom.strings[slv->eeprom.general.name_idx-1], 
                                value->value_len);

                        return 0;
                    } 
                } else {
                    ec_eeprom_cat_pdo_t *pdo;
                    struct ec_eeprom_cat_pdo_queue *pdos[] = {
                        &slv->eeprom.txpdos,
                        &slv->eeprom.rxpdos };

                    for (int qi = 0; qi < 2; qi++) {
                        TAILQ_FOREACH(pdo, pdos[qi], qh) {
                            if (pdo->pdo_index != value->index)
                                continue;

                            if (value->sub_index == 0) {
                                value->value_len = 1;
                                memcpy(value->value, &pdo->n_entry, 1);
                                return 0;
                            } else if (value->sub_index <= pdo->n_entry) {
                                ec_eeprom_cat_pdo_entry_t *entry = &pdo->entries[value->sub_index-1];
                                value->value_len = 2;
                                memcpy(value->value, &entry->entry_index, 2);
                                return 0;
                            }
                        } 
                    }
                }

                ret = -1;
            } else {
                if (_pec->slaves[value->slave_id].eeprom.mbx_supported & EC_EEPROM_MBX_COE) {
                    ec_coe_sdo_read(_pec, value->slave_id, value->index, value->sub_index, 
                            0, (uint8_t *)value->value, &size, &abort_code);
                    value->value_len = size;
                    ret = abort_code;
                } 
            }
            break;
        }
        case MOD_REQUEST_CANOPEN_WRITE_ELEMENT_VALUE: {
            canopen_element_value *value = (canopen_element_value *)ptr;
            size_t size = value->value_len;
            uint32_t abort_code = 0;

            log(verbose, "CANOPEN_WRITE_ELEMENT slave %d: index 0x%X, "
                    "sub_index %d, want to write %d bytes\n", value->slave_id, value->index,
                    value->sub_index, value->value_len);

            int i;
            for (i = 0; i < value->value_len; ++i)
                printf("%02X ", ((uint8_t *)value->value)[i]);
            printf("\n");

            if (_pec->slaves[value->slave_id].eeprom.mbx_supported & EC_EEPROM_MBX_COE) {
                ec_coe_sdo_write(_pec, value->slave_id, value->index, value->sub_index, 
                        0, (uint8_t *)value->value, &size, &abort_code);
                value->value_len = size;
                ret = abort_code;
            } else // search in eeprom entries
                memset(value->value, 0, value->value_len);
            break;
        }
        case MOD_REQUEST_SERCOS_SERVICE_TRANSFER: {
            sercos_service_transfer *t = (sercos_service_transfer *)ptr;
                
            int atn = ECAT_SLAVE_ID_GET_SUB(t->slave_id),
                slave_id = ECAT_SLAVE_ID_GET_SLAVE(t->slave_id);

            size_t byte_len = t->buflen * 2;

            if (t->direction == SSD_MASTER_TO_DRIVE) {
                if (ec_soe_write(_pec, slave_id, atn, t->idn, 
                            t->element >> 1, (uint8_t *)t->buf, byte_len) == 1)
                    ret = 0; 
            } else {
                if (ec_soe_read(_pec, slave_id, atn, t->idn, 
                            t->element >> 1, (uint8_t *)t->buf, &byte_len) == 1) {
                    ret = 0; 
                }
            }
            break;
        }
        case MOD_REQUEST_SERCOS_SET_COMMAND: {
            sercos_set_command_t *cmd = (sercos_set_command_t *)ptr;
            int atn = ECAT_SLAVE_ID_GET_SUB(cmd->slave_id),
                slave_id = ECAT_SLAVE_ID_GET_SLAVE(cmd->slave_id);

            uint16_t val = 1;
            size_t val_len = sizeof(val);
            ret = ec_soe_write(_pec, slave_id, atn, cmd->cmd, 0x80 >> 1, 
                    (uint8_t *)&val, val_len);
            if (ret != 0)
                log(warning, "setting command %d returned %d\n", cmd->cmd, ret);

            val = 3;
            val_len = sizeof(val);
            ret = ec_soe_write(_pec, slave_id, atn, cmd->cmd, 0x80 >> 1, 
                    (uint8_t *)&val, val_len);
            if (ret != 0)
                log(warning, "exec command %d returned %d\n", cmd->cmd, ret);
            
            val = 0;
            val_len = sizeof(val);
            ret = ec_soe_write(_pec, slave_id, atn, cmd->cmd, 0x80 >> 1, 
                    (uint8_t *)&val, val_len);
            if (ret != 0)
                log(warning, "resetting command %d returned %d\n", cmd->cmd, ret);

            ret = 0;
            break;            
        }
        default:
            ret = -1;
            break;
    }

    return ret;
}

//! module trigger callback
void master::trigger() {
    int i = 0;
    int64_t max_timeout = 0;
    ec_timer_t dc_timeout;

    if (state >= module_state_safeop) {
        for (i = 0; i < _pec->pd_group_cnt; ++i) {
            group *g = _group_info[i];
            if ((++g->_divisor_cnt % g->_divisor) != 0)
                continue; 

            // reset divisor cnt and queue datagram
            g->_divisor_cnt = 0;
            ec_send_process_data_group(_pec, i);
            ec_timer_init(&g->timeout, g->_recv_timeout);

            if (max_timeout < g->_recv_timeout)
                max_timeout = g->_recv_timeout;
        }

        if (_pec->dc.have_dc) {
            ec_send_distributed_clocks_sync(_pec);
            ec_timer_init(&dc_timeout, max_timeout);
        }
    }

    hw_tx(_pec->phw);

    if (state >= module_state_safeop) {

        pd_cookie++;
        pthread_cond_signal(&pd_cond);

        for (i = 0; i < _pec->pd_group_cnt; ++i) {
            group *g = _group_info[i];

            if (g->_divisor_cnt != 0)
                continue; 

            ec_receive_process_data_group(_pec, i, &g->timeout);

            trigger_modules(ECAT_SLAVE_ID_GROUP | g->_index);

            for (std::list<int>::iterator it = g->_slaves.begin(); it != g->_slaves.end(); ++it)
                trigger_modules(*it);
        }
        
        int slave;
        for (slave = 0; slave < _pec->slave_cnt; ++slave) {
            ec_slave_t *slv = &_pec->slaves[slave];

            if (slv->eeprom.mbx_supported && slv->mbx_read.sm_state) {
                if (*slv->mbx_read.sm_state & 0x08) {
                    pthread_cond_signal(&async_cond);
                    break;
                }
            }
        }

        if (_pec->dc.have_dc) {
            ec_receive_distributed_clocks_sync(_pec, &dc_timeout);

            if (_pec->dc.offset_compensation_cnt == 0)
                log(verbose, "dc receive, mode %d\n", _pec->dc.mode);

            if (    (_pec->dc.mode == ec_dc_info::dc_mode_ref_clock) && 
                    (_pec->dc.offset_compensation_cnt == 0)) {
                double diff = (_pec->dc.act_diff / 1E9);

                if (!_dc_sync.first_run) {
                    double tmp;
                    kernel::request_cb(trigger_mod_name.c_str(), 
                            MOD_REQUEST_GET_TRIGGER_INTERVAL, &tmp);

                    tmp -= (-0.1 * (diff/_pec->dc.offset_compensation) ) + 
                        (_dc_sync.last_diff - diff)/(_pec->dc.offset_compensation);

                    kernel::request_cb(trigger_mod_name.c_str(), 
                            MOD_REQUEST_SET_TRIGGER_INTERVAL, &tmp);

                    log(verbose, "setting new clock %13.10f, last_diff %13.10f, diff %13.10f, "
                            "offset_comp %d\n", tmp, _dc_sync.last_diff, diff, 
                            _pec->dc.offset_compensation);
                }

                _dc_sync.first_run = false;
                _dc_sync.last_diff = diff;
            }
        }
    }
}

//! async handler thread
void master::run() {
    log(info, "async handler thread running\n");

    pthread_mutex_lock(&async_lock);

    while (running()) {
        struct timespec timeout;
        ec_timer_t abstime;
        ec_timer_init(&abstime, 100000000);
        timeout.tv_sec = abstime.sec;
        timeout.tv_nsec = abstime.nsec;
//        log(info, "running %d, run_flag %d\n", running(), this->run_flag);

        if (pthread_cond_timedwait(&async_cond, &async_lock, &timeout) != 0)
            continue;

        int slave;
        for (slave = 0; slave < _pec->slave_cnt; ++slave) {
            ec_slave_t *slv = &_pec->slaves[slave];

            if (slv->eeprom.mbx_supported && slv->mbx_read.sm_state) {
                if (slv->mbx_read.skip_next == 1) {
                    slv->mbx_read.skip_next = 0;
                    continue;
                }

                if (pthread_mutex_trylock(&slv->mbx_lock) != 0)
                    continue;

                if (slv->mbx_read.sm_state && ((*slv->mbx_read.sm_state) & 0x08) == 0x08) {
                    log(verbose, "async worker: slave %d read mailbox is full\n", slave);

                    char buf[1024];
                    int wkc = ec_mbx_receive(_pec, slave, EC_DEFAULT_TIMEOUT_MBX);
                    if (wkc) {
                        int cnt = sprintf(buf, "wkc %d: ", wkc);

                        ec_mbx_header_t *mbx_hdr = (ec_mbx_header_t *)(slv->mbx_read.buf);
                        for (unsigned z = 0; z < mbx_hdr->length + sizeof(ec_mbx_header_t); ++z)
                            cnt += snprintf(buf+cnt, 1024 - cnt, "%02X ", slv->mbx_read.buf[z]);
                    
                        log(info, "async worker %s\n", buf);
                    }
                }

                pthread_mutex_unlock(&slv->mbx_lock);
            }
        }
    }

    pthread_mutex_unlock(&async_lock);

    log(info, "async handler thread stopped\n");
}

//! set new pdout pointers
/*!
 * \param pdout new pdout pointers
 * \return 0 on success
 */
int master::set_pdout(set_pd_t *pdout) {
    unsigned int i;

    if(!pdout || !pdout->cnt)
        return 0;

    // calculate differece (without sign, so we do not concert overflows)
    uint64_t difference = pd_cookie - pdout->pd_cookie;

    // check if we are commanding to slow
    if (difference > _cmd_delay) {
        if (_cmd_mode == user_defined) {
            throw robotkernel::str_exception("[module_ethercat|%s] commanding to slow"
                    ": have difference of %llu while configured cmd_delay is %llu\n", 
                    name.c_str(), difference, _cmd_delay);
        }

        _cmd_delay = difference;

        log(warning, "you are commanding to SLOW! Increased cmd_delay to %d!!!\n", 
                (unsigned int)_cmd_delay);
    }

    pthread_mutex_lock(&pd_lock);

    while (difference < _cmd_delay) {
        // need to wait until mdt cnt is big enough
        struct timespec timeout;
        ec_timer_t abstime;
        ec_timer_init(&abstime, 100000000);
        timeout.tv_sec = abstime.sec;
        timeout.tv_nsec = abstime.nsec;

        pthread_cond_timedwait(&pd_cond, &pd_lock, &timeout);
        
        difference = pd_cookie - pdout->pd_cookie;
    }

    pthread_mutex_unlock(&pd_lock);

    for (i = 0; i < pdout->cnt; ++i) {
        uint8_t *to = NULL;
        size_t to_len = 0;

        if (pdout->pd[i].slave_id & ECAT_SLAVE_ID_GROUP) {
            // group case
            int g_nr = ECAT_SLAVE_ID_GET_GROUP(pdout->pd[i].slave_id);
            if (g_nr < _pec->pd_group_cnt) {
                to = _pec->pd_groups[g_nr].pd;
                to_len = _pec->pd_groups[g_nr].pdout_len;
            }
        } else {
            int sub_slave_id = ECAT_SLAVE_ID_GET_SUB(pdout->pd[i].slave_id),
                slave_id = ECAT_SLAVE_ID_GET_SLAVE(pdout->pd[i].slave_id);

            if (slave_id < _pec->slave_cnt) {
                if (pdout->pd[i].slave_id & ECAT_SLAVE_ID_SUB) {
                    if (_pec->slaves[slave_id].subdev_cnt > (unsigned)sub_slave_id) {
                        to = _pec->slaves[slave_id].subdevs[sub_slave_id].pdout.pd;
                        to_len = _pec->slaves[slave_id].subdevs[sub_slave_id].pdout.len;
                    }
                } else {
                    to = _pec->slaves[slave_id].pdout.pd;
                    to_len = _pec->slaves[slave_id].pdout.len;
                }
            }
        }

        if (to && to_len)
            memcpy(to, pdout->pd[i].pd, min(to_len, pdout->pd[i].len));
    }

    return 0;
}

