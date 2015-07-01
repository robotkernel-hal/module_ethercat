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

void ethercat_log_func(int lvl, void *user, const char *format, ...) {
    master *e = (master *)user;
    va_list ap;
    va_start(ap, format);

    robotkernel::loglevel loglvl = module_verbose;
    if (lvl < 100)
        loglvl = module_info;
    if (lvl < 10)
        loglvl = module_warning;
    if (lvl < 1)
        loglvl = module_error;

    ethercat_log(loglvl, e->_name, format, ap);
    va_end(ap);
}

master::group::group(int index, const YAML::Node& node) {
    _index          = index;
    _divisor        = node["divisor"].to<int>();
    _divisor_cnt    = 0;

    for (YAML::Iterator it = node["slaves"].begin(); it != node["slaves"].end(); ++it)
        _slaves.push_back(it->to<int>());
}

//! register interfaces for group
/*!
 * \param ctx ethercat context
 * \return N/A
 */
void master::group::register_interfaces(std::string name) {
    std::stringstream group_name; 
    group_name << "group_" << _index;

    _pd_intf = robotkernel::kernel::register_interface_cb(name.c_str(), 
            "libinterface_process_data_inspection.so", group_name.str().c_str(), _index | 0x80000000);
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
master::master(const std::string& name, const YAML::Node& node) {
    _name       = name;
    _ifname     = node["ifname"].to<string>();
    _recv_prio  = node["recv_prio"].to<int>();
    _recv_mask  = node["recv_mask"].to<int>();
    _pec        = NULL;

    ec_log_func_user = this;
    ec_log_func = ethercat_log_func;

    // group settings
    const YAML::Node *groups_node = node.FindValue("groups");
    if (groups_node) {
        for (YAML::Iterator it = groups_node->begin();
                it != groups_node->end(); ++it) {
            int g_nr = it.first().to<int>();
            _group_info[g_nr] = new group(g_nr, it.second());
        }
    }

    if (node.FindValue("slaves") != NULL) {
        // parsing slave configurations
        const YAML::Node& slaves = node["slaves"];
        for (YAML::Iterator it = slaves.begin(); it != slaves.end(); ++it) {
            slave *slv = new slave(*it, this);
            _slave_info[slv->index] = slv;
        }
    }

    int ret = ec_open(&_pec, _ifname.c_str(), _recv_prio, _recv_mask);
    if (ret != 0) 
        throw str_exception("ec_open failed: %s!\n", strerror(ret));

    set_state(module_state_init);

    // add process data inspection 
    //    std::stringstream channel_name; 
    //    channel_name << "channel_" << (int)0;
    //    _pd_interface_id = robotkernel::kernel::register_interface_cb(_name.c_str(), 
    //            "libinterface_process_data_inspection.so", channel_name.str().c_str(), 0);
}

//! destruction 
master::~master() {
    if (_pec)
        ec_close(_pec);

    _pec = NULL;
}

//! cyclic process data read
/*!
 * \param buf process data buffer
 * \param bufsize size of process data buffer
 * \return size of read bytes
 */
size_t master::read(void* buf, size_t bufsize) {
    return 0;
}

//! set module state machine to defined state
/*!
 * \param state requested state
 * \return success or failure
 */
int master::set_state(module_state_t state) {
    int ret = 0, nr;

    switch (state) {
        case module_state_init: {
            _pec->tx_sync = 1;

            ec_set_state(_pec, EC_STATE_INIT);

            for (nr = 0; nr < _pec->slave_cnt; ++nr) {
                if (_slave_info.find(nr) == _slave_info.end()) {
                    ethercat_log(module_verbose, _name, "slave %d creating empty one\n", nr);

                    slave *slv = new slave(nr, this);
                    _slave_info[nr] = slv;
                }

                if (_slave_info[nr]->dc.has_dc)
                    _pec->slaves[nr].dc.use_dc = 1;
                else 
                    _pec->slaves[nr].dc.use_dc = 0;
            }

            break;
        }
        case module_state_preop: {
            _pec->tx_sync = 1;

            for (nr = 0; nr < _pec->slave_cnt; ++nr) {
                // apply sm and fmmu config
                for (slave::sm_map_t::iterator it = _slave_info[nr]->_sm_map.begin();
                        it != _slave_info[nr]->_sm_map.end(); ++it) {
                    int sm_nr = it->first;

                    if (sm_nr < _pec->slaves[nr].sm_ch) {
                        _pec->slaves[nr].sm[sm_nr].adr = it->second->_address;
                        _pec->slaves[nr].sm[sm_nr].len = it->second->_length;
                        _pec->slaves[nr].sm[sm_nr].flags = it->second->_flags;
                    }
                }
            }

            ec_set_state(_pec, EC_STATE_PREOP);

            for (nr = 0; nr < _pec->slave_cnt; ++nr)
                _slave_info[nr]->register_interfaces();

            break;
        }
        case module_state_safeop:
            ec_create_pd_groups(_pec, _group_info.size());
            for (group_map_t::iterator it = _group_info.begin(); it != _group_info.end(); ++it) {
                int g_nr = it->first;

                for (std::list<int>::iterator it2 = it->second->_slaves.begin();
                        it2 != it->second->_slaves.end(); ++it2) {

                    _pec->slaves[*it2].assigned_pd_group = g_nr;
                    _slave_info[*it2]->prepare_state_transition(preop_to_safeop);
                }

                it->second->register_interfaces(_name);
            }

            ec_set_state(_pec, EC_STATE_SAFEOP);
            _pec->tx_sync = 0;
            break;
        case module_state_op:
            _pec->tx_sync = 0;

            ec_set_state(_pec, EC_STATE_OP);
            break;
        default:
            ret = -1;
            break;
    }

    if (ret == 0)
        _state = state;

    return ret;
}

//! get module state machine state
/*!
 * \return current state
 */
module_state_t master::get_state() {
    return _state;
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

            if (pd->slave_id & 0x80000000) {
                // group case
                int g_nr = pd->slave_id & ~0x80000000; 
                if (g_nr < _pec->pd_group_cnt) {
                    pd->pd = _pec->pd_groups[g_nr].pd + _pec->pd_groups[g_nr].pdout_len;
                    pd->len = _pec->pd_groups[g_nr].pdin_len;
                }
            } else {
                if (pd->slave_id < (unsigned)_pec->slave_cnt) {
                    pd->pd = _pec->slaves[pd->slave_id].pdin;
                    pd->len = _pec->slaves[pd->slave_id].pdin_len;
                }
            }

            ethercat_log(module_verbose, _name, "GET_PDIN: %p/%d\n", pd->pd, pd->len);
            break;
        }
        case MOD_REQUEST_GET_PDOUT: {            
            process_data_t *pd = (process_data_t *)ptr;
            pd->pd = NULL;
            pd->len = 0;

            if (pd->slave_id & 0x80000000) {
                // group case
                int g_nr = pd->slave_id & ~0x80000000; 
                if (g_nr < _pec->pd_group_cnt) {
                    pd->pd = _pec->pd_groups[g_nr].pd;
                    pd->len = _pec->pd_groups[g_nr].pdout_len;
                }
            } else {
                if (pd->slave_id < (unsigned)_pec->slave_cnt) {
                    pd->pd = _pec->slaves[pd->slave_id].pdout;
                    pd->len = _pec->slaves[pd->slave_id].pdout_len;
                }
            }

            ethercat_log(module_verbose, _name, "GET_PDOUT: %p/%d\n", pd->pd, pd->len);
            break;
        }

        case MOD_REQUEST_MEMORY_READ:
        case MOD_REQUEST_MEMORY_WRITE:
        case MOD_REQUEST_MEMORY_GET_INFO: {
            memory_t *memory_req = (memory_t *)ptr;
            if (_slave_info.find(memory_req->slave_id) != _slave_info.end())
                _slave_info[memory_req->slave_id]->memory_request(reqcode, memory_req);
            else 
                ret = -1;

            break;
        }
        case MOD_REQUEST_SET_TRIGGER_CB: {
            set_trigger_cb_t *cb = (set_trigger_cb_t *)ptr;
            if (cb->cb == NULL) {
                ethercat_log(module_error, _name, "ERROR could not register, callback is NULL\n");
                break;
            }

            add_trigger_module(*cb);
            break;
        }
        case MOD_REQUEST_UNSET_TRIGGER_CB: {
            set_trigger_cb_t *cb = (set_trigger_cb_t *)ptr;

            if (cb->cb == NULL) {
                ethercat_log(module_error, _name, "ERROR could not remove, callback is NULL\n");
                break;
            }

            remove_trigger_module(*cb);
            break;
        }
        case MOD_REQUEST_CANOPEN_OBJECT_DICTIONARY_LIST: {
            canopen_object_dictionary_list *list = (canopen_object_dictionary_list *)ptr;

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
            } else { // search in eeprom entries
                list->indices_cnt = 0;

                ec_slave_t *slv = &_pec->slaves[list->slave_id];
                for (unsigned i = 0; i < slv->eeprom.txpdos_cnt; ++i) {
                    if (list->indices)
                        list->indices[list->indices_cnt++] = slv->eeprom.txpdos[i].pdo_index;
                    else list->indices_cnt++;
                }

                for (unsigned i = 0; i < slv->eeprom.rxpdos_cnt; ++i) {
                    if (list->indices)
                        list->indices[list->indices_cnt++] = slv->eeprom.rxpdos[i].pdo_index;
                    else list->indices_cnt++;
                }

                list->indices_cnt /= 2;
            }

            break;
        }
        case MOD_REQUEST_CANOPEN_READ_OBJECT_DESC: {
            int ret2;
            canopen_object_description *desc = (canopen_object_description *)ptr;

            if (_pec->slaves[desc->slave_id].eeprom.mbx_supported & EC_EEPROM_MBX_COE) {
                // get description
                ec_coe_sdo_desc_t obj_desc;
                ret2 = ec_coe_sdo_desc_read(_pec, desc->slave_id, desc->index, &obj_desc);

                desc->data_type      = obj_desc.data_type;
                desc->object_code    = obj_desc.obj_code;
                desc->max_subindices = obj_desc.max_subindices;
                strcpy(desc->name, obj_desc.name);          
            } else { // search in eeprom entries
                bool found = false;

                ec_slave_t *slv = &_pec->slaves[desc->slave_id];
                for (unsigned i = 0; i < slv->eeprom.txpdos_cnt; ++i) {
                    if (slv->eeprom.txpdos[i].pdo_index == desc->index) {
                        found = true;

                        desc->data_type         = slv->eeprom.txpdos[i].data_type;
                        desc->object_code       = 6;
                        desc->max_subindices    = slv->eeprom.txpdos[i].n_entry;
                        strcpy(desc->name, slv->eeprom.strings[slv->eeprom.txpdos[i].name_idx]);
                    }
                }

                if (found)
                    break;

                for (unsigned i = 0; i < slv->eeprom.rxpdos_cnt; ++i) {
                    if (slv->eeprom.rxpdos[i].pdo_index == desc->index) {
                        found = true;

                        desc->data_type         = slv->eeprom.rxpdos[i].data_type;
                        desc->object_code       = 6;
                        desc->max_subindices    = slv->eeprom.rxpdos[i].n_entry;
                        strcpy(desc->name, slv->eeprom.strings[slv->eeprom.rxpdos[i].name_idx]);
                    }
                }
            }
            break;
        }
        case MOD_REQUEST_CANOPEN_READ_ELEMENT_DESC: {
            int ret2;

            canopen_element_description *desc = (canopen_element_description *)ptr;

            if (_pec->slaves[desc->slave_id].eeprom.mbx_supported & EC_EEPROM_MBX_COE) {
                // get description
                ec_coe_sdo_entry_desc_t entry_desc;
                entry_desc.data = NULL;
                ret2 = ec_coe_sdo_entry_desc_read(_pec, desc->slave_id, desc->index, 
                        desc->sub_index, 0x7F, &entry_desc);
                if (ret2 <= 0)
                    break;

                entry_desc.data = (uint8_t *)malloc(entry_desc.data_len);
                ec_coe_sdo_entry_desc_read(_pec, desc->slave_id, desc->index, 
                        desc->sub_index, 0x7F, &entry_desc);

                desc->value_info    = 0x7F;
                desc->data_type     = entry_desc.data_type;
                desc->bit_length    = entry_desc.bit_length;
                desc->obj_access    = entry_desc.obj_access;

                size_t name_len = min(entry_desc.data_len, CANOPEN_MAXNAME - 1);
                memcpy(desc->name, &entry_desc.data[0], name_len);
                desc->name[name_len] = '\0';

                free(entry_desc.data);
            } else { // search in eeprom entries
                bool found = false;

                ec_slave_t *slv = &_pec->slaves[desc->slave_id];
                for (unsigned i = 0; i < slv->eeprom.txpdos_cnt; ++i) {
                    if (slv->eeprom.txpdos[i].pdo_index == desc->index) {
                        found = true;

                        desc->value_info        = 0x7F;
                        desc->data_type         = slv->eeprom.txpdos[i].data_type;
                        desc->bit_length        = slv->eeprom.txpdos[i].bit_len;
                        desc->obj_access        = 7;

                        size_t name_len = min(
                                strlen(slv->eeprom.strings[slv->eeprom.txpdos[i].name_idx]), CANOPEN_MAXNAME - 1);
                        memcpy(desc->name, slv->eeprom.strings[slv->eeprom.txpdos[i].name_idx], name_len);
                        desc->name[name_len] = '\0';
                    }
                }

                if (found)
                    break;

                for (unsigned i = 0; i < slv->eeprom.rxpdos_cnt; ++i) {
                    if (slv->eeprom.rxpdos[i].pdo_index == desc->index) {
                        found = true;

                        desc->value_info        = 0x7F;
                        desc->data_type         = slv->eeprom.rxpdos[i].data_type;
                        desc->bit_length        = slv->eeprom.rxpdos[i].bit_len;
                        desc->obj_access        = 7;

                        size_t name_len = min(
                                strlen(slv->eeprom.strings[slv->eeprom.rxpdos[i].name_idx]), CANOPEN_MAXNAME - 1);
                        memcpy(desc->name, slv->eeprom.strings[slv->eeprom.rxpdos[i].name_idx], name_len);
                        desc->name[name_len] = '\0';
                    }
                }
            }
            break;
        }
        case MOD_REQUEST_CANOPEN_READ_ELEMENT_VALUE: {
            canopen_element_value *value = (canopen_element_value *)ptr;
            size_t size = value->value_len;
            uint32_t abort_code = 0;

            ethercat_log(module_verbose, "MOD_REQUEST_CANOPEN_READ_ELEMENT_VALUE", "slave %d: index 0x%X, "
                    "sub_index %d, want to read %d bytes\n", value->slave_id, value->index,
                    value->sub_index, value->value_len);

            if (_pec->slaves[value->slave_id].eeprom.mbx_supported & EC_EEPROM_MBX_COE) {
                ec_coe_sdo_read(_pec, value->slave_id, value->index, value->sub_index, 
                        0, (uint8_t *)value->value, &size, &abort_code);
                value->value_len = size;
                ret = abort_code;
            } else // search in eeprom entries
                memset(value->value, 0, value->value_len);
            break;
        }
        case MOD_REQUEST_CANOPEN_WRITE_ELEMENT_VALUE: {
            canopen_element_value *value = (canopen_element_value *)ptr;
            size_t size = value->value_len;
            uint32_t abort_code = 0;

            ethercat_log(module_verbose, "MOD_REQUEST_CANOPEN_WRITE_ELEMENT_VALUE", "slave %d: index 0x%X, "
                    "sub_index %d, want to read %d bytes\n", value->slave_id, value->index,
                    value->sub_index, value->value_len);

            if (_pec->slaves[value->slave_id].eeprom.mbx_supported & EC_EEPROM_MBX_COE) {
                ec_coe_sdo_write(_pec, value->slave_id, value->index, value->sub_index, 
                        0, (uint8_t *)value->value, &size);
                value->value_len = size;
                ret = abort_code;
            } else // search in eeprom entries
                memset(value->value, 0, value->value_len);
            break;
        }
        case MOD_REQUEST_SERCOS_SERVICE_TRANSFER: {
            sercos_service_transfer *t = (sercos_service_transfer *)ptr;

            if (t->direction == SSD_MASTER_TO_DRIVE)
                ret = ec_soe_write(_pec, t->slave_id >> 16, t->slave_id & 0x0000FFFFF,
                        t->idn, t->element, (uint8_t *)t->buf, t->buflen); 
            else 
                ret = ec_soe_read(_pec, t->slave_id >> 16, t->slave_id & 0x0000FFFFF,
                        t->idn, t->element, (uint8_t *)t->buf, &t->buflen); 
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
    ec_timer_t timeout;
    ec_timer_init(&timeout, 10000000);

    if (_state >= module_state_safeop) {
        for (i = 0; i < _pec->pd_group_cnt; ++i) {
            group *g = _group_info[i];
            if ((++g->_divisor_cnt % g->_divisor) != 0)
                continue; 

            // reset divisor cnt and queue datagram
            g->_divisor_cnt = 0;
            ec_send_process_data_group(_pec, i);
        }

        if (_pec->dc.have_dc) 
            ec_send_distributed_clocks_sync(_pec);
    }

    hw_tx(_pec->phw);

    if (_state >= module_state_safeop) {
        for (i = 0; i < _pec->pd_group_cnt; ++i) {
            group *g = _group_info[i];

            if (g->_divisor_cnt != 0)
                continue; 

            ec_receive_process_data_group(_pec, i, &timeout);

            for (std::list<int>::iterator it = g->_slaves.begin(); it != g->_slaves.end(); ++it)
                trigger_modules(*it);
        }

        if (_pec->dc.have_dc)
            ec_receive_distributed_clocks_sync(_pec, &timeout);
    }
}

