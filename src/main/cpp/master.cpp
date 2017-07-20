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

MODULE_DEF(module_ethercat, module_ethercat::master)

using namespace std;
using namespace robotkernel;
using namespace string_util;
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


//! construction
/*!
 * \param node yaml intialization node
 */
master::master(const std::string& name, const YAML::Node& node) 
    : module_base("module_ethercat", name, node), 
      runnable(node), pec(NULL) 
{
#define get_yaml(t, n, d) \
    n = get_as<t>(node, #n, d);

    get_yaml(string,   ifname, "");
    get_yaml(int,      recv_prio, 0);
    get_yaml(int,      recv_mask, 0xff);
    get_yaml(bool,     log_eeprom_data, false);
    get_yaml(int,      trigger_interval, 0);
    get_yaml(bool,     threaded_startup, true);
    get_yaml(int,      dc_offset_compensation_cycles, 250);
    get_yaml(int,      dc_timer_override, -1);
    get_yaml(uint64_t, dc_offset_compensation_max, 100000000);

    ec_log_func_user = this;
    ec_log_func = log_func;

    // group settings
    if (node["groups"]) {
        for (YAML::const_iterator it = node["groups"].begin();
                it != node["groups"].end(); ++it) {
            int g_nr = it->first.as<int>();
            groups[g_nr] = make_shared<group>(this, g_nr, it->second);
            kernel::get_instance()->add_device(groups[g_nr]);
        }
    }

    if (node["slaves"] != NULL) {
        // parsing slave configurations
        const YAML::Node& slaves = node["slaves"];
        for (YAML::const_iterator it = slaves.begin(); it != slaves.end(); ++it) {
            std::shared_ptr<slave> slv = make_shared<slave>(*it, this);
            _slave_info[slv->index] = slv;
        }
    }

    _dc_mode_string = get_as<string>(node, "dc_mode", "master_clock");
    dc_sync.first_run = true;
    dc_sync.last_diff = 0.;

    pthread_mutex_init(&pd_lock, NULL);
    pthread_cond_init(&pd_cond, NULL);
   
    pthread_mutex_init(&async_lock, NULL);
    pthread_cond_init(&async_cond, NULL);

    pd_cookie = 0;
    
    // -----------------------------------------------------------
    // open ethercat interface
    int ret = ec_open(&pec, ifname.c_str(), recv_prio, recv_mask, log_eeprom_data);
    if (ret != 0) 
        throw str_exception("ec_open failed: %s!\n", strerror(ret));
        
    pec->threaded_startup = threaded_startup;

    // perform init_2_init transition
    set_state(module_state_init);
}

void master::open() {
    // -----------------------------------------------------------
    // setting init commands and distributed clocks
    for (slave_map_t::iterator it = _slave_info.begin(); 
            it != _slave_info.end(); ++it) {
        int slave_nr = it->first;
        sp_slave_t slv = it->second;

        if (pec->slave_cnt > slave_nr) {
            for (slave::coe_list_t::iterator it2 = slv->coe_init_cmds.begin();
                    it2 != slv->coe_init_cmds.end(); ++it2) {
                slave::coe_init_cmd_t *cmd = *it2;
                ec_slave_add_init_cmd(pec, slave_nr, EC_MBX_COE, 
                        (int)cmd->transition, cmd->index, cmd->subindex, 
                        cmd->ca, cmd->data, cmd->datalen);

                cmd->already_added = true;
            }
        } else {
            log(warning, "setting inits for slave %2d failed. no slave found!\n", slave_nr);
            continue;
        }
                
        if (slv->dc.has_dc) {
            pec->slaves[slave_nr].dc.use_dc        = 1;
            pec->slaves[slave_nr].dc.type          = slv->dc.type;
            pec->slaves[slave_nr].dc.cycle_time_0  = slv->dc.cycle_time_0;
            pec->slaves[slave_nr].dc.cycle_time_1  = slv->dc.cycle_time_1;
            pec->slaves[slave_nr].dc.cycle_shift   = slv->dc.cycle_shift;
        } else 
            pec->slaves[slave_nr].dc.use_dc = 0;
    }

    // -----------------------------------------------------------
    // creating and assigning process data groups
    ec_create_pd_groups(pec, groups.size());
            
    for (group_map_t::iterator it = groups.begin(); it != groups.end(); ++it) {
        int g_nr = it->first;

        for (std::list<int>::iterator it2 = it->second->_slaves.begin();
                it2 != it->second->_slaves.end(); ++it2) {

            int s_nr = *it2;
            if (pec->slave_cnt <= s_nr) {
                log(warning, "slave %d not connected to ethercat bus, "
                        "not adding to group %d\n", s_nr, g_nr);
                continue;
            }

            pec->slaves[s_nr].assigned_pd_group = g_nr;
        }
    }
    
    // -----------------------------------------------------------
    // set pdo mapping entries
    for (slave_map_t::iterator it = _slave_info.begin(); 
            it != _slave_info.end(); ++it) {
        int slave_nr = it->first;
        sp_slave_t slv = it->second;

        log(verbose, "trying to create mapping for slave %d\n", slave_nr);

        if (pec->slave_cnt > slave_nr) {
            // generate input mapping for coe
            int mapping_entries = slv->input_mapping.size();
            if (mapping_entries > 0) {
                uint16_t mapping[mapping_entries + 1];
                int mnr = 0;
                mapping[mnr++] = mapping_entries;
                for (slave::mapping_t::iterator mit = slv->input_mapping.begin(); 
                        mit != slv->input_mapping.end(); ++mit) {
                    mapping[mnr++] = *mit;

                    log(verbose, "adding input mapping for slave %d: 0x%08X\n", 
                            slave_nr, *mit);
                }

                ec_slave_add_init_cmd(pec, slave_nr, EC_MBX_COE, 0x24, 0x1C13, 
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

                    log(verbose, "adding output mapping for slave %d: 0x%08X\n", 
                            slave_nr, *mit);
                }

                ec_slave_add_init_cmd(pec, slave_nr, EC_MBX_COE, 0x24, 0x1C12, 
                        0, 1, (char *)mapping, 2 * (mapping_entries + 1));
            }
        } else {
            log(error, "setting mapping for slave %d, failed. no slave found!\n", slave_nr);
        }
    }
            
    for (int nr = 0; nr < pec->slave_cnt; ++nr) {
        if (_slave_info.find(nr) == _slave_info.end()) {
            log(verbose, "slave %d creating empty one\n", nr);

            wp_slave_t slv = make_shared<slave>(nr, this);
            _slave_info[nr] = slv;
        }

        sp_slave_t slv = _slave_info[nr];

        if (slv->dc.has_dc)
            pec->slaves[nr].dc.use_dc = 1;
        else 
            pec->slaves[nr].dc.use_dc = 0;

        slv->post_state_transition(module_state_init, module_state_init);
    }
}

//! destruction 
master::~master() {
    if (pec)
        ec_close(pec);

    pec = NULL;
    for (auto& kv : _slave_info) {
        kv.second->clean_up();
        kv.second = nullptr;
    }

    for (auto& kv : groups)
        kv.second = nullptr;
    
    pthread_mutex_destroy(&async_lock);
    pthread_cond_destroy(&async_cond);
    
    pthread_mutex_destroy(&pd_lock);
    pthread_cond_destroy(&pd_cond);
}

//! set module state machine to defined state
/*!
 * \param state requested state
 * \return success or failure
 */
int master::set_state(module_state_t state) {
    kernel& k = *kernel::get_instance();

    log(info, "setting state from %s to %s\n", 
            state_to_string(this->state), state_to_string(state));

    // get transition
    uint32_t transition = GEN_STATE(this->state, state);

#define STATE_TRANSITION(slave_func, to) { \
    for (int nr = 0; nr < pec->slave_cnt; ++nr) { \
        if (_slave_info.find(nr) == _slave_info.end()) continue; \
        sp_slave_t slv = _slave_info[nr]; \
        try { \
        slv->slave_func##_state_transition(this->state, to); \
        } catch (exception& e) { \
            log(warning, e.what()); \
        }}} 

    switch (transition) {
        case op_2_safeop:
        case op_2_preop:
        case op_2_init:
        case op_2_boot:
            // ====> stop sending commands
            if (state == module_state_safeop)
                break;
        case safeop_2_preop:
        case safeop_2_init:
        case safeop_2_boot:
            // ====> stop receiving measurements

            // remove group trigger devices
            for (const auto& kv : groups)
                k.remove_device(kv.second);

            stop();
            pec->tx_sync = 1;

            if (state == module_state_preop)
                break;
        case preop_2_init:
        case preop_2_boot:
            // ====> deinit devices
            t_dev = nullptr;
        case init_2_init:
            // ====> re-/open ethercat device

            STATE_TRANSITION(pre, module_state_init);
            ec_set_state(pec, EC_STATE_INIT);
            STATE_TRANSITION(post, module_state_init);
            
            try {
                open();
            } catch (exception& e) {
                log(error, e.what());
                state = module_state_init;
            }

            if (state == module_state_init)
                break;
        case init_2_boot:
            STATE_TRANSITION(pre, module_state_boot);
            ec_set_state(pec, EC_STATE_BOOT);
            STATE_TRANSITION(post, module_state_boot);
            break;
        case boot_2_init:
        case boot_2_preop:
        case boot_2_safeop:
        case boot_2_op:
            // ====> re-/open ethercat device
            try {
                open();
            } catch (exception& e) {
                log(error, e.what());
                state = module_state_init;
            }

            STATE_TRANSITION(pre, module_state_init);
            ec_set_state(pec, EC_STATE_INIT);
            STATE_TRANSITION(post, module_state_init);

            if (state == module_state_init)
                break;
        case init_2_op:
        case init_2_safeop:
        case init_2_preop:
            pec->dc.mode = _dc_mode_string == "ref_clock" ? 
                ec_dc_info::dc_mode_ref_clock : ec_dc_info::dc_mode_master_clock;
            
            STATE_TRANSITION(pre, module_state_preop);
            ec_set_state(pec, EC_STATE_PREOP);
            STATE_TRANSITION(post, module_state_preop);

            if (dc_offset_compensation_cycles > 0)
                pec->dc.offset_compensation = dc_offset_compensation_cycles;

            if (dc_timer_override > 0)
                pec->dc.timer_override = dc_timer_override;
            else {
                auto mdl = get_module();
                if (mdl->triggers.size() != 1) {
                    log(warning, "we have %d trigger devices set by robotkernel, "
                            "please use only 1!\n", mdl->triggers.size());
                } else {
                    auto et = mdl->triggers.front();
                    t_divisor = et->divisor;
                    t_dev = kernel::get_instance()->get_trigger(et->dev_name);
                }

                // trigger devices stores rate in [Hz]
                double rate = t_dev->get_rate() / t_divisor;

                // ethercat master need timer interval in [ns]
                dc_timer_override = 
                    pec->dc.timer_override = (1.f / rate) * 1E9;

                log(info, "got trigger rate %f Hz\n", rate);
            }
            
            if (dc_offset_compensation_max > 0)
                pec->dc.offset_compensation_max = dc_offset_compensation_max;

            for (int nr = 0; nr < pec->slave_cnt; ++nr) {
                log(verbose, "slave %d: propagation delay %d [ns]\n", 
                        nr, pec->slaves[nr].pdelay);

                sp_slave_t slv = _slave_info[nr];

                // apply sm and fmmu config
                for (slave::sm_map_t::iterator it = slv->_sm_map.begin();
                        it != slv->_sm_map.end(); ++it) {
                    int sm_nr = it->first;

                    if (sm_nr < pec->slaves[nr].sm_ch) {
                        log(verbose, "slave %d: applying sm%d: adr 0x%X, len %d, flags 0x%X\n",
                                nr, sm_nr, it->second->_address,
                                it->second->_length,
                                it->second->_flags);

                        pec->slaves[nr].sm[sm_nr].adr = it->second->_address;
                        pec->slaves[nr].sm[sm_nr].len = it->second->_length;
                        pec->slaves[nr].sm[sm_nr].flags = it->second->_flags;
                        pec->slaves[nr].sm_set_by_user = 1;
                    }
                }
            }

            // ====> initial devices            
            if (state == module_state_preop)
                break;
        case preop_2_op:
        case preop_2_safeop: {
            // ====> start receiving measurements
            dc_sync.first_run = true;
            
            // start cyclic operation via trigger
            pec->tx_sync = 0;
            start();

            STATE_TRANSITION(pre, module_state_safeop);
            ec_set_state(pec, EC_STATE_SAFEOP);
            STATE_TRANSITION(post, module_state_safeop);

            // process data is now available, create names process data
            for (int nr = 0; nr < pec->slave_cnt; ++nr) {
                log(verbose, "slave %d: propagation delay %d [ns]\n", 
                        nr, pec->slaves[nr].pdelay);

                sp_slave_t slv = _slave_info[nr];
            }

            // add group trigger devices
            for (const auto& kv : groups)
                k.add_device(kv.second);
                
            if (state == module_state_safeop)
                break;
        }
        case safeop_2_op:
            // ====> start sending commands
            STATE_TRANSITION(pre, module_state_op);
            ec_set_state(pec, EC_STATE_OP);
            STATE_TRANSITION(post, module_state_op);
            break;
        case op_2_op:
        case safeop_2_safeop:
        case preop_2_preop:
            // ====> do nothing
            break;

        default:
            break;
    }

    return (this->state = state);
}

//! module trigger callback
void master::tick() {
    int i = 0;
    int64_t max_timeout = 0;
    ec_timer_t dc_timeout;

    if (!pec || (pec->tx_sync == 1))
        return;
        
    for (i = 0; i < pec->pd_group_cnt; ++i) {
        auto& g = groups[i];
        if ((++g->_divisor_cnt % g->_divisor) != 0)
            continue; 

        for (const auto& slave : g->_slaves)
            _slave_info[slave]->pdout_handler();

        // reset divisor cnt and queue datagram
        g->_divisor_cnt = 0;
        ec_send_process_data_group(pec, i);
        ec_timer_init(&g->timeout, g->recv_timeout);

        if (max_timeout < g->recv_timeout)
            max_timeout = g->recv_timeout;
    }

    if (pec->dc.have_dc) {
        ec_send_distributed_clocks_sync(pec);
        ec_timer_init(&dc_timeout, max_timeout);
    }

    hw_tx(pec->phw);

    pd_cookie++;
    pthread_cond_signal(&pd_cond);

    for (i = 0; i < pec->pd_group_cnt; ++i) {
        auto& g = groups[i];

        if (g->_divisor_cnt != 0)
            continue; 

        int ret = ec_receive_process_data_group(pec, i, &g->timeout);

        if (ret == 0) {
            for (auto it = g->_slaves.begin(); it != g->_slaves.end(); ++it) {
                int slave = *it;
                _slave_info[slave]->pdin_handler();
            }

            g->trigger_modules();
        }
    }

    int slave;
    for (slave = 0; slave < pec->slave_cnt; ++slave) {
        ec_slave_t *slv = &pec->slaves[slave];

        if (slv->eeprom.mbx_supported && slv->mbx_read.sm_state) {
            if (*slv->mbx_read.sm_state & 0x08) {
                pthread_cond_signal(&async_cond);
                break;
            }
        }
    }

    if (pec->dc.have_dc) {
        ec_receive_distributed_clocks_sync(pec, &dc_timeout);

        if (    (pec->dc.mode == ec_dc_info::dc_mode_ref_clock) && 
                (pec->dc.offset_compensation_cnt == 0)) {
            double diff = (pec->dc.act_diff / 1E9);

            if (!dc_sync.first_run) {
                double tmp = 0.;
                
                // trigger devices stores rate in [Hz]
                double rate = t_dev->get_rate() / t_divisor;

                // calculate new rate in [s]
                double act_timer = (1.f / rate);
                act_timer -= (-0.1 * (diff/pec->dc.offset_compensation) ) + 
                    (dc_sync.last_diff - diff)/(pec->dc.offset_compensation);

                try {
                    t_dev->set_rate(1.f / act_timer);

                    log(verbose, "setting new clock %13.10f, last_diff %13.10f, diff %13.10f, "
                            "offset_comp %d\n", tmp, dc_sync.last_diff, diff, 
                            pec->dc.offset_compensation);
                } catch (exception& e) {
                    log(warning, "setting new clock failed: %s\n", e.what());
                }
            }

            dc_sync.first_run = false;
            dc_sync.last_diff = diff;
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
        for (slave = 0; slave < pec->slave_cnt; ++slave) {
            ec_slave_t *slv = &pec->slaves[slave];

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
                    int wkc = ec_mbx_receive(pec, slave, EC_DEFAULT_TIMEOUT_MBX);
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

#if oldcode
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
            throw str_exception("[module_ethercat|%s] commanding to slow"
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
            if (g_nr < pec->pd_group_cnt) {
                to = pec->pd_groups[g_nr].pd;
                to_len = pec->pd_groups[g_nr].pdout_len;
            }
        } else {
            int sub_slave_id = ECAT_SLAVE_ID_GET_SUB(pdout->pd[i].slave_id),
                slave_id = ECAT_SLAVE_ID_GET_SLAVE(pdout->pd[i].slave_id);

            if (slave_id < pec->slave_cnt) {
                if (pdout->pd[i].slave_id & ECAT_SLAVE_ID_SUB) {
                    if (pec->slaves[slave_id].subdev_cnt > (unsigned)sub_slave_id) {
                        to = pec->slaves[slave_id].subdevs[sub_slave_id].pdout.pd;
                        to_len = pec->slaves[slave_id].subdevs[sub_slave_id].pdout.len;
                    }
                } else {
                    to = pec->slaves[slave_id].pdout.pd;
                    to_len = pec->slaves[slave_id].pdout.len;
                }
            }
        }

        if (to && to_len)
            memcpy(to, pdout->pd[i].pd, min(to_len, pdout->pd[i].len));
    }

    return 0;
}
#endif
