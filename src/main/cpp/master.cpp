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
#include <robotkernel/rt_helper.h>

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

class ec_log_printer :
    public robotkernel::log_base 
{
    public:
        ec_log_printer() :
            log_base("libethercat", "module_ethercat", "libethercat")
        {};
};

ec_log_printer elp;

void log_func(int lvl, void *user, const char *format, ...) {
//    master *e = (master *)user;
    va_list ap;
    va_start(ap, format);

    robotkernel::loglevel loglvl = verbose;
    if (lvl < 100)
        loglvl = info;
    if (lvl < 10)
        loglvl = warning;
    if (lvl <= 1)
        loglvl = error;

    char buf[1024];
    vsnprintf(buf, sizeof(buf), format, ap);
//    e->log(loglvl, format, ap);
    va_end(ap);

    elp.log(loglvl, buf);
}

/*! run */
void dc_clock_setter::run() {
    while (running()) {
        std::unique_lock<std::mutex> lk(sync_m);

        if (sync_cv.wait_for(lk, std::chrono::milliseconds(100)) == std::cv_status::no_timeout) {
            /* got signal here */
            parent->dc_set_clock();
        }
    }
}

//! construction
/*!
 * \param node yaml intialization node
 */
master::master(const std::string& name, const YAML::Node& node) :
    pd_provider(name), module_base("module_ethercat", name, node), pec(NULL) 
{
    config = YAML::Clone(node);
    elp.ll = ll;
} 

//! second stage init routine
void master::init() {
#define get_yaml(t, n, d) \
    n = get_as<t>(config, #n, d);

    ifname = get_as<string>(config, "ifname");
    get_yaml(int,      recv_prio, 0);
    get_yaml(int,      recv_mask, 0xff);
    get_yaml(bool,     log_eeprom_data, false);
    get_yaml(bool,     threaded_startup, true);
    get_yaml(bool,     monitor_state, false);

    /* creating dccs */
    dccs = make_shared<dc_clock_setter>(shared_from_this());

    ec_log_func_user = this;
    ec_log_func = log_func;

    if (config["tun_ip"]) {
        tun_settings.configure_tun = true;

        sscanf(get_as<string>(config, "tun_ip").c_str(), "%hhu.%hhu.%hhu.%hhu", 
                &tun_settings.ip_address[3], 
                &tun_settings.ip_address[2], 
                &tun_settings.ip_address[1], 
                &tun_settings.ip_address[0]);
    } else {
        tun_settings.configure_tun = false;
    }

    // group settings
    if (config["groups"]) {
        for (YAML::const_iterator it = config["groups"].begin();
                it != config["groups"].end(); ++it) {
            int g_nr = it->first.as<int>();
            groups[g_nr] = make_shared<group>(this, g_nr, it->second);
            
            // add trigger device from group
            kernel::get_instance()->add_device(groups[g_nr]); 
        }
    }

    if (config["slaves"] != NULL) {
        // parsing slave configurations
        const YAML::Node& slaves = config["slaves"];

        if (slaves.IsMap()) {
            for (const auto& kv : slaves) {
                int index = kv.first.as<int>();
                auto& slave_node = kv.second;

                std::shared_ptr<slave> slv = make_shared<slave>(index, slave_node, this);
                _slave_info[slv->index] = slv;
            }
        }

        if (slaves.IsSequence()) {
            YAML::Node new_slaves(YAML::NodeType::Map);

            for (YAML::const_iterator it = slaves.begin(); it != slaves.end(); ++it) {
                auto& slave_node = *it;
                int index = get_as<int>(slave_node, "index");

                new_slaves[index] = YAML::Clone(slave_node);
                new_slaves[index].remove("index");

                std::shared_ptr<slave> slv = make_shared<slave>(index, slave_node, this);
                _slave_info[slv->index] = slv;
            }

            config["slaves"] = new_slaves;
        }
    }

    // read in distributed clocks settings
    dc_sync.log                        = get_as<bool>(config, "dc_sync_log", false);
    dc_sync.mode_string                = get_as<string>(config, "dc_sync_mode", "ref_clock");
    dc_sync.first_run                  = true;
    dc_sync.last_diff                  = 0.;
    dc_sync.diffsum                    = 0.;
    dc_sync.kp                         = get_as<double>(config, "dc_sync_kp", 0.5);
    dc_sync.ki                         = get_as<double>(config, "dc_sync_ki", 1.0);
    dc_sync.timer_override             = get_as<int>(config, "dc_sync_timer_override", -1);
    dc_sync.diff_converge_cycles       = 10;
    dc_sync.diff_converge_cnt          = 0;
    dc_sync.diff_converged             = false;
    dc_sync.v_part_old                 = 0.;

    if (config["dc_sync.kp"] || config["dc_sync.ki"] || config["dc_sync.kd"])
        log(warning, 
                "\n"
                "This is a newer version of module_ethercat which uses a better pi-control\n"
                "for dc clock synchronization. To modify the gains use parameters \"dc_sync_kp\"\n"
                "and \"dc_sync_ki\" in your config file. (Using kp=%7.3f, ki=%7.3f)\n", dc_sync.ki, dc_sync.kp);

    pd_cookie = 0;
}

void master::open() {
    // -----------------------------------------------------------
    // open ethercat interface
    int ret = ec_open(&pec, ifname.c_str(), recv_prio, recv_mask, log_eeprom_data);
    if (ret != 0) 
        throw str_exception("ec_open failed: %s!\n", strerror(ret));
        
    ec_set_state(pec, EC_STATE_INIT);

    pec->threaded_startup = threaded_startup;

    // -----------------------------------------------------------
    // setting init commands, distributed clocks and eoe
    for (slave_map_t::iterator it = _slave_info.begin(); 
            it != _slave_info.end(); ++it) {
        int slave_nr = it->first;
        sp_slave_t slv = it->second;
                
        if (pec->slave_cnt <= slave_nr)  {
            log(warning, "slave %d not connected to ethercat bus, "
                        "settings dc config failed!\n", slave_nr);

            continue;
        }
            
        if (slv->dc.has_dc)
            ec_slave_set_dc_config(pec, slave_nr, 1, slv->dc.type, slv->dc.cycle_time_0,
                    slv->dc.cycle_time_1, slv->dc.cycle_shift);
        else 
            ec_slave_set_dc_config(pec, slave_nr, 0, 0, 0, 0, 0);

        if (slv->eoe.has_eoe) {
            uint8_t *mac = slv->eoe.mac.size() > 0 ? &slv->eoe.mac[0] : NULL;
            uint8_t *ip_address = slv->eoe.ip_address.size() > 0 ? &slv->eoe.ip_address[0] : NULL;
            uint8_t *subnet = slv->eoe.subnet.size() > 0 ? &slv->eoe.subnet[0] : NULL;
            uint8_t *gateway = slv->eoe.gateway.size() > 0 ? &slv->eoe.gateway[0] : NULL;
            uint8_t *dns = slv->eoe.dns.size() > 0 ? &slv->eoe.dns[0] : NULL;
            char *dns_name = slv->eoe.dns_name.size() > 0 ? (char *)slv->eoe.dns_name.c_str() : NULL; 
            ec_slave_set_eoe_settings(pec, slave_nr, mac, ip_address, subnet, gateway, dns, dns_name);
        }
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
                        "not adding to group %d!\n", s_nr, g_nr);

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

        if (pec->slave_cnt <= slave_nr) {
            log(warning, "slave %d not connected to ethercat bus, "
                    "setting mapping failed!\n", slave_nr);

            continue;
        }

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

            ec_slave_add_coe_init_cmd(pec, slave_nr, 0x24, 0x1C13, 
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

            ec_slave_add_coe_init_cmd(pec, slave_nr, 0x24, 0x1C12, 
                    0, 1, (char *)mapping, 2 * (mapping_entries + 1));
        }
    }
            
    for (int nr = 0; nr < pec->slave_cnt; ++nr) {
        if (_slave_info.find(nr) == _slave_info.end()) {
            log(verbose, "slave %d creating empty one\n", nr);

            wp_slave_t slv = make_shared<slave>(nr, this);
            _slave_info[nr] = slv;
        }

        sp_slave_t slv = _slave_info[nr];

        for (int sm_nr = 0; sm_nr < pec->slaves[nr].sm_ch; ++sm_nr) {
            if (slv->_sm_map.find(sm_nr) == slv->_sm_map.end()) {
                auto sm = make_shared<slave::sync_manager_settings_t>();
                sm->_address = pec->slaves[nr].sm[sm_nr].adr;
                sm->_length  = pec->slaves[nr].sm[sm_nr].len;
                sm->_flags   = pec->slaves[nr].sm[sm_nr].flags;

                if (sm->is_set())
                    slv->_sm_map[sm_nr] = sm;
            }
        }

        if (slv->dc.has_dc)
            pec->slaves[nr].dc.use_dc = 1;
        else 
            pec->slaves[nr].dc.use_dc = 0;

        slv->post_state_transition(module_state_init, module_state_init);

        config["slaves"][nr] = slv->to_yaml();
    }

    if (tun_settings.configure_tun) {
        ec_configure_tun(pec, tun_settings.ip_address);
    }

//    auto mdl = kernel::get_instance()->get_module(name);
//    YAML::Emitter emit;
//    emit << config;
//
//    log(verbose, "setting new config: %s\n", emit.c_str());
//
//    mdl->config = emit.c_str();
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
            if (
                    (state == module_state_preop) ||
                    (state == module_state_init)  ||
                    (state == module_state_boot)) {
                // trigger is already deregistered by robotkernel
                pec->tx_sync = 1;
            }

            STATE_TRANSITION(pre, module_state_safeop);
            ec_set_state(pec, EC_STATE_SAFEOP);
            STATE_TRANSITION(post, module_state_safeop);

            if (state == module_state_safeop)
                break;
        case safeop_2_preop:
        case safeop_2_init:
        case safeop_2_boot:
            // ====> stop receiving measurements
            dccs->stop();
            pec->tx_sync = 1;

            if (pdin_dc) {
                k.remove_device(pdin_dc);
                pdin_dc = nullptr;
            }

            if (pdin_dc_trigger) {
                k.remove_device(pdin_dc_trigger);
                pdin_dc_trigger = nullptr;
            }
            
            if (recv_error_trigger) {
                k.remove_device(recv_error_trigger);
                recv_error_trigger = nullptr;
            }

            // remove group trigger devices
            for (const auto& kv : groups)
                k.remove_device(kv.second);

            STATE_TRANSITION(pre, module_state_preop);
            ec_set_state(pec, EC_STATE_PREOP);
            STATE_TRANSITION(post, module_state_preop);

            if (state == module_state_preop)
                break;
        case preop_2_init:
        case preop_2_boot:
            // ====> deinit devices
            STATE_TRANSITION(pre, module_state_init);
            ec_set_state(pec, EC_STATE_INIT);
            STATE_TRANSITION(post, module_state_init);

            t_dev = nullptr;

            ec_close(pec);
            pec = nullptr;
        case init_2_init:
            // ====> re-/open ethercat device
            if (state == module_state_init)
                break;
        case init_2_boot:
            try {
                open();
            } catch (exception& e) {
                log(error, e.what());
                state = module_state_init;
                return state;
            }
            
            STATE_TRANSITION(pre, module_state_boot);
            ec_set_state(pec, EC_STATE_BOOT);
            STATE_TRANSITION(post, module_state_boot);
            break;
        case boot_2_init:
        case boot_2_preop:
        case boot_2_safeop:
        case boot_2_op:
            // ====> re-/open ethercat device
            STATE_TRANSITION(pre, module_state_init);
            ec_set_state(pec, EC_STATE_INIT);
            STATE_TRANSITION(post, module_state_init);

            ec_close(pec);
            pec = nullptr;

            if (state == module_state_init)
                break;
        case init_2_op:
        case init_2_safeop:
        case init_2_preop: {
            try {
                open();
            } catch (exception& e) {
                log(error, e.what());
                state = module_state_init;
                return state;
            }

            pec->dc.mode = dc_sync.mode_string == "ref_clock" ? 
                ec_dc_info::dc_mode_ref_clock : dc_sync.mode_string == "master_as_ref_clock" ?
                ec_dc_info::dc_mode_master_as_ref_clock : ec_dc_info::dc_mode_master_clock;
            
            STATE_TRANSITION(pre, module_state_preop);
            ec_set_state(pec, EC_STATE_PREOP);
            STATE_TRANSITION(post, module_state_preop);

            auto mdl = get_module();
            if (mdl->triggers.size() != 1) {
                log(warning, "we have %d trigger devices set by robotkernel, "
                        "please use only 1!\n", mdl->triggers.size());
            } else {
                auto et = mdl->triggers.front();
                t_divisor = et->divisor;
                t_dev = kernel::get_instance()->get_trigger(et->dev_name);
            
                double rate = t_dev->get_rate() / t_divisor;
                dc_sync.start_timer = (1.f / rate);
                dc_sync.diff_converge_cycles = 10;//rate;
            }

            if (dc_sync.timer_override > 0) {
                pec->dc.timer_override = dc_sync.timer_override;

                rate = 1. / (dc_sync.timer_override / 1E9);
            } else {
                // trigger devices stores rate in [Hz]
                rate = t_dev->get_rate() / t_divisor;

                // ethercat master need timer interval in [ns]
                dc_sync.timer_override = 
                    pec->dc.timer_override = (1.f / rate) * 1E9;

                log(info, "got trigger rate %f Hz\n", rate);
            }

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

                        pec->slaves[nr].sm[sm_nr].adr   = it->second->_address;
                        pec->slaves[nr].sm[sm_nr].len   = it->second->_length;
                        pec->slaves[nr].sm[sm_nr].flags = it->second->_flags;
                    }
                        
                    pec->slaves[nr].sm_set_by_user  = slv->sm_set_by_user;
                }
            }

            // ====> initial devices            
            if (state == module_state_preop)
                break;
        }
        case preop_2_op:
        case preop_2_safeop: {
            // ====> start receiving measurements
            dc_sync.first_run = true;
            
            if (recv_error_trigger)
                k.remove_device(recv_error_trigger);
            
            recv_error_trigger = make_shared<robotkernel::trigger>(name, "recv_error");
            k.add_device(recv_error_trigger);

            // start cyclic operation via trigger
            pec->tx_sync = 0;
            dccs->start();

            // add group trigger devices
            for (auto& kv : groups) {
                auto& grp = kv.second; 

                double grp_rate = (rate / grp->divisor);
                grp->set_rate(grp_rate);
                k.add_device(grp);
    
                for (auto& s_nr : grp->_slaves) {
                    _slave_info[s_nr]->rate = grp_rate;
                }
            }
                
            STATE_TRANSITION(pre, module_state_safeop);
            ec_set_state(pec, EC_STATE_SAFEOP);
            STATE_TRANSITION(post, module_state_safeop);

            // process data is now available, create names process data
            for (int nr = 0; nr < pec->slave_cnt; ++nr) {
                log(verbose, "slave %d: propagation delay %d [ns]\n", 
                        nr, pec->slaves[nr].pdelay);

                sp_slave_t slv = _slave_info[nr];
            }

            // distributed clock info process data
            if (pdin_dc)
                k.remove_device(pdin_dc);

            if (pdin_dc_trigger)
                k.remove_device(pdin_dc_trigger);
            
            pdin_dc_trigger = make_shared<robotkernel::trigger>(name, "dc.inputs");
            k.add_device(pdin_dc_trigger);

            string pdo_desc = 
                "- uint64_t: dc_time\n"
                "- uint64_t: dc_cycle_sum\n"
                "- uint64_t: dc_cycle\n"
                "- uint32_t: dc_cycle_cnt\n"
                "- int64_t: dc_sto\n"
                "- int64_t: rtc_sto\n"
                "- uint64_t: rtc_time\n"
                "- uint64_t: rtc_cycle_sum\n"
                "- uint64_t: rtc_cycle\n"
                "- uint32_t: rtc_count\n"
                "- int32_t: act_diff\n"
                "- int64_t: prev_rtc\n"
                "- int64_t: prev_dc\n"
                "- int32_t: timer_override\n"
                "- int64_t: timer_prev\n";

            pdin_dc = make_shared<robotkernel::triple_buffer>(
                    (uint8_t *)&pec->dc.p_de_dc - (uint8_t *)&pec->dc.dc_time, 
                    name, "dc.inputs", pdo_desc, pdin_dc_trigger->id());
            dc_provider_hash = pdin_dc->set_provider(shared_from_this());
            k.add_device(pdin_dc);
            
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
    bool dc_sent = false;
    int64_t max_timeout = 0;
    ec_timer_t dc_timeout, ec_state_timeout;

    if (!pec || (pec->tx_sync == 1))
        return;

    for (i = 0; i < pec->pd_group_cnt; ++i) {
        auto& g = groups[i];
        if ((++g->_divisor_cnt % g->divisor) != 0)
            continue; 

        //log(verbose, "sending group %d\n", i);

        for (const auto& slave : g->_slaves)
            _slave_info[slave]->pdout_handler();

        // reset divisor cnt and queue datagram
        g->_divisor_cnt = 0;
        ec_send_process_data_group(pec, i);
        ec_timer_init(&g->timeout, g->recv_timeout);

        if (max_timeout < g->recv_timeout)
            max_timeout = g->recv_timeout;
    }

    if ((pec->dc.rtc_time != 0) && pec->dc.have_dc) {
        //log(verbose, "sending distributed clock sync\n");

        dc_sent = ec_send_distributed_clocks_sync(pec) == 0;
        ec_timer_init(&dc_timeout, max_timeout);
    }

    if (monitor_state) {
        ec_send_brd_ec_state(pec); 
        ec_timer_init(&ec_state_timeout, 1000000000);
    }

    hw_tx(pec->phw);

    pd_cookie++;
    pd_cond.notify_all();

    for (i = 0; i < pec->pd_group_cnt; ++i) {
        auto& g = groups[i];

        if (g->_divisor_cnt != 0)
            continue; 

        int ret = ec_receive_process_data_group(pec, i, &g->timeout);

        if (ret == -1) {
            recv_error_trigger->trigger_modules();
            
            if (errno == ETIMEDOUT) {
                log(warning, "receiving group %d returned timeout!\n");
                continue;
            }
        }

        //log(verbose, "received group %d\n", i);

        for (auto it = g->_slaves.begin(); it != g->_slaves.end(); ++it) {
            int slave = *it;
            _slave_info[slave]->pdin_handler();
        }

        g->trigger_modules();
    }

    if (dc_sent && pec->dc.have_dc) {
        ec_receive_distributed_clocks_sync(pec, &dc_timeout);

        //log(verbose, "received distributed clock sync\n");

        if (pec->dc.mode == ec_dc_info::dc_mode_ref_clock) {
            dc_set_clock();
        }        

        if (pdin_dc) {
            pdin_dc->write(dc_provider_hash, 0, (uint8_t *)&pec->dc.dc_time, 
                    (size_t)((uint8_t *)&pec->dc.p_de_dc - (uint8_t *)&pec->dc.dc_time));
            pdin_dc_trigger->trigger_modules();
        }
    }
    
    if (monitor_state)
        ec_receive_brd_ec_state(pec, &ec_state_timeout); 
}

/*! Correct Master clock according to distributed clock. */
void master::dc_set_clock() {
    double diff_per_cycle = (pec->dc.act_diff / 1E9);

    double kp = dc_sync.kp;
    double ki = dc_sync.ki;

    if (abs(diff_per_cycle) < (dc_sync.start_timer / 100.)) {
        // lower factors
        kp /= 10.;
        ki /= 10.;
    }

    // sum it up for integral part
    dc_sync.diffsum += ki * diff_per_cycle; 

    // limit diffsum
    double diffsum_limit = dc_sync.start_timer;
    if (dc_sync.diffsum > diffsum_limit) { dc_sync.diffsum = diffsum_limit; }
    else if (dc_sync.diffsum < (-1 * diffsum_limit)) { dc_sync.diffsum = -1 * diffsum_limit; }
    
    double act_timer = 1. / t_dev->get_rate();
    
    log(verbose, "old timer %8.3f, kp %7.3f, ki %7.3f, p part %1.12f, i_part %1.12f, i_antiwindup %1.12f\n", 
            1. / act_timer, kp, ki, (kp * diff_per_cycle), dc_sync.diffsum, diffsum_limit);

    // calculate new rate in [s]
    double v_part = (kp * diff_per_cycle) + dc_sync.diffsum;
    
    log(verbose, "v_part %.10f, v_part_old %.10f, correction %.10f\n", v_part, dc_sync.v_part_old, v_part - dc_sync.v_part_old);
    act_timer += (v_part - dc_sync.v_part_old);
    dc_sync.v_part_old = v_part;

    try {
        rate = 1.f / act_timer;
        t_dev->set_rate(rate);

        if (dc_sync.log) {
            log(verbose, "setting new clock rate to %8.3f [Hz], clock diff %8.3f [us]\n",
                    rate, diff_per_cycle * 1E6);
        }
    } catch (exception& e) {
        log(warning, "setting new clock failed: %s\n", e.what());
    }

    dc_sync.first_run = false;
    dc_sync.last_diff = diff_per_cycle;

    // check if diff converged
    if (    dc_sync.diff_converge_cycles && 
            ((++dc_sync.diff_converge_cnt % dc_sync.diff_converge_cycles) == 0)) {
        double margin = dc_sync.start_timer / 100.;

        if ((diff_per_cycle > margin) || (diff_per_cycle < -1 * margin)) {
            if (!dc_sync.diff_converged) {
                log(info, "DC diff did not converge until now... (start_timer %10.7f, act_timer %10.7f, margin %10.7f, diff %10.7f\n",
                        dc_sync.start_timer, act_timer, margin, diff_per_cycle);
            }
        } else {
            if (!dc_sync.diff_converged) {
                log(info, "DC diff converged!\n");
                dc_sync.diff_converged = true;
            }
        }
    }
}

