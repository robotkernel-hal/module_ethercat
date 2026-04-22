//! robotkernel module ethercat master
/*!
 * author: Robert Burger <robert.burger@dlr.de>
 */

/*
 * This file is part of module_ethercat.
 *
 * module_ethercat is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 * 
 * module_ethercat is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with module_ethercat; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <math.h>

#include "master.h"
#include <robotkernel/helpers.h>
#include <robotkernel/robotkernel.h>

MODULE_DEF(module_ethercat, module_ethercat::master)

using namespace std;
using namespace robotkernel;
using namespace robotkernel::helpers;
using namespace module_ethercat;

/* 
 * User-Level equivalent to eth_hw_addr_random()
 * uses "locally administered" bit (0x02) and clears Multicast (0x01)
 */
static void user_eth_hw_addr_random(uint8_t mac[6]) {
    if (!mac) return;
    
    // generate random bytes
    for (int i = 0; i < 6; i++) {
        mac[i] = (uint8_t)rand();
    }
    
    mac[0] = (mac[0] & 0xFE) | 0x02;
}

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

void log_func(ec_t *pec, int lvl, const char *format, ...) {
    master *e = (master *)pec->ec_log_func_user;
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
    e->log(loglvl, buf);
//    e->log(loglvl, format, ap);
    va_end(ap);

    //elp.log(loglvl, buf);
}

//! construction
/*!
 * \param node yaml intialization node
 */
master::master(const std::string& name, const YAML::Node& node) :
    service_provider_canopen_protocol::base(name, "master.mailbox"),
    module_base("module_ethercat", name, node),
    act_diff_avg(get_as<unsigned int>(node, "dc_act_diff_average_count", 100))
{
    config = YAML::Clone(node);
    elp.ll = ll;
    ec_opened = false;
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
    get_yaml(bool,     use_real_names, false);
    
    if (config["trigger"]) {
        trg = make_shared<triggerable>(config["trigger"], std::bind(&master::tick, this));

        if (config["send_trigger"]) log(warning, "send_trigger ignored, global trigger specified!");
        if (config["receive_trigger"]) log(warning, "receive_trigger ignored, global trigger specified!");
    } else {
        trg = make_shared<triggerable>(config["send_trigger"], std::bind(&master::send_trigger, this));
        recv_trg = make_shared<triggerable>(config["recv_trigger"], std::bind(&master::recv_trigger, this));
    }

    ec.ec_log_func_user = this;
    ec.ec_log_func = log_func;

    if (config["tun_device_name"]) {
        tun_settings.configure_tun = true;
        tun_settings.tun_device_name = get_as<std::string>(config, "tun_device_name");

        sscanf(get_as<string>(config, "tun_master_ip").c_str(), "%hhu.%hhu.%hhu.%hhu", 
                &tun_settings.tun_master_ip[0], 
                &tun_settings.tun_master_ip[1], 
                &tun_settings.tun_master_ip[2], 
                &tun_settings.tun_master_ip[3]);

        log(info, "using tun device \"%s\" with master ip %d.%d.%d.%d\n", 
                tun_settings.tun_device_name.c_str(),
                tun_settings.tun_master_ip[0], 
                tun_settings.tun_master_ip[1], 
                tun_settings.tun_master_ip[2], 
                tun_settings.tun_master_ip[3]);
                
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
            robotkernel::add_device(groups[g_nr]); 
        }
    }

    if (config["slaves"]) {
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

    max_timer_deviation_in_percent = get_as<unsigned int>(config, "max_timer_deviation_in_percent", 5);
    if (max_timer_deviation_in_percent > 100) { max_timer_deviation_in_percent = 100; }

    // read in distributed clocks settings
    dc_sync_log                        = get_as<bool>(config, "dc_sync_log", false);
    dc_sync_mode_string                = get_as<string>(config, "dc_sync_mode", "ref_clock");
    dc_sync.adjust_master_clock        = get_as<bool>(config, "dc_adjust_master_clock", true);
    dc_sync.first_run                  = true;
    dc_sync.last_diff                  = 0.;
    dc_sync.p_part                     = 0.;
    dc_sync.i_part                     = 0.;
    dc_sync.kp                         = get_as<double>(config, "dc_sync_kp", 0.5);
    dc_sync.ki                         = get_as<double>(config, "dc_sync_ki", 0.0025);
    dc_sync.i_limit                    = get_as<double>(config, "dc_sync_i_limit", 10.); // in [ns]
    dc_sync.timer_override             = get_as<int64_t>(config, "dc_sync_timer_override", -1);
    dc_sync.diff_converge_cycles       = get_as<uint64_t>(config, "dc_sync_converge_cycles", 10);
    dc_sync.diff_converge_cnt          = 0;
    dc_sync.diff_converged             = false;
    dc_sync.act_diff_threshold_dcsoffset_correction = get_as<uint64_t>(config, "dc_act_diff_threshold_dcsoffset_correction", 100000);

    if (config["dc_sync.kp"] || config["dc_sync.ki"] || config["dc_sync.kd"])
        log(warning, 
                "\n"
                "This is a newer version of module_ethercat which uses a better pi-control\n"
                "for dc clock synchronization. To modify the gains use parameters \"dc_sync_kp\"\n"
                "and \"dc_sync_ki\" in your config file. (Using kp=%7.3f, ki=%7.3f)\n", dc_sync.ki, dc_sync.kp);

    if (config["dc_sync_offset_compensation_cycles"]) {
        log(warning, 
                "\n"
                "'dc_sync_offset_compensation_cycles' is deprecated and can be removed from\n"
                "your config file\n");
    }
    
    if (config["dc_sync_kd"]) {
        log(warning, 
                "\n"
                "'dc_sync_kd' is deprecated and can be removed from\n"
                "your config file\n");
    }

    pd_cookie = 0;
}

//! Cyclic distributed clock datagram callback
static void cb_dc(void *arg, int num) {
    (void)num;

    master *m = (master *)arg;
    m->recv_dc();
}

void master::recv_dc() {
    if (ec.dc.mode == dc_mode_ref_clock) {
        if (dc_sync.adjust_master_clock) {
            uint64_t rate_in_ns = dc_sync.start_timer;
            rate_in_ns += ec.dc.timer_correction;

            // apply max deviation (all in [s])
            if (rate_in_ns < min_timer_rate) {
                rate_in_ns = min_timer_rate;
            } else if (rate_in_ns > max_timer_rate) {
                rate_in_ns = max_timer_rate;
            }
            
            rate = 1. / ((double)rate_in_ns / 1E9);
            trg->dev->set_rate(rate); // set in [Hz]
            if (recv_trg) recv_trg->dev->set_rate(rate); 
        } else {
            int64_t act_diff_middle = act_diff_avg.add(ec.dc.act_diff);

            if (    act_diff_avg.full() && 
                    (   (act_diff_middle > (int64_t)dc_sync.act_diff_threshold_dcsoffset_correction) || 
                        (act_diff_middle < (-1 * (int64_t)dc_sync.act_diff_threshold_dcsoffset_correction)))) {
                ec_async_check_dcsoffset(&ec.async_loop, act_diff_middle);
                act_diff_avg.reset();
            }
        }
                    
        // statistics (not really needed here)
        dc_sync.last_diff = ec.dc.act_diff;
        dc_sync.i_part = ec.dc.control.i_part;
        dc_sync.p_part = ec.dc.control.p_part;

        if (!dc_sync.diff_converged) {
            double margin = dc_sync.start_timer / 100.;
            double fast_margin = dc_sync.start_timer / 1000.;

            if (ec.dc.act_diff < fast_margin) {
                dc_sync.diff_converge_cnt += 10;
            } else if (ec.dc.act_diff < margin) {
                dc_sync.diff_converge_cnt ++;
            }

            if (dc_sync.diff_converge_cnt > dc_sync.diff_converge_cycles) {     
                dc_sync.diff_converged = true;
            }
        }
    }        
    
    if (pd_dc_sync) {
        pd_dc_sync->trigger();
    }

    if (pdin_dc) {
        pdin_dc->write(pdin_dc_provider, 0, (uint8_t *)&ec.dc.dc_time, 
                (size_t)((uint8_t *)&ec.dc.timer_correction - (uint8_t *)&ec.dc.dc_time));
    }
}

static void cb_group(void *arg, int group) {
    master *m = (master *)arg;

    m->recv_group(group);
}

void master::recv_group(int group_index) {
    auto& g = groups[group_index];

    for (auto it = g->_slaves.begin(); it != g->_slaves.end(); ++it) {
        int slave = *it;
        _slave_info[slave]->pdin_handler();
    }

    g->do_trigger();
}

extern "C" size_t hw_stream_read(void *user, void *buf, size_t len) {
    sp_stream_t *rk_stream = (sp_stream_t *)user;
    return (*rk_stream)->read(buf, len);
}

extern "C" size_t hw_stream_write(void *user, void *buf, size_t len) {
    sp_stream_t *rk_stream = (sp_stream_t *)user;
    return (*rk_stream)->write(buf, len);
}

void master::open() {
    bool abort = false;
    struct hw_common *phw = NULL;
    int ret = -1;
            
    if ((ifname.compare(0, 7, "stream:") == 0)) {
        string tmp = ifname.substr(7);
        rk_stream = robotkernel::get_device<stream>(tmp);
        ret = hw_device_stream_open(&hw_stream, &ec, &rk_stream, hw_stream_read, hw_stream_write, recv_prio - 1, recv_mask);

        if (ret == 0) {
            phw = &hw_stream.common;
        }
    }

#if LIBETHERCAT_BUILD_DEVICE_FILE == 1
    if ((ifname[0] == '/') || (ifname.compare(0, 5, "file:") == 0)) {
        string tmp = ifname; 

        // assume char device -> hw_file
        if (ifname.compare(0, 5, "file:") == 0) {
            tmp = ifname.substr(5);
        }

        log(info, "Opening interface as device file: %s\n", tmp.c_str());
        ret = hw_device_file_open(&hw_file, &ec, tmp.c_str(), recv_prio - 1, recv_mask);

        if (ret == 0) {
            phw = &hw_file.common;
        }
    }
#endif
#if LIBETHERCAT_BUILD_DEVICE_BPF == 1
    if (ifname.compare(0, 4, "bpf:") == 0) {
        string tmp = ifname.substr(4); 

        log(info, "Opening interface as BPF: %s\n", tmp.c_str());
        ret = hw_device_bpf_open(&hw_bpf, ifname.c_str());

        if (ret == 0) {
            tphw = &hw_bpf.common;
        }
    }
#endif
#if LIBETHERCAT_BUILD_DEVICE_PIKEOS == 1
    if (ifname.compare(0, 7, "pikeos:") == 0) {
        string tmp = ifname.substr(7);

        log(info, "HW_OPEN", "Opening interface as pikeos: %s\n", tmp.c_str());
        ret = hw_device_pikeos_open(&hw_pikeos, tmp.c_str(), recv_prio - 1, recv_mask);

        if (ret == 0) {
            phw = &hw_pikeos.common;
        }
    }
#endif
#if LIBETHERCAT_BUILD_DEVICE_SOCK_RAW_LEGACY == 1
    if (ifname.compare(0, 9, "sock-raw:") == 0) {
        string tmp = ifname.substr(9);
        
        log(info, "Opening interface as SOCK_RAW: %s\n", tmp.c_str());
        ret = hw_device_sock_raw_open(&hw_sock_raw, &ec, tmp.c_str(), recv_prio - 1, recv_mask);

        if (ret == 0) {
            phw = &hw_sock_raw.common;
        }
    }
#endif
#if LIBETHERCAT_BUILD_DEVICE_SOCK_RAW_MMAPED == 1
    if (ifname.compare(0, 16, "sock-raw-mmaped:") == 0) {
        string tmp = ifname.substr(16);

        log(info, "Opening interface as mmaped SOCK_RAW: %s\n", tmp.c_str());
        ret = hw_device_sock_raw_mmaped_open(&hw_sock_raw_mmaped, &ec, tmp.c_str(), recv_prio - 1, recv_mask);

        if (ret == 0) {
            phw = &hw_sock_raw_mmaped.common;
        }
    }
#endif

    if (ret != 0) {
        log(error, 
                "Unable to open or find an appropriate hardware layer for given ifname \"%s\".\n"
                "Examples for appropriate hardware layers:\n"
#ifdef LIBETHERCAT_BUILD_DEVICE_FILE
                "ifname: file:/dev/ecat0:polling:blocking:monitor     - Using file layer with additional options\n"
                "                                                       polling: Try to disable interrupts and do busy-loop-polling.\n"
                "                                                       blocking: Try to do kernel-blocking when waiting for response.\n"
                "                                                       monitor: Enable monitor device 'ecat_monitorX'. Dangerous for RT!\n"
#endif
#ifdef LIBETHERCAT_BUILD_DEVICE_BPF
                "ifname: bdf:eth0                                     - Using BPF device.\n"
#endif
#ifdef LIBETHERCAT_BUILD_DEVICE_SOCK_RAW_LEGACY
                "ifname: sock-raw:enp5s0                              - Using RAW socket network device. Needs root or CAP_NET_RAW!\n"
#endif
#ifdef LIBETHERCAT_BUILD_DEVICE_SOCK_RAW_MMAPED
                "ifname: sock-raw-mmaped:enp5s0                       - Using RAW socket network device with mmap. Needs root or CAP_NET_RAW!\n"
#endif
#ifdef LIBETHERCAT_BUILD_DEVICE_PIKEOS
                "ifname: pikeos:enp5s0                                - Using Pikeos socket layer\n."
#endif
                , ifname.c_str()); 

        throw runtime_error(string("opening hardware layer failed!\n"));
    }

    // -----------------------------------------------------------
    // open ethercat interface
    ret = ec_open(&ec, phw, log_eeprom_data);
    if (ret != 0) 
        throw runtime_error(string_printf("ec_open failed: %s!\n", strerror(ret)));

    ec_opened = true;
        
    if (ec_set_state(&ec, EC_STATE_INIT) != EC_STATE_INIT) {
        throw runtime_error(string("fatal: state switch to init failed!\n"));
    }

    ec.threaded_startup = threaded_startup;

    // -----------------------------------------------------------
    // setting init commands, distributed clocks and eoe
    for (slave_map_t::iterator it = _slave_info.begin(); 
            it != _slave_info.end(); ++it) {
        int slave_nr = it->first;
        sp_slave_t slv = it->second;
                
        if (ec.slave_cnt <= slave_nr)  {
            log(error, "slave %2d: expected but not connected to ethercat bus!\n", slave_nr);
            abort = true; 
            continue;
        }

        if (slv->expected_vendor != 0) {
            if (ec.slaves[slave_nr].eeprom.vendor_id != slv->expected_vendor) {
                log(error, "slave %2d: got vendor_id 0x%X but expected 0x%X!\n", slave_nr, 
                    ec.slaves[slave_nr].eeprom.vendor_id, slv->expected_vendor);

                abort = true;
            }
        }
        
        if (slv->expected_product != 0) {
            if (ec.slaves[slave_nr].eeprom.product_code != slv->expected_product) {
                log(error, "slave %2d: got product_code 0x%X but expected 0x%X!\n", slave_nr, 
                    ec.slaves[slave_nr].eeprom.product_code, slv->expected_product);

                abort = true;
            }
        }
            
        if (slv->dc.has_dc)
            ec_slave_set_dc_config(&ec, slave_nr, 1, slv->dc.type, slv->dc.cycle_time_0,
                    slv->dc.cycle_time_1, slv->dc.cycle_shift);
        else 
            ec_slave_set_dc_config(&ec, slave_nr, 0, 0, 0, 0, 0);

        if (slv->eoe.has_eoe) {
            uint8_t *mac = slv->eoe.mac.size() > 0 ? &slv->eoe.mac[0] : NULL;
            uint8_t *ip_address = slv->eoe.ip_address.size() > 0 ? &slv->eoe.ip_address[0] : NULL;
            uint8_t *subnet = slv->eoe.subnet.size() > 0 ? &slv->eoe.subnet[0] : NULL;
            uint8_t *gateway = slv->eoe.gateway.size() > 0 ? &slv->eoe.gateway[0] : NULL;
            uint8_t *dns = slv->eoe.dns.size() > 0 ? &slv->eoe.dns[0] : NULL;
            char *dns_name = slv->eoe.dns_name.size() > 0 ? (char *)slv->eoe.dns_name.c_str() : NULL; 
            ec_slave_set_eoe_settings(&ec, slave_nr, mac, ip_address, subnet, gateway, dns, dns_name);
        }
    }

    if (abort) {
        throw runtime_error(string("fatal: config mismatch!\n"));
    }

    // -----------------------------------------------------------
    // creating and assigning process data groups
    ec_create_pd_groups(&ec, groups.size());
            
    for (group_map_t::iterator it = groups.begin(); it != groups.end(); ++it) {
        int g_nr = it->first;
        auto g = it->second;

        ec.pd_groups[g_nr].divisor = g->divisor;
        ec.pd_groups[g_nr].cdg.user_cb = cb_group;
        ec.pd_groups[g_nr].cdg.user_cb_arg = (void *)this;
        ec.pd_groups[g_nr].overlapping = g->overlapping ? 1 : 0;
        ec.pd_groups[g_nr].use_lrw = g->lrw ? 1 : 0;

        g->_slaves.remove_if([&](int s_nr) { 
                bool rem = (ec.slave_cnt <= s_nr);
                if (rem) {
                    log(warning, "slave %d not connected to ethercat bus removing from group %d!\n", s_nr, g_nr);
                }

                return rem;
            });

        for (std::list<int>::iterator it2 = it->second->_slaves.begin();
                it2 != it->second->_slaves.end(); ++it2) {

            int s_nr = *it2;
            if (ec.slave_cnt <= s_nr) {
                log(warning, "slave %d not connected to ethercat bus, "
                        "not adding to group %d!\n", s_nr, g_nr);

                continue;
            }

            ec.slaves[s_nr].assigned_pd_group = g_nr;
        }
    }
    
    // -----------------------------------------------------------
    // add callback for cyclic dc datagram
    ec.dc.cdg.user_cb = cb_dc;
    ec.dc.cdg.user_cb_arg = (void *)this;
    ec.dc.control.kp = dc_sync.kp;
    ec.dc.control.ki = dc_sync.ki;
    ec.dc.control.i_part_limit = dc_sync.i_limit;
    
    // -----------------------------------------------------------
    // set pdo mapping entries
    for (slave_map_t::iterator it = _slave_info.begin(); 
            it != _slave_info.end(); ++it) {
        int slave_nr = it->first;
        sp_slave_t slv = it->second;

        log(verbose, "trying to create mapping for slave %d\n", slave_nr);

        if (ec.slave_cnt <= slave_nr) {
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

            ec_init_cmd_t& cmd = *init_cmds.insert(init_cmds.end(), ec_init_cmd_t());
            ec_slave_mailbox_coe_init_cmd_init(&cmd, 0x24, 0x1C13, 0, 1, (char *)mapping, 2 * (mapping_entries + 1));
            ec_slave_add_init_cmd(&ec, slave_nr, &cmd);
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

            ec_init_cmd_t& cmd = *init_cmds.insert(init_cmds.end(), ec_init_cmd_t());
            ec_slave_mailbox_coe_init_cmd_init(&cmd, 0x24, 0x1C12, 0, 1, (char *)mapping, 2 * (mapping_entries + 1));
            ec_slave_add_init_cmd(&ec, slave_nr, &cmd);
        }
    }
            
    for (int nr = 0; nr < ec.slave_cnt; ++nr) {
        if (_slave_info.find(nr) == _slave_info.end()) {
            log(verbose, "slave %d creating empty one\n", nr);

            wp_slave_t slv = make_shared<slave>(nr, this);
            _slave_info[nr] = slv;
        }

        sp_slave_t slv = _slave_info[nr];

        for (int sm_nr = 0; sm_nr < ec.slaves[nr].sm_ch; ++sm_nr) {
            if (slv->_sm_map.find(sm_nr) == slv->_sm_map.end()) {
                auto sm = make_shared<slave::sync_manager_settings_t>();
                sm->_address = ec.slaves[nr].sm[sm_nr].adr;
                sm->_length  = ec.slaves[nr].sm[sm_nr].len;
                sm->_flags   = ec.slaves[nr].sm[sm_nr].flags;

                if (sm->is_set())
                    slv->_sm_map[sm_nr] = sm;
            }
        }

        if (slv->dc.has_dc)
            ec.slaves[nr].dc.use_dc = 1;
        else 
            ec.slaves[nr].dc.use_dc = 0;

        slv->post_state_transition(module_state_init, module_state_init);

        config["slaves"][nr] = slv->to_yaml();
    }

    if (tun_settings.configure_tun) {
        osal_uint8_t master_mac[6] = { 0 };
        user_eth_hw_addr_random(master_mac);
        ec_veth_open_tun(
                &ec, tun_settings.tun_device_name.c_str(), 
                master_mac, 
                *(uint32_t *)&tun_settings.tun_master_ip[0]);
    }
}

//! destruction 
master::~master() {
    if (state != module_state_init) {
        set_state(module_state_init);
    }

    for (auto& kv : _slave_info) {
        kv.second->clean_up();
        kv.second = nullptr;
    }

    for (auto& kv : groups)
        kv.second = nullptr;
}

void master::state_transition(const module_state_t& to_state) {
    // pre stuff
    for (int nr = 0; nr < ec.slave_cnt; ++nr) {
        if (_slave_info.find(nr) == _slave_info.end()) continue;
        
        sp_slave_t slv = _slave_info[nr];
        
        try {
            slv->pre_state_transition(this->state, to_state);
        } catch (exception& e) {
            log(warning, e.what());
            set_error();
        }
    }

    auto ec_state = [to_state]() {
        if (to_state == module_state_boot) return EC_STATE_BOOT;
        if (to_state == module_state_init) return EC_STATE_INIT;
        if (to_state == module_state_preop) return EC_STATE_PREOP;
        if (to_state == module_state_safeop) return EC_STATE_SAFEOP;
        if (to_state == module_state_op) return EC_STATE_OP;
        return EC_STATE_INIT;
    };

    if (!is_error()) {
        ec_set_state(&ec, ec_state());
    }

    if (!is_error()) {
        // post stuff
        for (int nr = 0; nr < ec.slave_cnt; ++nr) {
            if (_slave_info.find(nr) == _slave_info.end()) continue;

            sp_slave_t slv = _slave_info[nr];

            try {
                slv->post_state_transition(this->state, to_state);
            } catch (exception& e) {
                log(warning, e.what());
                set_error();
            }
        }
    }
}

//! set module state machine to defined state
/*!
 * \param state requested state
 * \return success or failure
 */
int master::set_state(module_state_t state) {
    log(info, "setting state from %s to %s\n", 
            state_to_string(this->state), state_to_string(state));

    // get transition
    uint32_t transition = GEN_STATE(this->state, state);

    switch (transition) {
        case op_2_safeop:
        case op_2_preop:
        case op_2_init:
        case op_2_boot:
            // ====> stop sending commands
            state_transition(module_state_safeop);

            if (state == module_state_safeop)
                break;
        case safeop_2_preop:
        case safeop_2_init:
        case safeop_2_boot:
            // ====> stop receiving measurements
            robotkernel::remove_device(pd_dc_sync);
            pd_master_dc_sync_inputs::remove_definition();
            pd_dc_sync = nullptr;

            if (pdin_dc) {
                auto tmp = pdin_dc;
                pdin_dc = nullptr;

                tmp->reset_provider(pdin_dc_provider);
                pdin_dc_provider = nullptr;

                robotkernel::remove_device(tmp);
                pd_master_dc_inputs::remove_definition();
            }
            
            if (recv_error_trigger) {
                robotkernel::remove_device(recv_error_trigger);
                recv_error_trigger = nullptr;
            }

            state_transition(module_state_preop);

            if (state == module_state_preop)
                break;
        case preop_2_init:
        case preop_2_boot:
            // ====> deinit devices
            if (trg) trg->release();
            else { recv_trg->release(); recv_trg->release(); }

            state_transition(module_state_init);

            robotkernel::remove_device(shared_from_this_as<service_provider_canopen_protocol::base>());

            ec_close(&ec);
            ec_opened = false;
        case init_2_init:
            // ====> re-/open ethercat device
            if (!is_error() || (state == module_state_init))
                break;
        case init_2_boot:
            try {
                open();
            } catch (exception& e) {
                log(error, e.what());
                state = module_state_init;
                return state;
            }
            
            state_transition(module_state_boot);

            break;
        case boot_2_init:
        case boot_2_preop:
        case boot_2_safeop:
        case boot_2_op:
            // ====> re-/open ethercat device
            state_transition(module_state_init);

            ec_close(&ec);
            ec_opened = false;
            
            if (!is_error() || (state == module_state_init))
                break;
        case init_2_op:
        case init_2_safeop:
        case init_2_preop: {
            // ====> initialize devices
            
            try {
                open();
            } catch (exception& e) {
                log(error, e.what());
                set_error();
            }
            
            if (!is_error()) {
                robotkernel::add_device(shared_from_this_as<service_provider_canopen_protocol::base>());

                ec.dc.mode = dc_sync_mode_string == "ref_clock" ? 
                    dc_mode_ref_clock : dc_sync_mode_string == "master_as_ref_clock" ?
                    dc_mode_master_as_ref_clock : dc_mode_master_clock;
            }

            state_transition(module_state_preop);

            if (!is_error()) {
                trg->acquire();
                if (recv_trg) recv_trg->acquire();
            
                // trigger devices stores rate in [Hz]
                double rate = trg->dev->get_rate() / trg->divisor;

                if (dc_sync.timer_override > 0) {
                    ec.main_cycle_interval = dc_sync.timer_override;

                    rate = 1. / (dc_sync.timer_override / 1E9);
                } else {
                    // ethercat master need timer interval in [ns]
                    dc_sync.timer_override = 
                        ec.main_cycle_interval = (1.f / rate) * 1E9;

                    log(info, "got trigger rate %f Hz\n", rate);
                }

                dc_sync.start_timer = ec.main_cycle_interval;
                min_timer_rate = (1. - (max_timer_deviation_in_percent / 100.)) * ec.main_cycle_interval;
                max_timer_rate = (1. + (max_timer_deviation_in_percent / 100.)) * ec.main_cycle_interval;

                log(info, "start timer %13.9f, \n", dc_sync.start_timer / 1E9);

                for (int nr = 0; nr < ec.slave_cnt; ++nr) {
                    log(verbose, "slave %d: propagation delay %d [ns]\n", 
                            nr, ec.slaves[nr].pdelay);

                    sp_slave_t slv = _slave_info[nr];

                    // apply sm and fmmu config
                    for (slave::sm_map_t::iterator it = slv->_sm_map.begin();
                            it != slv->_sm_map.end(); ++it) {
                        int sm_nr = it->first;

                        if (sm_nr < ec.slaves[nr].sm_ch) {
                            log(verbose, "slave %d: applying sm%d: adr 0x%X, len %d, flags 0x%X\n",
                                    nr, sm_nr, it->second->_address,
                                    it->second->_length,
                                    it->second->_flags);

                            ec.slaves[nr].sm[sm_nr].adr   = it->second->_address;
                            ec.slaves[nr].sm[sm_nr].len   = it->second->_length;
                            ec.slaves[nr].sm[sm_nr].flags = it->second->_flags;
                        }

                        ec.slaves[nr].sm_set_by_user  = slv->sm_set_by_user;
                    }
                }
            }

            if (is_error() || (state == module_state_preop))
                break;
        }
        case preop_2_op:
        case preop_2_safeop: {
            // ====> start receiving measurements
            dc_sync.first_run = true;
            
            if (recv_error_trigger)
                robotkernel::remove_device(recv_error_trigger);
            
            recv_error_trigger = make_shared<robotkernel::trigger>(name, "recv_error");
            robotkernel::add_device(recv_error_trigger);

            // add group trigger devices
            for (auto& kv : groups) {
                auto& grp = kv.second; 

                double grp_rate = (rate * grp->divisor);
                grp->set_rate(1. / grp_rate);
    
                for (auto& s_nr : grp->_slaves) {
                    _slave_info[s_nr]->rate = grp_rate;
                }
            }
                
            state_transition(module_state_safeop);

            if (!is_error()) {
                if ((ec.dc.have_dc != 0) && (ec.dc.mode == dc_mode_ref_clock)) {
                    while (!dc_sync.diff_converged) {
                        double act_timer = 1. / trg->dev->get_rate();
                        log(info, "waiting for DC to converge... act_timer %13.9f, last_diff %13.9f, i_part %13.9f\n", act_timer, dc_sync.last_diff, dc_sync.i_part);
                        log(info, "ec info: act_diff %zd, p_part %13.9f, i_part %13.9f, timer corr %13.9f\n", ec.dc.act_diff, ec.dc.control.p_part, ec.dc.control.i_part, ec.dc.timer_correction);
                        osal_sleep(1000000000);
                    }
                }

                // process data is now available, create names process data
                for (int nr = 0; nr < ec.slave_cnt; ++nr) {
                    log(verbose, "slave %d: propagation delay %d [ns]\n", 
                            nr, ec.slaves[nr].pdelay);

                    sp_slave_t slv = _slave_info[nr];
                }

                // distributed clock info process data
                if (pdin_dc)
                    robotkernel::remove_device(pdin_dc);

                string pdo_desc = 
                    "- uint64_t: dc_time\n"
                    "- int64_t: dc_sto\n"
                    "- uint64_t: rtc_time\n"
                    "- int64_t: rtc_sto\n"
                    "- int64_t: act_diff\n"
                    "- uint64_t: packet_duration\n";

                pd_master_dc_inputs::register_definition();
                pdin_dc = make_shared<robotkernel::triple_buffer>(
                        pd_master_dc_inputs::size, name, "dc.inputs", pd_master_dc_inputs::definition_name);
                pdin_dc_provider = make_shared<robotkernel::pd_provider>(name);
                pdin_dc->set_provider(pdin_dc_provider);
                robotkernel::add_device(pdin_dc);

                pd_master_dc_sync_inputs::register_definition();
                pd_dc_sync = make_shared<robotkernel::pointer_buffer>(pd_master_dc_sync_inputs::size, (uint8_t *)&dc_sync, 
                           name, "dc_sync_ctrl.inputs", pd_master_dc_sync_inputs::definition_name);
                robotkernel::add_device(pd_dc_sync);
            }

            if (is_error() || (state == module_state_safeop))
                break;
        }
        case safeop_2_op:
            // ====> start sending commands
            state_transition(module_state_op);
            break;
        case op_2_op:
        case safeop_2_safeop:
        case preop_2_preop:
            // ====> do nothing
            break;

        default:
            break;
    }

    if (!is_error()) {
        this->state = state;
    }

    return this->state;
}

//! module trigger callback
void master::tick() {
    int i = 0;

    if (!ec_opened) { return; }

    for (i = 0; i < ec.pd_group_cnt; ++i) {
        auto& g = groups[i];
        if (ec_group_will_be_sent(&ec, i) != 0) {
            for (const auto& slave : g->_slaves) {
                _slave_info[slave]->pdout_handler();
            }
        }
    }

    ec_send_distributed_clocks_sync(&ec);
    ec_send_process_data(&ec);

    if (monitor_state) {
        ec_send_brd_ec_state(&ec); 
    }

    if (hw_tx(ec.phw) != 0) {
        hw_rx(ec.phw);
    }

    pd_cookie++;
    pd_cond.notify_all();
}

void master::send_trigger() {
    int i;

    if (!ec_opened) { return; }

    for (i = 0; i < ec.pd_group_cnt; ++i) {
        auto& g = groups[i];
        if (ec_group_will_be_sent(&ec, i) != 0) {
            for (const auto& slave : g->_slaves) {
                _slave_info[slave]->pdout_handler();
            }
        }
    }

    ec_send_distributed_clocks_sync(&ec);
    ec_send_process_data(&ec);

    if (monitor_state) {
        ec_send_brd_ec_state(&ec); 
    }

    if (hw_tx(ec.phw) != 0) {
        hw_rx(ec.phw);
    }
}

void master::recv_trigger() {
    hw_rx(ec.phw);

    pd_cookie++;
    pd_cond.notify_all();
}

