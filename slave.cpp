//! robotkernel module soemslave
/*!
  $Id$
 */

#include "slave.h"
#include "module_ethercat.h"
#include "robotkernel/kernel.h"
#include <iomanip>
#include <stdio.h>

using namespace std;
using namespace robotkernel;
using namespace module_ethercat;

pthread_mutex_t slave_lock = PTHREAD_MUTEX_INITIALIZER;

//! forward declaration ethercat state string
extern const string state_strings[];

void convert_string_to_hex(string input, char **output, size_t *outlen) {
    size_t len = input.length();
    *output = new char[len/2];
    uint32_t tmp;

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
coe_init_cmd::coe_init_cmd(const YAML::Node& node) {
    index      = node["index"].to<int>();
    subindex   = node["subindex"].to<int>();
    ca         = node["ca"].to<int>();
    transition = (transition_t)node["transition"].to<int>();
    convert_string_to_hex(node["data"].to<string>(), &data, &datalen);

    ethercat_log(module_verbose, string("coe"), "got coe init cmd: 0x%X/%d\n", 
            index, subindex);
}

//! destruction
coe_init_cmd::~coe_init_cmd() {
    if (data) {
        delete[] data;
    }
}

//! default construction
slave::slave_config::slave_config()
    : has_config(false) {}

//! construction
/*!
 * \param node yaml intialization node
 */
slave::slave_config::slave_config(const YAML::Node& node) {
    manufacturer = node["manufacturer"].to<uint32_t>();
    id           = node["id"].to<uint32_t>();
    dtype        = node["dtype"].to<uint32_t>();
    input_bits   = node["input_bits"].to<uint32_t>();
    output_bits  = node["output_bits"].to<uint32_t>();

    if (node.FindValue("sm0a"))
        sm0a     = node["sm0a"].to<uint32_t>();
    else 
        sm0a     = 0;
    
    if (node.FindValue("sm0f"))
        sm0f     = node["sm0f"].to<uint32_t>();
    else 
        sm0f     = 0;
    
    if (node.FindValue("sm0l"))
        sm0l     = node["sm0l"].to<uint32_t>();
    else 
        sm0l     = 0;

    if (node.FindValue("sm1a"))
        sm1a     = node["sm1a"].to<uint32_t>();
    else 
        sm1a     = 0;
    
    if (node.FindValue("sm1f"))
        sm1f     = node["sm1f"].to<uint32_t>();
    else 
        sm1f     = 0;

    if (node.FindValue("sm1l"))        
        sm1l     = node["sm1l"].to<uint32_t>();
    else 
        sm1l     = 0;

    sm2a         = node["sm2a"].to<uint32_t>();
    sm2f         = node["sm2f"].to<uint32_t>();
    sm3a         = node["sm3a"].to<uint32_t>();
    sm3f         = node["sm3f"].to<uint32_t>();
    
    if (node.FindValue("sm0type"))
        sm0type  = node["sm0type"].to<uint32_t>();
    else 
        sm0type  = 0;

    if (node.FindValue("sm1type"))
        sm1type  = node["sm1type"].to<uint32_t>();
    else 
        sm1type  = 0;
    
    fm0ac        = node["fm0ac"].to<uint32_t>();
    fm1ac        = node["fm1ac"].to<uint32_t>();

    if (node.FindValue("fm0func"))
        fm0func  = node["fm0func"].to<uint32_t>();
    else 
        fm0func  = 0;

    if (node.FindValue("fm1func"))
        fm1func  = node["fm1func"].to<uint32_t>();
    else 
        fm1func  = 0;

    has_config   = true;
}

//! destruction
slave::slave_config::~slave_config()
{}
            
//! default construction
slave::slave_dc::slave_dc() {
    has_dc = false;
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
 * \param index slave index
 * \param master_dev master device
 */
slave::slave(int index, master *master_dev) 
    : index(index), state_req(1), disable_ca(false), print_cnt(0), master_dev(master_dev) {
                
    _pd_intf = NULL;
    _coe_intf = NULL;

    ethercat_log(module_verbose, master_dev->_name, "default slave index %d created\n", index);
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
    state_req = 1;

    if (node.FindValue("disable_ca") != NULL)
        disable_ca = node["disable_ca"].to<bool>();
    else
        disable_ca = 0;

    if (node.FindValue("config") != NULL) {
        config = slave_config(node["config"]);
    }
    
    if (node.FindValue("dc") != NULL) {
        dc = slave_dc(node["dc"]);
    }

    if (node.FindValue("init_cmds") != NULL) {
        ethercat_log(module_verbose, master_dev->_name, 
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

    ethercat_log(module_verbose, master_dev->_name, 
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
                ethercat_log(module_info, master_dev->_name, "slave %2d configuring dc sync 01, cycle_times %d/%d, cycle_shift %d\n",
                        index, dc.cycle_time_0, dc.cycle_time_1, dc.cycle_shift);

//                ecx_dcsync01(ctx, index, true, dc.cycle_time_0, dc.cycle_time_1, dc.cycle_shift);
            } else {
                ethercat_log(module_info, master_dev->_name, "slave %2d configuring dc sync 0, cycle_time %d, cycle_shift %d\n",
                        index, dc.cycle_time_0, dc.cycle_shift);

//                ecx_dcsync0(ctx, index, true, dc.cycle_time_0, dc.cycle_shift);
            }
        } else
            ;
//            ecx_dcsync0(ctx, index, false, 0, 0);
    }

    ethercat_log(module_info, master_dev->_name, 
            "slave %2d prepare state transition from 0x%x/%s to 0x%x/%s\n",
            index, state_from, state_strings[state_from].c_str(),
            state_to, state_strings[state_to].c_str());

//    if (state_from < (ctx->slavelist[index].state & 0x0F)) {
//        ethercat_log(module_info, master_dev->_name, 
//                "slave %2d state mismatch, requested 0x%x/%s but have 0x%x/%s\n",
//                index, state_from&0xF, state_strings[state_from&0xF].c_str(),
//                ctx->slavelist[index].state&0xF, state_strings[ctx->slavelist[index].state&0xF].c_str());
//        return false;
//    }
//
//    if (ctx->slavelist[index].state & EC_STATE_ERROR) {
//        ethercat_log(module_verbose, master_dev->_name, "slave %2d is in ERROR, attempting ack.\n", index);
//        ctx->slavelist[index].state |= EC_STATE_ACK;
//        ecx_writestate(ctx, index);
//    }

    for (coe_list_t::iterator it = coe_init_cmds.begin();
            it != coe_init_cmds.end(); ++it) {

        coe_init_cmd_t *cmd = *it;

//        ctx->slavelist[1].CoEdetails &= ~ECT_COEDET_SDOCA;

        if (cmd->transition == transition) {
            ethercat_log(module_verbose, master_dev->_name, "sending coe init "
                    "command slave %d, index %X\n", index, cmd->index);

            uint8_t *buf = (uint8_t *)cmd->data;
            size_t buf_len = cmd->datalen;

            int wkc = ec_coe_sdo_write(master_dev->_pec, index, cmd->index, 
                    cmd->subindex, cmd->ca, buf, &buf_len);
            if (!wkc) {
                ethercat_log(module_info, master_dev->_name, "writing sdo, %s\n",
                     "todo");//ecx_elist2string(ctx));
            }
        } else 
            ethercat_log(module_verbose, master_dev->_name, 
                    "transition does not match %02X\n", cmd->transition);
        
    }

    return true;
}

//! state transitions
/*!
 * \param dev ethercat master device
 * \param state current state
 * \param transition state transition
 */
bool slave::state_transition(transition_t transition) {
    int state_from = (transition & 0xF0) >> 4,
        state_to = transition & 0x0F;

    ethercat_log(module_verbose, master_dev->_name, "slave %2d state transition from 0x%x/%s to 0x%x/%s\n",
         index, state_from, state_strings[state_from].c_str(),
         state_to, state_strings[state_to].c_str());

//    if (state_from < (ctx->slavelist[index].state & 0x0F)) {
//        ethercat_log(module_info, master_dev->_name, "slave %2d state mismatch, requested 0x%x/%s but have 0x%x/%s\n",
//             index, state_from&0xF, state_strings[state_from&0xF].c_str(),
//             ctx->slavelist[index].state&0xF, state_strings[ctx->slavelist[index].state&0xF].c_str());
//        return false;
//    }
//
//    pthread_mutex_lock(&slave_lock);
//
//    if (ctx->slavelist[index].state & EC_STATE_ERROR) {
//        ethercat_log(module_info, master_dev->_name, "slave %2d is in ERROR, attempting ack.\n", index);
//        ctx->slavelist[index].state |= EC_STATE_ACK;
//        ecx_writestate(ctx, index);
//    }
//
//    // switching state
//    ctx->slavelist[index].state = state_req = state_to;
//    ecx_writestate(ctx, index);
//
//    if (state_to >= EC_STATE_SAFE_OP) {
//        ecx_send_processdata_group(ctx, ctx->slavelist[index].group);
//        ecx_receive_processdata_group(ctx, ctx->slavelist[index].group, EC_TIMEOUTRET);
//    }
//
//    pthread_mutex_unlock(&slave_lock);
//
//    return ctx->slavelist[index].state == state_to;
    return false;
}

//! state check
/*!
 * \param ctx ethercat master device
 * \return success
 */
bool slave::state_check() {
//    pthread_mutex_lock(&slave_lock);
//
//    int state = ecx_statecheck(ctx, index, state_req, EC_TIMEOUTRET);
//    if (state == state_req) {
//        ethercat_log(module_verbose, master_dev->_name, "slave %2d checked state (act 0x%x/%s)\n", index, state_req, 
//                state_strings[ctx->slavelist[index].state&0xF].c_str());
//        pthread_mutex_unlock(&slave_lock);
//        return true;
//    } else if (state & EC_STATE_ERROR) {
//        ethercat_log(module_info, master_dev->_name, "slave %2d is in ERROR %X, attempting ack.\n",
//             index, ctx->slavelist[index].ALstatuscode);
//        ctx->slavelist[index].state |= EC_STATE_ACK;
//        int wkc = ecx_writestate(ctx, index);
//        ethercat_log(module_info, master_dev->_name, "slave %2d ackhnowledge error %s\n",
//             index, wkc == 1 ? "SUCCESSFULL" : "NOT SUCCESSFULL");
//    }
//
//    pthread_mutex_unlock(&slave_lock);
//
//    if (ctx->slavelist[index].state != state_req) {
//        ethercat_log(module_info, master_dev->_name, "slave %2d is not in requested state (act 0x%x/%s, req 0x%x/%s)\n",
//             index, ctx->slavelist[index].state&0xF,
//             state_strings[ctx->slavelist[index].state&0xF].c_str(), state_req&0xF,
//             state_strings[state_req&0xF].c_str());
//
//        int wkc = state_transition(ctx, (transition_t)(
//                                       ((ctx->slavelist[index].state&0xF) << 4) | (state_req&0xF))) ? 1 : 0;
//        ethercat_log(module-info, master_dev->_name, "slave %2d switching to requested state %s\n",
//             index, wkc == 1 ? "SUCCESSFULL" : "NOT SUCCESSFULL");
//    } else
//        ethercat_log(module_info, master_dev->_name, "slave %2d checked %d\n", index, state_req);
//
    return true;
}

//! register interfaces for slave
/*!
 * \param ctx ethercat context
 * \return N/A
 */
void slave::register_interfaces() {
    std::stringstream slave_name; 
    slave_name << "slave_" << index;

    if (master_dev->_pec->slaves[index].eeprom.mbx_supported & EC_EEPROM_MBX_COE)
        _coe_intf = robotkernel::kernel::register_interface_cb(master_dev->_name.c_str(), 
                "libinterface_canopen_protocol.so", slave_name.str().c_str(), index);
    if (master_dev->_pec->slaves[index].eeprom.mbx_supported & EC_EEPROM_MBX_SOE)
        _soe_intf = robotkernel::kernel::register_interface_cb(master_dev->_name.c_str(), 
                "libinterface_sercos_protocol.so", slave_name.str().c_str(), index);

    _pd_intf = robotkernel::kernel::register_interface_cb(master_dev->_name.c_str(), 
            "libinterface_process_data_inspection.so", slave_name.str().c_str(), index);
}

//! unregister interfaces of slave
/*!
 * \return N/A
 */
void slave::unregister_interfaces() {
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

