//! robotkernel ethercat module
/*!
  $Id$
 */

#include "robotkernel/exceptions.h"

#undef BUILD_DATE
#undef BUILD_HOST
#undef PACKAGE
#undef PACKAGE_NAME
#undef PACKAGE_STRING
#undef PACKAGE_TARNAME
#undef PACKAGE_VERSION
#undef VERSION

#include "module_ethercat.h"
#include "config.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <sys/mman.h> 
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>

using namespace robotkernel;
using namespace std;

//! log to kernel logging facility
void ethercat_log(robotkernel::loglevel lvl, string name, const char *format, ...) {
    char buf[1024];

    // format argument list
    va_list args;
    va_start(args, format);
    vsnprintf(buf, 1024, format, args);
    klog(lvl, "[module_ethercat|%s] %s", name.c_str(), buf);
}

void ethercat_log_func(void *user, const char *format, ...) {
    ethercat *e = (ethercat *)user;
    va_list ap;
    va_start(ap, format);
    ethercat_log(module_info, e->_name, format, ap);
    va_end(ap);
}

//! construction
/*!
 * \param node yaml intialization node
 */
ethercat::ethercat(const std::string& name, const YAML::Node& node) {
    _name       = name;
    _ifname     = node["ifname"].to<string>();
    _recv_prio  = node["recv_prio"].to<int>();
    _recv_mask  = node["recv_mask"].to<int>();

    ec_log_func_user = this;
    ec_log_func = ethercat_log_func;

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
ethercat::~ethercat() {
    if (_pec)
        ec_close(_pec);

    _pec = NULL;

//    if (_pd_interface_id)
//        robotkernel::kernel::unregister_interface_cb(_pd_interface_id);
}
        
//! handler function called if thread is running
void ethercat::run() {
    while (_running) {
        hw_tx(_pec->phw);

        struct timespec ts = { 0, 1000000 };
        nanosleep(&ts, NULL);
    }
}

//! cyclic process data read
/*!
 * \param buf process data buffer
 * \param bufsize size of process data buffer
 * \return size of read bytes
 */
size_t ethercat::read(void* buf, size_t bufsize) {
    return 0;
}

//! set module state machine to defined state
/*!
 * \param state requested state
 * \return success or failure
 */
int ethercat::set_state(module_state_t state) {
    int ret = 0, nr;

    switch (state) {
        case module_state_init: {
            for (slave_map_t::iterator it = _slave_info.begin();
                    it != _slave_info.end(); ++it)
                delete it->second;

            _slave_info.clear();

            start();

            ec_set_state(_pec, EC_STATE_INIT);

            for (nr = 0; nr < _pec->slave_cnt; ++nr) {
                slave_t *slv = new slave_t();
                slv->_pd_intf = NULL;
                slv->_coe_intf = NULL;
                _slave_info[nr] = slv;
            }
        
            break;
        }
        case module_state_preop: {
            start();

            ec_set_state(_pec, EC_STATE_PREOP);

            for (nr = 0; nr < _pec->slave_cnt; ++nr) {
                std::stringstream slave_name; 
                slave_name << "slave_" << nr;
                _slave_info[nr]->_coe_intf = robotkernel::kernel::register_interface_cb(_name.c_str(), 
                        "libinterface_canopen_protocol.so", slave_name.str().c_str(), nr);
                _slave_info[nr]->_pd_intf = robotkernel::kernel::register_interface_cb(_name.c_str(), 
                        "libinterface_process_data_inspection.so", slave_name.str().c_str(), nr);
            }
            break;
        }
        case module_state_safeop:
            stop();

            ec_create_pd_groups(_pec, 1);
            for (nr = 0; nr < _pec->slave_cnt; ++nr) {
                _pec->slaves[nr].assigned_pd_group = 0;
            }
            
            ec_set_state(_pec, EC_STATE_SAFEOP);
            break;
        case module_state_op:
            stop();
            
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
module_state_t ethercat::get_state() {
    return _state;
}

//! send a request to module
/*!
 * \param reqcode request code
 * \param ptr pointer to request structure
 * \return success or failure
 */
int ethercat::request(int reqcode, void* ptr) {
    int ret = 0;

    switch (reqcode) {
        case MOD_REQUEST_GET_PDIN: {            
            process_data_t *pd = (process_data_t *)ptr;
            pd->pd = NULL;
            pd->len = 0;

            if ((pd->slave_id >= 0) && (pd->slave_id < _pec->slave_cnt)) {
                pd->pd = _pec->slaves[pd->slave_id].pdin;
                pd->len = _pec->slaves[pd->slave_id].pdin_len;
            }
            
            ethercat_log(module_info, _name, "GET_PDIN: %p/%d\n", pd->pd, pd->len);
            break;
        }
        case MOD_REQUEST_GET_PDOUT: {            
            process_data_t *pd = (process_data_t *)ptr;
            pd->pd = NULL;
            pd->len = 0;

            if ((pd->slave_id >= 0) && (pd->slave_id < _pec->slave_cnt)) {
                pd->pd = _pec->slaves[pd->slave_id].pdout;
                pd->len = _pec->slaves[pd->slave_id].pdout_len;
            }

            ethercat_log(module_info, _name, "GET_PDOUT: %p/%d\n", pd->pd, pd->len);
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

            break;
        }
        case MOD_REQUEST_CANOPEN_READ_OBJECT_DESC: {
            int ret2;
            canopen_object_description *desc = (canopen_object_description *)ptr;

            // get description
            ec_coe_sdo_desc_t obj_desc;
            ret2 = ec_coe_sdo_desc_read(_pec, desc->slave_id, desc->index, &obj_desc);
            
            desc->data_type      = obj_desc.data_type;
            desc->object_code    = obj_desc.obj_type;
            desc->max_subindices = obj_desc.max_subindices;
            strcpy(desc->name, obj_desc.name);          
            break;
        }
        case MOD_REQUEST_CANOPEN_READ_ELEMENT_DESC: {
            int ret2;
            
            canopen_element_description *desc = (canopen_element_description *)ptr;

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

            size_t name_len = min(entry_desc.data_len, 40);//CANOPEN_MAXNAME - 1);
            memcpy(desc->name, &entry_desc.data[0], name_len);
            desc->name[name_len] = '\0';

            free(entry_desc.data);
            break;
        }
        default:
            ret = -1;
            break;
    }

    return ret;
}

//! local callack for syncronous read/write
static void cb_block(void *user_arg, struct datagram_entry *p) {
    idx_entry_t *entry = (idx_entry_t *)user_arg;
    sem_post(&entry->waiter);
}

//! module trigger callback
void ethercat::trigger() {
    int i = 0;

    if (_state == module_state_op) {
        for (i = 0; i < _pec->pd_group_cnt; ++i) {
            ec_pd_group_t *pd = &_pec->pd_groups[i];
            if (ec_index_get(_pec, &pd->p_idx) != 0) 
                continue;

            if (datagram_pool_get(_pec->pool, &pd->p_de, NULL) != 0) {
                ec_index_put(_pec, pd->p_idx);
                continue;
            }

            memset(&pd->p_de->datagram, 0, sizeof(ec_datagram_t) + pd->log_len + 2);
            pd->p_de->datagram.cmd = EC_CMD_LRW;
            pd->p_de->datagram.idx = pd->p_idx->idx;
            pd->p_de->datagram.adr = pd->log;
            pd->p_de->datagram.len = pd->log_len;
            pd->p_de->datagram.irq = 0;
            memcpy(ec_datagram_payload(&pd->p_de->datagram), pd->pd, pd->pdout_len);

            pd->p_de->user_cb = cb_block;
            pd->p_de->user_arg = pd->p_idx;

            // queue frame and trigger tx
            datagram_pool_put(_pec->phw->tx_low, pd->p_de);
        }
    }

    hw_tx(_pec->phw);

    if (_state == module_state_op) {
        for (i = 0; i < _pec->pd_group_cnt; ++i) {
            ec_pd_group_t *pd = &_pec->pd_groups[i];

            // wait for completion
            sem_wait(&pd->p_idx->waiter);

            uint16_t wkc = ec_datagram_wkc(&pd->p_de->datagram);
            if (wkc)
                memcpy(pd->pd+pd->pdout_len, ec_datagram_payload(&pd->p_de->datagram)+pd->pdout_len, pd->pdin_len);

            datagram_pool_put(_pec->pool, pd->p_de);
            ec_index_put(_pec, pd->p_idx);
        }
    }

    trigger_modules();
}

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

//! cyclic process data read
/*!
  \param hdl module handle
  \param buf process data buffer 
  \param bufsize size of process data buffer
  \return size of read bytes
 */
size_t mod_read(MODULE_HANDLE hdl, void* buf, size_t bufsize) {
    ethercat *ethercat_dev = (ethercat *)hdl;
    if (!ethercat_dev) {
        errno = EINVAL;
        return -1;
    }

    return ethercat_dev->read(buf, bufsize);
}

//! cyclic process data write
/*!
  \param hdl module handle
  \param buf process data buffer
  \param bufsize size of process data buffer 
  \return size of written bytes
 */
size_t mod_write(MODULE_HANDLE hdl, void* buf, size_t bufsize) {
    return 0;
}

//! configures module
/*!
  \param name module name
  \param config configure string
  \return handle on success, NULL otherwise
*/
MODULE_HANDLE mod_configure(const char* name, const char* config) {
    ethercat *ethercat_dev;

    // open config
    std::stringstream stream(config);
    YAML::Parser parser(stream);
    YAML::Node doc;

    ethercat_log(module_info, name, "build by: %s@%s\n", BUILD_USER, BUILD_HOST);
    ethercat_log(module_info, name, "build date: %s\n", BUILD_DATE);

    if (!parser.GetNextDocument(doc)) {
        ethercat_log(module_error, name, "parsing config file\n");
        return (MODULE_HANDLE)NULL;
    }
    
    ethercat_dev = new ethercat(name, doc);
    if (!ethercat_dev) {
        ethercat_log(module_error, name, "cannot allocate memory");
        return (MODULE_HANDLE)NULL;
    }

    return (MODULE_HANDLE)ethercat_dev;
}

//! unconfigure module
/*!
  \param hdl module handle
  \return success or failure
 */
int mod_unconfigure(MODULE_HANDLE hdl) {
    ethercat *ethercat_dev = (ethercat *)hdl;
    if (!ethercat_dev) {
        errno = EINVAL;
        return -1;
    }

    delete ethercat_dev;
    return 0;
}

//! set module state machine to defined state
/*!
  \param hdl module handle
  \param state requested state
  \return success or failure
 */
int mod_set_state(MODULE_HANDLE hdl, module_state_t state) {
    ethercat *ethercat_dev = (ethercat *)hdl;
    if (!ethercat_dev) {
        errno = EINVAL;
        return -1;
    }

    return ethercat_dev->set_state(state);
}

//! get module state machine state
/*!
  \param hdl module handle
  \return current state
 */
module_state_t mod_get_state(MODULE_HANDLE hdl) {
    ethercat *ethercat_dev = (ethercat *)hdl;
    if (!ethercat_dev) {
        errno = EINVAL;
        return module_state_unknown;
    }

    return ethercat_dev->get_state();
}

//! send a request to module
/*!
  \param hdl module handle
  \param reqcode request code
  \param ptr pointer to request structure
  \return success or failure
 */
int mod_request(MODULE_HANDLE hdl, int reqcode, void* ptr) {
    ethercat *ethercat_dev = (ethercat *)hdl;
    if (!ethercat_dev) {
        errno = EINVAL;
        return -1;
    }

    return ethercat_dev->request(reqcode, ptr);
}

//! module trigger callback
/*!
 * \param hdl module handle
 */
void mod_trigger(MODULE_HANDLE hdl) {
    ethercat *ethercat_dev = (ethercat *)hdl;
    if (!ethercat_dev) {
        errno = EINVAL;
        return;
    }

    ethercat_dev->trigger();
}

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

