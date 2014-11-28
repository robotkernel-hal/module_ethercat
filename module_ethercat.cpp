//! robotkernel ethercat module
/*!
  $Id$
 */

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

#include "ethercat_ioctl.h"

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

//! construction
/*!
 * \param node yaml intialization node
 */
ethercat::ethercat(const std::string& name, const YAML::Node& node) {
    set_state(module_state_init);

    // add process data inspection 
//    std::stringstream channel_name; 
//    channel_name << "channel_" << (int)0;
//    _pd_interface_id = robotkernel::kernel::register_interface_cb(_name.c_str(), 
//            "libinterface_process_data_inspection.so", channel_name.str().c_str(), 0);
}

//! destruction 
ethercat::~ethercat() {
//    if (_pd_interface_id)
//        robotkernel::kernel::unregister_interface_cb(_pd_interface_id);
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
    int ret = 0;

    switch (state) {
        case module_state_init:
            break;
        case module_state_preop: {
            break;
        }
        case module_state_safeop:
            break;
        case module_state_op:
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
        default:
            ret = -1;
            break;
    }

    return ret;
}

//! module trigger callback
void ethercat::trigger() {
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

