//! robotkernel ethercat module
/*!
  $Id$
 */

#include "robotkernel/exceptions.h"

#undef BUILD_DATE
#undef BUILD_HOST
#undef BUILD_USER
#undef PACKAGE
#undef PACKAGE_NAME
#undef PACKAGE_STRING
#undef PACKAGE_TARNAME
#undef PACKAGE_VERSION
#undef VERSION

#include "module_ethercat.h"
#include "master.h"
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
using namespace module_ethercat;

//! log to kernel logging facility
void ethercat_log(robotkernel::loglevel lvl, string name, const char *format, ...) {
    char buf[1024];

    // format argument list
    va_list args;
    va_start(args, format);
    vsnprintf(buf, 1024, format, args);
    klog(lvl, "[module_ethercat|%s] %s", name.c_str(), buf);
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
    master *master_dev = (master *)hdl;
    if (!master_dev) {
        errno = EINVAL;
        return -1;
    }

    return master_dev->read(buf, bufsize);
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
    master *master_dev;

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
    
    master_dev = new master(name, doc);
    if (!master_dev) {
        ethercat_log(module_error, name, "cannot allocate memory");
        return (MODULE_HANDLE)NULL;
    }

    return (MODULE_HANDLE)master_dev;
}

//! unconfigure module
/*!
  \param hdl module handle
  \return success or failure
 */
int mod_unconfigure(MODULE_HANDLE hdl) {
    master *master_dev = (master *)hdl;
    if (!master_dev) {
        errno = EINVAL;
        return -1;
    }

    delete master_dev;
    return 0;
}

//! set module state machine to defined state
/*!
  \param hdl module handle
  \param state requested state
  \return success or failure
 */
int mod_set_state(MODULE_HANDLE hdl, module_state_t state) {
    master *master_dev = (master *)hdl;
    if (!master_dev) {
        errno = EINVAL;
        return -1;
    }

    return master_dev->set_state(state);
}

//! get module state machine state
/*!
  \param hdl module handle
  \return current state
 */
module_state_t mod_get_state(MODULE_HANDLE hdl) {
    master *master_dev = (master *)hdl;
    if (!master_dev) {
        errno = EINVAL;
        return module_state_unknown;
    }

    return master_dev->get_state();
}

//! send a request to module
/*!
  \param hdl module handle
  \param reqcode request code
  \param ptr pointer to request structure
  \return success or failure
 */
int mod_request(MODULE_HANDLE hdl, int reqcode, void* ptr) {
    master *master_dev = (master *)hdl;
    if (!master_dev) {
        errno = EINVAL;
        return -1;
    }

    return master_dev->request(reqcode, ptr);
}

//! module trigger callback
/*!
 * \param hdl module handle
 */
void mod_trigger(MODULE_HANDLE hdl) {
    master *master_dev = (master *)hdl;
    if (!master_dev) {
        errno = EINVAL;
        return;
    }

    master_dev->trigger();
}

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

