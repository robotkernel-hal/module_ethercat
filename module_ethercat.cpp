//! robotkernel jr3 module
/*!
  $Id$
 */

#include "module_jr3.h"
#include "config.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <sys/mman.h> 
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "jr3_ioctl.h"

using namespace robotkernel;
using namespace std;

//! log to kernel logging facility
void jr3_log(robotkernel::loglevel lvl, string name, const char *format, ...) {
    char buf[1024];

    // format argument list
    va_list args;
    va_start(args, format);
    vsnprintf(buf, 1024, format, args);
    klog(lvl, "[module_jr3|%s] %s", name.c_str(), buf);
}

static const int JR3_DATA_OFFSET         = 0x6000;   // in Board-address
static const int JR3_RESET_ADDRESS       = 0x18000;  // in Board-address
static const int JR3_P8BIT_OFFSET        = 0x40000;  // in Pci-offset
static const int JR3_PD_DEVIDER          = 0x4000;   // in Board-address
static const int CARD_OFFSET             = 0x20000;
static const int JR3_MEM_SIZE            = 0x80000;  // 512K

template <typename type>
type jr3_dmread(void *base, off_t addr, int sensor) {
    return *(type *)(((addr + (sensor * CARD_OFFSET)) << 2) + (uint64_t)base);
}

template <typename type>
void jr3_dmwrite(void *base, off_t addr, int sensor, type data) {
    *(type *)(((addr + (sensor * CARD_OFFSET)) << 2) + (uint64_t)base) = data;
}

uint32_t jr3_pmread(void *base, off_t addr, int sensor) {
    uint16_t value, value2;
    value = jr3_dmread<uint32_t>(base, addr, sensor);
    value2 = jr3_dmread<uint32_t>(base, addr + (JR3_P8BIT_OFFSET >> 2), sensor);

    return (uint32_t)value << 16 | value2;
}

void jr3_pmwrite(void *base, off_t addr, int sensor, uint16_t value, uint16_t value2) {
    *(uint16_t *)((addr << 2) + (uint64_t)base) = value; 
    *(uint16_t *)((addr << 2) + (uint64_t)base + 0x40000) = value2;
}

//! construction
/*!
 * \param node yaml intialization node
 */
jr3::jr3(const std::string& name, const YAML::Node& node) {
    _fd = 0;
    _p_mem = NULL;

    _name = name;
    _devname = node["devname"].to<string>();
    _filter = node["filter"].to<int>();

    set_state(module_state_init);

    // add process data inspection 
    std::stringstream channel_name; 
    channel_name << "channel_" << (int)0;
    _pd_interface_id = robotkernel::kernel::register_interface_cb(_name.c_str(), 
            "libinterface_process_data_inspection.so", channel_name.str().c_str(), 0);
}

//! destruction 
jr3::~jr3() {
    if (_pd_interface_id)
        robotkernel::kernel::unregister_interface_cb(_pd_interface_id);

    if (_p_mem)
        munmap(_p_mem, JR3_MEM_SIZE);

    if (_fd > 0)
        close(_fd);
}

//! initialize dsp
/*!
 * \param sensor sensor number
 */
void jr3::init_dsp(int sensor) {
    int i = 0;
    uint16_t count, address, value, value2;

    // reset dsp
    jr3_dmwrite<uint32_t>(_p_mem, JR3_RESET_ADDRESS, sensor, 0);
        
    // first value is the count
    count = jr3_firmware[i++];
    while (count != 0xffff) {
        // second value is the address
        address = jr3_firmware[i++];

        jr3_log(module_info, _name, "dsp software download sensor %d. file pos. %d,"
                "address 0x%x, %d times\n", sensor, i, address, count);

        while (count > 0) {
            if (address & JR3_PD_DEVIDER) {
                // write to data memory
                value = jr3_firmware[i++];
                jr3_dmwrite<uint32_t>(_p_mem, address, sensor, value);
                if (value != jr3_dmread<int32_t>(_p_mem, address, sensor))
                    jr3_log(module_error, _name, "writing to dsp address %p, "
                            "sensor %d failed!\n", address, sensor);

                count--;
            } else {
                // write to program memory
                value = jr3_firmware[i++];
                value2 = jr3_firmware[i++];
                jr3_pmwrite(_p_mem, address, sensor, value, value2);
                if (((((uint32_t)value) << 16) | value2) 
                        != jr3_pmread(_p_mem, address, sensor))
                    jr3_log(module_error, _name, "writing to dsp address %p, "
                            "sensor %d failed!\n", address, sensor);

                count -= 2;
            }

            address++;
        }

        // first value is the count
        count = jr3_firmware[i++];
    }

    jr3_log(module_info, _name, "dsp software download complete, waiting for boot ...\n");

    struct timespec ts = { 0, 250000000 };
    nanosleep(&ts, NULL);

    /* warm up sensor */
    int16_t dummy;
    for (i = 1; i < 1000000; ++i) {
        dummy = jr3_dmread<int16_t>(_p_mem, JR3_OFFSET_FULLSCALE, sensor);
        if (dummy != 0)
            break;

        nanosleep(&ts, NULL);
    }

    jr3_log(module_info, _name, "boot ok, warmup-phase completed\n");
    
    // mark as programmed
    int flag = 1;
    ioctl(_fd, JR3_SET_DSP_FLAG, &flag);
}

//! print copyright information
/*!
 * \param sensor sensor number
 */
void jr3::print_copyright(int sensor) {
    char copyright[19];
    short day, year, units;

    for (int i = 0; i < 18; ++i)
        copyright[i] = (char)(jr3_dmread<uint32_t>(_p_mem, 
                    JR3_OFFSET_COPYRIGHT + i, sensor) >> 8);
    copyright[18]='\0';

    day   = jr3_dmread<uint32_t>(_p_mem, JR3_OFFSET_SOFTWARED, sensor);
    year  = jr3_dmread<uint32_t>(_p_mem, JR3_OFFSET_SOFTWAREY, sensor);
    units = jr3_dmread<uint32_t>(_p_mem, JR3_OFFSET_UNITS, sensor);
    
    string unit;
    if (units == 0)
        unit = "lbr";
    else if (units == 1)
        unit = "newtons";
    else if (units == 2)
        unit = "kilograms-force";
    else if (units == 3)
        unit = "kilolbs";
    else
        unit = "unknown";
    
    jr3_log(module_info, _name, "%s\n", copyright);
    jr3_log(module_info, _name, "dsp software updated day %d, year %d. units: %s\n",
            day, year, unit.c_str());
    
    uint16_t software_version = 
        jr3_dmread<uint16_t>(_p_mem, JR3_OFFSET_SOFTWAREVN, sensor);
    jr3_log(module_info, _name, "dsp software version %f\n", (double)software_version/100.);
}


//! send reset offset command to jr3 sensor
/*!
 * \param sensor sensor number
 */
void jr3::reset_offsets(int sensor) {
    int n = 0;
    jr3_dmwrite<uint16_t>(_p_mem, JR3_OFFSET_COMMANDW0, sensor, JR3_CMD_RESET_OFFSETS);    
    while (jr3_dmread<uint16_t>(_p_mem, JR3_OFFSET_COMMANDW0, sensor) != 0) {
        struct timespec ts = { 0, 100000000 };
        nanosleep(&ts, NULL);

        /* wait for ok (comm0 == 0) */
        if(++n > 100) {
            jr3_log(module_warning, _name, "jr3 offset failed\n");
            break;
        }
    }
}

//! get fullscales
/*!
 * \param sensor sensor number
 */
void jr3::get_fullscale(int sensor) {
    for (int i = 0; i < 8; i++) {
        int16_t val = jr3_dmread<int32_t>(_p_mem, JR3_OFFSET_FULLSCALE + i, sensor) & 0x0000FFFF;
        ((int16_t *)_fullscale)[i] = val;
    }
}

//! get data
/*!
 * \param sensor sensor number
 */
void jr3::get_data(int sensor) {
    _pdin[sensor].cnt = jr3_dmread<uint16_t>(_p_mem, 
            JR3_OFFSET_COUNT + _filter, sensor);

    double *tmp_val = (double *)_pdin[sensor].force;
    int16_t *tmp_scale = (int16_t *)&_fullscale[sensor];

    for (int i = 0; i < 6; i++) {
        off_t address = JR3_OFFSET_FTVALUES + 0x08 * _filter + i;
        double raw = jr3_dmread<int16_t>(_p_mem, address, sensor);
        tmp_val[i] = raw / 16384.0f * tmp_scale[i];
    }
}

//! cyclic process data read
/*!
 * \param buf process data buffer
 * \param bufsize size of process data buffer
 * \return size of read bytes
 */
size_t jr3::read(void* buf, size_t bufsize) {
    return 0;
}

//! set module state machine to defined state
/*!
 * \param state requested state
 * \return success or failure
 */
int jr3::set_state(module_state_t state) {
    int ret = 0;

    switch (state) {
        case module_state_init:
            if (_p_mem)
                munmap(_p_mem, JR3_MEM_SIZE);

            if (_fd > 0)
                close(_fd);

            break;
        case module_state_preop: {
            if (_fd > 0)
                close(_fd);

            _fd = open(_devname.c_str(), O_RDWR);
            if (_fd < 0) {
                jr3_log(module_error, _name, "unable to open device: %s\n", strerror(errno));
                ret = -1;
                break;
            }

            _p_mem = mmap(NULL, JR3_MEM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, _fd, 0);
            if (_p_mem == MAP_FAILED) {
                jr3_log(module_error, _name, "unable to map device memory: %s\n", strerror(errno));
                close(_fd);
                _fd = 0;
                ret = -1;
                break;
            }

            int sensor = 0;

            // check programmed
            int flag = 0;
            ioctl(_fd, JR3_GET_DSP_FLAG, &flag);
            if (flag)
                jr3_log(module_info, _name, "dsp flag set .. skip programming dsp!\n");
            else 
                init_dsp(sensor);

            // set initial values
            jr3_dmwrite<uint16_t>(_p_mem, JR3_OFFSET_ERRORBITS, sensor, 0);
            jr3_dmwrite<uint16_t>(_p_mem, JR3_OFFSET_WARNINGBITS, sensor, 0);
            jr3_dmwrite<uint16_t>(_p_mem, JR3_OFFSET_THICKNESS, sensor, 0);

            reset_offsets(sensor);
            get_fullscale(sensor);
            print_copyright(sensor);
            break;
        }
        case module_state_safeop:
            if (_state < module_state_preop) {
                jr3_log(module_error, _name, "state is smaller preop: current %d\n", _state);
                ret = -1;
            } 
            break;
        case module_state_op:
            if (_state < module_state_safeop)
                ret = -1;
            break;
        default:
            ret = -1;
    }

    if (ret == 0)
        _state = state;

    return ret;
}

//! get module state machine state
/*!
 * \return current state
 */
module_state_t jr3::get_state() {
    return _state;
}

//! send a request to module
/*!
 * \param reqcode request code
 * \param ptr pointer to request structure
 * \return success or failure
 */
int jr3::request(int reqcode, void* ptr) {
    int ret = 0;

    switch (reqcode) {
        case MOD_REQUEST_GET_PDIN: {            
            process_data_t *pd = (process_data_t *)ptr;
            pd->pd = NULL;
            pd->len = 0;

            if (pd->slave_id == 0 || pd->slave_id == 1) {
                pd->pd = &_pdin[pd->slave_id];
                pd->len = sizeof(_pdin[pd->slave_id]);;
            } else
                ret = -1;

            break;
        }
        case MOD_REQUEST_SET_TRIGGER_CB: {
            set_trigger_cb_t *cb = (set_trigger_cb_t *)ptr;
            if (cb->cb == NULL) {
                jr3_log(module_error, _name, "ERROR could not register, callback is NULL\n");
                break;
            }

            add_trigger_module(*cb);
            break;
        }
        case MOD_REQUEST_UNSET_TRIGGER_CB: {
            set_trigger_cb_t *cb = (set_trigger_cb_t *)ptr;

            if (cb->cb == NULL) {
                jr3_log(module_error, _name, "ERROR could not remove, callback is NULL\n");
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
void jr3::trigger() {
    get_data(0);
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
    jr3 *jr3_dev = (jr3 *)hdl;
    if (!jr3_dev) {
        errno = EINVAL;
        return -1;
    }

    return jr3_dev->read(buf, bufsize);
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
    jr3 *jr3_dev;

    // open config
    std::stringstream stream(config);
    YAML::Parser parser(stream);
    YAML::Node doc;

    jr3_log(module_info, name, "build by: %s@%s\n", BUILD_USER, BUILD_HOST);
    jr3_log(module_info, name, "build date: %s\n", BUILD_DATE);

    if (!parser.GetNextDocument(doc)) {
        jr3_log(module_error, name, "parsing config file\n");
        return (MODULE_HANDLE)NULL;
    }
    
    jr3_dev = new jr3(name, doc);
    if (!jr3_dev) {
        jr3_log(module_error, name, "cannot allocate memory");
        return (MODULE_HANDLE)NULL;
    }

    return (MODULE_HANDLE)jr3_dev;
}

//! unconfigure module
/*!
  \param hdl module handle
  \return success or failure
 */
int mod_unconfigure(MODULE_HANDLE hdl) {
    jr3 *jr3_dev = (jr3 *)hdl;
    if (!jr3_dev) {
        errno = EINVAL;
        return -1;
    }

    delete jr3_dev;
    return 0;
}

//! set module state machine to defined state
/*!
  \param hdl module handle
  \param state requested state
  \return success or failure
 */
int mod_set_state(MODULE_HANDLE hdl, module_state_t state) {
    jr3 *jr3_dev = (jr3 *)hdl;
    if (!jr3_dev) {
        errno = EINVAL;
        return -1;
    }

    return jr3_dev->set_state(state);
}

//! get module state machine state
/*!
  \param hdl module handle
  \return current state
 */
module_state_t mod_get_state(MODULE_HANDLE hdl) {
    jr3 *jr3_dev = (jr3 *)hdl;
    if (!jr3_dev) {
        errno = EINVAL;
        return module_state_unknown;
    }

    return jr3_dev->get_state();
}

//! send a request to module
/*!
  \param hdl module handle
  \param reqcode request code
  \param ptr pointer to request structure
  \return success or failure
 */
int mod_request(MODULE_HANDLE hdl, int reqcode, void* ptr) {
    jr3 *jr3_dev = (jr3 *)hdl;
    if (!jr3_dev) {
        errno = EINVAL;
        return -1;
    }

    return jr3_dev->request(reqcode, ptr);
}

//! module trigger callback
/*!
 * \param hdl module handle
 */
void mod_trigger(MODULE_HANDLE hdl) {
    jr3 *jr3_dev = (jr3 *)hdl;
    if (!jr3_dev) {
        errno = EINVAL;
        return;
    }

    jr3_dev->trigger();
}

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

