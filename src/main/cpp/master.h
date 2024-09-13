//! robotkernel module ethercat master
/*!
 * author: Robert Burger <robert.burger@dlr.de>
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

#ifndef __MASTER_H__
#define __MASTER_H__

#include <list>
#include <string>
#include <stdint.h>

#include "robotkernel/kernel.h"
#include "robotkernel/trigger_base.h"
#include "robotkernel/module_intf.h"
#include "robotkernel/module_base.h"
#include "robotkernel/exceptions.h"

#include "yaml-cpp/yaml.h"

#include "group.h"
#include "slave.h"
#include "hw_stream.h"

#include "libethercat/config.h"
#include "libethercat/ec.h"
#include "libethercat/slave.h"
#include "libethercat/coe.h"
#include "libethercat/mbx.h"
#include "libethercat/dc.h"
#include "libethercat/soe.h"
#include "libethercat/foe.h"
#include "libethercat/error_codes.h"

#if LIBETHERCAT_BUILD_DEVICE_FILE == 1
#include <libethercat/hw_file.h>
#endif

#if LIBETHERCAT_BUILD_DEVICE_BPF == 1
#include <libethercat/hw_bpf.h>
#endif

#if LIBETHERCAT_BUILD_DEVICE_PIKEOS == 1
#include <libethercat/hw_pikeos.h>
#endif

#if LIBETHERCAT_BUILD_DEVICE_SOCK_RAW_LEGACY == 1
#include <libethercat/hw_sock_raw.h>
#endif

#if LIBETHERCAT_BUILD_DEVICE_SOCK_RAW_MMAPED == 1
#include <libethercat/hw_sock_raw_mmaped.h>
#endif

#include "service_provider/canopen_protocol/base.h"

#define COE_DATA_MAXLEN     512

//! module_ethercat::
namespace module_ethercat {
#ifdef EMACS
}
#endif

/* forward declarations */
class master;
class slave;
extern const std::string state_strings[];

class dc_clock_setter :
    public robotkernel::runnable
{
    private:
        std::shared_ptr<master> parent;

        std::mutex sync_m;
        std::condition_variable sync_cv;

    public:
        dc_clock_setter(std::shared_ptr<master> parent) : parent(parent) {};
        
        /*! signal waiter */
        void signal() {
            sync_cv.notify_one();
        }

        /* run thread */
        void run();
};

class master :
    public std::enable_shared_from_this<master>,
    public service_provider::canopen_protocol::base,
    public robotkernel::pd_provider,
    public robotkernel::module_base
{
    public:
        friend class slave;


        typedef std::map<int, std::shared_ptr<group>> group_map_t;
        group_map_t groups;

        typedef std::shared_ptr<slave> wp_slave_t;
        typedef std::shared_ptr<slave> sp_slave_t;
        typedef std::map<int, wp_slave_t> slave_map_t;
        slave_map_t _slave_info;

        struct {
            bool log;
            std::string mode_string;

            bool first_run;
            double last_diff;
            double diffsum;
            double p_part;

            double start_timer;

            double kp;
            double ki;
            double kd;
            double i_limit;
            double slew_rate;
        
            int offset_compensation_cycles;
            int offset_compensation_cnt;
            int timer_override;
            
            uint64_t diff_converge_cycles;
            uint64_t diff_converge_cnt;
            bool diff_converged;

            double v_part_old;
        } dc_sync;

        robotkernel::sp_process_data_t pd_dc_sync;
        robotkernel::sp_trigger_t      trigger_dc_sync;

        struct {
            bool configure_tun;
            uint8_t ip_address[4];
        } tun_settings;

        bool ec_opened;
        ec_t ec;

#if LIBETHERCAT_BUILD_DEVICE_FILE == 1
        struct hw_file hw_file;
#endif
#if LIBETHERCAT_BUILD_DEVICE_BPF == 1
        struct hw_bpf hw_bpf;
#endif
#if LIBETHERCAT_BUILD_DEVICE_PIKEOS == 1
        struct hw_pikeos hw_pikeos;
#endif
#if LIBETHERCAT_BUILD_DEVICE_SOCK_RAW_LEGACY == 1
        struct hw_sock_raw hw_sock_raw;
#endif
#if LIBETHERCAT_BUILD_DEVICE_SOCK_RAW_MMAPED == 1
        struct hw_sock_raw_mmaped hw_sock_raw_mmaped; 
#endif
        struct hw_stream hw_stream;
        robotkernel::sp_stream_t rk_stream;
        std::function<size_t (void *, size_t)> stream_write;
        std::function<size_t (void *, size_t)> stream_read;

        std::list<ec_init_cmd_t> init_cmds;

        int recv_prio;
        int recv_mask;
        std::string ifname;
        bool log_eeprom_data;

        bool threaded_startup;
        bool monitor_state;
        bool use_real_names;

        double rate;

        uint64_t pd_cookie;
        std::mutex pd_mtx;
        std::condition_variable pd_cond;

        std::mutex async_mtx;
        std::condition_variable async_cond;

        std::string trigger_mod_name;

        int t_divisor;                        //!< trigger divisor
        robotkernel::sp_trigger_t t_dev;      //!< trigger device

        //! named process data for distributed clocks info
        robotkernel::sp_process_data_t pdin_dc;
        robotkernel::sp_trigger_t      pdin_dc_trigger;
        std::size_t dc_provider_hash;

        robotkernel::sp_trigger_t      recv_error_trigger;

        YAML::Node config;

        std::shared_ptr<dc_clock_setter> dccs;
    public:
        //! construction
        /*!
         * \param node yaml intialization node
         */
        master(const std::string& name, const YAML::Node& node);

        //! destruction 
        ~master();

        //! second stage init routine
        void init();

        void open();

        //! module trigger callback
        void tick();

        //! set module state machine to defined state
        /*!
         * \param state requested state
         * \return success or failure
         */
        int set_state(module_state_t state);

        /*! Correct Master clock according to distributed clock. */
        void dc_set_clock();

        void recv_group(int group_index);
        void recv_dc();
        
        //! return a list with all indices of the object dictionary
        // derived from service_provider::canopen_protocol::base
        /*!
         * \param list returns the list with all indices
         */
        void get_object_dictionary_list(
                service_provider::canopen_protocol::object_dictionary_list_t& list) {
            uint8_t buf[COE_DATA_MAXLEN];
            size_t len = COE_DATA_MAXLEN;
            int ret = ec_coe_master_odlist_read(&ec, buf, &len);

            if (ret != 0) {
                throw string_util::str_exception("master: reading CoE object dictionary list "
                        "returned errorcode 0x%X!\n", ret);
            }

            list.resize(len/2);
            memcpy(&list[0], buf, len);
        }

        //! return a object description of given index
        // derived from service_provider::canopen_protocol::base
        /*!
         * \param index requested index
         * \param desc returns the object description
         */
        void get_object_description(const uint16_t& index, 
                service_provider::canopen_protocol::object_description_t& desc) {
            
            // get description
            uint32_t error_code = 0;
            ec_coe_sdo_desc_t obj_desc;
            memset(&obj_desc, 0, sizeof(obj_desc));
            int ret = ec_coe_master_sdo_desc_read(&ec, index, &obj_desc, &error_code);

            if (ret != 0) {
                // decode ret
                throw string_util::str_exception("master: reading CoE object description index 0x%X "
                        "returned errorcode 0x%X: %s!\n", index, error_code, get_sdo_info_error_string(error_code));
            }

            desc.data_type      = obj_desc.data_type;
            desc.object_code    = obj_desc.obj_code;
            desc.max_subindices = obj_desc.max_subindices;

            if (obj_desc.name) {
                desc.name = std::string(obj_desc.name, obj_desc.name_len);
            }
        }

        //! return a element description of given index and sub index
        // derived from service_provider::canopen_protocol::base
        /*!
         * \param index requested index
         * \param sub_index requested sub index
         * \param desc returns the object description
         */
        void get_element_description(const uint16_t& index, const uint8_t& sub_index,
                service_provider::canopen_protocol::element_description_t& desc) {
            // get description
            uint32_t error_code = 0;
            ec_coe_sdo_entry_desc_t entry_desc;
            memset(&entry_desc, 0, sizeof(entry_desc));
            int ret = ec_coe_master_sdo_entry_desc_read(&ec, index, 
                    sub_index, 0x7F, &entry_desc, &error_code);

            if (ret != 0) {
                // decode ret
                throw string_util::str_exception("master: reading CoE element description index 0x%X sub index %d"
                        "returned errorcode 0x%X: %s!\n", index, sub_index, error_code, get_sdo_info_error_string(error_code));
            }

            desc.value_info    = entry_desc.value_info;
            desc.data_type     = entry_desc.data_type;
            desc.bit_length    = entry_desc.bit_length;
            desc.obj_access    = entry_desc.obj_access;
            desc.unit          = 0;

            if (entry_desc.data_len > 0) {
                // decode data
                uint8_t *tmp = entry_desc.data;
                if (entry_desc.value_info & EC_COE_SDO_VALUE_INFO_UNIT) {
                    if ((tmp + 2) <= (entry_desc.data + entry_desc.data_len)) {
                        desc.unit = *(uint16_t *)tmp;
                        tmp += 2;
                    }
                }

                if (entry_desc.value_info & EC_COE_SDO_VALUE_INFO_DEFAULT_VALUE) { 
                    size_t bytesize = (entry_desc.bit_length + 7) / 8;

                    if ((tmp + bytesize) <= (entry_desc.data + entry_desc.data_len)) {
                        desc.default_value.resize(bytesize);
                        memcpy(&desc.default_value[0], tmp, bytesize);
                        tmp += bytesize;
                    }
                }

                if (entry_desc.value_info & EC_COE_SDO_VALUE_INFO_MIN_VALUE) {
                    size_t bytesize = (entry_desc.bit_length + 7) / 8;

                    if ((tmp + bytesize) <= (entry_desc.data + entry_desc.data_len)) {
                        desc.min_value.resize(bytesize);
                        memcpy(&desc.min_value[0], tmp, bytesize);
                        tmp += bytesize;
                    }
                }

                if (entry_desc.value_info & EC_COE_SDO_VALUE_INFO_MAX_VALUE) {
                    size_t bytesize = (entry_desc.bit_length + 7) / 8;

                    if ((tmp + bytesize) <= (entry_desc.data + entry_desc.data_len)) {
                        desc.max_value.resize(bytesize);
                        memcpy(&desc.max_value[0], tmp, bytesize);
                        tmp += bytesize;
                    }
                }

                if (tmp < (entry_desc.data + entry_desc.data_len)) {
                    size_t restlen = (entry_desc.data + entry_desc.data_len) - tmp;
                    desc.name = std::string((char *)tmp, restlen);

                    if ((signed)desc.name.length() != std::count_if(desc.name.begin(), desc.name.end(), 
                                [](unsigned char c){ return std::isprint(c); } ))
                        desc.name = string_util::format_string("subindex_%d", sub_index); // name is not printable
                }
            }
        }

        //! reads one element
        // derived from service_provider::canopen_protocol::base
        /*!
         * \param index requested index
         * \param sub_index requested sub index
         * \param value returns read value 
         */
        void read_element(const uint16_t& index, const uint8_t& sub_index,
                service_provider::canopen_protocol::element_t& value) {
            uint8_t buf[64]; 
            size_t buf_len = 64;
            uint32_t abort_code = 0;

            int ret = ec_coe_master_sdo_read(&ec, index, sub_index, 
                    0, &buf[0], &buf_len, &abort_code);

            if (ret != 0) {
                //if (ret == EC_ERROR_MAILBOX_ABORT) {
                //    throw service_provider::canopen_protocol::sdo_abort_exception(abort_code);
                //}

                // decode ret
                throw string_util::str_exception("master: reading CoE element index 0x%X "
                        "sub index %d returned errorcode 0x%X!\n",
                        index, sub_index, ret);
            }

            if (buf_len) {
                value.resize(buf_len);    
                memcpy(&value[0], buf, buf_len);
            }
        }

        //! writes one element
        // derived from service_provider::canopen_protocol::base
        /*!
         * \param index requested index
         * \param sub_index requested sub index
         * \param value value to write
         */
        void write_element(const uint16_t& index, const uint8_t& sub_index,
                const service_provider::canopen_protocol::element_t& value) 
        {
            uint32_t abort_code = 0;

            int ret = ec_coe_master_sdo_write(&ec, index, sub_index, 
                    0, (uint8_t *)&value[0], value.size(), &abort_code);

            if (ret != 0) {
                // decode ret
                throw string_util::str_exception("master: writing CoE element value index 0x%X "
                        "sub index %d returned errorcode 0x%X!\n", 
                        index, sub_index, ret);
            }
        }

        //! pop next emergency message, throw exception if non present
        /*!
         * \param msg return emergency message
         */
        void pop_emergency_message(service_provider::canopen_protocol::emergency_message_t& msg) {}

        //! return process data description yaml string 
        /*!
         * \param idx pdo index, usually 0x1C12 (RxPDO) or 0x1C13 (TxPDO)
         */
        std::string get_pdo_description(uint16_t idx) { return std::string(""); }
};

//! module_ethercat::
#ifdef EMACS
{
#endif
};

#endif // __MASTER_H__

