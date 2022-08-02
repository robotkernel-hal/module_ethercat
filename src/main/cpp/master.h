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

#include "libethercat/ec.h"
#include "libethercat/slave.h"
#include "libethercat/coe.h"
#include "libethercat/mbx.h"
#include "libethercat/dc.h"
#include "libethercat/soe.h"
#include "libethercat/foe.h"
#include "libethercat/error_codes.h"

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

            double start_timer;

            double kp;
            double ki;
            double kd;
        
            int offset_compensation_cycles;
            int timer_override;
            
            int diff_converge_cycles;
            int diff_converge_cnt;
            bool diff_converged;

            double v_part_old;
        } dc_sync;

        struct {
            bool configure_tun;
            uint8_t ip_address[4];
        } tun_settings;

        ec_t *pec;

        int recv_prio;
        int recv_mask;
        std::string ifname;
        bool log_eeprom_data;

        bool threaded_startup;
        bool monitor_state;

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
};

//! module_ethercat::
#ifdef EMACS
{
#endif
};

#endif // __MASTER_H__

