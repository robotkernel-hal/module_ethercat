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

#ifndef __MASTER_H__
#define __MASTER_H__

#include <list>
#include <string>
#include <stdint.h>

#include "yaml-cpp/yaml.h"
#include "robotkernel/kernel.h"
#include "robotkernel/trigger_base.h"
#include "robotkernel/module_intf.h"
#include "robotkernel/module_base.h"
#include "robotkernel/cmd_delay.h"
#include "robotkernel/exceptions.h"
#include "slave.h"

#include "libethercat/ec.h"
#include "libethercat/slave.h"
#include "libethercat/coe.h"
#include "libethercat/mbx.h"
#include "libethercat/dc.h"
#include "libethercat/soe.h"
#include "libethercat/foe.h"

#define ECAT_SLAVE_ID_GROUP             (0x80000000)
#define ECAT_SLAVE_ID_DC                (0x20000000)
#define ECAT_SLAVE_ID_SUB               (0x10000000)
#define ECAT_SLAVE_ID_EEPROM            (0x01000000)
            
#define ECAT_SLAVE_ID_GET_MEM_TYPE(x)   (((x) & 0x0F000000) >> 24)
#define ECAT_SLAVE_ID_GET_SLAVE(x)      ((x) & 0x0000FFFF)
#define ECAT_SLAVE_ID_GET_GROUP(x)      ((x) & 0x0000FFFF)
#define ECAT_SLAVE_ID_GET_SUB(x)        (((x) & 0x00FF0000) >> 16)


//! module_ethercat::
namespace module_ethercat {

class slave;
extern const std::string state_strings[];

class master :  public robotkernel::module_base, 
                public robotkernel::trigger_base,
                public robotkernel::cmd_delay,
                public robotkernel::runnable {
    friend class slave;

    public:
        typedef struct group {
            group(int index, const YAML::Node& node);

            //! register interfaces for slave
            /*!
             * \param ctx ethercat context
             * \return N/A
             */
            void register_interfaces(std::string name, 
                    const robotkernel::loglevel& ll);

            //! unregister interfaces of slave
            /*!
             * \return N/A
             */
            void unregister_interfaces();

            int _recv_timeout;
            int _index;
            int _divisor;
            int _divisor_cnt;
            std::list<int> _slaves;
            robotkernel::kernel::interface_id_t _pd_intf;
            
            ec_timer_t timeout;
        } group_t;

        typedef std::map<int, group *> group_map_t;
        group_map_t _group_info;

        typedef std::map<int, slave *> slave_map_t;
        slave_map_t _slave_info;

        enum {
            dc_mode_ref_clock = 0,
            dc_mode_master_clock = 1
        } _dc_mode;

        struct {
            bool first_run;
            double last_diff;
        } _dc_sync;

        ec_t *_pec;

        int _recv_prio;
        int _recv_mask;
        std::string _ifname;
        bool _log_eeprom_data;

        int dc_offset_compensation_cycles;
        int dc_offset_compensation_max;
        int dc_timer_override;

        int _trigger_interval;
            
        robotkernel::kernel::interface_id_t dc_pd_intf;

        uint64_t pd_cookie;
        pthread_mutex_t pd_lock;
        pthread_cond_t pd_cond;

        pthread_mutex_t async_lock;
        pthread_cond_t async_cond;

        std::string trigger_mod_name;
    public:
        //! construction
        /*!
         * \param node yaml intialization node
         */
        master(const std::string& name, const YAML::Node& node);

        //! destruction 
        ~master();

        //! module trigger callback
        void trigger();

        //! set module state machine to defined state
        /*!
         * \param state requested state
         * \return success or failure
         */
        int set_state(module_state_t state);

        //! send a request to module
        /*!
         * \param reqcode request code
         * \param ptr pointer to request structure
         * \return success or failure
         */
        int request(int reqcode, void* ptr);

        //! async handler thread
        void run();

        //! set new pdout pointers
        /*!
         * \param pdout new pdout pointers
         * \return 0 on success
         */
        int set_pdout(set_pd_t *pdout);
};

//! module_ethercat::
};

#endif // __MASTER_H__

