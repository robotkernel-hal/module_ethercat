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
#include "robotkernel/exceptions.h"
#include "slave.h"

#include "libethercat/ec.h"
#include "libethercat/slave.h"
#include "libethercat/coe.h"
#include "libethercat/mbx.h"
#include "libethercat/dc.h"
#include "libethercat/soe.h"

//! module_ethercat::
namespace module_ethercat {

class slave;
extern const std::string state_strings[];

class master : public robotkernel::module_base, public robotkernel::trigger_base {
    friend class slave;

    public:
        typedef struct group {
            group(int index, const YAML::Node& node);

            //! register interfaces for slave
            /*!
             * \param ctx ethercat context
             * \return N/A
             */
            void register_interfaces(std::string name);

            //! unregister interfaces of slave
            /*!
             * \return N/A
             */
            void unregister_interfaces();

            int _index;
            int _divisor;
            int _divisor_cnt;
            std::list<int> _slaves;
            robotkernel::kernel::interface_id_t _pd_intf;
        } group_t;

        typedef std::map<int, group *> group_map_t;
        group_map_t _group_info;

        typedef std::map<int, slave *> slave_map_t;
        slave_map_t _slave_info;

        ec_t *_pec;

        int _recv_prio;
        int _recv_mask;
        std::string _ifname;
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
};

//! module_ethercat::
};

#endif // __MASTER_H__

