//! robotkernel module ethercat
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

#ifndef __MODULE_ETHERCAT_H__
#define __MODULE_ETHERCAT_H__

#include <sys/queue.h>
#include "robotkernel/module_intf.h"
#include "robotkernel/kernel.h"
#include "robotkernel/trigger_base.h"

void ethercat_log(robotkernel::loglevel lvl, std::string name, const char *format, ...);

class ethercat : public robotkernel::trigger_base {
    private: 
        robotkernel::kernel::interface_id_t _pd_interface_id;

    public:
        std::string _devname;
        std::string _name;          //!< module name
        module_state_t   _state;    //!< actual module state

    public:
        //! construction
        /*!
         * \param node yaml intialization node
         */
        ethercat(const std::string& name, const YAML::Node& node);

        //! destruction 
        ~ethercat();

        //! cyclic process data read
        /*!
         * \param buf process data buffer
         * \param bufsize size of process data buffer
         * \return size of read bytes
         */
        size_t read(void* buf, size_t bufsize);
        
        //! module trigger callback
        void trigger();

        //! set module state machine to defined state
        /*!
         * \param state requested state
         * \return success or failure
         */
        int set_state(module_state_t state);

        //! get module state machine state
        /*!
         * \return current state
         */
        module_state_t get_state();

        //! send a request to module
        /*!
         * \param reqcode request code
         * \param ptr pointer to request structure
         * \return success or failure
         */
        int request(int reqcode, void* ptr);
};

#endif // __MODULE_ETHERCAT_H__

