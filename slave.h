//! robotkernel module ethercat slave
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

#ifndef __MODULE_ETHERCAT_SLAVE_H__
#define __MODULE_ETHERCAT_SLAVE_H__

#include <list>
#include <string>
#include <stdint.h>

#include "yaml-cpp/yaml.h"
#include "robotkernel/kernel.h"
#include "interface_memory_inspection/module_intf.h"

#define LN_UNREGISTER_SERVICE_IN_BASE_DETOR  
//#include "ln_messages.h"
#undef LN_UNREGISTER_SERVICE_IN_BASE_DETOR

extern "C" void convert_string_to_hex(std::string input, char **output, size_t *outlen);

//! module_ethercat::
namespace module_ethercat {

class master;

//! init command transition
typedef enum transition {
    boot_to_boot     =  0x00,
    boot_to_init     =  0x01,
    init_to_init     =  0x11,
    init_to_preop    =  0x12,
    init_to_boot     =  0x10,
    preop_to_init    =  0x21,
    preop_to_preop   =  0x22,
    preop_to_safeop  =  0x24,
    safeop_to_init   =  0x41,
    safeop_to_preop  =  0x42,
    safeop_to_safeop =  0x44,
    safeop_to_op     =  0x48,
    op_to_init       =  0x81,
    op_to_preop      =  0x82,
    op_to_safeop     =  0x84,
    op_to_op         =  0x88,
} transition_t;


class slave {
    public:
        //! canopen over ethercat init cmd
        typedef struct coe_init_cmd {
            int index;                        //! canopen dictionary identifier
            int subindex;                     //! canopen sub index
            int ca;                           //! write in complete access mode
            char *data;                       //! new id data
            size_t datalen;                   //! new id data length
            transition_t transition; //! init command transition

            //! construction
            /*!
             * \param node yaml intialization node
             */
            coe_init_cmd(const YAML::Node& node);

            //! destruction
            ~coe_init_cmd();
        } coe_init_cmd_t;
        typedef std::list<coe_init_cmd_t *> coe_list_t;

        //! servodrive over ethercat init cmd
        typedef struct soe_init_cmd {
            int idx;     //! servodrive id number
            int element; //! servodrive element number
            int drive;   //! servodrive drive number
            int val;     //! servodrive id value
        } soe_init_cmd_t;
        typedef std::list<soe_init_cmd_t *> soe_list_t;

        coe_list_t coe_init_cmds;   //! canopen over ethercat init commands
        soe_list_t soe_init_cmds;   //! sercos over ethercat init commands

        //! slave distributed clocks
        struct slave_dc {
            bool has_dc;

            int type;              //! dc type, 0 = sync0, 1 = sync01
            uint32_t cycle_time_0; //! cycle time of sync 0 [ns]
            uint32_t cycle_time_1; //! cycle time of sync 1 [ns]
            uint32_t cycle_shift;  //! cycle shift time [ns]

            //! default construction
            slave_dc();

            //! construction
            /*!
             * \param node yaml intialization node
             */
            slave_dc(const YAML::Node& node);
        } dc;

        typedef struct sync_manager_settings {
            int      _address;      //! sync manager address
            unsigned _flags;        //! sync manager flags
            unsigned _length;       //! sync manager length
            
            //! construction
            /*!
             * \param node yaml intialization node
             */
            sync_manager_settings(const YAML::Node& node);
        } sync_manager_settings_t;

        typedef std::map<int, sync_manager_settings_t *> sm_map_t;
        sm_map_t _sm_map;       //! sync manager configs

        std::string name;       //! slave name
        int index;              //! slave bus index

        //! construction
        /*!
         * \param index slave index
         * \param master_dev master device
         */
        slave(int index, master *master_dev);

        //! construction
        /*!
         * \param node yaml intialization node
         * \param master_dev master device
         */
        slave(const YAML::Node& node, master *master_dev);

        //! destruction
        ~slave();

        //! prepare state transitions
        /*!
         * \param dev ethercat master device
         * \param transition state transition
         * \return success
         */
        bool prepare_state_transition(transition_t transition);

        //! register interfaces for slave
        /*!
         * \param ctx ethercat context
         * \return N/A
         */
        void register_interfaces();

        //! unregister interfaces of slave
        /*!
         * \return N/A
         */
        void unregister_interfaces();

        //! perform memory request
        /*!
         * \param code request code
         * \param memreq memory request structure
         *               address in range 0x00000000 - 0x0000FFFF slave memory
         *                       above    0x00010000              eeprom memory
         */
        void memory_request(int code, memory_t *memreq);

    private:
        robotkernel::kernel::interface_id_t _soe_intf;
        robotkernel::kernel::interface_id_t _coe_intf;
        robotkernel::kernel::interface_id_t _pd_intf;
        robotkernel::kernel::interface_id_t _mem_intf;

        module_ethercat::master *master_dev;
};

//! module_ethercat::
};

#endif // __ETHERCAT_MODULE_SLAVE_H__ 

