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


#include "service_provider/memory_inspection/base.h"
#include "service_provider/canopen_protocol/base.h"
#include "service_provider/sercos_protocol/base.h"
#include "service_provider/file_protocol/base.h"

//#define MEM_ADDRESS(x)          ((x) & 0x0000FFFF)
//#define MEM_TYPE_SLAVE_MEM      0x00000000
//#define MEM_TYPE_SLAVE_EEPROM   0x00010000
//#define MEM_TYPE_MASK           0x000F0000


extern "C" void convert_string_to_hex(std::string input, 
        char **output, size_t *outlen);

//! module_ethercat::
namespace module_ethercat {

class master;

#define TO_TRANSITION(from, to) ((((from) & 0xF) << 4) | ((to) & 0xF))

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

//! ethercat data types 
typedef enum {
   ECT_BOOLEAN         = 0x0001,
   ECT_INTEGER8        = 0x0002,
   ECT_INTEGER16       = 0x0003,
   ECT_INTEGER32       = 0x0004,
   ECT_UNSIGNED8       = 0x0005,
   ECT_UNSIGNED16      = 0x0006,
   ECT_UNSIGNED32      = 0x0007,
   ECT_REAL32          = 0x0008,
   ECT_VISIBLE_STRING  = 0x0009,
   ECT_OCTET_STRING    = 0x000A,
   ECT_UNICODE_STRING  = 0x000B,
   ECT_TIME_OF_DAY     = 0x000C,
   ECT_TIME_DIFFERENCE = 0x000D,
   ECT_DOMAIN          = 0x000F,
   ECT_INTEGER24       = 0x0010,
   ECT_REAL64          = 0x0011,
   ECT_INTEGER64       = 0x0015,
   ECT_UNSIGNED24      = 0x0016,
   ECT_UNSIGNED64      = 0x001B,
   ECT_BIT1            = 0x0030,
   ECT_BIT2            = 0x0031,
   ECT_BIT3            = 0x0032,
   ECT_BIT4            = 0x0033,
   ECT_BIT5            = 0x0034,
   ECT_BIT6            = 0x0035,
   ECT_BIT7            = 0x0036,
   ECT_BIT8            = 0x0037
} ec_data_type;

class slave : public std::enable_shared_from_this<slave> {
    public:
        enum request_type {
            request_type_memory,
            request_type_eeprom,
            request_type_mailbox
        };

        class memory_inspection : public service_provider::memory_inspection::base {
            public:
                std::shared_ptr<slave> slv;     //!< our slave pointer
                request_type type;              //!< request type
                
                memory_inspection(std::shared_ptr<slave> slv, const request_type& type);

                //! retreave all readable/writeable memory areas
                /*!
                 * \param areas list of areas
                 */
                void get_memory_areas(
                        service_provider::memory_inspection::area_list_t& areas);

                //! read memory
                /*!
                 * \param address start address
                 * \param data read data
                 */
                void read_memory(const uint64_t& address, 
                        service_provider::memory_inspection::data_t& data);

                //! write memory
                /*!
                 * \param address start address
                 * \param data data to write
                 */
                void write_memory(const uint64_t& address, 
                        service_provider::memory_inspection::data_t& data);
        };
        
        class canopen : public service_provider::canopen_protocol::base {
            public:
                std::shared_ptr<slave> slv;     //!< our slave pointer
                request_type type;              //!< request type

                canopen(std::shared_ptr<slave> slv, const request_type& type);

                //! return a list with all indices of the object dictionary
                // derived from service_provider::canopen_protocol::base
                /*!
                 * \param list returns the list with all indices
                 */
                void get_object_dictionary_list(
                        service_provider::canopen_protocol::object_dictionary_list_t& list);

                //! return a object description of given index
                // derived from service_provider::canopen_protocol::base
                /*!
                 * \param index requested index
                 * \param desc returns the object description
                 */
                void get_object_description(const uint16_t& index, 
                        service_provider::canopen_protocol::object_description_t& desc);

                //! return a element description of given index and sub index
                // derived from service_provider::canopen_protocol::base
                /*!
                 * \param index requested index
                 * \param sub_index requested sub index
                 * \param desc returns the object description
                 */
                void get_element_description(const uint16_t& index, const uint8_t& sub_index,
                        service_provider::canopen_protocol::element_description_t& desc);

                //! reads one element
                // derived from service_provider::canopen_protocol::base
                /*!
                 * \param index requested index
                 * \param sub_index requested sub index
                 * \param value returns read value 
                 */
                void read_element(const uint16_t& index, const uint8_t& sub_index,
                        service_provider::canopen_protocol::element_t& value);

                //! writes one element
                // derived from service_provider::canopen_protocol::base
                /*!
                 * \param index requested index
                 * \param sub_index requested sub index
                 * \param value value to write
                 */
                void write_element(const uint16_t& index, const uint8_t& sub_index,
                        const service_provider::canopen_protocol::element_t& value);
        };
        
        typedef enum mem_type {
            MEM_TYPE_SLAVE_MEM = 0,
            MEM_TYPE_SLAVE_EEPROM = 1,
        } mem_type_t;

        //! canopen over ethercat init cmd
        typedef struct coe_init_cmd {
            int index;                  //!< canopen dictionary identifier
            int subindex;               //!< canopen sub index
            int ca;                     //!< write in complete access mode
            char *data;                 //!< new id data
            size_t datalen;             //!< new id data length
            transition_t transition;    //!< init command transition
            std::string value;          //!< element value

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
            int idn;                    //!< servodrive id number
            int element;                //!< servodrive element number
            int atn;                    //!< servodrive drive number
            char *data;                 //!< servodrive id data
            size_t datalen;             //!< servodrive id data length
            transition_t transition;    //!< init command transition

            //! construction
            /*!
             * \param node yaml intialization node
             */
            soe_init_cmd(const YAML::Node& node);

            //! destruction
            ~soe_init_cmd();
        } soe_init_cmd_t;
        typedef std::list<soe_init_cmd_t *> soe_list_t;

        coe_list_t coe_init_cmds;   //! canopen over ethercat init commands
        soe_list_t soe_init_cmds;   //! sercos over ethercat init commands

        typedef std::list<int> mapping_t;
        mapping_t input_mapping;    //! process data input mapping values
        mapping_t output_mapping;   //! process data output mapping values

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

        std::string name;       //!< slave name
        int index;              //!< slave bus index
        master *master_dev;     //!< master device

        // service requesters
        robotkernel::sp_service_requester_t _mbx_coe;    //!< canopen service requester
        robotkernel::sp_service_requester_t _eeprom_coe; //!< canopen service requester
        robotkernel::sp_service_requester_t _eeprom_mi;  //!< eeprom memory inspection service requester
        robotkernel::sp_service_requester_t _memory_mi;  //!< memory inspection service requester

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
        void register_interfaces(module_state_t state);

//        int on_set_ec_state(ln::service_request& req, ln_service_module_ethercat_set_ec_state& svc);
//        int on_get_ec_state(ln::service_request& req, ln_service_module_ethercat_get_ec_state& svc);

    private:
        //! initialize common stuff
        void _init();
};

//! module_ethercat::
};

#endif // __ETHERCAT_MODULE_SLAVE_H__ 

