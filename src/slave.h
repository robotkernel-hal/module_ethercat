//! robotkernel module ethercat slave
/*!
 * author: Robert Burger <robert.burger@dlr.de>
 */

/*
 * This file is part of module_ethercat.
 *
 * module_ethercat is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 * 
 * module_ethercat is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with module_ethercat; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef MODULE_ETHERCAT__SLAVE_H
#define MODULE_ETHERCAT__SLAVE_H

#include <list>
#include <string>
#include <stdint.h>

#include "yaml-cpp/yaml.h"

#include "robotkernel/module_base.h"

#include "service_provider_memory_inspection/base.h"
#include "service_provider_canopen_protocol/base.h"
#include "service_provider_sercos_protocol/base.h"
#include "service_provider_file_protocol/base.h"
#include "service_provider_process_data_inspection/base.h"
#include "service_provider_key_value/base.h"
#include "service_provider_key_value/key_value_helper.h"

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

class slave : 
    public virtual robotkernel::shared_base,
    public service_provider_key_value::slave
{
    public:
        enum request_type {
            request_type_memory,
            request_type_eeprom,
            request_type_mailbox
        };

        class memory_inspection : public service_provider_memory_inspection::base {
            public:
                std::shared_ptr<slave> slv;     //!< our slave pointer
                request_type type;              //!< request type
                
                memory_inspection(std::shared_ptr<slave> slv, const request_type& type);

                //! retreave all readable/writeable memory areas
                /*!
                 * \param areas list of areas
                 */
                void get_memory_areas(
                        service_provider_memory_inspection::area_list_t& areas);

                //! read memory
                /*!
                 * \param address start address
                 * \param data read data
                 */
                void read_memory(const uint64_t& address, 
                        service_provider_memory_inspection::data_t& data);

                //! write memory
                /*!
                 * \param address start address
                 * \param data data to write
                 */
                void write_memory(const uint64_t& address, 
                        const service_provider_memory_inspection::data_t& data);
        };
        
        class canopen : public service_provider_canopen_protocol::base {
            public:
                std::shared_ptr<slave> slv;     //!< our slave pointer
                request_type type;              //!< request type

                canopen(std::shared_ptr<slave> slv, const request_type& type);

                //! return a list with all indices of the object dictionary
                // derived from service_provider_canopen_protocol::base
                /*!
                 * \param list returns the list with all indices
                 */
                void get_object_dictionary_list(
                        service_provider_canopen_protocol::object_dictionary_list_t& list);

                //! return a object description of given index
                // derived from service_provider_canopen_protocol::base
                /*!
                 * \param index requested index
                 * \param desc returns the object description
                 */
                void get_object_description(const uint16_t& index, 
                        service_provider_canopen_protocol::object_description_t& desc);

                //! return a element description of given index and sub index
                // derived from service_provider_canopen_protocol::base
                /*!
                 * \param index requested index
                 * \param sub_index requested sub index
                 * \param desc returns the object description
                 */
                void get_element_description(const uint16_t& index, const uint8_t& sub_index,
                        service_provider_canopen_protocol::element_description_t& desc);

                //! reads one element
                // derived from service_provider_canopen_protocol::base
                /*!
                 * \param index requested index
                 * \param sub_index requested sub index
                 * \param value returns read value 
                 */
                void read_element(const uint16_t& index, const uint8_t& sub_index,
                        service_provider_canopen_protocol::element_t& value);

                //! writes one element
                // derived from service_provider_canopen_protocol::base
                /*!
                 * \param index requested index
                 * \param sub_index requested sub index
                 * \param value value to write
                 */
                void write_element(const uint16_t& index, const uint8_t& sub_index,
                        const service_provider_canopen_protocol::element_t& value);
                
                //! pop next emergency message, throw exception if non present
                /*!
                 * \param msg return emergency message
                 */
                void pop_emergency_message(service_provider_canopen_protocol::emergency_message_t& msg);

                //! return process data description yaml string 
                /*!
                 * \param idx pdo index, usually 0x1C12 (RxPDO) or 0x1C13 (TxPDO)
                 */
                std::string get_pdo_description(uint16_t idx);
        };
        
        class sercos : public service_provider_sercos_protocol::base {
            public:
                std::shared_ptr<slave> slv;     //!< our slave pointer
                int atn;                        //!< sercos at number

                sercos(std::shared_ptr<slave> slv, int atn);
                
                //! read sercos id number
                /*!
                 * \param idn id number to read
                 * \param elements elements to read
                 * \param data data to read
                 */
                void sercos_read_idn(const uint16_t& idn, 
                        const service_provider_sercos_protocol::sercos_service_elements_t& elements, 
                        service_provider_sercos_protocol::service_data_t& data);

                //! write sercos id number
                /*!
                 * \param idn id number to write
                 * \param elements elements to write
                 * \param data data to write
                 */
                void sercos_write_idn(const uint16_t& idn, 
                        const service_provider_sercos_protocol::sercos_service_elements_t& elements, 
                        service_provider_sercos_protocol::service_data_t& data);
        };

        class file_protocol : public service_provider_file_protocol::base {
            public:
                std::shared_ptr<slave> slv;     //!< our slave pointer

                file_protocol(std::shared_ptr<slave> slv);

                //! read from file
                /*!
                 * \param info file info structure
                 */
                void file_read(
                        service_provider_file_protocol::file_readwrite_info_t& info);

                //! write to file
                /*!
                 * \param info file info structure
                 */
                void file_write(
                        const service_provider_file_protocol::file_readwrite_info_t& info);
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

            bool already_added;         //!< already added to ec master

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

            bool already_added;         //!< already added to ec master

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
        bool prefer_obj_names;
        bool skip_pdo_description;

        uint32_t expected_vendor;
        uint32_t expected_product;

        //! slave distributed clocks
        struct slave_dc {
            bool has_dc;

            int type;              //! dc type, 0 = sync0, 1 = sync01
            uint32_t cycle_time_0; //! cycle time of sync 0 [ns]
            uint32_t cycle_time_1; //! cycle time of sync 1 [ns]
            int32_t cycle_shift;   //! cycle shift time [ns]

            //! default construction
            slave_dc();

            //! construction
            /*!
             * \param node yaml intialization node
             */
            slave_dc(const YAML::Node& node);
            
            //! emit yaml node
            YAML::Node to_yaml();

            //! is dc set
            bool is_set() { return (cycle_time_0 != 0) || 
                (cycle_time_1 != 0) || (cycle_shift != 0); }
        } dc;

        struct slave_eoe {
            bool has_eoe;

            std::vector<uint8_t> mac;
            std::vector<uint8_t> ip_address;
            std::vector<uint8_t> subnet;
            std::vector<uint8_t> gateway;
            std::vector<uint8_t> dns;
            std::string dns_name;

            //! default construction
            slave_eoe() { has_eoe = false; };

            //! construction
            /*!
             * \param[in]   node        YAML initialization node.
             */
            slave_eoe(const YAML::Node& node);
        } eoe;

        typedef struct sync_manager_settings {
            int      _address;      //!< sync manager address
            unsigned _flags;        //!< sync manager flags
            unsigned _length;       //!< sync manager length
            
            //! default construction
            sync_manager_settings() {};

            //! construction
            /*!
             * \param node yaml intialization node
             */
            sync_manager_settings(const YAML::Node& node);

            //! emit yaml node
            YAML::Node to_yaml();

            //! is sync manager set
            bool is_set() { return (_address != 0) || 
                (_flags != 0) || (_length != 0); }
        } sync_manager_settings_t;

        typedef std::map<int, std::shared_ptr<sync_manager_settings_t> > sm_map_t;
        sm_map_t _sm_map;       //! sync manager configs
        bool   sm_set_by_user;  //!< sync manager read from config 

        std::string name;       //!< slave name
        int index;              //!< slave bus index
        master *master_dev;     //!< master device

        double rate;

        // named process data
        robotkernel::sp_process_data_t pdin;
        robotkernel::sp_pd_provider_t  pdin_provider;
        service_provider_process_data_inspection::sp_pd_inspection_t pdin_inspection;
        robotkernel::sp_process_data_t pdout;
        robotkernel::sp_pd_consumer_t  pdout_consumer;
        service_provider_process_data_inspection::sp_pd_inspection_t pdout_inspection;

        // service requesters
        robotkernel::sp_service_interface_t _mbx_foe;    //!< file service requester
        std::vector<robotkernel::sp_service_interface_t> 
            _mbx_soe_list;                               //!< servodrive service requester
        std::shared_ptr<canopen>            mbx_coe;    //!< canopen service requester
        robotkernel::sp_service_interface_t _eeprom_coe; //!< canopen service requester
        robotkernel::sp_service_interface_t _eeprom_mi;  //!< eeprom memory inspection service requester
        robotkernel::sp_service_interface_t _memory_mi;  //!< memory inspection service requester

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
        slave(int index, const YAML::Node& node, master *master_dev);

        //! destruction
        ~slave();
            
        //! emit yaml node
        YAML::Node to_yaml();

        void init_key_value();

        // perform robotkernel clean up
        void clean_up();

        //! process data out handler
        void pdout_handler();
        
        //! process data in handler
        void pdin_handler();

        //! sending slave init commands
        /*!
         */
        void add_init_cmds();

        //! prepare state transitions
        /*!
         * \param from state coming from
         * \param to state switching to
         */
        void pre_state_transition(module_state_t from, module_state_t to);

        //! register interfaces for slave
        /*!
         * \param from state coming from
         * \param to state switching to
         */
        void post_state_transition(module_state_t from, module_state_t to);
};

//! Emit YAML status of module instance.
/*! 
 * \param[out] out  Emitter output stream.
 * \param[in] sm    Module instance.
 * \return  Output emitter.
 */
YAML::Emitter& operator << (YAML::Emitter& out, slave::sync_manager_settings& sm);

//! module_ethercat::
};

#endif // __ETHERCAT_MODULE_SLAVE_H__ 

