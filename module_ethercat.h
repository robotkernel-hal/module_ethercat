//! robotkernel module jr3
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

#ifndef __MODULE_JR3_H__
#define __MODULE_JR3_H__

#include <sys/queue.h>
#include "robotkernel/module_intf.h"
#include "robotkernel/kernel.h"
#include "robotkernel/trigger_base.h"

/* host adapter error codes */
#define JR3_NO_ERROR                  0x0000
#define JR3_UNKNOWN_ERROR             0x0001
#define JR3_ERROR_TIMEOUT             0x0002
#define JR3_ERROR_DSP_BUSY            0x0003
#define JR3_ERROR_MEM_FAULT           0x0004
#define JR3_ERROR_NOT_PRESENT         0x0005
#define JR3_ERROR_CMD_FAILED          0x0006
#define JR3_ERROR_INVALID_CMD         0x0007

/* sensor sprecific error codes */
#define JR3_ERROR_OVERLOAD            0x0008  
#define JR3_ERROR_OLD_DATA            0x0009
#define JR3_ERROR_BAD_CRC             0x000A
#define JR3_ERROR_COMM_TIMEOUT        0x000B
#define JR3_ERROR_INTERNAL_CLOCK      0x000C

#define JR3_ERROR_FTSOFFSET_FAILED    0x1000

#define JR3_OVERLOAD_BITS             0x002f
#define JR3_ERROR_BITS                0xf422 
#define JR3_FATAL_ERROR               JR3_ERROR_BITS & ~JR3_OVERLOAD_BITS

/* Command constants */
#define JR3_CMD_MEM_READ              0x0100
#define JR3_CMD_MEM_WRITE             0x0200
#define JR3_CMD_BIT_SET               0x0300
#define JR3_CMD_BIT_RESET             0x0400
#define JR3_CMD_USE_TRANSFORM         0x0500
#define JR3_CMD_USE_OFFSET            0x0600
#define JR3_CMD_SET_OFFSETS           0x0700
#define JR3_CMD_RESET_OFFSETS         0x0800
#define JR3_CMD_SET_VECTOR_AXES       0x0900
#define JR3_CMD_SET_FULL_SCALES       0x0A00
#define JR3_CMD_READ_RESET_PEAKS      0x0B00
#define JR3_CMD_READ_PEAKS            0x0C00

/* Offsets */
#define JR3_OFFSET_PCI                0x6000
#define JR3_OFFSET_RAWC               ( JR3_OFFSET_PCI | 0x0000 )
#define JR3_OFFSET_COPYRIGHT          ( JR3_OFFSET_PCI | 0x0040 )
#define JR3_OFFSET_SHUNTS             ( JR3_OFFSET_PCI | 0x0060 )
#define JR3_OFFSET_DEFAULTFULLSCALE   ( JR3_OFFSET_PCI | 0x0068 )
#define JR3_OFFSET_LOADEN             ( JR3_OFFSET_PCI | 0x006f )
#define JR3_OFFSET_MINFULLSCALE       ( JR3_OFFSET_PCI | 0x0070 )
#define JR3_OFFSET_ACTIVETRANSFORM    ( JR3_OFFSET_PCI | 0x0077 )
#define JR3_OFFSET_MAXFULLSCALE       ( JR3_OFFSET_PCI | 0x0078 )
#define JR3_OFFSET_PEAKA              ( JR3_OFFSET_PCI | 0x007f )
#define JR3_OFFSET_FULLSCALE          ( JR3_OFFSET_PCI | 0x0080 )
#define JR3_OFFSET_OFFSETS            ( JR3_OFFSET_PCI | 0x0088 )
#define JR3_OFFSET_ACTIVEOFFSET       ( JR3_OFFSET_PCI | 0x008e )
#define JR3_OFFSET_VECTA              ( JR3_OFFSET_PCI | 0x008f )
#define JR3_OFFSET_FTVALUES           ( JR3_OFFSET_PCI | 0x0090 )
#define JR3_OFFSET_RATEDATA           ( JR3_OFFSET_PCI | 0x00c8 )
#define JR3_OFFSET_MINIMUM            ( JR3_OFFSET_PCI | 0x00d0 )
#define JR3_OFFSET_MAXIMUM            ( JR3_OFFSET_PCI | 0x00d8 )
#define JR3_OFFSET_NEARSATVALUE       ( JR3_OFFSET_PCI | 0x00e0 )
#define JR3_OFFSET_SATVALUE           ( JR3_OFFSET_PCI | 0x00e1 )
#define JR3_OFFSET_RATEADDRESS        ( JR3_OFFSET_PCI | 0x00e2 )
#define JR3_OFFSET_RATEDIVISOR        ( JR3_OFFSET_PCI | 0x00e3 )
#define JR3_OFFSET_RATECOUNT          ( JR3_OFFSET_PCI | 0x00e4 )
#define JR3_OFFSET_COMMANDW2          ( JR3_OFFSET_PCI | 0x00e5 )
#define JR3_OFFSET_COMMANDW1          ( JR3_OFFSET_PCI | 0x00e6 )
#define JR3_OFFSET_COMMANDW0          ( JR3_OFFSET_PCI | 0x00e7 )
#define JR3_OFFSET_COUNT              ( JR3_OFFSET_PCI | 0x00e8 )
#define JR3_OFFSET_ERRORCOUNT         ( JR3_OFFSET_PCI | 0x00ee )
#define JR3_OFFSET_IDLECOUNT          ( JR3_OFFSET_PCI | 0x00ef )
#define JR3_OFFSET_WARNINGBITS        ( JR3_OFFSET_PCI | 0x00f0 )
#define JR3_OFFSET_ERRORBITS          ( JR3_OFFSET_PCI | 0x00f1 )
#define JR3_OFFSET_THRESHOLDBITS      ( JR3_OFFSET_PCI | 0x00f2 )
#define JR3_OFFSET_LASTC              ( JR3_OFFSET_PCI | 0x00f3 )
#define JR3_OFFSET_EEPROMVERSION      ( JR3_OFFSET_PCI | 0x00f4 )
#define JR3_OFFSET_SOFTWAREVN         ( JR3_OFFSET_PCI | 0x00f5 )
#define JR3_OFFSET_SOFTWARED          ( JR3_OFFSET_PCI | 0x00f6 )
#define JR3_OFFSET_SOFTWAREY          ( JR3_OFFSET_PCI | 0x00f7 )
#define JR3_OFFSET_SERIALN            ( JR3_OFFSET_PCI | 0x00f8 )
#define JR3_OFFSET_MODELN             ( JR3_OFFSET_PCI | 0x00f9 )
#define JR3_OFFSET_CALD               ( JR3_OFFSET_PCI | 0x00fa )
#define JR3_OFFSET_CALY               ( JR3_OFFSET_PCI | 0x00fb )
#define JR3_OFFSET_UNITS              ( JR3_OFFSET_PCI | 0x00fc )
#define JR3_OFFSET_BITS               ( JR3_OFFSET_PCI | 0x00fd )
#define JR3_OFFSET_CHANNELS           ( JR3_OFFSET_PCI | 0x00fe )
#define JR3_OFFSET_THICKNESS          ( JR3_OFFSET_PCI | 0x00ff )
#define JR3_OFFSET_LOADENVELOPES      ( JR3_OFFSET_PCI | 0x0100 )
#define JR3_OFFSET_TRANSFORMS         ( JR3_OFFSET_PCI | 0x0200 )
#define JR3_OFFSET_RESETADDRESS       0x18000

void jr3_log(robotkernel::loglevel lvl, std::string name, const char *format, ...);

class jr3 : public robotkernel::trigger_base {
    private: 
        static uint16_t jr3_firmware[4900];
        robotkernel::kernel::interface_id_t _pd_interface_id;

    public:
        typedef struct fullscale {
            int16_t f[3];
            int16_t m[3];
            int16_t v[2];
        } fullscale_t;
        fullscale_t _fullscale[2];

        typedef struct __attribute__((__packed__)) pdin {
            uint16_t cnt;
            double force[3];
            double torque[3];
        } __attribute__((__packed__)) pdin_t;
        pdin_t _pdin[2];

        int _fd;
        int _filter;
        std::string _devname;
        std::string _name;          //!< module name
        module_state_t   _state;    //!< actual module state
        void *_p_mem;               //!< mmapped device memory

    public:
        //! construction
        /*!
         * \param node yaml intialization node
         */
        jr3(const std::string& name, const YAML::Node& node);

        //! destruction 
        ~jr3();

        //! initialize dsp
        /*!
         * \param sensor sensor number
         */
        void init_dsp(int sensor);
        
        //! get fullscales
        /*!
         * \param sensor sensor number
         */
        void get_fullscale(int sensor);
        
        //! get data
        /*!
         * \param sensor sensor number
         */
        void get_data(int sensor);

        //! send reset offset command to jr3 sensor
        /*!
         * \param sensor sensor number
         */
        void reset_offsets(int sensor);

        //! print copyright information
        /*!
         * \param sensor sensor number
         */
        void print_copyright(int sensor);

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

#endif // __MODULE_JR3_H__

