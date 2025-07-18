//! robotkernel module ethercat master hw from robotkernel stream
/*
 * \author Robert Burger <robert.burger@dlr.de>
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

#ifndef MODULE_ETHERCAT__HW_STREAM_H
#define MODULE_ETHERCAT__HW_STREAM_H

#include "libethercat/config.h"
#include "libethercat/common.h"
#include "libethercat/hw.h"

typedef size_t (*stream_read_t)(void *user, void *buf, size_t nbyte);
typedef size_t (*stream_write_t)(void *user, void *buf, size_t nbyte);

typedef struct hw_stream {
    struct hw_common common;
    
    int fd;                                 //!< \brief stream descriptor

    osal_uint8_t send_frame[ETH_FRAME_LEN]; //!< \brief Static send frame.
    osal_uint8_t recv_frame[ETH_FRAME_LEN]; //!< \brief Static receive frame.
    osal_bool_t polling_mode;               //!< \brief Special interrupt-less polling-mode flag.
    
    // receiver thread settings in non-polling mode
    osal_task_t rxthread;                   //!< receiver thread handle
    int rxthreadrunning;                    //!< receiver thread running flag
                                            
    void *user;
    stream_write_t stream_write;
    stream_read_t  stream_read;
} hw_stream_t;

#ifdef __cplusplus
extern "C" {
#endif

//! Opens EtherCAT hw device.
/*!
 * \param[in]   phw             Pointer to hw handle. 
 * \param[in]   user            Pointer to user data for callbacks.
 * \param[in]   stream_read     Function pointer to read data.
 * \param[in]   stream_write    Function pointer to write data.
 * \param[in]   prio            Priority of receive thread.
 * \param[in]   affinity        CPU affinity of receive thread.
 *
 * \return 0 or negative error code
 */
int hw_device_stream_open(struct hw_stream *phw, struct ec *pec, void *user, 
        stream_read_t stream_read, stream_write_t stream_write, int prio, int affinity);

#ifdef __cplusplus
}
#endif

#endif // MODULE_ETHERCAT__HW_STREAM_H

