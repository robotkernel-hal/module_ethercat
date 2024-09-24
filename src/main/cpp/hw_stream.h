/**
 * \stream hw_stream.h
 *
 * \author Robert Burger <robert.burger@dlr.de>
 *
 * \date 24 Nov 2016
 *
 * \brief stream/char device hardware access functions
 *
 */

/*
 * This stream is part of libethercat.
 *
 * libethercat is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 * 
 * libethercat is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License along with libethercat (LICENSE.LGPL-V3); if not, write 
 * to the Free Software Foundation, Inc., 51 Franklin Street, Fifth 
 * Floor, Boston, MA  02110-1301, USA.
 * 
 * Please note that the use of the EtherCAT technology, the EtherCAT 
 * brand name and the EtherCAT logo is only permitted if the property 
 * rights of Beckhoff Automation GmbH are observed. For further 
 * information please contact Beckhoff Automation GmbH & Co. KG, 
 * Hülshorstweg 20, D-33415 Verl, Germany (www.beckhoff.com) or the 
 * EtherCAT Technology Group, Ostendstraße 196, D-90482 Nuremberg, 
 * Germany (ETG, www.ethercat.org).
 *
 */

#ifndef LIBETHERCAT_HW_STREAM_H
#define LIBETHERCAT_HW_STREAM_H

#include <libethercat/hw.h>

typedef size_t (*stream_read_t)(void *buf, size_t nbyte);
typedef size_t (*stream_write_t)(void *buf, size_t nbyte);

typedef struct hw_stream {
    struct hw_common common;
    
    int fd;                                 //!< \brief stream descriptor

    osal_uint8_t send_frame[ETH_FRAME_LEN]; //!< \brief Static send frame.
    osal_uint8_t recv_frame[ETH_FRAME_LEN]; //!< \brief Static receive frame.
    osal_bool_t polling_mode;               //!< \brief Special interrupt-less polling-mode flag.
    
    // receiver thread settings in non-polling mode
    osal_task_t rxthread;                   //!< receiver thread handle
    int rxthreadrunning;                    //!< receiver thread running flag
                                            
    stream_write_t stream_write;
    stream_read_t  stream_read;
} hw_stream_t;

#ifdef __cplusplus
extern "C" {
#endif

//! Opens EtherCAT hw device.
/*!
 * \param[in]   phw             Pointer to hw handle. 
 * \param[in]   stream_read     Function pointer to read data.
 * \param[in]   stream_write    Function pointer to write data.
 *
 * \return 0 or negative error code
 */
int hw_device_stream_open(struct hw_stream *phw, struct ec *pec, stream_read_t stream_read, stream_write_t stream_write);

#ifdef __cplusplus
}
#endif

#endif // LIBETHERCAT_HW_STREAM_H
