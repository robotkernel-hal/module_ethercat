
/**
 * \file hw_pikeos.c
 *
 * \author Robert Burger <robert.burger@dlr.de>
 *
 * \date 24 Nov 2016
 *
 * \brief hardware access functions
 *
 */

/*
 * This file is part of libethercat.
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

#include "hw_stream.h"

#include <libethercat/config.h>
#include <libethercat/hw.h>
#include <libethercat/ec.h>
#include <libethercat/idx.h>
#include <libethercat/error_codes.h>
#include <libethercat/hw.h>
#include <libethercat/pool.h>

#include <libosal/io.h>

#include <assert.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <inttypes.h>

#if LIBETHERCAT_HAVE_NETINET_IN_H == 1
#include <netinet/in.h>
#endif

#if LIBETHERCAT_HAVE_WINSOCK_H == 1
#include <winsock.h>
#endif

#if LIBETHERCAT_HAVE_NET_UTIL_INET_H == 1
#include <net/util/inet.h>
#endif
    
// forward decls
int hw_device_stream_recv(struct hw_common *phw);
int hw_device_stream_get_tx_buffer(struct hw_common *phw, ec_frame_t **ppframe);
int hw_device_stream_send(struct hw_common *phw, ec_frame_t *pframe, pooltype_t pool_type);
void hw_device_stream_send_finished(struct hw_common *phw);
int hw_device_stream_close(struct hw_common *phw);

static void hw_device_stream_recv_internal(struct hw_stream *phw_stream);

//! Opens EtherCAT hw device.
/*!
 * \param[in]   phw             Pointer to hw handle. 
 * \param[in]   stream_read     Function pointer to read data.
 * \param[in]   stream_write    Function pointer to write data.
 *
 * \return 0 or negative error code
 */
int hw_device_stream_open(struct hw_stream *phw, stream_read_t stream_read, stream_write_t stream_write) {
    assert(phw != NULL);

    int ret = EC_OK;

    phw->stream_read = stream_read;
    phw->stream_write = stream_write;

    phw->common.send = hw_device_stream_send;
    phw->common.recv = hw_device_stream_recv;
    phw->common.send_finished = hw_device_stream_send_finished;
    phw->common.get_tx_buffer = hw_device_stream_get_tx_buffer;
    phw->common.close = hw_device_stream_close;
    phw->common.mtu_size = 1480;

    return ret;
}

int hw_device_stream_recv(struct hw_common *phw) {
    assert(phw != NULL);

    struct hw_stream *phw_stream = container_of(phw, struct hw_stream, common);

    hw_device_stream_recv_internal(phw_stream);

    return EC_OK;
}

//! Receive a frame from an EtherCAT hw device.
/*!
 * \param[in]   phw         Pointer to hw handle. 
 *
 * \return 0 or negative error code
 */
void hw_device_stream_recv_internal(struct hw_stream *phw_stream) {
    assert(phw_stream != NULL);

    // cppcheck-suppress misra-c2012-11.3
    ec_frame_t *pframe = (ec_frame_t *) &phw_stream->recv_frame;

    // using tradional recv function
    osal_ssize_t bytesrx = phw_stream->stream_read(pframe, ETH_FRAME_LEN);

    if (bytesrx > 0) {
        hw_process_rx_frame(&phw_stream->common, pframe);
    }
}

//! Get a free tx buffer from underlying hw device.
/*!
 * \param[in]   phw         Pointer to hw handle. 
 * \param[in]   ppframe     Pointer to return frame buffer pointer.
 *
 * \return 0 or negative error code
 */
int hw_device_stream_get_tx_buffer(struct hw_common *phw, ec_frame_t **ppframe) {
    assert(phw != NULL);
    assert(ppframe != NULL);

    int ret = EC_ERROR_UNAVAILABLE;
    ec_frame_t *pframe = NULL;
    struct hw_stream *phw_stream = container_of(phw, struct hw_stream, common);

    // cppcheck-suppress misra-c2012-11.3
    pframe = (ec_frame_t *)phw_stream->send_frame;

    // reset length to send new frame
    pframe->ethertype = htons(ETH_P_ECAT);
    pframe->type = 0x01;
    pframe->len = sizeof(ec_frame_t);

    *ppframe = pframe;

    return ret;
}

//! Send a frame from an EtherCAT hw device.
/*!
 * \param[in]   phw         Pointer to hw handle. 
 * \param[in]   pframe      Pointer to frame buffer.
 * \param[in]   pool_type   Pool type to distinguish between high and low prio frames.
 *
 * \return 0 or negative error code
 */
int hw_device_stream_send(struct hw_common *phw, ec_frame_t *pframe, pooltype_t pool_type) {
    assert(phw != NULL);
    assert(pframe != NULL);

    (void)pool_type;

    int ret = EC_OK;
    struct hw_stream *phw_stream = container_of(phw, struct hw_stream, common);

    // no more datagrams need to be sent or no more space in frame
    osal_ssize_t bytestx = phw_stream->stream_write(pframe, pframe->len);

    if ((osal_ssize_t)pframe->len != bytestx) {
        ec_log(1, "HW_TX", "got only %" PRId64 " bytes out of %d bytes "
                "through.\n", bytestx, pframe->len);

        if (bytestx == -1) {
            ec_log(1, "HW_TX", "error: %s\n", strerror(errno));
        }

        ret = EC_ERROR_HW_SEND;
    }
    
    phw_stream->common.bytes_sent += bytestx;

    return ret;
}

//! Doing internal stuff when finished sending frames
/*!
 * \param[in]   phw         Pointer to hw handle.
 */
void hw_device_stream_send_finished(struct hw_common *phw) {
    (void)phw;
}

