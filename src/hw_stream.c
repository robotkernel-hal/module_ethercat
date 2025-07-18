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
void *hw_device_stream_rx_thread(void *arg);

static void hw_device_stream_recv_internal(struct hw_stream *phw_stream);

static const osal_uint8_t mac_dest[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
static const osal_uint8_t mac_src[] = {0x00, 0x30, 0x64, 0x0f, 0x83, 0x35};

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
        stream_read_t stream_read, stream_write_t stream_write, int prio, int affinity)
{
    assert(phw != NULL);

    int ret = EC_OK;

    hw_open(&phw->common, pec);

    phw->user = user;
    phw->stream_read = stream_read;
    phw->stream_write = stream_write;

    phw->common.send = hw_device_stream_send;
    phw->common.recv = hw_device_stream_recv;
    phw->common.send_finished = hw_device_stream_send_finished;
    phw->common.get_tx_buffer = hw_device_stream_get_tx_buffer;
    phw->common.close = hw_device_stream_close;
    phw->common.mtu_size = 1480;

    if (ret == EC_OK) {
        phw->rxthreadrunning = 1;
        osal_task_attr_t attr;
        attr.policy = OSAL_SCHED_POLICY_FIFO;
        attr.priority = prio;
        attr.affinity = affinity;
        (void)strcpy(&attr.task_name[0], "ecat.rx");
        osal_task_create(&phw->rxthread, &attr, hw_device_stream_rx_thread, phw);
    }

    return ret;
}

//! receiver thread
void *hw_device_stream_rx_thread(void *arg) {
    // cppcheck-suppress misra-c2012-11.5
    struct hw_stream *phw_stream = (struct hw_stream *) arg;
    ec_t *pec = phw_stream->common.pec;

    assert(phw_stream != NULL);
    
    osal_task_sched_priority_t rx_prio;
    if (osal_task_get_priority(&phw_stream->rxthread, &rx_prio) != OSAL_OK) {
        rx_prio = 0;
    }

    ec_log(10, __func__, "receive thread running (prio %d)\n", rx_prio);

    while (phw_stream->rxthreadrunning != 0) {
        (void)hw_device_stream_recv(&phw_stream->common);
    }
    
    ec_log(10, __func__, "receive thread stopped\n");
    
    return NULL;
}

int hw_device_stream_close(struct hw_common *phw) {
    assert(phw != NULL);
    struct hw_stream *phw_stream = container_of(phw, struct hw_stream, common);

    phw_stream->rxthreadrunning = 0;
    osal_task_join(&phw_stream->rxthread, NULL);

    return EC_OK;
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
    osal_ssize_t bytesrx = phw_stream->stream_read(phw_stream->user, pframe, ETH_FRAME_LEN);

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
    (void)memcpy(pframe->mac_dest, mac_dest, 6);
    (void)memcpy(pframe->mac_src, mac_src, 6);
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
    ec_t *pec = phw->pec;
    struct hw_stream *phw_stream = container_of(phw, struct hw_stream, common);

    // no more datagrams need to be sent or no more space in frame
    osal_ssize_t bytestx = phw_stream->stream_write(phw_stream->user, pframe, pframe->len);

    if ((osal_ssize_t)pframe->len != bytestx) {
        ec_log(1, __func__, "got only %" PRId64 " bytes out of %d bytes "
                "through.\n", bytestx, pframe->len);

        if (bytestx == -1) {
            ec_log(1, __func__, "error: %s\n", strerror(errno));
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

