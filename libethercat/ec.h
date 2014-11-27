//! ethercat master
/*!
 * author: Robert Burger
 *
 * $Id$
 */

/*
 * This file is part of libethercat.
 *
 * libethercat is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * libethercat is distributed in the hope that 
 * it will be useful, but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with libethercat
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __EC_H__
#define __EC_H__

#include <pthread.h>
#include <stdint.h>

#include "common.h"
#include "hw.h"
#include "regs.h"
#include "datagram.h"
#include "datagram_pool.h"

struct ec;

typedef struct idx_entry {
    uint8_t    idx;             //!< datagram index
    sem_t      waiter;          //!< waiter semaphore for synchronous access
    struct ec *pec;

    TAILQ_ENTRY(idx_entry) qh;  //!< queue handle
} idx_entry_t;
TAILQ_HEAD(idx_queue, idx_entry);
    
typedef struct ec_slave_mbx {
    uint8_t  sm_nr;
    uint8_t *buf;
} ec_slave_mbx_t;

typedef struct PACKED ec_slave_sm {
   uint16_t adr;
   uint16_t len;
   uint32_t flags;
} PACKED ec_slave_sm_t;

typedef struct PACKED ec_slave_fmmu {
    uint32_t log;
    uint16_t log_len;
    uint8_t  log_bit_start;
    uint8_t  log_bit_stop;
    uint16_t phys;
    uint8_t  phys_bit_start;
    uint8_t  type;
    uint8_t  active;
    uint8_t reserverd[3];
} PACKED ec_slave_fmmu_t;


typedef struct ec_slave {
    int16_t auto_inc_address;
    uint16_t fixed_address;

    uint32_t vendor_id;
    uint32_t product_code;

    uint8_t sm_ch;      //!< number of sync manager channels
    uint8_t fmmu_ch;    //!< number of fmmu channels
    int ram_size;       //!< ram size in bytes
    uint16_t features;  //!< fmmu operation, dc available

    ec_slave_sm_t *sm;
    ec_slave_fmmu_t *fmmu;

    ec_slave_mbx_t mbx_read;
    ec_slave_mbx_t mbx_write;
} ec_slave_t;

typedef struct ec {
    hw_t *phw;
    datagram_pool_t *pool;

    struct idx_queue idx;

    int slave_cnt;
    ec_slave_t *slaves;
} ec_t;

void ec_log(const char *pre, const char *format, ...);

//! open ethercat master
/*!
 * \param ppec return value for ethercat master pointer
 * \param ifname ethercat master interface name
 * \param prio receive thread priority
 * \param cpumask receive thread cpumask
 * \return 0 on succes, otherwise error code
 */
int ec_open(ec_t **ppec, const char *ifname, int prio, int cpumask);

//! closes ethercat master
/*!
 * \param pec pointer to ethercat master
 * \return 0 on success 
 */
int ec_close(ec_t *pec);

//! get next free index entry
/*!
 * \param pec pointer to ethercat master
 * \param entry return entry of next free index 
 * \return 0 on succes, otherwise error code
 */
int ec_index_get(ec_t *pec, struct idx_entry **entry);

//! returns index entry
/*!
 * \param pec pointer to ethercat master
 * \param entry return index entry 
 * \return 0 on succes, otherwise error code
 */
int ec_index_put(ec_t *pec, struct idx_entry *entry);

//! syncronous ethercat read/write
/*!
 * \param pec pointer to ethercat master
 * \param cmd ethercat command
 * \param adr 32-bit address of slave
 * \param data data buffer to read/write 
 * \param datalen length of data
 * \param wkc return value for working counter
 * \return 0 on succes, otherwise error code
 */
int ec_transceive(ec_t *pec, uint8_t cmd, uint32_t adr, 
        uint8_t *data, size_t datalen, uint16_t *wkc);

//! asyncronous ethercat read/write, answer don't care
/*!
 * \param pec pointer to ethercat master
 * \param cmd ethercat command
 * \param adr 32-bit address of slave
 * \param data data buffer to read/write 
 * \param datalen length of data
 * \return 0 on succes, otherwise error code
 */
int ec_transmit_no_reply(ec_t *pec, uint8_t cmd, uint32_t adr, 
        uint8_t *data, size_t datalen);

#define ec_to_adr(ado, adp) \
    ((uint32_t)(ado) << 16) | ((adp) & 0xFFFF)

#define ec_brd(pec, ado, data, datalen, wkc) \
    ec_transceive((pec), EC_CMD_BRD, ((uint32_t)(ado) << 16), (uint8_t *)(data), (datalen), (wkc))
#define ec_bwr(pec, ado, data, datalen, wkc) \
    ec_transceive((pec), EC_CMD_BWR, ((uint32_t)(ado) << 16), (uint8_t *)(data), (datalen), (wkc))
#define ec_brw(pec, ado, data, datalen, wkc) \
    ec_transceive((pec), EC_CMD_BRW, ((uint32_t)(ado) << 16), (uint8_t *)(data), (datalen), (wkc))

#define ec_aprd(pec, adp, ado, data, datalen, wkc) \
    ec_transceive((pec), EC_CMD_APRD, ((uint32_t)(ado) << 16) | ((adp) & 0xFFFF), \
            (uint8_t *)(data), (datalen), (wkc))
#define ec_apwr(pec, adp, ado, data, datalen, wkc) \
    ec_transceive((pec), EC_CMD_APWR, ((uint32_t)(ado) << 16) | ((adp) & 0xFFFF), \
            (uint8_t *)(data), (datalen), (wkc))
#define ec_aprw(pec, adp, ado, data, datalen, wkc) \
    ec_transceive((pec), EC_CMD_APRW, ((uint32_t)(ado) << 16) | ((adp) & 0xFFFF), \
            (uint8_t *)(data), (datalen), (wkc))

#define ec_fprd(pec, adp, ado, data, datalen, wkc) \
    ec_transceive((pec), EC_CMD_FPRD, ((uint32_t)(ado) << 16) | ((adp) & 0xFFFF), \
            (uint8_t *)(data), (datalen), (wkc))
#define ec_fpwr(pec, adp, ado, data, datalen, wkc) \
    ec_transceive((pec), EC_CMD_FPWR, ((uint32_t)(ado) << 16) | ((adp) & 0xFFFF), \
            (uint8_t *)(data), (datalen), (wkc))
#define ec_fprw(pec, adp, ado, data, datalen, wkc) \
    ec_transceive((pec), EC_CMD_FPRW, ((uint32_t)(ado) << 16) | ((adp) & 0xFFFF), \
            (uint8_t *)(data), (datalen), (wkc))

#endif // __EC_H__

