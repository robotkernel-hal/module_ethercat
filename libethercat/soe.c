//! ethercat servodrive over ethercat mailbox handling
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

#include "libethercat/mbx.h"
#include "libethercat/soe.h"
#include "libethercat/timer.h"

#include <stdio.h>
#include <string.h>

typedef struct PACKED ec_soe_request {
    ec_mbx_header_t mbx_hdr;
    ec_soe_header_t soe_hdr;
    ec_data_t       data;
} ec_soe_request_t;

//! soe op codes
enum {
    EC_SOE_READ_REQ     = 0x01,
    EC_SOE_READ_RES,
    EC_SOE_WRITE_REQ,
    EC_SOE_WRITE_RES,
    EC_SOE_NOTIFICATION,
    EC_SOE_EMERGENCY
};

int ec_soe_read(ec_t *pec, uint16_t slave, uint8_t atn, uint16_t idn, 
        uint8_t elements, uint8_t *buf, size_t *len) {
    ec_slave_t *slv = (ec_slave_t *)&pec->slaves[slave];

    if (!slv->eeprom.mbx_supported)
        return 0;

    ec_soe_request_t *write_buf = 
        (ec_soe_request_t *)(slv->mbx_write.buf);

    // empty mailbox if anything in
    ec_mbx_clear(pec, slave, 1);
    ec_mbx_receive(pec, slave, 0);

    // mailbox header
    ec_mbx_clear(pec, slave, 0);
    write_buf->mbx_hdr.length   = sizeof(ec_soe_header_t);
    write_buf->mbx_hdr.address  = 0x0000;
    write_buf->mbx_hdr.priority = 0x00;
    write_buf->mbx_hdr.mbxtype  = EC_MBX_SOE;

    // soe header
    write_buf->soe_hdr.op_code    = EC_SOE_READ_REQ;
    write_buf->soe_hdr.incomplete = 0;
    write_buf->soe_hdr.error      = 0;
    write_buf->soe_hdr.atn        = atn;
    write_buf->soe_hdr.elements   = elements;
    write_buf->soe_hdr.idn        = idn;

    // send request
    if (!ec_mbx_send(pec, slave, EC_DEFAULT_TIMEOUT_MBX)) {
        ec_log(10, "ec_soe_read", "error on writing send mailbox\n");
        return -1;
    }

    uint8_t *to = buf;
    size_t left_len = *len;
    ec_soe_request_t *read_buf  = 
        (ec_soe_request_t *)(slv->mbx_read.buf); 

    while (1) {
        // wait for answer
        ec_mbx_clear(pec, slave, 1);
        if (!ec_mbx_receive(pec, slave, EC_DEFAULT_TIMEOUT_MBX)) {
            ec_log(10, "ec_soe_read", "error on reading receive mailbox\n");
            return -1;
        }

        // check for correct op_code
        if (!read_buf->soe_hdr.op_code != EC_SOE_READ_RES)
            continue; // TODO handle unexpected answer

        size_t read_len = read_buf->mbx_hdr.length - sizeof(ec_soe_header_t);
        memcpy(to, &read_buf->data, min(read_len, left_len));
        to += read_len;
        left_len -= read_len;

        if (!read_buf->soe_hdr.incomplete)
            break;
    }

    return 0;
}

int ec_soe_write(ec_t *pec, uint16_t slave, uint8_t atn, uint16_t idn, 
        uint8_t elements, uint8_t *buf, size_t len) {
    ec_slave_t *slv = (ec_slave_t *)&pec->slaves[slave];

    if (!slv->eeprom.mbx_supported)
        return 0;

    ec_soe_request_t *write_buf = 
        (ec_soe_request_t *)(slv->mbx_write.buf);

    // empty mailbox if anything in
    ec_mbx_clear(pec, slave, 1);
    ec_mbx_receive(pec, slave, 0);

    // mailbox header
    ec_mbx_clear(pec, slave, 0);
    write_buf->mbx_hdr.length   = sizeof(ec_soe_header_t);
    write_buf->mbx_hdr.address  = 0x0000;
    write_buf->mbx_hdr.priority = 0x00;
    write_buf->mbx_hdr.mbxtype  = EC_MBX_SOE;

    // soe header
    write_buf->soe_hdr.op_code    = EC_SOE_WRITE_REQ;
    write_buf->soe_hdr.error      = 0;
    write_buf->soe_hdr.atn        = atn;
    write_buf->soe_hdr.elements   = elements;
    write_buf->soe_hdr.idn        = idn;

    uint8_t *from = buf;
    size_t left_len = len;
    size_t mbx_len = slv->sm[0].len 
        - sizeof(ec_mbx_header_t) - sizeof(ec_soe_header_t);
    ec_soe_request_t *read_buf  = 
        (ec_soe_request_t *)(slv->mbx_read.buf); 

    while (1) {
        size_t send_len = min(left_len, mbx_len);
        write_buf->mbx_hdr.length += send_len;
        memcpy(&write_buf->data, from, send_len);
        from += send_len;
        left_len -= send_len;

        if (left_len) {
            write_buf->soe_hdr.incomplete = 1;
            write_buf->soe_hdr.fragments_left = left_len / mbx_len + 1;
        } else {
            write_buf->soe_hdr.incomplete = 0;
            write_buf->soe_hdr.idn = idn;
        }

        // send request
        if (!ec_mbx_send(pec, slave, EC_DEFAULT_TIMEOUT_MBX)) {
            ec_log(10, "ec_soe_read", "error on writing send mailbox\n");
            return -1;
        }

        // wait for answer
        ec_mbx_clear(pec, slave, 1);
        if (!ec_mbx_receive(pec, slave, EC_DEFAULT_TIMEOUT_MBX)) {
            ec_log(10, "ec_soe_read", "error on reading receive mailbox\n");
            return -1;
        }

        // check for correct op_code
        if (!read_buf->soe_hdr.op_code != EC_SOE_WRITE_RES)
            continue; // TODO handle unexpected answer

        if (!left_len)
            break;
    }

    return 0;
}

int ec_soe_generate_mapping_local(ec_t *pec, uint16_t slave, uint8_t atn, 
        uint16_t idn, int *bitsize) {
    int ret = 0, i;

    *bitsize = 0;

    // read size of mapping idn first
    uint16_t idn_len[2];
    size_t idn_len_size = sizeof(idn_len);
    if (ec_soe_read(pec, slave, atn, idn, EC_SOE_VALUE, 
                (uint8_t *)idn_len, &idn_len_size) != 0)
        return -1;

    // read mapping idn
    size_t idn_size = idn_len[0];
    uint16_t *idn_value = malloc(idn_size);
    if (ec_soe_read(pec, slave, atn, idn, EC_SOE_VALUE, 
                (uint8_t *)idn_value, &idn_size) != 0)
        return -1;

    // read all mapped idn's and add bit length, 
    // length is stored in idn attributes
    for (i = 0; i < (idn_len[0]/2); ++i) {
        uint16_t sub_idn = idn_value[i+2];
        ec_soe_idn_attribute_t sub_idn_attr;
        size_t sub_idn_attr_size = sizeof(sub_idn_attr);

        if (ec_soe_read(pec, slave, atn, sub_idn, EC_SOE_ATTRIBUTE, 
                    (uint8_t*)&sub_idn_attr, &sub_idn_attr_size) != 0)
            continue;

        // 0 = 8 bit, 1 = 16 bit, ...
        *bitsize += 8 << sub_idn_attr.length;
    }

    return ret;
}

int ec_soe_generate_mapping(ec_t *pec, uint16_t slave) {
    int atn;
    ec_slave_t *slv = (ec_slave_t *)&pec->slaves[slave];

    uint16_t idn_at = 16;
    int at_bits = 0, at_sm = 3;
    // generate at mapping over all at's of specified slave
    // at mapping is stored at idn 16 and should be written in preop
    // state by user
    for (atn = 0; atn < slv->eeprom.general.soe_channels; ++atn) {
        int bits = 0;
        if (ec_soe_generate_mapping_local(pec, slave, atn, 
                    idn_at, &bits) != 0)
            continue;

        at_bits += bits;
    }

    if (at_bits) {
        ec_log(10, __func__, "slave %2d: sm%d length bits %d, bytes %d\n", 
                slave, at_sm, at_bits, (at_bits + 7) / 8);

        if (slv->sm && slv->sm_ch > at_sm)
            slv->sm[at_sm].len = (at_bits + 7) / 8;
    }

    uint16_t idn_mdt = 24;
    int mdt_bits = 0, mdt_sm = 2;
    // generate mdt mapping over all mdt's of specified slave
    // mdt mapping is stored at idn 24 and should be written in preop
    // state by user
    for (atn = 0; atn < slv->eeprom.general.soe_channels; ++atn) {
        int bits = 0;
        if (ec_soe_generate_mapping_local(pec, slave, atn, 
                    idn_mdt, &bits) != 0)
            continue;

        mdt_bits += bits;
    }

    if (mdt_bits) {
        ec_log(10, __func__, "slave %2d: sm%d length bits %d, bytes %d\n", 
                slave, mdt_sm, mdt_bits, (mdt_bits + 7) / 8);

        if (slv->sm && slv->sm_ch > mdt_sm)
            slv->sm[mdt_sm].len = (mdt_bits + 7) / 8;
    }

    return -1;
}
