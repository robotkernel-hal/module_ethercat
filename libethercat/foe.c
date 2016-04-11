//! ethercat file over ethercat mailbox handling
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

#include "libethercat/foe.h"
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

//! read file over foe
/*!
 * \param pec pointer to ethercat master
 * \param slave slave number
 * \param password foe password
 * \param remote_file_name file_name to read from
 * \param local_file_name file_name to store file to
 * \return working counter
 */
int ec_foe_read(ec_t *pec, uint16_t slave, uint32_t password,
        char remote_file_name[MAX_FILE_NAME_SIZE], 
        char *local_file_name) {
    int wkc = -1;
    ec_slave_t *slv = (ec_slave_t *)&pec->slaves[slave];

    ec_log(10, __func__, "mbx support %X\n", slv->eeprom.mbx_supported);
    if (!(slv->eeprom.mbx_supported & EC_EEPROM_MBX_FOE))
        return 0;

    ec_log(10, __func__, "foe support! %d\n", __LINE__);

    pthread_mutex_lock(&slv->mbx_lock);

    ec_foe_rw_request_t *write_buf = 
        (ec_foe_rw_request_t *)(slv->mbx_write.buf);
    ec_log(10, __func__, "foe support! %d\n", __LINE__);

    // empty mailbox if anything in
    ec_mbx_clear(pec, slave, 1);
    ec_mbx_receive(pec, slave, 0);

    ec_log(10, __func__, "foe support! %d\n", __LINE__);
    // mailbox header
    ec_mbx_clear(pec, slave, 0);
    write_buf->mbx_hdr.length    = 6 + MAX_FILE_NAME_SIZE; 
    write_buf->mbx_hdr.address   = 0x0000;
    write_buf->mbx_hdr.priority  = 0x00;
    write_buf->mbx_hdr.mbxtype   = EC_MBX_FOE;

    ec_log(10, __func__, "foe support! %d\n", __LINE__);
    // foe header
    write_buf->foe_hdr.op_code   = EC_FOE_OP_CODE_READ_REQUEST;
    write_buf->foe_hdr.reserved  = 0x00;

    ec_log(10, __func__, "foe support! %d\n", __LINE__);
    // read request
    write_buf->password          = password;
    memcpy(write_buf->file_name, remote_file_name, MAX_FILE_NAME_SIZE);

    ec_log(10, __func__, "foe support! %d\n", __LINE__);
    // send request
    wkc = ec_mbx_send(pec, slave, EC_DEFAULT_TIMEOUT_MBX);
    if (!wkc) {
        ec_log(10, __func__, "error on writing send mailbox\n");
        goto exit;
    }

    ec_log(10, __func__, "foe support! %d\n", __LINE__);
    ec_foe_ack_request_t *write_buf_ack = 
        (ec_foe_ack_request_t *)(slv->mbx_write.buf);
    ec_foe_data_request_t *read_buf_data = 
        (ec_foe_data_request_t *)(slv->mbx_read.buf);

    int fd = open(local_file_name, O_CREAT | O_RDWR);

    ec_log(10, __func__, "foe support! %d\n", __LINE__);
    while (1) {
        // wait for answer
        ec_mbx_clear(pec, slave, 1);
        wkc = ec_mbx_receive(pec, slave, EC_DEFAULT_TIMEOUT_MBX);
        if (!wkc) {
            ec_log(10, __func__, "error on reading receive mailbox\n");
            goto exit;
        }

    ec_log(10, __func__, "foe support! %d\n", __LINE__);
        if (read_buf_data->foe_hdr.op_code != EC_FOE_OP_CODE_DATA_REQUEST) {
            ec_log(10, __func__, "got foe mbx %X, op_code %X\n", write_buf->mbx_hdr.mbxtype, read_buf_data->foe_hdr.op_code);
            continue;
//            goto exit;
        }

        size_t len = read_buf_data->mbx_hdr.length - 6;
        write(fd, read_buf_data->data.bdata, len);

    ec_log(10, __func__, "foe support! %d\n", __LINE__);
        // everthing is fine, send ack 
        // mailbox header
        ec_mbx_clear(pec, slave, 0);
        write_buf_ack->mbx_hdr.length    = 6 + MAX_FILE_NAME_SIZE; 
        write_buf_ack->mbx_hdr.address   = 0x0000;
        write_buf_ack->mbx_hdr.priority  = 0x00;
        write_buf_ack->mbx_hdr.mbxtype   = EC_MBX_FOE;
    ec_log(10, __func__, "foe support! %d\n", __LINE__);

        // foe
        write_buf_ack->foe_hdr.op_code   = EC_FOE_OP_CODE_ACK_REQUEST;
        write_buf_ack->foe_hdr.reserved  = 0x00;
        write_buf_ack->packet_nr         = read_buf_data->packet_nr;

        wkc = ec_mbx_send(pec, slave, EC_DEFAULT_TIMEOUT_MBX);
        if (!wkc) {
            ec_log(10, __func__, "error on writing send mailbox\n");
            goto exit;
        }

    ec_log(10, __func__, "foe support! %d\n", __LINE__);
        // compare length + mbx_hdr_size with mailbox size
        if ((read_buf_data->mbx_hdr.length + 6) < slv->sm[1].len)
            break;
    }

    ec_log(10, __func__, "foe support! %d\n", __LINE__);
exit:
    close(fd);

    ec_log(10, __func__, "foe support! %d\n", __LINE__);
    // reset mailbox state 
    if (slv->mbx_read.sm_state) {
        *slv->mbx_read.sm_state = 0;
        slv->mbx_read.skip_next = 1;
    }

    pthread_mutex_unlock(&slv->mbx_lock);
    return wkc;
}

