#include "mbx.h"
#include "ec.h"

#include <string.h>

//! check if mailbox is empty
/*!
 * \param pec pointer to ethercat master
 * \param slave slave number
 * \param mbx_nr number of mailbox
 * \return full (0) or empty (1)
 */
int ec_mbx_is_empty(ec_t *pec, uint16_t slave, uint8_t mbx_nr) {
    uint16_t wkc = 0;
    uint8_t sm_state = 0;
 
    ec_fprd(pec, pec->slaves[slave].fixed_address, EC_REG_SM0STAT + (mbx_nr * 8), 
            &sm_state, sizeof(sm_state), &wkc);

    if (!wkc)
        return -1;

    if ((sm_state & 0x08) == 0)
        return 1;

    return 0;
}

//! clears mailbox buffers 
/*!
 * \param pec pointer to ethercat master
 * \param slave slave number
 * \param read read mailbox (1) or write mailbox (0)
 */
void ec_mbx_clear(ec_t *pec, uint16_t slave, int read) {
    ec_slave_t *slv = &pec->slaves[slave];

    if (read)
        memset(slv->mbx_read.buf, 0, slv->sm[slv->mbx_read.sm_nr].len);
    else
        memset(slv->mbx_write.buf, 0, slv->sm[slv->mbx_write.sm_nr].len);
}

//! write mailbox to slave
/*!
 * \param pec pointer to ethercat master
 * \param slave slave number
 * \return working counter
 */
int ec_mbx_send(ec_t *pec, uint16_t slave) {
    uint16_t wkc = 0;
    ec_slave_t *slv = &pec->slaves[slave];

    if (!slv->sm[slv->mbx_write.sm_nr].len) {
        ec_log(__func__, "write mailbox on slave %d not available\n", slave);
        return 0;
    }

    // wait for read mailbox available 
    while (!ec_mbx_is_empty(pec, slave, slv->mbx_write.sm_nr) != 0) {
        ec_log(__func__, "waiting for mbx is full\n");
        struct timespec ts = { 0, 1000000 };
        nanosleep(&ts, NULL);
    }

    ec_fpwr(pec, slv->fixed_address, slv->sm[slv->mbx_write.sm_nr].adr, 
            slv->mbx_write.buf, slv->sm[slv->mbx_write.sm_nr].len, &wkc);

    if (!wkc)
        ec_log(__func__, "slave %d did not respond on writing to write mailbox\n",
                slave);

    return wkc;
}

//! read mailbox from slave
/*!
 * \param pec pointer to ethercat master
 * \param slave slave number
 * \return working counter
 */
int ec_mbx_receive(ec_t *pec, uint16_t slave) {
    uint16_t wkc = 0;
    ec_slave_t *slv = &pec->slaves[slave];

    if (!slv->sm[slv->mbx_read.sm_nr].len)
        return 0;

    int cnt = 100;

    // wait for read mailbox available 
    while (cnt-- > 0 && ec_mbx_is_empty(pec, slave, slv->mbx_read.sm_nr) != 0) {
        struct timespec ts = { 0, 1000000 };
        nanosleep(&ts, NULL);
    }

    if (cnt == 0) {
        ec_log(__func__, "slave %d read mailbox is still empty\n", 
                slave);
        return 0;
    }

    ec_fprd(pec, slv->fixed_address, slv->sm[slv->mbx_read.sm_nr].adr,
            slv->mbx_read.buf, slv->sm[slv->mbx_read.sm_nr].len, &wkc);

    if (!wkc)
        ec_log(__func__, "slave %d did not respond on reading from read mailbox\n", 
                slave);

   return wkc;
}

