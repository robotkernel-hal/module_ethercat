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

    if (!ec_mbx_is_empty(pec, slave, slv->mbx_write.sm_nr)) {
        ec_log(__func__, "write mailbox on slave %d not empty\n", slave);
        return 0;
    }

    ec_fpwr(pec, slv->fixed_address, slv->sm[slv->mbx_write.sm_nr].adr, 
            slv->mbx_write.buf, slv->sm[slv->mbx_write.sm_nr].len, &wkc);

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

//    uint16 mbxro,mbxl,configadr;
//    int wkc=0;
//    int wkc2;
//    uint16 SMstat;
//    uint8 SMcontr;
//    ec_mbxheadert *mbxh;
//    ec_emcyt *EMp;
//    ec_mbxerrort *MBXEp;
//
//    configadr = context->slavelist[slave].configadr;
//    mbxl = context->slavelist[slave].mbx_rl;
        
    // wait for read mailbox available 
    while (ec_mbx_is_empty(pec, slave, slv->mbx_read.sm_nr) != 0) {
        struct timespec ts = { 0, 1000000 };
        nanosleep(&ts, NULL);
    }

//    ec_log(__func__, "read mailbox on slave %d available\n", slave);

//    uint8_t mbx_buffer2[140];
//    memset(mbx_buffer2, 0, sizeof(mbx_buffer2));
    ec_fprd(pec, slv->fixed_address, slv->sm[slv->mbx_read.sm_nr].adr,
            slv->mbx_read.buf, slv->sm[slv->mbx_read.sm_nr].len, &wkc);

//      if ((wkc > 0) && ((SMstat & 0x08) > 0)) /* read mailbox available ? */
//      {
//         mbxro = context->slavelist[slave].mbx_ro;
//         mbxh = (ec_mbxheadert *)mbx;
//         do
//         {
//            wkc = ecx_FPRD(context->port, configadr, mbxro, mbxl, mbx, EC_TIMEOUTRET); /* get mailbox */
//            if ((wkc > 0) && ((mbxh->mbxtype & 0x0f) == 0x00)) /* Mailbox error response? */
//            {
//               MBXEp = (ec_mbxerrort *)mbx;
//               ecx_mbxerror(context, slave, etohs(MBXEp->Detail));
//               wkc = 0; /* prevent emergency to cascade up, it is already handled. */
//            }
//            else if ((wkc > 0) && ((mbxh->mbxtype & 0x0f) == 0x03)) /* CoE response? */
//            {
//               EMp = (ec_emcyt *)mbx;
//               if ((etohs(EMp->CANOpen) >> 12) == 0x01) /* Emergency request? */
//               {
//                  ecx_mbxemergencyerror(context, slave, etohs(EMp->ErrorCode), EMp->ErrorReg,
//                          EMp->bData, etohs(EMp->w1), etohs(EMp->w2));
//                  wkc = 0; /* prevent emergency to cascade up, it is already handled. */
//               }
//            }
//            else
//            {
//               if (wkc <= 0) /* read mailbox lost */
//               {
//                  SMstat ^= 0x0200; /* toggle repeat request */
//                  SMstat = htoes(SMstat);
//                  wkc2 = ecx_FPWR(context->port, configadr, ECT_REG_SM1STAT, sizeof(SMstat), &SMstat, EC_TIMEOUTRET);
//                  SMstat = etohs(SMstat);
//                  SMcontr = 0;
//                  do /* wait for toggle ack */
//                  {
//                     
//                     wkc2 = ecx_FPRD(context->port, configadr, ECT_REG_SM1CONTR, sizeof(SMcontr), &SMcontr, EC_TIMEOUTRET);
//                   } while (((wkc2 <= 0) || ((SMcontr & 0x02) != (HI_BYTE(SMstat) & 0x02))) && (osal_timer_is_expired(&timer) == FALSE));
//                  do /* wait for read mailbox available */
//                  {
//                     wkc2 = ecx_FPRD(context->port, configadr, ECT_REG_SM1STAT, sizeof(SMstat), &SMstat, EC_TIMEOUTRET);
//                     SMstat = etohs(SMstat);
//                     if (((SMstat & 0x08) == 0) && (timeout > EC_LOCALDELAY))
//                     {
//                        osal_usleep(EC_LOCALDELAY);
//                     }
//                  } while (((wkc2 <= 0) || ((SMstat & 0x08) == 0)) && (osal_timer_is_expired(&timer) == FALSE));
//               }
//            }
//         } while ((wkc <= 0) && (osal_timer_is_expired(&timer) == FALSE)); /* if WKC<=0 repeat */
//      }
//      else /* no read mailbox available */
//      {
//          wkc = 0;
//      }
//   }

   return wkc;
}
