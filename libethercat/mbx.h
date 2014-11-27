#ifndef __MBX_H__
#define __MBX_H__

#include "common.h"
#include "ec.h"

//! mailbox types
enum {
   EC_MBX_ERR = 0x00,   //!< error mailbox
   EC_MBX_AOE,          //!< ADS over EtherCAT mailbox
   EC_MBX_EOE,          //!< Ethernet over EtherCAT mailbox
   EC_MBX_COE,          //!< CANopen over EtherCAT mailbox
   EC_MBX_FOE,          //!< File over EtherCAT mailbox
   EC_MBX_SOE,          //!< Servo over EtherCAT mailbox
   EC_MBX_VOE = 0x0f    //!< Vendor over EtherCAT mailbox
};

//! ethercat mailbox header
typedef struct PACKED ec_mbxheader {
   uint16_t  length;
   uint16_t  address;
   uint8_t   priority;
   uint8_t   mbxtype;
} PACKED ec_mbxheader_t;

//! ethercat mailbox data
typedef struct PACKED ec_mbx {
    ec_mbxheader_t mbx_hdr;
    ec_data_t      mbx_data;
} PACKED ec_mbx_t;

//! check if mailbox is empty
/*!
 * \param pec pointer to ethercat master
 * \param slave slave number
 * \param mbx_nr number of mailbox
 * \return full (0) or empty (1)
 */
int ec_mbx_is_empty(ec_t *pec, uint16_t slave, uint8_t mbx_nr);

//! clears mailbox buffers 
/*!
 * \param pec pointer to ethercat master
 * \param slave slave number
 * \param read read mailbox (1) or write mailbox (0)
 */
void ec_mbx_clear(ec_t *pec, uint16_t slave, int read);

//! write mailbox to slave
/*!
 * \param pec pointer to ethercat master
 * \param slave slave number
 * \return working counter
 */
int ec_mbx_send(ec_t *pec, uint16_t slave);

//! read mailbox from slave
/*!
 * \param pec pointer to ethercat master
 * \param slave slave number
 * \return working counter
 */
int ec_mbx_receive(ec_t *pec, uint16_t slave);

#endif // __MBX_H__

