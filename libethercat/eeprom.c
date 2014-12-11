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

#include "eeprom.h"
#include "ec.h"

//! set eeprom control to ethercat master
/*!
 * \param pec pointer to ethercat master
 * \param slave ethercat slave number
 * \return 0 on success
 */
int ec_eepromconfig(ec_t *pec, uint16_t slave) {
    uint16_t wkc, eepctl = 2, cnt = 10;

    do {
        ec_fpwr(pec, pec->slaves[slave].fixed_address, EC_REG_EEPCFG, 
                (uint8_t *)&eepctl, sizeof(eepctl), &wkc);
    } while (--cnt > 0 && wkc != 1);
    if (wkc != 1)
        ec_log(__func__, "slave %d did not accept forcing eeprom to pdi\n", slave);
    
    eepctl = 0; cnt = 10;
    do {
        ec_fpwr(pec, pec->slaves[slave].fixed_address, EC_REG_EEPCFG, 
                (uint8_t *)&eepctl, sizeof(eepctl), &wkc);
    } while (--cnt > 0 && wkc != 1);
    if (wkc != 1)
        ec_log(__func__, "slave %d did not accept setting eeprom to ethercat\n", slave);
    
    ec_fprd(pec, pec->slaves[slave].fixed_address, EC_REG_EEPCFG, 
            (uint8_t *)&eepctl, sizeof(eepctl), &wkc);

    return 0;
}

//! read 32-bit word of eeprom
/*!
 * \param pec pointer to ethercat master
 * \param slave ethercat slave number
 * \param eepadr address in eeprom
 * \param returns data value
 * \return 0 on success
 */
int ec_eepromread(ec_t *pec, uint16_t slave, uint32_t eepadr, uint32_t *data) {
    ec_eepromconfig(pec, slave);
    
    int ret = 0, retry_cnt = 100;
    uint16_t wkc = 0, eepcsr = 0x0100; // read access
   
    do {
        eepcsr = 0;
        ec_fprd(pec, pec->slaves[slave].fixed_address, EC_REG_EEPCTL,
                (uint8_t *)&eepcsr, sizeof(eepcsr), &wkc);
        if (--retry_cnt == 0) {
            ec_log("EEPROM_READ", "reading eepctl failed, wkc %d\n", wkc);
            ret = -1;
            goto func_exit;
        }
    } while (eepcsr & 0x0100);

    ec_fpwr(pec, pec->slaves[slave].fixed_address, EC_REG_EEPADR,
            (uint8_t *)&eepadr, sizeof(eepadr), &wkc);
    if (wkc != 1) {
        ec_log("EEPROM_READ", "writing eepadr failed\n");
        ret = -1;
        goto func_exit;
    }

    eepcsr = 0x0100;
    ec_fpwr(pec, pec->slaves[slave].fixed_address, EC_REG_EEPCTL,
            (uint8_t *)&eepcsr, sizeof(eepcsr), &wkc);
    if (wkc != 1) {
        ec_log("EEPROM_READ", "wirting eepctl failed\n");
        ret = -1;
        goto func_exit;
    }

    retry_cnt = 100;

    do {
        eepcsr = 0;
        ec_fprd(pec, pec->slaves[slave].fixed_address, EC_REG_EEPCTL,
                (uint8_t *)&eepcsr, sizeof(eepcsr), &wkc);
        if (--retry_cnt == 0) {
            ec_log("EEPROM_READ", "reading eepctl failed, wkc %d\n", wkc);
            ret = -1;
            goto func_exit;
        }
    } while (eepcsr & 0x0100);

    *data = 0;
    ec_fprd(pec, pec->slaves[slave].fixed_address, EC_REG_EEPDAT,
            (uint8_t *)data, sizeof(*data), &wkc);
    if (wkc != 1) {
        ec_log("EEPROM_READ", "reading data failed\n");
        ret = -1;
        goto func_exit;
    }

func_exit:

    return ret;
}

//! read a burst of eeprom
/*!
 * \param pec pointer to ethercat master
 * \param slave ethercat slave number
 * \param eepadr address in eeprom
 * \param buf return buffer
 * \param buflen length in bytes to return
 * \return 0 on success
 */
int ec_eepromread_len(ec_t *pec, uint16_t slave, uint32_t eepadr, uint8_t *buf, size_t buflen) {
    unsigned offset = 0, i;

    while (offset < buflen) {
        uint32_t val;
        ec_eepromread(pec, slave, eepadr+(offset/2), &val);

        for (i = 0; (offset < buflen) && (i < 4); ++i, ++offset)
            buf[offset] = ((uint8_t *)&val)[i];
    }

    return 0;
};

