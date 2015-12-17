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

#include "libethercat/eeprom.h"
#include "libethercat/ec.h"

#include <string.h>

//! set eeprom control to pdi
/*!
 * \param pec pointer to ethercat master
 * \param slave ethercat slave number
 * \return 0 on success
 */
int ec_eeprom_to_pdi(ec_t *pec, uint16_t slave) {
    uint16_t wkc, cnt = 10;
    uint8_t eepctl = 2;

    do {
        ec_fpwr(pec, pec->slaves[slave].fixed_address, EC_REG_EEPCFG, 
                (uint8_t *)&eepctl, sizeof(eepctl), &wkc);
    } while (--cnt > 0 && wkc != 1);
    if (wkc != 1)
        ec_log(10, __func__, "slave %2d did not accept forcing eeprom to pdi\n", slave);
    
    eepctl = 1; cnt = 10;
    do {
        ec_fpwr(pec, pec->slaves[slave].fixed_address, EC_REG_EEPCFG, 
                (uint8_t *)&eepctl, sizeof(eepctl), &wkc);
    } while (--cnt > 0 && wkc != 1);
    if (wkc != 1)
        ec_log(10, __func__, "slave %2d did not accept setting eeprom to pdi\n", slave);
    
    ec_fprd(pec, pec->slaves[slave].fixed_address, EC_REG_EEPCFG, 
            (uint8_t *)&eepctl, sizeof(eepctl), &wkc);

//    ec_log(100, __func__, "slave %2d eeprom control set to pdi (eepctl 0x%X)\n", 
//            slave, eepctl);
    return 0;
}

//! set eeprom control to ec
/*!
 * \param pec pointer to ethercat master
 * \param slave ethercat slave number
 * \return 0 on success
 */
int ec_eeprom_to_ec(struct ec *pec, uint16_t slave) {
    uint16_t wkc, cnt = 10;
    uint8_t eepctl = 2;

    do {
        ec_fpwr(pec, pec->slaves[slave].fixed_address, EC_REG_EEPCFG, 
                (uint8_t *)&eepctl, sizeof(eepctl), &wkc);
    } while (--cnt > 0 && wkc != 1);
    if (wkc != 1)
        ec_log(10, __func__, "slave %d did not accept forcing eeprom to pdi\n", slave);
    
    eepctl = 0; cnt = 10;
    do {
        ec_fpwr(pec, pec->slaves[slave].fixed_address, EC_REG_EEPCFG, 
                (uint8_t *)&eepctl, sizeof(eepctl), &wkc);
    } while (--cnt > 0 && wkc != 1);
    if (wkc != 1)
        ec_log(10, __func__, "slave %d did not accept setting eeprom to ethercat\n", slave);
    
    ec_fprd(pec, pec->slaves[slave].fixed_address, EC_REG_EEPCFG, 
            (uint8_t *)&eepctl, sizeof(eepctl), &wkc);

//    ec_log(100, __func__, "slave %2d eeprom control set to ec (eepctl 0x%X)\n", 
//            slave, eepctl);
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
    ec_eeprom_to_ec(pec, slave);
    
    int ret = 0, retry_cnt = 100;
    uint16_t wkc = 0, eepcsr = 0x0100; // read access
   
    do {
        eepcsr = 0;
        ec_fprd(pec, pec->slaves[slave].fixed_address, EC_REG_EEPCTL,
                (uint8_t *)&eepcsr, sizeof(eepcsr), &wkc);
        if (--retry_cnt == 0) {
            ec_log(10, "EEPROM_READ", "reading eepctl failed, wkc %d\n", wkc);
            ret = -1;
            goto func_exit;
        }
    } while (eepcsr & 0x0100);

    ec_fpwr(pec, pec->slaves[slave].fixed_address, EC_REG_EEPADR,
            (uint8_t *)&eepadr, sizeof(eepadr), &wkc);
    if (wkc != 1) {
        ec_log(10, "EEPROM_READ", "writing eepadr failed\n");
        ret = -1;
        goto func_exit;
    }

    eepcsr = 0x0100;
    ec_fpwr(pec, pec->slaves[slave].fixed_address, EC_REG_EEPCTL,
            (uint8_t *)&eepcsr, sizeof(eepcsr), &wkc);
    if (wkc != 1) {
        ec_log(10, "EEPROM_READ", "wirting eepctl failed\n");
        ret = -1;
        goto func_exit;
    }

    retry_cnt = 100;

    do {
        eepcsr = 0;
        ec_fprd(pec, pec->slaves[slave].fixed_address, EC_REG_EEPCTL,
                (uint8_t *)&eepcsr, sizeof(eepcsr), &wkc);
        if (--retry_cnt == 0) {
            ec_log(10, "EEPROM_READ", "reading eepctl failed, wkc %d\n", wkc);
            ret = -1;
            goto func_exit;
        }
    } while (eepcsr & 0x0100);

    *data = 0;
    ec_fprd(pec, pec->slaves[slave].fixed_address, EC_REG_EEPDAT,
            (uint8_t *)data, sizeof(*data), &wkc);
    if (wkc != 1) {
        ec_log(10, "EEPROM_READ", "reading data failed\n");
        ret = -1;
        goto func_exit;
    }

func_exit:
    ec_eeprom_to_pdi(pec, slave);

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

//! read out whole eeprom and categories
/*!
 * \param pec pointer to ethercat master
 * \param slave ethercat slave number
 */
void ec_eeprom_dump(ec_t *pec, uint16_t slave) {
    int cat_offset = EC_EEPROM_ADR_CAT_OFFSET;
    uint16_t size, cat_len, cat_type = 0;
    uint32_t value32;
    ec_slave_t *slv = &pec->slaves[slave];

#define eeprom(adr, mem) \
    ec_eepromread_len(pec, slave, (adr), (uint8_t *)&(mem), sizeof(mem));

    // read soem eeprom values
    eeprom(EC_EEPROM_ADR_VENDOR_ID,     slv->eeprom.vendor_id);
    eeprom(EC_EEPROM_ADR_PRODUCT_CODE,  slv->eeprom.product_code);
    eeprom(EC_EEPROM_ADR_MBX_SUPPORTED, slv->eeprom.mbx_supported);
    eeprom(EC_EEPROM_ADR_SIZE,          value32);
    eeprom(EC_EEPROM_ADR_MBX_RECV_OFF,  slv->eeprom.mbx_receive_offset);
    eeprom(EC_EEPROM_ADR_MBX_RECV_SIZE, slv->eeprom.mbx_receive_size);
    eeprom(EC_EEPROM_ADR_MBX_SEND_OFF,  slv->eeprom.mbx_send_offset);
    eeprom(EC_EEPROM_ADR_MBX_SEND_SIZE, slv->eeprom.mbx_send_size);

    size = value32 & 0x0000FFFF;

    while (cat_type != EC_EEPROM_CAT_END) {
        eeprom(cat_offset, value32);
        cat_type = (value32 & 0x0000FFFF);
        cat_len  = (value32 & 0xFFFF0000) >> 16;

        switch (cat_type) {
            default: 
            case EC_EEPROM_CAT_END:
            case EC_EEPROM_CAT_NOP:
                break;
            case EC_EEPROM_CAT_STRINGS: {
                ec_log(100, "EEPROM_STRINGS", "slave %d, cat_len %d\n", 
                        slave, cat_len);
                
                uint8_t *buf = malloc(cat_len*2);
                ec_eepromread_len(pec, slave, cat_offset+2, buf, cat_len*2);

                int local_offset = 0, i;
                slv->eeprom.strings_cnt = buf[local_offset++];

                ec_log(100, "EEPROM_STRINGS", "slave %d, stored strings %d\n", 
                        slave, slv->eeprom.strings_cnt);

                if (!slv->eeprom.strings_cnt) {
                    free(buf);
                    break;
                }

                slv->eeprom.strings = (char **)malloc(sizeof(char *) * slv->eeprom.strings_cnt);

                for (i = 0; i < slv->eeprom.strings_cnt; ++i) {
                    uint8_t string_len = buf[local_offset++];

                    slv->eeprom.strings[i] = malloc(sizeof(char) * (string_len + 1));
                    strncpy(slv->eeprom.strings[i], (char *)&buf[local_offset], string_len);
                    local_offset+=string_len;

                    slv->eeprom.strings[i][string_len] = '\0';
                    
                    ec_log(100, "EEPROM_STRINGS", "slave %d, string %d, length %d : %s\n", 
                            slave, i, string_len, slv->eeprom.strings[i]);
                    if (local_offset > cat_len*2) {
                        ec_log(5, "EEPROM_STRINGS", "slave %d, something wrong in eeprom string section\n",
                                slave);
                        break;
                    }
                }

                free(buf);
                break;
            }
            case EC_EEPROM_CAT_DATATYPES:
                ec_log(100, "EEPROM_DATATYPES", "slave %d:\n", slave);

                break;
            case EC_EEPROM_CAT_GENERAL: {
                ec_log(100, "EEPROM_GENERAL", "slave %d:\n", slave);

                eeprom(cat_offset+2, slv->eeprom.general);

                ec_log(100, "EEPROM_GENERAL", "slave %d: group_idx %d, img_idx %d, order_idx %d, name_idx %d\n", 
                        slave, slv->eeprom.general.group_idx,
                        slv->eeprom.general.img_idx,
                        slv->eeprom.general.order_idx,
                        slv->eeprom.general.name_idx);
                break;
            }
            case EC_EEPROM_CAT_FMMU: {
                ec_log(100, "EEPROM_FMMU", "slave %d:\n", slave);

                // skip cat type and len
                int local_offset = cat_offset + 2;
                unsigned i, fmmu_idx = 0;
                while (local_offset < (cat_offset + cat_len + 2)) {
                    eeprom(local_offset, value32);
                    uint8_t *tmp = (uint8_t *)&value32;
                    for (i = 0; i < 4 && i < (cat_len*2); ++i, ++fmmu_idx)
                        if ((fmmu_idx < slv->fmmu_ch) && (tmp[i] >= 1) && (tmp[i] <= 3))
                            slv->fmmu[fmmu_idx].type = tmp[i];

                    local_offset += 2;
                }
                break;
            }
            case EC_EEPROM_CAT_SM: {
                ec_log(100, "EEPROM_SM", "slave %d:\n", slave);

                // skip cat type and len
                int j = 0, local_offset = cat_offset + 2;
                slv->eeprom.sms_cnt = cat_len/(sizeof(ec_eeprom_cat_sm_t)/2);

                if (!slv->eeprom.sms_cnt)
                    break;

                // alloc sms
                slv->eeprom.sms = (ec_eeprom_cat_sm_t *)malloc(
                        sizeof(ec_eeprom_cat_sm_t) * slv->eeprom.sms_cnt);

                // reallocate if we have more sm that previously declared
                if ((cat_len/(sizeof(ec_eeprom_cat_sm_t)/2)) > slv->sm_ch) {
                    if (slv->sm)
                        free(slv->sm);

                    slv->sm_ch = cat_len/(sizeof(ec_eeprom_cat_sm_t)/2);
                    slv->sm = (ec_slave_sm_t *)malloc(slv->sm_ch * sizeof(ec_slave_sm_t));
                    memset(slv->sm, 0, slv->sm_ch * sizeof(ec_slave_sm_t));
                }

                while (local_offset < (cat_offset + cat_len + 2)) {
                    eeprom(local_offset, slv->eeprom.sms[j]);
                    local_offset += sizeof(ec_eeprom_cat_sm_t) / 2;

                    if (slv->sm[j].adr == 0) {
                        slv->sm[j].adr = slv->eeprom.sms[j].adr;
                        slv->sm[j].len = slv->eeprom.sms[j].len;
                        slv->sm[j].flags = (slv->eeprom.sms[j].activate << 16) | slv->eeprom.sms[j].ctrl_reg;

                        ec_log(100, "EEPROM_SM", "slave %d, sm%d adr 0x%X, len %d, flags 0x%X\n", 
                                slave, j, slv->sm[j].adr, slv->sm[j].len, slv->sm[j].flags);
                    } else {
                        ec_log(100, "EEPROM_SM", "slave %d, sm%d adr 0x%X, len %d, flags 0x%X\n", 
                                slave, j, slv->eeprom.sms[j].adr, slv->eeprom.sms[j].len,
                                (slv->eeprom.sms[j].activate << 16) | slv->eeprom.sms[j].ctrl_reg);
                                
                        ec_log(100, "EEPROM_SM", "slave %d, sm%d already set by user\n", slave, j);
                    }

                    j++;
                }
                break;
            }
            case EC_EEPROM_CAT_TXPDO: {
                ec_log(100, "EEPROM_TXPDO", "slave %d:\n", slave);

                // skip cat type and len
                int j = 0, local_offset = cat_offset + 2;
                slv->eeprom.txpdos_cnt = cat_len/(sizeof(ec_eeprom_cat_pdo_t)/2);

                if (!slv->eeprom.txpdos_cnt)
                    break;

                // alloc pdos
                slv->eeprom.txpdos = (ec_eeprom_cat_pdo_t *)malloc(
                        sizeof(ec_eeprom_cat_pdo_t) * slv->eeprom.txpdos_cnt);

                while (local_offset < (cat_offset + cat_len + 2)) {
                    eeprom(local_offset, slv->eeprom.txpdos[j++]);
                    local_offset += sizeof(ec_eeprom_cat_pdo_t) / 2;

                    if (j >= slv->eeprom.txpdos_cnt)
                        break;
                }

                break;
            }
            case EC_EEPROM_CAT_RXPDO: {
                ec_log(100, "EEPROM_RXPDO", "slave %d:\n", slave);

                // skip cat type and len
                int j = 0, local_offset = cat_offset + 2;
                slv->eeprom.rxpdos_cnt = cat_len/(sizeof(ec_eeprom_cat_pdo_t)/2);

                if (!slv->eeprom.rxpdos_cnt)
                    break;

                // alloc pdos
                slv->eeprom.rxpdos = (ec_eeprom_cat_pdo_t *)malloc(
                        sizeof(ec_eeprom_cat_pdo_t) * slv->eeprom.rxpdos_cnt);

                while (local_offset < (cat_offset + cat_len + 2)) {
                    eeprom(local_offset, slv->eeprom.rxpdos[j++]);
                    local_offset += sizeof(ec_eeprom_cat_pdo_t) / 2;

                    if (j >= slv->eeprom.rxpdos_cnt)
                        break;
                }

                break;
            }
            case EC_EEPROM_CAT_DC:
                ec_log(100, "EEPROM_DC", "slave %d:\n", slave);
                break;
        }

        cat_offset += cat_len + 2; 
    }
}

