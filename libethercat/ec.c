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

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>

#include "ec.h"
#include "mbx.h"
#include "coe.h"

void ec_log(const char *pre, const char *format, ...) {
    va_list ap;
    va_start(ap, format);
    va_end(ap);
    printf("[%-20.20s] ", pre);
    vprintf(format, ap);
}

/** Set eeprom control to master. Only if set to PDI.
 * @param[in]  context        = context struct
 * @param[in] slave     = Slave number
 * @return >0 if OK
 */
int ec_eepromconfig(ec_t *pec, uint16_t slave) {
    uint16_t wkc, eepctl = 2;
    ec_fpwr(pec, pec->slaves[slave].fixed_address, EC_REG_EEPCFG, 
            (uint8_t *)&eepctl, sizeof(eepctl), &wkc);
    if (wkc != 1)
        ec_log(__func__, "slave %d did not accept forcing eeprom to pdi\n", slave);
    
    eepctl = 0;
    ec_fpwr(pec, pec->slaves[slave].fixed_address, EC_REG_EEPCFG, 
            (uint8_t *)&eepctl, sizeof(eepctl), &wkc);
    if (wkc != 1)
        ec_log(__func__, "slave %d did not accept setting eeprom to ethercat\n", slave);
    
    ec_fprd(pec, pec->slaves[slave].fixed_address, EC_REG_EEPCFG, 
            (uint8_t *)&eepctl, sizeof(eepctl), &wkc);

//    ec_log(__func__, "slave %d config register 0x%04X\n", slave, eepctl);

    return 0;
}

int ec_eepromread(ec_t *pec, uint16_t slave, uint32_t eepadr, uint32_t *data) {
    ec_eepromconfig(pec, slave);
    
    int ret = 0;
    uint16_t wkc, eepcsr = 0x0100; // read access

    ec_fpwr(pec, pec->slaves[slave].fixed_address, EC_REG_EEPADR,
            (uint8_t *)&eepadr, sizeof(eepadr), &wkc);
    if (wkc != 1) {
        printf("writing eepadr failed\n");
        ret = -1;
        goto func_exit;
    }

    eepcsr = 0x0100;
    ec_fpwr(pec, pec->slaves[slave].fixed_address, EC_REG_EEPCTL,
            (uint8_t *)&eepcsr, sizeof(eepcsr), &wkc);
    if (wkc != 1) {
        printf("wirting eepctl failed\n");
        ret = -1;
        goto func_exit;
    }

    do {
        ec_fprd(pec, pec->slaves[slave].fixed_address, EC_REG_EEPCTL,
                (uint8_t *)&eepcsr, sizeof(eepcsr), &wkc);
        if (wkc != 1) {
            printf("reading eepctl failed\n");
            ret = -1;
            goto func_exit;
        }
    } while (eepcsr & 0x0100);


    ec_fprd(pec, pec->slaves[slave].fixed_address, EC_REG_EEPDAT,
            (uint8_t *)data, sizeof(*data), &wkc);
    if (wkc != 1) {
        printf("reading data failed\n");
        ret = -1;
        goto func_exit;
    }

func_exit:

    return ret;
}

int ec_eepromread_2(ec_t *pec, uint16_t slave) {
    ec_eepromconfig(pec, slave);
    
    uint32_t eepadr = 0, data;
    uint16_t wkc, eepcsr = 0x0100; // read access

    for (eepadr = 0; eepadr < 32; ++eepadr) {
        ec_fpwr(pec, pec->slaves[slave].fixed_address, EC_REG_EEPADR,
                (uint8_t *)&eepadr, sizeof(eepadr), &wkc);
        if (wkc != 1)
            printf("writing eepadr failed\n");

        eepcsr = 0x0100;
        ec_fpwr(pec, pec->slaves[slave].fixed_address, EC_REG_EEPCTL,
                (uint8_t *)&eepcsr, sizeof(eepcsr), &wkc);
        if (wkc != 1)
            printf("wirting eepctl failed\n");

        do {
            ec_fprd(pec, pec->slaves[slave].fixed_address, EC_REG_EEPCTL,
                    (uint8_t *)&eepcsr, sizeof(eepcsr), &wkc);
            if (wkc != 1)
                printf("reading eepctl failed\n");
        } while (eepcsr & 0x0010);


        ec_fprd(pec, pec->slaves[slave].fixed_address, EC_REG_EEPDAT,
                (uint8_t *)&data, sizeof(data), &wkc);
        if (wkc != 1)
            printf("reading data failed\n");

        printf("%08X ", data);
        if (((eepadr+1)%8) == 0)
            printf("\n");
    }
    printf("\n");

    return 0;
}

typedef uint16_t ec_state_t;
static const ec_state_t EC_STATE_INIT       = 0x01;
static const ec_state_t EC_STATE_PREOP      = 0x02;
static const ec_state_t EC_STATE_SAFEOP     = 0x04;
static const ec_state_t EC_STATE_OP         = 0x08;
static const ec_state_t EC_STATE_MASK       = 0x0F;
static const ec_state_t EC_STATE_ERROR      = 0x10;
static const ec_state_t EC_STATE_RESET      = 0x10;

int ec_state_get(ec_t *pec, uint16_t slave, ec_state_t *state);

int ec_master_state_set(ec_t *pec, ec_state_t state) {
    uint16_t wkc = 0;
    uint16_t value = (uint16_t)state;
    ec_bwr(pec, EC_REG_ALCTL, &value, sizeof(value), &wkc); 
    return wkc;
}

int ec_state_set(ec_t *pec, uint16_t slave, ec_state_t state) {
    uint16_t wkc = 0, act_state, value;
    ec_fpwr(pec, pec->slaves[slave].fixed_address, 
            EC_REG_ALCTL, &state, sizeof(state), &wkc); 
    
    if (state & EC_STATE_RESET)
        return wkc; // just return here, we did an error reset

    do {
        act_state = 0;
        wkc = ec_state_get(pec, slave, &act_state);
        ec_log("EC_STATE_SET", "wkc %d, state %X, act_state %X\n", 
                wkc, state, act_state);
        
        if (act_state & EC_STATE_ERROR) {
            ec_fprd(pec, pec->slaves[slave].fixed_address, 
                    EC_REG_ALSTATCODE, &value, sizeof(value), &wkc);
            ec_log("EC_STATE_SET", "state switch to %d failed, alstatcode 0x%04X\n", 
                    state, value);

            ec_state_set(pec, slave, (act_state & EC_STATE_MASK) | EC_STATE_RESET);
            break;
        }

        struct timespec ts = { 0, 1000000 };
        nanosleep(&ts, NULL);
    } while (act_state != state);

    return wkc;
}

int ec_state_get(ec_t *pec, uint16_t slave, ec_state_t *state) {
    uint16_t wkc = 0, value = 0;
    ec_fprd(pec, pec->slaves[slave].fixed_address, 
            EC_REG_ALSTAT, &value, sizeof(value), &wkc);
    
    if (wkc)
        *state = (ec_state_t)value;
    
    if (*state & 0x10) {
        ec_fprd(pec, pec->slaves[slave].fixed_address, 
                EC_REG_ALSTATCODE, &value, sizeof(value), &wkc);
        printf("alstatcode 0x%04X\n", value);
    }

    return wkc;
}

typedef enum ec_state_transition {
    INIT_2_INIT      = 0x0101,
    INIT_2_PREOP     = 0x0102,
    INIT_2_SAFEOP    = 0x0104,
    INIT_2_OP        = 0x0108,
    PREOP_2_INIT     = 0x0201,
    PREOP_2_PREOP    = 0x0202,
    PREOP_2_SAFEOP   = 0x0204,
    PREOP_2_OP       = 0x0208,
    SAFEOP_2_INIT    = 0x0401,
    SAFEOP_2_PREOP   = 0x0402,
    SAFEOP_2_SAFEOP  = 0x0404,
    SAFEOP_2_OP      = 0x0408,
    OP_2_INIT        = 0x0801,
    OP_2_PREOP       = 0x0802,
    OP_2_SAFEOP      = 0x0804,
    OP_2_OP          = 0x0808,
} ec_state_transition_t;

enum {
    EC_EEPROM_SIZE  = 0x3E,
};

enum {
    EC_EEPROM_CAT_NOP       = 0,
    EC_EEPROM_CAT_STRINGS   = 10,
    EC_EEPROM_CAT_DATATYPES = 20,
    EC_EEPROM_CAT_GENERAL   = 30,
    EC_EEPROM_CAT_FMMU      = 40,
    EC_EEPROM_CAT_SM        = 41,
    EC_EEPROM_CAT_TXPDO     = 50,
    EC_EEPROM_CAT_RXPDO     = 51,
    EC_EEPROM_CAT_DC        = 60,
    EC_EEPROM_CAT_END       = 0xFFFF
};

typedef struct PACKED ec_eeprom_cat_sm {
    uint16_t adr;
    uint16_t len;
    uint8_t  ctrl_reg;
    uint8_t  status_reg;
    uint8_t  activate;
    uint8_t  pdi_ctrl;
} PACKED ec_eeprom_cat_sm_t;

typedef struct PACKED ec_eeprom_cat_fmmu {
    uint8_t type;
} PACKED ec_eeprom_cat_fmmu_t;

void eeprom_dump(ec_t *pec, uint16_t slave) {
    uint16_t size;
    uint32_t value32;
    ec_eepromread(pec, slave, EC_EEPROM_SIZE, &value32);
    size = value32 & 0x0000FFFF;

    int cat_offset = 0x40;
    uint16_t cat_type = 0, cat_len;
    while (cat_type != EC_EEPROM_CAT_END) {
        ec_eepromread(pec, slave, cat_offset, &value32);
        cat_type = value32 & 0x0000FFFF;
        cat_len  = (value32 & 0xFFFF0000) >> 16;
        
        switch (cat_type) {
            default: 
            case EC_EEPROM_CAT_END:
                break;
            case EC_EEPROM_CAT_NOP:
            case EC_EEPROM_CAT_STRINGS:
            case EC_EEPROM_CAT_DATATYPES:
            case EC_EEPROM_CAT_GENERAL: {
                break;
            }
            case EC_EEPROM_CAT_FMMU: {
                // skip cat type and len
                int i, local_offset = cat_offset + 2;

                while (local_offset < (cat_offset + cat_len + 2)) {
                    ec_eepromread(pec, slave, local_offset, &value32);
                    uint8_t *tmp = (uint8_t *)&value32;
                    for (i = 0; i < 4 && i < (cat_len*2); ++i)
                        ec_log("EEPROM_FMMU", "fmmu%d type: %d\n", i, tmp[i]);

                    local_offset += 2;
                }
                break;
            }
            case EC_EEPROM_CAT_SM: {
                // skip cat type and len
                int j = 0, i, local_offset = cat_offset + 2;

                while (local_offset < (cat_offset + cat_len + 2)) {
                    ec_eeprom_cat_sm_t cat_sm;
                    uint8_t *tmp = (uint8_t *)&cat_sm;
                    for (i = 0; i < sizeof(cat_sm)/2; i+=2, local_offset+=2)
                        ec_eepromread(pec, slave, local_offset, (uint32_t *)&(tmp[i*2]));

                    ec_log("EEPROM_SM", "sm%d: adr 0x%04X, len %d, ctrl %02X, activate %d\n", 
                            j, cat_sm.adr, cat_sm.len, cat_sm.ctrl_reg, cat_sm.activate);

                    pec->slaves[slave].sm[j].adr = cat_sm.adr;
                    pec->slaves[slave].sm[j].len = cat_sm.len;
                    pec->slaves[slave].sm[j].flags = (cat_sm.activate << 16) | cat_sm.ctrl_reg;
                    j++;
                }
                break;
            }
            case EC_EEPROM_CAT_TXPDO:
            case EC_EEPROM_CAT_RXPDO:
            case EC_EEPROM_CAT_DC:
                break;
        }

        cat_offset += cat_len + 2; 
    }
}

int ec_coe_calc_pd_len(ec_t *pec, uint16_t slave, uint16_t pdo_reg) {
//    ec_coe_sdo_read(pec, slave, pdo_reg, 0, buf, 
}

int ec_state_transition(ec_t *pec, uint16_t slave, ec_state_t state) {
    uint16_t wkc;
    ec_state_t act_state = 0;
    ec_slave_t *slv = &pec->slaves[slave];
    
    // check error state
    wkc = ec_state_get(pec, slave, &act_state);
    if (act_state & EC_STATE_ERROR) // reset error state first
        ec_state_set(pec, slave, (act_state & EC_STATE_MASK) | EC_STATE_RESET);
            
    // generate transition
    ec_state_transition_t transition = ((act_state & EC_STATE_MASK) << 8) | (state & EC_STATE_MASK); 

    switch (transition) {
        case INIT_2_PREOP:
        case INIT_2_SAFEOP:
        case INIT_2_OP: {
            // init to preop stuff
            uint32_t value32;
            ec_eepromread(pec, slave, 0x0008, &value32);
            pec->slaves[slave].vendor_id = value32;

            ec_eepromread(pec, slave, 0x000a, &value32);
            pec->slaves[slave].product_code = value32;

            ec_log("INIT_2_PREOP", "slave %d, vendor 0x%08X, product 0x%08X\n",
                slave, pec->slaves[slave].vendor_id, pec->slaves[slave].product_code);

            ec_eepromread(pec, slave, 0x001a, &value32);
            pec->slaves[slave].sm[1].adr = value32 & 0x0000FFFF;
            pec->slaves[slave].sm[1].len = (value32 & 0xFFFF0000) >> 16;
            pec->slaves[slave].sm[1].flags = 0x00010022;
            pec->slaves[slave].mbx_read.sm_nr = 1;
            pec->slaves[slave].mbx_read.buf = malloc(pec->slaves[slave].sm[1].len);
            memset(pec->slaves[slave].mbx_read.buf, 0, pec->slaves[slave].sm[1].len);

            ec_eepromread(pec, slave, 0x0018, &value32);
            pec->slaves[slave].sm[0].adr = value32 & 0x0000FFFF;
            pec->slaves[slave].sm[0].len = (value32 & 0xFFFF0000) >> 16;
            pec->slaves[slave].sm[0].flags = 0x00010026;
            pec->slaves[slave].mbx_write.sm_nr = 0;
            pec->slaves[slave].mbx_write.buf = malloc(pec->slaves[slave].sm[0].len);
            memset(pec->slaves[slave].mbx_write.buf, 0, pec->slaves[slave].sm[0].len);

            int i;
            for (i = 0; i < 2; ++i) {
                ec_log("INIT_2_PREOP", "slave %d: sm%d, adr 0x%X, len %d, flags 0x%X\n",
                        slave, i, pec->slaves[slave].sm[i].adr, 
                        pec->slaves[slave].sm[i].len, pec->slaves[slave].sm[i].flags);

                ec_fpwr(pec, pec->slaves[slave].fixed_address, 0x800 + (i * 8),
                        &pec->slaves[slave].sm[i], sizeof(ec_slave_sm_t), &wkc);
            }
            
            eeprom_dump(pec, slave);
    
            // write state to slave
            wkc = ec_state_set(pec, slave, EC_STATE_PREOP);
        

            if (transition == INIT_2_PREOP)
                break;
        }
        case PREOP_2_SAFEOP:
        case PREOP_2_OP: {
            // preop to safeop stuff            
//            slv->sm[2].len = 10;
//            slv->sm[3].len = 10;
            
            int i;
            for (i = 2; i < 4; ++i) {
                ec_log("PREOP_2_SAFEOP", "slave %d: sm%d, adr 0x%X, len %d, flags 0x%X\n",
                        slave, i, pec->slaves[slave].sm[i].adr, 
                        pec->slaves[slave].sm[i].len, pec->slaves[slave].sm[i].flags);

                ec_fpwr(pec, pec->slaves[slave].fixed_address, 0x800 + (i * 8),
                        &pec->slaves[slave].sm[i], sizeof(ec_slave_sm_t), &wkc);
            }

            // write state to slave
            wkc = ec_state_set(pec, slave, EC_STATE_SAFEOP);

            if (transition == INIT_2_SAFEOP || transition == PREOP_2_SAFEOP)
                break;
        }
        case SAFEOP_2_OP:
            // safeop to op stuff 
            
            // write state to slave
            wkc = ec_state_set(pec, slave, EC_STATE_OP);
            
            break;

        case OP_2_INIT:
        case SAFEOP_2_INIT:
        case PREOP_2_INIT:
        case INIT_2_INIT: {
            int i;
            uint16_t wkc = 0, features = 0;
            uint8_t sm_fmmu_ch[2], ram_size = 0;


            // get number of sync managers and fmmus
            ec_fprd(pec, pec->slaves[slave].fixed_address, EC_REG_SM_FFMU_CH,
                    sm_fmmu_ch, 2, &wkc);            
            pec->slaves[slave].sm_ch = sm_fmmu_ch[1];
            pec->slaves[slave].fmmu_ch = sm_fmmu_ch[0];
    
            // get ram size
            ec_fprd(pec, pec->slaves[slave].fixed_address, EC_REG_RAM_SIZE,
                    &ram_size, sizeof(ram_size), &wkc);            
            pec->slaves[slave].ram_size = ram_size << 10;
            
            // get features
            ec_fprd(pec, pec->slaves[slave].fixed_address, EC_REG_ESCSUP,
                    &features, sizeof(features), &wkc);            
            pec->slaves[slave].features = features;

            // clear sync managers
            if (pec->slaves[slave].sm) {
                free(pec->slaves[slave].sm);
                pec->slaves[slave].sm = NULL;
            }

            if (pec->slaves[slave].sm_ch) {
                pec->slaves[slave].sm = (ec_slave_sm_t *)malloc(
                        pec->slaves[slave].sm_ch * sizeof(ec_slave_sm_t));
                memset(pec->slaves[slave].sm, 0, pec->slaves[slave].sm_ch * sizeof(ec_slave_sm_t));

                for (i = 0; i < pec->slaves[slave].sm_ch; ++i) 
                    ec_transmit_no_reply(pec, EC_CMD_FPWR, 
                            ec_to_adr(pec->slaves[slave].fixed_address, 0x800 + (8 * i)),
                            (uint8_t *)&pec->slaves[slave].sm[i], sizeof(ec_slave_sm_t));
            }
                        
            // clear fmmus
            if (pec->slaves[slave].fmmu) {
                free(pec->slaves[slave].fmmu);
                pec->slaves[slave].fmmu = NULL;
            }

            if (pec->slaves[slave].fmmu_ch) {
                pec->slaves[slave].fmmu = (ec_slave_fmmu_t *)malloc(
                        pec->slaves[slave].fmmu_ch * sizeof(ec_slave_fmmu_t));
                memset(pec->slaves[slave].fmmu, 0, pec->slaves[slave].fmmu_ch * sizeof(ec_slave_fmmu_t));

                for (i = 0; i < pec->slaves[slave].fmmu_ch; ++i) 
                    ec_transmit_no_reply(pec, EC_CMD_FPWR, 
                            ec_to_adr(pec->slaves[slave].fixed_address, 0x600 + (16 * i)),
                            (uint8_t *)&pec->slaves[slave].fmmu[i], sizeof(ec_slave_fmmu_t));
            }
        }
        case PREOP_2_PREOP:
        case SAFEOP_2_SAFEOP:
        case OP_2_OP:
            // write state to slave
            wkc = ec_state_set(pec, slave, state);
            break;
        default:
            break;
    };

    return wkc;
}

pthread_t ec_tx;
void *ec_tx_thread(void *arg) {
    ec_t *pec = (ec_t *)arg;

    while (1) {
        hw_tx(pec->phw);
        struct timespec ts = { 0, 1000000 };
        nanosleep(&ts, NULL);
    }

    return 0;
}
    
//! open ethercat master
/*!
 * \param ppec return value for ethercat master pointer
 * \param ifname ethercat master interface name
 * \param prio receive thread priority
 * \param cpumask receive thread cpumask
 * \return 0 on succes, otherwise error code
 */
int ec_open(ec_t **ppec, const char *ifname, int prio, int cpumask) {
    int i, ret;
    uint16_t val, wkc, fixed = 1000;
    
    (*ppec) = (ec_t *)malloc(sizeof(ec_t));
    if (!(*ppec))
        return ENOMEM;

    // fill index queue
    TAILQ_INIT(&(*ppec)->idx);
    for (i = 0; i < 255; ++i) {
        idx_entry_t *entry = (idx_entry_t *)malloc(sizeof(idx_entry_t));
        entry->idx = i;
        sem_init(&entry->waiter, 0, 0);
        ec_index_put(*ppec, entry);
    }
    
    datagram_pool_open(&(*ppec)->pool, 1000);
    hw_open(&(*ppec)->phw, ifname, prio, cpumask);

    pthread_create(&ec_tx, NULL, ec_tx_thread, *ppec);

    // allocating slave structures
    ret = ec_brd((*ppec), EC_REG_TYPE, (uint8_t *)&val, sizeof(val), &wkc); 
    (*ppec)->slave_cnt = wkc;
    (*ppec)->slaves = (ec_slave_t *)malloc((*ppec)->slave_cnt * sizeof(ec_slave_t));
    memset((*ppec)->slaves, 0, (*ppec)->slave_cnt * sizeof(ec_slave_t));

    for (i = 0; i < 65536; ++i) {
        int auto_inc = -1 * i;

        ret = ec_aprd((*ppec), auto_inc, EC_REG_TYPE, (uint8_t *)&val, sizeof(val), &wkc);

        if (wkc == 0)
            break;  // break here, cause there seems to be no more slave
        
        printf("found slave with auto inc address %d, wkc %d\n", auto_inc, wkc);
        (*ppec)->slaves[i].auto_inc_address = auto_inc;
        (*ppec)->slaves[i].fixed_address = fixed;

        ec_apwr((*ppec), auto_inc, EC_REG_STADR, (uint8_t *)&fixed, sizeof(fixed), &wkc); 
        if (wkc == 1)
            printf("fixed address %d successfully written to slave %d\n", fixed, auto_inc);

        fixed++;

        ec_state_transition(*ppec, i, EC_STATE_INIT);
        
        ec_state_transition(*ppec, i, EC_STATE_PREOP);

#if 0
        ec_coe_odlist_read(*ppec, i, NULL, NULL);

        int k, l, j;
//        for (k = 0; k < 4; ++k)
//            ec_coe_sdo_desc_read(*ppec, i, 0x1000+k, NULL, NULL);
//        for (k = 0; k < 8; ++k)
//            ec_coe_sdo_desc_read(*ppec, i, 0x1A00+k, NULL, NULL);

        for (k = 0; k < 32; ++k) {            
            uint8_t idx_cnt = 0;
            size_t idx_cnt_len = sizeof(idx_cnt);
            ec_coe_sdo_read(*ppec, i, 0x1600 + k, 0, 0, &idx_cnt, &idx_cnt_len);

            if (!idx_cnt)
                continue;

            printf("0x%04X: elements: %d\n", 0x1600+k, idx_cnt);

            for (l = 1; l <= idx_cnt; ++l) {
                uint8_t buf[10];
                size_t buf_len = sizeof(uint8_t) * 10;
                ec_coe_sdo_read(*ppec, i, 0x1600 + k, l, 0, buf, &buf_len);

                printf("      : %2d - ", l);
                for (j = 0; j < buf_len; ++j) {
                    printf("%02X ", buf[j]);
                }
                printf("        0x%04X ", ((uint16_t *)buf)[1]);
                ec_coe_sdo_desc_read(*ppec, i, ((uint16_t *)buf)[1], 0,0);
            }
        }

        for (k = 0; k < 32; ++k) {            
            uint8_t idx_cnt = 0;
            size_t idx_cnt_len = sizeof(idx_cnt);
            ec_coe_sdo_read(*ppec, i, 0x1A00 + k, 0, 0, &idx_cnt, &idx_cnt_len);

            if (!idx_cnt)
                continue;

            printf("0x%04X: elements: %d\n", 0x1A00+k, idx_cnt);

            for (l = 1; l <= idx_cnt; ++l) {
                uint8_t buf[10];
                size_t buf_len = sizeof(uint8_t) * 10;
                ec_coe_sdo_read(*ppec, i, 0x1A00 + k, l, 0, buf, &buf_len);

                printf("      : %2d - ", l);
                for (j = 0; j < buf_len; ++j) {
                    printf("%02X ", buf[j]);
                }
                printf("        0x%04X ", ((uint16_t *)buf)[1]);
                ec_coe_sdo_desc_read(*ppec, i, ((uint16_t *)buf)[1], 0,0);
            }
        }
        
        for (k = 0; k < 4; ++k) {            
            uint8_t idx_cnt = 0;
            size_t idx_cnt_len = sizeof(idx_cnt);
            ec_coe_sdo_read(*ppec, i, 0x1C10 + k, 0, 0, &idx_cnt, &idx_cnt_len);

            if (!idx_cnt)
                continue;

            printf("0x%04X: elements: %d\n", 0x1C10+k, idx_cnt);

            for (l = 1; l <= idx_cnt; ++l) {
                uint8_t buf[10];
                size_t buf_len = sizeof(uint8_t) * 10;
                ec_coe_sdo_read(*ppec, i, 0x1C10 + k, l, 0, buf, &buf_len);

                printf("      : %2d - ", l);
                for (j = 0; j < buf_len; ++j) {
                    printf("%02X ", buf[j]);
                }
                printf("\n");
//                printf("        0x%04X ", ((uint16_t *)buf)[1]);
//                ec_coe_sdo_desc_read(*ppec, i, ((uint16_t *)buf)[1], 0,0);
            }
        }
#endif

        ec_state_transition(*ppec, i, EC_STATE_SAFEOP);
        ec_state_transition(*ppec, i, EC_STATE_OP);

    }

    return 0;
}

//! closes ethercat master
/*!
 * \param pec pointer to ethercat master
 * \return 0 on success 
 */
int ec_close(ec_t *pec) {
    hw_close(pec->phw);
    datagram_pool_close(pec->pool);

    idx_entry_t *idx;
    while ((idx = TAILQ_FIRST(&pec->idx)) != NULL) {
        TAILQ_REMOVE(&pec->idx, idx, qh);
        free(idx);
    }

    if (pec->slaves)
        free(pec->slaves);

    free(pec);

    return 0;
}

//! get next free index entry
/*!
 * \param pec pointer to ethercat master
 * \param entry return entry of next free index 
 * \return 0 on succes, otherwise error code
 */
int ec_index_get(ec_t *pec, struct idx_entry **entry) {
    *entry = (idx_entry_t *)TAILQ_FIRST(&pec->idx);
    if (*entry) {
        TAILQ_REMOVE(&pec->idx, *entry, qh);
        return 0;
    }

    return -1;
}

//! returns index entry
/*!
 * \param pec pointer to ethercat master
 * \param entry return index entry 
 * \return 0 on succes, otherwise error code
 */
int ec_index_put(ec_t *pec, struct idx_entry *entry) {
    if (!pec || !entry)
        return -1;

    TAILQ_INSERT_TAIL(&pec->idx, entry, qh);
    return 0;
}

//! local callack for syncronous read/write
static void cb_block(void *user_arg, struct datagram_entry *p) {
    idx_entry_t *entry = (idx_entry_t *)user_arg;
    sem_post(&entry->waiter);
}

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
        uint8_t *data, size_t datalen, uint16_t *wkc) {
    datagram_entry_t *p_de;
    idx_entry_t *p_idx;

    if (ec_index_get(pec, &p_idx) != 0) 
        return -1;

    if (datagram_pool_get(pec->pool, &p_de, NULL) != 0) {
        ec_index_put(pec, p_idx);
        return -1;
    }

    memset(&p_de->datagram, 0, sizeof(ec_datagram_t) + datalen + 2);
    p_de->datagram.cmd = cmd;
    p_de->datagram.idx = p_idx->idx;
    p_de->datagram.adr = adr;
    p_de->datagram.len = datalen;
    p_de->datagram.irq = 0;
    memcpy(ec_datagram_payload(&p_de->datagram), data, datalen);

    p_de->user_cb = cb_block;
    p_de->user_arg = p_idx;

    // queue frame and trigger tx
    datagram_pool_put(pec->phw->tx_low, p_de);

    // wait for completion
    sem_wait(&p_idx->waiter);

    *wkc = ec_datagram_wkc(&p_de->datagram);
    if (*wkc)
        memcpy(data, ec_datagram_payload(&p_de->datagram), datalen);

    datagram_pool_put(pec->pool, p_de);
    ec_index_put(pec, p_idx);

    return 0;
}

//! local callack for syncronous read/write
static void cb_no_reply(void *user_arg, struct datagram_entry *p) {
    idx_entry_t *entry = (idx_entry_t *)user_arg;
    datagram_pool_put(entry->pec->pool, p);
    ec_index_put(entry->pec, entry);
}

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
        uint8_t *data, size_t datalen) {
    datagram_entry_t *p_de;
    idx_entry_t *p_idx;

    if (ec_index_get(pec, &p_idx) != 0) 
        return -1;

    p_idx->pec = pec;

    if (datagram_pool_get(pec->pool, &p_de, NULL) != 0) {
        ec_index_put(pec, p_idx);
        return -1;
    }

    memset(&p_de->datagram, 0, sizeof(ec_datagram_t) + datalen + 2);
    p_de->datagram.cmd = cmd;
    p_de->datagram.idx = p_idx->idx;
    p_de->datagram.adr = adr;
    p_de->datagram.len = datalen;
    p_de->datagram.irq = 0;
    memcpy(ec_datagram_payload(&p_de->datagram), data, datalen);

    p_de->user_cb = cb_no_reply;
    p_de->user_arg = p_idx;

    // queue frame and return, we don't care about an answer
    datagram_pool_put(pec->phw->tx_low, p_de);

    return 0;
}

