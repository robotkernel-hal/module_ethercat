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

void *ec_log_func_user = NULL;
void (*ec_log_func)(void *user, const char *format, ...) = NULL;

void ec_log(const char *pre, const char *format, ...) {
    if (ec_log_func == NULL) {
        va_list ap;
        va_start(ap, format);
        printf("[%-20.20s] ", pre);
        vprintf(format, ap);
        va_end(ap);
    } else {
        char buf[512];
        char *tmp = &buf[0];

        // format argument list
        va_list args;
        va_start(args, format);
        int ret = snprintf(tmp, 512, "%s: ", pre);
        vsnprintf(tmp+ret, 512-ret, format, args);

        ec_log_func(ec_log_func_user, buf);
    }
}


int ec_state_get(ec_t *pec, uint16_t slave, ec_state_t *state);

int ec_master_state_set(ec_t *pec, ec_state_t state) {
    uint16_t wkc = 0;
    uint16_t value = (uint16_t)state;
    ec_bwr(pec, EC_REG_ALCTL, &value, sizeof(value), &wkc); 
    return wkc;
}

int ec_slave_set_state(ec_t *pec, uint16_t slave, ec_state_t state) {
    uint16_t wkc = 0, act_state, value;
    ec_fpwr(pec, pec->slaves[slave].fixed_address, 
            EC_REG_ALCTL, &state, sizeof(state), &wkc); 
    
    if (state & EC_STATE_RESET)
        return wkc; // just return here, we did an error reset

    do {
        act_state = 0;
        wkc = ec_state_get(pec, slave, &act_state);
        ec_log("EC_STATE_SET", "slave %d, state %X, act_state %X, wkc %d\n", 
                slave, state, act_state, wkc);
        
        if (act_state & EC_STATE_ERROR) {
            ec_fprd(pec, pec->slaves[slave].fixed_address, 
                    EC_REG_ALSTATCODE, &value, sizeof(value), &wkc);
            ec_log("EC_STATE_SET", "slave %d, state switch to %d failed, alstatcode 0x%04X\n", 
                    slave, state, value);

            ec_slave_set_state(pec, slave, (act_state & EC_STATE_MASK) | EC_STATE_RESET);
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
    
    if (*state & 0x10)
        ec_fprd(pec, pec->slaves[slave].fixed_address, 
                EC_REG_ALSTATCODE, &value, sizeof(value), &wkc);

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


int ec_coe_calc_pd_len(ec_t *pec, uint16_t slave, uint16_t pdo_reg) {
//    ec_coe_sdo_read(pec, slave, pdo_reg, 0, buf, 
    return 0;
}

int ec_slave_state_transition(ec_t *pec, uint16_t slave, ec_state_t state) {
    uint16_t wkc;
    ec_state_t act_state = 0;
    ec_slave_t *slv = &pec->slaves[slave];
    
    // check error state
    wkc = ec_state_get(pec, slave, &act_state);
    if (act_state & EC_STATE_ERROR) // reset error state first
        ec_slave_set_state(pec, slave, (act_state & EC_STATE_MASK) | EC_STATE_RESET);
            
    // generate transition
    ec_state_transition_t transition = ((act_state & EC_STATE_MASK) << 8) | (state & EC_STATE_MASK); 

    switch (transition) {
        case INIT_2_PREOP:
        case INIT_2_SAFEOP:
        case INIT_2_OP: {
            // init to preop stuff
            ec_eeprom_dump(pec, slave);
            
            ec_log("INIT_2_PREOP", "slave %d, vendor 0x%08X, product 0x%08X, mbx 0x%04X\n",
                slave, slv->eeprom.vendor_id, slv->eeprom.product_code, slv->eeprom.mbx_supported);

            // configure mailboxes if any supported
            if (slv->eeprom.mbx_supported) {
                // read mailbox
                slv->sm[1].adr = slv->eeprom.mbx_send_offset;
                slv->sm[1].len = slv->eeprom.mbx_send_size;
                slv->sm[1].flags = 0x00010022;
                slv->mbx_read.sm_nr = 1;
                if (slv->mbx_read.buf) free(slv->mbx_read.buf);
                slv->mbx_read.buf = malloc(slv->sm[1].len);
                memset(slv->mbx_read.buf, 0, slv->sm[1].len);

                // write mailbox
                slv->sm[0].adr = slv->eeprom.mbx_receive_offset;
                slv->sm[0].len = slv->eeprom.mbx_receive_size;
                slv->sm[0].flags = 0x00010026;
                slv->mbx_write.sm_nr = 0;
                if (slv->mbx_write.buf) free(slv->mbx_write.buf);
                slv->mbx_write.buf = malloc(slv->sm[0].len);
                memset(slv->mbx_write.buf, 0, slv->sm[0].len);

                for (int sm_idx = 0; sm_idx < 2; ++sm_idx) {
                    ec_log("INIT_2_PREOP", "slave %d: sm%d, adr 0x%X, len %d, flags 0x%X\n",
                            slave, sm_idx, slv->sm[sm_idx].adr, 
                            slv->sm[sm_idx].len, slv->sm[sm_idx].flags);

                    ec_fpwr(pec, slv->fixed_address, 0x800 + (sm_idx * 8),
                            &slv->sm[sm_idx], sizeof(ec_slave_sm_t), &wkc);
                }
            }
            
            // write state to slave
            wkc = ec_slave_set_state(pec, slave, EC_STATE_PREOP);

            // check sm settings
            if (slv->eeprom.mbx_supported & EC_EEPROM_MBX_COE) { // have coe mailbox, check objects 1c12, 1c13
                for (int sm_idx = 2; sm_idx <= 3; ++sm_idx) {
                    int wkc, bit_len = 0, idx = 0x1c10 + sm_idx;
                    uint8_t entry_cnt = 0, entry_cnt_2;
                    size_t entry_cnt_size = sizeof(entry_cnt);
                    wkc = ec_coe_sdo_read(pec, slave, idx, 0, 0, &entry_cnt, &entry_cnt_size);

                    if (wkc != 1)
                        ec_log("INIT_2_PREOP", "slave %d: reading 0x%04X/%d failed\n", slave, idx, 0);

                    ec_log("INIT_2_PREOP", "slave %d: 0x%04X count %d\n", slave, idx, entry_cnt); 

                    for (int i = 1; i <= entry_cnt; ++i) {
                        uint16_t entry_idx;
                        size_t entry_size = sizeof(entry_idx);
                        wkc = ec_coe_sdo_read(pec, slave, idx, i, 0, (uint8_t *)&entry_idx, &entry_size);

                        if (wkc != 1)
                            ec_log("INIT_2_PREOP", "slave %d: reading 0x%04X/%d failed\n", slave, idx, i);

                        entry_cnt_size = sizeof(entry_cnt_2);

                        wkc = ec_coe_sdo_read(pec, slave, entry_idx, 0, 0, (uint8_t *)&entry_cnt_2, &entry_cnt_size);

                        if (wkc != 1)
                            ec_log("INIT_2_PREOP", "slave %d: reading 0x%04X/%d failed\n", slave, entry_idx, 0);

                        ec_log("INIT_2_PREOP", "slave %d: 0x%04X count %d\n", slave, entry_idx, entry_cnt_2); 

                        for (int j = 1; j <= entry_cnt_2; ++j) {
                            uint32_t entry;
                            size_t entry_size = sizeof(entry);
                            wkc = ec_coe_sdo_read(pec, slave, entry_idx, j, 0, (uint8_t *)&entry, &entry_size);

                            if (wkc != 1)
                                ec_log("INIT_2_PREOP", "slave %d: reading 0x%04X/%d failed\n", 
                                        slave, entry_idx, j);

                            ec_log("INIT_2_PREOP", "slave %d: mapped entry %08X\n", slave, entry);

                            bit_len += entry & 0x000000FF;
                        }                        
                    }

                    ec_log("INIT_2_PREOP", "slave %d: sm%d length bits %d, bytes %d\n", 
                            slave, sm_idx, bit_len, (bit_len + 7) / 8);

                    if (slv->sm && slv->sm_ch > sm_idx)
                        slv->sm[sm_idx].len = (bit_len + 7) / 8;
                }
            } else {
                // try eeprom
                for (int sm_idx = 0; sm_idx < slv->sm_ch; ++sm_idx) {
                    size_t bit_len = 0;

                    // inputs and outputs
                    for (int txpdo_idx = 0; txpdo_idx < slv->eeprom.txpdos_cnt; ++txpdo_idx) {
                        ec_eeprom_cat_pdo_t *pdo = &slv->eeprom.txpdos[txpdo_idx];

                        if (sm_idx == pdo->sm_nr) 
                            bit_len += pdo->bit_len;
                    }

                    slv->sm[sm_idx].len = (bit_len + 7) / 8;
                }
            }
            
            if (transition == INIT_2_PREOP)
                break;
        }
        case PREOP_2_SAFEOP: 
        case PREOP_2_OP: {
            for (int sm_idx = 0; sm_idx < slv->sm_ch; ++sm_idx) {
                if (!slv->sm[sm_idx].adr)
                    continue;

                ec_log("PREOP_2_SAFEOP", "slave %d: sm%d, adr 0x%X, len %d, flags 0x%X\n",
                        slave, sm_idx, slv->sm[sm_idx].adr, 
                        slv->sm[sm_idx].len, slv->sm[sm_idx].flags);

                ec_fpwr(pec, slv->fixed_address, 0x800 + (sm_idx * 8),
                        &slv->sm[sm_idx], sizeof(ec_slave_sm_t), &wkc);
            }

            for (int fmmu_idx = 0; fmmu_idx < slv->fmmu_ch; ++fmmu_idx) { 
                if (!slv->fmmu[fmmu_idx].active) 
                    continue;

                // safeop to op stuff 
                ec_log("PREOP_2_SAFEOP", "slave %d: log 0x%X/%d/%d, len %d, "
                        "pyhs 0x%X/%d, type %d, active %d\n", slave,
                        slv->fmmu[fmmu_idx].log, slv->fmmu[fmmu_idx].log_bit_start,
                        slv->fmmu[fmmu_idx].log_bit_stop, slv->fmmu[fmmu_idx].log_len,
                        slv->fmmu[fmmu_idx].phys, slv->fmmu[fmmu_idx].phys_bit_start,
                        slv->fmmu[fmmu_idx].type, slv->fmmu[fmmu_idx].active);

                ec_fpwr(pec, slv->fixed_address, 0x600 + (16 * fmmu_idx),
                            (uint8_t *)&slv->fmmu[fmmu_idx], sizeof(ec_slave_fmmu_t), &wkc);

            }
            
            // write state to slave
            wkc = ec_slave_set_state(pec, slave, EC_STATE_SAFEOP);

            if (transition == INIT_2_SAFEOP || transition == PREOP_2_SAFEOP)
                break;
        }
        case SAFEOP_2_OP: {

            // write state to slave
            wkc = ec_slave_set_state(pec, slave, EC_STATE_OP);
            
            break;
        }
        case OP_2_INIT:
        case SAFEOP_2_INIT:
        case PREOP_2_INIT:
            // write state to slave
            wkc = ec_slave_set_state(pec, slave, state);
#define free_resource(a) \
            if ((a)) { \
                free((a)); \
                (a) = NULL; \
            }

            // free resources
            free_resource(slv->mbx_read.buf);
            free_resource(slv->mbx_write.buf);
            free_resource(slv->sm);
            free_resource(slv->fmmu);
        case INIT_2_INIT: {
            int i;
            uint16_t wkc = 0, features = 0, pdi_ctrl = 0;
            uint8_t sm_fmmu_ch[2], ram_size = 0;
            sm_fmmu_ch[0] = sm_fmmu_ch[1] = 0;

            // get number of sync managers and fmmus
            ec_fprd(pec, slv->fixed_address, EC_REG_SM_FFMU_CH,
                    sm_fmmu_ch, 2, &wkc);            
            slv->sm_ch = sm_fmmu_ch[1];
            slv->fmmu_ch = sm_fmmu_ch[0];
    
            // get ram size
            ec_fprd(pec, slv->fixed_address, EC_REG_RAM_SIZE,
                    &ram_size, sizeof(ram_size), &wkc);            
            slv->ram_size = ram_size << 10;

            // get pdi control 
            ec_fprd(pec, slv->fixed_address, EC_REG_PDICTL,
                    &pdi_ctrl, sizeof(pdi_ctrl), &wkc);            
            //slv->pdi_ctrl = pdi_ctrl;
            //
            ec_log("INIT_2_INIT", "slave %d pdi ctrl 0x%04X\n", slave, pdi_ctrl);
            // get pdi control 
            ec_fprd(pec, slv->fixed_address, 0xF8E,
                    &pdi_ctrl, sizeof(pdi_ctrl), &wkc);            
            ec_log("INIT_2_INIT", "slave %d pdi bytes syze 0x%04X\n", slave, pdi_ctrl);
            
            // get features
            ec_fprd(pec, slv->fixed_address, EC_REG_ESCSUP,
                    &features, sizeof(features), &wkc);            
            slv->features = features;

            // clear sync managers
            if (slv->sm) {
                free(slv->sm);
                slv->sm = NULL;
            }

            ec_log("INIT_2_INIT", "slave %d has %d sync managers\n", slave, slv->sm_ch);

            if (slv->sm_ch) {
                slv->sm = (ec_slave_sm_t *)malloc(
                        slv->sm_ch * sizeof(ec_slave_sm_t));
                memset(slv->sm, 0, slv->sm_ch * sizeof(ec_slave_sm_t));

                for (i = 0; i < slv->sm_ch; ++i) 
                    ec_transmit_no_reply(pec, EC_CMD_FPWR, 
                            ec_to_adr(slv->fixed_address, 0x800 + (8 * i)),
                            (uint8_t *)&slv->sm[i], sizeof(ec_slave_sm_t));
            }
                        
            // clear fmmus
            if (slv->fmmu) {
                free(slv->fmmu);
                slv->fmmu = NULL;
            }
            
            ec_log("INIT_2_INIT", "slave %d has %d fmmus\n", slave, slv->fmmu_ch);

            if (slv->fmmu_ch) {
                slv->fmmu = (ec_slave_fmmu_t *)malloc(
                        slv->fmmu_ch * sizeof(ec_slave_fmmu_t));
                memset(slv->fmmu, 0, slv->fmmu_ch * sizeof(ec_slave_fmmu_t));

                for (i = 0; i < slv->fmmu_ch; ++i) 
                    ec_transmit_no_reply(pec, EC_CMD_FPWR, 
                            ec_to_adr(slv->fixed_address, 0x600 + (16 * i)),
                            (uint8_t *)&slv->fmmu[i], sizeof(ec_slave_fmmu_t));
            }
        }
        case PREOP_2_PREOP:
        case SAFEOP_2_SAFEOP:
        case OP_2_OP:
            // write state to slave
            wkc = ec_slave_set_state(pec, slave, state);
            break;
        default:
            break;
    };

    return wkc;
}

//! create process data groups
/*!
 * \param pec ethercat master pointer
 * \param pd_group_cnt number of groups to create
 * \return 0 on success
 */
int ec_create_pd_groups(ec_t *pec, int pd_group_cnt) {
    int i;
    ec_destroy_pd_groups(pec);

    pec->pd_group_cnt = pd_group_cnt;
    pec->pd_groups = (ec_pd_group_t *)malloc(sizeof(ec_pd_group_t) * pd_group_cnt);
    for (i = 0; i < pec->pd_group_cnt; ++i) {
        pec->pd_groups[i].log = 0x10000 * (i+1);
        pec->pd_groups[i].log_len = 0;
        pec->pd_groups[i].pd = NULL;
        pec->pd_groups[i].pdout_len = 0;
        pec->pd_groups[i].pdin_len = 0;
    }

    return 0;
}

//! destroy process data groups
/*!
 * \param pec ethercat master pointer
 * \return 0 on success
 */
int ec_destroy_pd_groups(ec_t *pec) {
    int i;

    if (pec->pd_groups) {
        for (i = 0; i < pec->pd_group_cnt; ++i)
            if (pec->pd_groups[i].pd)
                free(pec->pd_groups[i].pd);
        free(pec->pd_groups);
    }

    pec->pd_group_cnt = 0;
    pec->pd_groups = NULL;

    return 0;
}

//! set state on ethercat bus
/*! 
 * \param pec ethercat master pointer
 * \param state new ethercat state
 * \return 0 on success
 */
int ec_set_state(ec_t *pec, ec_state_t state) {
    int ret = 0, i;

    switch (state) {
        case EC_STATE_INIT: {
            uint16_t fixed = 1000, wkc = 0, val = 0;

            ec_state_t init_state = EC_STATE_INIT | EC_STATE_RESET;
            ec_bwr(pec, EC_REG_ALCTL, &init_state, sizeof(init_state), &wkc); 

            // free resources if previosly initialized
            for (i = 0; i < pec->slave_cnt; ++i)
                ec_slave_state_transition(pec, i, state);

            pec->slave_cnt = 0;
            if (pec->slaves)
                free(pec->slaves);

            // allocating slave structures
            ret = ec_brd(pec, EC_REG_TYPE, (uint8_t *)&val, sizeof(val), &wkc); 
            pec->slave_cnt = wkc;
            pec->slaves = (ec_slave_t *)malloc(pec->slave_cnt * sizeof(ec_slave_t));
            memset(pec->slaves, 0, pec->slave_cnt * sizeof(ec_slave_t));    

            for (i = 0; i < 65536; ++i) {
                int auto_inc = -1 * i;

                ret = ec_aprd(pec, auto_inc, EC_REG_TYPE, (uint8_t *)&val, sizeof(val), &wkc);

                if (wkc == 0)
                    break;  // break here, cause there seems to be no more slave

                ec_log("INITIAL SCAN", "found slave with auto inc address %d, "
                        "wkc %d\n", auto_inc, wkc);

                pec->slaves[i].assigned_pd_group = -1;
                pec->slaves[i].auto_inc_address = auto_inc;
                pec->slaves[i].fixed_address = fixed;

                ec_apwr(pec, auto_inc, EC_REG_STADR, (uint8_t *)&fixed, sizeof(fixed), &wkc); 
                if (wkc == 1)
                    ec_log("INITIAL SCAN", "fixed address %d successfully "
                            "written to slave %d\n", fixed, auto_inc);

                fixed++;
            }

            for (i = 0; i < pec->slave_cnt; ++i)
                ec_slave_state_transition(pec, i, state);

            break;
        }
        case EC_STATE_SAFEOP: {
            int i, j;

            for (j = 0; j < pec->pd_group_cnt; ++j) {
                ec_pd_group_t *pd = &pec->pd_groups[j];
                pd->pdout_len = 0,
                pd->pdin_len = 0;

                for (i = 0; i < pec->slave_cnt; ++i) {
                    ec_slave_t *slv = &pec->slaves[i];

                    if (slv->assigned_pd_group != j)
                        continue;
                    int k;
                    for (k = 0; k < slv->sm_ch; ++k) {
                        if (slv->sm[k].flags & 0x00000002)
                            continue; // mailbox sm

                        if (slv->sm[k].flags & 0x00000004)
                            // outputs 
                            pd->pdout_len += slv->sm[k].len;
                        else 
                            // outputs 
                            pd->pdin_len += slv->sm[k].len;

                    }
                }
                
                ec_log("EC_STATE_SAFEOP", "got pd length in %d, out %d\n", pd->pdout_len, pd->pdin_len);

                pd->log_len = pd->pdout_len + pd->pdin_len;
                pd->pd = (uint8_t *)malloc(pd->log_len);
                memset(pd->pd, 0, pd->log_len);

                uint8_t *pdout = pd->pd,
                        *pdin = pd->pd + pd->pdout_len;

                uint32_t log_base_out = pd->log,
                         log_base_in = pd->log + pd->pdout_len;

                for (i = 0; i < pec->slave_cnt; ++i) {
                    ec_slave_t *slv = &pec->slaves[i];

                    if (slv->assigned_pd_group != j)
                        continue;
                    
                    int k, fmmu_next = 0;
                    for (k = 0; k < slv->sm_ch; ++k) {
                        if ((!slv->sm[k].len) || (slv->sm[k].flags & 0x00000002))
                            continue; // empty or mailbox sm
                    
                        if (slv->sm[k].flags & 0x00000004) {
                            slv->fmmu[fmmu_next].log = log_base_out;
                            slv->fmmu[fmmu_next].log_len = slv->sm[k].len;
                            slv->fmmu[fmmu_next].log_bit_stop = 7;
                            slv->fmmu[fmmu_next].phys = slv->sm[k].adr;
                            slv->fmmu[fmmu_next].type = 2;
                            slv->fmmu[fmmu_next].active = 1;

                            if (!slv->pdout_len) {
                                slv->pdout = pdout; 
                                slv->pdout_len = slv->sm[k].len;
                            } else 
                                slv->pdout_len += slv->sm[k].len;

                            pdout += slv->sm[k].len;
                            log_base_out += slv->sm[k].len;
                        } else {
                            slv->fmmu[fmmu_next].log = log_base_in;
                            slv->fmmu[fmmu_next].log_len = slv->sm[k].len;
                            slv->fmmu[fmmu_next].log_bit_stop = 7;
                            slv->fmmu[fmmu_next].phys = slv->sm[k].adr;
                            slv->fmmu[fmmu_next].type = 1;
                            slv->fmmu[fmmu_next].active = 1;

                            if (!slv->pdin_len) {
                                slv->pdin = pdin; 
                                slv->pdin_len = slv->sm[k].len;
                            } else 
                                slv->pdin_len += slv->sm[k].len;

                            pdin += slv->sm[k].len;
                            log_base_in += slv->sm[k].len;
                        }

                        fmmu_next++;
                    }
                }
            }
            
            for (i = 0; i < pec->slave_cnt; ++i)
                ec_slave_state_transition(pec, i, state);
            break;
        }
        default:
            for (i = 0; i < pec->slave_cnt; ++i)
                ec_slave_state_transition(pec, i, state);

            break;
    }

    return ret;
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
    int i;
    
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
    
    (*ppec)->phw = NULL;
    (*ppec)->slave_cnt = 0;
    (*ppec)->pd_group_cnt = 0;
    (*ppec)->slaves = NULL;
    (*ppec)->pd_groups = NULL;
    (*ppec)->tx_sync = 1;

    datagram_pool_open(&(*ppec)->pool, 1000);
    hw_open(&(*ppec)->phw, ifname, prio, cpumask);

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

    ec_destroy_pd_groups(pec);

    if (pec->slaves) {
        int slave;
        for (slave = 0; slave < pec->slave_cnt; ++slave) {
            ec_slave_t *slv = &pec->slaves[slave];

            if (slv->eeprom.strings) {
                int string;
                for (string = 0; string < slv->eeprom.strings_cnt; ++string)
                    free(slv->eeprom.strings[string]);

                free(slv->eeprom.strings);
            }

            if (slv->eeprom.sms)
                free(slv->eeprom.sms);

            if (slv->eeprom.fmmus)
                free(slv->eeprom.fmmus);

            if (slv->eeprom.txpdos)
                free(slv->eeprom.txpdos);

            if (slv->eeprom.rxpdos)
                free(slv->eeprom.rxpdos);

            if (slv->sm)
                free(slv->sm);

            if (slv->fmmu)
                free(slv->fmmu);

            if (slv->mbx_read.buf)
                free(slv->mbx_read.buf);
            if (slv->mbx_write.buf)
                free(slv->mbx_write.buf);
        }

        free(pec->slaves);
    }

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

    // send frame immediately if in sync mode
    if (pec->tx_sync)
        hw_tx(pec->phw);

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

    // send frame immediately if in sync mode
    if (pec->tx_sync)
        hw_tx(pec->phw);

    return 0;
}

