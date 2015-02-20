//! robotkernel module ethercat slave
/*!
 * author: Robert Burger
 *
 * $Id$
 */

/*
 * This file is part of robotkernel.
 *
 * robotkernel is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * robotkernel is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with robotkernel.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "libethercat/slave.h"
#include "libethercat/ec.h"
#include "libethercat/coe.h"

#include <string.h>

const char transition_string_init_to_init[]     = "INIT_2_INIT";
const char transition_string_init_to_preop[]    = "INIT_2_PREOP";
const char transition_string_init_to_safeop[]   = "INIT_2_SAFEOP";
const char transition_string_init_to_op[]       = "INIT_2_OP";
const char transition_string_preop_to_init[]    = "PREOP_2_INIT";
const char transition_string_preop_to_preop[]   = "PREOP_2_PREOP";
const char transition_string_preop_to_safeop[]  = "PREOP_2_SAFEOP";
const char transition_string_preop_to_op[]      = "PREOP_2_OP";
const char transition_string_safeop_to_init[]   = "SAFEOP_2_INIT";
const char transition_string_safeop_to_preop[]  = "SAFEOP_2_PREOP";
const char transition_string_safeop_to_safeop[] = "SAFEOP_2_SAFEOP";
const char transition_string_safeop_to_op[]     = "SAFEOP_2_OP";
const char transition_string_op_to_init[]       = "OP_2_INIT";
const char transition_string_op_to_preop[]      = "OP_2_PREOP";
const char transition_string_op_to_safeop[]     = "OP_2_SAFEOP";
const char transition_string_op_to_op[]         = "OP_2_OP";
const char transition_string_unknown[]          = "UNKNOWN";

const char *get_transition_string(ec_state_transition_t transition) {
    switch (transition) {
        case INIT_2_INIT:
            return transition_string_init_to_init;
        case INIT_2_PREOP:
            return transition_string_init_to_preop;
        case INIT_2_SAFEOP:
            return transition_string_init_to_safeop;
        case INIT_2_OP:
            return transition_string_init_to_op;
        case PREOP_2_INIT:
            return transition_string_preop_to_init;
        case PREOP_2_PREOP:
            return transition_string_preop_to_preop;
        case PREOP_2_SAFEOP:
            return transition_string_preop_to_safeop;
        case PREOP_2_OP:
            return transition_string_preop_to_op;
        case SAFEOP_2_INIT:
            return transition_string_safeop_to_init;
        case SAFEOP_2_PREOP:
            return transition_string_safeop_to_preop;
        case SAFEOP_2_SAFEOP:
            return transition_string_safeop_to_safeop;
        case SAFEOP_2_OP:
            return transition_string_safeop_to_op;
        case OP_2_INIT:
            return transition_string_op_to_init;
        case OP_2_PREOP:
            return transition_string_op_to_preop;
        case OP_2_SAFEOP:
            return transition_string_op_to_safeop;
        case OP_2_OP:
            return transition_string_op_to_op;
    }

    return transition_string_unknown;
}

extern const char *get_state_string(ec_state_t state);

//! set ethercat state on slave 
/*!
 * \param pec ethercat master pointer
 * \param slave number
 * \param state new ethercat state
 * \return wkc
 */
int ec_slave_set_state(ec_t *pec, uint16_t slave, ec_state_t state) {
    uint16_t wkc = 0, act_state, value;
    ec_fpwr(pec, pec->slaves[slave].fixed_address, 
            EC_REG_ALCTL, &state, sizeof(state), &wkc); 
    
    if (state & EC_STATE_RESET)
        return wkc; // just return here, we did an error reset

    pec->slaves[slave].expected_state = state;

    do {
        act_state = 0;
        wkc = ec_state_get_state(pec, slave, &act_state);

        ec_log(100, "EC_STATE_SET", "slave %d, state %X, act_state %X, wkc %d\n", 
                slave, state, act_state, wkc);
        
        if (act_state & EC_STATE_ERROR) {
            ec_fprd(pec, pec->slaves[slave].fixed_address, 
                    EC_REG_ALSTATCODE, &value, sizeof(value), &wkc);
            ec_log(10, "EC_STATE_SET", "slave %d, state switch to %d failed, alstatcode 0x%04X\n", 
                    slave, state, value);

            ec_slave_set_state(pec, slave, (act_state & EC_STATE_MASK) | EC_STATE_RESET);
            break;
        }

        struct timespec ts = { 0, 1000000 };
        nanosleep(&ts, NULL);
    } while (act_state != state);

    return wkc;
}

//! get ethercat state from slave 
/*!
 * \param pec ethercat master pointer
 * \param slave number
 * \param state return ethercat state
 * \return wkc
 */
int ec_state_get_state(ec_t *pec, uint16_t slave, ec_state_t *state) {
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

//! generate pd mapping
/*!
 * \param pec ethercat master pointer
 * \param slave slave number
 * \return wkc
 */
int ec_slave_generate_mapping(ec_t *pec, uint16_t slave) {
    ec_slave_t *slv = (ec_slave_t *)&pec->slaves[slave];
    
    // check sm settings
    if (slv->eeprom.mbx_supported & EC_EEPROM_MBX_COE) { // have coe mailbox, check objects 1c12, 1c13
        for (int sm_idx = 2; sm_idx <= 3; ++sm_idx) {
            int wkc, bit_len = 0, idx = 0x1c10 + sm_idx;
            uint8_t entry_cnt = 0, entry_cnt_2;
            size_t entry_cnt_size = sizeof(entry_cnt);
            wkc = ec_coe_sdo_read(pec, slave, idx, 0, 0, &entry_cnt, &entry_cnt_size);

            if (wkc != 1)
                ec_log(10, "GENERATE_MAPPING", "slave %2d: reading 0x%04X/%d failed\n", slave, idx, 0);

            ec_log(100, "GENERATE_MAPPING", "slave %2d: 0x%04X count %d\n", slave, idx, entry_cnt); 

            for (int i = 1; i <= entry_cnt; ++i) {
                uint16_t entry_idx;
                size_t entry_size = sizeof(entry_idx);
                wkc = ec_coe_sdo_read(pec, slave, idx, i, 0, (uint8_t *)&entry_idx, &entry_size);

                if (wkc != 1)
                    ec_log(10, "GENERATE_MAPPING", "slave %2d: reading 0x%04X/%d failed\n", slave, idx, i);

                entry_cnt_size = sizeof(entry_cnt_2);

                wkc = ec_coe_sdo_read(pec, slave, entry_idx, 0, 0, (uint8_t *)&entry_cnt_2, &entry_cnt_size);

                if (wkc != 1)
                    ec_log(10, "GENERATE_MAPPING", "slave %2d: reading 0x%04X/%d failed\n", slave, entry_idx, 0);

                ec_log(100, "GENERATE_MAPPING", "slave %2d: 0x%04X count %d\n", slave, entry_idx, entry_cnt_2); 

                for (int j = 1; j <= entry_cnt_2; ++j) {
                    uint32_t entry;
                    size_t entry_size = sizeof(entry);
                    wkc = ec_coe_sdo_read(pec, slave, entry_idx, j, 0, (uint8_t *)&entry, &entry_size);

                    if (wkc != 1)
                        ec_log(10, "GENERATE_MAPPING", "slave %2d: reading 0x%04X/%d failed\n", 
                                slave, entry_idx, j);

                    ec_log(100, "GENERATE_MAPPING", "slave %2d: mapped entry %08X\n", slave, entry);

                    bit_len += entry & 0x000000FF;
                }                        
            }
                
            if (bit_len) {
                ec_log(10, "GENERATE_MAPPING", "slave %2d: sm%d length bits %d, bytes %d\n", 
                        slave, sm_idx, bit_len, (bit_len + 7) / 8);

                if (slv->sm && slv->sm_ch > sm_idx)
                    slv->sm[sm_idx].len = (bit_len + 7) / 8;
            }
        }
    } else {
        // try eeprom
        for (int sm_idx = 0; sm_idx < slv->sm_ch; ++sm_idx) {
            size_t bit_len = 0;

            // inputs and outputs
            for (int txpdo_idx = 0; txpdo_idx < slv->eeprom.txpdos_cnt; ++txpdo_idx) {
                ec_eeprom_cat_pdo_t *pdo = &slv->eeprom.txpdos[txpdo_idx];
                ec_log(100, "GENERATE_MAPPING", "slave %2d: got txpdo bit_len %d, sm %d\n", 
                        slave, pdo->bit_len, pdo->sm_nr);

                if (sm_idx == pdo->sm_nr) 
                    bit_len += pdo->bit_len;
            }

            // outputs
            for (int rxpdo_idx = 0; rxpdo_idx < slv->eeprom.rxpdos_cnt; ++rxpdo_idx) {
                ec_eeprom_cat_pdo_t *pdo = &slv->eeprom.rxpdos[rxpdo_idx];
                ec_log(100, "GENERATE_MAPPING", "slave %2d: got rxpdo bit_len %d, sm %d\n", 
                        slave, pdo->bit_len, pdo->sm_nr);

                if (sm_idx == pdo->sm_nr) 
                    bit_len += pdo->bit_len;
            }
            
            ec_log(100, "GENERATE_MAPPING", "slave %2d: txpdos %d, rxpdos %d, bitlen%d %d\n", 
                    slave, slv->eeprom.txpdos_cnt, slv->eeprom.rxpdos_cnt, sm_idx, bit_len);
            
            if (bit_len > 0) {
                ec_log(10, "GENERATE_MAPPING", "slave %2d: sm%d length bits %d, bytes %d\n", 
                        slave, sm_idx, bit_len, (bit_len + 7) / 8);

                slv->sm[sm_idx].len = (bit_len + 7) / 8;
            }
        }
    }

    return 1;
}

//! state transition on ethercat slave
/*!
 * \param pec ethercat master pointer
 * \param slave slave number
 * \param state switch to state
 * \return wkc
 */
int ec_slave_state_transition(ec_t *pec, uint16_t slave, ec_state_t state) {
    uint16_t wkc;
    ec_state_t act_state = 0;
    ec_slave_t *slv = &pec->slaves[slave];
    
#define ec_reg_read(reg, buf, buflen) { uint16_t wkc; \
    ec_fprd(pec, pec->slaves[slave].fixed_address, (reg), (buf), (buflen), &wkc); \
    if (!wkc) ec_log(10, __func__, "reading reg 0x%X : no answer from slave %d\n", slave); }

#define free_resource(a) \
            if ((a)) { \
                free((a)); \
                (a) = NULL; \
            }


    // check error state
    wkc = ec_state_get_state(pec, slave, &act_state);
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
            
            ec_log(10, get_transition_string(transition), "slave %2d, vendor 0x%08X, product 0x%08X, mbx 0x%04X\n",
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
                    ec_log(10, get_transition_string(transition), "slave %2d: sm%d, adr 0x%04X, len %3d, flags 0x%08X\n",
                            slave, sm_idx, slv->sm[sm_idx].adr, 
                            slv->sm[sm_idx].len, slv->sm[sm_idx].flags);

                    ec_fpwr(pec, slv->fixed_address, 0x800 + (sm_idx * 8),
                            &slv->sm[sm_idx], sizeof(ec_slave_sm_t), &wkc);
                }
            }
            
            // write state to slave
            wkc = ec_slave_set_state(pec, slave, EC_STATE_PREOP);

            // check sm settings
//            ec_slave_generate_mapping(pec, slave);

            if (transition == INIT_2_PREOP)
                break;
        }
        case PREOP_2_SAFEOP: 
        case PREOP_2_OP: {
            int start_sm = slv->eeprom.mbx_supported ? 2 : 0;

            for (int sm_idx = start_sm; sm_idx < slv->sm_ch; ++sm_idx) {
                if (!slv->sm[sm_idx].adr)
                    continue;

                ec_log(10, get_transition_string(transition), "slave %2d: sm%d, adr 0x%04X, len %3d, flags 0x%08X\n",
                        slave, sm_idx, slv->sm[sm_idx].adr, 
                        slv->sm[sm_idx].len, slv->sm[sm_idx].flags);

                ec_fpwr(pec, slv->fixed_address, 0x800 + (sm_idx * 8),
                        &slv->sm[sm_idx], sizeof(ec_slave_sm_t), &wkc);
            }

            for (int fmmu_idx = 0; fmmu_idx < slv->fmmu_ch; ++fmmu_idx) { 
                if (!slv->fmmu[fmmu_idx].active) 
                    continue;

                // safeop to op stuff 
                ec_log(10, get_transition_string(transition), "slave %2d: log 0x%08X/%d/%d, len %3d, "
                        "pyhs 0x%04X/%d, type %d, active %d\n", slave,
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
        
        case INIT_2_INIT: {
            // free resources
            free_resource(slv->mbx_read.buf);
            free_resource(slv->mbx_write.buf);
            free_resource(slv->sm);
            free_resource(slv->fmmu);

            // get number of sync managers and fmmus
            ec_reg_read(EC_REG_SM_CH, &slv->sm_ch, 1);
            ec_reg_read(EC_REG_FMMU_CH, &slv->fmmu_ch, 1);
    
            // get ram size
            uint8_t ram_size = 0;
            ec_reg_read(EC_REG_RAM_SIZE, &ram_size, sizeof(ram_size));
            slv->ram_size = ram_size << 10;

            // get pdi control 
            ec_reg_read(EC_REG_PDICTL, &slv->pdi_ctrl, sizeof(slv->pdi_ctrl));
            
            // get features
            ec_reg_read(EC_REG_ESCSUP, &slv->features, sizeof(slv->features));

            if (slv->sm_ch) {
                slv->sm = (ec_slave_sm_t *)malloc(
                        slv->sm_ch * sizeof(ec_slave_sm_t));
                memset(slv->sm, 0, slv->sm_ch * sizeof(ec_slave_sm_t));

                for (int i = 0; i < slv->sm_ch; ++i) 
                    ec_transmit_no_reply(pec, EC_CMD_FPWR, 
                            ec_to_adr(slv->fixed_address, 0x800 + (8 * i)),
                            (uint8_t *)&slv->sm[i], sizeof(ec_slave_sm_t));
            }
                        
            if (slv->fmmu_ch) {
                slv->fmmu = (ec_slave_fmmu_t *)malloc(
                        slv->fmmu_ch * sizeof(ec_slave_fmmu_t));
                memset(slv->fmmu, 0, slv->fmmu_ch * sizeof(ec_slave_fmmu_t));

                for (int i = 0; i < slv->fmmu_ch; ++i) 
                    ec_transmit_no_reply(pec, EC_CMD_FPWR, 
                            ec_to_adr(slv->fixed_address, 0x600 + (16 * i)),
                            (uint8_t *)&slv->fmmu[i], sizeof(ec_slave_fmmu_t));
            }
                
            ec_log(10, get_transition_string(transition), "slave %2d: pdi ctrl 0x%04X, fmmus %d, syncm %d\n", 
                    slave, slv->pdi_ctrl, slv->fmmu_ch, slv->sm_ch);
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
