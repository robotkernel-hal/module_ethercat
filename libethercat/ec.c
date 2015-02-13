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
#include "slave.h"
#include "mbx.h"
#include "coe.h"

void *ec_log_func_user = NULL;
void (*ec_log_func)(int lvl, void *user, const char *format, ...) = NULL;

void ec_log(int lvl, const char *pre, const char *format, ...) {
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
        int ret = snprintf(tmp, 512, "%-20.20s: ", pre);
        vsnprintf(tmp+ret, 512-ret, format, args);

        ec_log_func(lvl, ec_log_func_user, buf);
    }
}


int ec_slave_state_get(ec_t *pec, uint16_t slave, ec_state_t *state);

int ec_master_state_set(ec_t *pec, ec_state_t state) {
    uint16_t wkc = 0;
    uint16_t value = (uint16_t)state;
    ec_bwr(pec, EC_REG_ALCTL, &value, sizeof(value), &wkc); 
    return wkc;
}


int ec_coe_calc_pd_len(ec_t *pec, uint16_t slave, uint16_t pdo_reg) {
//    ec_coe_sdo_read(pec, slave, pdo_reg, 0, buf, 
    return 0;
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

const char state_string_init[]    = "EC_STATE_INIT";
const char state_string_preop[]   = "EC_STATE_PREOP";
const char state_string_safeop[]  = "EC_STATE_SAFEOP";
const char state_string_op[]      = "EC_STATE_OP";
const char state_string_unknown[] = "EC_STATE_UNKNOWN";

const char *get_state_string(ec_state_t state) {
    if (state == EC_STATE_INIT)
        return state_string_init;
    if (state == EC_STATE_PREOP)
        return state_string_preop;
    if (state == EC_STATE_SAFEOP)
        return state_string_safeop;
    if (state == EC_STATE_OP)
        return state_string_op;

    return state_string_unknown;
}

//! set state on ethercat bus
/*! 
 * \param pec ethercat master pointer
 * \param state new ethercat state
 * \return 0 on success
 */
int ec_set_state(ec_t *pec, ec_state_t state) {
    int ret = 0, i;

    ec_log(10, "SET MASTER STATE", "switch to state %s\n", get_state_string(state));

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

                ec_log(100, get_state_string(state), "found slave with auto inc address %d, "
                        "wkc %d\n", auto_inc, wkc);

                pec->slaves[i].assigned_pd_group = -1;
                pec->slaves[i].auto_inc_address = auto_inc;
                pec->slaves[i].fixed_address = fixed;
                pec->slaves[i].dc.use_dc = 1;

                ec_apwr(pec, auto_inc, EC_REG_STADR, (uint8_t *)&fixed, sizeof(fixed), &wkc); 
                if (wkc == 1)
                    ec_log(100, get_state_string(state), "fixed address %d successfully "
                            "written to slave %d\n", fixed, auto_inc);

                fixed++;
            }

            ec_log(10, get_state_string(state), "found %d ethercat slaves\n", i);

            for (int slave = 0; slave < pec->slave_cnt; ++slave) {
                ec_slave_t *slv = &pec->slaves[slave]; 
                ec_slave_state_transition(pec, slave, state);

                uint16_t topology = 0;
                ec_fprd(pec, slv->fixed_address, EC_REG_DLSTAT, &topology, sizeof(topology), &wkc);

                slv->link_cnt = 0;
                slv->active_ports = 0;

                if ((topology & 0x0300) == 0x0200) { // port 0 open and communication established
                    slv->link_cnt++;
                    slv->active_ports |= 0x01;
                }
                if ((topology & 0x0c00) == 0x0800) { // port1 open and communication established
                    slv->link_cnt++;
                    slv->active_ports |= 0x02;
                }
                if ((topology & 0x3000) == 0x2000) { // port2 open and communication established
                    slv->link_cnt++;
                    slv->active_ports |= 0x04;
                }
                if ((topology & 0xc000) == 0x8000) { // port3 open and communication established
                    slv->link_cnt++;
                    slv->active_ports |= 0x08;
                }

                // read out physical type
                ec_fprd(pec, slv->fixed_address, EC_REG_PORTDES, &slv->ptype, sizeof(slv->ptype), &wkc);

                // 0=no links, not possible 
                // 1=1 link  , end of line 
                // 2=2 links , one before and one after 
                // 3=3 links , split point 
                // 4=4 links , cross point 

                // search for parent
                slv->parent = -1; // parent is master at beginning
                if (slave >= 1) {
                    int topoc = 0, tmp_slave = slave - 1;
                    do {
                        topology = pec->slaves[tmp_slave].link_cnt;
                        if (topology == 1)
                            topoc--;    // endpoint found
                        if (topology == 3)
                            topoc++;    // split found
                        if (topology == 4)
                            topoc += 2; // cross found
                        if (((topoc >= 0) && (topology > 1)) || (tmp_slave == 0)) { 
                            slv->parent = tmp_slave; // parent found
                            tmp_slave = 0;
                        }
                        tmp_slave--;
                    }
                    while (tmp_slave >= 0);
                }

                ec_log(100, get_state_string(state), "slave %2d has parent %d\n", slave, slv->parent);

            }

            break;
        }        
        case EC_STATE_PREOP:
            for (i = 0; i < pec->slave_cnt; ++i)
                ec_slave_state_transition(pec, i, state);

            ec_dc_config(pec);

            break;
        case EC_STATE_SAFEOP: {
            int i, j, k;
            for (int slave = 0; slave < pec->slave_cnt; ++slave)
                ec_slave_generate_mapping(pec, slave);

            for (j = 0; j < pec->pd_group_cnt; ++j) {
                ec_pd_group_t *pd = &pec->pd_groups[j];
                pd->pdout_len = pd->pdin_len = 0;
        
                for (i = 0; i < pec->slave_cnt; ++i) {
                    ec_slave_t *slv = &pec->slaves[i];
                    int start_sm = slv->eeprom.mbx_supported ? 2 : 0;

                    if (slv->assigned_pd_group != j)
                        continue;

                    for (k = start_sm; k < slv->sm_ch; ++k) {
                        if (slv->sm[k].flags & 0x00000004)
                            pd->pdout_len += slv->sm[k].len; // outputs
                        else 
                            pd->pdin_len += slv->sm[k].len;  // inputs

                    }
                }
                
                ec_log(10, get_state_string(state), "group %2d: pd out 0x%08X %3d bytes, in 0x%08X %3d bytes\n", 
                        j, pd->log, pd->pdout_len, pd->log + pd->pdout_len, pd->pdin_len);

                pd->log_len = pd->pdout_len + pd->pdin_len;
                pd->pd = (uint8_t *)malloc(pd->log_len);
                memset(pd->pd, 0, pd->log_len);

                uint8_t *pdout = pd->pd,
                        *pdin = pd->pd + pd->pdout_len;

                uint32_t log_base_out = pd->log,
                         log_base_in = pd->log + pd->pdout_len;

                for (i = 0; i < pec->slave_cnt; ++i) {
                    ec_slave_t *slv = &pec->slaves[i];
                    int start_sm = slv->eeprom.mbx_supported ? 2 : 0;

                    if (slv->assigned_pd_group != j)
                        continue;
                    
                    int fmmu_next = 0;
                    for (k = start_sm; k < slv->sm_ch; ++k) {
                        if ((!slv->sm[k].len))
                            continue; // empty 
                    
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
        memset(&entry->waiter, 0, sizeof(sem_t));
        sem_init(&entry->waiter, 0, 0);
        ec_index_put(*ppec, entry);
    }
    
    (*ppec)->phw = NULL;
    (*ppec)->slave_cnt = 0;
    (*ppec)->pd_group_cnt = 0;
    (*ppec)->slaves = NULL;
    (*ppec)->pd_groups = NULL;
    (*ppec)->tx_sync = 1;

    (*ppec)->dc.have_dc = 0;

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

#define free_resource(a) \
            if ((a)) { \
                free((a)); \
                (a) = NULL; \
            }

            free_resource(slv->eeprom.sms);
            free_resource(slv->eeprom.fmmus);
            free_resource(slv->eeprom.txpdos);
            free_resource(slv->eeprom.rxpdos);
            free_resource(slv->sm);
            free_resource(slv->fmmu);
            free_resource(slv->mbx_read.buf);
            free_resource(slv->mbx_write.buf);
        }

        free(pec->slaves);
    }

    free(pec);

    return 0;
}

pthread_mutex_t idx_lock = PTHREAD_MUTEX_INITIALIZER; 

//! get next free index entry
/*!
 * \param pec pointer to ethercat master
 * \param entry return entry of next free index 
 * \return 0 on succes, otherwise error code
 */
int ec_index_get(ec_t *pec, struct idx_entry **entry) {
    int ret = -1;

//    pthread_mutex_lock(&idx_lock);

    *entry = (idx_entry_t *)TAILQ_FIRST(&pec->idx);
    if (*entry) {
        TAILQ_REMOVE(&pec->idx, *entry, qh);
        ret = 0;
    }
    
//    pthread_mutex_unlock(&idx_lock);

    return ret;
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

//    pthread_mutex_lock(&idx_lock);
    TAILQ_INSERT_TAIL(&pec->idx, entry, qh);
//    pthread_mutex_unlock(&idx_lock);

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

