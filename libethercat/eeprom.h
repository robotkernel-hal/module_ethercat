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

#ifndef __EEPROM_H__
#define __EEPROM_H__

#include "common.h"
#include <stdlib.h>

typedef struct PACKED ec_eeprom_cat_general {
    uint8_t group_idx;          //!< group information, index to STRING
    uint8_t img_idx;            //!< image name, index to STRING
    uint8_t order_idx;          //!< device order number, index to STRING
    uint8_t name_idx;           //!< device name, index to STRING
    uint8_t physical_layer;     //!< physical layer, 0 e-bus, 1, 100base-tx
    uint8_t can_open;       
    uint8_t file_access;    
    uint8_t ethernet;   
    uint8_t soe_channels;   
    uint8_t ds402_channels;
    uint8_t sysman_class;
    uint8_t flags;
    uint16_t current_on_ebus;   //!< ebus current in [mA], negative = feed-in
} ec_eeprom_cat_general_t;

typedef struct PACKED ec_eeprom_cat_pdo {
    uint16_t pdo_index;
    uint8_t n_entry;
    uint8_t sm_nr;
    uint8_t dc_sync;
    uint8_t name_idx;
    uint16_t flags;
    uint16_t entry_sub_index;
    uint8_t sub_index;
    uint8_t entry_name_idx;
    uint8_t data_type;
    uint8_t bit_len;
    uint16_t flags_2;
} ec_eeprom_cat_pdo_t;

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
    
typedef struct eeprom_info {
    ec_eeprom_cat_general_t general;

    uint8_t strings_cnt;
    char **strings;

    uint8_t sms_cnt;
    ec_eeprom_cat_sm_t *sms;

    uint8_t fmmus_cnt;
    ec_eeprom_cat_fmmu_t *fmmus;

    uint8_t txpdos_cnt;
    ec_eeprom_cat_pdo_t *txpdos;
    
    uint8_t rxpdos_cnt;
    ec_eeprom_cat_pdo_t *rxpdos;
} eeprom_info_t;


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

// forward decl
struct ec;

//! read 32-bit word of eeprom
/*!
 * \param pec pointer to ethercat master
 * \param slave ethercat slave number
 * \param eepadr address in eeprom
 * \param returns data value
 * \return 0 on success
 */
int ec_eepromread(struct ec *pec, uint16_t slave, uint32_t eepadr, uint32_t *data);

//! read a burst of eeprom
/*!
 * \param pec pointer to ethercat master
 * \param slave ethercat slave number
 * \param eepadr address in eeprom
 * \param buf return buffer
 * \param buflen length in bytes to return
 * \return 0 on success
 */
int ec_eepromread_len(struct ec *pec, uint16_t slave, uint32_t eepadr, uint8_t *buf, size_t buflen);

#endif // __EEPROM_H__

