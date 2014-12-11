#include "mbx.h"
#include "coe.h"

#include <stdio.h>
#include <string.h>

typedef union ec_coeheader {
    uint16_t       canopen;
    struct PACKED {
        unsigned number   : 9;
        unsigned reserved : 3;
        unsigned service  : 4;
    };
} ec_coeheader_t;

typedef struct PACKED ec_sdoheader {
    union {
        uint8_t        value;
        struct PACKED {
            unsigned size_indicator     : 1;
            unsigned transfer_type      : 1;
            unsigned data_set_size      : 2;
            unsigned complete           : 1;
            unsigned command            : 3;
        };
    };
    uint16_t       index;
    uint8_t        sub_index;
} PACKED ec_sdoheader_t;

typedef struct PACKED ec_sdo {
    ec_mbxheader_t mbx_hdr;
    ec_coeheader_t coe_hdr;
    ec_sdoheader_t sdo_hdr;

    ec_data_t      sdo_data;
} PACKED ec_sdo_t;

typedef struct PACKED ec_sdo_download {
    ec_mbxheader_t mbx_hdr;
    ec_coeheader_t coe_hdr;
    ec_sdoheader_t sdo_hdr;
    uint32_t complete_size;

    ec_data_t      sdo_data;
} PACKED ec_sdo_download_t;

//! read coe sdo 
/*!
 * \param pec pointer to ethercat master
 * \param slave slave number
 * \param index sdo index
 * \param sub_index sdo sub index
 * \param complete complete access (only if sub_index == 0)
 * \param buf buffer to store answer
 * \param len length of buffer, outputs read length
 * \return working counter
 */
int ec_coe_sdo_read(ec_t *pec, uint16_t slave, uint16_t index, uint8_t sub_index, 
        int complete, uint8_t *buf, size_t *len) {
    int wkc;

    ec_mbx_clear(pec, slave, 0);
    ec_sdo_t *write_buf = (ec_sdo_t *)(pec->slaves[slave].mbx_write.buf);
    ec_sdo_t *read_buf  = (ec_sdo_t *)(pec->slaves[slave].mbx_read.buf); 

    // mailbox header
    write_buf->mbx_hdr.length       = 10; // (mbxhdr - length) + coehdr + sdohdr
    write_buf->mbx_hdr.address      = 0x0000;
    write_buf->mbx_hdr.priority     = 0x00;
    write_buf->mbx_hdr.mbxtype      = EC_MBX_COE;

    // coe header
    write_buf->coe_hdr.service      = EC_COE_SDOREQ;
    write_buf->coe_hdr.number       = 0x40;

    // sdo header
    write_buf->sdo_hdr.command      = EC_COE_SDO_UPLOAD_REQ;
    write_buf->sdo_hdr.complete     = complete;
    write_buf->sdo_hdr.index        = index;
    write_buf->sdo_hdr.sub_index    = sub_index;

    // send request
    wkc = ec_mbx_send(pec, slave);

    // wait for answer
    ec_mbx_clear(pec, slave, 1);
    wkc = ec_mbx_receive(pec, slave);

    size_t sdo_len = min(*len, read_buf->mbx_hdr.length - 6);
    memcpy(buf, read_buf->sdo_data.bdata, sdo_len);
    *len = sdo_len;

    return wkc;
}

//! write coe sdo 
/*!
 * \param pec pointer to ethercat master
 * \param slave slave number
 * \param index sdo index
 * \param sub_index sdo sub index
 * \param complete complete access (only if sub_index == 0)
 * \param buf buffer to write to sdo
 * \param len length of buffer, outputs written length
 * \return working counter
 */
int ec_coe_sdo_write(ec_t *pec, uint16_t slave, uint16_t index, 
        uint8_t sub_index, int complete, uint8_t *buf, size_t *len) {
    int wkc;

    ec_mbx_clear(pec, slave, 0);
    ec_sdo_t *write_buf = 
        (ec_sdo_t *)(pec->slaves[slave].mbx_write.buf);
    ec_sdo_t *read_buf  = 
        (ec_sdo_t *)(pec->slaves[slave].mbx_read.buf); 

    // mailbox header
    write_buf->mbx_hdr.length           = 10;// + *len; // (mbxhdr - length) + coehdr + sdohdr
    write_buf->mbx_hdr.address          = 0x0000;
    write_buf->mbx_hdr.priority         = 0x00;
    write_buf->mbx_hdr.mbxtype          = EC_MBX_COE;

    // coe header
    write_buf->coe_hdr.service          = EC_COE_SDOREQ;
    write_buf->coe_hdr.number           = 0x00;

    // sdo header
    write_buf->sdo_hdr.size_indicator   = 1;
    write_buf->sdo_hdr.command          = EC_COE_SDO_DOWNLOAD_REQ;
    write_buf->sdo_hdr.transfer_type    = 0;
    write_buf->sdo_hdr.data_set_size    = 0;
    write_buf->sdo_hdr.complete         = complete;
    write_buf->sdo_hdr.index            = index;
    write_buf->sdo_hdr.sub_index        = sub_index;

    if (*len <= 4) {
        write_buf->sdo_hdr.transfer_type = 1;
        write_buf->sdo_hdr.data_set_size = 4 - *len;
        memcpy(&write_buf->sdo_data.ldata[0], buf, *len);
    } else {
        write_buf->sdo_data.ldata[0] = *len;
        memcpy(&write_buf->sdo_data.ldata[1], buf, *len);
    }

    // send request
    wkc = ec_mbx_send(pec, slave);

    // wait for answer
    ec_mbx_clear(pec, slave, 1);
    wkc = ec_mbx_receive(pec, slave);

    size_t sdo_len = min(*len, read_buf->mbx_hdr.length - 6);
    memcpy(buf, read_buf->sdo_data.bdata, sdo_len);
    *len = sdo_len;

    return wkc;
}

typedef struct PACKED ec_sdoinfoheader {
    unsigned opcode     : 7;
    unsigned incomplete : 1;
    unsigned reserved   : 8;
    uint16_t fragments_left;
} PACKED ec_sdoinfoheader_t;

typedef struct PACKED ec_sdo_odlist_req {
    ec_mbxheader_t      mbx_hdr;
    ec_coeheader_t      coe_hdr;
    ec_sdoinfoheader_t  sdo_info_hdr;
    uint16_t            list_type;
} PACKED ec_sdo_odlist_req_t;

typedef struct PACKED ec_sdo_odlist_resp {
    ec_mbxheader_t      mbx_hdr;
    ec_coeheader_t      coe_hdr;
    ec_sdoinfoheader_t  sdo_info_hdr;
    ec_data_t           sdo_info_data;
} PACKED ec_sdo_odlist_resp_t;

//! read coe object dictionary list
/*!
 * \param pec pointer to ethercat master
 * \param slave slave number
 * \param buf buffer to store answer
 * \param len length of buffer, outputs read length
 * \return working counter
 */
int ec_coe_odlist_read(ec_t *pec, uint16_t slave, uint8_t *buf, size_t *len) {
    int wkc;

    ec_mbx_clear(pec, slave, 0);
    ec_sdo_odlist_req_t *write_buf = (ec_sdo_odlist_req_t *)(pec->slaves[slave].mbx_write.buf);
    ec_sdo_odlist_resp_t *read_buf = (ec_sdo_odlist_resp_t *)(pec->slaves[slave].mbx_read.buf); 

    // mailbox header
    write_buf->mbx_hdr.length       = 12; // (mbxhdr - length) + coehdr + sdohdr
    write_buf->mbx_hdr.address      = 0x0000;
    write_buf->mbx_hdr.priority     = 0x02;
    write_buf->mbx_hdr.mbxtype      = EC_MBX_COE;

    // coe header
    write_buf->coe_hdr.service      = EC_COE_SDOINFO;
    write_buf->coe_hdr.number       = 0x00;

    // sdo header
    write_buf->sdo_info_hdr.opcode  = EC_COE_SDO_INFO_ODLIST_REQ;
    write_buf->list_type            = 0x01;

    // send request
    wkc = ec_mbx_send(pec, slave);

    int val = 0;

    do {
        // wait for answer
        ec_mbx_clear(pec, slave, 1);
        wkc = ec_mbx_receive(pec, slave);
        
        uint8_t *from = val == 0 ? &read_buf->sdo_info_data.bdata[4] : 
            &read_buf->sdo_info_data.bdata[0];
        size_t len = val == 0 ? (read_buf->mbx_hdr.length - 10) : (read_buf->mbx_hdr.length - 6);

        memcpy(buf + val, from, len);
        val += len;
    } while (read_buf->sdo_info_hdr.fragments_left);

    *len = val;

    return wkc;
}

typedef struct PACKED ec_sdo_desc_req {
    ec_mbxheader_t      mbx_hdr;
    ec_coeheader_t      coe_hdr;
    ec_sdoinfoheader_t  sdo_info_hdr;
    uint16_t            index;
} PACKED ec_sdo_desc_req_t;

typedef struct PACKED ec_sdo_desc_resp {
    ec_mbxheader_t      mbx_hdr;
    ec_coeheader_t      coe_hdr;
    ec_sdoinfoheader_t  sdo_info_hdr;
    ec_data_t           sdo_info_data;
} PACKED ec_sdo_desc_resp_t;

//! read coe sdo description
/*!
 * \param pec pointer to ethercat master
 * \param slave slave number
 * \param index sdo index
 * \param buf buffer to store answer
 * \param len length of buffer, outputs read length
 * \return working counter
 */
int ec_coe_sdo_desc_read(ec_t *pec, uint16_t slave, uint16_t index,
        ec_coe_sdo_desc_t *desc) {
    int wkc;

    ec_mbx_clear(pec, slave, 0);
    ec_sdo_desc_req_t *write_buf = (ec_sdo_desc_req_t *)(pec->slaves[slave].mbx_write.buf);
    ec_sdo_desc_resp_t *read_buf = (ec_sdo_desc_resp_t *)(pec->slaves[slave].mbx_read.buf); 

    // mailbox header
    write_buf->mbx_hdr.length       = 12; // (mbxhdr - length) + coehdr + sdohdr
    write_buf->mbx_hdr.address      = 0x0000;
    write_buf->mbx_hdr.priority     = 0x02;
    write_buf->mbx_hdr.mbxtype      = EC_MBX_COE;

    // coe header
    write_buf->coe_hdr.service      = EC_COE_SDOINFO;
    write_buf->coe_hdr.number       = 0x00;

    // sdo header
    write_buf->sdo_info_hdr.opcode  = EC_COE_SDO_INFO_GET_OBJECT_DESC_REQ;
    write_buf->index                = index;

    // send request
    wkc = ec_mbx_send(pec, slave);

    // wait for answer
    ec_mbx_clear(pec, slave, 1);
    wkc = ec_mbx_receive(pec, slave);
    
    if (read_buf->coe_hdr.service == EC_COE_SDOINFO) {
        if (read_buf->sdo_info_hdr.opcode == EC_COE_SDO_INFO_GET_OBJECT_DESC_RESP) {
            // transfer was successfull
            desc->data_type         = read_buf->sdo_info_data.wdata[1];
            desc->obj_type          = read_buf->sdo_info_data.bdata[4];
            desc->max_subindices    = read_buf->sdo_info_data.bdata[5];

            size_t name_len = min(read_buf->mbx_hdr.length - 6 - 6, CANOPEN_MAXNAME - 1);
            memcpy(desc->name, &read_buf->sdo_info_data.bdata[6], name_len);
            desc->name[name_len] = '\0';
        }
    } else if (read_buf->coe_hdr.service == EC_COE_SDOREQ) {
        desc->data_type         = 0;
        desc->obj_type          = 0;
        desc->max_subindices    = 0;
        desc->name[0] = '\0';

        wkc = -1;
    }

    return wkc;
}

typedef struct PACKED ec_sdo_entry_desc_req {
    ec_mbxheader_t      mbx_hdr;
    ec_coeheader_t      coe_hdr;
    ec_sdoinfoheader_t  sdo_info_hdr;
    uint16_t            index;
    uint8_t             sub_index;
    uint8_t             value_info;
} PACKED ec_sdo_entry_desc_req_t;

typedef struct PACKED ec_sdo_entry_desc_resp {
    ec_mbxheader_t      mbx_hdr;
    ec_coeheader_t      coe_hdr;
    ec_sdoinfoheader_t  sdo_info_hdr;
    uint16_t            index;
    uint8_t             sub_index;
    uint8_t             value_info;
    uint16_t            data_type;
    uint16_t            bit_length;
    uint16_t            obj_access;
    ec_data_t           desc_data;
} PACKED ec_sdo_entry_desc_resp_t;

//! read coe sdo entry description
/*!
 * \param pec pointer to ethercat master
 * \param slave slave number
 * \param index sdo index
 * \param buf buffer to store answer
 * \param len length of buffer, outputs read length
 * \return working counter
 */
int ec_coe_sdo_entry_desc_read(ec_t *pec, uint16_t slave, uint16_t index, uint8_t sub_index,
        uint8_t value_info, ec_coe_sdo_entry_desc_t *desc) {
    int wkc;

    ec_mbx_clear(pec, slave, 0);
    ec_sdo_entry_desc_req_t *write_buf = (ec_sdo_entry_desc_req_t *)(pec->slaves[slave].mbx_write.buf);
    ec_sdo_entry_desc_resp_t *read_buf = (ec_sdo_entry_desc_resp_t *)(pec->slaves[slave].mbx_read.buf); 

    // mailbox header
    write_buf->mbx_hdr.length       = 12; // (mbxhdr - length) + coehdr + sdohdr
    write_buf->mbx_hdr.address      = 0x0000;
    write_buf->mbx_hdr.priority     = 0x02;
    write_buf->mbx_hdr.mbxtype      = EC_MBX_COE;

    // coe header
    write_buf->coe_hdr.service      = EC_COE_SDOINFO;
    write_buf->coe_hdr.number       = 0x00;

    // sdo header
    write_buf->sdo_info_hdr.opcode  = EC_COE_SDO_INFO_GET_ENTRY_DESC_REQ;
    write_buf->index                = index;
    write_buf->sub_index            = sub_index;
    write_buf->value_info           = value_info;

    // send request
    wkc = ec_mbx_send(pec, slave);

    // wait for answer
    ec_mbx_clear(pec, slave, 1);
    wkc = ec_mbx_receive(pec, slave);
    
    if (read_buf->coe_hdr.service == EC_COE_SDOINFO) {
        if (read_buf->sdo_info_hdr.opcode == EC_COE_SDO_INFO_GET_ENTRY_DESC_RESP) {
            // transfer was successfull
            desc->data_type     = read_buf->data_type;
            desc->bit_length    = read_buf->bit_length;
            desc->obj_access    = read_buf->obj_access;
            desc->data_len      = read_buf->mbx_hdr.length - 6 - 10;
            
            if (desc->data) {
                memcpy(desc->data, read_buf->desc_data.bdata, desc->data_len);
                int h;
                for (h = 0; h < desc->data_len; ++h) 
                    printf("%02X ", desc->data[h]);
                printf("\n");
            }

        }
    } else if (read_buf->coe_hdr.service == EC_COE_SDOREQ) {
        desc->data_type         = 0;
        desc->bit_length        = 0;
        desc->obj_access        = 0;
        desc->data_len          = 0;

        wkc = -1;
    }

    return wkc;
}

