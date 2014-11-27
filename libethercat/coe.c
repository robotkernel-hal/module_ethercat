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

    do {
    // wait for answer
    ec_mbx_clear(pec, slave, 1);
    wkc = ec_mbx_receive(pec, slave);

    int j;
    for (j = 0; j < (read_buf->mbx_hdr.length - 8) / 2; ++j) {
        printf("%02X ", read_buf->sdo_info_data.wdata[j]);
    }
    printf("\n");
    } while (read_buf->sdo_info_hdr.fragments_left);

//    size_t sdo_len = min(*len, read_buf->mbx_hdr.length - 6);
//    memcpy(buf, read_buf->sdo_data.bdata, sdo_len);
//    *len = sdo_len;

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
int ec_coe_sdo_desc_read(ec_t *pec, uint16_t slave, uint16_t index, uint8_t *buf, size_t *len) {
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

    do {
    // wait for answer
    ec_mbx_clear(pec, slave, 1);
    wkc = ec_mbx_receive(pec, slave);

//    int j;
//    for (j = 0; j < (read_buf->mbx_hdr.length - 6) / 2; ++j) {
//        printf("%02X ", read_buf->sdo_info_data.wdata[j]);
//    }
//    printf("\n");
    } while (read_buf->sdo_info_hdr.fragments_left);

    char *bufi = malloc(read_buf->mbx_hdr.length - 6 - 6 + 1);
    memcpy(bufi, &read_buf->sdo_info_data.bdata[6], read_buf->mbx_hdr.length - 6 - 6);
    bufi[read_buf->mbx_hdr.length - 6 - 6 + 1] = '\0';
    printf("%s\n", bufi);

//    size_t sdo_len = min(*len, read_buf->mbx_hdr.length - 6);
//    memcpy(buf, read_buf->sdo_data.bdata, sdo_len);
//    *len = sdo_len;

    return wkc;
}
