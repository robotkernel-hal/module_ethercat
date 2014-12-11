
/** CoE mailbox types */
enum {
    EC_COE_EMERGENCY  = 0x01,
    EC_COE_SDOREQ,
    EC_COE_SDORES,
    EC_COE_TXPDO,
    EC_COE_RXPDO,
    EC_COE_TXPDO_RR,
    EC_COE_RXPDO_RR,
    EC_COE_SDOINFO
};

enum {
    EC_COE_SDO_DOWNLOAD_REQ = 0x01,
    EC_COE_SDO_UPLOAD_REQ,
};

enum {
    EC_COE_SDO_INFO_ODLIST_REQ = 0x01,
    EC_COE_SDO_INFO_ODLIST_RESP,
    EC_COE_SDO_INFO_GET_OBJECT_DESC_REQ,
    EC_COE_SDO_INFO_GET_OBJECT_DESC_RESP,
    EC_COE_SDO_INFO_GET_ENTRY_DESC_REQ,
    EC_COE_SDO_INFO_GET_ENTRY_DESC_RESP,
};

#define CANOPEN_MAXNAME 40
    
typedef struct PACKED ec_coe_sdo_desc {
    uint16_t data_type;             //! element data type
    uint8_t  obj_type;              //! object type
    uint8_t  max_subindices;        //! maximum number of subindices
    char     name[CANOPEN_MAXNAME]; //! element name
} PACKED ec_coe_sdo_desc_t;

typedef struct PACKED ec_coe_sdo_entry_desc {
    uint16_t            data_type;
    uint16_t            bit_length;
    uint16_t            obj_access;
    uint8_t            *data;
    size_t              data_len;
} PACKED ec_coe_sdo_entry_desc_t;

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
int ec_coe_sdo_read(ec_t *pec, uint16_t slave, uint16_t index, 
        uint8_t sub_index, int complete, uint8_t *buf, size_t *len);

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
        uint8_t sub_index, int complete, uint8_t *buf, size_t *len);

//! read coe sdo description
/*!
 * \param pec pointer to ethercat master
 * \param slave slave number
 * \param index sdo index
 * \param desc buffer to store answer
 * \return working counter
 */
int ec_coe_sdo_desc_read(ec_t *pec, uint16_t slave, uint16_t index, 
        ec_coe_sdo_desc_t *desc);

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
        uint8_t value_info, ec_coe_sdo_entry_desc_t *desc);

//! read coe object dictionary list
/*!
 * \param pec pointer to ethercat master
 * \param slave slave number
 * \param buf buffer to store answer
 * \param len length of buffer, outputs read length
 * \return working counter
 */
int ec_coe_odlist_read(ec_t *pec, uint16_t slave, uint8_t *buf, size_t *len);

