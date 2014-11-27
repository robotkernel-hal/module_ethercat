#ifndef __COMMON_H__
#define __COMMON_H__

#include "stdint.h"

#define PACKED __attribute__((__packed__))
#define min(a, b)  ((a) < (b) ? (a) : (b))

typedef union ec_data {
    uint8_t     bdata[1]; /* variants for easy data access */
    uint16_t    wdata[1];
    uint32_t    ldata[1];
} ec_data_t;

#endif // __COMMON_H__

