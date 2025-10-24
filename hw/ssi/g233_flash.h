/*
 * G233 SPI Flash (W25X series)
 *
 * Copyright (c) 2025 LogosCoder(Mao Qin) forever@bupt.edu.cn
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SSI_G233_FLASH_H
#define HW_SSI_G233_FLASH_H

#include "hw/ssi/ssi.h"
#include "hw/qdev-properties.h"
#include "qapi-builtin-types.h"
#include "qom/object.h"

#define TYPE_G233_FLASH "g233-flash"
OBJECT_DECLARE_SIMPLE_TYPE(G233FlashState, G233_FLASH)

/* Internal state machine for the Flash device */
typedef enum {
    STATE_IDLE,         /* Waiting for a command */
    STATE_READ_ADDR,    /* Reading 3-byte address */
    STATE_READ_ID,      /* Sending JEDEC ID */
    STATE_READ_SR,      /* Sending Status Register */
    STATE_READ_DATA,    /* Reading flash data */
    STATE_WRITE_PAGE,   /* Receiving page program data */
} G233FlashInternalState;


struct G233FlashState {
    /* QEMU base class */
    SSIPeripheral parent_obj;

    /* Properties */
    BlockBackend *blk; /* Backend for disk.img */
    uint32_t size;     /* Flash size (2MB or 4MB) */

    /* Internal storage */
    uint8_t *storage;  /* In-memory flash content */
    uint8_t jedec_id[3];

    /* State machine */
    G233FlashInternalState state;
    uint8_t current_cmd;  /* Current command being processed */
    uint32_t current_addr; /* Current address being read */
    int addr_bytes_left;   /* How many address bytes are left to read */
    int data_pos;          /* Position in READ_ID or READ_DATA */
    
    /* Page program buffer */
    uint8_t page_buf[256]; /* Page size for W25X series is 256 bytes */
    int page_pos;

    /* Flash status */
    uint8_t status_reg;
    bool write_enable;
};

#endif /* HW_SSI_G233_FLASH_H */