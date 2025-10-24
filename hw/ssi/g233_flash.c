/*
 * G233 SPI Flash (W25X series)
 *
 * Copyright (c) 2025 LogosCoder(Mao Qin) forever@bupt.edu.cn
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/ssi/g233_flash.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "block/block.h"
#include "hw/qdev-properties.h"

/* Flash Commands */
#define CMD_WREN            0x06  /* Write Enable */
#define CMD_READ_STATUS     0x05  /* Read Status Register */
#define CMD_READ_DATA       0x03  /* Read Data */
#define CMD_JEDEC_ID        0x9F  /* Read JEDEC ID */
#define CMD_PAGE_PROGRAM    0x02  /* Page Program */
#define CMD_SECTOR_ERASE    0x20  /* Sector Erase (4KB) */

/* Status Register Bits */
#define SR_WIP              (1 << 0)  /* Write In Progress */
#define SR_WEL              (1 << 1)  /* Write Enable Latch */

/*
 * This function is called when the CS line is deasserted (set high).
 * This is where all write operations (Page Program, Sector Erase) are
 * actually executed.
 */
static void g233_flash_cs_deassert(G233FlashState *s)
{
    /* when in the page program state, commit the write now */
    if (s->state == STATE_WRITE_PAGE && s->page_pos > 0) {
        if (s->current_addr + s->page_pos <= s->size) {
            /* Write page buffer to internal storage */
            memcpy(s->storage + s->current_addr, s->page_buf, s->page_pos);
            qemu_log_mask(LOG_GUEST_ERROR,
                          "g233-flash: PageProgram wrote %d bytes to address 0x%x\n",
                          s->page_pos, s->current_addr);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "g233-flash: PageProgram out-of-bounds write, addr=0x%x\n",
                          s->current_addr);
        }
        /* After a page program, the write enable latch is cleared */
        s->write_enable = false;
    }
    
    /* After a sector erase, the write enable latch is cleared */
    if (s->current_cmd == CMD_SECTOR_ERASE) {
        s->write_enable = false;
    }
        
    /* Reset state machine back to IDLE */
    s->state = STATE_IDLE;
    s->current_cmd = 0;
    s->page_pos = 0;
    s->data_pos = 0;
}

/*
 * the core SPI transfer function.
 */
static uint32_t g233_flash_transfer(SSIPeripheral *ss, uint32_t tx)
{
    G233FlashState *s = G233_FLASH(ss);
    uint8_t rx = 0xFF; /* SPI MISO is pulled high by default (returns 0xFF) */

    switch (s->state) {
    case STATE_IDLE:
        s->current_cmd = tx;
        switch (tx) {
        case CMD_JEDEC_ID:
            s->state = STATE_READ_ID;
            s->data_pos = 0;
            break;
        case CMD_READ_STATUS:
            s->state = STATE_READ_SR;
            break;
        case CMD_WREN:
            s->write_enable = true;
            break;
        case CMD_READ_DATA:
        case CMD_PAGE_PROGRAM:
        case CMD_SECTOR_ERASE:
            s->state = STATE_READ_ADDR;
            s->current_addr = 0;
            s->addr_bytes_left = 3;
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                          "g233-flash: Unknown command 0x%02x\n", tx);
            break;
        }
        break;

    case STATE_READ_ADDR:
        s->current_addr = (s->current_addr << 8) | tx;
        s->addr_bytes_left--;

        if (s->addr_bytes_left == 0) {
            if (s->current_cmd == CMD_READ_DATA) {
                s->state = STATE_READ_DATA;
                s->data_pos = 0;
            } else if (s->current_cmd == CMD_PAGE_PROGRAM) {
                s->state = s->write_enable ? STATE_WRITE_PAGE : STATE_IDLE;
                s->page_pos = 0;
            } else if (s->current_cmd == CMD_SECTOR_ERASE) {
                if (s->write_enable) {
                    uint32_t sector_addr = s->current_addr & ~0xFFF;
                    if (sector_addr < s->size) {
                        memset(s->storage + sector_addr, 0xFF, 4096);
                    }
                }
                s->state = STATE_IDLE; /* Erase is immediate */
            }
        }
        break;

    case STATE_READ_ID:
        if (s->data_pos < 3) {
            rx = s->jedec_id[s->data_pos++];
        }
        break;

    case STATE_READ_SR:
        s->status_reg = s->write_enable ? SR_WEL : 0;
        rx = s->status_reg;
        break;

    case STATE_READ_DATA:
        if (s->current_addr + s->data_pos < s->size) {
            rx = s->storage[s->current_addr + s->data_pos];
        }
        s->data_pos++;
        break;

    case STATE_WRITE_PAGE:
        if (s->page_pos < 256) {
            s->page_buf[s->page_pos++] = tx;
        }
        break;
    }

    return rx;
}

static int g233_flash_set_cs(SSIPeripheral *dev, bool select)
{
    G233FlashState *s = G233_FLASH(dev);
    if (!select) {  // deassert
        g233_flash_cs_deassert(s);
    }
    return 0;
}

static void g233_flash_realize(SSIPeripheral *dev, Error **errp)
{
    G233FlashState *s = G233_FLASH(dev);

    /* Default size (W25X16) */
    if (!s->size) {
        s->size = 2 * 1024 * 1024;
    }

    /* Set JEDEC ID based on size */
    s->jedec_id[0] = 0xEF; /* Winbond */
    s->jedec_id[1] = 0x30; /* Memory Type */
    if (s->size == 2 * 1024 * 1024) {
        s->jedec_id[2] = 0x15; /* W25X16 (2MB) */
    } else if (s->size == 4 * 1024 * 1024) {
        s->jedec_id[2] = 0x16; /* W25X32 (4MB) */
    }

    s->storage = g_malloc0(s->size);
    memset(s->storage, 0xFF, s->size);
}

static void g233_flash_finalize(Object *obj)
{
    G233FlashState *s = G233_FLASH(obj);
    g_free(s->storage);
}

static const Property g233_flash_properties[] = {
    DEFINE_PROP_UINT32("size", G233FlashState, size, 0),
};

static const VMStateDescription vmstate_g233_flash = {
    .name = TYPE_G233_FLASH,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(status_reg, G233FlashState),
        VMSTATE_BOOL(write_enable, G233FlashState),
        VMSTATE_END_OF_LIST()
    }
};

static void g233_flash_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    k->transfer = g233_flash_transfer;
    k->realize = g233_flash_realize;
    k->set_cs = g233_flash_set_cs;         
    k->cs_polarity = SSI_CS_LOW;           
    dc->vmsd = &vmstate_g233_flash;
    device_class_set_props(dc, g233_flash_properties);
}

static const TypeInfo g233_flash_info = {
    .name = TYPE_G233_FLASH,
    .parent = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(G233FlashState),
    .instance_finalize = g233_flash_finalize,
    .class_init = g233_flash_class_init,
};

static void g233_flash_register_types(void)
{
    type_register_static(&g233_flash_info);
}

type_init(g233_flash_register_types)