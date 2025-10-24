/*
 * G233 SPI Controller (Master)
 *
 * Copyright (c) 2025 LogosCoder(Mao Qin) forever@bupt.edu.cn
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/riscv/g233_spi.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"

static void g233_spi_update_irq(G233SPIState *s)
{
    bool irq_state = false;
    
    /* RXNE interrupt (Datasheet: CR2.RXNEIE & SR.RXNE) */
    if ((s->cr2 & SPI_CR2_RXNEIE) && (s->sr & SPI_SR_RXNE)) {
        irq_state = true;
    }
    
    /* TXE interrupt (Datasheet: CR2.TXEIE & SR.TXE) */
    if ((s->cr2 & SPI_CR2_TXEIE) && (s->sr & SPI_SR_TXE)) {
        irq_state = true;
    }
    
    /* Error interrupt (Datasheet: CR2.ERRIE & (SR.OVERRUN | SR.UNDERRUN)) */
    if ((s->cr2 & SPI_CR2_ERRIE) && (s->sr & (SPI_SR_UNDERRUN | SPI_SR_OVERRUN))) {
        irq_state = true;
    }
    
    qemu_set_irq(s->irq, irq_state);
}

static uint64_t g233_spi_read(void *opaque, hwaddr addr, unsigned size)
{
    G233SPIState *s = G233_SPI(opaque);
    uint32_t ret = 0;

    switch (addr) {
    case SPI_CR1:
        ret = s->cr1;
        break;
    case SPI_CR2:
        ret = s->cr2;
        break;
    case SPI_SR:
        ret = s->sr;
        break;
    case SPI_DR:
        ret = s->dr_rx & 0xFF; /* 8bit */
        /* Reading DR clears the RXNE flag */
        s->sr &= ~SPI_SR_RXNE;
        s->rx_buf_full = false;
        g233_spi_update_irq(s);
        break;
    case SPI_CSCTRL:
        ret = s->csctrl;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "g233_spi_read: Bad offset 0x%lx\n", addr);
        break;
    }
    return ret;
}

static void g233_spi_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    G233SPIState *s = G233_SPI(opaque);

    switch (addr) {
    case SPI_CR1:
        s->cr1 = value;
        break;
    case SPI_CR2:
        s->cr2 = value;
        g233_spi_update_irq(s);
        break;
    case SPI_SR:
        /* Datasheet: OVERRUN and UNDERRUN are "write 1 to clear" */
        s->sr &= ~(value & SPI_SR_OVERRUN);
        s->sr &= ~(value & SPI_SR_UNDERRUN);
        g233_spi_update_irq(s);
        break;
    case SPI_DR:
        /* Check if SPI is enabled */
        if (!(s->cr1 & SPI_CR1_SPE)) {
            qemu_log_mask(LOG_GUEST_ERROR, "g233-spi: SPI not enabled during transfer!\n");
            return;
        }
        /*
         * If RXNE is still 1 (last data not read) and a write is attempted,
         * trigger OVERRUN.
         */
        if (s->rx_buf_full || (s->sr & SPI_SR_RXNE)) {
            s->sr |= SPI_SR_OVERRUN;
            g233_spi_update_irq(s);
            return; /* Transfer is discarded */
        }

        /* 1. Set status: Clear TXE (no longer empty), set BSY (busy) */
        s->sr &= ~SPI_SR_TXE;
        s->sr |= SPI_SR_BSY;
        g233_spi_update_irq(s);
        
        /* 2. Perform 8-bit transfer on the SSI bus */
        uint8_t tx_data = value & 0xFF;
        uint8_t rx_data = ssi_transfer(s->ssi, tx_data);
        
        /* 3. Transfer complete: Save received data */
        s->dr_rx = rx_data;
        s->rx_buf_full = true;
        
        /* 4. Update status: Set RXNE (receive full), set TXE (ready for next), clear BSY */
        s->sr |= (SPI_SR_RXNE | SPI_SR_TXE);
        s->sr &= ~SPI_SR_BSY;
        
        g233_spi_update_irq(s);
        break;
        
    case SPI_CSCTRL:
        s->csctrl = value;
        /*
         * Manual control of CS GPIO lines
         */
        for (int i = 0; i < 4; i++) {
            if (s->cs_lines && s->cs_lines[i]) {
                /* Datasheet: 1 = active */
                bool enabled = (value >> (i + 0)) & 1; /* Bits 0-3: EN */
                bool active = (value >> (i + 4)) & 1; /* Bits 4-7: ACT */
                
                /* QEMU GPIO is active-low, so when (enabled && active), set_irq(0) */
                qemu_set_irq(s->cs_lines[i], !(enabled && active));
            }
        }

        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "g233_spi_write: Bad offset 0x%lx\n", addr);
        break;
    }
}

static const MemoryRegionOps g233_spi_ops = {
    .read = g233_spi_read,
    .write = g233_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN, /* RISC-V is little-endian */
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void g233_spi_reset(Object *obj, ResetType type)
{
    G233SPIState *s = G233_SPI(obj);

    s->cr1 = 0x00000000;
    s->cr2 = 0x00000000;
    s->sr = 0x00000002;    /* TXE=1 on reset */
    s->dr_rx = 0x0000000C;  /* Reset value  0x0 Cfor DR from datasheet */
    s->csctrl = 0x00000000;
    
    s->rx_buf_full = false;
    
    /* Deassert all CS lines (set to high, 1) */
    if (s->cs_lines) {
        for (int i = 0; i < 4; i++) {
            if (s->cs_lines[i]) {
                qemu_set_irq(s->cs_lines[i], 1); /* Inactive (high) */
            }
        }
    }
    g233_spi_update_irq(s);
}

static void g233_spi_realize(DeviceState *dev, Error **errp)
{
    G233SPIState *s = G233_SPI(dev);
    
    /* 1. Create the SSI bus for peripheral connections */
    s->ssi = ssi_create_bus(dev, "ssi");
    
    /* 2. Initialize the interrupt line (to PLIC) */
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    
    /* 3. Initialize 4 CS GPIO lines */
    s->cs_lines = g_new0(qemu_irq, 4);
    qdev_init_gpio_out_named(dev, s->cs_lines, SSI_GPIO_CS, 4);
    
    /* 4. Initialize MMIO */
    memory_region_init_io(&s->iomem, OBJECT(s), &g233_spi_ops, s,
                          TYPE_G233_SPI, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static const VMStateDescription vmstate_g233_spi = {
    .name = TYPE_G233_SPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cr1, G233SPIState),
        VMSTATE_UINT32(cr2, G233SPIState),
        VMSTATE_UINT32(sr, G233SPIState),
        VMSTATE_UINT32(dr_rx, G233SPIState),
        VMSTATE_UINT32(csctrl, G233SPIState),
        VMSTATE_BOOL(rx_buf_full, G233SPIState),
        VMSTATE_END_OF_LIST()
    }
};

static void g233_spi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = g233_spi_realize;
    rc->phases.hold = g233_spi_reset;
    dc->vmsd = &vmstate_g233_spi;
    dc->desc = "G233 SPI Controller";
}

static const TypeInfo g233_spi_info = {
    .name          = TYPE_G233_SPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(G233SPIState),
    .class_init    = g233_spi_class_init,
};

static void g233_spi_register_types(void)
{
    type_register_static(&g233_spi_info);
}

type_init(g233_spi_register_types)