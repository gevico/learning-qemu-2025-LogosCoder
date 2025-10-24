/*
 * G233 SPI Controller (Master)
 *
 * Copyright (c) 2025 LogosCoder(Mao Qin) forever@bupt.edu.cn
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_G233_SPI_H
#define HW_MISC_G233_SPI_H

#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"
#include "qom/object.h"

#define TYPE_G233_SPI "g233-spi"
OBJECT_DECLARE_SIMPLE_TYPE(G233SPIState, G233_SPI)

/* Register Offsets (from G233 Datasheet) */
#define SPI_CR1     0x00
#define SPI_CR2     0x04
#define SPI_SR      0x08
#define SPI_DR      0x0C
#define SPI_CSCTRL  0x10

/* SPI_CR1 - Control Register 1 (0x00) */
#define SPI_CR1_SPE     (1 << 6)   /* SPI Enable */
#define SPI_CR1_MSTR    (1 << 2)   /* Master Mode Select */

/* SPI_CR2 - Control Register 2 (0x04) */
#define SPI_CR2_TXEIE   (1 << 7)   /* TXE Interrupt Enable */
#define SPI_CR2_RXNEIE  (1 << 6)   /* RXNE Interrupt Enable */
#define SPI_CR2_ERRIE   (1 << 5)   /* Error Interrupt Enable */

/* SPI_SR - Status Register (0x08) */
#define SPI_SR_BSY      (1 << 7)   /* Busy Flag */
#define SPI_SR_OVERRUN  (1 << 3)   /* Overrun Error (OVR) */
#define SPI_SR_UNDERRUN (1 << 2)   /* Underrun Error (UDR) */
#define SPI_SR_TXE      (1 << 1)   /* Transmit Buffer Empty */
#define SPI_SR_RXNE     (1 << 0)   /* Receive Buffer Not Empty */

struct G233SPIState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    SSIBus *ssi;          /* SSI bus (to peripherals) */
    qemu_irq irq;         /* Our interrupt output (to PLIC) */
    qemu_irq *cs_lines;   /* Our CS GPIO outputs (to Flash) */
    
    /* Registers */
    uint32_t cr1;
    uint32_t cr2;
    uint32_t sr;
    uint32_t dr_rx;      /* Receive data buffer (for 0x0C) */
    uint32_t csctrl;
    
    bool rx_buf_full; /* Internal state to track RXNE and OVR */
};

#endif /* HW_MISC_G233_SPI_H */