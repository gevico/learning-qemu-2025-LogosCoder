/*
 * G233 RISC-V Custom Instruction Helpers
 *
 * Copyright (c) 2025 LogosCoder(Mao Qin) forever@bupt.edu.cn
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "qemu/log.h"
#include "system/address-spaces.h"
#include "system/memory.h"

/*
 * Custom DMA Instruction Helper
 *
 * This function simulates a hardware DMA engine that performs a matrix
 * transpose operation. It reads a matrix from a source physical address,
 * transposes it, and writes the result to a destination physical address.
 *
 * The operation's granularity is determined by the 'grain' parameter,
 * which sets the matrix dimensions (8x8, 16x16, or 32x32). Based on the
 * test case, the matrix element size is always 4 bytes (uint32_t).
 *
 * This helper operates on physical addresses, bypassing the CPU's MMU,
 * which correctly models the behavior of a DMA device.
 *
 * @env: The CPU state (passed automatically).
 * @dest_addr: The starting physical address of the destination matrix (from rd).
 * @src_addr: The starting physical address of the source matrix (from rs1).
 * @grain_reg: The grain parameter determining matrix size (from rs2).
 */
void helper_dma(CPURISCVState *env, target_ulong dest_addr,
                       target_ulong src_addr, target_ulong grain_reg)
{
    uint32_t grain = grain_reg & 0x3;
    uint32_t matrix_size;
    // Per the test case, the element size is always 4 bytes (uint32_t).
    const uint32_t element_size = 4;
    uint32_t i, j;
    MemTxResult result;

    switch (grain) {
    case 0:
        matrix_size = 8;
        break;
    case 1:
        matrix_size = 16;
        break;
    case 2:
        matrix_size = 32;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "riscv: custom_dma: Invalid grain %u\n", grain);
        helper_raise_exception(env, RISCV_EXCP_ILLEGAL_INST);
        return;
    }

    // Perform the matrix transpose operation.
    for (i = 0; i < matrix_size; ++i) {
        for (j = 0; j < matrix_size; ++j) {
            target_ulong src_offset = (i * matrix_size + j) * element_size;
            target_ulong dest_offset = (j * matrix_size + i) * element_size;
            uint32_t buffer; // Use a 4-byte buffer for a single element.

            // Read element from source matrix at [i][j]
            result = address_space_read(&address_space_memory,
                                        src_addr + src_offset,
                                        MEMTXATTRS_UNSPECIFIED,
                                        &buffer,
                                        element_size);
            if (result != MEMTX_OK) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "riscv: custom_dma: Read failed at phys 0x%"
                              PRIx64 "\n", src_addr + src_offset);
                return;
            }

            // Write element to destination matrix at [j][i]
            result = address_space_write(&address_space_memory,
                                         dest_addr + dest_offset,
                                         MEMTXATTRS_UNSPECIFIED,
                                         &buffer,
                                         element_size);
            if (result != MEMTX_OK) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "riscv: custom_dma: Write failed at phys 0x%"
                              PRIx64 "\n", dest_addr + dest_offset);
                return;
            }
        }
    }
}

/*
 * Helper function for the G233 'sort' instruction.
 *
 * This function simulates a hardware sorting unit that performs a bubble sort
 * on an array of 32-bit integers in memory.
 *
 * @env: The CPU state (passed automatically).
 * @addr: The starting physical address of the array (from rs1).
 * @array_num: The total size of the array (from rs2, unused in this
 *             implementation but available for bounds checking).
 * @sort_num: The number of elements to sort (from rd).
 */
void helper_sort(CPURISCVState *env, target_ulong addr,
                   target_ulong array_num, target_ulong sort_num)
{
    const uint32_t element_size = 4;
    int32_t val_j, val_j_plus_1;
    MemTxResult result;
    AddressSpace *as = &address_space_memory;
    for (int i = 0; i < sort_num - 1; i++) {
        bool swapped = false;
        for (int j = 0; j < sort_num - i - 1; j++) {
            target_ulong addr_j = addr + (j * element_size);
            target_ulong addr_j_plus_1 = addr + ((j + 1) * element_size);

            // 1. Read arr[j]
            result = address_space_read(as, addr_j, MEMTXATTRS_UNSPECIFIED,
                                        &val_j, element_size);
            if (result != MEMTX_OK) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "riscv: sort: read failed at 0x%" PRIx64 "\n", addr_j);
                return;
            }

            // 2. Read arr[j + 1]
            result = address_space_read(as, addr_j_plus_1, MEMTXATTRS_UNSPECIFIED,
                                        &val_j_plus_1, element_size);
            if (result != MEMTX_OK) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "riscv: sort: read failed at 0x%" PRIx64 "\n", addr_j_plus_1);
                return;
            }

            // 3. Compare
            if (val_j > val_j_plus_1) {
                // 4. Swap
                //  Write val_j_plus_1 to arr[j]
                result = address_space_write(as, addr_j, MEMTXATTRS_UNSPECIFIED,
                                             &val_j_plus_1, element_size);
                if (result != MEMTX_OK) {
                     qemu_log_mask(LOG_GUEST_ERROR,
                                   "riscv: sort: write failed at 0x%" PRIx64 "\n", addr_j);
                     return;
                }

                // Write val_j to arr[j + 1]
                result = address_space_write(as, addr_j_plus_1, MEMTXATTRS_UNSPECIFIED,
                                             &val_j, element_size);
                if (result != MEMTX_OK) {
                     qemu_log_mask(LOG_GUEST_ERROR,
                                   "riscv: sort: write failed at 0x%" PRIx64 "\n", addr_j_plus_1);
                     return;
                }
                
                swapped = true;
            }
        }
        // Optimization: if no elements were swapped, the array is sorted.
        if (!swapped) {
            break; 
        }
    }
}

/*
 * Helper function for the G233 'crush' instruction.
 *
 * This function simulates a hardware packer. It reads 'num' 8-bit elements
 * from a source address, extracts the lower 4 bits of each, and packs
 * two consecutive 4-bit nibbles into a single 8-bit byte at a destination
 * address.
 *
 * @env: The CPU state.
 * @dst_addr: The destination address (from rd).
 * @src_addr: The source address (from rs1).
 * @num: The number of elements to process from the source (from rs2).
 */
void helper_crush(CPURISCVState *env, target_ulong dst_addr,
                    target_ulong src_addr, target_ulong num)
{
    AddressSpace *as = &address_space_memory;
    MemTxResult result;
    uint8_t src_byte1, src_byte2;
    uint8_t packed_byte;
    target_ulong i = 0; // Source array index
    target_ulong j = 0; // Destination array index

    // Process two source bytes at a time
    while (i + 1 < num) {
        // 1. Read src[i]
        result = address_space_read(as, src_addr + i, MEMTXATTRS_UNSPECIFIED,
                                    &src_byte1, 1);
        if (result != MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "riscv: crush: read failed at 0x%" PRIx64 "\n", src_addr + i);
            return;
        }

        // 2. Read src[i + 1]
        result = address_space_read(as, src_addr + i + 1, MEMTXATTRS_UNSPECIFIED,
                                    &src_byte2, 1);
        if (result != MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "riscv: crush: read failed at 0x%" PRIx64 "\n", src_addr + i + 1);
            return;
        }

        // 3. Pack: dst[j] = (src[i] & 0x0F) | ((src[i+1] & 0x0F) << 4)
        packed_byte = (src_byte1 & 0x0F) | ((src_byte2 & 0x0F) << 4);

        // 4. Write to dst[j]
        result = address_space_write(as, dst_addr + j, MEMTXATTRS_UNSPECIFIED,
                                     &packed_byte, 1);
        if (result != MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "riscv: crush: write failed at 0x%" PRIx64 "\n", dst_addr + j);
            return;
        }

        i += 2; 
        j += 1; 
    }

    // Handle the last element if the source array has an odd number of elements
    if (i < num) {
        // 1. Read the last src[i]
        result = address_space_read(as, src_addr + i, MEMTXATTRS_UNSPECIFIED,
                                    &src_byte1, 1);
        if (result != MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "riscv: crush: read failed at 0x%" PRIx64 "\n", src_addr + i);
            return;
        }

        // 2. Pack: dst[j] = src[i] & 0x0F
        packed_byte = (src_byte1 & 0x0F);

        // 3. Write to dst[j]
        result = address_space_write(as, dst_addr + j, MEMTXATTRS_UNSPECIFIED,
                                     &packed_byte, 1);
        if (result != MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "riscv: crush: write failed at 0x%" PRIx64 "\n", dst_addr + j);
            return;
        }
    }
}


/*
 * Helper function for the G233 'expand' instruction.
 *
 * This function simulates a hardware unpacker. It reads 'num' 8-bit elements
 * from a source address, splits each byte into two 4-bit nibbles (low and high),
 * and writes each nibble as a separate 8-bit byte to a destination address.
 *
 * @env: The CPU state.
 * @dst_addr: The destination address (from rd).
 * @src_addr: The source address (from rs1).
 * @num: The number of elements to process from the source (from rs2).
 */
void helper_expand(CPURISCVState *env, target_ulong dst_addr,
                     target_ulong src_addr, target_ulong num)
{
    AddressSpace *as = &address_space_memory;
    MemTxResult result;
    uint8_t src_byte;
    uint8_t low_nibble, high_nibble;
    target_ulong i = 0; // Source array index
    target_ulong j = 0; // Destination array index

    for (i = 0; i < num; i++) {
        // 1. Read src[i]
        result = address_space_read(as, src_addr + i, MEMTXATTRS_UNSPECIFIED,
                                    &src_byte, 1);
        if (result != MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "riscv: expand: read failed at 0x%" PRIx64 "\n", src_addr + i);
            return;
        }

        // 2. Unpack:
        low_nibble = src_byte & 0x0F;
        high_nibble = (src_byte >> 4) & 0x0F;

        // 3. Write to dst[j] (low 4-bits)
        result = address_space_write(as, dst_addr + j, MEMTXATTRS_UNSPECIFIED,
                                     &low_nibble, 1);
        if (result != MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "riscv: expand: write failed at 0x%" PRIx64 "\n", dst_addr + j);
            return;
        }
        j++; 

        // 4. Write to dst[j+1] (high 4-bits)
        result = address_space_write(as, dst_addr + j, MEMTXATTRS_UNSPECIFIED,
                                     &high_nibble, 1);
        if (result != MEMTX_OK) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "riscv: expand: write failed at 0x%" PRIx64 "\n", dst_addr + j);
            return;
        }
        j++; 
    }
}