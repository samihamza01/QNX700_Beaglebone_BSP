/*
 * $QNXLicenseC:
 * Copyright 2015, QNX Software Systems.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"). You
 * may not reproduce, modify or distribute this software except in
 * compliance with the License. You may obtain a copy of the License
 * at: http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" basis,
 * WITHOUT WARRANTIES OF ANY KIND, either express or implied.
 *
 * This file may contain contributions from others, either as
 * contributors under the License or as licensors under other terms.
 * Please review this entire file for other proprietary rights or license
 * notices, as well as the QNX Development Suite License Guide at
 * http://licensing.qnx.com/license-guide/ for other information.
 * $
 */


/*
 * Reset code for TI AM335x
 * the main code execute in SRAM.
 */

#define STACK_SIZE              12288

    .text
    .align  2
    .globl  _start

_start:

    /*
     * Set the cpu to SVC32 mode
     */
    mrs     r0, cpsr
    bic     r0, r0, #0x1f
    orr     r0, r0, #0xd3
    msr     cpsr, r0

    /*
     * Invalidate L1 I/D
     */
    mov     r0, #0                  // set up for MCR
    mcr     p15, 0, r0, c8, c7, 0   // invalidate TLBs
    mcr     p15, 0, r0, c7, c5, 0   // invalidate icache
    mcr     p15, 0, r0, c7, c5, 6   // invalidate BP array
    mcr     p15, 0, r0, c7, c10, 4  // DSB
    mcr     p15, 0, r0, c7, c5, 4   // ISB
    /*
     * disable MMU stuff and caches
     */
    mrc     p15, 0, r0, c1, c0, 0
    bic     r0, r0, #0x00002000     // clear bits 13 (--V-)
    bic     r0, r0, #0x00000007     // clear bits 2:0 (-CAM)
    orr     r0, r0, #0x00000002     // set bit 1 (--A-) Align
    orr     r0, r0, #0x00000800     // set bit 11 (Z---) BTB
    bic     r0, r0, #0x00001000     // clear bit 12 (I) I-cache
    mcr     p15, 0, r0, c1, c0, 0

    /*
     * Set stack pointer
     */
    ldr     sp, =_stack_bottom + STACK_SIZE
    bl      main

    /*
     * In case return from main
     */
    b       _start

    /*
     * Reserved IPL stack
     */
    .section .stack
_stack_bottom:
    .space STACK_SIZE

    .text

#ifdef __QNXNTO__
#ifdef __USESRCVERSION
.section .ident, "SM",%progbits,1;
.asciz "$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/ipl/boards/am335x/_start.S $ $Rev: 786744 $";
.previous
#endif
#endif