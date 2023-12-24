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

#include <ipl.h>
#include <hw/inout.h>
#include <hw/omap_i2c.h>
#include <arm/am335x.h>
#include <arm/omap.h>
#include <private/fat-fs.h>
#include <private/omap_sdhc.h>

#ifndef __AM335X_IPL_H_INCLUDED
#define __AM335X_IPL_H_INCLUDED

#define UART_SYSCFG                      (AM335X_UART0_BASE + 0x54)
#define UART_SYSSTS                      (AM335X_UART0_BASE + 0x58)

#define LDELAY                           12000000

typedef enum {
    IPL_BOOT_SD,
    IPL_BOOT_EMMC,
    IPL_BOOT_NAND,
    IPL_BOOT_SERIAL,
    IPL_UPDATE_IPL,
    IPL_UPDATE_IFS,
    IPL_UPDATE_IFS_ALL,
    IPL_UPDATE_IMAGE,
    IPL_MEM_TEST,
    IPL_RESET,
    IPL_PRINT_BOOT_STRUCT,
    IPL_CLEAR_IMAGE_FLAGS,
    IPL_CLEAR_UPDATE_FLAG,
    IPL_BOOT_UNKNOWN
} IPL_BootOpt_t;

extern void init_edma(void);
extern void init_am335x(void);

#endif /* __AM335X_IPL_H_INCLUDED */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/ipl/boards/am335x/ipl-am335x.h $ $Rev: 798438 $")
#endif