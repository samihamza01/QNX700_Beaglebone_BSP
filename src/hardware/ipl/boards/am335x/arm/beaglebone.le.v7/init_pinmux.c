/*
 * $QNXLicenseC:
 * Copyright 2016, QNX Software Systems.
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

#include "ipl-am335x.h"
#include "am335x_pinmux.h"

static void init_uart0_pin_mux(void)
{
    out32(conf_uart0_rxd        , (MODE(0) | PULLUP_EN | RXACTIVE | PULLUDEN));    /* UART0_RXD  */
    out32(conf_uart0_txd        , (MODE(0) | PULLUDEN ));                          /* UART0_TXD  */
    out32(conf_uart0_ctsn       , (MODE(0) | RXACTIVE | PULLUDEN));                /* UART0_CTSN */
    out32(conf_uart0_rtsn       , (MODE(0) | PULLUP_EN | PULLUDEN));               /* UART0_RTSN */
}

void init_pinmux()
{
    init_uart0_pin_mux();
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/ipl/boards/am335x/arm/beaglebone.le.v7/init_pinmux.c $ $Rev: 814385 $")
#endif
