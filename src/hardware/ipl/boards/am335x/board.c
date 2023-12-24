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

#include "ipl.h"
#include <arm/am335x.h>
#include "ipl-am335x.h"
#include "board.h"
#include "init_am335x_ddr.h"
#include "am335x_pinmux.h"

#define CM_CLOCK_ENABLE      0x2

static void interface_clocks_en(void)
{
    out32(AM335X_CM_PER_L3_CLKCTRL,       2);
    out32(AM335X_CM_PER_L4LS_CLKCTRL,     2);
    out32(AM335X_CM_PER_L4FW_CLKCTRL,     2);
    out32(AM335X_CM_WKUP_L4WKUP_CLKCTRL,  2);
    out32(AM335X_CM_PER_L3_INSTR_CLKCTRL, 2);
    out32(AM335X_CM_PER_L4HS_CLKCTRL,     2);
    out32(AM335X_CM_PER_SPI1_CLKCTRL,     2);
    /* GPIO0 */
    out32(AM335X_CM_WKUP_GPIO0_CLKCTRL,   2);

    while (in32(AM335X_CM_PER_L3_CLKCTRL      ) != 2);
    while (in32(AM335X_CM_PER_L4LS_CLKCTRL    ) != 2);
    while (in32(AM335X_CM_PER_L4FW_CLKCTRL    ) != 2);
    while (in32(AM335X_CM_WKUP_L4WKUP_CLKCTRL ) != 2);
    while (in32(AM335X_CM_PER_L3_INSTR_CLKCTRL) != 2);
    while (in32(AM335X_CM_PER_L4HS_CLKCTRL    ) != 2);
    while (in32(AM335X_CM_PER_SPI1_CLKCTRL    ) != 2);
    while (in32(AM335X_CM_WKUP_GPIO0_CLKCTRL  ) != 2);
}

static void powerdomain_clocks_en(void)
{
    /*
     * Power domain wake up transitions
     * Must make its interface clock run before using the peripheral
     */
    out32(AM335X_CM_PER_L3_CLKSTCTRL,   2);
    out32(AM335X_CM_PER_L4LS_CLKSTCTRL, 2);
    out32(AM335X_CM_WKUP_CLKSTCTRL,     2);
    out32(AM335X_CM_PER_L4FW_CLKSTCTRL, 2);
    out32(AM335X_CM_PER_L3S_CLKSTCTRL,  2);
}

/*
 * Enable peripherals
 */
static void peripheral_clocks_en(void)
{
    /* Enable the module clock */
    out32(AM335X_CM_PER_TIMER2_CLKCTRL, 2);

    /* Select the Master osc (19.2 MHz) as Timer2 clock source */
    out32(AM335X_CLKSEL_TIMER2_CLK, 0x1);

    /* UART0 */
    out32(AM335X_CM_WKUP_UART0_CLKCTRL, 2);

    /* ELM */
    out32(AM335X_CM_PER_ELM_CLKCTRL, 2);

    /* i2c0 */
    out32(AM335X_CM_WKUP_I2C0_CLKCTRL, 2);

    /* MMC 0 */
    out32(AM335X_CM_PER_MMC0_CLKCTRL, 2);

    /* Enable the control module though RBL would have done it*/
    out32(AM335X_CM_WKUP_CONTROL_CLKCTRL, 2);

    while (in32(AM335X_CM_PER_TIMER2_CLKCTRL  ) != 2);
    while (in32(AM335X_CM_WKUP_UART0_CLKCTRL  ) != 2);
    while (in32(AM335X_CM_PER_ELM_CLKCTRL     ) != 2);
    while (in32(AM335X_CM_WKUP_I2C0_CLKCTRL   ) != 2);
    while (in32(AM335X_CM_WKUP_CONTROL_CLKCTRL) != 2);
}


void
init_edma(void)
{
    unsigned    chnl;
    unsigned    base = AM335X_EDMA0_CC_BASE;            /* EDMA base */

    out32(AM335X_CM_PER_TPCC_CLKCTRL,    CM_CLOCK_ENABLE);
    out32(AM335X_CM_PER_TPTC0_CLKCTRL,   CM_CLOCK_ENABLE);
    out32(AM335X_CM_PER_TPTC1_CLKCTRL,   CM_CLOCK_ENABLE);
    out32(AM335X_CM_PER_TPTC2_CLKCTRL,   CM_CLOCK_ENABLE);
    in32(AM335X_CM_PER_TPCC_CLKCTRL);

    /* DMA Channel mapping */
    for (chnl = 0; chnl < 64; chnl++)
        out32(base + 0x100 + chnl * 4, chnl << 5);

    /* Enable region 0 access for all channels */
    out32(base + AM335X_EDMA_DRAE(0), 0xFFFFFFFF);
    out32(base + AM335X_EDMA_DRAEH(0), 0xFFFFFFFF);

    /* Disable all events */
    base += AM335X_EDMA_GLOBAL;
    out32(base + AM335X_EDMA_EECR, 0xFFFFFFFF);
    out32(base + AM335X_EDMA_EECRH, 0xFFFFFFFF);
    out32(base + AM335X_EDMA_SECR, 0xFFFFFFFF);
    out32(base + AM335X_EDMA_SECRH, 0xFFFFFFFF);

    /* Clear events just in case */
    out32(base + AM335X_EDMA_ECR, 0xFFFFFFFF);
    out32(base + AM335X_EDMA_ECRH, 0xFFFFFFFF);

    /* Disable all interrupts */
    out32(base + AM335X_EDMA_IECR, 0xFFFFFFFF);
    out32(base + AM335X_EDMA_IECRH, 0xFFFFFFFF);

    /* Clear all interrupts just in case */
    out32(base + AM335X_EDMA_ICR, 0xFFFFFFFF);
    out32(base + AM335X_EDMA_ICRH, 0xFFFFFFFF);
}


static void init_timer(unsigned int base)
{
    /* Timer input clk set to external osc (19.2 MHz) by peripheral_clocks_en() */

    /* Reset the Timer */
    out32(base + AM335X_TIMER_TSICR, 0x2);

    /* Wait until the reset is done */
    while (in32(base + AM335X_TIMER_TIOCP_CFG) & 1);

    /* Start the Timer */
    out32((base + AM335X_TIMER_TCLR), 0x2b);
}

void wdt_disable(void)
{
    out32(AM335X_WDT_BASE+WDT_WSPR, 0xAAAA);
    while(in32(AM335X_WDT_BASE+WDT_WWPS) != 0x0);
    out32(AM335X_WDT_BASE+WDT_WSPR, 0x5555);
    while(in32(AM335X_WDT_BASE+WDT_WWPS) != 0x0);
}

 void init_am335x( )
{
    int count = LDELAY;

    init_pinmux();

    /* WDT1 is already running when the bootloader gets control
     * Disable it to avoid "random" resets
     */
    wdt_disable();

    interface_clocks_en();
    powerdomain_clocks_en();
    peripheral_clocks_en();

    /* Get Timer and UART out of reset */
    /* UART softreset */
    out32(UART_SYSCFG, in32(UART_SYSCFG) | 0x02);

    while(( (in32(UART_SYSSTS) & 0x1) != 0x1) && count--);

    /* Disable smart idle */
    out32(UART_SYSCFG, in32(UART_SYSCFG) | (1<<3));

    /* We use UART0 as debug output */
    init_seromap(AM335X_UART0_BASE, 115200, 48000000, 16);

    init_am335x_ddr();
    init_timer(AM335X_TIMER2_BASE);
    init_edma();
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/ipl/boards/am335x/board.c $ $Rev: 814385 $")
#endif
