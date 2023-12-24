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

#include "ipl-am335x.h"

#define MAX_SCAN             0x200
#define QNX_LOAD_ADDRESS     0x84000000

// NOTE: Buffers are allotted space in the SRAM for EDMA.
static unsigned char fat_buf1[FAT_FS_INFO_BUF_SIZE] __attribute__ ((section(".scratch")));
static unsigned char fat_buf2[FAT_COMMON_BUF_SIZE] __attribute__ ((section(".scratch")));

volatile uint32_t temp1;

// this could be optimized
unsigned get_uint(unsigned port_address){
    int value = 0;
    unsigned output=0;
    unsigned tmp_out;
    char buf[8];
    char hex[17]="0123456789ABCDEF";

    while (1) {
        while(!(value & OMAP_LSR_RXRDY)){
            value = in8(port_address+OMAP_UART_LSR);
        }
        tmp_out = in8(port_address);
        if (tmp_out == 0xD) break;
        if (tmp_out > 0x5A) tmp_out-=32;    // make uppercase
        tmp_out -= 0x30;            // make integer
        if (tmp_out < 0) continue;
        if (tmp_out > 9) tmp_out -= 7;      // convert alphabetic to hex
        if (tmp_out > 15) continue;
        buf[0]=hex[tmp_out];
        buf[1]='\0';
        ser_putstr(buf);
        output <<= 4;
        output |= tmp_out;
        value = 0;
    }
    return(output);
}

IPL_BootOpt_t ipl_boot_menu()
{
    char    opt;

unsigned tmp_addr, tmp_val;

    while (1) {
        ser_putstr("\nCommand: \n");
        ser_putstr("Press 'S' for SERIAL download, using the 'sendnto' utility to download file qnx-ifs .\n");
        ser_putstr("Press 'M' for SDMMC download, file qnx-ifs assumed.\n");
        ser_putstr("Press 'r' followed by physical address to read memory\n");
        ser_putstr("Press 'w' followed by write address, followed by new value to write to memory\n");

        opt = ser_getchar();

        switch (opt) {
        case 'M':
        case 'm':
            return (IPL_BOOT_SD);
        case 's':
        case 'S':
            return (IPL_BOOT_SERIAL);
        case 'R': case 'r':
            ser_putstr((char *)"Enter physical address to read: 0x");
            tmp_addr = get_uint(AM335X_UART0_BASE);
            ser_putstr((char *)"\n Value of 0x");
            ser_puthex(tmp_addr);
            ser_putstr((char *)" is: ");
            ser_puthex(in32(tmp_addr));
            ser_putstr((char *)"\n");
            continue;
        case 'W': case 'w':
            ser_putstr("Enter physical address to write: 0x");
            tmp_addr = get_uint(AM335X_UART0_BASE);
            ser_putstr("\nEnter value (8-bits): 0x");
            tmp_val = get_uint(AM335X_UART0_BASE);
            *(unsigned char*)tmp_addr = (unsigned char)(tmp_val&0xff);
            continue;
        }

        ser_putstr("Unrecognized option\n");
    }
    return 0;
}



static int sdmmc_load_file (unsigned address, const char *fn){
    sdmmc_t            sdmmc;
    int                status;
    omap_edma_ext_t dma_ext = {
        .dma_pbase = AM335X_EDMA0_CC_BASE,
        .dma_chnl = 25
    };
    omap_sdmmc_init_hc(&sdmmc, AM335X_MMCHS0_BASE, 192000, SDMMC_VERBOSE_LVL_0, OMAP_SDMMC_EDMA, &dma_ext);

    if (sdmmc_init_sd(&sdmmc)) {
        ser_putstr("SD/MMC card init failed\n");
        status = SDMMC_ERROR;
        goto done;
    }

    ser_putstr("Load QNX image ");
    ser_putstr(fn);
    ser_putstr(" from SDMMC...\n");

    fat_sdmmc_t    fat = {
        .ext = &sdmmc,
        .buf1 = fat_buf1,
        .buf1_len = FAT_FS_INFO_BUF_SIZE,
        .buf2 = fat_buf2,
        .buf2_len = FAT_COMMON_BUF_SIZE,
        .verbose = 3
    };

    if (fat_init(&fat)) {
        ser_putstr("Failed to init fat-fs\n");
        status = SDMMC_ERROR;
        goto done;
    }

    status = fat_copy_named_file((unsigned char *)address, (char *)fn);

done:
    sdmmc_fini(&sdmmc);

#if defined (DEBUG_BOOT_TIMING)
    omap_timer_curr("IFS loading from SDMMC", TIMING_MILLI_SECOND);
#endif

    return status;
}

int main(void) {

    unsigned image;
    IPL_BootOpt_t bootOpt;


    init_am335x();

    ser_putstr((char *)"\nQNX Neutrino Initial Program Loader for AM335x Board\n");

    image = QNX_LOAD_ADDRESS;

    bootOpt = ipl_boot_menu();

    while (1) {
        switch (bootOpt) {
            case IPL_BOOT_SD:
                ser_putstr("\nload image from SD ...\n");

                if (sdmmc_load_file(image, "QNX-IFS") != 0) {
                    ser_putstr("load image from SD failed\n");
                    goto print_boot_menu;
                }
                break;

            case IPL_BOOT_SERIAL:
                ser_putstr("Send IFS image through serial now...\n");

                if (image_download_ser(image) != 0) {
                    ser_putstr("Download image failed\n");
                    goto print_boot_menu;
                }

                ser_putstr("Download ok...\n");

                /* get remaining bytes */
                while (ser_poll())
                    ser_getchar();

                break;

            default:
                goto print_boot_menu;
        }

        if (bootOpt == IPL_BOOT_SERIAL) {
            image = image_scan_2(image, image + MAX_SCAN, 0);
        } else {
            image = image_scan_2(image, image + MAX_SCAN, 1);
        }

        if (image != 0xffffffff) {
            ser_putstr((char *)"\nFound image @ 0x");
            ser_puthex(image);
            ser_putstr((char *)"\n");

            image_setup_2(image);

            ser_putstr((char *)"\nJumping to startup @ 0x");
            ser_puthex(startup_hdr.startup_vaddr);
            ser_putstr((char *)"\n\n");
            image_start_2(image);

            /* Never reach here */
            return 0;
        } else {
            ser_putstr((char *)"Image_scan failed...\n");
        }

print_boot_menu:
       bootOpt = ipl_boot_menu();
    }

    return 0;


}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/ipl/boards/am335x/main.c $ $Rev: 798438 $")
#endif