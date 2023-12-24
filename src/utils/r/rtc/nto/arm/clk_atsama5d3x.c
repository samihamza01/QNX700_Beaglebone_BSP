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

#include "rtc.h"
#include <time.h>
#include <arm/atsama5d3x.h>

#define RTC_CR     0x00 /* Control register      */
#define RTC_CR_UPDATE   0x3UL /* UPDCAL, UPDTIME */
#define RTC_MR     0x04 /* Mode register         */
#define RTC_MR_EXPECTED 0x0UL /* 24 hour mode    */
#define RTC_TIMR   0x08 /* Time register         */
#define RTC_CALR   0x0C /* Calendar register     */
#define RTC_SR     0x18 /* Status register       */
#define RTC_SR_UPDATE   0x1UL /* ACKUPD          */
#define RTC_SCCR   0x1C /* Status clear register */
#define RTC_VER    0x2C /* Valid entry register  */
#define RTC_VER_MASK    0x3UL /* NVTIM, NVCAL    */

/* The RTC was setup to be very generic and it does not perfectly line up with Posix.
 * We have to make some decisions to resolve these issues:
 *
 * I. 12 hour or 24 hour time
 * We will always use 24 hour time. If the hardware is configured to use 12 hour time,
 * we'll refuse to get the time. Yes, it would be easy to support both; however, if
 * the hardware is configured to use 12 hour time, it was not programmed by us, so it
 * is not safe to assume that our other assumptions hold.
 *
 * II. Day of the week
 * Posix counts the days from 0 to 6 where 0 is Sunday.
 * The RTC counts the days from 1 to 7, and the first day is undefined.
 * We will assume day 1 is Sunday.
 *
 * III. Leap seconds
 * Posix allows seconds from 0 to 61 inclusive.
 * The RTC allows seconds from 0 to 59 inclusive.
 * We will map 60 and 61 to 59.
 */

static unsigned bcd2bin(unsigned a) {
    return ((a & 0xf000) >> 12) * 1000 + ((a & 0xf00) >> 8) * 100 + ((a & 0xf0) >> 4) * 10 + (a & 0xf);
}

static unsigned bin2bcd(unsigned a) {
    return ((a / 1000) << 12) | (((a % 1000)/100) << 8) | (((a % 100)/10) << 4) | (a % 10);
}

int
RTCFUNC(init,atsama5d3x)(struct chip_loc *chip, char *argv[])
{
    if (chip->phys == NIL_PADDR) {
        chip->phys = SAMA5D3X_RTC_BASE;
    }
    if (chip->access_type == NONE) {
        chip->access_type = MEMMAPPED;
    }

    return SAMA5D3X_RTC_SIZE;
}

/* Verify the validity of the RTC time.
 * Returns -1 on failure, 0 on success.
 */
static int validity_check(void) {
    /* Verify that the entries are valid */
    if((chip_read32(RTC_VER) & RTC_VER_MASK) != 0) {
        fprintf(stderr, "ERROR: Invalid RTC entries found.\n");
        return (-1);
    }
    /* Verify that the RTC is in the correct mode */
    if(chip_read32(RTC_MR) != RTC_MR_EXPECTED) {
        fprintf(stderr, "ERROR: Unsupported RTC mode.\n");
        return (-1);
    }
    return 0;
}

int
RTCFUNC(get, atsama5d3x)(struct tm *tm, int cent_reg)
{
    /* Perform validity check and return early if it fails */
    if(validity_check() != 0) {
        return -1;
    }

    /* Extract time */
    uint32_t timr = chip_read32(RTC_TIMR);
    unsigned sec  = bcd2bin((timr >>  0) & 0x7F);
    unsigned minu = bcd2bin((timr >>  8) & 0x7F);
    unsigned hour = bcd2bin((timr >> 16) & 0x3F);
    if(verbose) {
        printf("<< Time <<\n");
        printf("%02u:%02u:%02u\n", hour, minu, sec);
    }

    /* Extract date */
    uint32_t calr = chip_read32(RTC_CALR);
    unsigned cent = bcd2bin((calr >>  0) & 0x7F);
    unsigned year = bcd2bin((calr >>  8) & 0xFF);
    unsigned mnth = bcd2bin((calr >> 16) & 0x1F);
    unsigned day  = bcd2bin((calr >> 21) & 0x07);
    unsigned date = bcd2bin((calr >> 24) & 0x3F);
    if(verbose) {
        printf("<< Date <<\n");
        printf("%u%02u/%02u/%02u (%u)\n", cent, year, mnth, date, day);
    }

    /* Fill out tm */
    tm->tm_sec   = sec;
    tm->tm_min   = minu;
    tm->tm_hour  = hour;
    tm->tm_mday  = date;
    tm->tm_mon   = mnth-1;
    tm->tm_year  = (year+(cent*100))-1900;
    tm->tm_wday  = day-1;
    tm->tm_yday  = 0; /* XXX: tm_yday is ignored */
    tm->tm_isdst = (-1);

    /* Success */
    return 0;
}

/* Disable the RTC.
 * This must be called before the RTC time and date can be set.
 */
static void rtc_lock(void) {
    /* Turn off the RTC */
    uint32_t cr = chip_read32(RTC_CR);
    chip_write32(RTC_CR, cr | RTC_CR_UPDATE);

    /* Wait for ACK */
    while((chip_read32(RTC_SR) & RTC_SR_UPDATE) == 0) {
        delay(1);
    }
}

/* Enable the RTC.
 * This must be called after the RTC time and date is set.
 */
static void rtc_unlock(void) {
    /* Turn on the RTC */
    uint32_t cr = chip_read32(RTC_CR);
    chip_write32(RTC_CR, cr & (~RTC_CR_UPDATE));
    /* Clear ACK */
    chip_write32(RTC_SCCR, RTC_SR_UPDATE);
}

int
RTCFUNC(set, atsama5d3x)(struct tm *tm, int cent_reg)
{
    /* Construct new time */
    unsigned sec  = min(tm->tm_sec, 59);
    unsigned minu = tm->tm_min;
    unsigned hour = tm->tm_hour;
    if(verbose) {
        printf(">> Time >>\n");
        printf("%02u:%02u:%02u\n", hour, minu, sec);
    }
    uint32_t timr = 0;
    timr |= (bin2bcd(sec ) & 0x7F) <<  0;
    timr |= (bin2bcd(minu) & 0x7F) <<  8;
    timr |= (bin2bcd(hour) & 0x3F) << 16;

    /* Construct new date */
    unsigned cent = (tm->tm_year+1900)/100;
    unsigned year = (tm->tm_year)%100;
    unsigned mnth = tm->tm_mon +1;
    unsigned day  = tm->tm_wday+1;
    unsigned date = tm->tm_mday;
    if(verbose) {
        printf(">> Date >>\n");
        printf("%u%02u/%02u/%02u (%u)\n", cent, year, mnth, date, day);
    }
    uint32_t calr = 0;
    calr |= (bin2bcd(cent) & 0x7F) <<  0;
    calr |= (bin2bcd(year) & 0xFF) <<  8;
    calr |= (bin2bcd(mnth) & 0x1F) << 16;
    calr |= (bin2bcd(day ) & 0x07) << 21;
    calr |= (bin2bcd(date) & 0x3F) << 24;

    /* Configure RTC */
    rtc_lock();
    chip_write32(RTC_MR, RTC_MR_EXPECTED);
    chip_write32(RTC_TIMR, timr);
    chip_write32(RTC_CALR, calr);
    rtc_unlock();
    return(0);
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/utils/r/rtc/nto/arm/clk_atsama5d3x.c $ $Rev: 798443 $")
#endif
