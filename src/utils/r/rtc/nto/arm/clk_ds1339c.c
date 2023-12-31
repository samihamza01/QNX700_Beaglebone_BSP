/*
 * $QNXLicenseC:
 * Copyright 2014, QNX Software Systems.
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
#include <fcntl.h>
#include <hw/i2c.h>

/*
 * Dallas/Maxim DS1339C Serial RTC
 */
#define DS1339C_SEC          0   /* 00-59 */
#define DS1339C_MIN          1   /* 00-59 */
#define DS1339C_HOUR         2   /* 0-1/00-23 */
#define DS1339C_DAY          3   /* 01-07 */
#define DS1339C_DATE         4   /* 01-31 */
#define DS1339C_MONTH        5   /* 01-12 */
#define DS1339C_YEAR         6   /* 00-99 */
#define DS1339C_CTRL         7

#define DS1339C_I2C_ADDRESS  (0x68)
#define DS1339C_I2C_DEVNAME  "/dev/i2c0"

static int fd = -1;

static int
ds1339c_i2c_read(unsigned char reg, unsigned char val[], unsigned char num)
{
    iov_t           siov[2], riov[2];
    i2c_sendrecv_t  hdr;

    hdr.slave.addr = DS1339C_I2C_ADDRESS;
    hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    hdr.send_len = 1;
    hdr.recv_len = num;
    hdr.stop = 1;

    SETIOV(&siov[0], &hdr, sizeof(hdr));
    SETIOV(&siov[1], &reg, sizeof(reg));

    SETIOV(&riov[0], &hdr, sizeof(hdr));
    SETIOV(&riov[1], val, num);

    return devctlv(fd, DCMD_I2C_SENDRECV, 2, 2, siov, riov, NULL);
}

static int
ds1339c_i2c_write(unsigned char reg, unsigned char val[], unsigned char num)
{
    iov_t           siov[3];
    i2c_send_t      hdr;

    hdr.slave.addr = DS1339C_I2C_ADDRESS;
    hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    hdr.len = num + 1;
    hdr.stop = 1;

    SETIOV(&siov[0], &hdr, sizeof(hdr));
    SETIOV(&siov[1], &reg, sizeof(reg));
    SETIOV(&siov[2], val, num);

    return devctlv(fd, DCMD_I2C_SEND, 3, 0, siov, NULL, NULL);
}

int
RTCFUNC(init,ds1339c)(struct chip_loc *chip, char *argv[])
{
    fd = open((argv && argv[0] && argv[0][0])?
            argv[0]: DS1339C_I2C_DEVNAME, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Unable to open I2C device\n");
        return -1;
    }
    return 0;
}

int
RTCFUNC(get,ds1339c)(struct tm *tm, int cent_reg)
{
    unsigned char   date[7];

    ds1339c_i2c_read(DS1339C_SEC, date, 7);

    tm->tm_sec  = BCD2BIN(date[DS1339C_SEC]);
    tm->tm_min  = BCD2BIN(date[DS1339C_MIN]);
    tm->tm_hour = BCD2BIN(date[DS1339C_HOUR]);
    tm->tm_mday = BCD2BIN(date[DS1339C_DATE]);
    tm->tm_mon  = BCD2BIN(date[DS1339C_MONTH]) - 1;
    tm->tm_year = BCD2BIN(date[DS1339C_YEAR]);
    tm->tm_wday = BCD2BIN(date[DS1339C_DAY]) - 1;

    if(tm->tm_year < 70)
        tm->tm_year += 100;

    return(0);
}

int
RTCFUNC(set,ds1339c)(struct tm *tm, int cent_reg)
{
    unsigned char   date[7];

    date[DS1339C_SEC]   = BIN2BCD(tm->tm_sec);
    date[DS1339C_MIN]   = BIN2BCD(tm->tm_min);
    date[DS1339C_HOUR]  = BIN2BCD(tm->tm_hour);
    date[DS1339C_DATE]  = BIN2BCD(tm->tm_mday);
    date[DS1339C_MONTH] = BIN2BCD(tm->tm_mon + 1);
    date[DS1339C_YEAR]  = BIN2BCD(tm->tm_year % 100);
    date[DS1339C_DAY]   = BIN2BCD(tm->tm_wday + 1);

    ds1339c_i2c_write(DS1339C_SEC, date, 7);

    return(0);

}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/utils/r/rtc/nto/arm/clk_ds1339c.c $ $Rev: 798443 $")
#endif
