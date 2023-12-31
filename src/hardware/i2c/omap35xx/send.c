/*
 * $QNXLicenseC:
 * Copyright 2009, QNX Software Systems.
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

#include "proto.h"

i2c_status_t
omap_send(void *hdl, void *buf, unsigned int len, unsigned int stop)
{
    omap_dev_t      *dev = hdl;
    i2c_status_t    ret = I2C_STATUS_ERROR;
    int num_bytes;

    if (len <= 0)
        return I2C_STATUS_DONE;

    if (-1 == omap_wait_bus_not_busy(dev, stop))
        return I2C_STATUS_BUSY;

    omap_clock_enable(dev);

	dev->xlen = len;
	dev->buf = buf;
	dev->status = 0;
	dev->intexpected = 1;
	DEBUG("omap_send: Starting omap_send with len %d for slave 0x%x", dev->xlen, dev->slave_addr);

    /* set slave address */
    if (dev->slave_addr_fmt == I2C_ADDRFMT_7BIT)
        out16(dev->regbase + OMAP_I2C_CON, in16(dev->regbase + OMAP_I2C_CON) & (~OMAP_I2C_CON_XSA));
    else
        out16(dev->regbase + OMAP_I2C_CON, in16(dev->regbase + OMAP_I2C_CON) | OMAP_I2C_CON_XSA);

    out16(dev->regbase + OMAP_I2C_SA, dev->slave_addr);

    /* set data count */
    out16(dev->regbase + OMAP_I2C_CNT, len);

	/* Clear the FIFO Buffers */
	out16(dev->regbase + OMAP_I2C_BUF, in16(dev->regbase + OMAP_I2C_BUF)| OMAP_I2C_BUF_RXFIF_CLR | OMAP_I2C_BUF_TXFIF_CLR);

    /* pre-fill the fifo with outgoing data */
    if (dev->xlen > dev->fifo_size)
      num_bytes = dev->fifo_size;
    else
      num_bytes = dev->xlen;
    while (num_bytes)
    {
      DEBUG("omap_send: Writing byte[%d] = 0x%x to fifo for slave 0x%x", len-dev->xlen, *dev->buf, dev->slave_addr);
      out8(dev->regbase + OMAP_I2C_DATA, *dev->buf++);
      dev->xlen--;
      num_bytes--;
    }

    DEBUG("omap_send: Setting condition for slave 0x%x", dev->slave_addr);
    /* set start condition */
    out16(dev->regbase + OMAP_I2C_CON,
            OMAP_I2C_CON_EN  |
            OMAP_I2C_CON_MST |
            OMAP_I2C_CON_TRX |
            OMAP_I2C_CON_STT |
            (stop? OMAP_I2C_CON_STP : 0)|
            (in16(dev->regbase + OMAP_I2C_CON)&OMAP_I2C_CON_XA));

	ret=  omap_wait_status(dev);
	DUMP_REGS(dev);
	CLEAR_DEBUG;
	omap_clock_disable(dev);
    return ret;
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/i2c/omap35xx/send.c $ $Rev: 756950 $")
#endif
