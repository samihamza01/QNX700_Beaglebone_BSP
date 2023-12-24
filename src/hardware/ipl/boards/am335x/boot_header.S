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
 * NAND boot header, tell the boot ROM the size and load address of IPL.
 * Should be put at the beginning of NAND.
 * The IPL size is set as 24K bytes, the load address is 0x402f0400
 * adjust the size to suite your need and also consider reduce the size
 * as small as possible to reduce the boot time.
 */


	.text
	.align	2

_IPL_SIZE:		.word	0x00007800
_IPL_ADDRESS:	.word	0x402f0400

#ifdef __QNXNTO__
#ifdef __USESRCVERSION
.section .ident, "SM",%progbits,1;
.asciz "$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/ipl/boards/am335x/boot_header.S $ $Rev: 786744 $";
.previous
#endif
#endif