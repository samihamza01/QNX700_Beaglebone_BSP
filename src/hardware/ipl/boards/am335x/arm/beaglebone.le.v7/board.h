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

#define BEAGLEBONE

// DDR Operational Frequency
#define OP_FRQ							                400

//*************************************************************************************
//DDR2 PHY parameters
//*************************************************************************************
#define  CMD_PHY_CTRL_SLAVE_RATIO                      0x80
#define  CMD_PHY_INVERT_CLKOUT                         0x0

#define  DATA_PHY_RD_DQS_SLAVE_RATIO                   0x38
#define  DATA_PHY_FIFO_WE_SLAVE_RATIO                  0x94  //RD_DQS_GATE
#define  DATA_PHY_WR_DQS_SLAVE_RATIO                   0x44
#define  DATA_PHY_WR_DATA_SLAVE_RATIO                  0x7D  //WRITE_DATA

#define  DDR_IOCTRL_VALUE                              0x18B

//*************************************************************************************
//EMIF parameters
//*************************************************************************************
#define DDR_READ_LATENCY                              0x100007                  //RD_Latency = (CL + 2) - 1
#define DDR_SDRAM_TIMING1                             0x0AAAD4DB
#define DDR_SDRAM_TIMING2                             0x266B7FDA
#define DDR_SDRAM_TIMING3                             0x501F867F

#define DDR_SDRAM_CONFIG                              0x61C05332		//termination = 1 (75ohm)
                                                                                //dynamic ODT = 2 (RZQ/2)
                                                                                //SDRAM drive = 0 (RZQ/6)
                                                                                //CWL = 0 (CAS write latency = 5)
                                                                                //CL = 4 (CAS latency = 6)
                                                                                //ROWSIZE = 6 (15 row bits)
                                                                                //PAGESIZE = 2 (10 column bits)
#define DDR_REF_CTRL                                  0x00000C30                //266*7.8us = 2074.8 = 0x81A
#define DDR_ZQ_CONFIG                                 0x50074BE4

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/ipl/boards/am335x/arm/beaglebone.le.v7/board.h $ $Rev: 798438 $")
#endif
