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

//*************************************************************************************
//PRCM module definitions
//*************************************************************************************
#define PRCM_BASE_ADDR                                  (0x44E00000)
#define CM_PER_EMIF_CLKCTRL                             (PRCM_BASE_ADDR + 0x028)
#define CM_PER_EMIF_FW_CLKCTRL                          (PRCM_BASE_ADDR + 0x0D0)

#define CM_IDLEST_DPLL_MPU                              (PRCM_BASE_ADDR + 0x420)
#define CM_CLKSEL_DPLL_MPU                              (PRCM_BASE_ADDR + 0x42C)
#define CM_IDLEST_DPLL_DDR                              (PRCM_BASE_ADDR + 0x434)
#define CM_CLKSEL_DPLL_DDR                              (PRCM_BASE_ADDR + 0x440)
#define CM_IDLEST_DPLL_DISP                             (PRCM_BASE_ADDR + 0x448)
#define CM_CLKSEL_DPLL_DISP                             (PRCM_BASE_ADDR + 0x454)
#define CM_IDLEST_DPLL_CORE                             (PRCM_BASE_ADDR + 0x45C)
#define CM_CLKSEL_DPLL_CORE                             (PRCM_BASE_ADDR + 0x468)
#define CM_IDLEST_DPLL_PER                              (PRCM_BASE_ADDR + 0x470)
#define CM_CLKSEL_DPLL_PER                              (PRCM_BASE_ADDR + 0x49C)

#define CM_DIV_M4_DPLL_CORE                             (PRCM_BASE_ADDR + 0x480)
#define CM_DIV_M5_DPLL_CORE                             (PRCM_BASE_ADDR + 0x484)
#define CM_CLKMODE_DPLL_MPU                             (PRCM_BASE_ADDR + 0x488)
#define CM_CLKMODE_DPLL_PER                             (PRCM_BASE_ADDR + 0x48C)
#define CM_CLKMODE_DPLL_CORE                            (PRCM_BASE_ADDR + 0x490)
#define CM_CLKMODE_DPLL_DDR                             (PRCM_BASE_ADDR + 0x494)
#define CM_CLKMODE_DPLL_DISP                            (PRCM_BASE_ADDR + 0x498)

#define CM_DIV_M2_DPLL_DDR                              (PRCM_BASE_ADDR + 0x4A0)
#define CM_DIV_M2_DPLL_DISP                             (PRCM_BASE_ADDR + 0x4A4)
#define CM_DIV_M2_DPLL_MPU                              (PRCM_BASE_ADDR + 0x4A8)
#define CM_DIV_M2_DPLL_PER                              (PRCM_BASE_ADDR + 0x4AC)
#define CM_DIV_M6_DPLL_CORE                             (PRCM_BASE_ADDR + 0x4D8)

//*************************************************************************************
//Control module definitions
//*************************************************************************************

#define CONTROL_BASE_ADDR                              (0x44E10000)

#define CONTROL_STATUS                                 (CONTROL_BASE_ADDR + 0x40)
#define CONF_XDMA_EVENT_INTR1                          (CONTROL_BASE_ADDR + 0x9b4)

//DDR IO Control Registers
#define DDR_IO_CTRL                                    (CONTROL_BASE_ADDR + 0x0E04)
#define VTP_CTRL_REG                                   (CONTROL_BASE_ADDR + 0x0E0C)
#define DDR_CKE_CTRL                                   (CONTROL_BASE_ADDR + 0x131C)
#define DDR_CMD0_IOCTRL                                (CONTROL_BASE_ADDR + 0x1404)
#define DDR_CMD1_IOCTRL                                (CONTROL_BASE_ADDR + 0x1408)
#define DDR_CMD2_IOCTRL                                (CONTROL_BASE_ADDR + 0x140C)
#define DDR_DATA0_IOCTRL                               (CONTROL_BASE_ADDR + 0x1440)
#define DDR_DATA1_IOCTRL                               (CONTROL_BASE_ADDR + 0x1444)

//*************************************************************************************
//EMIF4DC module definitions
//*************************************************************************************
 #define EMIF_BASE_ADDR                                (0x4C000000)
 #define EMIF_SDRAM_CONFIG_REG                         (EMIF_BASE_ADDR + 0x008)
 #define EMIF_SDRAM_REF_CTRL_REG                       (EMIF_BASE_ADDR + 0x010)
 #define EMIF_SDRAM_REF_CTRL_SHDW_REG                  (EMIF_BASE_ADDR + 0x014)
 #define EMIF_SDRAM_TIM_1_REG                          (EMIF_BASE_ADDR + 0x018)
 #define EMIF_SDRAM_TIM_1_SHDW_REG                     (EMIF_BASE_ADDR + 0x01C)
 #define EMIF_SDRAM_TIM_2_REG                          (EMIF_BASE_ADDR + 0x020)
 #define EMIF_SDRAM_TIM_2_SHDW_REG                     (EMIF_BASE_ADDR + 0x024)
 #define EMIF_SDRAM_TIM_3_REG                          (EMIF_BASE_ADDR + 0x028)
 #define EMIF_SDRAM_TIM_3_SHDW_REG                     (EMIF_BASE_ADDR + 0x02C)
 #define EMIF_DDR_PHY_CTRL_1_REG                       (EMIF_BASE_ADDR + 0x0E4)
 #define EMIF_DDR_PHY_CTRL_1_SHDW_REG                  (EMIF_BASE_ADDR + 0x0E8)
 #define EMIF_DDR_PHY_CTRL_2_REG                       (EMIF_BASE_ADDR + 0x0EC)
 #define EMIF_ZQ_CONFIG_REG                            (EMIF_BASE_ADDR + 0x0C8)

//*************************************************************************************
//DDR PHY registers
//*************************************************************************************

// //DATA0
#define    DATA0_REG_PHY_RD_DQS_SLAVE_RATIO_0          (0x0C8 + DDR_PHY_BASE_ADDR)
#define    DATA0_REG_PHY_WR_DQS_SLAVE_RATIO_0          (0x0DC + DDR_PHY_BASE_ADDR)
#define    DATA0_REG_PHY_FIFO_WE_SLAVE_RATIO_0         (0x108 + DDR_PHY_BASE_ADDR)
#define    DATA0_REG_PHY_WR_DATA_SLAVE_RATIO_0         (0x120 + DDR_PHY_BASE_ADDR)


extern void init_am335x_ddr (void);

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/ipl/boards/am335x/init_am335x_ddr.h $ $Rev: 798438 $")
#endif
