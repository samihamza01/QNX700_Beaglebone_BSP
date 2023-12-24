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
#include <arm/am335x.h>
#include <hw/inout.h>
#include "board.h"
#include "init_am335x_ddr.h"

void MPU_PLL_Config(unsigned int N,unsigned int M,unsigned int M2)
{
    unsigned int clkmode,clksel,div_m2;

    clkmode=in32(CM_CLKMODE_DPLL_MPU);
    clksel= in32(CM_CLKSEL_DPLL_MPU);
    div_m2= in32(CM_DIV_M2_DPLL_MPU);


    //put the DPLL in bypass mode
    out32(CM_CLKMODE_DPLL_MPU,0x4);

    while(((in32(CM_IDLEST_DPLL_MPU) & 0x101) != 0x00000100)); //wait for bypass status

    //set multiply and divide values
    clksel = clksel & (~0x7FFFF);
    clksel = clksel | ((M <<0x8) | N);
    out32(CM_CLKSEL_DPLL_MPU,clksel);
    div_m2 = div_m2 & ~0x1F;
    div_m2 = div_m2 | M2;
    out32(CM_DIV_M2_DPLL_MPU,div_m2);

    //now lock the DPLL
    clkmode = clkmode | 0x7;  //enables lock mode
    out32(CM_CLKMODE_DPLL_MPU,clkmode);
    while(((in32(CM_IDLEST_DPLL_MPU) & 0x101) != 0x1)); //wait for lock
}


void CORE_PLL_Config(unsigned int N,unsigned int M,unsigned int M4,unsigned int M5,unsigned int M6)
{
    unsigned int clkmode,clksel,div_m4,div_m5,div_m6;

    clkmode=in32(CM_CLKMODE_DPLL_CORE);
    clksel= in32(CM_CLKSEL_DPLL_CORE);
    div_m4= in32(CM_DIV_M4_DPLL_CORE);
    div_m5= in32(CM_DIV_M5_DPLL_CORE);
    div_m6= in32(CM_DIV_M6_DPLL_CORE);

    //put DPLL in bypass mode
    clkmode = (clkmode & 0xfffffff8)|0x00000004;
    out32(CM_CLKMODE_DPLL_CORE,clkmode);
    while((in32(CM_IDLEST_DPLL_CORE) & 0x00000100 )!=0x00000100); //wait for bypass status


    //set multiply and divide values
    clksel = clksel & (~0x7FFFF);
    clksel = clksel | ((M <<0x8) | N);
    out32(CM_CLKSEL_DPLL_CORE,clksel);
    div_m4= M4;   //200MHz
    out32(CM_DIV_M4_DPLL_CORE,div_m4);
    div_m5=  M5;  //250MHz
    out32(CM_DIV_M5_DPLL_CORE,div_m5);
    div_m6=  M6;  //500MHz
    out32(CM_DIV_M6_DPLL_CORE,div_m6);


    //now lock the PLL
    clkmode =(clkmode&0xfffffff8)|0x00000007;
    out32(CM_CLKMODE_DPLL_CORE,clkmode);

    while((in32(CM_IDLEST_DPLL_CORE) & 0x00000001 )!=0x00000001);
}

void DDR_PLL_Config(unsigned int N,unsigned int M,unsigned int M2)
{
    unsigned int clkmode,clksel,div_m2;

    clkmode=in32(CM_CLKMODE_DPLL_DDR);
    clksel= in32(CM_CLKSEL_DPLL_DDR);
    div_m2= in32(CM_DIV_M2_DPLL_DDR);

    clkmode =(clkmode&0xfffffff8)|0x00000004;
    out32(CM_CLKMODE_DPLL_DDR,clkmode);
    while((in32(CM_IDLEST_DPLL_DDR) & 0x00000100 )!=0x00000100);


    clksel = clksel & (~0x7FFFF);
    clksel = clksel | ((M <<0x8) | N);
    out32(CM_CLKSEL_DPLL_DDR,clksel);
    div_m2 = in32(CM_DIV_M2_DPLL_DDR);
    div_m2 = (div_m2&0xFFFFFFE0) | M2;
    out32(CM_DIV_M2_DPLL_DDR,div_m2);
    clkmode =(clkmode&0xfffffff8)|0x00000007;
    out32(CM_CLKMODE_DPLL_DDR,clkmode);


    while((in32(CM_IDLEST_DPLL_DDR) & 0x00000001 )!=0x00000001);

}

void PER_PLL_Config(unsigned int N,unsigned int M,unsigned int M2)
{
    unsigned int clkmode,clksel,div_m2;

    clkmode=in32(CM_CLKMODE_DPLL_PER);
    clksel= in32(CM_CLKSEL_DPLL_PER);
    div_m2= in32(CM_DIV_M2_DPLL_PER);

    clkmode =(clkmode&0xfffffff8)|0x00000004;
    out32(CM_CLKMODE_DPLL_PER,clkmode);
    while((in32(CM_IDLEST_DPLL_PER) & 0x00000100 )!=0x00000100);

    clksel = clksel & (~0xFF0FFFFF);
    clksel = clksel | 0x04000000;  //SD divider = 4 for both OPP100 and OPP50
    clksel = clksel | ((M <<0x8) | N);
    out32(CM_CLKSEL_DPLL_PER,clksel);
    div_m2= 0xFFFFFF80 | M2;
    out32(CM_DIV_M2_DPLL_PER,div_m2);
    clkmode =(clkmode&0xfffffff8)|0x00000007;
    out32(CM_CLKMODE_DPLL_PER,clkmode);

    while((in32(CM_IDLEST_DPLL_PER) & 0x00000001 )!=0x00000001);
}

void DISP_PLL_Config(unsigned int N,unsigned int M,unsigned int M2)
{
    unsigned int clkmode,clksel,div_m2;

    clkmode=in32(CM_CLKMODE_DPLL_DISP);
    clksel= in32(CM_CLKSEL_DPLL_DISP);
    div_m2= in32(CM_DIV_M2_DPLL_DISP);

    clkmode =(clkmode&0xfffffff8)|0x00000004;
    out32(CM_CLKMODE_DPLL_DISP,clkmode);
    while((in32(CM_IDLEST_DPLL_DISP) & 0x00000100 )!=0x00000100);

    clksel = clksel & (~0x7FFFF);
    clksel = clksel | ((M <<0x8) | N);
    out32(CM_CLKSEL_DPLL_DISP,clksel);
    div_m2= 0xFFFFFFE0 | M2;
    out32(CM_DIV_M2_DPLL_DISP,div_m2);
    clkmode =(clkmode&0xfffffff8)|0x00000007;
    out32(CM_CLKMODE_DPLL_DISP,clkmode);

    while((in32(CM_IDLEST_DPLL_DISP) & 0x00000001 )!=0x00000001);
}


void PHY_Config_CMD()
{
    unsigned int i=0;
    for(i=0;i<3;i++)
       {
          out32(CMD0_CTRL_SLAVE_RATIO_0 + (i*0x34),CMD_PHY_CTRL_SLAVE_RATIO);
          out32(CMD0_INVERT_CLKOUT_0    + (i*0x34),CMD_PHY_INVERT_CLKOUT);
       }
}

void PHY_Config_DATA()
{
    unsigned int i;
    for(i=0;i<2;i++)
    {
        out32(DATA0_REG_PHY_RD_DQS_SLAVE_RATIO_0     + (i*0xA4),DATA_PHY_RD_DQS_SLAVE_RATIO);
        out32(DATA0_REG_PHY_WR_DQS_SLAVE_RATIO_0     + (i*0xA4),DATA_PHY_WR_DQS_SLAVE_RATIO);
        out32(DATA0_REG_PHY_FIFO_WE_SLAVE_RATIO_0    + (i*0xA4),DATA_PHY_FIFO_WE_SLAVE_RATIO);
        out32(DATA0_REG_PHY_WR_DATA_SLAVE_RATIO_0    + (i*0xA4),DATA_PHY_WR_DATA_SLAVE_RATIO);
    }
}

void EMIF_PRCM_CLK_ENABLE()
{
    /* Enable EMIF4 clocks*/
    out32(CM_PER_EMIF_CLKCTRL,0x02);
    /* Poll if module is functional */
    while(in32(CM_PER_EMIF_CLKCTRL)!= 0x02);
}

void VTP_Enable()
{
      //clear the register
      out32(VTP_CTRL_REG ,0x0);
      //set filter bits to 011b
      out32(VTP_CTRL_REG ,0x6);
      //Write 1 to enable VTP
      out32(VTP_CTRL_REG ,(in32(VTP_CTRL_REG) | 0x00000040));
      //Write 0 to CLRZ bit
      out32(VTP_CTRL_REG ,(in32(VTP_CTRL_REG) & 0xFFFFFFFE));
      //Write 1 to CLRZ bit
      out32(VTP_CTRL_REG ,(in32(VTP_CTRL_REG) | 0x00000001));
     //Check for VTP ready bit
      while((in32(VTP_CTRL_REG) & 0x00000020) != 0x00000020);
}

void init_am335x_ddr()
{
    unsigned int temp = in32(CONTROL_STATUS) >> 22;
    if(temp == 1) // Implies CLKIN = 24MHz
    {
       MPU_PLL_Config(  23, 500,    1);
       CORE_PLL_Config( 23, 1000,   10, 8, 4);
       DDR_PLL_Config(  23, OP_FRQ, 1);
       PER_PLL_Config(  23, 960,    5);
       DISP_PLL_Config( 23, 48,     1);
    }
    else
    {
        ser_putstr("CLKIN is not 24MHz\n");
    }

    EMIF_PRCM_CLK_ENABLE();

    VTP_Enable();

    PHY_Config_CMD();
    PHY_Config_DATA();

    out32(DDR_CMD0_IOCTRL,DDR_IOCTRL_VALUE);
    out32(DDR_CMD1_IOCTRL,DDR_IOCTRL_VALUE);
    out32(DDR_CMD2_IOCTRL,DDR_IOCTRL_VALUE);
    out32(DDR_DATA0_IOCTRL,DDR_IOCTRL_VALUE);
    out32(DDR_DATA1_IOCTRL,DDR_IOCTRL_VALUE);

    //IO to work for DDR2
    out32(DDR_IO_CTRL, in32(DDR_IO_CTRL) & ~0x10000000 );

    //CKE controlled by EMIF/DDR_PHY
    out32(DDR_CKE_CTRL, in32(DDR_CKE_CTRL) | 0x00000001);

    out32(EMIF_DDR_PHY_CTRL_1_REG, DDR_READ_LATENCY);
    out32(EMIF_DDR_PHY_CTRL_1_SHDW_REG, DDR_READ_LATENCY);
    out32(EMIF_DDR_PHY_CTRL_2_REG, DDR_READ_LATENCY);

    out32(EMIF_SDRAM_TIM_1_REG,DDR_SDRAM_TIMING1);
    out32(EMIF_SDRAM_TIM_1_SHDW_REG,DDR_SDRAM_TIMING1);

    out32(EMIF_SDRAM_TIM_2_REG,DDR_SDRAM_TIMING2);
    out32(EMIF_SDRAM_TIM_2_SHDW_REG,DDR_SDRAM_TIMING2);

    out32(EMIF_SDRAM_TIM_3_REG,DDR_SDRAM_TIMING3);
    out32(EMIF_SDRAM_TIM_3_SHDW_REG,DDR_SDRAM_TIMING3);

    out32(EMIF_SDRAM_REF_CTRL_REG,DDR_REF_CTRL);
    out32(EMIF_SDRAM_REF_CTRL_SHDW_REG,DDR_REF_CTRL);

    out32(EMIF_SDRAM_CONFIG_REG, DDR_SDRAM_CONFIG);

#ifdef BEAGLEBONE
    out32(EMIF_ZQ_CONFIG_REG, DDR_ZQ_CONFIG);
#endif

}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/ipl/boards/am335x/init_am335x_ddr.c $ $Rev: 798438 $")
#endif
