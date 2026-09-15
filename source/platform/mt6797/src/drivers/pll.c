/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein is
 * confidential and proprietary to MediaTek Inc. and/or its licensors. Without
 * the prior written permission of MediaTek inc. and/or its licensors, any
 * reproduction, modification, use or disclosure of MediaTek Software, and
 * information contained herein, in whole or in part, shall be strictly
 * prohibited.
 *
 * MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER
 * ON AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL
 * WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR
 * NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH
 * RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 * INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES
 * TO LOOK ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO.
 * RECEIVER EXPRESSLY ACKNOWLEDGES THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO
 * OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES CONTAINED IN MEDIATEK
 * SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE
 * RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S
 * ENTIRE AND CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE
 * RELEASED HEREUNDER WILL BE, AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE
 * MEDIATEK SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE
 * CHARGE PAID BY RECEIVER TO MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek
 * Software") have been modified by MediaTek Inc. All revisions are subject to
 * any receiver's applicable license agreements with MediaTek Inc.
 */

#include "typedefs.h"
#include "platform.h"

#include "pll.h"
#include "timer.h"
#include "spm.h"
#include "spm_mtcmos.h"

#include "dramc_pi_api.h"
#include "dramc_common.h"
#include "dramc_register.h"
#include "wdt.h"
#include "emi.h"

#include "../security/inc/sec_devinfo.h"



#if 1

#define _FREQ_METER_DIV_
#define PERI_DVFS0_TX_APB_ASYNC_STA 0x10003420
#define PERI_DVFS1_TX_APB_ASYNC_STA 0x10003424

unsigned int mt_get_cpu_freq(void)
{
	int output = 0, i = 0;
    unsigned int temp, clk26cali_0, clk_dbg_cfg, clk_misc_cfg_0, clk26cali_1;

    clk_dbg_cfg = DRV_Reg32(CLK_DBG_CFG);
    DRV_WriteReg32(CLK_DBG_CFG, (clk_dbg_cfg & 0xFFFFFFFC)|(42 << 16)); //sel abist_cksw and enable freq meter sel abist

	#ifdef _FREQ_METER_DIV_
    clk_misc_cfg_0 = DRV_Reg32(CLK_MISC_CFG_0);
    DRV_WriteReg32(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0x00FFFFFF)); // select divider, WAIT CONFIRM
	#endif

    clk26cali_1 = DRV_Reg32(CLK26CALI_1);
    //DRV_WriteReg32(CLK26CALI_1, 0x00ff0000); // cycle count default 1024,[25:16]=3FF

    //temp = DRV_Reg32(CLK26CALI_0);
    DRV_WriteReg32(CLK26CALI_0, 0x1000);
    DRV_WriteReg32(CLK26CALI_0, 0x1010);

    /* wait frequency meter finish */
    while (DRV_Reg32(CLK26CALI_0) & 0x10)
    {
        mdelay(10);
        i++;
        if(i > 10)
        	break;
    }

    temp = DRV_Reg32(CLK26CALI_1) & 0xFFFF;

    output = ((temp * 26000) ) / 1024; // Khz

    DRV_WriteReg32(CLK_DBG_CFG, clk_dbg_cfg);
    DRV_WriteReg32(CLK_MISC_CFG_0, clk_misc_cfg_0);
    DRV_WriteReg32(CLK26CALI_0, clk26cali_0);
    DRV_WriteReg32(CLK26CALI_1, clk26cali_1);

    if(i>10)
        return 0;
    else
        return output;
}


unsigned int mt_get_abist_freq(unsigned int ID)
{
	int output = 0, i = 0;
    unsigned int temp, clk26cali_0, clk_dbg_cfg, clk_misc_cfg_0, clk26cali_1;

    clk_dbg_cfg = DRV_Reg32(CLK_DBG_CFG);
    DRV_WriteReg32(CLK_DBG_CFG, (clk_dbg_cfg & 0xFFFFFFFC)|(ID << 16)); //sel abist_cksw and enable freq meter sel abist

	#ifdef _FREQ_METER_DIV_
    clk_misc_cfg_0 = DRV_Reg32(CLK_MISC_CFG_0);
    DRV_WriteReg32(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0x00FFFFFF)); // select divider, WAIT CONFIRM
	#endif

    clk26cali_1 = DRV_Reg32(CLK26CALI_1);
    //DRV_WriteReg32(CLK26CALI_1, 0x00ff0000); // cycle count default 1024,[25:16]=3FF

    //temp = DRV_Reg32(CLK26CALI_0);
    DRV_WriteReg32(CLK26CALI_0, 0x1000);
    DRV_WriteReg32(CLK26CALI_0, 0x1010);

    /* wait frequency meter finish */
    while (DRV_Reg32(CLK26CALI_0) & 0x10)
    {
        mdelay(10);
        i++;
        if(i > 10)
        	break;
    }

    temp = DRV_Reg32(CLK26CALI_1) & 0xFFFF;

    output = ((temp * 26000) ) / 1024; // Khz

    DRV_WriteReg32(CLK_DBG_CFG, clk_dbg_cfg);
    DRV_WriteReg32(CLK_MISC_CFG_0, clk_misc_cfg_0);
    DRV_WriteReg32(CLK26CALI_0, clk26cali_0);
    DRV_WriteReg32(CLK26CALI_1, clk26cali_1);

    //print("CA15 freq = %d\n", output);

    if(i>10)
        return 0;
    else
        return output;
}


unsigned int mt_get_mpll_freq(void)
{
    int output = 0, i = 0;
    unsigned int temp, clk26cali_0, clk_dbg_cfg, clk_misc_cfg_0, clk26cali_1;

    clk_dbg_cfg = DRV_Reg32(CLK_DBG_CFG);
    DRV_WriteReg32(CLK_DBG_CFG, (clk_dbg_cfg & 0xFFFFFFFC)|(41 << 16)); //sel abist_cksw and enable abist meter sel abist

	#ifdef _FREQ_METER_DIV_
    clk_misc_cfg_0 = DRV_Reg32(CLK_MISC_CFG_0);
    DRV_WriteReg32(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0x00FFFFFF)); // select divider
	#endif

    clk26cali_1 = DRV_Reg32(CLK26CALI_1);
    //DRV_WriteReg32(CLK26CALI_1, 0x00ff0000); //

    //temp = DRV_Reg32(CLK26CALI_0);
    DRV_WriteReg32(CLK26CALI_0, 0x1000);
    DRV_WriteReg32(CLK26CALI_0, 0x1010);

    /* wait frequency meter finish */
    while (DRV_Reg32(CLK26CALI_0)& 0x10)
    {
        mdelay(10);
		i++;
        if(i > 10)
        	break;
    }

    temp = DRV_Reg32(CLK26CALI_1) & 0xFFFF;

    output = ((temp * 26000) ) / 1024; // Khz

    DRV_WriteReg32(CLK_DBG_CFG, clk_dbg_cfg);
    DRV_WriteReg32(CLK_MISC_CFG_0, clk_misc_cfg_0);
    DRV_WriteReg32(CLK26CALI_0, clk26cali_0);
    DRV_WriteReg32(CLK26CALI_1, clk26cali_1);

	if(i>10)
        return 0;
    else
        return output;

}


unsigned int mt_get_mem_freq(void)
{
    int output = 0, i = 0;
    unsigned int temp, clk26cali_0, clk_dbg_cfg, clk_misc_cfg_0, clk26cali_1;

    clk_dbg_cfg = DRV_Reg32(CLK_DBG_CFG);
    DRV_WriteReg32(CLK_DBG_CFG, (clk_dbg_cfg & 0xFFFFFFFC)|(23 << 16)); //sel abist_cksw and enable abist meter sel abist

	#ifdef _FREQ_METER_DIV_
    clk_misc_cfg_0 = DRV_Reg32(CLK_MISC_CFG_0);
    DRV_WriteReg32(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0x00FFFFFF)); // select divider
	#endif

    clk26cali_1 = DRV_Reg32(CLK26CALI_1);
    //DRV_WriteReg32(CLK26CALI_1, 0x00ff0000); //

    //temp = DRV_Reg32(CLK26CALI_0);
    DRV_WriteReg32(CLK26CALI_0, 0x1000);
    DRV_WriteReg32(CLK26CALI_0, 0x1010);

    /* wait frequency meter finish */
    while (DRV_Reg32(CLK26CALI_0)& 0x10)
    {
        //print("wait for frequency meter finish, CLK26CALI = 0x%x\n", DRV_Reg32(CLK26CALI_0));
        mdelay(10);
		i++;
        if(i > 10)
        	break;
    }

    temp = DRV_Reg32(CLK26CALI_1) & 0xFFFF;

    output = ((temp * 26000) ) / 1024; // Khz

    DRV_WriteReg32(CLK_DBG_CFG, clk_dbg_cfg);
    DRV_WriteReg32(CLK_MISC_CFG_0, clk_misc_cfg_0);
    DRV_WriteReg32(CLK26CALI_0, clk26cali_0);
    DRV_WriteReg32(CLK26CALI_1, clk26cali_1);

    //print("CLK26CALI = 0x%x, mem bus frequency = %d Khz\n", temp, output);
	if(i>10)
        return 0;
    else
        return output;

}

static unsigned int mt_get_pll_freq(unsigned int ID)
{
    int output = 0, i = 0;
    unsigned int temp, clk26cali_0, clk_dbg_cfg, clk_misc_cfg_0, clk26cali_1;

    clk_dbg_cfg = DRV_Reg32(CLK_DBG_CFG);
    DRV_WriteReg32(CLK_DBG_CFG, (ID << 8) | 0x01); //sel ckgen_cksw[0] and enable freq meter sel ckgen[13:8], 01:hd_faxi_ck

	#ifdef _FREQ_METER_DIV_
    clk_misc_cfg_0 = DRV_Reg32(CLK_MISC_CFG_0);
    //DRV_WriteReg32(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0x00FFFFFF) | (0x07 << 24)); // select divider?dvt set zero
    DRV_WriteReg32(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0x00FFFFFF)); // select divider?dvt set zero
	#endif

    clk26cali_1 = DRV_Reg32(CLK26CALI_1);
    //DRV_WriteReg32(CLK26CALI_1, 0x00ff0000); //

    //temp = DRV_Reg32(CLK26CALI_0);
    DRV_WriteReg32(CLK26CALI_0, 0x1000);
    DRV_WriteReg32(CLK26CALI_0, 0x1010);

    /* wait frequency meter finish */
    while (DRV_Reg32(CLK26CALI_0) & 0x10)
    {
        //print("wait for frequency meter finish, CLK26CALI = 0x%x\n", DRV_Reg32(CLK26CALI_0));
        mdelay(10);
		i++;
        if(i > 10)
        	break;
    }

    temp = DRV_Reg32(CLK26CALI_1) & 0xFFFF;

    output = ((temp * 26000) ) / 1024; // Khz

    DRV_WriteReg32(CLK_DBG_CFG, clk_dbg_cfg);
    DRV_WriteReg32(CLK_MISC_CFG_0, clk_misc_cfg_0);
    DRV_WriteReg32(CLK26CALI_0, clk26cali_0);
    DRV_WriteReg32(CLK26CALI_1, clk26cali_1);

    //print("CLK26CALI = 0x%x, frequency meter[%d] = %d Khz\n", temp, ID, output);

	if(i>10)
        return 0;
    else
        return output;

}

unsigned int mt_get_bus_freq(void)
{
    int output = 0, i = 0;
    unsigned int temp, clk26cali_0, clk_dbg_cfg, clk_misc_cfg_0, clk26cali_1;

    clk_dbg_cfg = DRV_Reg32(CLK_DBG_CFG);
    DRV_WriteReg32(CLK_DBG_CFG, (1 << 8) | 0x01); //sel abist_cksw and enable freq meter sel abist

	#ifdef _FREQ_METER_DIV_
    clk_misc_cfg_0 = DRV_Reg32(CLK_MISC_CFG_0);
    DRV_WriteReg32(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0x00FFFFFF)); // select divider
	#endif

    clk26cali_1 = DRV_Reg32(CLK26CALI_1);
    //DRV_WriteReg32(CLK26CALI_1, 0x00ff0000); //

    //temp = DRV_Reg32(CLK26CALI_0);
    DRV_WriteReg32(CLK26CALI_0, 0x1000);
    DRV_WriteReg32(CLK26CALI_0, 0x1010);

    /* wait frequency meter finish */
    while (DRV_Reg32(CLK26CALI_0) & 0x10)
    {
        mdelay(10);
		i++;
        if(i > 10)
        	break;
    }

    temp = DRV_Reg32(CLK26CALI_1) & 0xFFFF;

    output = ((temp * 26000) ) / 1024; // Khz

    DRV_WriteReg32(CLK_DBG_CFG, clk_dbg_cfg);
    DRV_WriteReg32(CLK_MISC_CFG_0, clk_misc_cfg_0);
    DRV_WriteReg32(CLK26CALI_0, clk26cali_0);
    DRV_WriteReg32(CLK26CALI_1, clk26cali_1);

    return output;
}
#endif

#define _RESULT_COMPARE_

#define Range 1000

const char *ckgen_golden_array[] =
{
    156000, 109000, 104000, 500000, 450000,
	48000, 26000, 109000, 273000, 384000,
	192000, 	52000, 	26000, 	26000, 136500,
	26000, 	364000, 	273000, 	450000, 	148500,
	180600, 	196500, 	273000, 	500500, 	78000,
	494000, 	125000, 	68250, 	26000, 	136500,
	196500
};

const char *ckgen_array[] =
{
    "hd_faxi_ck", "hf_fddrphycfg_ck", "f_fpwm_ck", "hf_fvdec_ck", "hf_fmm_ck",
	"hf_fcamtg_ck", "f_fuart_ck", "hf_fspi_ck", "hf_fmsdc50_0_h_ck", "hf_fmsdc50_0_ck",
	"hf_fmsdc30_1_ck", 	"hf_fmsdc30_2_ck", 	"f52m_mfg_ck", 	"hf_faudio_ck", "hf_faud_intbus_ck",
	"hf_fpmicspi_ck", 	"hf_fscp_ck", 	"hf_fatb_ck", 	"hf_fmjc_ck", 	"hf_fdpi0_ck",
	"hf_faud_1_ck", 	"hf_faud_2_ck", 	"hf_fanc_md32_ck", 	"hf_fmfg_ck", 	"f_fusb20_ck",
	"hf_fenc_ck", 	"f_fssusb_top_sys_ck", 	"hg_fspm_ck", 	"fmem_ck", 	"hf_fbsi_spi_ck",
	"hf_faudio_h_ck"
};

#if 1
const int ckgen_abist_needCheck_array[] =
{
/* 1 - 5*/	0,	0, 	0,	1,	1,
/* 6 - 10*/	1,	1,	1,	1,	1,
/* 11 - 15*/	1,  1,	1,	1,	1,
/* 16 - 20*/	0,  1,	1,	1,	1,
/* 21 - 25*/	1,	1,  0,	0,	0,
/* 26 - 30*/	0,	0, 	0,	0,	0,
/* 31 - 35*/	0,	0,	0,	1,  1,
/* 36 - 40*/	1, 	1,  0,  0,  0,
/* 41 - 45*/	0,  1,	1,	1,	1,
/* 46 - 50*/	1,	1,  0,	0,	0,
/* 51 - 55*/	0,	0,	0,	0,	0,
/* 56 - 60*/	0,	0,	0,  0,	0,
/* 61 - 63*/	0,	0,	0,
};

/* if needCheck_array[x]=1 but golden_array[x]=0: means we only want to read the value*/
const int ckgen_abist_golden_array[] =
{
/* 1 - 5*/	0,						0, 	0,	546000,	364000,
/* 6 - 10*/	218400,		156000,	178300,	48000,		624000,
/* 11 - 15*/	416000, 249600,	180633,	196608,	26000,
/* 16 - 20*/	0, 500000,	450000,	594000,	494000,
/* 21 - 25*/	384000,	48000, 0,	0,	0,
/* 26 - 30*/	0,	0, 	0,	0,	0,
/* 31 - 35*/	0,	0,	0,	897000, 1274000,    /*35: change 1274000 ->  0*/
/* 36 - 40*/	630000, 0, 0, 0, 0,
/* 41 - 45*/	0, 897000,	1274000,	630000,	1000000, /*43: change 1274000 ->  0*/
/* 46 - 50*/	0,	48000, 0,	0,	0,
/* 51 - 55*/	0,	0,	0,	0,	0,
/* 56 - 60*/	0,	0,	0, 0,	0,
/* 61 - 63*/	0,	0,	0,
};

const char *ckgen_abist_array[] =
{
	"b0",                   	"AD_OSC_CK_abist_mon ",   	"AD_OSC_CK_D3 ",   	"AD_MAIN_H546M_CK ",  	"AD_MAIN_H364M_CK ",
	"AD_MAIN_H218P4M_CK ",      "AD_MAIN_H156M_CK ",   	"AD_UNIV_178P3M_CK ",  	"AD_UNIV_48M_CK ",     	"AD_UNIV_624M_CK ",
	"AD_UNIV_416M_CK ",	"AD_UNIV_249P6M_CK ",	"AD_APLL1_180P6336M_CK ",	"AD_APLL2_196P608M_CK ",	"AD_MDPLL1_26M_CK ",
	"rtc32k_ck_i ",	"AD_MFGPLL_500M_CK ",	"AD_IMGPLL_450M_CK ",	"AD_TVDPLL_594M_CK ",	"AD_CODECPLL_494M_CK ",
	"AD_MSDCPLL_400M_CK ",	"AD_USB20_48M_CK ",	"AD_MEMPLL_MONCLK ",	"MIPI0_DSI_TST_CK ",	"AD_PLLGP_TST_CK ",
	"AD_MCUPLLGP_TSTDIV2_CK ", 	"fmem_ck ",  	"clkm_ck_out_0 ", 	"clkm_ck_out_1 ",  	"clkm_ck_out_2 ",
	"clkm_ck_out_3 ",	"clkm_ck_out_4 ",	"clkm_ck_out_5 ",	"armpll_arm_clk_out1 ",	"armpll_arm_clk_out2 ",
	"armpll_arm_clk_out3 ", 	"armpll_arm_clk_out0 ",	"1b0",	"1b0",	"AD_VDECPLL_500M_CK ",
	"AD_MPLL_104M_CK ",	"AD_ARMCAXPLL1_CK_MON ",	"AD_ARMCAXPLL2_CK_MON ",	"AD_ARMCAXPLL3_CK_MON ",	"AD_ARMCAXPLL4_CK_MON ",
	"AD_ARMCAXPLL0_CK_MON ",	"AD_SSUSB_48M_CK ",	"AD_CA_RPHYPLL_DIV4_CK ",	"AD_CA_RCLKRPLL_DIV4_CK ",	"AD_CB_RPHYPLL_DIV4_CK ",
	"AD_CB_RCLKRPLL_DIV4_CK ",	"AD_AB_RPHYPLL_DIV4_CK ",	"AD_AB_RCLKRPLL_DIV4_CK ",	"AD_CSI0A_DELAYCAL_CK_mux ",	"AD_CSI0B_DELAYCAL_CK_mux ",
	"AD_CSI1A_DELAYCAL_CK_mux ",	"AD_CSI1B_DELAYCAL_CK_mux ",	"AD_CSI2_DELAYCAL_CK_mux ",	"MIPI1_DSI_TST_CK ",	"AD_MDPLLGP_TSTDIV2_CK ",
	"AD_CA_RPHYPLL_TSTDIV2_CK ",	"AD_CB_RPHYPLL_TSTDIV2_CK ",	"AD_AB_RPHYPLL_TSTDIV2_CK ",
};
#endif


unsigned int ret_feq_store[100];
unsigned int ret_feq_total=0;
unsigned int pll_chk_is_fail = 0;

//after pmic_init
//#define _FREQ_SCAN_
void mt_pll_post_init(void)
{
	unsigned int temp,clk_misc_cfg_0,ret_feq,reg_temp;
	unsigned int isFail = 0;

	#ifdef _FREQ_SCAN_
	print("Pll post init start...\n");
	//print("mt_pll_post_init: mt_get_bus_freq (AXI)= %dKhz\n", mt_get_bus_freq());
	//print("mt_pll_post_init: mt_get_cpu_freq (ARMCAXPLL1) = %dKhz\n", mt_get_cpu_freq());
	//print("mt_pll_post_init: mt_get_mem_freq (MEMPLL) = %dKhz\n", mt_get_mem_freq());
	//print("mt_pll_post_init: mt_get_mpll_freq (MPLL) = %dKhz\n", mt_get_mpll_freq());

	print("==============================\n");
	print("==      Parsing Start       ==\n");
	print("==============================\n");
	for(temp=1; temp<32; temp++)
	{
		ret_feq = 0;
		if((temp==20))
		{
		  	//f52m_mfg_ck
		  	clk_misc_cfg_0 = DRV_Reg32(CLK_MISC_CFG_0);
			DRV_WriteReg32(CLK_MISC_CFG_0, (clk_misc_cfg_0 | 0x1)); //disable clk

			clk_misc_cfg_0 = DRV_Reg32(CLK_MISC_CFG_0);
			DRV_WriteReg32(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0xFFFFFFF2)); //select 52M

			clk_misc_cfg_0 = DRV_Reg32(CLK_MISC_CFG_0);
			DRV_WriteReg32(CLK_MISC_CFG_0, (clk_misc_cfg_0 & 0xFFFFFFFE)); //select 52M
		}

		ret_feq = mt_get_pll_freq(temp);
		ret_feq_store[ret_feq_total] = ret_feq;
		ret_feq_total++;
		//print("%s:", ckgen_array[temp-1]);
		print("%d",ret_feq);
#ifdef _RESULT_COMPARE_
		if((ret_feq < (ckgen_golden_array[temp-1] - Range)) || (ret_feq > (ckgen_golden_array[temp-1] + Range)))
		{
			print(" : ERROR");
			isFail = 1;
		}
#endif
		printf("\n");

	}

	//abist 
	for(temp=1; temp<64; temp++)
	{
		DRV_WriteReg32(ARMPLLDIV_MON_EN,0xFFFFFFFF);
		//power on
		//big mon
		DRV_WriteReg32(0x102224A0,0x00ff00fe);
		DRV_WriteReg32(0x102224A4,0xd0c4ec4e);
		DRV_WriteReg32(0x102224A0,0x00ff00ff);
		if(!ckgen_abist_needCheck_array[temp-1])
		{ //skip undefine case 1,38,39
		  //sip bigpll 37
		  //print("skip:%d\n",temp);
		 	continue;
		}
		ret_feq = mt_get_abist_freq(temp);
		ret_feq_store[ret_feq_total] = ret_feq;
		ret_feq_total++;

		//print("%s:", ckgen_abist_array[temp-1]);

		print("%d", ret_feq);
#ifdef _RESULT_COMPARE_
		if(ckgen_abist_golden_array[temp-1] > 0)
		{
			if((ret_feq < (ckgen_abist_golden_array[temp-1] - Range)) || (ret_feq > (ckgen_abist_golden_array[temp-1] + Range)))
			{	
				print(" : ERROR");
				isFail = 1;
			}
		}
#endif
		printf("\n");

	}


#ifdef _RESULT_COMPARE_
	if(isFail)
	{
		print("Check PLL/CLK Freq Fail..!!!\n");
		//while(1);
	}
	else
	{
		print("Pass\n");
	}

	pll_chk_is_fail = isFail;

	#endif

#if 0
	// add debug log here
	// for example: print("mt_pll_post_init: AP_PLL_CON3        = 0x%x, GS = 0xFFFCCCCC\n", DRV_Reg32(AP_PLL_CON3));
#endif
	print("Pll post init Done!\n");
#endif // _FREQ_SCAN_
}

void mt_print_pll_chcek_result(void)
{
	unsigned int temp,clk_misc_cfg_0,ret_feq,reg_temp;
	unsigned int isFail = 0;
	unsigned int total = 0;

	#ifdef _FREQ_SCAN_

	print("==============================\n");
	print("==      Parsing Start       ==\n");
	print("==============================\n");
	for(temp=1; temp<32; temp++)
	{
		ret_feq = ret_feq_store[total];
		total++;
		print("%d",ret_feq);
#ifdef _RESULT_COMPARE_
		if((ret_feq < (ckgen_golden_array[temp-1] - Range)) || (ret_feq > (ckgen_golden_array[temp-1] + Range)))
		{
			print(" : ERROR");
			isFail = 1;
		}
#endif
		printf("\n");

	}

	//abist
	for(temp=1; temp<64; temp++)
	{
		if(!ckgen_abist_needCheck_array[temp-1])
		{ //skip undefine case 1,38,39
		  //sip bigpll 37
		  //print("skip:%d\n",temp);
		 	continue;
		}
		ret_feq = ret_feq_store[total];
		total++;

		//print("%s:", ckgen_abist_array[temp-1]);

		print("%d", ret_feq);
#ifdef _RESULT_COMPARE_
		if(ckgen_abist_golden_array[temp-1] > 0)
		{
			if((ret_feq < (ckgen_abist_golden_array[temp-1] - Range)) || (ret_feq > (ckgen_abist_golden_array[temp-1] + Range)))
			{	
				print(" : ERROR");
				isFail = 1;
			}
		}
#endif
		printf("\n");

	}

#ifdef _RESULT_COMPARE_
	if(isFail)
	{
		print("Post Check PLL/CLK Freq Fail..!!!\n");
		//while(1);
	}
	else
	{
		print("Pass\n");
	}
#endif
#endif //_FREQ_SCAN_

}

#if 0
//after pmic_init
void mt_arm_pll_sel(void)
{
    unsigned int temp;

    temp = DRV_Reg32(TOP_CKMUXSEL);
    //DRV_WriteReg32(TOP_CKMUXSEL, temp | 0x5); // switch CA7_ck to ARMCA7PLL, and CA15_ck to ARMCA15PLL
    DRV_WriteReg32(TOP_CKMUXSEL, temp | 0x1); // switch CA7_ck to ARMCA7PLL

    print("[PLL] mt_arm_pll_sel done\n");
}
#endif

void mt_set_topck_default(void)
{
	DRV_WriteReg32(CLK_CFG_0, 0x00000000);
	DRV_WriteReg32(CLK_CFG_1, 0x00000000);
	DRV_WriteReg32(CLK_CFG_2, 0x00000000);
	DRV_WriteReg32(CLK_CFG_3, 0x00000000);
	DRV_WriteReg32(CLK_CFG_4, 0x00000000);
	DRV_WriteReg32(CLK_CFG_5, 0x00000000);
	DRV_WriteReg32(CLK_CFG_6, 0x00000000);
	DRV_WriteReg32(CLK_CFG_7, 0x00000000);
	DRV_WriteReg32(CLK_CFG_8, 0x00000000);
	DRV_WriteReg32(CLK_CFG_UPDATE, 0xFFFFFFFD);    //update all clocks except "fmem & axi"
    //print("[PLL] mt_set_topck_default done\n");
}

#define VOLT_TO_PMIC_VAL(volt)  ((((volt) - 90000) / 2500) + 3)
#define CPULDO_CTRL_1		(INFRACFG_AO_BASE+0xF9C)
#define CPULDO_CTRL_2		(INFRACFG_AO_BASE+0xFA0)
#define CPULDO_CTRL_6		(INFRACFG_AO_BASE+0xFB0)

void mt_pll_init(void)
{
    int ret = 0;
    unsigned int temp;
    unsigned int rdata = 0;
    unsigned int wdata = 0;
    int i;

    rdata = VOLT_TO_PMIC_VAL(110000); // set SRAMLDO to 1.1V

    for (i = 0; i < 4; i++)
        wdata = wdata | (rdata << (i*8));

    DRV_WriteReg32(CPULDO_CTRL_1, wdata);
    DRV_WriteReg32(CPULDO_CTRL_2, wdata);
    print("Set SRAM to 1.1V...\n");
    temp = DRV_Reg32(CPULDO_CTRL_1);
    print("Read SRAM1 = 0x%x...\n", temp);
    temp = DRV_Reg32(CPULDO_CTRL_2);
    print("Read SRAM2 = 0x%x...\n", temp);

    DRV_WriteReg32(CPULDO_CTRL_6, 0x88888888);
    temp = DRV_Reg32(CPULDO_CTRL_6);
    print("Read SRAM2 CTRL6= 0x%x...\n", temp);

    print("Pll init start...\n");
    DRV_WriteReg32(ACLKEN_DIV, 0x12); // MCU Bus DIV2

    //step 1
    DRV_WriteReg32(CLKSQ_STB_CON0, 0x05010501); // reduce CLKSQ disable time

    //step 2
    DRV_WriteReg32(PLL_ISO_CON0, 0x00080008); // extend PWR/ISO control timing to 1us

    //step 3
    DRV_WriteReg32(AP_PLL_CON6, 0x00000000); //

	//step 4
    temp = DRV_Reg32(MCU_PLL_CON2);
    DRV_WriteReg32(MCU_PLL_CON2, temp & 0xffff00ff);

    /*************
    * xPLL PWR ON
    **************/
    //step 5
    temp = DRV_Reg32(ARMCAXPLL0_PWR_CON0);
    DRV_WriteReg32(ARMCAXPLL0_PWR_CON0, temp | 0x1);

    temp = DRV_Reg32(ARMCAXPLL1_PWR_CON0);
    DRV_WriteReg32(ARMCAXPLL1_PWR_CON0, temp | 0x1);

    temp = DRV_Reg32(ARMCAXPLL2_PWR_CON0);
    DRV_WriteReg32(ARMCAXPLL2_PWR_CON0, temp | 0x1);

    temp = DRV_Reg32(ARMCAXPLL3_PWR_CON0);
    DRV_WriteReg32(ARMCAXPLL3_PWR_CON0, temp | 0x1);

    //step 6
    temp = DRV_Reg32(MAINPLL_PWR_CON0);
    DRV_WriteReg32(MAINPLL_PWR_CON0, temp | 0x1);

    //step 7
    temp = DRV_Reg32(UNIVPLL_PWR_CON0);
    DRV_WriteReg32(UNIVPLL_PWR_CON0, temp | 0x1);

    //step 8
    temp = DRV_Reg32(MFGPLL_PWR_CON0);
    DRV_WriteReg32(MFGPLL_PWR_CON0, temp | 0x1);

    //step 9
    temp = DRV_Reg32(MSDCPLL_PWR_CON0);
    DRV_WriteReg32(MSDCPLL_PWR_CON0, temp | 0x1);

    //step 10
    temp = DRV_Reg32(VDECPLL_PWR_CON0);
    DRV_WriteReg32(VDECPLL_PWR_CON0, temp | 0x1);

    //step 11
    temp = DRV_Reg32(IMGPLL_PWR_CON0);
    DRV_WriteReg32(IMGPLL_PWR_CON0, temp | 0x1);

    //step 12
    temp = DRV_Reg32(CODECPLL_PWR_CON0);
    DRV_WriteReg32(CODECPLL_PWR_CON0, temp | 0x1);

    //step 13
    temp = DRV_Reg32(TVDPLL_PWR_CON0);
    DRV_WriteReg32(TVDPLL_PWR_CON0, temp | 0x1);

    //step 14
    //temp = DRV_Reg32(MPLL_PWR_CON0);
    //DRV_WriteReg32(MPLL_PWR_CON0, temp | 0x1);

    //step 15
    temp = DRV_Reg32(APLL1_PWR_CON0);
    DRV_WriteReg32(APLL1_PWR_CON0, temp | 0x1);

    //step 16
    temp = DRV_Reg32(APLL2_PWR_CON0);
    DRV_WriteReg32(APLL2_PWR_CON0, temp | 0x1);

    //gpt_busy_wait_us(5); // wait for xPLL_PWR_ON ready (min delay is 1us)

    /******************
    * xPLL ISO Disable
    *******************/
    //step 17
    temp = DRV_Reg32(ARMCAXPLL0_PWR_CON0);
    DRV_WriteReg32(ARMCAXPLL0_PWR_CON0, temp & 0xFFFFFFFD);

    temp = DRV_Reg32(ARMCAXPLL1_PWR_CON0);
    DRV_WriteReg32(ARMCAXPLL1_PWR_CON0, temp & 0xFFFFFFFD);

    temp = DRV_Reg32(ARMCAXPLL2_PWR_CON0);
    DRV_WriteReg32(ARMCAXPLL2_PWR_CON0, temp & 0xFFFFFFFD);

    temp = DRV_Reg32(ARMCAXPLL3_PWR_CON0);
    DRV_WriteReg32(ARMCAXPLL3_PWR_CON0, temp & 0xFFFFFFFD);

	//step 18
    temp = DRV_Reg32(MAINPLL_PWR_CON0);
    DRV_WriteReg32(MAINPLL_PWR_CON0, temp & 0xFFFFFFFD);

    //step 19
    temp = DRV_Reg32(UNIVPLL_PWR_CON0);
    DRV_WriteReg32(UNIVPLL_PWR_CON0, temp & 0xFFFFFFFD);

    //step 20
    temp = DRV_Reg32(MFGPLL_CON0);
    DRV_WriteReg32(MFGPLL_CON0, temp & 0xFFFFFFFD);

    //step 21
    temp = DRV_Reg32(MSDCPLL_PWR_CON0);
    DRV_WriteReg32(MSDCPLL_PWR_CON0, temp & 0xFFFFFFFD);

    //step 22
    temp = DRV_Reg32(VDECPLL_PWR_CON0);
    DRV_WriteReg32(VDECPLL_PWR_CON0, temp & 0xFFFFFFFD);

	//step 23
    temp = DRV_Reg32(IMGPLL_PWR_CON0);
    DRV_WriteReg32(IMGPLL_PWR_CON0, temp & 0xFFFFFFFD);

    //step 24
    temp = DRV_Reg32(CODECPLL_PWR_CON0);
    DRV_WriteReg32(CODECPLL_PWR_CON0, temp & 0xFFFFFFFD);

    //step 25
    temp = DRV_Reg32(TVDPLL_PWR_CON0);
    DRV_WriteReg32(TVDPLL_PWR_CON0, temp & 0xFFFFFFFD);

    //step 26
    //temp = DRV_Reg32(MPLL_PWR_CON0);
    //DRV_WriteReg32(MPLL_PWR_CON0, temp & 0xFFFFFFFD);

    //step 27
    temp = DRV_Reg32(APLL1_PWR_CON0);
    DRV_WriteReg32(APLL1_PWR_CON0, temp & 0xFFFFFFFD);

    //step 28
    temp = DRV_Reg32(APLL2_PWR_CON0);
    DRV_WriteReg32(APLL2_PWR_CON0, temp & 0xFFFFFFFD);

    /********************
       * xPLL Frequency Set
       *********************/

    //step 29: CPU SPEED*****
    DRV_WriteReg32(ARMCAXPLL0_CON1, 0xC1114000); // 897MHz
    DRV_WriteReg32(ARMCAXPLL1_CON1, 0xC00C4000); // 1274MHz
    DRV_WriteReg32(ARMCAXPLL2_CON1, 0xC10C1D89); // 630MHz
    DRV_WriteReg32(ARMCAXPLL3_CON1, 0xC1133B14); // 1000MHz

    //step 30
    DRV_WriteReg32(MAINPLL_CON1, 0x800A8000); //1092MHz

    //step 31
    DRV_WriteReg32(MFGPLL_CON1, 0x82134000); //500MHz

    //step 32
    DRV_WriteReg32(MSDCPLL_CON1, 0x800EC4EC); //384MHz

    //step 33
    // DRV_WriteReg32(VDECPLL_CON1, 0x80133B14); //500MHz
    // Boot in LPM mode
    DRV_WriteReg32(VDECPLL_CON1, 0x800D0000); //333MHz
    
    //step 34
    // DRV_WriteReg32(IMGPLL_CON1, 0x80114EC5); //450MHz
    DRV_WriteReg32(IMGPLL_CON1, 0x800C8000); //320Hz
    
    //step 35
    // DRV_WriteReg32(CODECPLL_CON1, 0x80130000); //500MHz
    DRV_WriteReg32(CODECPLL_CON1, 0x800C8000); //320MHz
    
    //step 36
    DRV_WriteReg32(TVDPLL_CON1, 0x8016d89E); // 445.5MHz

    //step 37
    //DRV_WriteReg32(MPLL_CON1, 0x801C0000); //52MHz
    //DRV_WriteReg32(MPLL_CON0, 0x00010110); //mpll_postdiv


    /***********************
      * xPLL Frequency Enable
      ************************/
    //step 38
    temp = DRV_Reg32(ARMCAXPLL0_CON0);
    DRV_WriteReg32(ARMCAXPLL0_CON0, temp | 0x1);

    temp = DRV_Reg32(ARMCAXPLL1_CON0);
    DRV_WriteReg32(ARMCAXPLL1_CON0, temp | 0x1);

    temp = DRV_Reg32(ARMCAXPLL2_CON0);
    DRV_WriteReg32(ARMCAXPLL2_CON0, temp | 0x1);

    temp = DRV_Reg32(ARMCAXPLL3_CON0);
    DRV_WriteReg32(ARMCAXPLL3_CON0, temp | 0x1);

    //step 39
    temp = DRV_Reg32(MAINPLL_CON0);
    DRV_WriteReg32(MAINPLL_CON0, temp | 0x1);

    //step 40
    temp = DRV_Reg32(UNIVPLL_CON0);
    DRV_WriteReg32(UNIVPLL_CON0, temp | 0x1);

    //step 41
    temp = DRV_Reg32(MFGPLL_CON0);
    DRV_WriteReg32(MFGPLL_CON0, temp | 0x1);

    //step 42
    temp = DRV_Reg32(MSDCPLL_CON0);
    DRV_WriteReg32(MSDCPLL_CON0, temp | 0x1);

    //step 43
    temp = DRV_Reg32(VDECPLL_CON0);
    DRV_WriteReg32(VDECPLL_CON0, temp | 0x1);

	//step 44
	temp = DRV_Reg32(CODECPLL_CON0);
	DRV_WriteReg32(CODECPLL_CON0, temp | 0x1);

	//step 45
	temp = DRV_Reg32(IMGPLL_CON0);
	DRV_WriteReg32(IMGPLL_CON0, temp | 0x1);

    //step 46
    temp = DRV_Reg32(TVDPLL_CON0);
    DRV_WriteReg32(TVDPLL_CON0, temp | 0x1);

    //step 47
    //temp = DRV_Reg32(MPLL_CON0);
    //DRV_WriteReg32(MPLL_CON0, temp | 0x1);

    //step 48
    temp = DRV_Reg32(APLL1_CON0);
    DRV_WriteReg32(APLL1_CON0, temp | 0x1);

    //step 49
    temp = DRV_Reg32(APLL2_CON0);
    DRV_WriteReg32(APLL2_CON0, temp | 0x1);

	// step 50
    gpt_busy_wait_us(20); // wait for PLL stable (min delay is 20us)

    /***************
      * xPLL DIV RSTB
      ****************/
	//step 51
	temp = DRV_Reg32(MAINPLL_CON0);
	DRV_WriteReg32(MAINPLL_CON0, temp | 0x01000000);

	//step 52
	temp = DRV_Reg32(UNIVPLL_CON0);
	DRV_WriteReg32(UNIVPLL_CON0, temp | 0x01000000);


    /*****************
	* xPLL HW Control
    	******************/

	//switch to CLKSQ & PLL to HW Control
    //step 53
    //DRV_WriteReg32(AP_PLL_CON3, 0x00044440);

    //step 54
    //DRV_WriteReg32(AP_PLL_CON4, temp & 0xC);

	/*************
	   * set armpll HW mode
	**************/
	temp = DRV_Reg32(MCU_PLL_CON0);
	DRV_WriteReg32(MCU_PLL_CON0, temp & 0xffff0000);

	temp = DRV_Reg32(MCU_PLL_CON1);
	DRV_WriteReg32(MCU_PLL_CON1, temp & 0xffffff00);

	 /*************
    	* ULPOSC Init
   	 **************/
	temp = DRV_Reg32(INFRA_PLL_ULPOSC_CON0);
	DRV_WriteReg32(INFRA_PLL_ULPOSC_CON0, temp & 0xC0FFFFFF);

	temp = DRV_Reg32(INFRA_PLL_ULPOSC_CON0);
	DRV_WriteReg32(INFRA_PLL_ULPOSC_CON0, temp | 0x28000000);

	/* enable */
	DRV_WriteReg32(ULPOSC_CON, 0x00000001);
	DRV_WriteReg32(ULPOSC_CON, 0x00000003);
	gpt_busy_wait_us(100);
	DRV_WriteReg32(ULPOSC_CON, 0x00000001);
	DRV_WriteReg32(ULPOSC_CON, 0x00000005);


    /*************
    	* MEMPLL Init
   	 **************/

//    mt_mempll_pre();

    /**************
    * INFRA CLKMUX
    ***************/
  	#if 0 //no this bit
    temp = DRV_Reg32(TOP_DCMCTL);
    DRV_WriteReg32(TOP_DCMCTL, temp | 0x1); // enable infrasys DCM
    #endif

	temp = DRV_Reg32(ARMPLLDIV_CKDIV) ;
    DRV_WriteReg32(ARMPLLDIV_CKDIV, temp & 0x1f); // CPU clock divide by 1

    temp = DRV_Reg32(ARMPLLDIV_MUXSEL);
    DRV_WriteReg32(ARMPLLDIV_MUXSEL, (temp & 0x3) | 0x54); // switch to PLL speed (L/LL/CCI)

    /************
    * TOP CLKMUX
    *************/

	// //----------------------------------------------------------
    // 1.0v clock
    // axi = syspll_d7 = 1092/7 = 156MHz
    // mm_ck = imgpll_ck = 450MHz
    // ddrphycfg_ck = syspll3_d2 = 1092/5/2 = 109.2MHz
    // pwm_ck = univpll2_d4 = 1248/3/4= 104MHz
    // vdec_ck = vdecpll_ck = 500MHz
    // venc_ck = codecpll_ck = 494MHz
    // mfg_ck = mfgpll_ck = 500MHz
    // usb20_ck = univpll1_d8 = 1248/2/8 = 78MHz
    // spi_ck = syspll3_d2 = 1092/5/2 = 109.2MHz
    // camtg_ck = univpll2_d2 = 1248/3/2 = 208MHz
    // msdc50_0_hclk = syspll1_d2 = 1092/2/2 = 273MHz
    // msdc50_0 = msdcpll_ck = 384MHz
    // msdc30_1 = msdcpll_d2 = 384/2=192 MHz
    // msdc30_2 = univpll2_d8 = 1248/3/8 = 52MHz
    // aud_intbus = syspll1_d4 = 1092/2/4= 136.5MHz
    // mjc_ck = imgpll_ck = 450MHz
    // atb_ck = syspll1_d2 = 1092/2/2= 273MHz
    // scp_ck = syspll_d3 = 1092/3 = 364MHz
    // aud2_ck = APLL2
    // aud1_ck = APLL1
    // dpi0_ck = tvdpll_d4 = 594/4=148.5MHz
    // ssusb_top_ck = univpll3_d2 = 1248/5/2=124.8MHz
    // hf_fspm = syspll1_d8 = 1092/2/8=68.25MHz
    // bsi_spi = syspll1_d4 = 1092/2/4=136.5MHz
    // aud_h_ck = APLL2
    // anc_md32_ck = syspll1_d2 = 1092/2/2=273MHz
    //-----------------------------------------------------------

    // [22] set this register before AXI clock switch to fast clock (26MHz => 156MHz)
    DRV_WriteReg32(INFRA_BUS_DCM_CTRL, DRV_Reg32(INFRA_BUS_DCM_CTRL) | (1 << 22));


    // config AXI clock first
    DRV_WriteReg32(CLK_CFG_0, 0x00000001);      //axi=syspll_d7
    DRV_WriteReg32(CLK_CFG_UPDATE, 0x00000001); //update AXI clock


    // config other clocks
    DRV_WriteReg32(CLK_CFG_0, 0x01010001);//mm_ck=imgpll_ck, ddrphycfg_ck=syspll3_d2, mem_ck=NULL, axi=syspll_d7,
    DRV_WriteReg32(CLK_CFG_1, 0x01010101);//mfg_ck=mfgpll_ck, venc_ck=codecpll_ck, vdec_ck=vdecpll_ck, pwm_ck=univpll2_d4
    DRV_WriteReg32(CLK_CFG_2, 0x01010001);//usb20_ck=univpll1_d8, spi_ck=syspll3_d2, uart=26M, camtg=univpll_d26
    DRV_WriteReg32(CLK_CFG_3, 0x02010100);//msdc30_1=msdcpll_d2, msdc50_0=msdcpll_ck, msdc50_0_hclk=syspll1_d2, Null
    DRV_WriteReg32(CLK_CFG_4, 0x01000101);//aud_intbus=syspll1_d4, aud=26M, Null, msdc30_2=univpll2_d8
    DRV_WriteReg32(CLK_CFG_5, 0x01010100);//mjc_ck=imgpll_ck, atb_ck=syspll1_d2, scp_ck=syspll_d3, pmicspi=26MHz
    DRV_WriteReg32(CLK_CFG_6, 0x01010002);//aud2_ck=apll2 , aud1_ck=apll1, Null, dpi0_ck=tvdpll_d4
    DRV_WriteReg32(CLK_CFG_7, 0x00000100);//Null, Null, ssusb_top_sys= univpll3_d2, Null
    DRV_WriteReg32(CLK_CFG_8, 0x01010201);//anc_md32_ck= syspll1_d2, aud_h_ck=apll2,bsi_spi=syspll1_d4, hg_fspm=syspll1_d8


    DRV_WriteReg32(CLK_CFG_UPDATE, 0xFFFFFFFC);    //update all clocks except "fmem & axi"

    //DRV_WriteReg32(CLK_SCP_CFG_0,  0x00000FFF);    // enable scpsys clock off control
    //DRV_WriteReg32(CLK_SCP_CFG_1,  0x00000007);    // enable scpsys clock off control


    /*for MTCMOS*/
    //spm_write(POWERON_CONFIG_EN, (SPM_PROJECT_CODE << 16) | (1U << 0));
    print("mtcmos Start..\n");
    spm_mtcmos_ctrl_dis(STA_POWER_ON);	
#if 0
	spm_mtcmos_ctrl_md1( STA_POWER_ON);
	spm_mtcmos_ctrl_isp( STA_POWER_ON);
	spm_mtcmos_ctrl_vde( STA_POWER_ON);
	spm_mtcmos_ctrl_mfg_async( STA_POWER_ON);
	spm_mtcmos_ctrl_mfg( STA_POWER_ON);
	spm_mtcmos_ctrl_mfg_core3( STA_POWER_ON);
	spm_mtcmos_ctrl_mfg_core2( STA_POWER_ON);
	spm_mtcmos_ctrl_mfg_core1( STA_POWER_ON);
	spm_mtcmos_ctrl_mfg_core0( STA_POWER_ON);
	spm_mtcmos_ctrl_mjc( STA_POWER_ON);
	spm_mtcmos_ctrl_ven( STA_POWER_ON);
	spm_mtcmos_ctrl_audio( STA_POWER_ON);
	spm_mtcmos_ctrl_c2k( STA_POWER_ON);
#endif

    print("mtcmos Done!\n");
    /*for CG*/
	//infra default disable CG but auxadc default on
    DRV_WriteReg32(INFRA_SW_CG0_CLR, 0xFBEFFFFF); //bit 20,26,: no use
    DRV_WriteReg32(INFRA_SW_CG1_CLR, 0xFFFF3FFF); //bit 14, 15: no use , bit 11 is auxadc
    DRV_WriteReg32(INFRA_SW_CG2_CLR, 0xFFFFFDFF); //bit 9: no use, bit 14 dvfs_spm0
	/*dvfs cpu, gpu setting*/
	temp = DRV_Reg32(PERI_DVFS0_TX_APB_ASYNC_STA);
	DRV_WriteReg32(PERI_DVFS0_TX_APB_ASYNC_STA, temp | 0x1 );

	temp = DRV_Reg32(PERI_DVFS1_TX_APB_ASYNC_STA);
	DRV_WriteReg32(PERI_DVFS1_TX_APB_ASYNC_STA, temp | 0x1);

    /*DISP CG*/
    DRV_WriteReg32(MMSYS_CG_CLR0, 0xFFFFFFFF);
    DRV_WriteReg32(MMSYS_CG_CLR1, 0xFF);

    print("Pll init Done!\n");
}

