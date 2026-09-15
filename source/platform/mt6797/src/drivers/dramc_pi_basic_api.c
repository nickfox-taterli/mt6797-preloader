/*----------------------------------------------------------------------------*
 * Copyright Statement:                                                       *
 *                                                                            *
 *   This software/firmware and related documentation ("MediaTek Software")   *
 * are protected under international and related jurisdictions'copyright laws *
 * as unpublished works. The information contained herein is confidential and *
 * proprietary to MediaTek Inc. Without the prior written permission of       *
 * MediaTek Inc., any reproduction, modification, use or disclosure of        *
 * MediaTek Software, and information contained herein, in whole or in part,  *
 * shall be strictly prohibited.                                              *
 * MediaTek Inc. Copyright (C) 2010. All rights reserved.                     *
 *                                                                            *
 *   BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND     *
 * AGREES TO THE FOLLOWING:                                                   *
 *                                                                            *
 *   1)Any and all intellectual property rights (including without            *
 * limitation, patent, copyright, and trade secrets) in and to this           *
 * Software/firmware and related documentation ("MediaTek Software") shall    *
 * remain the exclusive property of MediaTek Inc. Any and all intellectual    *
 * property rights (including without limitation, patent, copyright, and      *
 * trade secrets) in and to any modifications and derivatives to MediaTek     *
 * Software, whoever made, shall also remain the exclusive property of        *
 * MediaTek Inc.  Nothing herein shall be construed as any transfer of any    *
 * title to any intellectual property right in MediaTek Software to Receiver. *
 *                                                                            *
 *   2)This MediaTek Software Receiver received from MediaTek Inc. and/or its *
 * representatives is provided to Receiver on an "AS IS" basis only.          *
 * MediaTek Inc. expressly disclaims all warranties, expressed or implied,    *
 * including but not limited to any implied warranties of merchantability,    *
 * non-infringement and fitness for a particular purpose and any warranties   *
 * arising out of course of performance, course of dealing or usage of trade. *
 * MediaTek Inc. does not provide any warranty whatsoever with respect to the *
 * software of any third party which may be used by, incorporated in, or      *
 * supplied with the MediaTek Software, and Receiver agrees to look only to   *
 * such third parties for any warranty claim relating thereto.  Receiver      *
 * expressly acknowledges that it is Receiver's sole responsibility to obtain *
 * from any third party all proper licenses contained in or delivered with    *
 * MediaTek Software.  MediaTek is not responsible for any MediaTek Software  *
 * releases made to Receiver's specifications or to conform to a particular   *
 * standard or open forum.                                                    *
 *                                                                            *
 *   3)Receiver further acknowledge that Receiver may, either presently       *
 * and/or in the future, instruct MediaTek Inc. to assist it in the           *
 * development and the implementation, in accordance with Receiver's designs, *
 * of certain softwares relating to Receiver's product(s) (the "Services").   *
 * Except as may be otherwise agreed to in writing, no warranties of any      *
 * kind, whether express or implied, are given by MediaTek Inc. with respect  *
 * to the Services provided, and the Services are provided on an "AS IS"      *
 * basis. Receiver further acknowledges that the Services may contain errors  *
 * that testing is important and it is solely responsible for fully testing   *
 * the Services and/or derivatives thereof before they are used, sublicensed  *
 * or distributed. Should there be any third party action brought against     *
 * MediaTek Inc. arising out of or relating to the Services, Receiver agree   *
 * to fully indemnify and hold MediaTek Inc. harmless.  If the parties        *
 * mutually agree to enter into or continue a business relationship or other  *
 * arrangement, the terms and conditions set forth herein shall remain        *
 * effective and, unless explicitly stated otherwise, shall prevail in the    *
 * event of a conflict in the terms in any agreements entered into between    *
 * the parties.                                                               *
 *                                                                            *
 *   4)Receiver's sole and exclusive remedy and MediaTek Inc.'s entire and    *
 * cumulative liability with respect to MediaTek Software released hereunder  *
 * will be, at MediaTek Inc.'s sole discretion, to replace or revise the      *
 * MediaTek Software at issue.                                                *
 *                                                                            *
 *   5)The transaction contemplated hereunder shall be construed in           *
 * accordance with the laws of Singapore, excluding its conflict of laws      *
 * principles.  Any disputes, controversies or claims arising thereof and     *
 * related thereto shall be settled via arbitration in Singapore, under the   *
 * then current rules of the International Chamber of Commerce (ICC).  The    *
 * arbitration shall be conducted in English. The awards of the arbitration   *
 * shall be final and binding upon both parties and shall be entered and      *
 * enforceable in any court of competent jurisdiction.                        *
 *---------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------
 *
 * $Author: jc.wu $
 * $Date: 2012/6/5 $
 * $RCSfile: pi_basic_api.c,v $
 * $Revision: #5 $
 *
 *---------------------------------------------------------------------------*/

/** @file pi_basic_api.c
 *  Basic DRAMC API implementation
 */

//-----------------------------------------------------------------------------
// Include files
//-----------------------------------------------------------------------------
//#include "..\Common\pd_common.h"
//#include "Register.h"
#include "dramc_common.h"
#include "x_hal_io.h"
//#include "DramC_reg.h"
//#include "System_reg.h"
//#include "string.h"
//#define mcFPRINTF(_x_)            do{}while(0)

#if  EVEREST_MEM_C_TEST_CODE
//for output mem.c log use
int tmpVar = 0;
unsigned int *DRAMC_WBR = &tmpVar;
unsigned int *CLK_CFG_0 = &tmpVar;
unsigned int *CLK_CFG_UPDATE = &tmpVar;
unsigned int *CLK_DDRPHY_REG = &tmpVar;
unsigned int *CLK_MEM_DFS_CFG = &tmpVar;
unsigned int *DFS_MEM_DCM_CTRL = &tmpVar;
unsigned int *DPY_PWR_CON = &tmpVar;
unsigned int *DPY_CH1_PWR_CON = &tmpVar;
unsigned int *SPM_PLL_CON = &tmpVar;
unsigned int *PCM_PWR_IO_EN = &tmpVar;
unsigned int *SPM_POWER_ON_VAL0 = &tmpVar;
unsigned int *MPLL_PWR_CON0 = &tmpVar;
unsigned int *MPLL_CON0 = &tmpVar;
#endif

#ifdef DUAL_RANKS
unsigned int uiDualRank = 1;
#endif

U32 u4MR2Value;

#ifdef _WIN32
#define dsb()
#endif

#define COMPILE_A60817_LP3_INIT_CODE 0

#define APPLY_LP4_4266_SETTING 1
#define APPLY_LP3_2133_SETTING 1

#define SSC_TEST_ORIGINAL 0  //old setting
#define PATTERN_TEST_ORIGINAL 1//old setting
#define GATING_PULSE_MODE 1

#define APPLY_SIMULATION_SETTING 0
#define APPLY_A60501_NEW_SETTING 1
#define PHY_MONCLK_SETTING_TEST 0// test for Justin
#define PLL_JITTER_FINE_TUNE 1


//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
#if fcFOR_CHIP_ID == fcA60501
// for TA test engine (only for test chip)
static DRAM_TA_PATTERN_T last_testpat_cha = TA_PATTERN_IDLE;
static DRAM_TA_PATTERN_T last_testpat_chb = TA_PATTERN_IDLE;
static DRAM_TA_PATTERN_T last_testpat_chc = TA_PATTERN_IDLE;

static U8 ta1_mc_running_cha = DISABLE;
static U8 ta1_mc_running_chb = DISABLE;
static U8 ta1_mc_running_chc = DISABLE;

static U8 ta_option_turn_around = ENABLE;
#endif

#if fcFOR_CHIP_ID == fcMT6595
extern const U32 uiLPDDR_PHY_Mapping_POP_CHA[32];
extern const U32 uiLPDDR_PHY_Mapping_POP_CHB[32];
#endif

#if REG_ACCESS_PORTING_DGB
extern U8 RegLogEnable;
#endif


//-------------------------------------------------------------------------
/** Round_Operation
 *  Round operation of A/B
 *  @param  A   
 *  @param  B   
 *  @retval round(A/B) 
 */
//-------------------------------------------------------------------------
U16 Round_Operation(U16 A, U16 B)
{
    U16 temp;

    if (B == 0)
    {
        return 0xffff;
    }
    
    temp = A/B;
        
    if ((A-temp*B) >= ((temp+1)*B-A))
    {
        return (temp+1);
    }
    else
    {
        return temp;
    }    
}


U32 PLL_FBKDIV_6_0, PLL_IC_3_0, PLL_BR_3_0, PLL_BC_3_0, PLL_BC_4;
U32 PLL_BP_3_0, PLL_BP_4, PLL_BAND_6_0;

U32 PLL_FBKSEL_0;

void RISCWriteDRAM(unsigned int UI_offset_address, unsigned int UI_content_value)
{
#if !EVEREST_MEM_C_TEST_CODE
	*((volatile unsigned int *)(DRAMC0_BASE + UI_offset_address)) = UI_content_value;
	*((volatile unsigned int *)(DRAMC0_NAO_BASE + UI_offset_address)) = UI_content_value;
#else
	printf("\n[REG_ACCESS_PORTING_DBG]   ucDramC_Register_Write Reg(0x%x) = 0x%x\n",(DRAMC0_BASE + UI_offset_address),  UI_content_value);	
#endif
} // end of RISCWriteDRAM

void RISCWriteDDRPHY(unsigned int UI_offset_address, unsigned int UI_content_value)
{
#if !EVEREST_MEM_C_TEST_CODE
	*((volatile unsigned int *)(DDRPHY_BASE + UI_offset_address)) = UI_content_value;
#else
	printf("\n[REG_ACCESS_PORTING_DBG]   ucDramC_Register_Write Reg(0x%x) = 0x%x\n",(DDRPHY_BASE + UI_offset_address),  UI_content_value);
#endif
} // end of RISCWriteDDRPHY

void RISCWrite(unsigned int UI_offset_address, unsigned int UI_content_value)
{
#if !EVEREST_MEM_C_TEST_CODE
	*((volatile unsigned int *)(UI_offset_address)) = UI_content_value;
#else
	printf("\n[REG_ACCESS_PORTING_DBG]   ucDramC_Register_Write Reg(0x%x) = 0x%x\n",(UI_offset_address),  UI_content_value);
#endif
} // end of RISCWrite

unsigned int RISCRead(unsigned int UI_offset_address)
{
#if !EVEREST_MEM_C_TEST_CODE
	return *((volatile unsigned int *)(UI_offset_address));
#else
	return 0;
#endif
} // end of RISCWrite

#if 0//__ETT__
void RISCReadAll(void)
{
    unsigned int address = DRAMC0_BASE;
    unsigned int value = 0;
    
    /*TINFO=">>>>>DRAMC0<<<<<\n"*/
    mcSHOW_DBG_MSG((">>>>>DRAMC0<<<<<\n"));
    for(address=DRAMC0_BASE; address<=DRAMC0_BASE+0xD54; address+=4)
    {
        value = *((volatile unsigned int *)(address));
        #ifdef ETT_PRINT_FORMAT
        mcSHOW_DBG_MSG(("Addr. 0x%X = 0x%X\n", address, value));
        #else
        mcSHOW_DBG_MSG(("Addr. 0x%8x = 0x%8x\n", address, value));
        #endif
        /*TINFO="[REG_ACCESS_PORTING_DBG] Register_Dump Reg(%h) = %h\n", address, value*/
    }
    
    /*TINFO=">>>>>DRAMC1<<<<<\n"*/
    mcSHOW_DBG_MSG((">>>>>DRAMC1<<<<<\n"));
    for(address=DRAMC1_BASE; address<=DRAMC1_BASE+0xD54; address+=4)
    {
        value = *((volatile unsigned int *)(address));
        #ifdef ETT_PRINT_FORMAT
        mcSHOW_DBG_MSG(("Addr. 0x%X = 0x%X\n", address, value));
        #else
        mcSHOW_DBG_MSG(("Addr. 0x%8x = 0x%8x\n", address, value));
        #endif
        /*TINFO="[REG_ACCESS_PORTING_DBG] Register_Dump Reg(%h) = %h\n", address, value*/
    }

    /*TINFO=">>>>>DRAMC0_NAO<<<<<\n"*/
    mcSHOW_DBG_MSG((">>>>>DRAMC0_NAO<<<<<\n"));
    for(address=DRAMC0_NAO_BASE; address<=DRAMC0_NAO_BASE+0x59C; address+=4)
    {
        value = *((volatile unsigned int *)(address));
        #ifdef ETT_PRINT_FORMAT
        mcSHOW_DBG_MSG(("Addr. 0x%X = 0x%X\n", address, value));
        #else
        mcSHOW_DBG_MSG(("Addr. 0x%8x = 0x%8x\n", address, value));
        #endif
        /*TINFO="[REG_ACCESS_PORTING_DBG] Register_Dump Reg(%h) = %h\n", address, value*/
    }
    
    /*TINFO=">>>>>DRAMC1_NAO<<<<<\n"*/
    mcSHOW_DBG_MSG((">>>>>DRAMC1_NAO<<<<<\n"));
    for(address=DRAMC1_NAO_BASE; address<=DRAMC1_NAO_BASE+0x59C; address+=4)
    {
        value = *((volatile unsigned int *)(address));
        #ifdef ETT_PRINT_FORMAT
        mcSHOW_DBG_MSG(("Addr. 0x%X = 0x%X\n", address, value));
        #else
        mcSHOW_DBG_MSG(("Addr. 0x%8x = 0x%8x\n", address, value));
        #endif
        /*TINFO="[REG_ACCESS_PORTING_DBG] Register_Dump Reg(%h) = %h\n", address, value*/
    }

    /*TINFO=">>>>>DDRPHY_A<<<<<\n"*/
    mcSHOW_DBG_MSG((">>>>>DDRPHY_A<<<<<\n"));
    for(address=DDRPHY_BASE; address<=DDRPHY_BASE+0xFD4; address+=4)
    {
        value = *((volatile unsigned int *)(address));
        #ifdef ETT_PRINT_FORMAT
        mcSHOW_DBG_MSG(("Addr. 0x%X = 0x%X\n", address, value));
        #else
        mcSHOW_DBG_MSG(("Addr. 0x%8x = 0x%8x\n", address, value));
        #endif
        /*TINFO="[REG_ACCESS_PORTING_DBG] Register_Dump Reg(%h) = %h\n", address, value*/
    }
    
    /*TINFO=">>>>>DDRPHY_B<<<<<\n"*/
    mcSHOW_DBG_MSG((">>>>>DDRPHY_B<<<<<\n"));
    for(address=DDRPHY1_BASE; address<=DDRPHY1_BASE+0xFD4; address+=4)
    {
        value = *((volatile unsigned int *)(address));
        #ifdef ETT_PRINT_FORMAT
        mcSHOW_DBG_MSG(("Addr. 0x%X = 0x%X\n", address, value));
        #else
        mcSHOW_DBG_MSG(("Addr. 0x%8x = 0x%8x\n", address, value));
        #endif
        /*TINFO="[REG_ACCESS_PORTING_DBG] Register_Dump Reg(%h) = %h\n", address, value*/
    }

    /*TINFO=">>>>>DDRPHY_C<<<<<\n"*/
    mcSHOW_DBG_MSG((">>>>>DDRPHY_C<<<<<\n"));
    for(address=DDRPHY2_BASE; address<=DDRPHY2_BASE+0xFD4; address+=4)
    {
        value = *((volatile unsigned int *)(address));
        #ifdef ETT_PRINT_FORMAT
        mcSHOW_DBG_MSG(("Addr. 0x%X = 0x%X\n", address, value));
        #else
        mcSHOW_DBG_MSG(("Addr. 0x%8x = 0x%8x\n", address, value));
        #endif
        /*TINFO="[REG_ACCESS_PORTING_DBG] Register_Dump Reg(%h) = %h\n", address, value*/
    }
    /*TINFO=">>>>>DDRPHY_D<<<<<\n"*/
    mcSHOW_DBG_MSG((">>>>>DDRPHY_D<<<<<\n"));
    for(address=DDRPHY3_BASE; address<=DDRPHY3_BASE+0xFD4; address+=4)
    {
        value = *((volatile unsigned int *)(address));
        #ifdef ETT_PRINT_FORMAT
        mcSHOW_DBG_MSG(("Addr. 0x%X = 0x%X\n", address, value));
        #else
        mcSHOW_DBG_MSG(("Addr. 0x%8x = 0x%8x\n", address, value));
        #endif
        /*TINFO="[REG_ACCESS_PORTING_DBG] Register_Dump Reg(%h) = %h\n", address, value*/
    }
}
#endif

#if PHY_MONCLK_SETTING_TEST
static void DdrPhyMonclkSetting(DRAMC_CTX_T *p)
{

    vIO32WriteFldAlign((DDRPHY_PLL15+ (2<<POS_BANK_NUM)), 1, PLL15_RG_RPHYPLL_TSTCK_EN);
    vIO32WriteFldAlign((DDRPHY_PLL15+ (2<<POS_BANK_NUM)), 1, PLL15_RG_RPHYPLL_TSTFM_EN);
    vIO32WriteFldAlign((DDRPHY_PLL15+ (2<<POS_BANK_NUM)), 1, PLL15_RG_RPHYPLL_TSTOD_EN);
    vIO32WriteFldAlign((DDRPHY_PLL15+ (2<<POS_BANK_NUM)), 0, PLL15_RG_RPHYPLL_TSTOP_EN);
    vIO32WriteFldAlign((DDRPHY_PLL15+ (2<<POS_BANK_NUM)), 1, PLL15_RG_RPHYPLL_TST_EN);
    vIO32WriteFldAlign((DDRPHY_PLL15+ (2<<POS_BANK_NUM)), 0, PLL15_RG_RPHYPLL_TST_SEL); //0, 1, 4 ,5 ,6
    //vIO32WriteFldAlign((DDRPHY_PLL15+ (2<<POS_BANK_NUM)), 4, PLL15_RG_RPHYPLL_TST_SEL); //0, 1, 4=LJ ,5 ,6

    vIO32WriteFldAlign((DDRPHY_PLL4+ (2<<POS_BANK_NUM)), 1, PLL4_RG_RCLRPLL_MONCK_EN); // 1 one in a time
    vIO32WriteFldAlign((DDRPHY_PLL4+ (2<<POS_BANK_NUM)), 0, PLL4_RG_RCLRPLL_MONVC_EN);
    vIO32WriteFldAlign((DDRPHY_PLL4+ (2<<POS_BANK_NUM)), 0, PLL4_RG_RCLRPLL_MONREF_EN);

    vIO32WriteFldAlign((DDRPHY_PLL2+ (2<<POS_BANK_NUM)), 1, PLL2_RG_RPHYPLL_MONCK_EN);// 1 one in a time
    vIO32WriteFldAlign((DDRPHY_PLL2+ (2<<POS_BANK_NUM)), 0, PLL2_RG_RPHYPLL_MONVC_EN);
    vIO32WriteFldAlign((DDRPHY_PLL2+ (2<<POS_BANK_NUM)), 0, PLL2_RG_RPHYPLL_MONREF_EN);

    vIO32WriteFldAlign((DDRPHY_PLL15+ (2<<POS_BANK_NUM)), 0, PLL15_RG_RVREF_VREF_EN);
}
#endif


static void DDRPhyPLLSetting(DRAMC_CTX_T *p)
{
    if(p->freq_sel<LJ_MAX_SEL)
    {
        //PHYPLL setting
        //RPHYPLL_BP =  {RG_*_RPHYPLL_BPB[1:0], RG_*_RPHYPLL_BPA[1:0]} 
        #if CHECK_PLL_OK
        if(is_pll_good() == 0) //not all pll is good, use fix band
        {
            vIO32WriteFldMulti_All(DDRPHY_PLL2, P_Fld(0x0, PLL2_RG_RPHYPLL_AUTOK_EN) | P_Fld(0x0, PLL2_RG_RPHYPLL_AUTOK_LOAD) \
                | P_Fld(0x3, PLL2_RG_RPHYPLL_AUTOK_KS) | P_Fld(0x3, PLL2_RG_RPHYPLL_AUTOK_KF) | P_Fld(0x20, PLL2_RG_RPHYPLL_BAND));           
        }
        else if(is_pll_good() == 1) // All pll is good, use auto k band
        #endif
        {
            vIO32WriteFldMulti_All(DDRPHY_PLL2, P_Fld(0x1, PLL2_RG_RPHYPLL_AUTOK_EN) | P_Fld(0x0, PLL2_RG_RPHYPLL_AUTOK_LOAD) \
                | P_Fld(0x0, PLL2_RG_RPHYPLL_AUTOK_KS) | P_Fld(0x0, PLL2_RG_RPHYPLL_AUTOK_KF) | P_Fld(0x20, PLL2_RG_RPHYPLL_BAND));              
        }          

        if(p->freq_sel==LJ_DDR1866 || p->freq_sel==LJ_DDR933)
        {
            #if CHECK_PLL_OK
            if(is_pll_good() == 0) //not all pll is good, use fix band
            {
                vIO32WriteFldAlign_All(DDRPHY_PLL2, 0x30, PLL2_RG_RPHYPLL_BAND);  
            }            
            #endif
          
            vIO32WriteFldMulti_All(DDRPHY_PLL1, P_Fld(0x0, PLL1_RG_RPHYPLL_EN) | P_Fld(0x22, PLL1_RG_RPHYPLL_FBKDIV) \
                                 | P_Fld(0x1, PLL1_RG_RPHYPLL_FBKSEL) | P_Fld(0x2, PLL1_RG_RPHYPLL_PREDIV) \
                                 | P_Fld(0x1, PLL1_RG_RPHYPLL_IC) | P_Fld(0xc, PLL1_RG_RPHYPLL_BP));
            vIO32WriteFldMulti_All(DDRPHY_PLL_9E, P_Fld(0x2, PLL_9E_RG_RPHYPLL_BR) | P_Fld(0x3, PLL_9E_RG_RPHYPLL_BC) \
                                | P_Fld(0x8, PLL_9E_RG_RPHYPLL_IR) | P_Fld(0x7, PLL_9E_RG_RPHYPLL_BPB) | P_Fld(0x0, PLL_9E_RG_RPHYPLL_BPA));
            vIO32WriteFldAlign_All(DDRPHY_PLL8, 0 ,PLL8_RG_RPI_CAP_SEL);
        }
        else if(p->freq_sel==LJ_DDR1700 || p->freq_sel==LJ_DDR850)
        {
            #if CHECK_PLL_OK
            if(is_pll_good() == 0) //not all pll is good, use fix band
            {
                vIO32WriteFldAlign_All(DDRPHY_PLL2, 0x28, PLL2_RG_RPHYPLL_BAND);                
            }        
            #endif  
            vIO32WriteFldMulti_All(DDRPHY_PLL1, P_Fld(0x0, PLL1_RG_RPHYPLL_EN) | P_Fld(0x20, PLL1_RG_RPHYPLL_FBKDIV) \
                                 | P_Fld(0x1, PLL1_RG_RPHYPLL_FBKSEL) | P_Fld(0x2, PLL1_RG_RPHYPLL_PREDIV) \
                                 | P_Fld(0x1, PLL1_RG_RPHYPLL_IC) | P_Fld(0xc, PLL1_RG_RPHYPLL_BP));
            vIO32WriteFldMulti_All(DDRPHY_PLL_9E, P_Fld(0x2, PLL_9E_RG_RPHYPLL_BR) | P_Fld(0x3, PLL_9E_RG_RPHYPLL_BC) \
                                | P_Fld(0x8, PLL_9E_RG_RPHYPLL_IR) | P_Fld(0x7, PLL_9E_RG_RPHYPLL_BPB) | P_Fld(0x0, PLL_9E_RG_RPHYPLL_BPA));
            vIO32WriteFldAlign_All(DDRPHY_PLL8, 0 ,PLL8_RG_RPI_CAP_SEL);
        }
        else //if(p->freq_sel==LJ_DDR1600 || p->freq_sel==LJ_DDR800)
        {
            vIO32WriteFldMulti_All(DDRPHY_PLL1, P_Fld(0x0, PLL1_RG_RPHYPLL_EN) | P_Fld(0xe, PLL1_RG_RPHYPLL_FBKDIV) \
                                 | P_Fld(0x1, PLL1_RG_RPHYPLL_FBKSEL) | P_Fld(0x1, PLL1_RG_RPHYPLL_PREDIV) \
                                 | P_Fld(0x1, PLL1_RG_RPHYPLL_IC) | P_Fld(0xe, PLL1_RG_RPHYPLL_BP));
            vIO32WriteFldMulti_All(DDRPHY_PLL_9E, P_Fld(0x3, PLL_9E_RG_RPHYPLL_BR) | P_Fld(0x3, PLL_9E_RG_RPHYPLL_BC) \
                                | P_Fld(0xD, PLL_9E_RG_RPHYPLL_IR) | P_Fld(0x7, PLL_9E_RG_RPHYPLL_BPB) | P_Fld(0x2, PLL_9E_RG_RPHYPLL_BPA));
            vIO32WriteFldAlign_All(DDRPHY_PLL8, 1 ,PLL8_RG_RPI_CAP_SEL);
        }
        
        #if CHECK_PLL_OK
          if(is_pll_good() == 0) //not all pll is good, use 26M reference clock
          { 
              if(p->freq_sel==LJ_DDR1866 || p->freq_sel==LJ_DDR933 || p->freq_sel==LJ_DDR1700 || p->freq_sel==LJ_DDR850)
                  vIO32WriteFldAlign_All(DDRPHY_PLL1, 1, PLL1_RG_RPHYPLL_PREDIV);
              else
                  vIO32WriteFldAlign_All(DDRPHY_PLL1, 0, PLL1_RG_RPHYPLL_PREDIV);
          }      
        #endif  
        vIO32WriteFldAlign_All(DDRPHY_PLL1, 1, PLL1_RG_RPHYPLL_EN);
        
        #if 1
        {
            U8 confIdx = 0;
            U16 u2Band = 0x0;
            for(confIdx=0; confIdx<3; confIdx++)
            {
                U8 u1Pass = 0;
                U8 u1Fail = 0;
                
                #if CHECK_PLL_OK
                if(is_pll_good() == 0) 
                {
                    u1Pass = u4IO32ReadFldAlign(DDRPHY_PHY_RO_3+(confIdx<<POS_BANK_NUM), PHY_RO_3_RGS_RPHYPLL_AUTOK_PASS);
                    u1Fail = u4IO32ReadFldAlign(DDRPHY_PHY_RO_2+(confIdx<<POS_BANK_NUM), PHY_RO_2_RGS_RPHYPLL_AUTOK_FAIL);
                    u2Band = u4IO32ReadFldAlign(DDRPHY_PHY_RO_3+(confIdx<<POS_BANK_NUM), PHY_RO_3_RGS_RPHYPLL_AUTOK_BAND);
                    mcSHOW_DBG_MSG(("[LJ_PHYPLL_%d], PASS=%d, FAIL=%d, BAND=%B\n", confIdx, u1Pass, u1Fail, u2Band));
                    mcDELAY_US(1);
                }
                else  // if(is_pll_good() == 1) //all pll is good, use auto k band
                #endif
                {        
                    mcSHOW_DBG_MSG(("[LJ_PHYPLL_%d] waiting for K_Band\n", confIdx));
                    while(u1Pass==0 && u1Fail==0)
                    {
                        u1Pass = u4IO32ReadFldAlign(DDRPHY_PHY_RO_3+(confIdx<<POS_BANK_NUM), PHY_RO_3_RGS_RPHYPLL_AUTOK_PASS);
                        u1Fail = u4IO32ReadFldAlign(DDRPHY_PHY_RO_2+(confIdx<<POS_BANK_NUM), PHY_RO_2_RGS_RPHYPLL_AUTOK_FAIL);
                        u2Band = u4IO32ReadFldAlign(DDRPHY_PHY_RO_3+(confIdx<<POS_BANK_NUM), PHY_RO_3_RGS_RPHYPLL_AUTOK_BAND);
                        if(u1Pass)
                        {
                            vIO32WriteFldAlign(DDRPHY_PLL2+(confIdx<<POS_BANK_NUM), u2Band, PLL2_RG_RPHYPLL_BAND);
                            #if DFS_SUPPORT_THIRD_SHUFFLE
                            if(p->freq_sel==LJ_DDR1700 || p->freq_sel==LJ_DDR1866)
                            {
                                vIO32WriteFldAlign(DDRPHY_PLL4+(confIdx<<POS_BANK_NUM), u2Band, PLL4_RG_RCLRPLL_BAND);
                            }
                            else if(p->freq_sel==LJ_DDR1600)
                            {
                                vIO32WriteFldAlign(DDRPHY_SHUFFLE103+(confIdx<<POS_BANK_NUM), u2Band, PLL4_RG_RCLRPLL_BAND);
                            }
                            #endif
                        }
                        mcSHOW_DBG_MSG(("[LJ_PHYPLL_%d], PASS=%d, FAIL=%d, BAND=%B\n", confIdx, u1Pass, u1Fail, u2Band));
                        mcDELAY_US(1);
                    }
                }
                
            }
        }
        #else        
            //CONF-ABC
            for(i=0; i<DDRPHY_CONF_MAX; i++)
            {
                while(1)
                {
                    u4value = u4IO32ReadFldAlign(DDRPHY_PHY_RO_2+((U32)i<<POS_BANK_NUM), PHY_RO_2_RGS_RPHYPLL_AUTOK_FAIL);
                    if (u4value)
                    {
                        break;
                    }
                    u4value = u4IO32ReadFldAlign(DDRPHY_PHY_RO_3+((U32)i<<POS_BANK_NUM), PHY_RO_3_RGS_RPHYPLL_AUTOK_PASS);
                    if (u4value)
                    {
                        break;
                    }
                }
                mcDELAY_US(1);
            	//Save RGS_*_AUTOK_BAND
                u4value = u4IO32ReadFldAlign(DDRPHY_PHY_RO_3+((U32)i<<POS_BANK_NUM), PHY_RO_3_RGS_RPHYPLL_AUTOK_BAND);
                //Set RG_*PLL_BAND
            	vIO32WriteFldAlign(DDRPHY_PLL2+((U32)i<<POS_BANK_NUM), u4value, PLL2_RG_RPHYPLL_BAND);
            	vIO32WriteFldAlign(DDRPHY_PLL2+((U32)i<<POS_BANK_NUM), 0, PLL2_RG_RPHYPLL_AUTOK_EN);
            	mcSHOW_DBG_MSG(("[DdrPhyInit] PHYPLL_BAND(%d)=0x%X\n",i , u4value));
            	mcFPRINTF((fp_A60501, "[DdrPhyInit] PHYPLL_BAND(%d)=0x%X\n", i, u4value));
            }
        #endif
        vIO32WriteFldAlign_All(DDRPHY_PLL2, 0, PLL2_RG_RPHYPLL_AUTOK_EN);
        mcDELAY_US(100);//wait for load Band and PLL ready
    }
    else
    {
        //RCLRPLL_POSDIV= {RG_*_RCLRPLL_LBAND_EN, RG_*_RCLRPLL_POSDIV[1:0]}
        vIO32WriteFldMulti_All(DDRPHY_PLL3, P_Fld(0x0, PLL3_RG_RCLRPLL_EN) | P_Fld(0x50, PLL3_RG_RCLRPLL_FBKDIV) | \ 
                             P_Fld(0x1, PLL3_RG_RCLRPLL_PREDIV) | P_Fld(0x0, PLL3_RG_RCLRPLL_POSDIV) | \
                             P_Fld(0x0, PLL3_RG_RCLRPLL_FBKSEL));
        vIO32WriteFldMulti_All(DDRPHY_PLL4, P_Fld(0x0, PLL4_RG_RCLRPLL_LVROD_EN) | P_Fld(0x0, PLL4_RG_RCLRPLL_MONCK_EN) | \
                             P_Fld(0x0, PLL4_RG_RCLRPLL_MONREF_EN) | P_Fld(0x0, PLL4_RG_RCLRPLL_MONVC_EN) | \
                             P_Fld(0x0, PLL4_RG_RCLRPLL_AUTOK_EN)); //RCLRPLL_SDM_FRA_EN 
        vIO32WriteFldMulti_All(DDRPHY_PLL_9F, P_Fld(0x0, PLL_9F_RG_RCLRPLL_LBAND_EN)); //RCLRPLL_POSDIV[2] 
        //RCLRPLL_SDM_PCW_CHG = ?
        if(p->freq_sel==LC_DDR533 || p->freq_sel==LC_DDR635 || p->freq_sel==LC_DDR800)
        {
            vIO32WriteFldAlign_All(DDRPHY_PLL3, 0x1, PLL3_RG_RCLRPLL_POSDIV);
        }
#if PLL_JITTER_FINE_TUNE        
        if(p->freq_sel==LC_DDR1066 || p->freq_sel==LC_DDR533)
        {
            vIO32WriteFldAlign_All(DDRPHY_PLL3, 0x28,PLL3_RG_RCLRPLL_FBKDIV);//PLL freq : 520
            vIO32WriteFldAlign_All(DDRPHY_PLL8, 3 ,PLL8_RG_RPI_CAP_SEL);
        }
        else if(p->freq_sel==LC_DDR1270 || p->freq_sel==LC_DDR635)
        {
            vIO32WriteFldAlign_All(DDRPHY_PLL3, 0x31,PLL3_RG_RCLRPLL_FBKDIV);//PLL freq : 637
            vIO32WriteFldAlign_All(DDRPHY_PLL8, 3 ,PLL8_RG_RPI_CAP_SEL);
        }
        else //if(p->freq_sel==LC_DDR1600 || p->freq_sel==LC_DDR800)
        {
            vIO32WriteFldAlign_All(DDRPHY_PLL3, 0x3D,PLL3_RG_RCLRPLL_FBKDIV);//PLL freq : 799.5
            if(p->freq_sel==LC_DDR1600)
                vIO32WriteFldAlign_All(DDRPHY_PLL8, 1 ,PLL8_RG_RPI_CAP_SEL);
            else if(p->freq_sel==LC_DDR800)
                vIO32WriteFldAlign_All(DDRPHY_PLL8, 3 ,PLL8_RG_RPI_CAP_SEL);
        }
        
        #if CHECK_PLL_OK
            if(is_pll_good() == 0) //not all pll is good, use 26M reference clock
            {         
                vIO32WriteFldAlign_All(DDRPHY_PLL3, 0, PLL3_RG_RCLRPLL_PREDIV);//=> overwrite PREDIV
                vIO32WriteFldAlign_All(DDRPHY_PLL16, 1, Fld(1,12,AC_MSKB1));     
            }       
        #endif
#else
        if(p->freq_sel==LC_DDR1066 || p->freq_sel==LC_DDR533)
        {
            vIO32WriteFldAlign_All(DDRPHY_PLL3, 0x50,PLL3_RG_RCLRPLL_FBKDIV);//PLL freq : 520
            vIO32WriteFldAlign_All(DDRPHY_PLL8, 3 ,PLL8_RG_RPI_CAP_SEL);
        }
        else if(p->freq_sel==LC_DDR1270 || p->freq_sel==LC_DDR635)
        {
            vIO32WriteFldAlign_All(DDRPHY_PLL3, 0x62,PLL3_RG_RCLRPLL_FBKDIV);//PLL freq : 637
            vIO32WriteFldAlign_All(DDRPHY_PLL8, 3 ,PLL8_RG_RPI_CAP_SEL);
        }
        else //if(p->freq_sel==LC_DDR1600 || p->freq_sel==LC_DDR800)
        {
            vIO32WriteFldAlign_All(DDRPHY_PLL3, 0x7B,PLL3_RG_RCLRPLL_FBKDIV);//PLL freq : 799.5
            vIO32WriteFldAlign_All(DDRPHY_PLL8, 1 ,PLL8_RG_RPI_CAP_SEL);
        }
        
        #if CHECK_PLL_OK
            if(is_pll_good() == 0) //not all pll is good, use 26M reference clock       
                vIO32WriteFldAlign_All(DDRPHY_PLL3, 0, PLL3_RG_RCLRPLL_PREDIV);           
        #endif    
#endif

        //CLRPLL_EN
        vIO32WriteFldAlign_All(DDRPHY_PLL3, 1, PLL3_RG_RCLRPLL_EN);
        mcDELAY_US(22); //wait for PLL ready
    }
    
    // Test code: force PI_CAP_SEL = 2'b11
    #if 0
    vIO32WriteFldAlign_All(DDRPHY_PLL8, 0x3 ,PLL8_RG_RPI_CAP_SEL);
    #endif
    
    //PLL en [31]
    if(p->freq_sel < LJ_MAX_SEL)
    {
        vIO32WriteFldAlign_All(DDRPHY_PLL3, 0, PLL3_RG_RCLRPLL_EN);
    }
    else
    {
        #if CHECK_PLL_OK
            if(is_pll_good() == 1) //all pll is good, use auto k band
        #endif
            {
                vIO32WriteFldAlign_All(DDRPHY_PLL1, 0, PLL1_RG_RPHYPLL_EN);
            }        
    }
}

static DRAM_STATUS_T DdrPhySetting_Everest_LP3(DRAMC_CTX_T *p)
{
  /*TINFO="DQSINCTL = %h\n", DQSINCTL*/
  /*TINFO="R_DMDATLAT_DSEL = %h\n", R_DMDATLAT_DSEL*/
  /*TINFO="TX_DLY_DQSIN = %h\n", TX_DLY_DQSIN*/
  /*TINFO="DLY_DQSIN = %h\n", DLY_DQSIN*/


  /*TINFO="Enable DRAMC APB write-broadcast"*/
  *((volatile unsigned int *)(DRAMC_WBR)) = 0x7;
  //[2] ddrphy_conf write broadcast enable
  //[1] dramc_conf write boardcast enable
  //[0] dramc write broadcast enable

  //DRAM Translate On
  //
  //translate log to initial sequence
  //log format example:                95800 RSICWrite     ADDR    0707c    DATA    0x8583628d
  //
  //:%s/^\s\s*\(\S\S*\)\s\s*RSICWrite\s*ADDR\s*\<87\(\S\S*\)\>\s*DATA\s*\(\S\S*\)/\tRISCWriteDRAM(0x0\2,\t\3);\t\/\/ \1ns/g
  //:%s/^\s\s*\(\S\S*\)\s\s*RSICWrite\s*ADDR\s*\<87\(\S\S*\)\>\s*DATA\s*\(\S\S*\)/\tRISCWriteDDRPHY(0x0\2,\t\3);\t\/\/ \1ns/g
  //
  // --------------------------------------------------
  // Power configuration
  // --------------------------------------------------
  
// no power switch in K2, remove by jjsu on 20131213
//    RISCWriteDRAM(0x0640, 0x09008000);    // power_switch[15]
//    RISCWriteDRAM((0x0640+(0x0014<<2)), 0x09008000);
//    RISCWriteDRAM(0x0640, 0x09028000);    // power_sbs_en[16]
//    RISCWriteDRAM((0x0640+(0x0014<<2)), 0x09028000);
//    RISCWriteDRAM(0x0640, 0x09020000);    // power_switch[15]
//    RISCWriteDRAM((0x0640+(0x0014<<2)), 0x09020000);

  // --------------------------------------------------
  // MEMPLL configuration
  // --------------------------------------------------
// Added for one pll mode by jjsu on 20131213
//RISCWriteDRAM(0x0698, 0x00001e20);    // 4xck = VCO/3
RISCWriteDRAM(0x0698, 0x00001e41);    // 4xck = VCO/4, RG_MEMPLL_RSV[1]=1

//*MEM_DCM_CTRL =  (0x1<<28) | (0x1f << 21) | (0x1 << 9) | (0x1 << 8) | (0x1 << 7) |(0x1f << 1);
//*MEM_DCM_CTRL |= 0x1; // toggle enable
//*MEM_DCM_CTRL &= ~(0x1);

//mask by chiying@20140626
//// default sel 26M
//*DFS_MEM_DCM_CTRL = (0x1 << 28) | (0x1f << 21) | (0x0 << 16) | (0x1 << 9) | (0x1 << 8) | (0x1 << 7) | (0x1f << 1);
//*DFS_MEM_DCM_CTRL |= 0x1; // toggle enable
  
    //RISCWriteDRAM((0x0000001f << 2), 0xc00632f1);
    //RISCWriteDRAM(0x0080, 0x00070540);    // 114563ns, new DATLAT for TOP_PIPE, RX pipe [7:5]

//if (dram_type == 2) {
                /*TINFO="=== LP3 used ==="*/
  /*TINFO="=== SPM setting start ==="*/
  *((volatile unsigned int *)(DPY_PWR_CON)) = 0xd ;
  *((volatile unsigned int *)(DPY_CH1_PWR_CON)) = 0xd ;
  *((volatile unsigned int *)(SPM_PLL_CON)) = (*((volatile unsigned int *)(SPM_PLL_CON))) & ~(1<<4);
  *((volatile unsigned int *)(PCM_PWR_IO_EN)) = (*((volatile unsigned int *)(PCM_PWR_IO_EN))) & ~(1<<0) ;
  *((volatile unsigned int *)(SPM_POWER_ON_VAL0)) = (*((volatile unsigned int *)(SPM_POWER_ON_VAL0))) & ~(1<<2);
  /*TINFO="=== SPM setting end ==="*/
              
  /*TINFO="=== set ddrphy peri to ddrphy conf control ==="*/
    RISCWriteDDRPHY(0x0580 , 0x0);
    RISCWriteDDRPHY(0x0584 , 0x0);
    RISCWriteDDRPHY(0x0588 , 0x0);
    /*TINFO="=== set ddrphy peri to ddrphy conf control end==="*/
    
	//MPLL_MODE_SEL=1, 26M refrence
    #if CHECK_PLL_OK
      if(is_pll_good() == 0) //not all pll is good, use 26M reference clock
      {
	        *((volatile unsigned int *)(MPLL_CON0)) |= 1<<16;
	        /*TINFO="=== Select DPHY_RREF_CK=52MHz ==="*/
          *((volatile unsigned int *)(CLK_DDRPHY_REG)) = 0x00000001;        
      }
      else
    #endif    
      {
          /*TINFO="=== Select DPHY_RREF_CK=52MHz ==="*/
          *((volatile unsigned int *)(CLK_DDRPHY_REG)) = 0x00000002;        
      }

    ///TODO: EVEREST_PORTING_TODO
    //Need to confirm whether MPLL init cab by skipped in DDR RESERVE mode
    /*TINFO="=== MPLL init ==="*/
    *((volatile unsigned int *)(MPLL_PWR_CON0))  |= 0x1;
    //*MDM_TM_WAIT_US = 10;  // Wait 10us
    //while (*MDM_TM_WAIT_US>0);
    mcDELAY_US(10);
    *((volatile unsigned int *)(MPLL_PWR_CON0))  &= 0xFFFFFFFD;
    //*MDM_TM_WAIT_US = 1;  // Wait 1us
    //while (*MDM_TM_WAIT_US>0);
    mcDELAY_US(1);
    *((volatile unsigned int *)(MPLL_CON0))  |= 0x1;

  /*TINFO="=== Turn off MPLL SSC ==="*/
  //*FHCTL3_CFG = *FHCTL3_CFG & ~(1<<1);
  //*MPLL_CON1 = *FHCTL3_DDS;
  //*FHCTL_HP_EN = *FHCTL_HP_EN & ~(1<<3);

  /*TINFO="=== disable dramc/ddrphy/dramc_pipe MCK, freerun CK FR_CLK DCM & CG ==="*/
    (*((volatile unsigned int *)(DFS_MEM_DCM_CTRL))) &= ~((0x1 << 3) | (0x1 << 4) | (0x1 << 8) | (0x1 << 9) );
    
    /*TINFO="=== FB_CK divider setting Start==="*/
    *((volatile unsigned int *)(CLK_MEM_DFS_CFG)) = 0x00000100;
    
    *((volatile unsigned int *)(DRAMC_WBR)) = 0x3;
        /*TINFO="=== disable DMSUS==="*/
        //RISCWriteDDRPHY(0x058c , 0x3);
        RISCWrite((DDRPHY_BASE  + 0x058c  ) , 0x0);//3
        RISCWrite((DDRPHY1_BASE  + 0x058c  ) , 0x0);//3
        RISCWrite((DDRPHY2_BASE  + 0x058c  ) , 0x0);//2
        RISCWrite((DDRPHY3_BASE  + 0x058c  ) , 0x0);//2
    *((volatile unsigned int *)(DRAMC_WBR)) = 0x7;

    /*TINFO="=== wait 20 us for ddrphy settle==="*/
    //*MDM_TM_WAIT_US = 20;  // Wait 10us
    //while (*MDM_TM_WAIT_US>0);
    mcDELAY_US(20);
        //Everest 
//#ifndef BYPASS_APHY_INIT_FOR_CONN



    *((volatile unsigned int *)(DRAMC_WBR)) = 0x3;
        // 0417 ECO turn off CA VREF 
        //p_sequencer.regmodel.DDRPHY_blk[0].PLL15.write(status,           'h08080000);  // 0x10e turn off CA VREF 
        RISCWrite((DDRPHY_BASE  +  0x0438 ) , 0x08080000); 
        //
    
    *((volatile unsigned int *)(DRAMC_WBR)) = 0x7;



    //RISCWriteDDRPHY(0x0580 , 0xffffffff);
    //RISCWriteDDRPHY(0x0584 , 0xffffffff);
    //RISCWriteDDRPHY(0x0588 , 0xffffffff);

    //RISCWriteDDRPHY(0x058C , 0xffffffff);
        ////  Turn on OE_DIS
        //p_sequencer.regmodel.DDRPHY_blk[0].TXDQ0.write(status,    'h00000000);  //0x008 cfg0x02 OE_DIS //NEW_DSIM
        //p_sequencer.regmodel.DDRPHY_blk[0].TXDQ8.write(status,    'h00000000);  //0x068 cfg0x1a OE_DIS //NEW_DSIM
        //p_sequencer.regmodel.DDRPHY_blk[0].CMD10.write(status,    'h00008888);  //0x1a8 cfg0x6a OE_DIS //NEW_DSIM
        //p_sequencer.regmodel.DDRPHY_blk[1].TXDQ0.write(status,    'h00000000);  //cfg0x02 OE_DIS //NEW_DSIM
        //p_sequencer.regmodel.DDRPHY_blk[1].TXDQ8.write(status,    'h00000000);  //cfg0x1a OE_DIS //NEW_DSIM
        //p_sequencer.regmodel.DDRPHY_blk[1].CMD10.write(status,    'h00008888);  //cfg0x6a OE_DIS //NEW_DSIM
        //p_sequencer.regmodel.DDRPHY_blk[2].TXDQ0.write(status,    'h00000000);  //cfg0x02 OE_DIS //NEW_DSIM
        //p_sequencer.regmodel.DDRPHY_blk[2].TXDQ8.write(status,    'h00000000);  //cfg0x1a OE_DIS //NEW_DSIM
        //p_sequencer.regmodel.DDRPHY_blk[2].CMD10.write(status,    'h00008888);  //cfg0x6a OE_DIS //NEW_DSIM
        //p_sequencer.regmodel.DDRPHY_blk[3].TXDQ0.write(status,    'h00000000);  //cfg0x02 OE_DIS //NEW_DSIM
        //p_sequencer.regmodel.DDRPHY_blk[3].TXDQ8.write(status,    'h00000000);  //cfg0x1a OE_DIS //NEW_DSIM
        //p_sequencer.regmodel.DDRPHY_blk[3].CMD10.write(status,    'h00008888);  //cfg0x6a OE_DIS //NEW_DSIM

        RISCWriteDDRPHY(0x0008 , 0x00150000);//ODTEN DIS =1
        RISCWriteDDRPHY(0x01b8 , 0x83100000);//ODTEN DIS =1
        RISCWriteDDRPHY(0x0104 , 0x00004444);//PDB_PRE=01
        RISCWriteDDRPHY(0x0068 , 0x00110000);
        RISCWriteDDRPHY(0x01b0 , 0x00000400);//RG buff en /CMD BUFF_EN=0 CLK_BUFF_EN=0
        RISCWriteDDRPHY(0x044c ,0x00000040);//RG buff en//[6]IMPA_DDR3_SEL=1       
        RISCWriteDDRPHY(0x01bc , 0x00000201);
        //RISCWriteDDRPHY(0x01a8 , 0x00008888);
        RISCWriteDDRPHY(0x0278 , 0x03080733);//DDR1866 [15:14] RG_*_RPHYPLL_BP=1100
        
        RISCWriteDDRPHY(0x0100 , 0x00004444);//RG_TX_*_PDB_PRE* = 01
        RISCWriteDDRPHY(0x01a8 , 0x00154444);//RG_TX_*_PDB_PRE* = 01
    *((volatile unsigned int *)(DRAMC_WBR)) = 0x3;
        //*((UINT32P)(DDRPHY_BASE  + 0x05c0)) = 0x000000b0;
    //*((UINT32P)(DDRPHY1_BASE + 0x05c0)) = 0x000000c0;
    ////*((UINT32P)(DDRPHY2_BASE + 0x05c0)) = 0x00000080;
    //*((UINT32P)(DDRPHY2_BASE + 0x05c0)) = 0x00007080;

        //*((UINT32P)(DDRPHY_BASE  + 0x0128 )) = 0x0ffffff6;   //958000.0 ps  
        //*((UINT32P)(DDRPHY_BASE  + 0x012c )) = 0xf0000000;   //964000.0 ps  
        //*((UINT32P)(DDRPHY_BASE  + 0x0130 )) = 0xefb03cff;   //972000.0 ps  
        //*((UINT32P)(DDRPHY_BASE  + 0x0134 )) = 0xfff03cff;   //981000.0 ps  
        //*((UINT32P)(DDRPHY_BASE  + 0x0138 )) = 0x904fc300;   //990000.0 ps  
        //*((UINT32P)(DDRPHY_BASE  + 0x013c )) = 0x000fc300;   //999000.0 ps  

        //*((UINT32P)(DDRPHY_BASE  +  0x0128 )) = 0x20000006; // 958000.0 ps  
        RISCWrite((DDRPHY_BASE  +  0x0128 ) , 0x20000016); // 958000.0 ps  //ECO opt

        RISCWrite((DDRPHY_BASE  +  0x012c ) , 0x00000000); // 989000.0 ps  
        RISCWrite((DDRPHY_BASE  +  0x0130 ) , 0xefb03cff); // 1022000.0 ps 
        RISCWrite((DDRPHY_BASE  +  0x0134 ) , 0xffffffff); // 1054000.0 ps 
        RISCWrite((DDRPHY_BASE  +  0x0138 ) , 0xf04fc300); // 1087000.0 ps 
        RISCWrite((DDRPHY_BASE  +  0x013c ) , 0x00000000); // 1119000.0 ps 

        //*((UINT32P)(DDRPHY1_BASE  +  0x0128 )) = 0x00000009; // 1151000.0 ps 
        RISCWrite((DDRPHY1_BASE  +  0x0128 ) , 0x00000019); // 1151000.0 ps //ECO opt

        RISCWrite((DDRPHY1_BASE  +  0x012c ) , 0x20000000); // 1184000.0 ps 
        RISCWrite((DDRPHY1_BASE  +  0x0130 ) , 0xef003f1f); // 1216000.0 ps 
        RISCWrite((DDRPHY1_BASE  +  0x0134 ) , 0xffffffff); // 1249000.0 ps 
        RISCWrite((DDRPHY1_BASE  +  0x0138 ) , 0xf001c0e0); // 1281000.0 ps 
        RISCWrite((DDRPHY1_BASE  +  0x013c ) , 0x00000000); // 1313000.0 ps 

        //*((UINT32P)(DDRPHY2_BASE  +  0x0128 )) = 0x0000000a; // 1346000.0 ps 
        RISCWrite((DDRPHY2_BASE  +  0x0128 ) , 0x0000001a); // 1346000.0 ps //ECO opt

        RISCWrite((DDRPHY2_BASE  +  0x012c ) , 0x20000000); // 1378000.0 ps 
        RISCWrite((DDRPHY2_BASE  +  0x0130 ) , 0xef003c00); // 1411000.0 ps 
        RISCWrite((DDRPHY2_BASE  +  0x0134 ) , 0xffffffff); // 1443000.0 ps 
        RISCWrite((DDRPHY2_BASE  +  0x0138 ) , 0xf000001e); // 1475000.0 ps 
        RISCWrite((DDRPHY2_BASE  +  0x013c ) , 0x00000000); // 1508000.0 ps 

        //*((UINT32P)(DDRPHY3_BASE  +  0x0128 )) = 0x00000000; // 1540000.0 ps 
        RISCWrite((DDRPHY3_BASE  +  0x0128 ) , 0x00000010); // 1540000.0 ps  //ECO opt

        RISCWrite((DDRPHY3_BASE  +  0x012c ) , 0x00000000); // 1573000.0 ps 
        RISCWrite((DDRPHY3_BASE  +  0x0130 ) , 0xef003c1e); // 1605000.0 ps 
        RISCWrite((DDRPHY3_BASE  +  0x0134 ) , 0xffffffff); // 1637000.0 ps 
        RISCWrite((DDRPHY3_BASE  +  0x0138 ) , 0xf0000000); // 1670000.0 ps 
        RISCWrite((DDRPHY3_BASE  +  0x013c ) , 0x00000000); // 1702000.0 ps 

        RISCWrite((DDRPHY_BASE  + 0x05b8 ) , 0x00000000);   //1008000.0 ps 
        RISCWrite((DDRPHY1_BASE + 0x05b8 ) , 0x00000000);   //1017000.0 ps 
        RISCWrite((DDRPHY2_BASE + 0x05b8 ) , 0x00000000);   //1026000.0 ps 
        RISCWrite((DDRPHY3_BASE + 0x05b8 ) , 0x00000000);   //1035000.0 ps 
 
    ////With CA pinmux
    RISCWrite((DDRPHY_BASE  + 0x05c0 ) , 0x010041b8);   //1044000.0 ps 
        RISCWrite((DDRPHY1_BASE + 0x05c0 ) , 0x010032c8);   //1052000.0 ps 
    //NO CA pinmux
    //*((UINT32P)(DDRPHY_BASE  + 0x05c0 )) = 0x010040b8;   //1044000.0 ps 
        //*((UINT32P)(DDRPHY1_BASE + 0x05c0 )) = 0x010030c8;   //1052000.0 ps 
        //

        RISCWrite((DDRPHY2_BASE + 0x05c0 ) , 0x01007088);   //1061000.0 ps 
        RISCWrite((DDRPHY3_BASE + 0x05c0 ) , 0x01000088);   //1061000.0 ps 



    *((volatile unsigned int *)(DRAMC_WBR)) = 0x7;

        //RISCWriteDRAM(0x01c0 , 0x00100000);  //981000.0 ps  
        //RISCWriteDRAM(0x007c , 0x8583628d);  //990000.0 ps  
        //RISCWriteDRAM(0x0054 , 0x00000119);  //999000.0 ps  
        //trun off ddrphy dcm from dramc
        RISCWriteDDRPHY(0x0130 , 0x00000000);
        RISCWriteDDRPHY(0x0138 , 0x00000000);
        //trun off ddrphy dcm from dramc end
        RISCWriteDDRPHY(0x01c0 , 0x08000000);   //1070000.0 ps
        RISCWriteDDRPHY(0x01c0 , 0x08100000);   //1088000.0 ps 

  ///#ifdef DRAM_LP1333
  ///      /*TINFO="===  define DRAM_LP1333 ==="*/
  ///      RISCWriteDDRPHY(0x007c , 0x8583538d);   //1106000.0 ps 
  ///#else
  ///      #ifdef DRAM_LP1600
  ///      /*TINFO="===  define DRAM_LP1600 ==="*/
  ///       RISCWriteDDRPHY(0x007c , 0x8583528d);   //1106000.0 ps 
  ///      #else
  ///       //RISCWriteDDRPHY(0x007c , 0x8583628d);   //1106000.0 ps 
  ///       RISCWriteDDRPHY(0x007c , 0x8583638d);   //1106000.0 ps 
  ///      #endif
  ///#endif

        //RISCWriteDDRPHY(0x0054 , 0x00000119);   //1115000.0 ps 

        RISCWriteDDRPHY(0x0018 , 0x00001010);   //1141000.0 ps 
        RISCWriteDDRPHY(0x001c , 0x8110000e);   //1177000.0 ps //[31]DVS EN

    //RISCWriteDDRPHY(0x0020 , 0x010c1011);   //1221000.0 ps 
    RISCWriteDDRPHY(0x0020 , 0x010c4211);   //1221000.0 ps  //bufen dynamic //[15:14] VREF_SEL=01, [12]DQ_BIAS_EN =0 [9]O1_SEL=1


        RISCWriteDDRPHY(0x001c , 0x8311000e);   //1257000.0 ps //[31]DVS EN//VREF EN

#if 0  // change RX default value for 1866, don't use simulation
        // RK0 RX
        RISCWriteDDRPHY(0x003c , 0x00001010);   //1292000.0 ps 
        RISCWriteDDRPHY(0x009c , 0x00001010);   //1505000.0 ps 

        // RK1 RX
        RISCWriteDDRPHY(0x0058 , 0x00000f0f);   //1328000.0 ps 
        RISCWriteDDRPHY(0x00b8 , 0x00000f0f);   //1541000.0 ps 
#endif

        RISCWriteDDRPHY(0x0078 , 0x00001010);   //1363000.0 ps 
        RISCWriteDDRPHY(0x007c , 0x8110000e);   //1399000.0 ps [31]DVS EN

        //RISCWriteDDRPHY(0x0080 , 0x010c1011);   //1434000.0 ps
        RISCWriteDDRPHY(0x0080 , 0x010c4211);   //1434000.0 ps //bufen dynamic[15:14] VREF_SEL=01,[12]DQ_BIAS_EN =0 [9]O1_SEL=1

        RISCWriteDDRPHY(0x007c , 0x8311000e);   //1470000.0 ps [31]DVS EN//VREF EN

        RISCWriteDDRPHY(0x0180 , 0x00000000);   //1576000.0 ps 

        //RISCWriteDDRPHY(0x0184 , 0x00000010);   //1612000.0 ps 
        RISCWriteDDRPHY(0x0184 , 0x00000414);   //1612000.0 ps   //ECO CKE //[11:10] VREF_SEL=01

        RISCWriteDDRPHY(0x0188 , 0x00000ae2);   //1647000.0 ps [11] CMD_DDR3_SEL =1

        //LP-1600-52M
        RISCWriteDDRPHY(0x0400 , 0x0e1712fd);   //1683000.0 ps  //upd A-PHY model

        //RISCWriteDDRPHY(0x0404 , 0x81c01004);   //1718000.0 ps
        *((volatile unsigned int *)(DRAMC_WBR)) = 0x3;
        #if 0
          RISCWrite((DDRPHY_BASE  + 0x0404 ) , 0x81240006);   //22176000.0 ps
        RISCWrite((DDRPHY1_BASE + 0x0404 ) , 0x81240003);   //22182000.0 ps
        RISCWrite((DDRPHY2_BASE + 0x0404 ) , 0x81240003);   //22191000.0 ps
        RISCWrite((DDRPHY3_BASE + 0x0404 ) , 0x81240003);   //22200000.0 ps//[12]PHYPLL_LVROD_EN=0
        #else // test code
        //RISCWrite((DDRPHY_BASE  + 0x0404 ) , 0x8fA40006);   //22176000.0 ps
        RISCWrite((DDRPHY_BASE  + 0x0404 ) , 0x8f240006);   //22176000.0 ps
        RISCWrite((DDRPHY1_BASE + 0x0404 ) , 0x8f240003);   //22182000.0 ps
        RISCWrite((DDRPHY2_BASE + 0x0404 ) , 0x8f240003);   //22191000.0 ps
        RISCWrite((DDRPHY3_BASE + 0x0404 ) , 0x8f240003);   //22200000.0 ps//[12]PHYPLL_LVROD_EN=0
        #endif
        *((volatile unsigned int *)(DRAMC_WBR)) = 0x7;


        //RISCWriteDDRPHY(0x0408 , 0x500012fd); //LC-1066-52M
        //RISCWriteDDRPHY(0x0408 , 0x500412fd); //LC-1066-26M
        //RISCWriteDDRPHY(0x0408 , 0x600412fd); //LC-1270-52M
        RISCWriteDDRPHY(0x0408 , 0x7b0412fd); //1600-52M

        //CAP_SEL (1700=0, 1600=1, 1270=3, 1066=3)
        RISCWriteDDRPHY(0x041c , 0x1000000);
                
        RISCWriteDDRPHY(0x0420 , 0x00000000);   //1825000.0 ps 
        //RISCWriteDDRPHY(0x0430 , 0x00660600);   //1860000.0 ps
        *((volatile unsigned int *)(DRAMC_WBR)) = 0x3;
          RISCWrite((DDRPHY_BASE  + 0x0430 ) , 0x00690600);   //22176000.0 ps
        RISCWrite((DDRPHY1_BASE + 0x0430 ) , 0x00790600);   //22182000.0 ps
        RISCWrite((DDRPHY2_BASE + 0x0430 ) , 0x00790600);   //22191000.0 ps
        RISCWrite((DDRPHY3_BASE + 0x0430 ) , 0x00790600);   //22200000.0 ps
        *((volatile unsigned int *)(DRAMC_WBR)) = 0x7;          
        RISCWriteDDRPHY(0x0434 , 0x00690600);   //1896000.0 ps 
        RISCWriteDDRPHY(0x0450 , 0x00000000);   //1932000.0 ps 
        RISCWriteDDRPHY(0x0458 , 0x00100000);   //1967000.0 ps 
        RISCWriteDDRPHY(0x0460 , 0x00001010);   //2003000.0 ps 
        RISCWriteDDRPHY(0x0468 , 0x00001010);   //2038000.0 ps 

        //PHYPLL setting
        //RPHYPLL_BP =  {RG_*_RPHYPLL_BPB[1:0], RG_*_RPHYPLL_BPA[1:0]}       
        *((volatile unsigned int *)(DRAMC_WBR)) = 0x3;
        
        #if DDRPHY_LOWPOWER_ENABLE
        //RG_CB_RPHYPLL_TSTOP_EN, Change to R mode(1'b0) before PLL init
        vIO32WriteFldAlign(DDRPHY_PLL15+(1<<POS_BANK_NUM), 0 , PLL15_RG_RPHYPLL_TSTOP_EN);
        #endif
        
        DDRPhyPLLSetting(p);
        
        *((volatile unsigned int *)(DRAMC_WBR)) = 0x7;


#if PLL_JITTER_FINE_TUNE //bit 12 : LC FBK_SEL
        U8 u1PGain =8;
        RISCWriteDDRPHY(0x043c , 0x00111000 +((u1PGain>>3)<<10) + ((u1PGain&7)<<1));   //22120000.0 ps   //FBK_SEL =1
        //RISCWriteDDRPHY(0x043c , 0x00111000);   //22120000.0 ps   //FBK_SEL =1
        //RISCWriteDDRPHY(0x043c , 0x00110000);   //22120000.0 ps//FBK_SEL =0
#else
        RISCWriteDDRPHY(0x043c , 0x00110000);   //22120000.0 ps
#endif
        //disable ddrphy conf boardcast
        ///*TINFO="disable DDRPHY CONF APB write-broadcast"*/
        
        *((volatile unsigned int *)(DRAMC_WBR)) = 0x3;
        if(p->freq_sel > LJ_MAX_SEL) //LCPLL, mpdiv_in_sel=1
        {
            RISCWrite((DDRPHY_BASE  + 0x042c ) , 0x00038c0c);   //22176000.0 ps//change to MPDIV CLRPLL
            RISCWrite((DDRPHY1_BASE + 0x042c ) , 0xf0038c0c);   //22182000.0 ps
            RISCWrite((DDRPHY2_BASE + 0x042c ) , 0xf0038c0c);   //22191000.0 ps
            RISCWrite((DDRPHY3_BASE + 0x042c ) , 0xf0038c0c);   //22200000.0 ps 
            
            //*MDM_TM_WAIT_US = 1; //WAIT 20us  //upd A-PHY model wait MPDIV
            //  while (*MDM_TM_WAIT_US>0); //
            mcDELAY_US(1);

    		if(p->freq_sel==LJ_DDR933 || p->freq_sel==LJ_DDR850 || p->freq_sel==LJ_DDR800)
    		//|| p->freq_sel==LC_DDR533 || p->freq_sel==LC_DDR635 || p->freq_sel==LC_DDR800)
            {
                //mpdiv2
                RISCWrite((DDRPHY_BASE  + 0x042c ) , 0x00038505);   //22176000.0 ps//change to MPDIV CLRPLL
                RISCWrite((DDRPHY1_BASE + 0x042c ) , 0xf0038505);   //22182000.0 ps
                RISCWrite((DDRPHY2_BASE + 0x042c ) , 0xf0038505);   //22191000.0 ps
                RISCWrite((DDRPHY3_BASE + 0x042c ) , 0xf0038505);   //22200000.0 ps         
            }
            else
            {
                RISCWrite((DDRPHY_BASE  + 0x042c ) , 0x00038000);   //22176000.0 ps//change to MPDIV CLRPLL
                RISCWrite((DDRPHY1_BASE + 0x042c ) , 0xf0038000);   //22182000.0 ps
                RISCWrite((DDRPHY2_BASE + 0x042c ) , 0xf0038000);   //22191000.0 ps
                RISCWrite((DDRPHY3_BASE + 0x042c ) , 0xf0038000);   //22200000.0 ps         
    		}
        }
        else //LJPLL, mpdiv_in_sel=0
        {
            RISCWrite((DDRPHY_BASE  + 0x042c ) , 0x00008c0c);   //22176000.0 ps//[7] BRPI_MPDIV_EN off
            RISCWrite((DDRPHY1_BASE + 0x042c ) , 0xf0008c0c);   //22182000.0 ps//change to MPDIV PHYPLL
            RISCWrite((DDRPHY2_BASE + 0x042c ) , 0xf0008c0c);   //22191000.0 ps
            RISCWrite((DDRPHY3_BASE + 0x042c ) , 0xf0008c0c);   //22200000.0 ps 

            //*MDM_TM_WAIT_US = 1; //WAIT 20us  //upd A-PHY model wait MPDIV
            // while (*MDM_TM_WAIT_US>0); //
            mcDELAY_US(1);

    		if(p->freq_sel==LJ_DDR933 || p->freq_sel==LJ_DDR850 || p->freq_sel==LJ_DDR800)
    		//|| p->freq_sel==LC_DDR533 || p->freq_sel==LC_DDR635 || p->freq_sel==LC_DDR800)
    		{
                RISCWrite((DDRPHY_BASE  + 0x042c ) , 0x00008505);   //22176000.0 ps//[7] BRPI_MPDIV_EN off
                RISCWrite((DDRPHY1_BASE + 0x042c ) , 0xf0008505);   //22182000.0 ps//change to MPDIV PHYPLL
                RISCWrite((DDRPHY2_BASE + 0x042c ) , 0xf0008505);   //22191000.0 ps
                RISCWrite((DDRPHY3_BASE + 0x042c ) , 0xf0008505);   //22200000.0 ps                         
            }
            else
            {
                RISCWrite((DDRPHY_BASE  + 0x042c ) , 0x00008000);   //22176000.0 ps//[7] BRPI_MPDIV_EN off
                RISCWrite((DDRPHY1_BASE + 0x042c ) , 0xf0008000);   //22182000.0 ps//change to MPDIV PHYPLL
                RISCWrite((DDRPHY2_BASE + 0x042c ) , 0xf0008000);   //22191000.0 ps
                RISCWrite((DDRPHY3_BASE + 0x042c ) , 0xf0008000);   //22200000.0 ps                             		
            }
        }
        RISCWrite((DDRPHY_BASE  + 0x0424 ) , 0x000ffdfe);   //22213000.0 ps//[0]turn off CLKIEN
        RISCWrite((DDRPHY1_BASE + 0x0424 ) , 0x0007fdfe);   //22220000.0 ps//turn off CLKIEN
        RISCWrite((DDRPHY2_BASE + 0x0424 ) , 0x0007fdfe);   //22229000.0 ps//turn off CLKIEN
        RISCWrite((DDRPHY3_BASE + 0x0424 ) , 0x0007fdfe);   //22238000.0 ps//turn off CLKIEN

        *((volatile unsigned int *)(DRAMC_WBR)) = 0x7;


    // YT begin: mem_clkmux    
    *((volatile unsigned int *)(CLK_CFG_0))  = (*((volatile unsigned int *)(CLK_CFG_0))) | 0x00000100 ; 
        *((volatile unsigned int *)(CLK_CFG_UPDATE)) = 0x00000002;
        // YT end : mem_clkmux   
  
      //chi-ying begin: dramc rate selection @20140626
      ///*TINFO="Set DRAMC free clock division rate...begin"*/

      ////enable dramc cg
      //*DFS_MEM_DCM_CTRL |=  (0x3 <<3);

      ////enable dramc pipe cg (new ECO)
      //*DFS_MEM_DCM_CTRL |=  (0x1 <<8); 

      ////enable dramc freerun, pipe slowdown cg (new ECO)
      //*DFS_MEM_DCM_CTRL |=  (0x1 <<9); 

      ////select rate: 0:DIV1,1:DIV16,2:DIV32,3:DIV64
      //*DFS_MEM_DCM_CTRL |=  (0x1 <<1);
      //*DFS_MEM_DCM_CTRL &=  ~(0x1 <<0);
  
      ////change rate enable
      //*DFS_MEM_DCM_CTRL |= (0x1<<2);
  
      ////change rate disable
      //*DFS_MEM_DCM_CTRL &= ~(0x1<<2);
   
      ///*TINFO="Set DRAMC free clock division rate...end"*/
      //chi-ying end

      //RISCWriteDDRPHY(0x0014 , 0x000007fe); //22051000.0 ps
      ////RISCWriteDDRPHY(0x0014 , 0x000007fe); //22060000.0 ps
      ////RISCWriteDDRPHY(0x0014 , 0x000007fe); //22069000.0 ps
      ////RISCWriteDDRPHY(0x0014 , 0x000007fe); //22078000.0 ps
      //RISCWriteDDRPHY(0x0074 , 0x000007fe); //22087000.0 ps
      ////RISCWriteDDRPHY(0x0074 , 0x000007fe); //22096000.0 ps
      ////RISCWriteDDRPHY(0x0074 , 0x000007fe); //22105000.0 ps
      ////RISCWriteDDRPHY(0x0074 , 0x000007fe); //22114000.0 ps
      //RISCWriteDDRPHY(0x0430 , 0x00660e00); //22122000.0 ps
      //RISCWriteDDRPHY(0x0430 , 0x00660e00); //22131000.0 ps
      //RISCWriteDDRPHY(0x0430 , 0x00660e00); //22140000.0 ps
      //RISCWriteDDRPHY(0x0430 , 0x00660e00); //22149000.0 ps

#if (!PLL_JITTER_FINE_TUNE)
        RISCWriteDDRPHY(0x043c , 0x00110000);   //22120000.0 ps
#endif
        //RISCWriteDDRPHY(ADDR=4043c with 0x00110000);   //22127000.0 ps
        //RISCWriteDDRPHY(ADDR=6043c with 0x00110000);   //22136000.0 ps
        //RISCWriteDDRPHY(ADDR=8043c with 0x00110000);   //22145000.0 ps

        *((volatile unsigned int *)(DRAMC_WBR)) = 0x3;

        RISCWrite((DDRPHY_BASE  + 0x0424 ) , 0x000ffdfe);   //22213000.0 ps
        RISCWrite((DDRPHY1_BASE + 0x0424 ) , 0x0007fdfe);   //22220000.0 ps
        RISCWrite((DDRPHY2_BASE + 0x0424 ) , 0x0007fdfe);   //22229000.0 ps
        RISCWrite((DDRPHY3_BASE + 0x0424 ) , 0x0007fdfe);   //22238000.0 ps
#if PLL_JITTER_FINE_TUNE
        U8 aru1DLLGain[4];
        aru1DLLGain[0] = aru1DLLGain[1] = aru1DLLGain[2] = aru1DLLGain[3] =7;

        RISCWrite((DDRPHY_BASE  + 0x0430 ) , 0x00090600+ (aru1DLLGain[0] << 20));   //22247000.0 ps
        RISCWrite((DDRPHY1_BASE + 0x0430 ) , 0x00090600+ (aru1DLLGain[1] << 20));   //22256000.0 ps
        RISCWrite((DDRPHY2_BASE + 0x0430 ) , 0x00090600+ (aru1DLLGain[2] << 20));   //22264000.0 ps
        RISCWrite((DDRPHY3_BASE + 0x0430 ) , 0x00090600+ (aru1DLLGain[3] << 20));   //22273000.0 ps
        //CA PHDET_EN
        RISCWrite((DDRPHY_BASE  + 0x0430 ) , 0x00090e00+ (aru1DLLGain[0] << 20));   //22282000.0 ps
#else
        RISCWrite((DDRPHY_BASE  + 0x0430 ) , 0x00690600);   //22247000.0 ps
        RISCWrite((DDRPHY1_BASE + 0x0430 ) , 0x00790600);   //22256000.0 ps
        RISCWrite((DDRPHY2_BASE + 0x0430 ) , 0x00790600);   //22264000.0 ps
        RISCWrite((DDRPHY3_BASE + 0x0430 ) , 0x00790600);   //22273000.0 ps
        //CA PHDET_EN
        RISCWrite((DDRPHY_BASE  + 0x0430 ) , 0x00690e00);   //22282000.0 ps
#endif

        //wait AD_ARDLL_PD_EN_CA go low
        //  for (i=0; i < 1 ; ++i)
        //      ;
    //    *MDM_TM_WAIT_US = 1; //WAIT 1
    //while (*MDM_TM_WAIT_US>0); //
    mcDELAY_US(1);

#if PLL_JITTER_FINE_TUNE        
        RISCWrite((DDRPHY1_BASE + 0x0430 ) , 0x00090e00 + (aru1DLLGain[1] << 20));   //22402000.0 ps
        RISCWrite((DDRPHY2_BASE + 0x0430 ) , 0x00090e00 + (aru1DLLGain[2] << 20));   //22409000.0 ps
        RISCWrite((DDRPHY3_BASE + 0x0430 ) , 0x00090e00 + (aru1DLLGain[3] << 20));   //22418000.0 ps
#else
        RISCWrite((DDRPHY1_BASE + 0x0430 ) , 0x00790e00);   //22402000.0 ps
        RISCWrite((DDRPHY2_BASE + 0x0430 ) , 0x00790e00);   //22409000.0 ps
        RISCWrite((DDRPHY3_BASE + 0x0430 ) , 0x00790e00);   //22418000.0 ps
    #endif
        RISCWrite((DDRPHY_BASE  + 0x0434 ) , 0x00690e00);   //22427000.0 ps
        RISCWrite((DDRPHY1_BASE + 0x0434 ) , 0x00690e00);   //22435000.0 ps
        RISCWrite((DDRPHY2_BASE + 0x0434 ) , 0x00690e00);   //22444000.0 ps
        RISCWrite((DDRPHY3_BASE + 0x0434 ) , 0x00690e00);   //22453000.0 ps

        *((volatile unsigned int *)(DRAMC_WBR)) = 0x7;

        //wait all DLL PD EN ready
    //for (i=0; i < 1 ; ++i)
    //    ;
    //    *MDM_TM_WAIT_US = 1; 
    //while (*MDM_TM_WAIT_US>0); 
    mcDELAY_US(1);

        //ECO CKE glitch
        *((volatile unsigned int *)(DRAMC_WBR)) = 0x3;

        //p_sequencer.regmodel.DDRPHY_blk[0].CMD2.write(status,   'h0000e410);  //Turn off CMD_PULL_DN //Mv after PHDET 
        //p_sequencer.regmodel.DDRPHY_blk[1].CMD2.write(status,   'h0000e410);  //Turn off CMD_PULL_DN //Mv after PHDET
        RISCWrite((DDRPHY_BASE  +  0x0184 ) , 0x0000e410); 
        RISCWrite((DDRPHY1_BASE +  0x0184 ) , 0x0000e410); 

        *((volatile unsigned int *)(DRAMC_WBR)) = 0x7;
    //

        //RISCWriteDDRPHY(0x0014 , 0x000007fe); //22051000.0 ps
        //RISCWriteDDRPHY(0x0074 , 0x000007fe); //22087000.0 ps
        //RISCWriteDDRPHY(0x0014 , 0x000007fe);    //22573000.0 ps
        RISCWriteDDRPHY(0x0014 , 0x0000051e);    //22573000.0 ps  // RG_BUFEN=0


        //RISCWriteDDRPHY(0x40014 , 0x000007fe);    //22580000.0 ps
        //RISCWriteDDRPHY(0x60014 , 0x000007fe);    //22589000.0 ps
        //RISCWriteDDRPHY(0x80014 , 0x000007fe);    //22597000.0 ps
        //RISCWriteDDRPHY(0x0074 , 0x000007fe);    //22606000.0 ps

    //RISCWriteDDRPHY(0x0074 , 0x000007fe);    //22606000.0 ps
    RISCWriteDDRPHY(0x0074 , 0x0000051e);    //22606000.0 ps  //RG BUFEN=0


        //RISCWriteDDRPHY(0x40074 , 0x000007fe);    //22615000.0 ps
        //RISCWriteDDRPHY(0x60074 , 0x000007fe);    //22624000.0 ps
        //RISCWriteDDRPHY(0x80074 , 0x000007fe);    //22633000.0 ps


        //ARCLK ARCMD OE
        *((volatile unsigned int *)(DRAMC_WBR)) = 0x3;

        //*((UINT32P)(DDRPHY_BASE  + 0x05c0)) = 0x000000b8;//22193000.0 ps
        //*((UINT32P)(DDRPHY1_BASE + 0x05c0)) = 0x000000c8;//22202000.0 ps
        //*((UINT32P)(DDRPHY_BASE  + 0x05c0)) = 0x000000bc;//22211000.0 ps
        //*((UINT32P)(DDRPHY1_BASE + 0x05c0)) = 0x000000cc;//22220000.0 ps

        //*((UINT32P)(DDRPHY_BASE   + 0x05c0)) =  0x010000b8;   //22642000.0 ps
        //*((UINT32P)(DDRPHY1_BASE  + 0x05c0)) =  0x010000c8;   //22651000.0 ps

        ////CA pinmux
    RISCWrite((DDRPHY_BASE   + 0x05c0) ,  0x010041bc);   //22660000.0 ps
        RISCWrite((DDRPHY1_BASE  + 0x05c0) ,  0x010032cc);   //22669000.0 ps
        //No CA pinmux
    //*((UINT32P)(DDRPHY_BASE   + 0x05c0)) =  0x010040bc;   //22660000.0 ps
        //*((UINT32P)(DDRPHY1_BASE  + 0x05c0)) =  0x010030cc;   //22669000.0 ps


        *((volatile unsigned int *)(DRAMC_WBR)) = 0x7;

    //RISCWriteDDRPHY(0x0074 , 0x000005fe);   //23790000.0 ps
    RISCWriteDDRPHY(0x0074 , 0x0000051e);   //23790000.0 ps
    //RISCWriteDDRPHY(0x0014 , 0x000005fe);   //23823000.0 ps
    RISCWriteDDRPHY(0x0014 , 0x0000051e);   //23823000.0 ps

    //Add after finail a-phy model for low-power? -->DSIM_0320
    RISCWriteDDRPHY(0x0014 , 0x001205fc); //RISCWriteDDRPHY(0x0014 , 0x0002051e); //TXDQ3.write(status, 'h0002051e); //rank_sel eco option //buff en
    RISCWriteDDRPHY(0x0074 , 0x001205fc); //RISCWriteDDRPHY(0x0074 , 0x0002051e); //RXDQ13.write(status,'h0002051e); //rank_sel eco option//buff en


    //0310 rxdly lowpower
    //RISCWriteDDRPHY(0x04c4 , 0x80000000); //rk0 rx dly tracking hw mode
    RISCWriteDDRPHY(0x04c4 , 0x00000000);   //rk0 rx dly tracking rg mode
    //RISCWriteDDRPHY(0x0504 , 0x80000000);//rk1 rx dly tracking hw mode
    RISCWriteDDRPHY(0x0504 , 0x00000000);  //rk1 rx dly tracking rg mode
    RISCWriteDDRPHY(0x0544 , 0x80000000);   

    RISCWriteDDRPHY(0x04dc , 0x80000000);   
    RISCWriteDDRPHY(0x04e4 , 0x80000000);   
    RISCWriteDDRPHY(0x051c , 0x80000000);   
    RISCWriteDDRPHY(0x0524 , 0x80000000);  

    //*((volatile unsigned int *)(DRAMC_WBR)) = 0x3;
#if 1 // PLL_CA tracking leaf mode.
    vIO32WriteFldAlign_All((DDRPHY_PLL2+ (0<<POS_BANK_NUM)), 0, PLL2_RG_ARDLL_PD_CK_SEL);
    vIO32WriteFldAlign_All((DDRPHY_PLL12+ (0<<POS_BANK_NUM)), 1, PLL12_RG_ARDLL_PHDET_OUT_SEL);
    vIO32WriteFldAlign_All((DDRPHY_PLL12+ (0<<POS_BANK_NUM)), 1, PLL12_RG_ARDLL_PHDET_IN_SWAP);
    vIO32WriteFldAlign_All((DDRPHY_PLL2+ (0<<POS_BANK_NUM)), 1, PLL2_RG_ARDLL_FASTPJ_CK_SEL);
    vIO32WriteFldAlign_All((DDRPHY_PLL13+ (0<<POS_BANK_NUM)), 7, PLL13_RG_ARDLL_GAIN);   
    vIO32WriteFldAlign_All((DDRPHY_PLL5+ (0<<POS_BANK_NUM)), 0, PLL5_RG_RPHYPLL_EXT_FBDIV);
#endif
    *((volatile unsigned int *)(DRAMC_WBR)) = 0x7;

#if LP3_JV_WORKAROUND
        if(p->frequency >=800)
        {
            //set TX CLK/CS/CA delay line to max(0xf)      
            vIO32WriteFldAlign((DDRPHY_CMD5), 0xffffffff, PHY_FLD_FULL);
            vIO32WriteFldAlign((DDRPHY_CMD6), 0xff000fff, PHY_FLD_FULL);
            vIO32WriteFldAlign((DDRPHY_CMD7), 0x0ff00000, PHY_FLD_FULL);
            vIO32WriteFldAlign((DDRPHY_CMD7_1_CA), 0xff000000, PHY_FLD_FULL);
        }
        else
        {
            //set TX CLK/CS/CA delay line to default(0)
            vIO32WriteFldAlign((DDRPHY_CMD5), 0x00000000, PHY_FLD_FULL);
            vIO32WriteFldAlign((DDRPHY_CMD6), 0x00000000, PHY_FLD_FULL);
            vIO32WriteFldAlign((DDRPHY_CMD7), 0x00000000, PHY_FLD_FULL);
            vIO32WriteFldAlign((DDRPHY_CMD7_1_CA), 0x00000000, PHY_FLD_FULL);
        }
#endif

    *((volatile unsigned int *)(DRAMC_WBR)) = 0x3;

    #if PHY_MONCLK_SETTING_TEST
    //vIO32WriteFldAlign_All(DDRPHY_PLL4, 1, PLL4_RG_RCLRPLL_LVROD_EN); 
    DdrPhyMonclkSetting(p); // For Justin test.
    #endif

    vIO32WriteFldAlign_All((DDRPHY_PLL13), 0x7, PLL13_RG_ARDLL_IDLECNT);
    vIO32WriteFldAlign_All((DDRPHY_PLL14), 0x7, PLL14_RG_BRDLL_IDLECNT);
    
    #if JADE_TRACKING_MODE
    vIO32WriteFldAlign_All((DDRPHY_PLL16), 0x38, Fld(6,4,AC_MSKW10));
    #endif
}


#if COMPILE_A60817_LP3_INIT_CODE
static DRAM_STATUS_T DdrPhySetting_LP3(DRAMC_CTX_T *p)
{
        U32 u4value;

        // SYS_REG                                                    
        // [29:28] Control Uart to Access which Target, 00 -> chA?    
        //     [P]support UART Memory R/W 時,要對哪一個 channel 發memory R/W, 
        //        因為test chip目前UART沒有 Memory R/W 只有 I/O,所以可以不管
        // [10:8] CTO_MUX_SEL*, 1 -> Test Engine 0 -> UART            
        //     [P]是用來選 DRAMC的外部Test Agent source, CTO Test Agent或UART,
        //        目前固定選 CTO Test Agent. 因為 UART 不會發Memory Request 
        vIO32Write4B(mcSET_SYS_REG_ADDR(0x0020), 0x00000700);         
        //mcFPRINTF((fp_A60501, "RISCWrite : address %05x data %08x wait  0\n", mcSET_SYS_REG_ADDR(0x0020)&0x000fffff, 0x00000700));
                                                                      
        // NA in regmap                                               
        // [P]target到dramc_conf(AO) Offset 149,因為沿用舊的 pattern (Enable reg_clk_0, reg_4x_mode, PHY_M_CK),
        //    此 offset  再 60501 沒有作用. 可以 skip 掉              
        //RISCWrite : address 60524 data 60008000 wait  0             
                                                                      
        // PLL GP (MEMPLL)                                            
        // Fvco=26M*76=1976M; 1976/2=988M; 988/(18+1)=52MHz           
        // MEMPLL_SDM_PCW, 8 bits integer & 24 bits fraction          
        vIO32Write4B(mcSET_SYS_REG_ADDR(0x2000), 0x4c000000);         
        //mcFPRINTF((fp_A60501, "RISCWrite : address %05x data %08x wait  0\n", mcSET_SYS_REG_ADDR(0x2000)&0x000fffff, 0x4c000000));
                                                                      
        // [10] MEMPLL_BYPASS, 0 -> MEMPLL, 1 -> XTAL, need to toggle?
        // [9] MEMPLL_EXTPODIV_EN                                     
        // [8] MEMPLL_RESETB, NA in timing diagram                    
        // [5:0] MEMPLL_EXT_PODIV (6'd18): /19                        
        vIO32Write4B(mcSET_SYS_REG_ADDR(0x201c), 0x00000412);         
        //mcFPRINTF((fp_A60501, "RISCWrite : address %05x data %08x wait  0\n", mcSET_SYS_REG_ADDR(0x201c)&0x000fffff, 0x00000412));
        // debug, temp marked
        vIO32Write4B(mcSET_SYS_REG_ADDR(0x201c), 0x00000312);         
        //mcFPRINTF((fp_A60501, "RISCWrite : address %05x data %08x wait  0\n", mcSET_SYS_REG_ADDR(0x201c)&0x000fffff, 0x00000312));
                                                                      
        // [0] MEMPLL_SDM_PCW_CHG (1)                                 
        // [1] MEMPLL_EN (1)                                          
        // [6:4] MEMPLL_POSDIV (001)                                  
        // [9:8] MEMPLL_PREDIV (00)                                   
        // [15] MEMPLL_SDM_FRA_EN (1)                                 
        // [16] MEMPLL_SDM_HREN (1)                                   
        // [17] MEMPLL_LVROD_EN (0)                                   
        // [18] MEMPLL_BP (1)                                         
        // [19] MEMPLL_BR (1)                                         
        // [20] MEMPLL_BLP (1)                                        
        // [25:24] MEMPLL_RST_DLY (00)                                
        // [26] MEMPLL_SDM_SSC_PH_INIT (1)                            
        // [28] MEMPLL_SDM_SSC_EN (0)                                 
        vIO32Write4B(mcSET_SYS_REG_ADDR(0x2004), 0x041d8013);         
        //mcFPRINTF((fp_A60501, "RISCWrite : address %05x data %08x wait  0\n", mcSET_SYS_REG_ADDR(0x2004)&0x000fffff, 0x041d8013));

    #if fcFOR_CHIP_ID == fcA60501
        // [8] Channel C : TA4_3 32 bit enable(1)
        // [4] Channel B : TA4_3 32 bit diable (0)
        // [0] Channel A : TA4_3 32 bit diable (0)
        vIO32Write4B(mcSET_SYS_REG_ADDR(0x0090), 0x00000100);       
    #endif
                                                              
        // [28] AG0MWR: 1'b1: dramc handle all EMI's write request as mask write, no matter EMI's MWR[4:0] signals? 
        // LP4: 0x10000000
        // [30] R_DMPINMUX2 (chA/C floating, no use, chD MUX for LP3 or LP4)        
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x00f0), 0x40000000);
                                                      
        // [31] RODTE
        // [26:24] RODT
        // [22:16] TWODT
        // [15:12] TR2W
        // [10:8] TRTP
        // [3] WOEN? Disable ODT for LP3
        // [2] ROEN? Disable ODT for LP3
        // [0] FDIV2
        // RISCWrite : address 6007c data 8583628d
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x007c), 0x85836281);
  
        // [10:8] NORMPOP_LEN -> used for HW RX DQS/DQ calibration
        // [7] FREQDIV4
        // [4] DISWODTE (1'b1: extend 1T)
        // [3:1] DISWODT (3'b001: read_odt_enable_syn1t)
        // [0] WODTFIXOFF (1'b1: fixed to disable ODT)?
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0054), 0x0000011b);

        // [31] FASTWAKE (MT6595 enabled), [D] enable, enabled at the last?
        //      It is controlled by EMI, not used in test chip.       
        // [29] LPDDR4EN                                              
        // [23:16] REFRCNT: MR4 read                                  
        // [25:0] AC timing (see reg map)                             
        // [15:8] TRFCPB (0x00): ?                                    
        //vIO32Write4B_All(mcSET_DRAMC_AO_REG_ADDR(0x01e8), 0x21000001);
                                                                      
        // [10] RG_RX_ARDQ_STBENCMP_EN_B0: DQ Ring counter output enable Byte 0 DQ7-0
        // [9] RG_RX_ARDQS0_DQSIENMODE: 0: Normal (pulse) 1: Manual (burst)->for sim? [DP] Use pulse mode @ HW tracking
        // [8] RG_RX_ARDQS0_STBEN_RESETB? (NO need for LPDDR4?)
        // [7] RG_RX_ARDQ_IN_BUFF_EN_B0                               
        // [6] RG_RX_ARDQM0_IN_BUFF_EN                                
        // [5] RG_RX_ARDQS0_IN_BUFF_EN                                
        // [4] RG_RX_ARDQ_STBEN_RESETB_B0                             
        // [3] RG_ARDQ_RESETB_B0                                      
        // [2] RG_TX_ARDQ_EN_B0                                       
        // [1] RG_RX_ARDQ_SMT_EN_B0 (disabled @ init? [SP] Yes)       
        // [0] RG_ARDQ_ATPG_EN_B0       
        // LP4: 0x000007fe
        //RISCWrite : address 70014 data 000006fe wait  0             
        //RISCWrite : address 80014 data 000006fe wait  0             
        //RISCWrite : address 90014 data 000006fe wait  0             
        //vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0014), 0x000006fc);
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0014), 0x000004fc);//pulse mode
                                                                      
        // RX EYE Scan DQ/DQS rising/falling delay line settings (byte0)
        // Not necessary @ init?                                      
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0018), 0x00001010);  
                                                                      
        // RX eye scan config (byte0)                                 
        // [24] RG_RX_ARDQ_EYE_EN_B0 (1), Not necessary to enable RX eye scan @ init?
        // [23:20] RG_RX_ARDQ_EYE_SEL_B0 (0x1)                        
        // [17] RG_RX_ARDQ_EYE_VREF_EN_B0 (0)                         
        // [16] RG_RX_ARDQ_VREF_EN_B0 (0)                             
        // [13:8] RG_RX_ARDQ_EYE_VREF_SEL_B0 (0x00)                   
        // [5:0] RG_RX_ARDQ_VREF_SEL_B0 (0x00)                        
        //vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x001c), 0x01100000);  
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x001c), 0x0113100e);  //LP4
        
        // [24] RG_RX_ARDQ_EYE_DLY_DQS_BYPASS_B0 (1)                  
        // [19] RG_TX_ARDQ_DDR3_SEL_B0 (1)                            
        // [18] RG_RX_ARDQ_DDR3_SEL_B0 (1)                            
        // [17] RG_TX_ARDQ_DDR4_SEL_B0 (0)                            
        // [16] RG_RX_ARDQ_DDR4_SEL_B0 (0)                            
        // [12] RG_RX_ARDQ_BIAS_EN_B0 (0)?
        // [4] RG_TX_ARDQ_SER_MODE_B0 (1)?
        // [0] RG_RX_ARDQ_BIAS_PS_B0 (1): RX Biase power-saving enable
        //vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0020), 0x010c0011);  
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0020), 0x010c5011); //LP4 + SER_MODE
        
        // Set before                                                 
        // [25] RG_RX_ARDQ_EYE_STBEN_RESETB_B0                        
        // Not necessary to reset?                                    
        //vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x001c), 0x03100000);  
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x001c), 0x0313100e);   //LP4     
        
        // RX DQM/DQS delay line control (byte0, rank0)               
        // Not necessary set here? Use calibration                    
        //vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x003c), 0x00001010);  
                                                                      
        // RX DQM/DQS delay line control (byte0, rank1)               
        // Not necessary set here? Use calibration                    
        //vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0058), 0x00001010);  
                                                                      
        // Refer to 0x014                                             
        //RISCWrite : address 70074 data 000006fe wait  0             
        //RISCWrite : address 80074 data 000006fe wait  0             
        //RISCWrite : address 90074 data 000006fe wait  0             
        //vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0074), 0x000006fc);
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0074), 0x000004fc);//pulse mode

        // RX EYE Scan DQ/DQS rising/falling delay line settings (byte1)
        // Not necessary @ init?                                      
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0078), 0x00001010);  
                                                                      
        // RX eye scan config (byte1)                                 
        // Refer to 0x1c
        //vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x007c), 0x01100000); 
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x007c), 0x0113100e);    //LP4                                                          
        // Refer to 0x020                                             
        //vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0080), 0x010c0011);  
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0080), 0x010c5011); //LP4 +SER_Mode
        
        // Refer to 0x1c                                              
        // Not necessary to reset?                                    
        //vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x007c), 0x03100000);  
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x007c), 0x0313100e); //LP4
        
        // RX DQM/DQS delay line control (byte1, rank0)               
        // Not necessary set here? Use calibration                    
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x009c), 0x00001010);  
                                                                      
        // RX DQM/DQS delay line control (byte1, rank1)               
        // Not necessary set here? Use calibration                    
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x00b8), 0x00001010);  
        
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0180), 0x00000000);  
        
        // [4] RG_TX_ARCMD_SER_MODE
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0184), 0x00000010);  
                                                                      
        // [31:16] RG_TX_*_PULL_UP/DN (0)                             
        // [11] RG_RX_ARCMD_DDR3_SEL (0), RX?                         
        // [10] RG_RX_ARCMD_DDR4_SEL (0), RX?                         
        // [9] RG_TX_ARCMD_DDR3_SEL (1)                               
        // [8] RG_TX_ARCMD_DDR4_SEL (0)                               
        // [7] RG_TX_ARCMD_EN (1)                                     
        // [6] RG_TX_ARCLK_EN (1)                                     
        // [5] RG_TX_ARCS1_EN (1)                                     
        // [4] RG_TX_ARCS2_EN (0)                                     
        // [3] RG_TX_ARCMD_SCAN_IN_EN (0)                             
        // [2] RG_RX_ARCMD_SMT_EN (0)                                 
        // [1] RG_ARCMD_RESETB (1)                                    
        // [0] RG_ARCMD_ATPG_EN (0)                                   
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0188), 0x000002e2);  
                                                                      
        // [6:0] RG_ARPHYPLL_FBKDIV (52M*(PLL_FBKDIV_6_0+1))       
        // sim: 0xf
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0400), 0x00000000 | PLL_FBKDIV_6_0);
                                                                      
        // [31] RG_ARPHYPLL_EN (0)                                    
        // [28] RG_ARPHYPLL_EXT_FBKCK_SEL (0) : internal feedback clock
        // [27:24] RG_ARPHYPLL_BC : (0000), need to update BW settings?
        // [23:20] RG_ARPHYPLL_IC : (0000)                            
        // [19:16] RG_ARPHYPLL_BR : (0000)                            
        // [15:14] RG_ARPHYPLL_VREF_RAGE_SEL (00)                     
        // [13:12] RG_ARPHYPLL_BP2 (00) -> (11)                       
        // [11:8] RG_ARPHYPLL_BP (0000)                               
        // [6:0] RG_ARPHYPLL_BAND (0000000)                           
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0404), 0x00003000 | PLL_BC_3_0 | PLL_IC_3_0 | PLL_BR_3_0 | PLL_BP_3_0 | PLL_BAND_6_0);
                                                                      
        // [31] RG_ARPHYPLL_LVROD_EN                                  
        // [30] RG_ARPHYPLL_MONREF_EN                                 
        // [29] RG_ARPHYPLL_MONCK_EN                                  
        // [28] RG_ARPHYPLL_MONVC_EN                                  
        // [25:24] RG_ARPHYPLL_RST_DLY                                
        // [23:22] RG_ARPHYPLL_PREDIV                                 
        // [21:20] RG_ARPHYPLL_POSDIV                                 
        // [19] RG_ARPHYPLL_LOAD_EN                                   
        // [18] RG_ARPHYPLL_SLEEP_EN                                  
        // [17:16] RG_ARPHYPLL_KBAND_PREDIV                           
        // [12:0] RG_ARPHYPLL_IPATH_IDAC                              
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0408), 0x02000000);  
                                                                      
        // [30:28] RG_ARPHYPLL_IPATH_GAIN_SEL                         
        // [25:25] RG_ARPHYPLL_IPATH_SDM_IFM                          
        // [24:24] RG_ARPHYPLL_IPATH_SDM_DI_EN                        
        // [23:22] RG_ARPHYPLL_IPATH_SDM_DI_LS                        
        // [21:20] RG_ARPHYPLL_IPATH_SDM_ORD                          
        // [19:19] RG_ARPHYPLL_IPATH_SDM_OUT                          
        // [18:16] RG_ARPHYPLL_IPATH_DEBUG_SEL                        
        // [15:8] RG_ARPHYPLL_RESERVE_0                               
        // [7:0] RG_ARPHYPLL_RESERVE_1                                
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x040c), 0x00000000 | PLL_BC_4 | PLL_BP_4);
                                                                      
        // [6:0] RG_ARCLRPLL_FBKDIV                                   
        // CLRPLL?                                                    
        // No need to set?                                            
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0410), 0x00000000);  
                                                                      
        // [31] RG_ARCLRPLL_EN                                        
        // [28] RG_ARCLRPLL_EXT_FBKCK_SEL                             
        // [27:24] RG_ARCLRPLL_BC                                     
        // [23:20] RG_ARCLRPLL_IC                                     
        // [19:16] RG_ARCLRPLL_BR                                     
        // [15:14] RG_ARCLRPLL_VREF_RAGE_SEL                          
        // [13:12] RG_ARCLRPLL_BP2                                    
        // [11:8] RG_ARCLRPLL_BP                                      
        // [6:0] RG_ARCLRPLL_BAND                                     
        // no need to set?                                            
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0414), 0x00004000);  
                                                                      
        //[31] RG_ARCLRPLL_LVROD_EN                                   
        //[30] RG_ARCLRPLL_MONREF_EN                                  
        //[29] RG_ARCLRPLL_MONCK_EN                                   
        //[28] RG_ARCLRPLL_MONVC_EN                                   
        //[25:24] RG_ARCLRPLL_RST_DLY                                 
        //[23:22] RG_ARCLRPLL_PREDIV                                  
        //[21:20] RG_ARCLRPLL_POSDIV                                  
        //[19] RG_ARCLRPLL_LOAD_EN                                    
        //[18] RG_ARCLRPLL_SLEEP_EN                                   
        //[17:16] RG_ARCLRPLL_KBAND_PREDIV                            
        //[12:0] RG_ARCLRPLL_IPATH_IDAC                               
        // No need to set?                                            
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0418), 0x00000000);  
                                                                      
        //[30:28] RG_ARCLRPLL_IPATH_GAIN_SEL                          
        //[25:25] RG_ARCLRPLL_IPATH_SDM_IFM                           
        //[24:24] RG_ARCLRPLL_IPATH_SDM_DI_EN                         
        //[23:22] RG_ARCLRPLL_IPATH_SDM_DI_LS                         
        //[21:20] RG_ARCLRPLL_IPATH_SDM_ORD                           
        //[19:19] RG_ARCLRPLL_IPATH_SDM_OUT                           
        //[18:16] RG_ARCLRPLL_IPATH_DEBUG_SEL                         
        //[15:8] RG_ARCLRPLL_RESERVE_0                                
        //[7:0] RG_ARCLRPLL_RESERVE_1                                 
        // No need to set?                                            
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x041c), 0x03300000);  

        //Different for LPDDR3
        //[31:30] RG_ARPHYPLL_MCK_SEL_CB (10): MCK seleltion 2'b00: B0 clock 2'b01: B1 clock 2'b10: CA clock 2'b11: CB clock
        //[29:28] RG_ARPHYPLL_MCK_SEL_B0 (10)                         
        //[27:26] RG_ARPHYPLL_MCK_SEL_B1 (10)                         
        //[25:24] RG_ARPHYPLL_MCK_SEL_CA (10)                         
        //[23:23] RG_ARPHYPLL_EXTFBDIV_EN                             
        //[21:16] RG_ARPHYPLL_EXT_FBDIV                               
        //[15:15] RG_ARCLRPLL_EXTFBDIV_EN                             
        //[13:8] RG_ARCLRPLL_EXT_FBDIV                                
        //[7:7] RG_ARCLRPLL_EXTPODIV_EN                               
        //[6:6] RG_ARCLRPLL_BYPASSS (1): CLRPLL BYPASS mode 1'b0: normal 1'b1: bypass
        //[5:0] RG_ARCLRPLL_EXT_PODIV                                 
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHC(0x0420), 0xaa000040);          
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHA(0x0420), 0xc6000040);  
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHB(0x0420), 0xc6000040);  
                                                                      
        //[29:24] RG_ARPHYPLL_REF_DL                                  
        //[21:16] RG_ARPHYPLL_FB_DL                                   
        //[13:8] RG_ARCLRPLL_REF_DL                                   
        //[5:0] RG_ARCLRPLL_FB_DL                                     
        // No need to set? (N/A for 1PLL?)                            
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0424), 0x00000000);  
                                                                      
        //[30:28] RG_ARPI_SET_UPDN                                    
        //[25:24] RG_ARPI_CAP_SEL                                     
        //[14:8] RG_ARPI_FB_CB : TX offset for PLL feedback clock path
        //[6:0] RG_ARPI_MCTL_CB                                       
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0428), 0x01000000); 
                                                                      
        //[30:24] RG_ARPI_MCTL_CA                                     
        //[22:16] RG_ARPI_FB_B0                                       
        //[14:8] RG_ARPI_FB_B1                                        
        //[6:0] RG_ARPI_FB_CA                                         
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x042c), 0x00000000);  
        
        //[31] RG_ARPI_MCTL_CB_JUMP_EN                                
        //[30] RG_ARPI_FB_CB_JUMP_EN                                  
        //[29] RG_ARPI_CS_CB_JUMP_EN                                  
        //[28] RG_ARPI_CK_CB_JUMP_EN                                  
        //[27] RG_ARPI_CMD_CB_JUMP_EN                                 
        //[26] RG_ARPI_MCTL_CA_JUMP_EN                                
        //[25] RG_ARPI_FB_B0_JUMP_EN                                  
        //[24] RG_ARPI_FB_B1_JUMP_EN                                  
        //[23] RG_ARPI_FB_CA_JUMP_EN                                  
        //[22] RG_ARPI_CS_JUMP_EN                                     
        //[21] RG_ARPI_CK_JUMP_EN                                     
        //[20] RG_ARPI_CMD_JUMP_EN                                    
        //[19] RG_ARPI_DQ_B0_JUMP_EN                                  
        //[18] RG_ARPI_DQ_B1_JUMP_EN                                  
        //[17] RG_ARPI_DQSIEN_B0_JUMP_EN                              
        //[16] RG_ARPI_DQSIEN_B1_JUMP_EN                              
        //[15] RG_ARPI_CG_MCTL_CB                                     
        //[14] RG_ARPI_CG_FB_CB                                       
        //[13] RG_ARPI_CG_CS_CB                                       
        //[12] RG_ARPI_CG_CK_CB                                       
        //[11] RG_ARPI_CG_CMD_CB                                      
        //[10] RG_ARPI_CG_MCTL_CA                                     
        //[9]  RG_ARPI_CG_FB_B0                                       
        //[8]  RG_ARPI_CG_FB_B1                                       
        //[7]  RG_ARPI_CG_FB_CA                                       
        //[6]  RG_ARPI_CG_CS                                          
        //[5]  RG_ARPI_CG_CK                                          
        //[4]  RG_ARPI_CG_CMD                                         
        //[3]  RG_ARPI_CG_DQ_B0                                       
        //[2]  RG_ARPI_CG_DQ_B1                                       
        //[1]  RG_ARPI_CG_DQSIEN_B0 (?)
        //[0]  RG_ARPI_CG_DQSIEN_B1 (?)
        //LP4: ffff0000
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0430), 0xffff0003);  

        //[31] RG_ARPI_MCTL_CB_EN                                     
        //[30] RG_ARPI_FB_CB_EN                                       
        //[29] RG_ARPI_CS_CB_EN                                       
        //[28] RG_ARPI_CK_CB_EN                                       
        //[27] RG_ARPI_CMD_CB_EN                                      
        //[26] RG_ARPI_MCTL_CA_EN                                     
        //[25] RG_ARPI_FB_B0_EN                                       
        //[24] RG_ARPI_FB_B1_EN                                       
        //[23] RG_ARPI_FB_CA_EN                                       
        //[22] RG_ARPI_CS_EN                                          
        //[21] RG_ARPI_CK_EN                                          
        //[20] RG_ARPI_CMD_EN                                         
        //[19] RG_ARPI_DQ_B0_EN                                       
        //[18] RG_ARPI_DQ_B1_EN                                       
        //[17] RG_ARPI_DQSIEN_B0_EN                                   
        //[16] RG_ARPI_DQSIEN_B1_EN                                   
        //[15] RG_ARPI_BYPASS_MCTL_CB: PLL FB path bypass PI. 0: PI mode 1: Bypass PI
        //[14] RG_ARPI_BYPASS_FB_CB                                   
        //[13] RG_ARPI_BYPASS_CS_CB                                   
        //[12] RG_ARPI_BYPASS_CK_CB                                   
        //[11] RG_ARPI_BYPASS_CMD_CB                                  
        //[10] RG_ARPI_BYPASS_MCTL_CA                                 
        //[9]  RG_ARPI_BYPASS_FB_B0                                   
        //[8]  RG_ARPI_BYPASS_FB_B1                                   
        //[7]  RG_ARPI_BYPASS_FB_CA                                   
        //[6]  RG_ARPI_BYPASS_CS                                      
        //[5]  RG_ARPI_BYPASS_CK                                      
        //[4]  RG_ARPI_BYPASS_CMD                                     
        //[3]  RG_ARPI_BYPASS_DQ_B0                                   
        //[2]  RG_ARPI_BYPASS_DQ_B1                                   
        //[1]  RG_ARPI_BYPASS_DQSIEN_B0                               
        //[0]  RG_ARPI_BYPASS_DQSIEN_B1                               
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0434), 0xffff0000);  
                                                                      
        //[15:15] RG_ARPI_MPDIV_CHA_EN (0): NOT enabled? (@ last)     
        //[11:10] RG_ARPI_MPDIV_CHA_SEL (11): DIV8 dividor enable     
        //[9:8]   RG_ARPI_MPDIV_CHA_OUT_SEL (00): DIV1                
        //[7:7]   RG_ARPI_MPDIV_CHB_EN (0): NOT enabled? (@ last)     
        //[3:2]   RG_ARPI_MPDIV_CHB_SEL (11): DIV8 dividor enable     
        //[1:0]   RG_ARPI_MPDIV_CHB_OUT_SEL (00): DIV1       
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0438), 0x00000c0c);  
        //vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0438), 0x00000f0c);  //div 8
        //vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0438), 0x00000d0c);   //div 2

        //[29:29] RG_ARDLL_PI_CKIN_SEL_B0 (0): 0: MPDIV CHA           
        //[28:28] RG_ARDLL_PHDET_OUTPUT_SEL_B0 (0)                    
        //[26:26] RG_ARDLL_FJ_OUT_MODE_B0 (0)                         
        //[25:25] RG_ARDLL_FJ_OUT_MODE_SEL_B0 (0)                     
        //[24:24] RG_ARDLL_PHDET_IN_SWAP_B0 (0)                       
        //[23:20] RG_ARDLL_GAIN_B0 (0x6)                              
        //[19:16] RG_ARDLL_IDLECNT_B0 (0x6)                           
        //[11:11] RG_ARDLL_PHDET_EN_B0 (0)                            
        //[10:10] RG_ARDLL_PHJUMP_EN_B0 (1)                           
        //[9:9]   RG_ARDLL_PHDIV_B0 (1)                               
        //[8:8]   RG_ARDLL_DIV_DEC_B0 (0)                             
        //[7:4]   RG_ARDLL_MON_SEL_B0 (0)                             
        //[1:0]   RG_ARDLL_DIV_B0 (0)
        //LP4: 0x00660600
        #if SSC_TEST_ORIGINAL
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHC(0x043c), 0x00660600);  
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHA(0x043c), 0x11660600);  
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHB(0x043c), 0x11660600);  
        #else
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHC(0x043c), 0x11860600);   //test
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHA(0x043c), 0x11860600);  
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHB(0x043c), 0x11860600);  
        #endif
                                                                              
        // B1   
        //LP4: 0x00660600
        #if SSC_TEST_ORIGINAL
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHC(0x0440), 0x00660600);  
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHA(0x0440), 0x11660600);  
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHB(0x0440), 0x11660600); 
        #else
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHC(0x0440), 0x11860600);  //test
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHA(0x0440), 0x11860600);  
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHB(0x0440), 0x11860600); 
        #endif
                                                                              
        // [31:30] RG_ARDLL_DIV_SEL_MCTL_CA (10): MCTL Loop control selection -> CA
        // [29:29] RG_ARDLL_PI_CKIN_SEL_CA                            
        // [28:28] RG_ARDLL_PHDET_OUTPUT_SEL_CA                       
        // [26:26] RG_ARDLL_FJ_OUT_MODE_CA                            
        // [25:25] RG_ARDLL_FJ_OUT_MODE_SEL_CA                        
        // [24:24] RG_ARDLL_PHDET_IN_SWAP_CA                          
        // [23:20] RG_ARDLL_GAIN_CA                                   
        // [19:16] RG_ARDLL_IDLECNT_CA                                
        // [11:11] RG_ARDLL_PHDET_EN_CA                               
        // [10:10] RG_ARDLL_PHJUMP_EN_CA                              
        // [9:9]   RG_ARDLL_PHDIV_CA                                  
        // [8:8]   RG_ARDLL_DIV_DEC_CA                                
        // [7:4]   RG_ARDLL_MON_SEL_CA                                
        // [3:2]   RG_ARDLL_DIV_MCTL_CA                               
        // [1:0]   RG_ARDLL_DIV_CA
        // LP4: 0x80690608
        #if SSC_TEST_ORIGINAL
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHC(0x0444), 0x80690600);  
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHA(0x0444), 0x91690600);  
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHB(0x0444), 0x91690600); 
        #else
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHC(0x0444), 0x91890600);  //test
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHA(0x0444), 0x51890600);  
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHB(0x0444), 0x51890600); 
        #endif
                                                                              
        // [31:30] RG_ARDLL_DIV_SEL_MCTL_CA (11): MCTL Loop control selection -> CB
        // [29:29] RG_ARDLL_PI_CKIN_SEL_CB                            
        // [28:28] RG_ARDLL_PHDET_OUTPUT_SEL_CB                       
        // [26:26] RG_ARDL_FJ_OUT_MODE_CB                             
        // [25:25] RG_ARDLL_FJ_OUT_MODE_SEL_CB                        
        // [24:24] RG_ARDLL_PHDET_IN_SWAP_CB                          
        // [23:20] RG_ARDLL_GAIN_CB                                   
        // [19:16] RG_ARDLL_IDLECNT_CB                                
        // [11:11] RG_ARDLL_PHDET_EN_CB                               
        // [10:10] RG_ARDLL_PHJUMP_EN_CB                              
        // [9:9]   RG_ARDLL_PHDIV_CB                                  
        // [8:8]   RG_ARDLL_DIV_DEC_CB                                
        // [7:4]   RG_ARDLL_MON_SEL_CB                                
        // [3:2]   RG_ARDLL_DIV_MCTL_CB                               
        // [1:0]   RG_ARDLL_DIV_CB                
        // LP4: 0xc0690608
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHC(0x0448), 0xc0690600);  
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHA(0x0448), 0xd1690600);  
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHB(0x0448), 0xd1690600); 
                                                                             
        // [20:20] RG_ARPHYPLL_TSTLVROD_EN                            
        // [19:16] RG_ARPHYPLL_TST_SEL                                
        // [15:15] RG_ARPHYPLL_TST_EN                                 
        // [14:14] RG_ARPHYPLL_TSTCK_EN                               
        // [13:13] RG_ARPHYPLL_A2DCK_EN                               
        // [12:12] RG_ARPHYPLL_TSTOD_EN                               
        // [8:8]   RG_ARPHYPLL_TSTOP_EN                               
        // [6:6]   RG_IMPA_DDR3_SEL                                   
        // [5:5]   RG_IMPA_DDR4_SEL                                   
        // [4:4]   RG_IMPA_DDR4_ODT_EN                                
        // [3:3]   RG_IMPA_EN                                         
        // [2:2]   RG_IMPA_OCD_PUCMP_EN                               

        // vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x044c), 0x00000000);  
        // Yulia test setting from SP
        vIO32Write4B_All(mcSET_DDRPHY_REG_ADDR(0x044c), 0x00000040);  //RG_IMPA_DDR3_SEL=1

                                                                      
        // [21:16] RG_IMPA_VREF_SEL                                   
        // [12:8]  RG_IMPA_DRVN                                       
        // [4:0]   RG_IMPA_DRVP                                       
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0450), 0x00000000);  
                                                                      
        // PHYPLL reset : Add delay?                                  
        // [24:24] RG_ARMCTLPLL_CK_SEL                                
        // [20:20] RG_ARPHYPLL_SER_MODE                               
        // [19:19] RG_ARPHYPLL_LS_SEL                                 
        // [18:18] RG_ARPHYPLL_LS_EN                                  
        // [17:17] RG_ARPHYPLL_ATPG_EN                                
        // [16:16] RG_ARPHYPLL_RESETB                                 
        // [15:0]  RG_ARPHYPLL_TOP_REV           
        // LP4: 0x00000000
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0454), 0x00100000);  

        // LP4: NA
        // Only Ch-C? 
        // DDRPHY Rx116 (0x458) is both active and functional in Ch-A&B, but useless in Ch-D
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHC(0x0458), 0x00100000);   // Partickl

        // LP4: NA
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x05c0), 0x0000017c);  

        // LP4: NA
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0460), 0x00001010); 

        // LP4: NA
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0468), 0x00001010); 

        // LP4: 0x00000000 / 0x00010000
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0454), 0x00100000);  
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0454), 0x00110000);  
                                                                      
        // Enable PHYPLL                                              
        // [31] RG_ARPHYPLL_EN (1)                                    
        //vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0404), 0x80000000);  
        vIO32Write4B_All(mcSET_DDRPHY_REG_ADDR(0x0404), 0x80003000 | PLL_BC_3_0 | PLL_IC_3_0 | PLL_BR_3_0 | PLL_BP_3_0 | PLL_BAND_6_0);
                                                                      
        // wait 10 or 20us (sim) for settling?
        mcDELAY_US(20);
                                                                      
        // Print log (for debug)                                      
        mcSHOW_DBG_MSG(("[DdrPhyInit] PHYPLL Enable...\n"));          
        mcFPRINTF((fp_A60501, "[DdrPhyInit] PHYPLL Enable...\n"));    
        u4value = u4IO32Read4B(mcSET_DDRPHY_REG_ADDR(0x0f84));        
        //mcSHOW_DBG_MSG(("[DdrPhyInit] Channel A PHYPLL Kband CPLT=%d\n", (u4value&0x01000000)>>24));
        //mcFPRINTF((fp_A60501, "[DdrPhyInit] Channel A PHYPLL Kband CPLT=%d\n", (u4value&0x01000000)>>24));
        mcSHOW_DBG_MSG(("[DdrPhyInit] Channel A PHYPLL Kband CPLT=%d\n", (u4value&0x01000000)>>24));
        mcFPRINTF((fp_A60501, "[DdrPhyInit] Channel A PHYPLL Kband CPLT=%d\n", (u4value&0x01000000)>>24));
                                                                      
        u4value = u4IO32Read4B(mcSET_DDRPHY_REG_ADDR(0x0f84)+0x00010000);
        mcSHOW_DBG_MSG(("[DdrPhyInit] Channel C PHYPLL Kband CPLT=%d\n", (u4value&0x01000000)>>24));
        mcFPRINTF((fp_A60501, "[DdrPhyInit] Channel C PHYPLL Kband CPLT=%d\n", (u4value&0x01000000)>>24));
                                                                      
        u4value = u4IO32Read4B(mcSET_DDRPHY_REG_ADDR(0x0f84)+0x00020000);
        mcSHOW_DBG_MSG(("[DdrPhyInit] Channel D PHYPLL Kband CPLT=%d\n", (u4value&0x01000000)>>24));
        mcFPRINTF((fp_A60501, "[DdrPhyInit] Channel D PHYPLL Kband CPLT=%d\n", (u4value&0x01000000)>>24));    
                                                                      
        // [15] RG_ARPI_MPDIV_CHA_EN                                  
        //[7]  RG_ARPI_MPDIV_CHB_EN         
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0438), 0x00008c8c);  
        //vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0438), 0x00008d8d);   //div 2
        //vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0438), 0x00008f8c);  //div 8
                                                                      
        // [11] RG_ARDLL_PHDET_EN_B0    
        // LP4: 0x00660e00
        #if SSC_TEST_ORIGINAL
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHC(0x043c), 0x00660e00);  
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHA(0x043c), 0x11660e00);  
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHB(0x043c), 0x11660e00); 
        #else
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHC(0x043c), 0x11860e00);  //test        
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHA(0x043c), 0x11860e00);  
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHB(0x043c), 0x11860e00); 
        #endif
                                                                              
        // B1  
        // LP4: 0x00660e00
        #if SSC_TEST_ORIGINAL
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHC(0x0440), 0x00660e00);  
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHA(0x0440), 0x11660e00);  
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHB(0x0440), 0x11660e00); 
        #else
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHC(0x0440), 0x11860e00);  //test
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHA(0x0440), 0x11860e00);  
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHB(0x0440), 0x11860e00); 
        #endif
                                                                              
        // [11:11] RG_ARDLL_PHDET_EN_CA      
        // LP4: 0x80690e08
        #if SSC_TEST_ORIGINAL
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHC(0x0444), 0x80690e00);  
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHA(0x0444), 0x91690e00);  
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHB(0x0444), 0x91690e00);         
        #else
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHC(0x0444), 0x91890e00);  //test
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHA(0x0444), 0x51890e00);  
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHB(0x0444), 0x51890e00);         
        #endif
                                                                              
        // [11:11] RG_ARDLL_PHDET_EN_CB                               
        // LP4: 0xc0690e08
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHC(0x0448), 0xc0690e00);  
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHA(0x0448), 0xd1690e00);  
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHB(0x0448), 0xd1690e00);  

        // LP4: NA 
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0430), 0xffff0000);  

        // LP4: NA
        // [8] RG_RX_ARDQS0_STBEN_RESETB
        // 000007fe
        //vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0014), 0x000007fc);
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0014), 0x000005fc);//pulse mode

        // LP4: NA
        // [8] RG_RX_ARDQS0_STBEN_RESETB
        // 000007fe
        //vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0074), 0x000007fc);   
        vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0074), 0x000005fc);//pulse mode   

    #if 1 // SW impedance will set these value
    vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x000c), 0x00000a0a); 
    vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0010), 0x00000a0a);
    vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x006c), 0x00000a0a); 
    vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x0070), 0x00000a0a);  
    vIO32Write4B_All2(mcSET_DDRPHY_REG_ADDR(0x018c), 0x0a0a0a0a);   
    #endif
        return DRAM_OK;
}

static DRAM_STATUS_T DramcModeRegInit_LP3(DRAMC_CTX_T *p)
{
        // MR63 -> Reset
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0088), 0x0000003f);
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x01e4), 0x00000001);
        mcDELAY_US(10);    // Wait >=10us if not check DAI.
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x01e4), 0x00000000);
        
        // MR10 -> ZQ Init
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0088), 0x00ff000a);
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x01e4), 0x00000001);
        mcDELAY_US(1);    // tZQINIT>=1us
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x01e4), 0x00000000);
        
        // MR1
        // nWR: OP[7:5] (12 -> LP3-1600)
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0088), 0x00430001);
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x01e4), 0x00000001);
        mcDELAY_US(1);
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x01e4), 0x00000000);
        
        // MR2
        if(p->frequency <= 400)
        {
            u4MR2Value = 0x14;
        }
        else if(p->frequency == 1066)
        {
            u4MR2Value = 0x1e;
        }
        else if(p->frequency == 933)
        {
            u4MR2Value = 0x1c;
        }
        else
        {
            u4MR2Value = 0x1a;
        }
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0088), 0x00000002 | (u4MR2Value <<16));
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x01e4), 0x00000001);
        mcDELAY_US(1);
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x01e4), 0x00000000);

        // ?
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0054), 0x0000011b);
        
        // MR11, ODT disable.
        // RISCWrite : address 60088 data 0003000b
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0088), 0x0000000b);
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x01e4), 0x00000001);
        mcDELAY_US(1);
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x01e4), 0x00000000);

        // MR4
        //vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0088), 0x00000004);
        //vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x01e4), 0x00000002);
        //mcDELAY_US(1);

        return DRAM_OK;
}
#endif //COMPILE_A60817_LP3_INIT_CODE

DRAM_STATUS_T DramcModeRegInit_Everest_LP3(DRAMC_CTX_T *p)
{
    U32 u4RankIdx, u4CKE0Bak, u4CKE1Bak, u4MIOCKBak;
    
    u4MIOCKBak =u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DRAMC_PD_CTRL), DRAMC_PD_CTRL_MIOCKCTRLOFF);
    u4CKE0Bak = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_PADCTL4), PADCTL4_CKEFIXON);
    u4CKE1Bak = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_GDDR3CTL1), GDDR3CTL1_CKE1FIXON);
    
    // Disable HW MIOCK control to make CLK always on
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DRAMC_PD_CTRL), 1, DRAMC_PD_CTRL_MIOCKCTRLOFF);
    mcDELAY_US(1);

    //if CKE2RANK=1, only need to set CKEFIXON, it will apply to both rank.
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_PADCTL4), 1, PADCTL4_CKEFIXON);
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_GDDR3CTL1), 1, GDDR3CTL1_CKE1FIXON);
    
    //wait cke to reset timing constraint 'tinit3'
    //for (i=0; i < 10 ; ++i);
    mcDELAY_US(200);

    for(u4RankIdx =0; u4RankIdx < (U32)(p->support_rank_num); u4RankIdx++)
    {
        mcSHOW_DBG_MSG2(("DramcModeRegInit_Everest_LP3 for Rank%d\n", u4RankIdx));          
        mcFPRINTF((fp_A60501, "DramcModeRegInit_Everest_LP3 for Rank%d\n", u4RankIdx));    

        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_MRS), u4RankIdx, MRS_MRSRK);
    
        DramcModeRegWrite(p, 0x3f, 0);       //MR63=0   -> Reset
        mcDELAY_US(10);  //wait 10us for dram reset

        DramcModeRegWrite(p, 0xa, 0xff);   //MR10=0xff  -> ZQ Init

        DramcModeRegWrite(p, 1, 0x83);      //MR1=0x83 nWR: OP[7:5] (14 -> LP3-1866),   A60817 is 0x43  nWR: OP[7:5] (12 -> LP3-1600)
                
        // MR2
        #if SUPPORT_LP3_800
        if(p->frequency<=400)
        {
            u4MR2Value = 0x18;
        }
        else
        #endif
        if(p->frequency==533)
        {
            u4MR2Value = 0x16;
        }
        else if(p->frequency == 635)
        {
            u4MR2Value = 0x18;
        }
        else if(p->frequency == 800)
        {
            u4MR2Value = 0x1a;
        }
        else
        {
            u4MR2Value = 0x1c;
        }
        DramcModeRegWrite(p, 2, u4MR2Value);

        DramcModeRegWrite(p, 0xb, 0x3);      //MR11=0x3 , ODT disable.
    }        
    
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_MRS), 0, MRS_MRSRK);

    // Restore CKE fix on setting
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_PADCTL4), u4CKE0Bak, PADCTL4_CKEFIXON);
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_GDDR3CTL1), u4CKE1Bak, GDDR3CTL1_CKE1FIXON);
    
    // Restore HW MIOCK control setting
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DRAMC_PD_CTRL), u4MIOCKBak, DRAMC_PD_CTRL_MIOCKCTRLOFF);
    
    /*TINFO="===  dram initial end (DramcModeRegInit_Everest_LP3)==="*/
    return DRAM_OK;
}


static DRAM_STATUS_T DramcSetting_Everest_LP3(DRAMC_CTX_T *p)
{
    *((volatile unsigned int *)(DRAMC_WBR)) = 0x7;

    RISCWriteDRAM(0x0c4 , 0x00000000);
    RISCWriteDRAM(0x400 , 0x00000000);   //22677000.0 ps
    RISCWriteDRAM(0x404 , 0x00000000);   //0x00000000);

    //rx gating setting
    #if 1
    if(p->frequency<=533)//DDR_CRL1066
    {        
        RISCWriteDRAM(0x404 , 0x00000000);   //0x00000000);   //22695000.0 ps
        RISCWriteDRAM(0x410 , 0x57555555);   //0x55d55555);   //22713000.0 ps
        RISCWriteDRAM(0x418 , 0x0000000d);   //0x00000007);   //22731000.0 ps
        RISCWriteDRAM(0x430 , 0x000000ff);   //0x000000ff);   //22748000.0 ps
        RISCWriteDRAM(0x434 , 0x757575ff);   //0x535353ff);   //22766000.0 ps  //Fix selphy 0212
        RISCWriteDRAM(0x438 , 0x000000ff);   //0x000000ff);   //22784000.0 ps
        RISCWriteDRAM(0x43c , 0x757575ff);   //0x535353ff);   //22802000.0 ps  //Fix selph 0212
        RISCWriteDRAM(0x454 , 0xf0000000);   //0xa0000000);   //22828000.0 ps  //Fix selphy 0212
    }
    else if(p->frequency<=635)//DDR_CLR1270
    {
        RISCWriteDRAM(0x404 , 0x00000000);   //22695000.0 ps
        RISCWriteDRAM(0x410 , 0x57555555);   //RISCWriteDRAM(0x410 , 0x57555555);   //22713000.0 ps  
        RISCWriteDRAM(0x418 , 0x0000000d);   //RISCWriteDRAM(0x418 , 0x0000000d);   //22731000.0 ps 
        RISCWriteDRAM(0x430 , 0x000000ff);   //22748000.0 ps
        RISCWriteDRAM(0x434 , 0x313131ff);   //RISCWriteDRAM(0x434 , 0x313131ff);   //22766000.0 ps  //Fix selphy 0212
        RISCWriteDRAM(0x438 , 0x000000ff);   //22784000.0 ps
        RISCWriteDRAM(0x43c , 0x313131ff);   //RISCWriteDRAM(0x43c , 0x313131ff);   //22802000.0 ps  //Fix selph 0212
        RISCWriteDRAM(0x454 , 0x00000000);   //RISCWriteDRAM(0x454 , 0x00000000);   //22828000.0 ps  //Fix selphy 0212
    }
    else if(p->frequency<=800)//DDR_CLR1600, DDR_LP1600
    {
        RISCWriteDRAM(0x404 , 0x00000000);   //22695000.0 ps
        RISCWriteDRAM(0x410 , 0x54b55555);   //RISCWriteDRAM(0x410 , 0x57555555);   //22713000.0 ps  
        RISCWriteDRAM(0x418 , 0x00000002);   //RISCWriteDRAM(0x418 , 0x0000000d);   //22731000.0 ps 
        RISCWriteDRAM(0x430 , 0x000000ff);   //22748000.0 ps
        RISCWriteDRAM(0x434 , 0x424242ff);   //RISCWriteDRAM(0x434 , 0x313131ff);   //22766000.0 ps  //Fix selphy 0212
        RISCWriteDRAM(0x438 , 0x000000ff);   //22784000.0 ps
        RISCWriteDRAM(0x43c , 0x424242ff);   //RISCWriteDRAM(0x43c , 0x313131ff);   //22802000.0 ps  //Fix selph 0212
        RISCWriteDRAM(0x454 , 0xa0000000);   //RISCWriteDRAM(0x454 , 0x00000000);   //22828000.0 ps
    }
    else if(p->frequency<=933)//DDR_LP1866
    {
        RISCWriteDRAM(0x404 , 0x00000000);   //22695000.0 ps
        RISCWriteDRAM(0x410 , 0x55d55555);   //RISCWriteDRAM(0x410 , 0x57555555);   //22713000.0 ps  
        RISCWriteDRAM(0x418 , 0x00000007);   //RISCWriteDRAM(0x418 , 0x0000000d);   //22731000.0 ps 
        RISCWriteDRAM(0x430 , 0x000000ff);   //22748000.0 ps
        RISCWriteDRAM(0x434 , 0x535353ff);   //RISCWriteDRAM(0x434 , 0x313131ff);   //22766000.0 ps  //Fix selphy 0212
        RISCWriteDRAM(0x438 , 0x000000ff);   //22784000.0 ps
        RISCWriteDRAM(0x43c , 0x535353ff);   //RISCWriteDRAM(0x43c , 0x313131ff);   //22802000.0 ps  //Fix selph 0212
        RISCWriteDRAM(0x454 , 0xa0000000);   //RISCWriteDRAM(0x454 , 0x00000000);   //22828000.0 ps
    }
    #endif
    //end Gating setting


    RISCWriteDRAM(0x408 , 0x00000000);   //22837000.0 ps
    RISCWriteDRAM(0x40c , 0x00000000);   //22846000.0 ps

    //TX-phase
    if(p->frequency<=933)
    {
        RISCWriteDRAM(0x41c , 0x33333333);   //22855000.0 ps
        RISCWriteDRAM(0x420 , 0x33333333);   //22864000.0 ps
        RISCWriteDRAM(0x424 , 0x33333333);   //22873000.0 ps
    }


    RISCWriteDRAM(0x428 , 0x5555ffff);   //22882000.0 ps
    RISCWriteDRAM(0x42c , 0x005500ff);   //22944000.0 ps
    RISCWriteDRAM(0x448 , 0x33333333);   //23015000.0 ps
    RISCWriteDRAM(0x44c , 0x11111111);   //23024000.0 ps

    RISCWriteDRAM(0x458 , 0x00000000);   //23024000.0 ps


    RISCWriteDRAM(0x1e0 , 0x2201ffff);   //22962000.0 ps


    if(p->frequency<=933)
    {
        RISCWriteDRAM(0x440 , 0x33333333);   //22997000.0 ps
        RISCWriteDRAM(0x444 , 0x33333333);   //23006000.0 ps
    }



    RISCWriteDRAM(0x008 , 0x1040b070);   //RISCWriteDRAM(0x008 , 0x10003300);   //23068000.0 ps //upd actiming

    //enable tx pipe x3
    RISCWriteDRAM(0x08c , 0x00e00001);   //23104000.0 ps
    RISCWriteDRAM(0x0d8 , 0x00100510);   //23113000.0 ps
    RISCWriteDRAM(0x0e4 , 0x00082101);   //23121000.0 ps
    RISCWriteDRAM(0x0b8 , 0x99169952);   //23130000.0 ps
    RISCWriteDRAM(0x0bc , 0x99109950);   //23139000.0 ps


    //R0, R1 DQSINCTL (for 1866)
    if(p->frequency<=933)
    {
        RISCWriteDRAM(0x0e0 , 0x16200200);   //23192000.0 ps
        RISCWriteDRAM(0x118 , 0x00000006);   //23201000.0 ps        
    }

    RISCWriteDRAM(0x0f0 , 0x40000000);   //23210000.0 ps
    RISCWriteDRAM(0x0f4 , 0x01000000);   //23246000.0 ps
    RISCWriteDRAM(0x168 , 0x00007080);   //23263000.0 ps
    RISCWriteDRAM(0x130 , 0x30000000);   //23281000.0 ps
    RISCWriteDRAM(0x0d8 , 0x00300510);   //23290000.0 ps

    if(p->frequency<=933)
    {
        RISCWriteDRAM(0x004 , 0xf0048403);   //23299000.0 ps
    }

    //150327 R_DMRD_START_EXT1 =1 R_DMRD_START_EXT2 =1 R_DMRD_START_EXT3 =1
    RISCWriteDRAM(0x138 , 0xa8908403);   //23335000.0 ps   //[23] DQSIENCG_CHG_EN=1 (default),  [11:10] DMSTBLAT=1 , [22:20] PICGLAT =1 


    RISCWriteDRAM(0x094 , 0x00000000);   //23343000.0 ps
    RISCWriteDRAM(0x098 , 0x00000000);   //23352000.0 ps

    RISCWriteDRAM(0x1c4 , 0x41024000);   //23388000.0 ps   //rankctl

    if(p->frequency<=933)
    {
        RISCWriteDRAM(0x000 , 0xaafd478c);   //RISCWriteDRAM(0x000 , 0xaafd488c);   //23428000.0 ps           //upd actiming
        RISCWriteDRAM(0x0fc , 0x910B0000);   //RISCWriteDRAM(0x0fc , 0x27030000);   //23434000.0 ps  //CKEPRD //upd actiming
        RISCWriteDRAM(0x1f8 , 0x000025f1);   //RISCWriteDRAM(0x1f8 , 0x040020c7);   //23443000.0 ps           //upd actiming
    }


    RISCWriteDRAM(0x1ec , 0x1030c311);   //23479000.0 ps  //DSIM_0320 //Performance //0616

    if(p->frequency<=933)
    {
        RISCWriteDRAM(0x07c , 0xf5b86381);   //RISCWriteDRAM(0x07c , 0xf5b8648d);   //23497000.0 ps //upd actiming 
    }


    //dramc pipe en
    RISCWriteDRAM(0x080 , 0x00f30de0);   //23514000.0 ps

    *((volatile unsigned int *)(DRAMC_WBR)) = 0x5;//set r_dram_cha =1 for dramc_conf
    RISCWrite((DRAMC0_BASE  + 0x028  ) , 0x50300041);
    RISCWrite((DRAMC1_BASE  + 0x028  ) , 0x50200041);
    *((volatile unsigned int *)(DRAMC_WBR)) = 0x7;
       

    RISCWriteDRAM(0x1e8 , 0x15FF1115);   //23550000.0 ps   //DSIM_0226

    #if EVEREST_RX_POWER_ECO
    mcSHOW_DBG_MSG(("EVEREST_RX_POWER_ECO,  E1: 0xCA00, E2: 0xCA01\n"));     
    mcSHOW_DBG_MSG(("IC version detect 0x%x\n", (*(volatile unsigned int*)(0x08000008))));     
    if((*(volatile unsigned int*)(0x08000008)) == 0xCA00) // Everest E1   
    {    
        RISCWriteDRAM(0x140 , 0x00000001);  //BUFFEN OPT
    }
    else// Everest E2
    {
        RISCWriteDRAM(0x140 , 0x00000000);  //BUFFEN OPT
    }
    #else
    RISCWriteDRAM(0x140 , 0x00000001);  //BUFFEN OPT
    #endif

    RISCWriteDRAM(0x0d8, 0x00000410);  //22966000.0 ps


    RISCWriteDRAM(0x054 , 0x00000119);   //25863000.0 ps
    
    /*TINFO="===  dram initial start ==="*/

    RISCWriteDRAM(0x0e4 , 0x00080001);   //PADCTL4_DMPGVLD_IG=0


    DramcModeRegInit_Everest_LP3(p);
    

    ///TODO:ENABLE AFTER EVEREST_BRING_UP
    RISCWriteDRAM(0x110 , 0x00696802);   //disable per-bank refresh

    ///TODO:ENABLE AFTER EVEREST_BRING_UP
    if(p->frequency<=933)
    {
        RISCWriteDRAM(0x1dc , 0x15656440);   //27981000.0 ps  //R_DMREFFRERUN=1 //0418 tXSR, MIOCKCTRLOFF=1 (always enable clk)
    }
        
    RISCWriteDRAM(0x1ec , 0x1030c311);   //27999000.0 ps  //DSIM_0320  //0616

    /*TINFO="===  dram initial end ==="*/

    RISCWriteDRAM(0x084 , 0x00000a56);   //32141000.0 ps

    RISCWriteDRAM(0x044 , 0x28070400);   //32188000.0 ps

    if(p->frequency<=933)
    {
        //RISCWriteDRAM(0x1e8 , 0x15041f59);   //32215000.0 ps
        //RISCWriteDRAM(0x1e8 , 0x95041f59);   //32215000.0 ps   //DSIM_0226 //wait PIPE cg eco
        //RISCWriteDRAM(0x1e8 , 0x15041f59);   //32215000.0 ps   //DSIM_0226  
        //RISCWriteDRAM(0x008 , 0x00003371);   //32232000.0 ps
        //RISCWriteDRAM(0x008 , 0x0040b070);   //RISCWriteDRAM(0x008 , 0x00403371);   //32232000.0 ps //Turn on HW selftest2 ena //upd actiming
        RISCWriteDRAM(0x008 , 0x0140b070);   //RISCWriteDRAM(0x008 , 0x00403371);   //32232000.0 ps //Turn on HW selftest2 ena //upd actiming //REFTHD=1
    }

    //turn off high-ref auto-ref
    RISCWriteDRAM(0x95c , 0x80000000);   //32250000.0 ps

    RISCWriteDRAM(0x100 , 0x01008110);   //32250000.0 ps
    RISCWriteDRAM(0x01c , 0x12121212);   //32259000.0 ps
    RISCWriteDRAM(0x0f8 , 0x00000000);   //32268000.0 ps

    if(p->frequency<=933)
    {
        RISCWriteDRAM(0x0fc , 0x910B0000);   //RISCWriteDRAM(0x0fc , 0x27030000);   //32277000.0 ps //CKEPRD //update actiming
        RISCWriteDRAM(0x1dc , 0x15656440);   //32286000.0 ps  //R_DMREFFRERUN=1  //MIOCKCTRLOFF=1 (always enable clk)
    }

    RISCWriteDRAM(0x348 , 0x00140408);   //32294000.0 ps
    RISCWriteDRAM(0x03c , 0x01010000);   //32357000.0 ps
    //decrease selftest run time
    RISCWriteDRAM(0x040 , 0x00000040);   //32365000.0 ps //2us

    if(p->frequency<=933)
    {
        RISCWriteDRAM(0x004 , 0xf0048403);   //32374000.0 ps
        RISCWriteDRAM(0x044 , 0xa8070400);   //32383000.0 ps  //DSIM_0301
        RISCWriteDRAM(0x048 , 0x2801110d);   //RISCWriteDRAM(0x048 , 0x2b01110d);   //32392000.0 ps //update actiming
    }
    RISCWriteDRAM(0x1e4 , 0x04FF2000);   //32445000.0 ps   //0616 CLR_EN

    RISCWriteDRAM(0x234 , 0x13010100);   //32463000.0 ps   //[25] REFR_BLOCKEN=1 ,


    RISCWriteDRAM(0x0f8 , 0x00002800);   //32481000.0 ps //mask


    RISCWriteDRAM(0x130 , 0x70000000);   //32499000.0 ps

    if(p->frequency<=933)
    {
        RISCWriteDRAM(0x1e8 , 0x15FF1F5D);   //32960000.0 ps  //for postsim [26]REFRDIS=1
    }
    //0320 shu_opt
    RISCWriteDRAM(0x958 , 0x00002000);  

    //start Turn on dqs-gating tracking
    //  ADDR=071dc with DATA=d3624340 //67960000.0 ps
    //  ADDR=071c4 with DATA=41044000 //68014000.0 ps
    //  ADDR=07138 with DATA=00908003 //68079000.0 ps
    //  ADDR=071c0 with DATA=a8180000 //68144000.0 ps
    //  ADDR=071dc with DATA=d3624342 //68209000.0 ps
    if(p->frequency<=933)
    {
        RISCWriteDRAM(0x1dc , 0x15656440 );   //MIOCKCTRLOFF=1 (always enable clk)
        RISCWriteDRAM(0x1c4 , 0x41044000 ); 
        //150707 change dramc selftest to dual rank access
        RISCWriteDRAM(0x138 , 0xa8908003 ); //bit [17:16] test agent rank sel
        ///TODO:ENABLE AFTER EVEREST_BRING_UP
        RISCWriteDRAM(0x1c0 , 0x08180000 ); //[31][29] turn off stbcal
        RISCWriteDRAM(0x1dc , 0x15656440 ); //MIOCKCTRLOFF=1 (always enable clk)
    }
    //end Turn on dqs-gating tracking
        
    //MRR SET
    *((volatile unsigned int *)(DRAMC_WBR)) = 0x5;
    //*((UINT32P)(DRAMC0_BASE + 0x1f4)) = 0x03020100;
    RISCWrite((DRAMC0_BASE  + 0x1f4 ) , 0x03020001);
    //*((UINT32P)(DRAMC0_BASE + 0x1fc)) = 0x06070405;
    RISCWrite((DRAMC0_BASE  + 0x1fc ) , 0x06070504);
    //*((UINT32P)(DRAMC1_BASE + 0x1f4)) = 0x03020100;
    RISCWrite((DRAMC1_BASE  + 0x1f4 ) , 0x03020001);
    //*((UINT32P)(DRAMC1_BASE + 0x1fc)) = 0x06070405;
    RISCWrite((DRAMC1_BASE  + 0x1fc ) , 0x06070504);
    *((volatile unsigned int *)(DRAMC_WBR)) = 0x7;

    /*TINFO="=== set ddrphy peri to spm control ==="*/
    ///TODO:ENABLE AFTER EVEREST_BRING_UP
    //*((volatile unsigned int *)(DRAMC_WBR)) = 0x7;        
    //RISCWriteDDRPHY(0x0580 , 0xffffffff);
    //RISCWriteDDRPHY(0x0584 , 0xffffffff);
    //RISCWriteDDRPHY(0x0588 , 0xffffffff);
    *((volatile unsigned int *)(DRAMC_WBR)) = 0x0;

    #if WRITE_LEVELING_MOVE_DQS_INSTEAD_OF_CLK
    memset(p->arfgWriteLevelingInitShif, FALSE, sizeof(p->arfgWriteLevelingInitShif));
    //>fgWriteLevelingInitShif= FALSE;
    #endif
    #if TX_PERBIT_INIT_FLOW_CONTROL
    memset(p->fgTXPerbifInit, FALSE, sizeof(p->fgTXPerbifInit));
    #endif

    #if RX_DLY_TRACKING_VERIFY
    DramcRxInputDlyTrackingRG(p);
    //ECO item : RK0/RK1 use different Rx DQS dly
    vIO32WriteFldAlign_All(DDRPHY_TXDQ3, 0, Fld(1,20,AC_MSKB2)); //TXDQ3_RG_ARDQ_REV_B0[4]
    vIO32WriteFldAlign_All(DDRPHY_RXDQ13, 0, Fld(1,20,AC_MSKB2)); //RXDQ13_RG_ARDQ_REV_B1[4]
    #endif
    
    //DVFS, DLL wait time
    #if DFS_SUPPORT_THIRD_SHUFFLE
        if(DFS_THIRD_SHUFFLE_FREQ==LC_DDR800)
            vIO32WriteFldAlign_All(DRAMC_REG_RESERVED_DRAMC_2, 0x1C, RESERVED_DRAMC_2_R_DLL_IDLE);//800
        else
            vIO32WriteFldAlign_All(DRAMC_REG_RESERVED_DRAMC_2, 0x15, RESERVED_DRAMC_2_R_DLL_IDLE);//1066
    #else
        vIO32WriteFldAlign_All(DRAMC_REG_RESERVED_DRAMC_2, 0x12, RESERVED_DRAMC_2_R_DLL_IDLE);//1270
    #endif

    #if JADE_TRACKING_MODE
    if(p->frequency <= 533)   //1066
    {
        vIO32WriteFldAlign_All(DRAMC_REG_DUMMY, 1, DUMMY_DMSTBLAT);
    }
    else if(p->frequency <= 670)   //1270
    {   
        vIO32WriteFldAlign_All(DRAMC_REG_DUMMY, 2, DUMMY_DMSTBLAT);
    }
    else //1600
    {
        vIO32WriteFldAlign_All(DRAMC_REG_DUMMY, 3, DUMMY_DMSTBLAT);    
    }
    #endif
}

#if COMPILE_A60817_LP3_INIT_CODE
static DRAM_STATUS_T DramcSetting_LP3(DRAMC_CTX_T *p)
{
        U32 u4RankIdx;

    //mcSHOW_DBG_MSG(("[DramcSetting_LP3] ====Begin====\n"));
    //mcFPRINTF((fp_A60501, "[DramcSetting_LP3] ====Begin====\n"));    

        // SELPH - GATING
        // RANK 0 HW DQS gating tracking (1x clock)
        // NO need to set @ init?
        // DQS0? -> TXDLY_DQSGATE, TXDLY_DQSGATE_P1  
        // LP4: 0x22222200
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0430), 0x101010ff);
        
        // SELPH - GATING
        // RANK 0 HW DQS gating tracking (0.5 * 2x clock)
        // NO need to set @ init?
        // DQS0? -> dly_DQSGATE, dly_DQSGATE_P1   
        // LP4: 0x62626200
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0434), 0x020202ff);
                
        // SELPH - GATING
        // RANK 1 HW DQS gating tracking (1x clock)
        // NO need to set @ init?
        // DQS0? -> TXDLY_R1DQSGATE, TXDLY_R1DQSGATE_P1
        // LP4: 0x33333300
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0438), 0x101010ff);
        
        // SELPH - GATING
        // RANK 1 HW DQS gating tracking (0.5 * 2x clock)
        // NO need to set @ init?
        // DQS0? -> dly_R1DQSGATE, dly_R1DQSGATE_P1
        // LP4: 0x51515100
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x043c), 0x020202ff);
        
        // SELPH - TX CMD
        // 1x clock
        // TXDLY_CS1 TXDLY_RAS TXDLY_CAS TXDLY_WE TXDLY_RESET TXDLY_ODT TXDLY_CKE TXDLY_CS
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0400), 0x00000000);
        
        // SELPH - TX CMD
        // 1x clock
        // TXDLY_CKE1 TXDLY_DQSGATE_P1 TXDLY_CMD TXDLY_DQSGATE TXDLY_BA2 TXDLY_BA1 TXDLY_BA0   
        // LP4: 0x00202000
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0404), 0x00100000);        
        
        // SELPH
        // reg_TX_DLY_RANK_MCK reg_dly_RANK TXDLY_RODTEN dly_RODTEN TXDLY_R1DQSGATE_P1 TXDLY_R1DQSGATE dly_R1DQSGATE_P1 dly_R1DQSGATE
        // LP4: 0x00020335
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0418), 0x00000102);
        
        // SELPH
        // TXDLY_RA7~0 (1x clock)
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0408), 0x00000000);
        
        // SELPH
        // TXDLY_RA15~8 (1x clock)
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x040c), 0x00000000);
        
        // SELPH
        // 0.5 * (2x clock)
        // dly_CKE1 dly_DQSGATE_P1 dly_DQSGATE dly_BA2 dly_BA1 dly_BA0 dly_CS1 dly_RAS dly_CAS dly_WE dly_RESET dly_ODT dly_CKE dly_CS  
        // LP4: 0x56855545
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0410), 0x54955555);

        // Write latency
        {
            // SELPH
            // 1x clock
            // TXDLY_OEN_DQ3 TXDLY_OEN_DQ2 TXDLY_OEN_DQ1 TXDLY_OEN_DQ0 TXDLY_DQ3 TXDLY_DQ2 TXDLY_DQ1 TXDLY_DQ0
            // LP4: 0x33333333
            vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x041c), 0x22222222);
            
            // SELPH
            // 1x clock
            // TXDLY_OEN_DQM3 TXDLY_OEN_DQM2 TXDLY_OEN_DQM1 TXDLY_OEN_DQM0 TXDLY_DQM3 TXDLY_DQM2 TXDLY_DQM1 TXDLY_DQM0
            // LP4: 0x33333333
            vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0420), 0x22222222);

            // SELPH
            // 1x clock
            // TXDLY_OEN_DQS3 TXDLY_OEN_DQS2 TXDLY_OEN_DQS1  TXDLY_OEN_DQS0 TXDLY_DQS3 TXDLY_DQS2 TXDLY_DQS1 TXDLY_DQS0
            // no rank1 DQS SELPH (only DQ/DQM)
            // LP4: 0x33333333
            vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0424), 0x22222222);

            // SELPH
            // 0.5 * (2x clock)
            // dly_oen_DQM3 dly_oen_DQM2 dly_oen_DQM1 dly_oen_DQM0 dly_oen_DQ3 dly_oen_DQ2 dly_oen_DQ1 dly_oen_DQ0 dly_DQM3         
            // dly_DQM2 dly_DQM1 dly_DQM0 dly_DQ3  dly_DQ2dly_DQ1 dly_DQ0
            // LP4: 0x0000ffff
            vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0428), 0x5555ffff);

            // SELPH
            // reg_TX_DLY_R1RODTEN_MCK reg_dly_R1RODTEN dly_oen_DQS3 dly_oen_DQS2 dly_oen_DQS1 dly_oen_DQS0 dly_DQS3 dly_DQS2 dly_DQS1 dly_DQS0
            // LP4: 0x00ff0055
            vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x042c), 0x005500ff);
        }

        // [31] ADRDECEN: 0: by EMI, 1: by DRAMC    
        // 6595: 0x2201ffff
        // DDR mode: [29] [25] [16:0] (NA on reg map)
        // LP4: 0x80000000
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x01e0), 0xa201ffff);
        
        // AC Timing 05T
        // LP4: 0x040000c1
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x01f8), 0x04000400);

        // NA on reg map (NA for A60501)
        // 6595: 0x2201ffff (Clock 1x phase selection)       
        // LP4: No set
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x023c), 0x2201ffff);

        // SELPH
        // 1x clock
        // TX_DLY_R1oen_DQ3 TX_DLY_R1oen_DQ2 TX_DLY_R1oen_DQ1 TX_DLY_R1oen_DQ0 TX_DLY_R1DQ3 TX_DLY_R1DQ2 TX_DLY_R1DQ1 TX_DLY_R1DQ0
        // LP4: 0x33333333
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0440), 0x22222222);
        
        // SELPH
        // 1x clock
        // TX_DLY_R1oen_DQM3 TX_DLY_R1oen_DQM2 TX_DLY_R1oen_DQM1 TX_DLY_R1oen_DQM0 TX_DLY_R1DQM3 TX_DLY_R1DQM2 TX_DLY_R1DQM1 TX_DLY_R1DQM0
        // LP4: 0x33333333
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0444), 0x22222222);

        // SELPH
        // 0.5 * (2x clock)
        // dly_R1DQM3 dly_R1DQM2 dly_R1DQM1 dly_R1DQM0 dly_R1DQ3 dly_R1DQ2 dly_R1DQ1 dly_R1DQ0
        // LP4: 0x77777777
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0448), 0x33333333);
        
        // SELPH
        // 0.5 * (2x clock)
        // dly_R1oen_DQM3 dly_R1oen_DQM2 dly_R1oen_DQM1 dly_R1oen_DQM0 dly_R1oen_DQ3 dly_R1oen_DQ2 dly_R1oen_DQ1 dly_R1oen_DQ0
        // LP4: 0x44444444
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x044c), 0x11111111);

        // SELPH
        // 0.5 * (2x clock)
        // extra bit (bit 2) for DQ*
        //vIO32Write4B_All(mcSET_DRAMC_AO_REG_ADDR(0x0458), 0x00ffff0f);
        
        // SELPH
        // 0.5 * (2x clock)
        // extra bit (bit 2) for CA
        //vIO32Write4B_All(mcSET_DRAMC_AO_REG_ADDR(0x0454), 0xa0000000);

        // [19:16] RANKINCTL?
        //vIO32Write4B_All(mcSET_DRAMC_AO_REG_ADDR(0x01c4), 0x00430000);

        // [7:0] REFCNT: disable refresh first (enable later)
        // [22] ?
        // [23] REFBW_FREN: enabled?
        // [17:8] REFBW_FR    
        // 6595: 0x0000006C
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0008), 0x00801700);

        // [31:24] TZQCS
        // [16] TESTXTALKPAT
        // 6595: 0x2701110D            
        //RISCWrite : address 60048 data 2200110d wait  0    
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0048), 0x2201110d);

        // [0]? NA on reg map
        // [23] PHYPIPE3EN?
        // [22] PHYPIPE2EN?
        // [21] PHYPIPE1EN?
        // 6595: 0x00e00000
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x008c), 0x00000001);

        // [8] DQSIEN pulse mode DQSIEN tracking, 0: pulse mode 1: burst mode->for sim. [DP] Set pulse mode @ HW tracking
        // [11:10] DQIENQKEND, 2'b01: bigger input window
        // 6595: 0x00100110    
        //vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x00d8), 0x00100510);  
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x00d8), 0x00100410);//pulse mode

        // [13] [8]: NA on reg map?
        // [1] GDDR3RST
        // LP4: 0x00002103
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x00e4), 0x00002101);

        // DQ driving?
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x00b8), 0x99169952);

        // CLK driving?
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x00bc), 0x99109950);

        // NA on reg map?
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0090), 0x00000000);

        // NA on reg map? 
        // LP4: 0x83000000
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x00dc), 0x83200200);

        // [28] DQSIENMODE, DQS gating mode selection 0: pulse mode 1: burst mode; set 1, don't set 0
        // [27:24] DQSINCTL
        // LP4: 0x15000000
        // [21], [9] NA on reg map
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x00e0), 0x16200200);

        // [3:0] R1DQSINCTL, set as R0? 
        // LP4: 0x00000000
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0118), 0x00000004);

        //RISCWrite : address 900f0 data 40000000 wait  0
        //RISCWrite : address 700f0 data 40000000 wait  0
        //RISCWrite : address 800f0 data 40000000 wait  0
        // NA on reg map, NO need to do

        // DDRPHY
        // [3] R_DM_TX_ARCMD_OE
        // [2] R_DM_TX_ARCLK_OE 
        //vIO32Write4B_All(mcSET_DDRPHY_REG_ADDR(0x05c0), 0x0000000c);

        // [24] _8BKEN
        // 6595: 0x11000000, [28] PHYSYNCM (NA on A60501?)
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x00f4), 0x01000000);
        
        //Wrok around only for A60501
        // 1.	CBT may cause problem because block_by_maxpend (due to wrong write_data_buffer_pop behavior)
        //a. do CBT offline and run normal access alone, and
        //b. disable block by maxpend: set R_DMMAXPENDCNT(dramc AO 0x168[7:0])=0
        #if 0  //normal
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0168), 0x00000080);
        #else // work around
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0168), 0x00000000);
        #endif
        
        // [29] CLK_EN_1?
        // [28] CLK_EN_0
        // 6595: 0x10000000 
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0130), 0x30000000);
        
        // [8] DQSIEN pulse mode DQSIEN tracking, 0: pulse mode 1: burst mode->for sim. [DP] Set pulse mode @ HW tracking
        // [21] NA on reg map?
        // 6595: 0x00100110    
        //vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x00d8), 0x00300510);
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x00d8), 0x00300410);//pulse mode
        
        // [31:30] SREF_CK_DLY, 6595 -> 00, K2 -> 11, 60501?
        // [29:28] TCKESRX (new)
        // [9:8] MATYPE -> 10 (LPDDR3/4 at least 10 column bits. LPDDR4 design only support >10bits)
        // 6595: 0x00048403, K2: 0xC00484C3     
        // LP4: 0xf0048683
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0004), 0xf00486c3);
        
        // NA on reg map
        // 6595: 0xc0000011
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0124), 0xc0000011);
        
        // [31] READ_START_EXTEND1
        // [15] DISRDPHASE1 (NEW): set DISRDPHASE1=1'b0 and DMRCDRSV=1'b1 to disable read at phase 1 function.
        // [8] DMRCDRSV; DISRDPHASE1=1, DMRCDRSV=0, is enable.
        // [4] CKEEXTEND; set to 1?
        // [11:10] DMSTBLAT: DLLFRZ_pulse Latency for stb_sm, 2'b00: original setting, it is the same as 89/82?
        // [3:0] RANKINCTL_ROOT1
        // 6595: 0x80000c10
        // LP4: 0x00008000
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0138), 0x00008003);
        
        // rank0 gating delay control
        // not use in A60501?
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0094), 0x00000000);
        
        // rank1 gating delay control
        // not use in A60501?
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0098), 0x00000000);
        
        // [16] NA on reg map?
        // [20] STBSTATE_OPT: HW DQS gating tracking mode ?
        //                    1'b0: tracking when every dle_enable 
        //                    1'b1: tracking when DLLFRZ
        // [24] NA on reg map?
        // [28] REFUICHG: HW DQS gating tracking update UI delay option? 
        //                1'b1: update it when cross rank and DLLFRZ 
        //                1'b0: update it only when cross rank
        // [29] STB_SELPHCALEN: HW DQS gating tracking update UI delay enable: 1'b1: enable 1'b0: disable?
        // [21] PHYVALID_IG: HW DQS gating tracking reference?
        //                   1'b1: DQS gating tracking without  DDRPHY tracking valid information 
        //                   1'b0: DQS gating tracking only be actived when DDRPHY tracking valid information is hihg 
        // 6595: 0x00000000 (reg re-define)
        // LP4: 0x11110000
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x01c0), 0x01010000);
        
        // NEW
        // [31:16] ACTEN_ROW
        // [14:12] ACTEN_BK
        // [8:8]   SRFPD_DIS
        // [7:4]   IORGTIM
        // [3:2]   CKEHCMD: SREFX, 2'b10:3 MCK
        // [1:1]   IORGCNTRST
        // [0:0]   IORGCNTEN
        //vIO32Write4B_All(mcSET_DRAMC_AO_REG_ADDR(0x0240), 0x00000008);
        
        // AC timing    
        // LP4: 0x668d4719
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0000), 0x99cb4649);
        
        // [26] REFP_ARB_EN? Per-bank refresh blocks EMI arbitration 0: disable 1: enable
        // [25] REFA_ARB_EN? All-bank refresh blocks EMI arbitration 0: disable 1: enable
        // [20] [21] NA on reg map?
        // [18:16] CKEPRD
        // 6595: 0x21000000, K2: 0x27000000           
        // LP4: 0x17030000
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x00fc), 0x17020000);
        
        // AC_TIME_05T    
        // LP4: 0x040000c1
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x01f8), 0x04000400);
        
        // [31] WRFIFOEN (NEW): LPDDR4 MPC WR FIFO enable
        // [30] RDFIFOEN (NEW): LPDDR4 MPC RD FIFO enable
        // [29:28] RDDQC_INTV (NEW): LPDDR4 RD DQ Calibration loop interval, unit M_CK cycle.
        // [26] RDDQCDIS (NEW): LPDDR4 RD DQ Calibration loop stop
        // [21] TWPSTEXT (NEW)
        // [2] DMIWR (NEW): LPDDR4 write data bus inversion
        // [1] WPST2T (NEW): LPDDR4 write post-amble option: 1'b0: 1T 1'b1: 2T
        // 6595: 0x00100001 
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x01ec), 0x10100003);
        //vIO32Write4B_All(mcSET_DRAMC_AO_REG_ADDR(0x01ec), 0x10100002); //disble PERFCTL0_DUALSCHEN for test
        
        // [26:24] RODT
        // [23] NA on reg map?
        // [22:16] TWODT
        // [15:12] TR2W
        // [10:8] TRTP
        // [3] WOEN (0)
        // [2] ROEN (0)
        // [0] FDIV2 (1)
        // 6595: 0x000064B1, K2: 0x000032B1
        // LP4: 0x0781548c
        // RISCWrite : address 6007c data e5b8538d
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x007c), 0xe5b85381);

        // [31:28] ADVREF_CNT? Advanced refresh count
        // [26:24] CATRAINLAT? Internal read data timing control for CA trainging of LPDDR4 data compare
        // [20:16] DATLAT
        // [12:8] DATLAT_DSEL
        // [7] PHYRXPIPE3?
        // [6] PHYRXPIPE2?
        // [5] PHYRXPIPE1?
        // 6595: 0x00000be0, K2: 0x00f00ae0
        // LP4: 0x700b0b00
        //vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0080),  0x000a0800);
        //vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0080),  0x000b0b00);
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0080),  0x000d0d00);

        // [30] DLLFRZ: Auto-calibration value update when refresh cycle
        // [29] DMPAT32 (NEW): 1'b1: selftest2 use 32bit pattern; 1'b0: selftest2 use 16 bit pattern
        // [28] MDQS: Manual mode for DQS input delay setting
        // [19] MANUTXUPD (NEW): runtime modify TX DQ setting, manual update
        // [18] TXUPMODE (NEW): runtime modify TX DQ setting, update mode
        // [17] DMSHU_DRAMC (NEW): dramc shuffle
        // [16] DMSHU_LOW (NEW): shuffle low
        // 6595: 0xf1000000, K2: 0xf1200f01
        // LP4: 0x50000000
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0028), 0x50200000);

        // [31] FASTWAKE (MT6595 enabled), [D] enable, enabled at the last?
        //      It is controlled by EMI, not used in test chip.       
        // [29] LPDDR4EN   
        // [28] LPDDR3EN
        // [26] 1 :Disable MR4 refresh rate update
        // [23:16] REFRCNT: MR4 read                                  
        // [25:0] AC timing (see reg map)                             
        // [15:8] TRFCPB (0x00): ?     
        // LP4: 21000001
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x01e8), 0x15001101);

        // NA on reg map    
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0158), 0x00000000);

        // has set before (00002101)
        // [13] [8]: NA on reg map?
        // [2] CKEFIXON
        // 6595/K2: 0x00000005
        // exit Power Down 
        // LP4: 0x00000007
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x00e4), 0x00000005);
        
        mcDELAY_US(200);	// tINIT3 > 200us

        //Set mode register settings for all ranks
        //for(u4RankIdx =(p->support_rank_num-1); u4RankIdx>0; u4RankIdx--)
        for(u4RankIdx =0; u4RankIdx < (U32)(p->support_rank_num); u4RankIdx++)
        {
            mcSHOW_DBG_MSG(("DramcModeRegInit_LP3 for Rank%d\n", u4RankIdx));          
            mcFPRINTF((fp_A60501, "DramcModeRegInit_LP3 for Rank%d\n", u4RankIdx));    

            DramcRankSwap(p, u4RankIdx);
            DramcModeRegInit_LP3(p);
        }        

        // RISCWrite : address 601e4 data 00050000
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x01e4), 0x00000000);
        
        // [22] CLKWITRFC? (add in K2 (1: disabled due to bug), A60501? -> NA)
        // [20:18] XRTW2W
        // [15:12] XRTR2W
        // [2:0] RKMODE?
        // [5] DQSOSC2RK (NEW)
        // [7] PBREFEN? Per bank refresh enable
        // [6] PBREF_DISBYRATE? disable per bank refresh when refresh rate > 3'h5
        // 6595: 0x00215641, K2: 0x00615641 
        // LP4: 0x00697780
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0110),  0x00291700);  //disbale perbank rerfresh beacuse of LP3 Hynix problem.

        // [25] DCMEN, enabled later...?
        // [24] REFFRERUN: Using FREE-RUN CLK to count refresh period? (set to 1?)
        // [23:16] REFCNT_FR_CLK
        // [15:8] tXSR
        // [1] NA on reg map?    
        // 6595: 0xD1643D42, K2: 0xD10B6442    
        // LP4: 0xd0625440
        // RISCWrite : address 601dc data d2623840
            //enable free-run clk 26Mhz, and refresh rate 0x62(3.8ns)
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x01dc), 0xd1623840);

        // has set before (3 times, 0x00002103 / 0x00000007)
        // [8], [13] NA on reg map
        // [2] CKEFIXON=0 after MRS
        // LP4: 0x00002103
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x00e4), 0x00002101);

        // has set before, the same value
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x01ec), 0x10100003);
        //vIO32Write4B_All(mcSET_DRAMC_AO_REG_ADDR(0x01ec), 0x10100002);//disble PERFCTL0_DUALSCHEN for test

        // [15:8] ZQCSAD?
        // [7:0] ZQCSOP?
        // 6595 / K2: 0x00000a56  
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0084), 0x00000a56);
        
        // NA on reg map
        // 6595 / K2: 0x00000000
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x000c), 0x00000000);

        // has set before, the same value
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0000), 0x99cb4649);

        // [31]:ADVPREEN :Advanced precharge function enable, When page is idle for DMPGTIM cycles, the page is closed automatically
        // [29:24] DMPGTIM: Advanced precharge function timer?
        // [23] DQSICALI_NEW?
        // [22:20] DQSICALBLCOK_CNT?
        // [19:16] TRFC
        // [3:0] TESTCNT
        // 6595: 0xBFC50401, K2: 0xBFC80401
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0044), 0xa8090400);

        // has set before (0x11001101)
        // [31] FASTWAKE?
        // [30] DAREFEN: for LPDDR4 only
        //               1'b1: per-bank refresh by DAR(directed bank)
        //               1'b0: per-bank refresh by RR(round robin bank) 
        //               set 0. (DAR is not ready.)
        // [29] LPDDR4EN
        // [28] LPDDR3EN
        // [27] WPRE2T: 2T
        // [26] 1 :Disable MR4 refresh rate update
        // [23:16] REFRCNT: MR4 read (enable @ run time?)
        // [15:8] TRFCPB
        // [7:4] TRFC_BIT7_4
        // [3] TRRD_BIT2
        // [1] TFAW_BIT4
        // [0] TRC_BIT4
        // 6595: 0x9100256B, K2: 0x91000D21
        // LP4: 0x29002041
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x01e8), 0x15001941);

        // has set before (0x00403300)
        // [7:0] REFCNT: AC timing
        // LP4: 0x00403360
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0008), 0x00801761);

        // NA on reg map
        // 6595: 0x00000000 
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0010), 0x00000000);

        // [24] DRVREF: Driving change only when refresh cycle?
        // [15] DRDELSWEN: Enable DQS input delay switching for different ranks?
        // [8] NA on reg map (6595: AUTOCALDRV)
        // [7:0] NA on reg map (6595: AUTOKCNT)
        // 6595: 0x00000000, K2: 0x00008110
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0100), 0x01008110);

        // R1DELDLY: RANK1 DQS3 input delay line setting?
        // 6595 / K2: 0x12121212 
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x001c), 0x12121212);

        // 6595 / K2: 0x00000000
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x00f8), 0x00000000);

        // has set before, the same value
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x00fc), 0x17020000);

        // has set before, the same value
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x01dc), 0xd1623840);

        // NEW
        // [31:16] STBCAL_FILTER
        // [15:15] STBCNT_SW_RST
        // [14:14] DQSERRCNT_DIS
        // [13:13] DCBLNCINS
        // [12:12] DCBLNCEN
        // [11:11] STBCNT_LATCH_EN
        // [10:10] STBENCMPEN
        // [7:4]   RX_DQ_EYE_SEL
        // [3:3]   STB_DLLFRZ_IG
        // [2:2]   RG_RX_MIOCK_JIT_EN
        // [1:1]   RG_EX_EYE_SCAN_EN
        // [0:0]   reg_sw_rst
        vIO32Write4B(mcSET_DRAMC_AO_REG_ADDR_CHC(0x0348), 0x00010008);  


        //vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHC(0x0434), 0x7fff0000);  
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHA(0x0434), 0x7bff0000);  //CHA [26] =0
        vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHB(0x0434), 0x7bff0000);  //CHB [26] =0

    #if WRITE_LEVELING_MOVE_DQS_INSTEAD_OF_CLK
    memset(p->arfgWriteLevelingInitShif, FALSE, sizeof(p->arfgWriteLevelingInitShif));
    //>fgWriteLevelingInitShif= FALSE;
    #endif
    #if TX_PERBIT_INIT_FLOW_CONTROL
    memset(p->fgTXPerbifInit, FALSE, sizeof(p->fgTXPerbifInit));
    #endif
    return DRAM_OK;
}
#endif //COMPILE_A60817_LP3_INIT_CODE

enum
{
    AC_TIMING_DRAM_TYPE=0,
    AC_TIMING_FREQUENCY,

    AC_TIMING_TRAS,
    AC_TIMING_TRP,
    AC_TIMING_TRPAB,
    AC_TIMING_TRC,
    AC_TIMING_TRFC,
    AC_TIMING_TRFCPB,
    AC_TIMING_TXP,
    AC_TIMING_TRTP,
    AC_TIMING_TRCD,
    AC_TIMING_TWR,
    AC_TIMING_TWTR,
    AC_TIMING_TRRD,
    AC_TIMING_TFAW,
    AC_TIMING_TRTW,
    AC_TIMING_REFCNT, //(REFFRERUN = 0)
    AC_TIMING_REFCNT_FR_CLK, //(REFFRERUN = 1)
    AC_TIMING_TXREFCNT,
    AC_TIMING_TZQCS,

    AC_TIMING_TRAS_05T,
    AC_TIMING_TRP_05T,
    AC_TIMING_TRPAB_05T,
    AC_TIMING_TRC_05T,
    AC_TIMING_TRFC_05T,
    AC_TIMING_TRFCPB_05T,
    AC_TIMING_TXP_05T,
    AC_TIMING_TRTP_05T,
    AC_TIMING_TRCD_05T,
    AC_TIMING_TWR_05T,
    AC_TIMING_TWTR_05T,
    AC_TIMING_TRRD_05T,
    AC_TIMING_TFAW_05T,
    AC_TIMING_TRTW_05T,
    AC_TIMING_REFCNT_05T, //(REFFRERUN = 0)
    AC_TIMING_REFCNT_FR_CLK_05T, //(REFFRERUN = 1)
    AC_TIMING_TXREFCNT_05T,
    AC_TIMING_TZQCS_05T,

    AC_TIMING_XRTW2W,
    AC_TIMING_XRTW2R,
    AC_TIMING_XRTR2W,
    AC_TIMING_XRTR2R,

    AC_TIMING_WRITE_LATENCY_0x41c,
    AC_TIMING_WRITE_LATENCY_0x420,
    AC_TIMING_WRITE_LATENCY_0x424,
    AC_TIMING_WRITE_LATENCY_0x428,
    AC_TIMING_WRITE_LATENCY_0x42c,

    AC_TIMING_DQSINCTL_FOR_GATING,
    AC_TIMING_DATLAT,
    AC_TIMING_MODE_REG_WL,
    AC_TIMING_MODE_REG_RL,

    AC_TIMING_ITEM_NUM
};

//Everest
#if 0
#define DFS_FREQ_LJ         933
#define DFS_FREQ_LC_HIGH    800
#define DFS_FREQ_LC_LOW     635
#define DFS_FREQ_LC_SLEEP   533
#endif


#if SUPPORT_LP3_800
#define TOTAL_AC_TIMING_NUMBER  5
#else
#define TOTAL_AC_TIMING_NUMBER  4
#endif

const U32 ACTimingTable[TOTAL_AC_TIMING_NUMBER][AC_TIMING_ITEM_NUM] = 
{
    // LP3-1866, 933MHz
   {
        TYPE_LPDDR3 , // Dram type
        933 , // Dram frequency
            
            //====  AC timing ====
        12  , // 0 : TRAS
        10  , // 1 : TRP
        1   , // 2: TPRAB
        24  , // 3:TRC
        87  , // 4:TFRC
        31  , // 5:TRFCPB
        1   , //6:TXP
        3   , //7:TRTP
        10  , //8:TRCD
        13  , //9:TWR
        7   , //10: TWTR
        4   , //11:TRRD
        15  , //12:TFAW
#if JADE_TRACKING_MODE
        7   , //13:TRTW
#else
        6   , //13:TRTW
#endif
        112 , //14:REFCNT
        101 , //15:REFCNT_FR_CLK
        100 , //16:TXREFCNT
        40  , //17:TZQCS
            
            // ====  AC timing 0.5T====  
        0   ,// TRAS_0.5T
        1   ,// TRP_0.5T
        1   ,// TRPAB_0.5T
        1   ,// TRC_0.5T
        0   ,// TRFC_0.5T
        0   ,// TRFCPB_0.5T
        1   ,// TXP_0.5T
        1   ,// TRTP_0.5T
        1   ,// TRCD_0.5T
        1   ,// TWR_0.5T
        0   ,// TWTR_0.5T
        0   ,// TRRD_0.5T
        1   ,// TFAW_0.5T
#if JADE_TRACKING_MODE
        0   ,// TRTW_0.5T
#else
        1   , // TRTW_0.5T
#endif
        0   ,// REFCNT(REFFRERUN = 0)
        0   ,// REFCNT_FR_CLK(REFFRERUN = 1)
        0   ,// TXREFCNT
        0   ,// TZQCS_0.5T
            
            // Cross rank setting
        2   , // XRTW2W
#if JADE_TRACKING_MODE
        5   , // XRTW2R
#else
        1   , // XRTW2R
#endif
        6   , // XRTR2W
        8   , // XRTR2R
            
            // ====  Write latency====  
        0x33333333  , // 41ch
        0x33333333  , // 420h
        0x33333333  , // 424h
        0x5555FFFF  , // 428h
        0x5500FF    , // 42ch
            
        4   ,  // DQSINCTL
        20 , // Datlat
        8,    // TODO: WL
        14   // TODO: RL
    },    
    
    // LP3-1600, 800MHz
    {
        TYPE_LPDDR3 , // Dram type
        800 , // Dram frequency
            
            //====  AC timing ====
        9   , // 0 : TRAS
        9   , // 1 : TRP
        1   , // 2: TPRAB
        20  , // 3:TRC
        73  , // 4:TFRC
        25  , // 5:TRFCPB
        1   , //6:TXP
        2   , //7:TRTP
        9   , //8:TRCD
        11  , //9:TWR
        6   , //10: TWTR
        3   , //11:TRRD
        12  , //12:TFAW
#if JADE_TRACKING_MODE
        6   , //13:TRTW
#else
        5   , //13:TRTW
#endif
        96  , //14:REFCNT
        101 , //15:REFCNT_FR_CLK
        85  , //16:TXREFCNT
        34  , //17:TZQCS
            
            // ====  AC timing 0.5T====  
        0   ,// TRAS_0.5T
        0   ,// TRP_0.5T
        0   ,// TRPAB_0.5T
        0   ,// TRC_0.5T
        0   ,// TRFC_0.5T
        0   ,// TRFCPB_0.5T
        0   ,// TXP_0.5T
        0   ,// TRTP_0.5T
        0   ,// TRCD_0.5T
        1   ,// TWR_0.5T
        1   ,// TWTR_0.5T
        0   ,// TRRD_0.5T
        0   ,// TFAW_0.5T
#if JADE_TRACKING_MODE
        0   ,// TRTW_0.5T
#else
        1   , // TRTW_0.5T
#endif
        0   ,// REFCNT(REFFRERUN = 0)
        0   ,// REFCNT_FR_CLK(REFFRERUN = 1)
        0   ,// TXREFCNT
        0   ,// TZQCS_0.5T
            
            // Cross rank setting
        2   , // XRTW2W
#if JADE_TRACKING_MODE
            5   , // XRTW2R
#else
        1   , // XRTW2R
#endif
        6   , // XRTR2W
        8   , // XRTR2R
    
        // ====  Write latency====  
        0x22222222  , // 41ch
        0x22222222  , // 420h
        0x22222222  , // 424h
        0x5555FFFF  , // 428h
        0x5500FF    , // 42ch
            
        3   ,  // DQSINCTL
        19 , // Datlat
        6, 
        12
    },

    // LP3-1270, 635MHz
    {
    TYPE_LPDDR3 , // Dram type
        635 , // Dram frequency
            
            //====  AC timing ====
        5   , // 0 : TRAS
        7   , // 1 : TRP
        1   , // 2: TPRAB
        14  , // 3:TRC
        56  , // 4:TFRC
        18  , // 5:TRFCPB
        0   , //6:TXP
        2   , //7:TRTP
        7   , //8:TRCD
        10  , //9:TWR
        5   , //10: TWTR
        2   , //11:TRRD
        8   , //12:TFAW
#if JADE_TRACKING_MODE
        5   , //13:TRTW
#else
        4   , //13:TRTW
#endif
        76  , //14:REFCNT
        101 , //15:REFCNT_FR_CLK
        67  , //16:TXREFCNT
        27  , //17:TZQCS
            
            // ====  AC timing 0.5T====  
        1   ,// TRAS_0.5T
        0   ,// TRP_0.5T
        0   ,// TRPAB_0.5T
        0   ,// TRC_0.5T
        0   ,// TRFC_0.5T
        0   ,// TRFCPB_0.5T
        1   ,// TXP_0.5T
        1   ,// TRTP_0.5T
        0   ,// TRCD_0.5T
        1   ,// TWR_0.5T
        0   ,// TWTR_0.5T
        1   ,// TRRD_0.5T
        0   ,// TFAW_0.5T
#if JADE_TRACKING_MODE
        0   ,// TRTW_0.5T
#else
        1   , // TRTW_0.5T
#endif
        0   ,// REFCNT(REFFRERUN = 0)
        0   ,// REFCNT_FR_CLK(REFFRERUN = 1)
        0   ,// TXREFCNT
        0   ,// TZQCS_0.5T
            
            // Cross rank setting
        2   , // XRTW2W
#if JADE_TRACKING_MODE
            5   , // XRTW2R
#else
        1   , // XRTW2R
#endif
        6   , // XRTR2W
        8   , // XRTR2R
            
            // ====  Write latency====  
        0x22222222  , // 41ch
        0x22222222  , // 420h
        0x22222222  , // 424h
        0x5555FFFF  , // 428h
        0x5500FF    , // 42ch
            
        3   ,  // DQSINCTL
        18 , // Datlat
        6,
        10,
    },
    
    // LP3-1066, 533MHz
    {
        TYPE_LPDDR3 , // Dram type
        533 , // Dram frequency
            
            //====  AC timing ====
        3   , // 0 : TRAS
        5   , // 1 : TRP
        1   , // 2: TPRAB
        10  , // 3:TRC
        45  , // 4:TFRC
        13  , // 5:TRFCPB
        0   , //6:TXP
        1   , //7:TRTP
        5   , //8:TRCD
        8   , //9:TWR
        4   , //10: TWTR
        2   , //11:TRRD
        5   , //12:TFAW
#if JADE_TRACKING_MODE
        4   , //13:TRTW
#else
        3   , //13:TRTW
#endif
        63  , //14:REFCNT
        101 , //15:REFCNT_FR_CLK
        56  , //16:TXREFCNT
        22  , //17:TZQCS
            
            // ====  AC timing 0.5T====  
        1   ,// TRAS_0.5T
        1   ,// TRP_0.5T
        0   ,// TRPAB_0.5T
        1   ,// TRC_0.5T
        0   ,// TRFC_0.5T
        0   ,// TRFCPB_0.5T
        0   ,// TXP_0.5T
        0   ,// TRTP_0.5T
        1   ,// TRCD_0.5T
        1   ,// TWR_0.5T
        1   ,// TWTR_0.5T
        0   ,// TRRD_0.5T
        1   ,// TFAW_0.5T
#if JADE_TRACKING_MODE
        0   ,// TRTW_0.5T
#else
        1   , // TRTW_0.5T
#endif
        0   ,// REFCNT(REFFRERUN = 0)
        0   ,// REFCNT_FR_CLK(REFFRERUN = 1)
        0   ,// TXREFCNT
        0   ,// TZQCS_0.5T
            
            // Cross rank setting
        2   , // XRTW2W
#if JADE_TRACKING_MODE
            5   , // XRTW2R
#else
        1   , // XRTW2R
#endif
        6   , // XRTR2W
        8   , // XRTR2R
            
            // ====  Write latency====  
        0x11111111  , // 41ch
        0x11111111  , // 420h
        0x11111111  , // 424h
        0x5555FFFF  , // 428h
        0x5500FF    , // 42ch
            
        2  ,  // DQSINCTL
        16 , // Datlat
        4, 
        8,
    },

#if SUPPORT_LP3_800
    // LP3-800, 400MHz
    {
        TYPE_LPDDR3 , // Dram type
        400 , // Dram frequency
            
            //====  AC timing ====
        0   , // 0 : TRAS
        4   , // 1 : TRP
        0   , // 2: TPRAB
        6  , // 3:TRC
        31  , // 4:TFRC
        7  , // 5:TRFCPB
        0   , //6:TXP
        1   , //7:TRTP
        4   , //8:TRCD
        8   , //9:TWR
        5   , //10: TWTR
        1   , //11:TRRD
        2   , //12:TFAW
#if JADE_TRACKING_MODE
        4   , //13:TRTW
#else
        3   , //13:TRTW
#endif
        47  , //14:REFCNT
        101 , //15:REFCNT_FR_CLK
        41  , //16:TXREFCNT
        16  , //17:TZQCS

            // ====  AC timing 0.5T====  
        1   ,// TRAS_0.5T
        0   ,// TRP_0.5T
        1   ,// TRPAB_0.5T
        0   ,// TRC_0.5T
        0   ,// TRFC_0.5T
        0   ,// TRFCPB_0.5T
        0   ,// TXP_0.5T
        0   ,// TRTP_0.5T
        0   ,// TRCD_0.5T
        1   ,// TWR_0.5T
        1   ,// TWTR_0.5T
        0   ,// TRRD_0.5T
        0   ,// TFAW_0.5T
#if JADE_TRACKING_MODE
        0   , // TRTW_0.5T
#else
        1   , // TRTW_0.5T
#endif
        0   ,// REFCNT(REFFRERUN = 0)
        0   ,// REFCNT_FR_CLK(REFFRERUN = 1)
        0   ,// TXREFCNT
        0   ,// TZQCS_0.5T
            
            // Cross rank setting
        2   , // XRTW2W
#if JADE_TRACKING_MODE
            5   , // XRTW2R
#else
            1, // XRTW2R
#endif
        6   , // XRTR2W
        8   , // XRTR2R
            
            // ====  Write latency====  
        0x22222222  , // 41ch
        0x22222222  , // 420h
        0x22222222  , // 424h
        0x5555FFFF  , // 428h
        0x5500FF    , // 42ch
            
        2  ,  // DQSINCTL
        16 , // Datlat
        6,  // ==> 1270 setting
        10,  // ==> 1270 setting
    },
#endif//#if SUPPORT_LP3_800
};

DRAM_STATUS_T DdrUpdateACTimingReg(DRAMC_CTX_T *p, U32* ACTable)
{
    if(ACTable == NULL)
        return DRAM_FAIL;
    
    vIO32WriteFldMulti_All(DRAMC_REG_ADDR(DRAMC_REG_ACTIM0),    \
                                    P_Fld( ACTable[AC_TIMING_TRCD], ACTIM0_TRCD)         | \
                                    P_Fld( ACTable[AC_TIMING_TRP], ACTIM0_TRP)               |   \
                                    P_Fld( ACTable[AC_TIMING_TFAW], ACTIM0_TFAW)         |    \
                                    P_Fld( ACTable[AC_TIMING_TWR], ACTIM0_TWR)           |   \
                                    P_Fld((ACTable[AC_TIMING_TWR] >> 4) &0x1, ACTIM0_TWR_BIT4 ) |    \
                                    P_Fld( ACTable[AC_TIMING_TWTR], ACTIM0_TWTR)                             |    \
                                    P_Fld( ACTable[AC_TIMING_TRC], ACTIM0_TRC)                                   |  \
                                    P_Fld( ACTable[AC_TIMING_TRAS], ACTIM0_TRAS));

    vIO32WriteFldMulti_All(DRAMC_REG_ADDR(DRAMC_REG_ACTIM1),    \
                                    P_Fld( ACTable[AC_TIMING_TRPAB], ACTIM1_TRPAB)         | \
                                    P_Fld( ACTable[AC_TIMING_TRFCPB], ACTIM1_TRFCPB)               |   \
                                    P_Fld((ACTable[AC_TIMING_TRFC] >>4) & 0xf, ACTIM1_TRFC_BIT7_4)         |    \
                                    P_Fld((ACTable[AC_TIMING_TRRD]>>2), ACTIM1_TRRD_BIT2)           |   \
                                    P_Fld((ACTable[AC_TIMING_TFAW] >> 4) &0x1, ACTIM1_TFAW_BIT4 ) |    \
                                    P_Fld((ACTable[AC_TIMING_TRC]>> 4), ACTIM1_TRC_BIT4));

    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_CONF1), ACTable[AC_TIMING_TRRD] & 0x3, CONF1_TRRD);
    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_TEST2_3), ACTable[AC_TIMING_TRFC] & 0xf, TEST2_3_TRFC);
    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_MISCTL0), ACTable[AC_TIMING_TXP], MISCTL0_TXP);
    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_DDR2CTL), ACTable[AC_TIMING_TRTP], DDR2CTL_TRTP);
    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_DDR2CTL), ACTable[AC_TIMING_TRTW], DDR2CTL_TR2W);
    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_TEST2_4), ACTable[AC_TIMING_TZQCS], TEST2_4_TZQCS);


    // AC timing 0.5T
    vIO32WriteFldMulti_All(DRAMC_REG_ADDR(DRAMC_REG_AC_TIME_05T),    \
                                    P_Fld(ACTable[AC_TIMING_TRAS_05T],      AC_TIME_05T_TRAS_05T)         | \
                                    P_Fld(ACTable[AC_TIMING_TRP_05T],        AC_TIME_05T_TRP_05T)               |   \
                                    P_Fld(ACTable[AC_TIMING_TRPAB_05T],    AC_TIME_05T_TRPAB_05T)         |    \
                                    P_Fld(ACTable[AC_TIMING_TRC_05T],        AC_TIME_05T_TRC_05T)           |   \
                                    P_Fld(ACTable[AC_TIMING_TRFC_05T],       AC_TIME_05T_TRFC_05T ) |    \
                                    P_Fld(ACTable[AC_TIMING_TRFCPB_05T],     AC_TIME_05T_TRFCPB_05T)         | \
                                    P_Fld(ACTable[AC_TIMING_TXP_05T],        AC_TIME_05T_TXP_05T)               |   \
                                    P_Fld(ACTable[AC_TIMING_TRTP_05T],       AC_TIME_05T_TRTP_05T)         |    \
                                    P_Fld(ACTable[AC_TIMING_TRCD_05T],   AC_TIME_05T_TRCD_05T)           |   \
                                    P_Fld(ACTable[AC_TIMING_TWR_05T] ,    AC_TIME_05T_TWR_M05T ) |    \
                                    P_Fld(ACTable[AC_TIMING_TWTR_05T],   AC_TIME_05T_TWTR_M05T)         | \
                                    P_Fld(ACTable[AC_TIMING_TRRD_05T],   AC_TIME_05T_TRRD_05T)               |   \
                                    P_Fld(ACTable[AC_TIMING_TFAW_05T] ,  AC_TIME_05T_TFAW_05T)         |    \
                                    P_Fld(ACTable[AC_TIMING_TRTW_05T],   AC_TIME_05T_TR2W_05T)           
                                    );

    vIO32WriteFldMulti_All(DRAMC_REG_ADDR(DRAMC_REG_RKCFG),    \
                                    P_Fld(ACTable[AC_TIMING_XRTW2W], RKCFG_XRTW2W)         | \
                                    P_Fld(ACTable[AC_TIMING_XRTW2R] & 0x3, RKCFG_XRTW2R)               |   \
                                    P_Fld((ACTable[AC_TIMING_XRTW2R] >>2) & 0x1, RKCFG_XRTW2R_BIT2)               |   \
                                    P_Fld(ACTable[AC_TIMING_XRTR2W], RKCFG_XRTR2W)         |    \
                                    P_Fld(ACTable[AC_TIMING_XRTR2R], RKCFG_XRTR2R));

    // Write latency
    vIO32Write4B_All(DRAMC_REG_ADDR(DRAMC_REG_SELPH7), ACTable[AC_TIMING_WRITE_LATENCY_0x41c]);
    vIO32Write4B_All(DRAMC_REG_ADDR(DRAMC_REG_SELPH8), ACTable[AC_TIMING_WRITE_LATENCY_0x420]);
    vIO32Write4B_All(DRAMC_REG_ADDR(DRAMC_REG_SELPH9), ACTable[AC_TIMING_WRITE_LATENCY_0x424]);
    vIO32Write4B_All(DRAMC_REG_ADDR(DRAMC_REG_SELPH10), ACTable[AC_TIMING_WRITE_LATENCY_0x428]);
    vIO32Write4B_All(DRAMC_REG_ADDR(DRAMC_REG_SELPH11), ACTable[AC_TIMING_WRITE_LATENCY_0x42c]);

    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_DQSCTL1), ACTable[AC_TIMING_DQSINCTL_FOR_GATING], DQSCTL1_DQSINCTL);// Rank 0 DQSINCTL
    // AC timing table only need to apply to rank0, because of CSSwap for calibration will use only rank0
    //vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_DQSCTL2), ACTable[AC_TIMING_DQSINCTL_FOR_GATING], DQSCTL2_R1DQSINCTL); //Rank1 DQSINCTL, no use in A-PHY.
    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_MISC), ACTable[AC_TIMING_DATLAT], MISC_DATLAT);
    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_MISC), ACTable[AC_TIMING_DATLAT] - DATLAT_DSEL_DIFFERENCE, MISC_DATLAT_DSEL);  //DSEL = DATLAT -6

    return DRAM_OK;
}


DRAM_STATUS_T DdrUpdateACTiming(DRAMC_CTX_T *p)
{
    U8 u1TimingIdx=0xff, u1TmpIdx;

    for(u1TmpIdx=0; u1TmpIdx<TOTAL_AC_TIMING_NUMBER; u1TmpIdx++)
    {
        if((ACTimingTable[u1TmpIdx][0] == (U32)p->dram_type) &&
            (ACTimingTable[u1TmpIdx][1] == p->frequency))
        {
            u1TimingIdx = u1TmpIdx;
            mcSHOW_DBG_MSG(("[DdrUpdateACTiming] match AC timing %d\n", u1TimingIdx));
            mcFPRINTF((fp_A60501, "[DdrUpdateACTiming] match AC timing %d\n", u1TimingIdx));          
            break;
        }
    }

    if(u1TimingIdx == 0xff)
    {
        u1TimingIdx =0;
        mcSHOW_DBG_MSG(("[DdrUpdateACTiming] Error, no match AC timing, use default timing 0\n"));
        mcFPRINTF((fp_A60501, "[DdrUpdateACTiming] Error, no match AC timing, use default timing 0\n"));          
        //return DRAM_FAIL;
    }

    DdrUpdateACTimingReg(p, ACTimingTable[u1TimingIdx]);

    //DramcModeRegInit_Everest_LP3(p);

    return DRAM_OK;
}


DRAM_STATUS_T DdrUpdateACTiming_EMI(DRAMC_CTX_T *p, AC_TIMING_EXTERNAL_T *ACRegFromEmi)
{
    U8 u1TimingIdx=0xff, u1TmpIdx;
    U32 ACTiming[AC_TIMING_ITEM_NUM];

   if(ACRegFromEmi == NULL)
        return DRAM_FAIL;

    //Get AC timing from emi setting
    ACTiming[AC_TIMING_DRAM_TYPE] =  TYPE_LPDDR3;
    ACTiming[AC_TIMING_FREQUENCY] =  ACRegFromEmi->AC_TIME_EMI_FREQUENCY ;
    ACTiming[AC_TIMING_TRAS] =  ACRegFromEmi->AC_TIME_EMI_TRAS ;
    ACTiming[AC_TIMING_TRP] =  ACRegFromEmi->AC_TIME_EMI_TRP ;
    
    ACTiming[AC_TIMING_TRPAB] =  ACRegFromEmi->AC_TIME_EMI_TRPAB ;
    ACTiming[AC_TIMING_TRC] =  ACRegFromEmi->AC_TIME_EMI_TRC ;
    ACTiming[AC_TIMING_TRFC] =  ACRegFromEmi->AC_TIME_EMI_TRFC ;
    ACTiming[AC_TIMING_TRFCPB] =  ACRegFromEmi->AC_TIME_EMI_TRFCPB ;
    
    ACTiming[AC_TIMING_TXP] =  ACRegFromEmi->AC_TIME_EMI_TXP ;
    ACTiming[AC_TIMING_TRTP] =  ACRegFromEmi->AC_TIME_EMI_TRTP ;
    ACTiming[AC_TIMING_TRCD] =  ACRegFromEmi->AC_TIME_EMI_TRCD ;
    ACTiming[AC_TIMING_TWR] =  ACRegFromEmi->AC_TIME_EMI_TWR ;
    ACTiming[AC_TIMING_TWTR] =  ACRegFromEmi->AC_TIME_EMI_TWTR ;
    ACTiming[AC_TIMING_TRRD] =  ACRegFromEmi->AC_TIME_EMI_TRRD ;
    ACTiming[AC_TIMING_TFAW] =  ACRegFromEmi->AC_TIME_EMI_TFAW ;
    ACTiming[AC_TIMING_TRTW] =  ACRegFromEmi->AC_TIME_EMI_TRTW ;
    ACTiming[AC_TIMING_REFCNT] =  ACRegFromEmi->AC_TIME_EMI_REFCNT ; //(REFFRERUN = 0)
    ACTiming[AC_TIMING_REFCNT_FR_CLK] =  ACRegFromEmi->AC_TIME_EMI_REFCNT_FR_CLK ; //(REFFRERUN = 1)
    ACTiming[AC_TIMING_TXREFCNT] =  ACRegFromEmi->AC_TIME_EMI_TXREFCNT ;
    ACTiming[AC_TIMING_TZQCS] =  ACRegFromEmi->AC_TIME_EMI_TZQCS ;

    ACTiming[AC_TIMING_TRAS_05T] =  ACRegFromEmi->AC_TIME_EMI_TRAS_05T ;
    ACTiming[AC_TIMING_TRP_05T] =  ACRegFromEmi->AC_TIME_EMI_TRP_05T ;
    ACTiming[AC_TIMING_TRPAB_05T] =  ACRegFromEmi->AC_TIME_EMI_TRPAB_05T ;
    ACTiming[AC_TIMING_TRC_05T] =  ACRegFromEmi->AC_TIME_EMI_TRC_05T ;
    ACTiming[AC_TIMING_TRFC_05T] =  ACRegFromEmi->AC_TIME_EMI_TRFC_05T ;
    ACTiming[AC_TIMING_TRFCPB_05T] =  ACRegFromEmi->AC_TIME_EMI_TRFCPB_05T ;
    ACTiming[AC_TIMING_TXP_05T] =  ACRegFromEmi->AC_TIME_EMI_TXP_05T ;
    ACTiming[AC_TIMING_TRTP_05T] =  ACRegFromEmi->AC_TIME_EMI_TRTP_05T ;
    ACTiming[AC_TIMING_TRCD_05T] =  ACRegFromEmi->AC_TIME_EMI_TRCD_05T ;
    ACTiming[AC_TIMING_TWR_05T] =  ACRegFromEmi->AC_TIME_EMI_TWR_05T ;
    ACTiming[AC_TIMING_TWTR_05T] =  ACRegFromEmi->AC_TIME_EMI_TWTR_05T ;
    ACTiming[AC_TIMING_TRRD_05T] =  ACRegFromEmi->AC_TIME_EMI_TRRD_05T ;
    ACTiming[AC_TIMING_TFAW_05T] =  ACRegFromEmi->AC_TIME_EMI_TFAW_05T ;
    ACTiming[AC_TIMING_TRTW_05T] =  ACRegFromEmi->AC_TIME_EMI_TRTW_05T ;
    ACTiming[AC_TIMING_REFCNT_05T] =  ACRegFromEmi->AC_TIME_EMI_REFCNT_05T ; //(REFFRERUN = 0)
    ACTiming[AC_TIMING_REFCNT_FR_CLK_05T] =  ACRegFromEmi->AC_TIME_EMI_REFCNT_FR_CLK_05T ; //(REFFRERUN = 1)
    ACTiming[AC_TIMING_TXREFCNT_05T] =  ACRegFromEmi->AC_TIME_EMI_TXREFCNT_05T ;
    ACTiming[AC_TIMING_TZQCS_05T] =  ACRegFromEmi->AC_TIME_EMI_TZQCS_05T ;

    //Get AC timing from internal ACTimingTable
    for(u1TmpIdx=0; u1TmpIdx<TOTAL_AC_TIMING_NUMBER; u1TmpIdx++)
    {
        if((ACTimingTable[u1TmpIdx][0] == (U32)p->dram_type) &&
            (ACTimingTable[u1TmpIdx][1] == p->frequency))
        {
            u1TimingIdx = u1TmpIdx;
            mcSHOW_DBG_MSG(("[DdrUpdateACTiming_EMI] match AC timing %d\n", u1TimingIdx));
            mcFPRINTF((fp_A60501, "[DdrUpdateACTiming_EMI] match AC timing %d\n", u1TimingIdx));          
            break;
        }
    }

    if(u1TimingIdx == 0xff)
    {
        u1TimingIdx =0;
        mcSHOW_DBG_MSG(("[DdrUpdateACTiming_EMI] Error, no match AC timing, use default timing 0\n"));
        mcFPRINTF((fp_A60501, "[DdrUpdateACTiming_EMI] Error, no match AC timing, use default timing 0\n"));          
        //return DRAM_FAIL;
    }
    
    ACTiming[AC_TIMING_XRTW2W] =  ACTimingTable[u1TimingIdx][AC_TIMING_XRTW2W];
    ACTiming[AC_TIMING_XRTW2R] =  ACTimingTable[u1TimingIdx][AC_TIMING_XRTW2R];
    ACTiming[AC_TIMING_XRTR2W] =  ACTimingTable[u1TimingIdx][AC_TIMING_XRTR2W];
    ACTiming[AC_TIMING_XRTR2R] =  ACTimingTable[u1TimingIdx][AC_TIMING_XRTR2R];

    ACTiming[AC_TIMING_WRITE_LATENCY_0x41c] =  ACTimingTable[u1TimingIdx][AC_TIMING_WRITE_LATENCY_0x41c];
    ACTiming[AC_TIMING_WRITE_LATENCY_0x420] =  ACTimingTable[u1TimingIdx][AC_TIMING_WRITE_LATENCY_0x420];
    ACTiming[AC_TIMING_WRITE_LATENCY_0x424] =  ACTimingTable[u1TimingIdx][AC_TIMING_WRITE_LATENCY_0x424];
    ACTiming[AC_TIMING_WRITE_LATENCY_0x428] =  ACTimingTable[u1TimingIdx][AC_TIMING_WRITE_LATENCY_0x428];
    ACTiming[AC_TIMING_WRITE_LATENCY_0x42c] =  ACTimingTable[u1TimingIdx][AC_TIMING_WRITE_LATENCY_0x42c];

    ACTiming[AC_TIMING_DQSINCTL_FOR_GATING] =  ACTimingTable[u1TimingIdx][AC_TIMING_DQSINCTL_FOR_GATING];
    ACTiming[AC_TIMING_DATLAT] =  ACTimingTable[u1TimingIdx][AC_TIMING_DATLAT];
    ACTiming[AC_TIMING_MODE_REG_WL] =  ACTimingTable[u1TimingIdx][AC_TIMING_MODE_REG_WL];
    ACTiming[AC_TIMING_MODE_REG_RL] =  ACTimingTable[u1TimingIdx][AC_TIMING_MODE_REG_RL];    

    DdrUpdateACTimingReg(p, ACTiming);
    
    return DRAM_OK;
}

DRAM_STATUS_T DramcInit(DRAMC_CTX_T *p)
{        
    U32 u4RefreshRate;	
    EMI_SETTINGS *emi_set;
    int highest_freq;

#if DFS_COMBINATION_TYPE1
    highest_freq = 933; //1866
#elif DFS_COMBINATION_TYPE3
    highest_freq = 850; //1700
#else
    highest_freq = 800; //1600
#endif

    if(get_chip_id_by_efuse() == CHIP_MT6797M)
        highest_freq = 800; //1600

    mcSHOW_DBG_MSG(("[DramcInit] ====Begin====\n"));
    mcFPRINTF((fp_A60501, "[DramcInit] ====Begin====\n"));    

    if (p->dram_type == TYPE_LPDDR4)
    {
        //DramcSetting_LP4(p);
    }
    else // LPDDR3
    {
        DramcSetting_Everest_LP3(p);
#if !__ETT__
        if(p->frequency == highest_freq)
        {    
            if(emi_setting_index == -1)
                emi_set = default_emi_setting; 
            else
                emi_set = &emi_settings[emi_setting_index];
            
            if(emi_set->AcTimeEMI.AC_TIME_EMI_FREQUENCY == p->frequency)
            {
            	DdrUpdateACTiming_EMI(p, &(emi_set->AcTimeEMI));
            }
           	else
            {
            	mcSHOW_DBG_MSG(("[DdrUpdateACTiming_EMI] frequency not match (MDL=%d, target=%d), use inner table setting\n", emi_set->AcTimeEMI.AC_TIME_EMI_FREQUENCY, p->frequency));
           		DdrUpdateACTiming(p);
            }
        }
        else
#endif          
        {
            DdrUpdateACTiming(p);  
        }
    }

    // for free-run clk 26MHz, 0x62 * (1/26) = 3.8ns
    vIO32WriteFldAlign_All(DRAMC_REG_DRAMC_PD_CTRL, 0x62, DRAMC_PD_CTRL_REFCNT_FR_CLK);  
    // for non-fre-run clk,  reg = 3.8 ns * f / 4 / 16;
    u4RefreshRate = 38*p->frequency/640;
    vIO32WriteFldAlign_All(DRAMC_REG_CONF2, u4RefreshRate, CONF2_REFCNT);   

    mcSHOW_DBG_MSG(("[DramcInit] ====Done====\n"));
    mcFPRINTF((fp_A60501, "[DramcInit] ====Done====\n"));    

    #if LJPLL_FREQ_DEBUG_LOG
    #if CHECK_PLL_OK
        if(is_pll_good() == 0) //not all pll is good, use fix band
        {
          if(p->freq_sel<LJ_MAX_SEL)
              mcDELAY_MS(1000);  
        }        
    #endif
    {
        U8 confIdx = 0;
        U8 u1Pass = 0;
        U8 u1Fail = 0;
        U16 u2Band = 0x0;
        for(confIdx=0; confIdx<3; confIdx++)
        {
            u1Pass = u4IO32ReadFldAlign(DDRPHY_PHY_RO_3+(confIdx<<POS_BANK_NUM), PHY_RO_3_RGS_RPHYPLL_AUTOK_PASS);
            u1Fail = u4IO32ReadFldAlign(DDRPHY_PHY_RO_2+(confIdx<<POS_BANK_NUM), PHY_RO_2_RGS_RPHYPLL_AUTOK_FAIL);
            u2Band = u4IO32ReadFldAlign(DDRPHY_PHY_RO_3+(confIdx<<POS_BANK_NUM), PHY_RO_3_RGS_RPHYPLL_AUTOK_BAND);
            mcSHOW_DBG_MSG(("[LJ_PHYPLL_%d], PASS=%d, FAIL=%d, BAND=%B\n", confIdx, u1Pass, u1Fail, u2Band));
        }
    }
    #endif
    //while(1);
    return DRAM_OK;
}


//-------------------------------------------------------------------------
/** DdrPhyInit
 *  DDRPHY Initialization.
 *  @param p                Pointer of context created by DramcCtxCreate.
 *  @retval status          (DRAM_STATUS_T): DRAM_OK or DRAM_FAIL 
 */
//-------------------------------------------------------------------------
DRAM_STATUS_T DdrPhyInit(DRAMC_CTX_T *p)
{
    mcSHOW_DBG_MSG(("[DdrPhyInit] ====Begin: Freq=%d ====\n",p->frequency));
    mcFPRINTF((fp_A60501, "[DdrPhyInit] ====Begin: Freq=%d ====\n",p->frequency));    
    
    PLL_FBKSEL_0 =0;
    
    if (p->frequency == 3200) // LPDDR4-4266
    {
        //PLL_PREDIV_1_0 = (0x00000000<<22);        
        PLL_FBKDIV_6_0 = (0x0000001e<<0);
        //PLL_FBKDIV_7 = (0x00000000<<10); // RESERVE_0<2>
        PLL_FBKSEL_0 = (0x00000001<<0); // RESERVE_1<0>
        PLL_IC_3_0 = (0x00000008<<20);
        PLL_BR_3_0 = (0x00000003<<16);
        PLL_BC_3_0 = (0x00000007<<24);
        PLL_BC_4 = (0x00000001<<8); // RESERVE_0<0>
        PLL_BP_3_0 = (0x0000000f<<8);
        PLL_BP_4 = (0x00000001<<9); // RESERVE_0<1>
        PLL_BAND_6_0 = (0x0000003d<<0); 
        //PLL_BP2_1_0 = (0x00000003<<12);
        // PLL_POSDIV_1_0 = (0x00000000<<20);
    }
    else if (p->frequency == 2400) // LPDDR4-4266
    {
        //PLL_PREDIV_1_0 = (0x00000000<<22);        
        PLL_FBKDIV_6_0 = (0x0000002e<<0);
        //PLL_FBKDIV_7 = (0x00000000<<10); // RESERVE_0<2>
        //PLL_FBKSEL_0 = (0x00000000<<0); // RESERVE_1<0>
        PLL_IC_3_0 = (0x00000006<<20);
        PLL_BR_3_0 = (0x00000003<<16);
        PLL_BC_3_0 = (0x00000007<<24);
        PLL_BC_4 = (0x00000001<<8); // RESERVE_0<0>
        PLL_BP_3_0 = (0x0000000f<<8);
        PLL_BP_4 = (0x00000001<<9); // RESERVE_0<1>
        PLL_BAND_6_0 = (0x00000031<<0); 
        //PLL_BP2_1_0 = (0x00000003<<12);
        // PLL_POSDIV_1_0 = (0x00000000<<20);
    }
    else
    if (p->frequency == 2133) // LPDDR4-4266
    {
        //PLL_PREDIV_1_0 = (0x00000000<<22);        
        PLL_FBKDIV_6_0 = (0x00000029<<0);
        //PLL_FBKDIV_7 = (0x00000000<<10); // RESERVE_0<2>
        //PLL_FBKSEL_0 = (0x00000000<<0); // RESERVE_1<0>
        PLL_IC_3_0 = (0x00000005<<20);
        PLL_BR_3_0 = (0x00000003<<16);
        PLL_BC_3_0 = (0x00000007<<24);
        PLL_BC_4 = (0x00000001<<8); // RESERVE_0<0>
        PLL_BP_3_0 = (0x0000000f<<8);
        PLL_BP_4 = (0x00000001<<9); // RESERVE_0<1>
        PLL_BAND_6_0 = (0x0000002d<<0); 
        //PLL_BP2_1_0 = (0x00000003<<12);
        // PLL_POSDIV_1_0 = (0x00000000<<20);
    }
    else if (p->frequency == 2000) // LPDDR4-3200
    {
        //PLL_PREDIV_1_0 = (0x00000000<<22);        
        PLL_FBKDIV_6_0 = (0x00000026<<0);
        //PLL_FBKDIV_7 = (0x00000000<<10); // RESERVE_0<2>
        //PLL_FBKSEL_0 = (0x00000000<<0); // RESERVE_1<0>
        PLL_IC_3_0 = (0x00000005<<20);
        PLL_BR_3_0 = (0x00000003<<16);
        PLL_BC_3_0 = (0x00000007<<24);
        PLL_BC_4 = (0x00000001<<8); // RESERVE_0<0>
        PLL_BP_3_0 = (0x0000000f<<8);
        PLL_BP_4 = (0x00000001<<9); // RESERVE_0<1>
        PLL_BAND_6_0 = (0x00000025<<0);
        //PLL_BP2_1_0 = (0x00000003<<12);
        // PLL_POSDIV_1_0 = (0x00000000<<20);
    }
    else if (p->frequency == 1866) // LPDDR4-3200
    {
        //PLL_PREDIV_1_0 = (0x00000000<<22);        
        PLL_FBKDIV_6_0 = (0x00000023<<0);
        //PLL_FBKDIV_7 = (0x00000000<<10); // RESERVE_0<2>
        //PLL_FBKSEL_0 = (0x00000000<<0); // RESERVE_1<0>
        PLL_IC_3_0 = (0x00000004<<20);
        PLL_BR_3_0 = (0x00000003<<16);
        PLL_BC_3_0 = (0x00000007<<24);
        PLL_BC_4 = (0x00000001<<8); // RESERVE_0<0>
        PLL_BP_3_0 = (0x0000000f<<8);
        PLL_BP_4 = (0x00000001<<9); // RESERVE_0<1>
        PLL_BAND_6_0 = (0x00000025<<0);
        //PLL_BP2_1_0 = (0x00000003<<12);
        // PLL_POSDIV_1_0 = (0x00000000<<20);
    }
    else if (p->frequency == 1600) // LPDDR4-3200
    {
        //PLL_PREDIV_1_0 = (0x00000000<<22);        
        PLL_FBKDIV_6_0 = (0x0000001e<<0);
        //PLL_FBKDIV_7 = (0x00000000<<10); // RESERVE_0<2>
        //PLL_FBKSEL_0 = (0x00000000<<0); // RESERVE_1<0>
        PLL_IC_3_0 = (0x00000004<<20);
        PLL_BR_3_0 = (0x00000003<<16);
        PLL_BC_3_0 = (0x00000007<<24);
        PLL_BC_4 = (0x00000001<<8); // RESERVE_0<0>
        PLL_BP_3_0 = (0x0000000f<<8);
        PLL_BP_4 = (0x00000001<<9); // RESERVE_0<1>
        PLL_BAND_6_0 = (0x00000016<<0);
        //PLL_BP2_1_0 = (0x00000003<<12);
        // PLL_POSDIV_1_0 = (0x00000000<<20);
    }
    else if (p->frequency == 1066) // LPDDR4-2133
    {
        //PLL_PREDIV_1_0 = (0x00000000<<22);        
        PLL_FBKDIV_6_0 = (0x00000014<<0);
        //PLL_FBKDIV_7 = (0x00000000<<10); // RESERVE_0<2>
        //PLL_FBKSEL_0 = (0x00000000<<0); // RESERVE_1<0>
        PLL_IC_3_0 = (0x00000002<<20);
        PLL_BR_3_0 = (0x00000003<<16);
        PLL_BC_3_0 = (0x00000007<<24);
        PLL_BC_4 = (0x00000001<<8); // RESERVE_0<0>
        PLL_BP_3_0 = (0x0000000f<<8);
        PLL_BP_4 = (0x00000001<<9); // RESERVE_0<1>
        PLL_BAND_6_0 = (0x0000000f<<0) ;
        //PLL_BP2_1_0 = (0x00000003<<12);
        // PLL_POSDIV_1_0 = (0x00000000<<20);
        mcSHOW_DBG_MSG(("[DdrPhyInit] ====1066====\n"));
        mcFPRINTF((fp_A60501, "[DdrPhyInit] ====1066====\n"));    

    }
    else if (p->frequency == 1200) // LPDDR4-2400
    {
        //PLL_PREDIV_1_0 = (0x00000000<<22);        
        PLL_FBKDIV_6_0 = (0x00000017<<0);
        //PLL_FBKDIV_7 = (0x00000000<<10); // RESERVE_0<2>
        //PLL_FBKSEL_0 = (0x00000000<<0); // RESERVE_1<0>
        PLL_IC_3_0 = (0x00000002<<20);
        PLL_BR_3_0 = (0x00000003<<16);
        PLL_BC_3_0 = (0x00000006<<24);
        PLL_BC_4 = (0x00000001<<8); // RESERVE_0<0>
        PLL_BP_3_0 = (0x0000000c<<8);
        PLL_BP_4 = (0x00000001<<9); // RESERVE_0<1>
        PLL_BAND_6_0 = (0x0000000f<<0) ;
        //PLL_BP2_1_0 = (0x00000003<<12);
        // PLL_POSDIV_1_0 = (0x00000000<<20);
    }
    else if (p->frequency == 933) // LPDDR4-1866
    {
        //PLL_PREDIV_1_0 = (0x00000000<<22);        
        PLL_FBKDIV_6_0 = (0x00000011<<0);
        //PLL_FBKDIV_7 = (0x00000000<<10); // RESERVE_0<2>
        //PLL_FBKSEL_0 = (0x00000000<<0); // RESERVE_1<0>
        PLL_IC_3_0 = (0x00000001<<20);
        PLL_BR_3_0 = (0x00000003<<16);
        PLL_BC_3_0 = (0x00000006<<24);
        PLL_BC_4 = (0x00000001<<8); // RESERVE_0<0>
        PLL_BP_3_0 = (0x0000000c<<8);
        PLL_BP_4 = (0x00000001<<9); // RESERVE_0<1>
        PLL_BAND_6_0 = (0x0000000b<<0) ;
        //PLL_BP2_1_0 = (0x00000003<<12);
        // PLL_POSDIV_1_0 = (0x00000000<<20);
    }
    else if (p->frequency == 800) // LPDDR4-1600
    {
        //PLL_PREDIV_1_0 = (0x00000000<<22);        
        PLL_FBKDIV_6_0 = (0x0000000f<<0);
        //PLL_FBKDIV_7 = (0x00000000<<10); // RESERVE_0<2>
        //PLL_FBKSEL_0 = (0x00000000<<0); // RESERVE_1<0>
        PLL_IC_3_0 = (0x00000001<<20);
        PLL_BR_3_0 = (0x00000003<<16);
        PLL_BC_3_0 = (0x00000006<<24);
        PLL_BC_4 = (0x00000001<<8); // RESERVE_0<0>
        PLL_BP_3_0 = (0x0000000c<<8);
        PLL_BP_4 = (0x00000001<<9); // RESERVE_0<1>
        PLL_BAND_6_0 = (0x0000000a<<0) ;
        //PLL_BP2_1_0 = (0x00000003<<12);
        // PLL_POSDIV_1_0 = (0x00000000<<20);
    }
    else if (p->frequency == 400) // LPDDR4-800
    {
        //PLL_PREDIV_1_0 = (0x00000000<<22);        
        PLL_FBKDIV_6_0 = (0x00000007<<0);
        //PLL_FBKDIV_7 = (0x00000000<<10); // RESERVE_0<2>
        //PLL_FBKSEL_0 = (0x00000000<<0); // RESERVE_1<0>
        PLL_IC_3_0 = (0x00000001<<20);
        //PLL_BR_3_0 = (0x00000010<<16);
        PLL_BR_3_0 = (0x0000003<<16);
        PLL_BC_3_0 = (0x0000000f<<24);
        PLL_BC_4 = (0x00000001<<8); // RESERVE_0<0>
        PLL_BP_3_0 = (0x0000000f<<8);
        PLL_BP_4 = (0x00000001<<9); // RESERVE_0<1>
        PLL_BAND_6_0 = (0x00000003<<0) ;
        //PLL_BP2_1_0 = (0x00000003<<12);
        // PLL_POSDIV_1_0 = (0x00000000<<20);
    }
    else // LPDDR4-3200 (default)
    {
        //PLL_PREDIV_1_0 = (0x00000000<<22);        
        PLL_FBKDIV_6_0 = (0x0000001e<<0);
        //PLL_FBKDIV_7 = (0x00000000<<10); // RESERVE_0<2>
        //PLL_FBKSEL_0 = (0x00000000<<0); // RESERVE_1<0>
        PLL_IC_3_0 = (0x00000004<<20);
        PLL_BR_3_0 = (0x00000003<<16);
        PLL_BC_3_0 = (0x00000007<<24);
        PLL_BC_4 = (0x00000001<<8); // RESERVE_0<0>
        PLL_BP_3_0 = (0x0000000f<<8);
        PLL_BP_4 = (0x00000001<<9); // RESERVE_0<1>
        PLL_BAND_6_0 = (0x00000016<<0) ;
        //PLL_BP2_1_0 = (0x00000003<<12);
        // PLL_POSDIV_1_0 = (0x00000000<<20);
    }

    if (p->dram_type == TYPE_LPDDR4)
    {
        //DdrPhySetting_LP4(p);
    }
    else // LPDDR3-PoP
    {
        DdrPhySetting_Everest_LP3(p);
    }

    mcSHOW_DBG_MSG(("[DdrPhyInit] ====Done====\n"));
    mcFPRINTF((fp_A60501, "[DdrPhyInit] ====Done====\n"));    

    return DRAM_OK;
}

#if 0
DRAM_STATUS_T ApplySSCSettingReg(U8 u1Percent, U8 u1Slope, U8 u1Dir)
{
    U32 u4PRD, u4Delta, u4Delta1;

    u4PRD = 13000/u1Slope; 
    u4Delta = ((76 << 18) *u1Percent)/ (u4PRD *100);
    u4Delta1 = u4Delta;

    #if EVEREST_PORTING_TODO
    vIO32WriteFldAlign(SYS_REG_ADDR(PLLGP_RG_02), u4PRD, RG_02_MEMPLL_SDM_SSC_PRD);
    vIO32WriteFldAlign(SYS_REG_ADDR(PLLGP_RG_03), u4Delta1, RG_03_MEMPLL_SDM_SSC_DELTA1);
    vIO32WriteFldAlign(SYS_REG_ADDR(PLLGP_RG_04), u4Delta, RG_04_MEMPLL_SDM_SSC_DELTA);
#endif
    mcSHOW_DBG_MSG2(("[ApplySSCSettingReg] Percent %d, Slope %d kHz, Dir %d (Down)\n", \
    	u1Percent, u1Slope, u1Dir));

    mcSHOW_DBG_MSG2(("[ApplySSCSettingReg] u4PRD =(0x%x) , u4Delta (0x%x),  u4Delta1 (0x%x)\n", u4PRD, u4Delta, u4Delta1));
    return DRAM_OK;
}


DRAM_STATUS_T SscEnable(DRAMC_CTX_T *p)
{
    if (p->ssc_en == ENABLE)
    {
        mcSHOW_DBG_MSG(("Enable SSC...\n"));
        mcFPRINTF((fp_A60501, "Enable SSC...\n"));
        // RG_SYSPLL_SDM_SSC_EN = 1 (0x14[26])
        
#if EVEREST_PORTING_TODO
        vIO32WriteFldAlign(SYS_REG_ADDR(PLLGP_RG_11), 1, RG_11_SYSPLL_SDM_SSC_EN);
#endif
    }
    return DRAM_OK;
}
#endif

#if 0
void DramcEnterSelfRefresh(DRAMC_CTX_T *p, U8 op)
{
    U8 ucstatus = 0;
    U32 uiTemp;
    U32 u4TimeCnt;

    u4TimeCnt = TIME_OUT_CNT;

    mcSHOW_DBG_MSG(("[DramcEnterSelfRefresh]  op:%d (0:exit, 1:enter)\n", op));
    mcFPRINTF((fp_A60501, "[DramcEnterSelfRefresh] op:%d(0:exit, 1:enter)\n", op));    

    if (op == 1) // enter self refresh
    {
        if(p->dram_type == TYPE_LPDDR4)
        {
            // ONLY work for LP4, not LP3
            // MISCA_SRFPD_DIS =1, self-refresh
            // MISCA_SRFPD_DIS =0, self-refresh power down
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_MISCA), 1, MISCA_SRFPD_DIS);  
        }
        
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_CONF1), 1, CONF1_SELFREF);
        mcDELAY_US(2);
        uiTemp = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMDRESP), SPCMDRESP_SREF_STATE);
        while((uiTemp==0) &&(u4TimeCnt>0))
        {
            mcSHOW_DBG_MSG2(("Still not enter self refresh(%d)\n",u4TimeCnt));
            mcDELAY_US(1);
            uiTemp = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMDRESP), SPCMDRESP_SREF_STATE);
            u4TimeCnt --;
        }
    }
    else // exit self refresh
    {
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_CONF1), 0, CONF1_SELFREF);

        mcDELAY_US(2);
        uiTemp = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMDRESP), SPCMDRESP_SREF_STATE);
        while ((uiTemp!=0) &&(u4TimeCnt>0))
        {
            mcSHOW_DBG_MSG2(("Still not exit self refresh(%d)\n", u4TimeCnt));
            mcDELAY_US(1);
            uiTemp = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMDRESP), SPCMDRESP_SREF_STATE);
            u4TimeCnt--;
        }
    }    

   if(u4TimeCnt ==0)
   {
        mcSHOW_DBG_MSG(("[DramcEnterSelfRefresh]  Self Rresh operation fail\n"));
        mcFPRINTF((fp_A60501, "[DramcEnterSelfRefresh] Self Rresh operation fail\n"));    
   }
   else
   {
        mcSHOW_DBG_MSG(("[DramcEnterSelfRefresh]  Self Rresh operation done\n"));
        mcFPRINTF((fp_A60501, "[DramcEnterSelfRefresh] Self Rresh operation done\n"));    
   }
}
#endif


void DramcLowFreqWrite(DRAMC_CTX_T *p)
{
#if 0
    U8 ucstatus = 0;
    U16 u2freq_orig;
    U32 u4err_value;

    if (p->fglow_freq_write_en == ENABLE)
    {        
        u2freq_orig = p->frequency;
        p->frequency = p->frequency_low;
        mcSHOW_DBG_MSG(("Enable low speed write function...\n"));
        mcFPRINTF((fp_A60501, "Enable low speed write function...\n"));
        // we will write data in memory on a low frequency,to make sure the data we write is  right
        // then use engine2 read to do the calibration
        // so ,we will do :
        // 1.change freq 
        // 2. use self test engine2 write to write data ,and check the data is right or not
        // 3.change freq to original value        

        // 1. change freq         
        DramcEnterSelfRefresh(p, 1); // enter self refresh
        mcDELAY_US(1);
        DdrPhyInit(p);
        mcDELAY_US(1);
        DramcEnterSelfRefresh(p, 0); // exit self refresh

        // Need to do phase sync after change frequency
        //DramcDiv2PhaseSync(p);
        
        // 2. use self test engine2 to write data (only support AUDIO or XTALK pattern)
        if (p->test_pattern== TEST_AUDIO_PATTERN)
        {
            u4err_value = DramcEngine2(p, TE_OP_WRITE_READ_CHECK, p->test2_1, p->test2_2, 1, 0, 0, 0);            
        }
        else if (p->test_pattern== TEST_XTALK_PATTERN)
        {
            u4err_value = DramcEngine2(p, TE_OP_WRITE_READ_CHECK, p->test2_1, p->test2_2, 2, 0, 0, 0);
        }
        else
        {
            mcSHOW_ERR_MSG(("ERROR! Only support AUDIO or XTALK in Low Speed Write and High Speed Read calibration!! \n"));
            u4err_value = 0xffffffff;
        }

        // May error due to gating not calibrate @ low speed
        mcSHOW_DBG_MSG(("Low speed write error value: 0x%8x\n", u4err_value));
        mcFPRINTF((fp_A60501, "Low speed write error value: 0x%8x\n", u4err_value));

        // do phy reset due to ring counter may be wrong
        DramcPhyReset(p);

        // 3. change to original freq 
        p->frequency = u2freq_orig;
        DramcEnterSelfRefresh(p, 1); // enter self refresh
        mcDELAY_US(1);
        DdrPhyInit(p);
        mcDELAY_US(1);
        DramcEnterSelfRefresh(p, 0); // exit self refresh

        // Need to do phase sync after change frequency
        //DramcDiv2PhaseSync(p);
    }    
#endif
}


void DramcRunTimeConfig(DRAMC_CTX_T *p)
{
    mcSHOW_DBG_MSG(("=== [DramcRunTimeConfig] ===\n"));

#ifdef HW_GATING
        DramcHWGatingInit(p);                                   // HW gating initial before RunTime config.
        DramcHWGatingOnOff((DRAMC_CTX_T *) p, 1); // Enable HW gating tracking
        mcSHOW_DBG_MSG(("HW_GATING: ON\n"));
#else
        DramcHWGatingOnOff((DRAMC_CTX_T *) p, 0);
        mcSHOW_DBG_MSG(("HW_GATING: OFF\n"));
#endif
    	
//HW sync gating tracking
#if DFS_HW_SYNC_GATING_TRACKING
    vIO32WriteFldAlign_All(DRAMC_REG_PADCTL7, 0, PADCTL7_DVFS_SYNC_MASK);
    mcSHOW_DBG_MSG(("DFS_HW_SYNC_GATING_TRACKING: ON\n"));
#else        
    mcSHOW_DBG_MSG(("DFS_HW_SYNC_GATING_TRACKING: OFF\n"));
#endif

#ifdef ZQCS_ENABLE
    vIO32WriteFldAlign_All(DRAMC_REG_SPCMD, 0xff, SPCMD_ZQCSCNT);
    vIO32WriteFldAlign_All(DRAMC_REG_DRAMC_PD_CTRL, 1, DRAMC_PD_CTRL_ZQCSCNT8);
    vIO32WriteFldAlign_All(DRAMC_REG_SPCMD, 1, SPCMD_ZQCSDISB);  // LP3 and LP4 are different, be careful.
    mcSHOW_DBG_MSG(("ZQCS_ENABLE: ON\n"));
#else
    vIO32WriteFldAlign_All(DRAMC_REG_SPCMD, 0, SPCMD_ZQCSDISB);// LP3 and LP4 are different, be careful.
    mcSHOW_DBG_MSG(("ZQCS_ENABLE: OFF\n"));
#endif

#if APPLY_LOWPOWER_GOLDEN_SETTINGS
    DDRPhyLowPowerEnable();
    DDRAllLowPowerSettings((DRAMC_CTX_T *) p);
    mcSHOW_DBG_MSG(("LOWPOWER_GOLDEN_SETTINGS(DCM): ON\n"));
#else
    mcSHOW_DBG_MSG(("LOWPOWER_GOLDEN_SETTINGS(DCM): OFF\n"));
#endif
    
#ifdef SPM_CONTROL_AFTERK
      TransferToSPMControl(p);  //don't enable in ETT
      mcSHOW_DBG_MSG(("SPM_CONTROL_AFTERK: ON\n"));
  #else
      mcSHOW_DBG_MSG(("SPM_CONTROL_AFTERK: OFF\n"));
#endif
    
#if APPLY_LOWPOWER_GOLDEN_SETTINGS
    EnableDramcPhyDCM(p, 1);
#endif
    
    // 0x1c0[31]
    //vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DQSCAL0), 0, DQSCAL0_STBCALEN);

#ifdef TEMP_SENSOR_ENABLE
    // enable MR4 refresh rate reference, interval = 0x10
    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_ACTIM1), 0x10, ACTIM1_REFRCNT);
    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_ACTIM1), 0, ACTIM1_REFRDIS);
    mcSHOW_DBG_MSG(("TEMP_SENSOR_ENABLE: ON\n"));
#else
    mcSHOW_DBG_MSG(("TEMP_SENSOR_ENABLE: OFF\n"));
#endif

    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_RKCFG), 1, RKCFG_PBREFEN);

mcSHOW_DBG_MSG(("=========================\n"));

}


#if GATING_ONLY_FOR_DEBUG
void DramcGatingDebugInit(DRAMC_CTX_T *p)
{
    DramPhyReset(p);

    //enable &reset DQS counter
    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 1, SPCMD_DQSGCNTEN);
    
    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 1, SPCMD_DQSGCNTRST);
    mcDELAY_US(1);//delay 2T
    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 0, SPCMD_DQSGCNTRST);
    //mcSHOW_DBG_MSG(("DramcGatingDebugInit done\n" ));
}

void DramcGatingDebugExit(DRAMC_CTX_T *p)
{
    //enable &reset DQS counter
    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 0, SPCMD_DQSGCNTEN);
    vIO32WriteFldAlign_All(DDRPHY_PINMUX, 0, PINMUX_R_DMSTBENCMP_RK_OPT); 
}

void DramcGatingDebugRankSel(DRAMC_CTX_T *p, U8 u1Rank)
{
    #ifdef DUAL_RANKS
    if (uiDualRank==0) 
    #endif
    {
        u1Rank =0;
    }

    vIO32WriteFldAlign_All(DDRPHY_PINMUX, u1Rank, PINMUX_R_DMSTBENCMP_RK_OPT); 
}

void DramcGatingDebug(DRAMC_CTX_T *p, U8 u1Channel)
{
    U32 LP3_DataPerByte[DQS_NUMBER];
    U32 u4DebugCnt[DQS_NUMBER];
    U16 u2DebugCntPerByte;

    U32 u4value, u4all_result_R, u4all_result_F, u4err_value;

    p->channel = u1Channel;
    vSetPHY2ChannelMapping(p, u1Channel);
    
    mcDELAY_MS(10);
    
    LP3_DataPerByte[0] = (u4IO32ReadFldAlign(DDRPHY_STBERR_RK0_R+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), STBERR_RK0_R_STBERR_RK0_R));//PHY_B
    LP3_DataPerByte[2] = (LP3_DataPerByte[0] >>8) & 0xff;
    LP3_DataPerByte[0] &= 0xff;
    
    LP3_DataPerByte[1] = (u4IO32ReadFldAlign(DDRPHY_STBERR_RK0_R+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), STBERR_RK0_R_STBERR_RK0_R)) ;//PHY_C
    LP3_DataPerByte[3] = (LP3_DataPerByte[1] >>8) & 0xff;
    LP3_DataPerByte[1] &= 0xff;
    u4all_result_R = LP3_DataPerByte[0] | (LP3_DataPerByte[1] <<8) |(LP3_DataPerByte[2] <<16) |(LP3_DataPerByte[3] <<24);
    
    // falling
    LP3_DataPerByte[0] = (u4IO32ReadFldAlign(DDRPHY_STBERR_RK0_F+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), STBERR_RK0_F_STBERR_RK0_F));//PHY_B
    LP3_DataPerByte[2] = (LP3_DataPerByte[0] >>8) & 0xff;
    LP3_DataPerByte[0] &= 0xff;
    
    LP3_DataPerByte[1] = (u4IO32ReadFldAlign(DDRPHY_STBERR_RK0_F+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), STBERR_RK0_F_STBERR_RK0_F)) ;//PHY_C
    LP3_DataPerByte[3] = (LP3_DataPerByte[1] >>8) & 0xff;
    LP3_DataPerByte[1] &= 0xff;
    u4all_result_F = LP3_DataPerByte[0] | (LP3_DataPerByte[1] <<8) |(LP3_DataPerByte[2] <<16) |(LP3_DataPerByte[3] <<24);

     //read DQS counter
    u4DebugCnt[0] = u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_DQSGNWCNT0));
    u4DebugCnt[1] = (u4DebugCnt[0] >> 16) & 0xffff;
    u4DebugCnt[0] &= 0xffff;
        
    u4DebugCnt[2] = u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_DQSGNWCNT1));
    u4DebugCnt[3] = (u4DebugCnt[2] >> 16) & 0xffff;
    u4DebugCnt[2] &= 0xffff;

    mcSHOW_DBG_MSG(("\n[DramcGatingDebug] Channel %d , DQS count (B3->B0) 0x%H, 0x%H, 0x%H, 0x%H \nError flag Rank0 (B3->B0) %B %B  %B %B  %B %B  %B %B\n", \
                                   u1Channel, u4DebugCnt[3], u4DebugCnt[2], u4DebugCnt[1], u4DebugCnt[0], \
                                   (u4all_result_F>>24)&0xff, (u4all_result_R>>24)&0xff, \
                                   (u4all_result_F>>16)&0xff, (u4all_result_R>>16)&0xff, \
                                   (u4all_result_F>>8)&0xff,   (u4all_result_R>>8)&0xff, \
                                   (u4all_result_F)&0xff,         (u4all_result_R)&0xff));           

    #ifdef DUAL_RANKS  
    if(uiDualRank)
    {
        LP3_DataPerByte[0] = (u4IO32ReadFldAlign(DDRPHY_STBERR_RK1_R+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), STBERR_RK1_R_STBERR_RK1_R));//PHY_B
        LP3_DataPerByte[2] = (LP3_DataPerByte[0] >>8) & 0xff;
        LP3_DataPerByte[0] &= 0xff;
        
        LP3_DataPerByte[1] = (u4IO32ReadFldAlign(DDRPHY_STBERR_RK1_R+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), STBERR_RK1_R_STBERR_RK1_R)) ;//PHY_C
        LP3_DataPerByte[3] = (LP3_DataPerByte[1] >>8) & 0xff;
        LP3_DataPerByte[1] &= 0xff;
        u4all_result_R = LP3_DataPerByte[0] | (LP3_DataPerByte[1] <<8) |(LP3_DataPerByte[2] <<16) |(LP3_DataPerByte[3] <<24);
        
        // falling
        LP3_DataPerByte[0] = (u4IO32ReadFldAlign(DDRPHY_STBERR_RK1_F+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), STBERR_RK1_F_STBERR_RK1_F));//PHY_B
        LP3_DataPerByte[2] = (LP3_DataPerByte[0] >>8) & 0xff;
        LP3_DataPerByte[0] &= 0xff;
        
        LP3_DataPerByte[1] = (u4IO32ReadFldAlign(DDRPHY_STBERR_RK1_F+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), STBERR_RK1_F_STBERR_RK1_F)) ;//PHY_C
        LP3_DataPerByte[3] = (LP3_DataPerByte[1] >>8) & 0xff;
        LP3_DataPerByte[1] &= 0xff;
        u4all_result_F = LP3_DataPerByte[0] | (LP3_DataPerByte[1] <<8) |(LP3_DataPerByte[2] <<16) |(LP3_DataPerByte[3] <<24);
        
        mcSHOW_DBG_MSG(("Error flag Rank1 (B3->B0) %B %B  %B %B  %B %B  %B %B\n", \
                                       (u4all_result_F>>24)&0xff, (u4all_result_R>>24)&0xff, \
                                       (u4all_result_F>>16)&0xff, (u4all_result_R>>16)&0xff, \
                                       (u4all_result_F>>8)&0xff,   (u4all_result_R>>8)&0xff, \
                                       (u4all_result_F)&0xff,         (u4all_result_R)&0xff));     
    }
    #else
    mcSHOW_DBG_MSG((" \n" ));
    #endif

    //vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 1, SPCMD_DQSGCNTRST);
    //mcDELAY_US(1);//delay 2T
    //vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 0, SPCMD_DQSGCNTRST);
}
#endif


#if (__ETT__ || CPU_RW_TEST_AFTER_K)
void DramcDumpDebugInfo(DRAMC_CTX_T *p)
{
    U8 mpdivInSel, cali_shu_sel, mpdiv_shu_sel;

    mpdivInSel= u4IO32ReadFldAlign(DDRPHY_PLL_SHU_GP, PLL_SHU_GP_PLL_SHU_GP);
    cali_shu_sel= u4IO32ReadFldAlign(DRAMC_REG_DLLCONF, DLLCONF_R_OTHER_SHU_GP);
    mpdiv_shu_sel= u4IO32ReadFldAlign(DRAMC_REG_DLLCONF, DLLCONF_R_MPDIV_SHU_GP);
    
    mcSHOW_DBG_MSG(("=========================\n"));
    
    // Read shuffle selection
    mcSHOW_ERR_MSG(("[DramcDumpDebugInfo] mpdivInSel %d, cali_shu_sel %d, mpdiv_shu_sel %d\n", mpdivInSel, cali_shu_sel, mpdiv_shu_sel));

    #if GATING_ONLY_FOR_DEBUG
    // Read gating error flag
    //DramcGatingDebugInit(p);
    DramcGatingDebug(p, CHANNEL_A);
    #if !SINGLE_CHANNEL_ENABLE
    DramcGatingDebug(p, CHANNEL_B);
    #endif
    #endif

    // Read HW gating tracking
    DramcPrintHWGatingStatus(p, CHANNEL_A);
    #if !SINGLE_CHANNEL_ENABLE
    DramcPrintHWGatingStatus(p, CHANNEL_B);
    #endif

    mcSHOW_DBG_MSG(("=========================\n"));
}
#endif

void DramcDebugTest(DRAMC_CTX_T *p)
{
#if 0
    U32 u4ErrorValue;

    // TestEngine2 R/W test
    p->test2_1 = DEFAULT_TEST2_1_CAL;
    u4ErrorValue = TestEngineCompare(p);
    mcSHOW_ERR_MSG(("[DramcDebugTest] First R/W test . Rank0: u4ErrorValue 0x%X\n", u4ErrorValue));
    #ifdef DUAL_RANKS
    if(uiDualRank)
    {
        p->test2_1 = 0x55a00000;
        u4ErrorValue = TestEngineCompare(p);
        mcSHOW_ERR_MSG(("[DramcDebugTest] First R/W test . Rank1: u4ErrorValue 0x%X\n", u4ErrorValue));
    }
    #endif

    // Reset PHY
    DramPhyReset(p);

    // TestEngine2 R/W test
    p->test2_1 = DEFAULT_TEST2_1_CAL;
    u4ErrorValue = TestEngineCompare(p);
    mcSHOW_ERR_MSG(("[DramcDebugTest] Reset PHY R/W test. Rank0: u4ErrorValue 0x%X\n", u4ErrorValue));
    
    #ifdef DUAL_RANKS
    if(uiDualRank)
    {
        p->test2_1 = 0x55a00000;
        u4ErrorValue = TestEngineCompare(p);
        mcSHOW_ERR_MSG(("[DramcDebugTest] Reset PHY R/W test . Rank1:u4ErrorValue 0x%X\n", u4ErrorValue));
    }
    #endif

    // Reset Dram
    Dram_Reset(p);

    // TestEngine2 R/W test
    p->test2_1 = DEFAULT_TEST2_1_CAL;
    u4ErrorValue = TestEngineCompare(p);
    mcSHOW_ERR_MSG(("[DramcDebugTest] Reset Dram R/W test. Rank0: u4ErrorValue 0x%X\n", u4ErrorValue));
    
    #ifdef DUAL_RANKS
    if(uiDualRank)
    {
        p->test2_1 = 0x55a00000;
        u4ErrorValue = TestEngineCompare(p);
        mcSHOW_ERR_MSG(("[DramcDebugTest] Reset Dram R/W test . Rank1:u4ErrorValue 0x%X\n", u4ErrorValue));
    }
    #endif

    p->test2_1 = DEFAULT_TEST2_1_CAL;
    #endif
}


void TransferToSPMControl(DRAMC_CTX_T *p)
{
    if(p->freq_sel > LJ_MAX_SEL) //LCPLL
    {
        *((UINT32P)(DDRPHY_BASE    + 0x580 )) = 0x7DFFA07F;
        *((UINT32P)(DDRPHY_BASE    + 0x584 )) = 0x7DFF607F;
        *((UINT32P)(DDRPHY_BASE    + 0x588 )) = 0x7FFF6FFF; 
        *((UINT32P)(DDRPHY1_BASE    + 0x580 )) = 0x7DFFA07F;
        *((UINT32P)(DDRPHY1_BASE    + 0x584 )) = 0x7DFF607F;
        *((UINT32P)(DDRPHY1_BASE    + 0x588 )) = 0x7FFF6FFF;    
        *((UINT32P)(DDRPHY2_BASE    + 0x580 )) = 0x7DFFA07F;
        *((UINT32P)(DDRPHY2_BASE    + 0x584 )) = 0x7DFF607F;
        *((UINT32P)(DDRPHY2_BASE    + 0x588 )) = 0x7FFF6FFF;    
        *((UINT32P)(DDRPHY3_BASE    + 0x580 )) = 0x7DFFA07F;
        *((UINT32P)(DDRPHY3_BASE    + 0x584 )) = 0x7DFF607F;
        *((UINT32P)(DDRPHY3_BASE    + 0x588 )) = 0x7FFF6FFF;    
    }
    else //LJPLL
    {
        *((UINT32P)(DDRPHY_BASE    + 0x580 )) = 0x7DFF907F;
        *((UINT32P)(DDRPHY_BASE    + 0x584 )) = 0x7DFF507F;
        *((UINT32P)(DDRPHY_BASE    + 0x588 )) = 0x7FFF5FFF;   
        *((UINT32P)(DDRPHY1_BASE    + 0x580 )) = 0x7DFF907F;
        *((UINT32P)(DDRPHY1_BASE    + 0x584 )) = 0x7DFF507F;
        *((UINT32P)(DDRPHY1_BASE    + 0x588 )) = 0x7FFF5FFF;  
        *((UINT32P)(DDRPHY2_BASE    + 0x580 )) = 0x7DFF907F;
        *((UINT32P)(DDRPHY2_BASE    + 0x584 )) = 0x7DFF507F;
        *((UINT32P)(DDRPHY2_BASE    + 0x588 )) = 0x7FFF5FFF;  
        *((UINT32P)(DDRPHY3_BASE    + 0x580 )) = 0x7DFF907F;
        *((UINT32P)(DDRPHY3_BASE    + 0x584 )) = 0x7DFF507F;
        *((UINT32P)(DDRPHY3_BASE    + 0x588 )) = 0x7FFF5FFF;  
    }
    vIO32WriteFldAlign_All(DDRPHY_PHY_SPM_CTL3, 1, PHY_SPM_CTL3_R_DMSUS_10_MUX);
}

void TransferToRegControl(void)
{
    vIO32Write4B_All(DDRPHY_PHY_SPM_CTL0, 0);
    vIO32Write4B_All(DDRPHY_PHY_SPM_CTL1, 0);
    vIO32Write4B_All(DDRPHY_PHY_SPM_CTL2, 0);
    //vIO32Write4B_All(DDRPHY_PHY_SPM_CTL3, 0);
}

void EnableDramcPhyDCM(DRAMC_CTX_T *p, bool bEn)
{
    if(bEn)
    {   
        *((UINT32P)(DDRPHY_BASE    + 0x130 )) = 0xEFB03CFF;
        *((UINT32P)(DDRPHY_BASE    + 0x138 )) = 0xF04FC300;
        *((UINT32P)(DDRPHY1_BASE    + 0x130 )) = 0xEF003F1F;
        *((UINT32P)(DDRPHY1_BASE    + 0x138 )) = 0xF001C0E0;
        *((UINT32P)(DDRPHY2_BASE    + 0x130 )) = 0xEF003C00;
        *((UINT32P)(DDRPHY2_BASE    + 0x138 )) = 0xF000001E;
        *((UINT32P)(DDRPHY3_BASE    + 0x130 )) = 0xEF003C1E;
        *((UINT32P)(DDRPHY3_BASE    + 0x138 )) = 0xF0000000;

        /*TINFO="=== set DRAMC low power golden setting ==="*/
        *((UINT32P)(DRAMC0_BASE    + 0x0c4 )) |= (0x1 << 30);
        *((UINT32P)(DRAMC0_BASE    + 0x1dc )) |= ((0x1 << 31) | (0x1 << 30) | (0x1 << 25) | (0x1 << 1));
        *((UINT32P)(DRAMC0_BASE    + 0x1dc )) &= ~((0x1 << 26) | (0x1 << 3));
        
        *((UINT32P)(DRAMC1_BASE    + 0x0c4 )) |= (0x1 << 30);
        *((UINT32P)(DRAMC1_BASE    + 0x1dc )) |= ((0x1 << 31) | (0x1 << 30) | (0x1 << 25) | (0x1 << 1));
        *((UINT32P)(DRAMC1_BASE    + 0x1dc )) &= ~((0x1 << 26) | (0x1 << 3));
    }
    else
    {
        /*TINFO="=== disable dramc/ddrphy/dramc_pipe MCK, freerun CK FR_CLK DCM & CG ==="*/
        //(*((volatile unsigned int *)(DFS_MEM_DCM_CTRL))) &= ~((0x1 << 3) | (0x1 << 4) | (0x1 << 8) | (0x1 << 9) );

        //trun off ddrphy dcm from dramc
        *((UINT32P)(DDRPHY_BASE    + 0x130 )) = 0;
        *((UINT32P)(DDRPHY_BASE    + 0x138 )) = 0;
        *((UINT32P)(DDRPHY1_BASE    + 0x130 )) = 0;
        *((UINT32P)(DDRPHY1_BASE    + 0x138 )) = 0;
        *((UINT32P)(DDRPHY2_BASE    + 0x130 )) = 0;
        *((UINT32P)(DDRPHY2_BASE    + 0x138 )) = 0;
        *((UINT32P)(DDRPHY3_BASE    + 0x130 )) = 0;
        *((UINT32P)(DDRPHY3_BASE    + 0x138 )) = 0;

        *((UINT32P)(DRAMC0_BASE    + 0x0c4 )) &= ~(0x1 << 30);
        *((UINT32P)(DRAMC0_BASE    + 0x1dc )) &= ~((0x1 << 31) | (0x1 << 30) | (0x1 << 25) | (0x1 << 1));
        *((UINT32P)(DRAMC0_BASE    + 0x1dc )) |= ((0x1 << 26) | (0x1 << 3));
        
        *((UINT32P)(DRAMC1_BASE    + 0x0c4 )) &= ~(0x1 << 30);
        *((UINT32P)(DRAMC1_BASE    + 0x1dc )) &= ~((0x1 << 31) | (0x1 << 30) | (0x1 << 25) | (0x1 << 1));
        *((UINT32P)(DRAMC1_BASE    + 0x1dc )) |= ((0x1 << 26) | (0x1 << 3));
    }
}

void DDRPhyLowPowerEnable(void)
{
    /*TINFO="=== set DDRPHY low power golden setting ==="*/

    //RG_*RPI_CAP_SEL*
    //->set in PLL init flow
    //RG_PISM_MCK_SEL_*=0
    vIO32WriteFldAlign_All(DDRPHY_PLL2, 0, PLL2_RG_PISM_MCK_SEL);

    //RG_RX_*_DVS_EN*=1
    vIO32WriteFldAlign_All(DDRPHY_EYE2, 1, Fld(1,31,AC_MSKB3)); //EYE2_RG_RX_ARDQS0_DVS_DLY[3]
    vIO32WriteFldAlign_All(DDRPHY_EYEB1_2, 1, Fld(1,31,AC_MSKB3)); //EYEB1_2_RG_RX_ARDQS1_DVS_DLY[3]
    vIO32WriteFldAlign_All(DDRPHY_CMD14, 1, Fld(1,31,AC_MSKB3)); //CMD14_RG_RX_ARCMDS0_DVS_DLY[3]
    //RG_RX_*_VREF_BYPASS*=1
    vIO32WriteFldAlign_All(DDRPHY_PHY_PDB_PU_0, 1 ,PHY_PDB_PU_0_RG_RX_ARDQ_VREF_BYPASS_B0);
    vIO32WriteFldAlign_All(DDRPHY_PHY_PDB_PU_1, 1 ,PHY_PDB_PU_1_RG_RX_ARDQ_VREF_BYPASS_B1);
    //RG_RX_*RDQS*_DQSSTB_CG_EN*=1, DQ_REV_B*[5]
    vIO32WriteFldAlign_Phy_All((DDRPHY_TXDQ3), 1, Fld(1,21,AC_MSKB2));
    vIO32WriteFldAlign_Phy_All((DDRPHY_RXDQ13), 1, Fld(1,21,AC_MSKB2));  
    //RG_RX_*_BIAS_PS*=2b'00 (DA mode)
    vIO32WriteFldAlign_All(DDRPHY_EYE3, 0, EYE3_RG_RX_ARDQ_BIAS_PS_B0);
    vIO32WriteFldAlign_All(DDRPHY_EYEB1_3, 0, EYEB1_3_RG_RX_ARDQ_BIAS_PS_B1);
    vIO32WriteFldAlign_All(DDRPHY_CMD15, 0, CMD15_RG_RX_ARCMD_BIAS_PS);

    //DA mode = RG | DRAMC control
    vIO32WriteFldMulti_All(DDRPHY_TXDQ3, P_Fld(0, TXDQ3_RG_RX_ARDQ_IN_BUFF_EN_B0) |
            P_Fld(0, TXDQ3_RG_RX_ARDQM0_IN_BUFF_EN) | P_Fld(0, TXDQ3_RG_RX_ARDQS0_IN_BUFF_EN));
    vIO32WriteFldMulti_All(DDRPHY_RXDQ13, P_Fld(0, RXDQ13_RG_RX_ARDQ_IN_BUFF_EN_B1) |
            P_Fld(0, RXDQ13_RG_RX_ARDQM1_IN_BUFF_EN) | P_Fld(0, RXDQ13_RG_RX_ARDQS1_IN_BUFF_EN));
    vIO32WriteFldMulti_All(DDRPHY_CMD12, P_Fld(0, CMD12_RG_RX_ARCMD_IN_BUFF_EN) | P_Fld(0, CMD12_RG_RX_ARCLK_IN_BUFF_EN));
    
    //RG_RX_*SMT_EN*=0
    vIO32WriteFldAlign_All(DDRPHY_TXDQ3, 0 , TXDQ3_RG_RX_ARDQ_SMT_EN_B0);
    vIO32WriteFldAlign_All(DDRPHY_RXDQ13, 0 , RXDQ13_RG_RX_ARDQ_SMT_EN_B1);
    vIO32WriteFldAlign_All(DDRPHY_CMD3, 0 , CMD3_RG_RX_ARCMD_SMT_EN);
    vIO32WriteFldAlign_All(DDRPHY_PLL17, 0 , PLL17_RG_RX_RRESETB_SMT_EN);
    //RG_RX_*RDQ_*EYE_VREF_EN*=0
    vIO32WriteFldAlign_All(DDRPHY_EYE2, 0 , EYE2_RG_RX_ARDQ_EYE_VREF_EN_B0);
    vIO32WriteFldAlign_All(DDRPHY_EYEB1_2, 0 , EYEB1_2_RG_RX_ARDQ_EYE_VREF_EN_B1);
    vIO32WriteFldAlign_All(DDRPHY_CMD14, 0 , CMD14_RG_RX_ARCMD_EYE_VREF_EN);
    
    //RG_CA_RIMP_EN=0, RG_CA_RIMP_BIAS_EN=0, RG_CA_RIMP_VREF_EN=0, RG_CA_RIMP_REV<0>=0
    vIO32WriteFldMulti(DDRPHY_IMPA1, P_Fld(0, IMPA1_RG_RIMP_BIAS_EN) | P_Fld(0, IMPA1_RG_RIMP_VREF_EN) | \
            P_Fld(0, Fld(1,8,AC_MSKB1)));
}

#if 0
//-------------------------------------------------------------------------
/** DramcEngine1
 *  start the self test engine inside dramc to test dram w/r.
 *  @param p                Pointer of context created by DramcCtxCreate.
 *  @param  test2_0         (U32): 16bits,set pattern1 [31:24] and set pattern0 [23:16].
 *  @param  test2_1         (U32): 28bits,base address[27:0].
 *  @param  test2_2         (U32): 28bits,offset address[27:0].
 *  @param  loopforever     (S16):  0 read\write one time ,then exit
 *                                 >0 enable eingie1, after "loopforever" second ,write log and exit
 *                                 -1 loop forever to read\write, every "period" seconds ,check result ,only when we find error,write log and exit
 *                                 -2 loop forever to read\write, every "period" seconds ,write log ,only when we find error,write log and exit
 *                                 -3 just enable loop forever ,then exit
 *  @param period           (U8):  it is valid only when loopforever <0; period should greater than 0
 *  @retval status          (U32): return the value of DM_CMP_ERR  ,0  is ok ,others mean  error
 */
//-------------------------------------------------------------------------
U32 DramcEngine1(DRAMC_CTX_T *p, U32 test2_1, U32 test2_2, S16 loopforever, U8 period)
{
    // This function may not need to be modified unless test engine-1 design has changed

    U8 ucengine_status;
    U8 ucstatus = 0, ucnumber;
    U32 u4result = 0xffffffff;
    U8 ucloop_count = 0;

    // error handling
    if (!p)
    {
        mcSHOW_ERR_MSG(("context is NULL\n"));
        return u4result;;
    }

    // This is TA1 limitation
    // offset must be 0x7ff
    if ((test2_2&0x00ffffff) != 0x000007ff)
    {
        mcSHOW_ERR_MSG(("TA1 offset must be 0x7ff!!\n"));
        mcSET_FIELD(test2_2, 0x7ff, 0x00ffffff, 0);
        mcSHOW_DBG_MSG2(("Force test2_2 to 0x%8x\n", test2_2));
    }
        
    // we get the status 
    // loopforever    period    status    mean 
    //     0             x         1       read\write one time ,then exit ,don't write log 
    //    >0             x         2       read\write in a loop,after "loopforever" seconds ,disable it ,return the R\W status  
    //    -1            >0         3       read\write in a loop,every "period" seconds ,check result ,only when we find error,write log and exit 
    //    -2            >0         4       read\write in a loop,every "period" seconds ,write log ,only when we find error,write log and exit
    //    -3             x         5       just enable loop forever , then exit (so we should disable engine1 outside the function)
    if (loopforever == 0)
    {
        ucengine_status = 1;
    }
    else if (loopforever > 0)
    {
        ucengine_status = 2;
    }
    else if (loopforever == -1)
    {
        if (period > 0)
        {
            ucengine_status = 3;
        }
        else
        {
            mcSHOW_ERR_MSG(("parameter 'status' should be equal or greater than 0\n"));
            return u4result;
        }
    }
    else if (loopforever == -2)
    {
        if (period > 0)
        {
            ucengine_status = 4;
        }
        else
        {
            mcSHOW_ERR_MSG(("parameter 'status' should be equal or greater than 0\n"));
            return u4result;
        }
    }
    else if (loopforever == -3)
    {
        ucengine_status = 5;
    }
    else
    {
        mcSHOW_ERR_MSG(("wrong parameter!\n"));
        mcSHOW_ERR_MSG(("loopforever    period    status    mean \n"));
        mcSHOW_ERR_MSG(("      0                x           1         read/write one time ,then exit ,don't write log\n"));
        mcSHOW_ERR_MSG(("    >0                x           2         read/write in a loop,after [loopforever] seconds ,disable it ,return the R/W status\n"));
        mcSHOW_ERR_MSG(("    -1              >0           3         read/write in a loop,every [period] seconds ,check result ,only when we find error,write log and exit\n"));
        mcSHOW_ERR_MSG(("    -2              >0           4         read/write in a loop,every [period] seconds ,write log ,only when we find error,write log and exit\n"));
        mcSHOW_ERR_MSG(("    -3                x           5         just enable loop forever , then exit (so we should disable engine1 outside the function)\n"));
        return u4result;
    }

    // set ADRDECEN=0,address decode not by DRAMC
    //2012/10/03, the same as A60806, for TA&UART b'31=1; for TE b'31=0
    //2013/7/9, for A60501, always set to 1
#if 0
    ucstatus |= ucDram_Register_Read(mcSET_DRAMC_REG_ADDR(DRAMC_REG_LPDDR2), &u4value);
    mcCLR_BIT(u4value, POS_LPDDR2_ADRDECEN);
    ucstatus |= ucDram_Register_Write(mcSET_DRAMC_REG_ADDR(DRAMC_REG_LPDDR2), u4value);
#endif

    // step
    // 1.select loop forever or not
    // 2.set pattern, base address,offset address
    // 3.enable test1 
    // 4.run different code according status
    // 5.diable test1
    // 6.return DM_CMP_ERR

    if (ucengine_status == 4)
    {
        mcSHOW_DBG_MSG(("============================================\n"));
        mcSHOW_DBG_MSG(("enable test egine1 loop forever\n"));        
        mcSHOW_DBG_MSG(("============================================\n"));
        ucnumber = 1;
    }

    // 1.
    if (loopforever != 0)
    {
        // enable infinite loop
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_CONF1), 1, CONF1_TESTLP);
    }
    else
    {
        // disable infinite loop
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_CONF1), 0, CONF1_TESTLP);
    }
    // 2.
    ucstatus |= vIO32Write4B(DRAMC_REG_ADDR(DRAMC_REG_TEST2_1), test2_1);    
    ucstatus |= vIO32Write4B(DRAMC_REG_ADDR(DRAMC_REG_TEST2_2), test2_2);    
    // 3.
    // enable test engine 1 (first write and then read???)
    // disable it before enable ,DM_CMP_ERR may not be 0,because may be loopforever and don't disable it before
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_CONF2), 0, CONF2_TEST1);

    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_CONF2), 1, CONF2_TEST1);

    // 4.
    if (ucengine_status == 1)
    {
        // read data compare ready check
        // infinite loop??? check DE about the time???
        ucloop_count = 0;
        while((u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_TESTRPT), TESTRPT_DM_CMP_CPT)) ==0)
        {            
            //ucstatus |= ucDram_Register_Read(mcSET_DRAMC_REG_ADDR(DRAMC_REG_TESTRPT), &u4value);
            mcDELAY_MS(CMP_CPT_POLLING_PERIOD);
            ucloop_count++;
            if (ucloop_count > MAX_CMP_CPT_WAIT_LOOP)
            {
                mcSHOW_ERR_MSG(("TESTRPT_DM_CMP_CPT polling timeout\n"));
                break;
            }
        }

        // delay 10ns after ready check from DE suggestion (1ms here)
        mcDELAY_MS(1);

        // save  DM_CMP_ERR, 0 is ok ,others are fail,disable test engine 1
        u4result = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_TESTRPT), TESTRPT_DM_CMP_ERR);
        //mcSHOW_DBG_MSG2(("0x3fc = %d\n", u4value));
    }
    else if (ucengine_status == 2)
    {
        // wait "loopforever" seconds
        mcDELAY_MS(loopforever*1000);
        // get result, no need to check read data compare ready???
        u4result = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_TESTRPT), TESTRPT_DM_CMP_ERR);
    }
    else if (ucengine_status == 3)
    {
        while(1)
        {
            // wait "period" seconds
            mcDELAY_MS(period*1000);
            // get result
            u4result = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_TESTRPT), TESTRPT_DM_CMP_ERR);
            if (u4result == 0)
            {
                // pass, continue to check
                continue;
            }
            // some bit error
            // write log
            mcSHOW_DBG_MSG(("%d#    CMP_ERR = 0x%8x\n", ucnumber, u4result));
            break;
        }
    }
    else if (ucengine_status == 4)
    {
        while(1)
        {
            // wait "period" seconds
            mcDELAY_MS(period*1000);
            // get result
            u4result = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_TESTRPT), TESTRPT_DM_CMP_ERR);
            // write log
            mcSHOW_DBG_MSG(("%d#    CMP_ERR = 0x%8x\n", ucnumber, u4result));
            
            if (u4result == 0)
            {
                // pass, continue to check
                continue;
            }
            // some bit error            
            break;
        }
    }
    else if (ucengine_status == 5)
    {
        // loopforever is  enable ahead ,we just exit this function
        return 0;
    }
    else
    {
    }

    // 5. disable engine1
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_CONF2), 0, CONF2_TEST1);

    // 6. 
    // set ADRDECEN to 1
    //2013/7/9, for A60501, always set to 1
#if 0
    ucstatus |= ucDram_Register_Read(mcSET_DRAMC_REG_ADDR(DRAMC_REG_LPDDR2), &u4value);
    mcSET_BIT(u4value, POS_LPDDR2_ADRDECEN);
    ucstatus |= ucDram_Register_Write(mcSET_DRAMC_REG_ADDR(DRAMC_REG_LPDDR2), u4value);
#endif

    return u4result;
}
#endif

void DramcSetRankEngine2(DRAMC_CTX_T *p, U8 u1RankSel)
{
    //LPDDR2_3_ADRDECEN_TARKMODE =0, always rank0
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_LPDDR2_3), 1, LPDDR2_3_ADRDECEN_TARKMODE);

    // DUMMY_TESTAGENTRKSEL =0, select rank according to CATRAIN_TESTAGENTRK
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DUMMY), 0, DUMMY_TESTAGENTRKSEL);

    //CATRAIN_TESTAGENTRK = u1RankSel
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_CATRAIN), u1RankSel, CATRAIN_TESTAGENTRK);
}

//-------------------------------------------------------------------------
/** DramcEngine2
 *  start the self test engine 2 inside dramc to test dram w/r.
 *  @param p                Pointer of context created by DramcCtxCreate.
 *  @param  wr              (DRAM_TE_OP_T): TE operation
 *  @param  test2_1         (U32): 28bits,base address[27:0].
 *  @param  test2_2         (U32): 28bits,offset address[27:0]. (unit is 16-byte, i.e: 0x100 is 0x1000).
 *  @param  loopforever     (S16): 0    read\write one time ,then exit
 *                                >0 enable eingie2, after "loopforever" second ,write log and exit
 *                                -1 loop forever to read\write, every "period" seconds ,check result ,only when we find error,write log and exit
 *                                -2 loop forever to read\write, every "period" seconds ,write log ,only when we find error,write log and exit
 *                                -3 just enable loop forever ,then exit
 *  @param period           (U8):  it is valid only when loopforever <0; period should greater than 0
 *  @param log2loopcount    (U8): test loop number of test agent2 loop number =2^(log2loopcount) ,0 one time
 *  @retval status          (U32): return the value of DM_CMP_ERR  ,0  is ok ,others mean  error
 */
//-------------------------------------------------------------------------
U32 DramcEngine2(DRAMC_CTX_T *p, DRAM_TE_OP_T wr, U32 test2_1, U32 test2_2, U8 testaudpat, S16 loopforever, U8 period, U8 log2loopcount)
{
    U8 ucengine_status;
    U32 u4loop_count = 0;
    U32 u4result = 0xffffffff;
    U32 u4log2loopcount = (U32) log2loopcount;

    // error handling
    if (!p)
    {
        mcSHOW_ERR_MSG(("context is NULL\n"));
        return u4result;
    }

    // check loop number validness
//    if ((log2loopcount > 15) || (log2loopcount < 0))		// U8 >=0 always.
    if (log2loopcount > 15)
    {
        mcSHOW_ERR_MSG(("wrong parameter log2loopcount:    log2loopcount just 0 to 15 !\n"));
        return u4result;
    }

    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_LPDDR2_3), 0, LPDDR2_3_DMA_LP4MATAB_OPT);

    // disable self test engine1 and self test engine2 
    vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_CONF2), P_Fld(0, CONF2_TEST2W) | P_Fld(0, CONF2_TEST2R) | P_Fld(0, CONF2_TEST1));

    // we get the status 
    // loopforever    period    status    mean 
    //     0             x         1       read\write one time ,then exit ,don't write log 
    //    >0             x         2       read\write in a loop,after "loopforever" seconds ,disable it ,return the R\W status  
    //    -1            >0         3       read\write in a loop,every "period" seconds ,check result ,only when we find error,write log and exit 
    //    -2            >0         4       read\write in a loop,every "period" seconds ,write log ,only when we find error,write log and exit
    //    -3             x         5       just enable loop forever , then exit (so we should disable engine1 outside the function)
    if (loopforever == 0)
    {
        ucengine_status = 1;
    }
    else if (loopforever > 0)
    {
        ucengine_status = 2;
    }
    else if (loopforever == -1)
    {
        if (period > 0)
        {
            ucengine_status = 3;
        }
        else
        {
            mcSHOW_ERR_MSG(("parameter 'period' should be equal or greater than 0\n"));
            return u4result;
        }
    }
    else if (loopforever == -2)
    {
        if (period > 0)
        {
            ucengine_status = 4;
        }
        else
        {
            mcSHOW_ERR_MSG(("parameter 'period' should be equal or greater than 0\n"));
            return u4result;
        }
    }
    else if (loopforever == -3)
    {
        if (period > 0)
        {
            ucengine_status = 5;
        }
        else
        {
            mcSHOW_ERR_MSG(("parameter 'period' should be equal or greater than 0\n"));
            return u4result;
        }
    }
    else
    {
        mcSHOW_ERR_MSG(("parameter 'loopforever' should be 0 -1 -2 -3 or greater than 0\n"));        
        return u4result;
    }

    // set ADRDECEN=0, address decode not by DRAMC
    //2012/10/03, the same as A60806, for TA&UART b'31=1; for TE b'31=0
    //2013/7/9, for A60501, always set to 1
#ifdef fcFOR_A60806_TEST
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_LPDDR2_3),0, LPDDR2_3_ADRDECEN);
#endif

    // 1.set pattern ,base address ,offset address
    // 2.select  ISI pattern or audio pattern or xtalk pattern
    // 3.set loop number
    // 4.enable read or write
    // 5.loop to check DM_CMP_CPT
    // 6.return CMP_ERR
    // currently only implement ucengine_status = 1, others are left for future extension    
    /*if (ucengine_status == 4)
    {
        mcSHOW_DBG_MSG(("============================================\n"));
        mcSHOW_DBG_MSG(("enable test egine2 loop forever\n"));        
        mcSHOW_DBG_MSG(("============================================\n"));        
    }*/
    u4result = 0;
    while(1)
    {
        // 1
        vIO32Write4B(DRAMC_REG_ADDR(DRAMC_REG_TEST2_1), test2_1);    
        vIO32Write4B(DRAMC_REG_ADDR(DRAMC_REG_TEST2_2), test2_2);    

        // 2 & 3
        // (TESTXTALKPAT, TESTAUDPAT) = 00 (ISI), 01 (AUD), 10 (XTALK), 11 (UNKNOWN)
        if (testaudpat == 2)   // xtalk
        {
            // select XTALK pattern
            // set addr 0x044 [7] to 0
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_TEST2_3), P_Fld(0, TEST2_3_TESTAUDPAT)|P_Fld(u4log2loopcount,TEST2_3_TESTCNT));

            // set addr 0x48[16] to 1, TESTXTALKPAT = 1
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_TEST2_4), P_Fld(1, TEST2_4_TESTXTALKPAT)|P_Fld(0,TEST2_4_TESTAUDMODE)|P_Fld(0,TEST2_4_TESTAUDBITINV));
        }
        else if (testaudpat == 1)   // audio
        {
            // set AUDINIT=0x11 AUDINC=0x0d AUDBITINV=1 AUDMODE=1
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_TEST2_4), \
                P_Fld(0x00000011, TEST2_4_TESTAUDINIT)|P_Fld(0x0000000d, TEST2_4_TESTAUDINC)| \
                P_Fld(0, TEST2_4_TESTXTALKPAT)|P_Fld(1,TEST2_4_TESTAUDMODE)|P_Fld(1,TEST2_4_TESTAUDBITINV));

            // set addr 0x044 [7] to 1 ,select audio pattern
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_TEST2_3), P_Fld(1, TEST2_3_TESTAUDPAT)|P_Fld(u4log2loopcount,TEST2_3_TESTCNT));
        } 
        else   // ISI
        {
            // select ISI pattern
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_TEST2_3), P_Fld(0, TEST2_3_TESTAUDPAT)|P_Fld(u4log2loopcount,TEST2_3_TESTCNT));
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_TEST2_4), 0, TEST2_4_TESTXTALKPAT);
        }

        // 4
        if (wr == TE_OP_READ_CHECK)
        {
            if ((testaudpat == 1) || (testaudpat == 2))
            {
                //if audio pattern, enable read only (disable write after read), AUDMODE=0x48[15]=0
                vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_TEST2_4), 0, TEST2_4_TESTAUDMODE);
            }
            
            // enable read, 0x008[31:29]
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_CONF2), P_Fld(0, CONF2_TEST2W) | P_Fld(1, CONF2_TEST2R) | P_Fld(0, CONF2_TEST1));
        }
        else if (wr == TE_OP_WRITE_READ_CHECK)
        {
            // enable write
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_CONF2), P_Fld(1, CONF2_TEST2W) | P_Fld(0, CONF2_TEST2R) | P_Fld(0, CONF2_TEST1));

            // read data compare ready check            
            u4loop_count = 0;
            while((u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_TESTRPT), TESTRPT_DM_CMP_CPT))==0)
            {
                //ucstatus |= ucDram_Register_Read(mcSET_DRAMC_REG_ADDR(DRAMC_REG_TESTRPT), &u4value);
                mcDELAY_US(CMP_CPT_POLLING_PERIOD);
                u4loop_count++;
                if ((u4loop_count > 3) &&(u4loop_count <= MAX_CMP_CPT_WAIT_LOOP))
                {
                    //mcSHOW_ERR_MSG(("TESTRPT_DM_CMP_CPT (Write): %d\n", u4loop_count));
                }
                else if(u4loop_count > MAX_CMP_CPT_WAIT_LOOP)
                {
                   /*TINFO="fcWAVEFORM_MEASURE_A %d: time out\n", u4loop_count*/
                   mcSHOW_DBG_MSG(("fcWAVEFORM_MEASURE_A %d :time out\n", u4loop_count));
                   mcFPRINTF((fp_A60501, "fcWAVEFORM_MEASURE_A %d: time out\n", u4loop_count));
                    break;
                }
            }
            
            // disable write
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_CONF2), P_Fld(0, CONF2_TEST2W) | P_Fld(0, CONF2_TEST2R) | P_Fld(0, CONF2_TEST1));
            // enable read
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_CONF2), P_Fld(0, CONF2_TEST2W) | P_Fld(1, CONF2_TEST2R) | P_Fld(0, CONF2_TEST1));
        }

        // 5
        // read data compare ready check
        u4loop_count = 0;
        while((u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_TESTRPT), TESTRPT_DM_CMP_CPT))==0)
        {
                mcDELAY_US(CMP_CPT_POLLING_PERIOD);
                u4loop_count++;
                if ((u4loop_count > 3)&&(u4loop_count <= MAX_CMP_CPT_WAIT_LOOP))
                {
                    //mcSHOW_ERR_MSG(("TESTRPT_DM_CMP_CPT (Read): %d\n", u4loop_count));
                }
                else if(u4loop_count > MAX_CMP_CPT_WAIT_LOOP)
                {
                   /*TINFO="fcWAVEFORM_MEASURE_B %d: time out\n", u4loop_count*/
                   mcSHOW_DBG_MSG(("fcWAVEFORM_MEASURE_B %d: time out\n", u4loop_count));
                   mcFPRINTF((fp_A60501, "fcWAVEFORM_MEASURE_B %d: time out\n", u4loop_count));
                   break;
                }
        }

        // delay 10ns after ready check from DE suggestion (1ms here)
        mcDELAY_US(1);

        // 6
        // return CMP_ERR, 0 is ok ,others are fail,diable test2w or test2r
        // get result
        // or all result
        u4result |= (u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_CMP_ERR)));
        // disable read
        vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_CONF2), P_Fld(0, CONF2_TEST2W) | P_Fld(0, CONF2_TEST2R) | P_Fld(0, CONF2_TEST1));

        // handle status
        if (ucengine_status == 1)
        {
            // set ADRDECEN to 1
            //2013/7/9, for A60808, always set to 1
            #ifdef fcFOR_A60806_TEST
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_LPDDR2_3),1, LPDDR2_3_ADRDECEN);
            #endif
            break;    
        }
        else if (ucengine_status == 2)
        {
            mcSHOW_ERR_MSG(("not support for now\n"));
            break;
        }
        else if (ucengine_status == 3)
        {
            mcSHOW_ERR_MSG(("not support for now\n"));
            break;
        }
        else if (ucengine_status == 4)
        {
            mcSHOW_ERR_MSG(("not support for now\n"));
            break;
        }
        else if (ucengine_status == 5)
        {
            mcSHOW_ERR_MSG(("not support for now\n"));
            break;
        }
        else
        {
            mcSHOW_ERR_MSG(("not support for now\n"));
            break;
        }
    }

    return u4result;
}

U32 TestEngineCompare(DRAMC_CTX_T *p)
{
    U8 jj;
    U32 u4err_value;
    
    if (p->test_pattern== TEST_AUDIO_PATTERN)
    {
        // enable TE2, audio pattern
        if (p->fglow_freq_write_en == ENABLE)
        {
            u4err_value = DramcEngine2(p, TE_OP_READ_CHECK, p->test2_1, p->test2_2, 1, 0, 0, 0);
        }
        else
        {
            u4err_value = 0;
            for (jj = 0; jj < 1; jj++)
            {
                u4err_value |= DramcEngine2(p, TE_OP_WRITE_READ_CHECK, p->test2_1, p->test2_2, 1, 0, 0, 0);
            }
        }
    }
    else if (p->test_pattern == TEST_ISI_PATTERN)
    {
        // enable TE2, ISI pattern
        u4err_value = 0;
        for (jj = 0; jj < 1; jj++)
        {
            u4err_value |= DramcEngine2(p, TE_OP_WRITE_READ_CHECK, p->test2_1, p->test2_2, 0, 0, 0, 0);
        }            
    }
#if fcFOR_CHIP_ID == fcA60501
    else if (p->test_pattern == TEST_TESTPAT4)
    {
        // Not implement yet!!
        ////u4err_value = pi_testpat4(channel);   // OR the error result and return
    }
    else if (p->test_pattern == TEST_TESTPAT4_3)
    {
        u4err_value = DramcPat4_3(p);
    }
#endif
    else if (p->test_pattern == TEST_XTALK_PATTERN)
    {
        if (p->fglow_freq_write_en == ENABLE)
        {
            u4err_value = DramcEngine2(p, TE_OP_READ_CHECK, p->test2_1, p->test2_2, 2, 0, 0, 0);
        }
        else
        {
            u4err_value = DramcEngine2(p, TE_OP_WRITE_READ_CHECK, p->test2_1, p->test2_2, 2, 0, 0, 0);
        }
    }
    else if (p->test_pattern == TEST_MIX_PATTERN)
    {
        u4err_value = DramcEngine2(p, TE_OP_WRITE_READ_CHECK, p->test2_1, p->test2_2, 1, 0, 0, 0);
        u4err_value |= DramcEngine2(p, TE_OP_WRITE_READ_CHECK, p->test2_1, p->test2_2, 2, 0, 0, 0);
    }
#if fcFOR_CHIP_ID == fcA60501
    else if (p->test_pattern == TEST_DMA)
    {
        u4err_value = DramcDmaEngine(p, DMA_OP_READ_WRITE, 0x20, 0x20, 0x1ff, 3, 0); // to be fix. DMA_DATA_ACCESS_AND_COMPARE
    }
#endif
    else
    {
        mcSHOW_ERR_MSG(("Not support test pattern!! Use audio pattern by default.\n"));
        u4err_value = DramcEngine2(p, TE_OP_WRITE_READ_CHECK, p->test2_1, p->test2_2, 1, 0, 0, 0);
    }        
	return u4err_value;
}


#if __ETT__
//-------------------------------------------------------------------------
/** DramcRegDump
 *  Dump all registers (DDRPHY and DRAMC)
 *  @param p                Pointer of context created by DramcCtxCreate.
 *  @retval status          (DRAM_STATUS_T): DRAM_OK or DRAM_FAIL 
 */
//-------------------------------------------------------------------------
void DramcRegDumpRange(U32 u4StartAddr, U32 u4EndAddr)
{
    U32 ii;

    for(ii=u4StartAddr; ii<u4EndAddr; ii+=4)
    {
        mcSHOW_DBG_MSG(("Address 0x%X = 0x%X\n", ii, (*(volatile unsigned int *)(ii))));
    }
}

DRAM_STATUS_T DramcRegDump(DRAMC_CTX_T *p)
        {
    mcSHOW_DBG_MSG(("\n=================CHA_DRAMC_NAO_BASE================================\n"));
    DramcRegDumpRange(0x1020E000, 0x1020E5a0);
    mcSHOW_DBG_MSG(("\n=================CHB_DRAMC_NAO_BASE================================\n"));
    DramcRegDumpRange(0x10216000, 0x102165a0);
    mcSHOW_DBG_MSG(("\n=================CHA_DRAMC_AO_BASE================================\n"));
    DramcRegDumpRange(0x10004000, 0x10004d58);
    mcSHOW_DBG_MSG(("\n=================CHB_DRAMC_AO_BASE================================\n"));
    DramcRegDumpRange(0x10013000, 0x10013d58);


    mcSHOW_DBG_MSG(("\n=================PHY_A_BASE================================\n"));
    DramcRegDumpRange(0x1000f000, 0x1000ffe8);
    mcSHOW_DBG_MSG(("\n=================PHY_B_BASE================================\n"));
    DramcRegDumpRange(0x10012000, 0x10012fe8);
    mcSHOW_DBG_MSG(("\n=================PHY_C_BASE================================\n"));
    DramcRegDumpRange(0x10018000, 0x10018fe8);
    mcSHOW_DBG_MSG(("\n=================PHY_D_BASE================================\n"));
    DramcRegDumpRange(0x10019000, 0x10019fe8);

    return DRAM_OK;
}
#endif

//-------------------------------------------------------------------------
/** DramcStopTA1Mc
 *  Stop TA1 Micro Controller
 *  @param p                Pointer of context created by DramcCtxCreate.
 *  @param timeout          (U16) timeout
 *  @retval void 
 */
//-------------------------------------------------------------------------
void DramcStopTA1Mc(DRAMC_CTX_T *p, U16 timeout)
{
#if fcFOR_CHIP_ID == fcA60501
U8 TA_ADDR, ucstatus = 0, ii, ta1_mc_running;
U32 u4addr, u4value;

    if (p->channel == CHANNEL_A)
    {
        TA_ADDR = 10;
        ta1_mc_running = ta1_mc_running_cha;
    }
    else if (p->channel == CHANNEL_B)
    {
        TA_ADDR = 11;
        ta1_mc_running = ta1_mc_running_chb;
    }
    else
    {
        TA_ADDR = 12;
        ta1_mc_running = ta1_mc_running_chc;
    }

    if (ta1_mc_running == ENABLE)
    {

        u4addr = (DRAMC_BASE_ADDRESS | (TA_ADDR << POS_BANK_NUM) | (0xe30));
        ucstatus |= vIO32Write4B(u4addr, 2);


        u4addr = (DRAMC_BASE_ADDRESS | (TA_ADDR << POS_BANK_NUM) | (0xafc));
        for (ii = 0; ii < timeout; ii++)
        {
            u4value = u4IO32Read4B(u4addr);
            if ((u4value & 72) == 0)
            {
                break;
            }
            mcDELAY_MS(1);
        }

        if (ii == timeout)
        {
            mcSHOW_ERR_MSG(("stop_ta1_mc timeout=%d\n", ii));
        }
        else
        {
            //mcSHOW_DBG_MSG2(("stop_ta1_mc %d trial(s).\n", ii+1));
        }

        if (p->channel == CHANNEL_A)
        {
            ta1_mc_running_cha = DISABLE;
        }
        else if (p->channel == CHANNEL_B)
        {
            ta1_mc_running_chb = DISABLE;
        }   
        else
        {
            ta1_mc_running_chc = DISABLE;
        }
    }
    else
    {
        // if we exit tcl shell when we are running testpat4-3 ,then entry cli again
        // mc is running ,but variable ta1_mc_running is false
        // at this point ,we disable TA1 and mc directly by setting register 0xafc 
        u4addr = (DRAMC_BASE_ADDRESS | (TA_ADDR << POS_BANK_NUM) | (0xafc));
        u4value = u4IO32Read4B(u4addr);

        if ( (u4value & 0x48) != 0 )
        {
            mcSHOW_DBG_MSG(("ta1_mc_running is false ,but TA1 is enabled according to the register\n"));

            u4value = u4value & 0xffffffb7;
            ucstatus |= vIO32Write4B(u4addr, u4value);

            mcDELAY_MS(1);

            if (p->channel == CHANNEL_A)
            {
                last_testpat_cha = TA_PATTERN_IDLE;
            }
            else if (p->channel == CHANNEL_B)
            {
                last_testpat_chb = TA_PATTERN_IDLE;
            }
            else
            {
                last_testpat_chc = TA_PATTERN_IDLE;
            }
        }
    }
#endif
}

//-------------------------------------------------------------------------
/** DramcClearTA1
 *  Clear TA1
 *  @param p                Pointer of context created by DramcCtxCreate.
 *  @retval void 
 */
//-------------------------------------------------------------------------
void DramcClearTA1(DRAMC_CTX_T *p)
{
#if fcFOR_CHIP_ID == fcA60501
U8 TA_ADDR, ucstatus = 0, ta1_mc_running;
U32 u4addr, u4value;

    if (p->channel == CHANNEL_A)
    {
        TA_ADDR = 10;
        ta1_mc_running = ta1_mc_running_cha;
    }
    else if (p->channel == CHANNEL_B)
    {
        TA_ADDR = 11;
        ta1_mc_running = ta1_mc_running_chb;
    }
    else
    {
        TA_ADDR = 12;
        ta1_mc_running = ta1_mc_running_chc;
    }
    
    if (ta1_mc_running == ENABLE)
    {
        u4addr = (DRAMC_BASE_ADDRESS | (TA_ADDR << POS_BANK_NUM) | (0xe30));
        ucstatus |= vIO32Write4B(u4addr, 1);
    }
    else
    {
        //mcSHOW_DBG_MSG(("according to ta1_mc_running, ta1_mc is disabled, so clear ta1 log directly.\n"));

        u4addr = (DRAMC_BASE_ADDRESS | (TA_ADDR << POS_BANK_NUM) | (0xafc));
        u4value = u4IO32Read4B(u4addr);
        if (u4value&0x48)
        {
            mcSHOW_ERR_MSG(("ta1_mc is running?! Cannot clear ta1 error log now!!\n"));
            return;
        }

        ucstatus |= vIO32Write4B(u4addr, (u4value | 0x20));
        ucstatus |= vIO32Write4B(u4addr, ((u4value & ~0x20) & 0xffffffff));    
    }
#endif
}


//-------------------------------------------------------------------------
/** DramcTestPat4_3
 *  Run TA test pattern 4_3 by OP type
 *  @param p                Pointer of context created by DramcCtxCreate.
 *  @param op_type          (DRAM_TA43_OP_TYPE_T) operation type
 *  @retval void 
 */
//-------------------------------------------------------------------------
void DramcTestPat4_3(DRAMC_CTX_T *p, DRAM_TA43_OP_TYPE_T type)
{
#if fcFOR_CHIP_ID == fcA60501
U8 TA_ADDR, ucstatus = 0, ta1_mc_running;
U32 u4addr, u4value;
DRAM_TA_PATTERN_T last_testpat;

    // for A60501
    // change REG_MEMBUS_SEL of reg0x04[17:16] to 2`b00 (01 for UART r/w memory)
    u4value = u4IO32Read4B(mcSET_SYS_REG_ADDR(0x004));
    mcSET_FIELD(u4value, 0x0, 0x00030000, 16);
    ucstatus |= vIO32Write4B(mcSET_SYS_REG_ADDR(0x004), u4value);

    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_LPDDR2_3), 1, LPDDR2_3_DMA_LP4MATAB_OPT);

    if (p->channel == CHANNEL_A)
    {
        TA_ADDR = 10;
#if 0
        // 2012/09/27, only for A60807, bypass arbiter, 0120[3:0]=0010 for TA
        u4value = u4IO32Read4B(0x20000120, &u4value);
        mcSET_FIELD(u4value, 2, 0x0000000f, 0);
        ucstatus |= vIO32Write4B(0x20000120, u4value);
#endif
        last_testpat = last_testpat_cha;
        ta1_mc_running = ta1_mc_running_cha;	  
    }
    else if (p->channel == CHANNEL_B)
    {
        TA_ADDR = 11;
#if 0
        // 2012/09/27, only for A60807, bypass arbiter, 0120[7:4]=0010 for TA
        u4value = u4IO32Read4B(0x20000120, &u4value);
        mcSET_FIELD(u4value, 2, 0x000000f0, 4);
        ucstatus |= vIO32Write4B(0x20000120, u4value);
#endif
        last_testpat = last_testpat_chb;
        ta1_mc_running = ta1_mc_running_chb;	 
    }
    else
    {
        TA_ADDR = 12;
#if 0
        // 2012/09/27, only for A60807, bypass arbiter, 0120[7:4]=0010 for TA
        u4value = u4IO32Read4B(0x20000120, &u4value);
        mcSET_FIELD(u4value, 2, 0x000000f0, 4);
        ucstatus |= vIO32Write4B(0x20000120, u4value);
#endif
        last_testpat = last_testpat_chc;
        ta1_mc_running = ta1_mc_running_chc;	 
    }

    if (type == TA43_OP_STOP)
    {
        DramcStopTA1Mc(p, 1000);
        return;
    }
    else if (type == TA43_OP_CLEAR)
    {      
        DramcClearTA1(p);
        return;
    }
    else if ((type != TA43_OP_RUN) && (type != TA43_OP_RUNQUIET))
    {
        mcSHOW_ERR_MSG(("testpat4_3 cannot recognize user parameter: %d\n", type));
        return;
    }

    if (ta1_mc_running == ENABLE)
    {
        DramcStopTA1Mc(p, 1000);
    }

    // ta1_mc + ta1 + ta0 test pattern, endless loop
    // (testpat4_3.v random pattern, add toggle on R_TAAG1GENERAL<13:12>)

    // ------------------------- fill ta0 ref_mem ... ---------------------
    if (last_testpat != TA_PATTERN_TA43)
    {
        // load code.
        // run_quiet,
        // load code.
        // last_testpat 

        // the same as 'write 0x20003ae0 0x0fffffff'
        // PATS<31:0> = 0x0fff_ffff
        u4addr = (DRAMC_BASE_ADDRESS | (TA_ADDR << POS_BANK_NUM) | (0x400));
        ucstatus |= vIO32Write4B(u4addr, 0x10802ae0);
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x0fffffff);

        // the same as 'write 0x20003ae4 0x7ffffff8'
        // PATS<63:32> = 0x7fff_fff8
        u4addr += 4;
        ucstatus |= vIO32Write4B(u4addr, 0x10802ae4);
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x7ffffff8);

        // the same as 'write 0x20003ae8 0x70000000'
        // PATO<31:0> = 0x7000_0000
        u4addr += 4;
        ucstatus |= vIO32Write4B(u4addr, 0x10802ae8);
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x70000000);

        // the same as 'write 0x20003aec 0x00000007'
        // PATO<63:32> = 0x0000_0007
        u4addr += 4;
        ucstatus |= vIO32Write4B(u4addr, 0x10802aec);
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x00000007);

        // the same as 'write 0x20003af0 0x601f1c1c'
        // PATAUX<15:0> = 0x601f , PATOSC<15:0> = 0x1c1c
        u4addr += 4;
        ucstatus |= vIO32Write4B(u4addr, 0x10802af0);
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x60131c1c);

        // toggle bit range, bit 0 ~ 0x1f
        u4addr += 4;
        ucstatus |= vIO32Write4B(u4addr, 0x10802adc);
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x00001f00);

        // the same as write 0x20003e3c 0x00000000
        // address scramble control
        u4addr += 4;
        ucstatus |= vIO32Write4B(u4addr, 0x10802e3c);
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x00000000);

        // ---------------------------------
        if (ta_option_turn_around == ENABLE)
        {
            // constant 0xf put in #4
            //#load #4, 0xf
            u4addr +=4;
            ucstatus |= vIO32Write4B(u4addr, 0x11c0000f);
        }
        else
        {
            // constant 7 put in #4
            // load #4, 7
            u4addr +=4;
            ucstatus |= vIO32Write4B(u4addr, 0x11c00007);
        }

        // an iterator counting 0, 3, 2, 1, 0, 3, 2, 1, 0, 3, 2, 1, 0, 3, ... ...
        // load #5, 0
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x11d00000);    

        //a base value for register TA1GENERAL
        //load #6, 0x05400048
        //----<31:28>TestINIT=0 , <27:24>TestSTEP = 5 , <23>Test16 = 0
        //----<22:20>BurstNumLevel=4 (0: 32 iter, 1: 64 iter, ..., 4 or more: 512 iter)
        //----<14>PATMODE<2>=1 (other bit) (1: Has pulse) (pulse direction = ! <0> ^ <1>)
        //----<13>PATMODE<1>=1 (other bit) (1: opposite base value) (base value = <0> ^ <1>)
        //----<12>PATMODE<0>=0 (the tested bit) (0: Pulse-1)
        //----<8:11>Write turn around (+ x)
        //----<7>Read wait DLE
        //----<6>batch enable
        //----<5>clear log
        //----<4>PAUSE
        //----<3>AG1EN=1
        //NOTE : bit<6> batch enable ==> maintain the batch mode, but not yet run now !
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x11ef0540);
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x11ec0040);

        //load #7, imm val 0
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x11f00000);

        //store R_TAAG1RX18C, #7 (now it's zero)'
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x10702e30);

        //store R_TAAG1RX18D, #7 (now it's zero)'
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x10702e34);

        //---------------------------------
        //label_new_rndseed_loop_start:
        //store &(R_TAAG1RNDSEED), #7
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x10702af4);        
        
        //label_loop_start:
        //update #6<13:12>=#5<1:0>
        //update #6<13:12>=#5<5:4>
        //prog_add 0x185e3020
        //prog_add 0x185e3024
        //prog_add 0x184e3023 ; # val==2
        //prog_add 0x184e3025 ; # val==1
        //prog_add 0x184e3022 ; # val==0
        //#6[14:12] =#5[2:0]
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x185e3040);

        if (ta_option_turn_around == ENABLE)
        {
            u4addr +=4;
            ucstatus |= vIO32Write4B(u4addr, 0x185e1c00);    //#6[7]=#5[0]	
            u4addr +=4;
            ucstatus |= vIO32Write4B(u4addr, 0x185e2000);    //#6[8]=#5[0]
        }

        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x26620008);    //set ta1_enable bit: #6 = #6 | 0x0008    

        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x10602afc);    //enable ta1: store #6 to 0x20003afc (TA1GENERAL)
    
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x00010000);    //wait (disable fetch code) until ta1 state=pause (ta1 done)
    

        //write 0x20003afc 0x03006040
        //bit<3> = 0 to disable the ta1 loop
        //prog_add 0x10802afc
        //prog_add 0x03006040

        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x26e1fff7);    //disable ta1_enable bit : #6 = #6 & signed 0xfff7
    
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x10602afc);
    
        //#5=(#4|0) &(#5+#4),where # 4 == const 7 or 0xf
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x694b1400);
        
        //prog_add 0x694b0c07

        //if #5 !=0 goto -6*4 ()
        //prog_add {0x0cd83fe0 label_loop_start:}
        //#prog_add 0x2774f3b5 ; prog_add {0x0cd83fe0 label_new_rndseed_loop_start:}
        //#(ins_16imm_alu 3'h7 sign-ext=0 3'h7, op=4(add) imm16=16'hf3b5)  '
        //#(7 += 16'hf3b5)         '
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x0cd83fdc);


        //#0 = *(TA1RX18C)
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x11002e30);
   
        //#1 = *(TA1RX18D)
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x11102e34);
    
        //#7 = #7 + 16'hf3b5 unsigned                  '
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x2774f3b5);
    
        //store &(R_TAAG1RX18E), #7
        //user read it as a monitor register
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x10702e38);
    
        //NOP
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0xffffff00);

        //if #0 == #1 goto -13*4 (再往前 8) (label_new_rndseed_loop_start)
        //prog_add {0x0c013fc4 label_new_rndseed_loop_start:}
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x0c013fc0);

        //else ==> user specified a change.

        //store &(TARX18C), #1
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x10102e30);

        //#0 = #0 ^ #1 = (#0 & 0) ^ (#0 ^ #1)
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x42004b00);
    
        //#1 = #0 & 1
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x21010001);

        //if #1 == 0 goto +3*4 
        //prog_add {0x0c180014 label3:}
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x0c180010);

        //write TA1GENERAL, bit<5> = 1 to clear log
        //prog_add 0x10802afc
        //prog_add 0x03006060
    
        //prog_add 0x10802afc
        //prog_add 0x03006040
        //set #6 的 clear log bit
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x26620020);    //#6 = #6 | 0x0020
    
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x10602afc);
    
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x26e1ffdf);    //#6 = #6 & signed 0xffdf

        //prog_add 0xffffff00

        //label3:
        //#0 = #0 ^ #1 = (#0 & 0) ^ (#0 ^ #1)
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x42004b00);

        //if #0==0 goto -21*4
        //5'b00001, 1'b1, 2'b00, 1'b0, 3'b000, 1'b1, 3'b0, 2'b0, 12'hfeb, 2'b00
        //ins_br  , rela, delay,  BNE,     #0, cnst,    x,    x,     -21,     0
        //prog_add {0x0c083f9c label_new_rndseed_loop_start:}
        //user toggle the bits in 0x20003e34, and test agent copy it to 0x20003e30
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x0c083f9c);

        //------------------------------
        //write 0x20003afc 0x03006000
        //bit<6> = 0 to disable batch mode
        //label_quit:
        //prog_add 0x10802afc
        //prog_add 0x03006000
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x26e1ffbf);    //#6 = #6 & signed 0xffbf (or 0xffb7)
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x10602afc);    

        //wait (disable fetch code) forever => next time cannot enter batch mode
        //write 0x200034b4 0x00800000
        //if #0 == #0 goto +0*4 (endless loop) => will hit bug?
        //write 0x200034b4 0x0c000000
        //if #0 == #0 goto +1*4
        //prog_add {0x00800004 label4:} ; 
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0x00800004);

        // 2 NOPs
        //label4:
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0xffffff00);    
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0xffffff00);
        u4addr +=4;
        ucstatus |= vIO32Write4B(u4addr, 0xffffff00);

        if (p->channel == CHANNEL_A)
        {
            last_testpat_cha = TA_PATTERN_TA43;
        }
        else if (p->channel == CHANNEL_B)
        {
            last_testpat_chb = TA_PATTERN_TA43;
        }
        else
        {
            last_testpat_chc = TA_PATTERN_TA43;
        }
    }

    //-------------------------------------------------------------
    u4addr = (DRAMC_BASE_ADDRESS | (TA_ADDR << POS_BANK_NUM) | (0xafc));
    ucstatus |= vIO32Write4B(u4addr, 0x00006160);

    //----<6> is 0 means the code is stopped.
    //set hex_check_status [proc_read 0x20003afc 0x4 0]
    //if {$hex_check_status=="03006000"} {
    //   mtk_log warning "ta1_mc is stopped by itself right after getting started."
    //} elseif {[expr 0x$hex_check_status & 72]==0} {
    //   mtk_log error "ta1_mc is stopped unexpectedly, right after getting started!"
    //}

    if (type != TA43_OP_RUNQUIET)
    {
        u4addr = (DRAMC_BASE_ADDRESS | (TA_ADDR << POS_BANK_NUM) | (0xe38));
        u4value = u4IO32Read4B(u4addr);
        //mcSHOW_DBG_MSG(("0x%8x=0x%8x\n", u4addr, u4value));
    }

    //------------- check log 32-bit mode ---------------------------
    //---- error bits (first DQS rising)
    if (type != TA43_OP_RUNQUIET)
    {
        u4addr = (DRAMC_BASE_ADDRESS | (TA_ADDR << POS_BANK_NUM) | (0xe00));
        u4value = u4IO32Read4B(u4addr);
        //mcSHOW_DBG_MSG(("0x%8x=0x%8x\n", u4addr, u4value));
        u4addr += 4;
        u4value = u4IO32Read4B(u4addr);
        //mcSHOW_DBG_MSG(("0x%8x=0x%8x\n", u4addr, u4value));
        u4addr += 4;
        u4value = u4IO32Read4B(u4addr);
        //mcSHOW_DBG_MSG(("0x%8x=0x%8x\n", u4addr, u4value));
        u4addr += 4;
        u4value = u4IO32Read4B(u4addr);
        //mcSHOW_DBG_MSG(("0x%8x=0x%8x\n", u4addr, u4value));
    }
  
    //---- error bits (first DQS falling)
    //proc_read 0x20003e04
    //---- error bits (second DQS rising)
    //proc_read 0x20003e08
    //---- error bits (second DQS falling)
    //proc_read 0x20003e0c
    // read 0x20003e10
    // read 0x20003e14
    // read 0x20003e18
    // read 0x20003e1c

    //summary:
    //---- error bits (DQS rising)
    //proc_read 0x20003e20
    //---- error bits (DQS falling)
    //proc_read 0x20003e24

    if (type != TA43_OP_RUNQUIET)
    {
        u4addr = (DRAMC_BASE_ADDRESS | (TA_ADDR << POS_BANK_NUM) | (0xe38));
        u4value = u4IO32Read4B(u4addr);
        //mcSHOW_DBG_MSG(("0x%8x=0x%8x\n", u4addr, u4value));
        u4addr = (DRAMC_BASE_ADDRESS | (TA_ADDR << POS_BANK_NUM) | (0xe38));
        u4value = u4IO32Read4B(u4addr);
        //mcSHOW_DBG_MSG(("0x%8x=0x%8x\n", u4addr, u4value));
        u4addr = (DRAMC_BASE_ADDRESS | (TA_ADDR << POS_BANK_NUM) | (0xe38));
        u4value = u4IO32Read4B(u4addr);
        //mcSHOW_DBG_MSG(("0x%8x=0x%8x\n", u4addr, u4value));
    }

    u4addr = (DRAMC_BASE_ADDRESS | (TA_ADDR << POS_BANK_NUM) | (0xe00));
    u4value = u4IO32Read4B(u4addr);
    //mcSHOW_DBG_MSG(("0x%8x=0x%8x\n", u4addr, u4value));
    u4addr += 4;
    u4value = u4IO32Read4B(u4addr);
    //mcSHOW_DBG_MSG(("0x%8x=0x%8x\n", u4addr, u4value));
    u4addr += 4;
    u4value = u4IO32Read4B(u4addr);
    //mcSHOW_DBG_MSG(("0x%8x=0x%8x\n", u4addr, u4value));
    u4addr += 4;
    u4value = u4IO32Read4B(u4addr);
    //mcSHOW_DBG_MSG(("0x%8x=0x%8x\n", u4addr, u4value));

    if (p->channel == CHANNEL_A)
    {
        ta1_mc_running_cha = ENABLE;
    }
    else if (p->channel == CHANNEL_B)
    {
        ta1_mc_running_chb = ENABLE;
    }
    else
    {
        ta1_mc_running_chc = ENABLE;
    }
#endif
}

#if 0//A60501_TEST_CODE
void DramcTestPat4_3_Tmp(DRAMC_CTX_T *p)
{
#if fcFOR_CHIP_ID == fcA60501
U8 TA_ADDR, ucstatus = 0, ta1_mc_running;
U32 u4addr, u4value;
DRAM_TA_PATTERN_T last_testpat;

    if (p->channel == CHANNEL_A)
    {
        TA_ADDR = 10;
#if 0
        // 2012/09/27, only for A60807, bypass arbiter, 0120[3:0]=0010 for TA
        u4value = u4IO32Read4B(0x20000120, &u4value);
        mcSET_FIELD(u4value, 2, 0x0000000f, 0);
        ucstatus |= vIO32Write4B(0x20000120, u4value);
#endif
        last_testpat = last_testpat_cha;
        ta1_mc_running = ta1_mc_running_cha;	  
    }
    else if (p->channel == CHANNEL_B)
    {
        TA_ADDR = 11;
#if 0
        // 2012/09/27, only for A60807, bypass arbiter, 0120[7:4]=0010 for TA
        u4value = u4IO32Read4B(0x20000120, &u4value);
        mcSET_FIELD(u4value, 2, 0x000000f0, 4);
        ucstatus |= vIO32Write4B(0x20000120, u4value);
#endif
        last_testpat = last_testpat_chb;
        ta1_mc_running = ta1_mc_running_chb;	 
    }
    else
    {
        TA_ADDR = 12;
#if 0
        // 2012/09/27, only for A60807, bypass arbiter, 0120[7:4]=0010 for TA
        u4value = u4IO32Read4B(0x20000120, &u4value);
        mcSET_FIELD(u4value, 2, 0x000000f0, 4);
        ucstatus |= vIO32Write4B(0x20000120, u4value);
#endif
        last_testpat = last_testpat_chc;
        ta1_mc_running = ta1_mc_running_chc;	 
    }

	
	u4addr = (DRAMC_BASE_ADDRESS | (TA_ADDR << POS_BANK_NUM) | (0x400));
	ucstatus |= vIO32Write4B(u4addr, 0x108a0ae0);
	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x0fffffff);

	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x108a0ae4);
	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x7ffffff8);

	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x108a0ae8);
	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x70000000);

	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x108a0aec);
	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x00000007);

	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x108a0af0);
	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x601f1c1c);

	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x11f00000);
	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x11900000);
	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x11a01010);

	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x108a0e3c);
	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x00000000);

	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x101a0adc);
	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x108a0afc);
	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x03006048);
	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x00010000);

	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x108a0afc);
	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x03006040);
	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x21140101);
	
	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x0c923fe4);
	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x27740001);

	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x107a0e38);
	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0xffffff03);
	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x0c123fc4);

	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x108a0afc);
	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x03006000);
	u4addr +=4;
	ucstatus |= vIO32Write4B(u4addr, 0x00800000);
	
	u4addr = (DRAMC_BASE_ADDRESS | (TA_ADDR << POS_BANK_NUM) | (0xafc));
	ucstatus |= vIO32Write4B(u4addr, 0x00006140);

	u4addr = (DRAMC_BASE_ADDRESS | (TA_ADDR << POS_BANK_NUM) | (0x800));
	ucstatus |= vIO32Write4B(u4addr, 0xFFFFFFFF);

	while(u4addr!=(DRAMC_BASE_ADDRESS | (TA_ADDR << POS_BANK_NUM) | (0x98c)))
	{
		u4addr +=4;
		ucstatus |= vIO32Write4B(u4addr, 0xFFFFFFFF);
	}

	u4addr = (DRAMC_BASE_ADDRESS | (TA_ADDR << POS_BANK_NUM) | (0xe00));
	u4value = u4IO32Read4B(u4addr);
	mcSHOW_DBG_MSG(("0x%8x=0x%8x\n", u4addr, u4value));
	//mcFPRINTF((fp_A60501, "0x%8x=0x%8x\n", u4addr, u4value));
	u4addr += 4;
	u4value = u4IO32Read4B(u4addr);
	mcSHOW_DBG_MSG(("0x%8x=0x%8x\n", u4addr, u4value));
	//mcFPRINTF((fp_A60501, "0x%8x=0x%8x\n", u4addr, u4value));
	u4addr += 4;
	u4value = u4IO32Read4B(u4addr);
	mcSHOW_DBG_MSG(("0x%8x=0x%8x\n", u4addr, u4value));
	//mcFPRINTF((fp_A60501, "0x%8x=0x%8x\n", u4addr, u4value));
	u4addr += 4;
	u4value = u4IO32Read4B(u4addr);
	mcSHOW_DBG_MSG(("0x%8x=0x%8x\n", u4addr, u4value));
	//mcFPRINTF((fp_A60501, "0x%8x=0x%8x\n", u4addr, u4value));
		
#endif
}
#endif

//-------------------------------------------------------------------------
/** DramcPat4_3
 *  Run TA test pattern 4_3 and return result
 *  @param p                Pointer of context created by DramcCtxCreate.
 *  @retval status          (U32): return the value of ERR_VALUE, 0 is ok, others mean  error
 */
//-------------------------------------------------------------------------
U32 DramcPat4_3(DRAMC_CTX_T *p)
{
#if fcFOR_CHIP_ID == fcA60501
U32 u4addr, u4value, u4err_value = 0;
U8 ii, TA_ADDR, ucstatus = 0;

    if (p->channel == CHANNEL_A)
    {
        TA_ADDR = 10;
    }
    else if (p->channel == CHANNEL_B)
    {
        TA_ADDR = 11;
    }
    else
    {
        TA_ADDR = 12;
    }
    
    // stop
    DramcTestPat4_3(p, TA43_OP_STOP);

    // run quiet
    DramcTestPat4_3(p, TA43_OP_RUNQUIET);
    //DramcTestPat4_3(p, TA43_OP_RUN);

    // clear
    DramcTestPat4_3(p, TA43_OP_CLEAR);

    mcDELAY_MS(50);

    u4addr = (DRAMC_BASE_ADDRESS | (TA_ADDR << POS_BANK_NUM) | (0xe00));
    for (ii = 0; ii < 4; ii++)
    {
        u4value = u4IO32Read4B(u4addr);
        p->ta43_result[ii] = u4value;
        u4addr += 4;
    }
    #if A60501_TA43_WA
    for (ii = 2; ii < 4; ii++)
    #else
    for (ii = 0; ii < 4; ii++)
    #endif
    {
        u4err_value |= (p->ta43_result[ii]);
    }

    // stop
    DramcTestPat4_3(p, TA43_OP_STOP);

    return u4err_value;
#else
    return 0;
#endif
}

#if 0
void DramcGeneralPat_mem(DRAMC_CTX_T *p, U32 src_addr)
{
    U32 uPat, ii;
    mcSHOW_DBG_MSG(("START to write general pattern to memory...\n"));

    for (ii =0 ; ii < (3584/4); ii++)
    {
        uPat = ii & 0xff;
        if (uPat==0xff) {
        //	uPat = 0x5a;
        }
        ucDram_Write(src_addr, uPat + (uPat<<8) + (uPat<<16) + (uPat<<24));
        src_addr +=4;
    }
    mcSHOW_DBG_MSG(("END to write general pattern to memory!\n"));
}
#endif

U32 DramcPat4_3_stress(DRAMC_CTX_T *p)
{
#if fcFOR_CHIP_ID == fcA60501
U32 u4addr, u4value, u4err_value = 0;
U8 ii, TA_ADDR, ucstatus = 0;

    if (p->channel == CHANNEL_A)
    {
        TA_ADDR = 10;
    }
    else if (p->channel == CHANNEL_B)
    {
        TA_ADDR = 11;
    }
    else
    {
        TA_ADDR = 12;
    }
    
    // stop
    DramcTestPat4_3(p, TA43_OP_STOP);

    // run quiet
    DramcTestPat4_3(p, TA43_OP_RUNQUIET);
    //DramcTestPat4_3(p, TA43_OP_RUN);

    // clear
    DramcTestPat4_3(p, TA43_OP_CLEAR);

    mcDELAY_MS(600);
    //mcDELAY_MS(25);

    u4addr = (DRAMC_BASE_ADDRESS | (TA_ADDR << POS_BANK_NUM) | (0xe00));
    for (ii = 0; ii < 4; ii++)
    {
        u4value = u4IO32Read4B(u4addr);
        p->ta43_result[ii] = u4value;
        u4addr += 4;
    }

    for (ii = 0; ii < 4; ii++)
    {
        u4err_value |= (p->ta43_result[ii]);
    }

    // stop
    DramcTestPat4_3(p, TA43_OP_STOP);
    
    return u4err_value;
#else
    return 0;
#endif

}


void DramcTest_DualSch_stress(DRAMC_CTX_T *p)
{
	U32 count = 0;
	vIO32WriteFldAlign_All(DRAMC_REG_PERFCTL0, 1, PERFCTL0_DUALSCHEN);
	while(count<10)
	{
		count++;
		U8 u1Value = 0;
		DramcModeRegWrite(p, 12, 0x14);
		DramcModeRegRead(p, 12, &u1Value);
		//mcSHOW_DBG_MSG(("MR12 = 0x%0X\n", u1Value));
	}
}


void DramcModeRegRead(DRAMC_CTX_T *p, U8 u1MRIdx, U8 *u1pValue)
{
    U32 u4MRValue;

    //vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_MRS), p->rank, MRS_MRRRK);  
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_MRS), u1MRIdx, MRS_MRSMA);   

    // MRR command will be fired when MRREN 0->1
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 1, SPCMD_MRREN);    

    // wait MRR command fired.
    while(u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMDRESP), SPCMDRESP_MRR_RESPONSE) ==0)
    {
        mcDELAY_US(1);
    }

    // Read out mode register value
    u4MRValue = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMDRESP), SPCMDRESP_MRR_REG);
    *u1pValue = (U8)u4MRValue;   

    // Set MRREN =0 for next time MRR.
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 0, SPCMD_MRREN);
    
    mcSHOW_DBG_MSG3(("Read MR%d =0x%x,", u1MRIdx, u4MRValue));
}


void DramcModeRegReadByRank(DRAMC_CTX_T *p, U8 u1Rank, U8 u1MRIdx, U8 *u1pValue)
{
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_MRS), u1Rank, MRS_MRRRK);
    DramcModeRegRead(p,u1MRIdx, u1pValue);
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_MRS), 0, MRS_MRRRK);
}


void DramcModeRegWrite(DRAMC_CTX_T *p, U8 u1MRIdx, U8 u1Value)
{
    //vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_MRS), p->rank, MRS_MRSRK);
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_MRS), u1MRIdx, MRS_MRSMA);
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_MRS), u1Value, MRS_MRSOP);

    // MRW command will be fired when MRWEN 0->1  
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 1, SPCMD_MRWEN);
    
    // wait MRW command fired.
    while(u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMDRESP), SPCMDRESP_MRW_RESPONSE) ==0)
    {
        mcSHOW_DBG_MSG2(("wait MRW command fired\n"));
        mcDELAY_US(1);
    }
    
    // Set MRWEN =0 for next time MRW.
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 0, SPCMD_MRWEN);

    mcSHOW_DBG_MSG3(("Write MR%d =0x%x\n", u1MRIdx, u1Value));
    //mcFPRINTF((fp_A60501, "Write MR%d=%x,", u1MRIdx, u1Value));
}


#if 0
void LP4_ESD_GPIOTest(DRAMC_CTX_T *p)
{
#if fcFOR_CHIP_ID == fcA60501

    U32 u4Result;
    vIO32WriteFldAlign(mcSET_SYS_REG_ADDR(0x10 <<2), 1, PADCTL4_CKEBYCTL);
    //vIO32Write4B(mcSET_SYS_REG_ADDR(0x12 <<2), 0xc030000c);// GPIO[3:2] output pin enable, GPIO[5:4] and GPIO[15:14] input pin enable.
    //vIO32Write4B(mcSET_SYS_REG_ADDR(0x12 <<2), 0xffffc00c);
    
    vIO32Write4B(mcSET_SYS_REG_ADDR(0x11 <<2), 0);// GPIO[3:2] =00
    u4Result = u4IO32Read4B(mcSET_SYS_REG_ADDR(0x13 <<2));
    #ifdef ETT_PRINT_FORMAT
    mcSHOW_DBG_MSG(("LP4_ESD_GPIOTest (0)  = 0x%B, ", u4Result));
    #else
    mcSHOW_DBG_MSG(("LP4_ESD_GPIOTest (0)  = 0x%2x, ", u4Result));
    #endif
    if(u4Result ==0) 
        mcSHOW_DBG_MSG(("Pass\n"));
    else
        mcSHOW_DBG_MSG(("Fail\n"));

    vIO32Write4B(mcSET_SYS_REG_ADDR(0x11 <<2),  0x1 <<2);// GPIO[3:2] =01
    u4Result = u4IO32Read4B(mcSET_SYS_REG_ADDR(0x13 <<2));
    #ifdef ETT_PRINT_FORMAT
    mcSHOW_DBG_MSG(("LP4_ESD_GPIOTest (1)  = 0x%B, ", u4Result));
    #else
    mcSHOW_DBG_MSG(("LP4_ESD_GPIOTest (1)  = 0x%2x, ", u4Result));
    #endif
    if(u4Result ==0x14) 
        mcSHOW_DBG_MSG(("Pass\n"));
    else
        mcSHOW_DBG_MSG(("Fail\n"));


    vIO32Write4B(mcSET_SYS_REG_ADDR(0x11 <<2),  0x2 <<2);// GPIO[3:2] =10
    u4Result = u4IO32Read4B(mcSET_SYS_REG_ADDR(0x13 <<2));
    #ifdef ETT_PRINT_FORMAT
    mcSHOW_DBG_MSG(("LP4_ESD_GPIOTest (2)  = 0x%B, ", u4Result));
    #else
    mcSHOW_DBG_MSG(("LP4_ESD_GPIOTest (2)  = 0x%2x, ", u4Result));
    #endif
    if(u4Result ==0x28) 
        mcSHOW_DBG_MSG(("Pass\n"));
    else
        mcSHOW_DBG_MSG(("Fail\n"));

    vIO32Write4B(mcSET_SYS_REG_ADDR(0x11 <<2), 0x3 <<2);// GPIO[3:2] =11
    u4Result = u4IO32Read4B(mcSET_SYS_REG_ADDR(0x13 <<2));
    #ifdef ETT_PRINT_FORMAT
    mcSHOW_DBG_MSG(("LP4_ESD_GPIOTest (3)  = 0x%B, ", u4Result));
    #else
    mcSHOW_DBG_MSG(("LP4_ESD_GPIOTest (3)  = 0x%2x, ", u4Result));
    #endif
    if(u4Result ==0x3c) 
        mcSHOW_DBG_MSG(("Pass\n"));
    else
        mcSHOW_DBG_MSG(("Fail\n"));

#endif
}
#endif

#if DUAL_FREQ_K

#define FREQREG_SIZE 139
#define FREQREG_SIZE_PHY 62
#define FREQREG_SIZE_DRAMC 77

typedef struct _SHUFFLE_REG
{
	U32 uiNormalAddr;
	U32 uiShuffle1Addr;
	U32 uiShuffle2Addr;
} SHUFFLE_REG;

typedef struct DFS_SW_BACKUP_REG
{
	U32 Dramc_Backup[CHANNEL_MAX][FREQREG_SIZE_DRAMC];
	U32 PHY_Backup[DDRPHY_CONF_MAX][FREQREG_SIZE_PHY];
	U32 PHY_Backup06A[DDRPHY_CONF_MAX];
	U32 PHY_Backup107[DDRPHY_CONF_MAX];
} DFS_SW_BACKUP_REG_T;


//DRAMC : 77, PHY per channel : 62
const SHUFFLE_REG ShuffleRegTable[FREQREG_SIZE] =
{
    {DRAMC_AO_BASE_ADDRESS+(0x000<<2), DRAMC_REG_SHUFFLE00 , DRAMC_REG_THRD_SHUFFLE00 },
    {DRAMC_AO_BASE_ADDRESS+(0x001<<2), DRAMC_REG_SHUFFLE01 , DRAMC_REG_THRD_SHUFFLE01 },
    {DRAMC_AO_BASE_ADDRESS+(0x002<<2), DRAMC_REG_SHUFFLE02 , DRAMC_REG_THRD_SHUFFLE02 },
    {DRAMC_AO_BASE_ADDRESS+(0x07C<<2), DRAMC_REG_SHUFFLE7C , DRAMC_REG_THRD_SHUFFLE7C },
    {DRAMC_AO_BASE_ADDRESS+(0x07E<<2), DRAMC_REG_SHUFFLE7E , DRAMC_REG_THRD_SHUFFLE7E },
    {DRAMC_AO_BASE_ADDRESS+(0x044<<2), DRAMC_REG_SHUFFLE44 , DRAMC_REG_THRD_SHUFFLE44 },
    {DRAMC_AO_BASE_ADDRESS+(0x006<<2), DRAMC_REG_SHUFFLE06 , DRAMC_REG_THRD_SHUFFLE06 },
    {DRAMC_AO_BASE_ADDRESS+(0x007<<2), DRAMC_REG_SHUFFLE07 , DRAMC_REG_THRD_SHUFFLE07 },
    {DRAMC_AO_BASE_ADDRESS+(0x011<<2), DRAMC_REG_SHUFFLE11 , DRAMC_REG_THRD_SHUFFLE11 },
    {DRAMC_AO_BASE_ADDRESS+(0x01F<<2), DRAMC_REG_SHUFFLE1F , DRAMC_REG_THRD_SHUFFLE1F },
    {DRAMC_AO_BASE_ADDRESS+(0x077<<2), DRAMC_REG_SHUFFLE77 , DRAMC_REG_THRD_SHUFFLE77 },
    {DRAMC_AO_BASE_ADDRESS+(0x07A<<2), DRAMC_REG_SHUFFLE7A , DRAMC_REG_THRD_SHUFFLE7A },
    {DRAMC_AO_BASE_ADDRESS+(0x03F<<2), DRAMC_REG_SHUFFLE3F , DRAMC_REG_THRD_SHUFFLE3F },
    {DRAMC_AO_BASE_ADDRESS+(0x023<<2), DRAMC_REG_SHUFFLE23 , DRAMC_REG_THRD_SHUFFLE23 },
    {DRAMC_AO_BASE_ADDRESS+(0x020<<2), DRAMC_REG_SHUFFLE20 , DRAMC_REG_THRD_SHUFFLE20 },
    {DRAMC_AO_BASE_ADDRESS+(0x038<<2), DRAMC_REG_SHUFFLE38 , DRAMC_REG_THRD_SHUFFLE38 },
    {DRAMC_AO_BASE_ADDRESS+(0x025<<2), DRAMC_REG_SHUFFLE25 , DRAMC_REG_THRD_SHUFFLE25 },
    {DRAMC_AO_BASE_ADDRESS+(0x026<<2), DRAMC_REG_SHUFFLE26 , DRAMC_REG_THRD_SHUFFLE26 },
    {DRAMC_AO_BASE_ADDRESS+(0x039<<2), DRAMC_REG_SHUFFLE39 , DRAMC_REG_THRD_SHUFFLE39 },
    {DRAMC_AO_BASE_ADDRESS+(0x046<<2), DRAMC_REG_SHUFFLE46 , DRAMC_REG_THRD_SHUFFLE46 },
    {DRAMC_AO_BASE_ADDRESS+(0x071<<2), DRAMC_REG_SHUFFLE71 , DRAMC_REG_THRD_SHUFFLE71 },
    {DRAMC_AO_BASE_ADDRESS+(0x04E<<2), DRAMC_REG_SHUFFLE4E , DRAMC_REG_THRD_SHUFFLE4E },
    {DRAMC_AO_BASE_ADDRESS+(0x049<<2), DRAMC_REG_SHUFFLE49 , DRAMC_REG_THRD_SHUFFLE49 },
    {DRAMC_AO_BASE_ADDRESS+(0x012<<2), DRAMC_REG_SHUFFLE12 , DRAMC_REG_THRD_SHUFFLE12 },
    {DRAMC_AO_BASE_ADDRESS+(0x100<<2), DRAMC_REG_SHUFFLE100, DRAMC_REG_THRD_SHUFFLE100},
    {DRAMC_AO_BASE_ADDRESS+(0x101<<2), DRAMC_REG_SHUFFLE101, DRAMC_REG_THRD_SHUFFLE101},
    {DRAMC_AO_BASE_ADDRESS+(0x102<<2), DRAMC_REG_SHUFFLE102, DRAMC_REG_THRD_SHUFFLE102},
    {DRAMC_AO_BASE_ADDRESS+(0x103<<2), DRAMC_REG_SHUFFLE103, DRAMC_REG_THRD_SHUFFLE103},
    {DRAMC_AO_BASE_ADDRESS+(0x104<<2), DRAMC_REG_SHUFFLE104, DRAMC_REG_THRD_SHUFFLE104},
    {DRAMC_AO_BASE_ADDRESS+(0x105<<2), DRAMC_REG_SHUFFLE105, DRAMC_REG_THRD_SHUFFLE105},
    {DRAMC_AO_BASE_ADDRESS+(0x106<<2), DRAMC_REG_SHUFFLE106, DRAMC_REG_THRD_SHUFFLE106},
    {DRAMC_AO_BASE_ADDRESS+(0x107<<2), DRAMC_REG_SHUFFLE107, DRAMC_REG_THRD_SHUFFLE107},
    {DRAMC_AO_BASE_ADDRESS+(0x108<<2), DRAMC_REG_SHUFFLE108, DRAMC_REG_THRD_SHUFFLE108},
    {DRAMC_AO_BASE_ADDRESS+(0x109<<2), DRAMC_REG_SHUFFLE109, DRAMC_REG_THRD_SHUFFLE109},
    {DRAMC_AO_BASE_ADDRESS+(0x10A<<2), DRAMC_REG_SHUFFLE10A, DRAMC_REG_THRD_SHUFFLE10A},
    {DRAMC_AO_BASE_ADDRESS+(0x10B<<2), DRAMC_REG_SHUFFLE10B, DRAMC_REG_THRD_SHUFFLE10B},
    {DRAMC_AO_BASE_ADDRESS+(0x070<<2), DRAMC_REG_SHUFFLE70 , DRAMC_REG_THRD_SHUFFLE70 },
    {DRAMC_AO_BASE_ADDRESS+(0x072<<2), DRAMC_REG_SHUFFLE72 , DRAMC_REG_THRD_SHUFFLE72 },
    {DRAMC_AO_BASE_ADDRESS+(0x115<<2), DRAMC_REG_SHUFFLE115, DRAMC_REG_THRD_SHUFFLE115},
    {DRAMC_AO_BASE_ADDRESS+(0x116<<2), DRAMC_REG_SHUFFLE116, DRAMC_REG_THRD_SHUFFLE116},
    {DRAMC_AO_BASE_ADDRESS+(0x084<<2), DRAMC_REG_SHUFFLE84 , DRAMC_REG_THRD_SHUFFLE84 },
    {DRAMC_AO_BASE_ADDRESS+(0x085<<2), DRAMC_REG_SHUFFLE85 , DRAMC_REG_THRD_SHUFFLE85 },
    {DRAMC_AO_BASE_ADDRESS+(0x086<<2), DRAMC_REG_SHUFFLE86 , DRAMC_REG_THRD_SHUFFLE86 },
    {DRAMC_AO_BASE_ADDRESS+(0x087<<2), DRAMC_REG_SHUFFLE87 , DRAMC_REG_THRD_SHUFFLE87 },
    {DRAMC_AO_BASE_ADDRESS+(0x088<<2), DRAMC_REG_SHUFFLE88 , DRAMC_REG_THRD_SHUFFLE88 },
    {DRAMC_AO_BASE_ADDRESS+(0x089<<2), DRAMC_REG_SHUFFLE89 , DRAMC_REG_THRD_SHUFFLE89 },
    {DRAMC_AO_BASE_ADDRESS+(0x08A<<2), DRAMC_REG_SHUFFLE8A , DRAMC_REG_THRD_SHUFFLE8A },
    {DRAMC_AO_BASE_ADDRESS+(0x08B<<2), DRAMC_REG_SHUFFLE8B , DRAMC_REG_THRD_SHUFFLE8B },
    {DRAMC_AO_BASE_ADDRESS+(0x10C<<2), DRAMC_REG_SHUFFLE10C, DRAMC_REG_THRD_SHUFFLE10C},
    {DRAMC_AO_BASE_ADDRESS+(0x10D<<2), DRAMC_REG_SHUFFLE10D, DRAMC_REG_THRD_SHUFFLE10D},
    {DRAMC_AO_BASE_ADDRESS+(0x10E<<2), DRAMC_REG_SHUFFLE10E, DRAMC_REG_THRD_SHUFFLE10E},
    {DRAMC_AO_BASE_ADDRESS+(0x10F<<2), DRAMC_REG_SHUFFLE10F, DRAMC_REG_THRD_SHUFFLE10F},
    {DRAMC_AO_BASE_ADDRESS+(0x110<<2), DRAMC_REG_SHUFFLE110, DRAMC_REG_THRD_SHUFFLE110},
    {DRAMC_AO_BASE_ADDRESS+(0x111<<2), DRAMC_REG_SHUFFLE111, DRAMC_REG_THRD_SHUFFLE111},
    {DRAMC_AO_BASE_ADDRESS+(0x112<<2), DRAMC_REG_SHUFFLE112, DRAMC_REG_THRD_SHUFFLE112},
    {DRAMC_AO_BASE_ADDRESS+(0x113<<2), DRAMC_REG_SHUFFLE113, DRAMC_REG_THRD_SHUFFLE113},
    {DRAMC_AO_BASE_ADDRESS+(0x08D<<2), DRAMC_REG_SHUFFLE8D , DRAMC_REG_THRD_SHUFFLE8D },
    {DRAMC_AO_BASE_ADDRESS+(0x08E<<2), DRAMC_REG_SHUFFLE8E , DRAMC_REG_THRD_SHUFFLE8E },
    {DRAMC_AO_BASE_ADDRESS+(0x08F<<2), DRAMC_REG_SHUFFLE8F , DRAMC_REG_THRD_SHUFFLE8F },
    {DRAMC_AO_BASE_ADDRESS+(0x08c<<2), DRAMC_REG_SHUFFLE8C , DRAMC_REG_THRD_SHUFFLE8C },
    {DRAMC_AO_BASE_ADDRESS+(0x083<<2), DRAMC_REG_SHUFFLE83 , DRAMC_REG_THRD_SHUFFLE83 },
    {DRAMC_AO_BASE_ADDRESS+(0x082<<2), DRAMC_REG_SHUFFLE82 , DRAMC_REG_THRD_SHUFFLE82 },
    {DRAMC_AO_BASE_ADDRESS+(0x032<<2), DRAMC_REG_SHUFFLE32 , DRAMC_REG_THRD_SHUFFLE32 },
    {DRAMC_AO_BASE_ADDRESS+(0x140<<2), DRAMC_REG_SHUFFLE140, DRAMC_REG_THRD_SHUFFLE140},
    {DRAMC_AO_BASE_ADDRESS+(0x141<<2), DRAMC_REG_SHUFFLE141, DRAMC_REG_THRD_SHUFFLE141},
    {DRAMC_AO_BASE_ADDRESS+(0x142<<2), DRAMC_REG_SHUFFLE142, DRAMC_REG_THRD_SHUFFLE142},
    {DRAMC_AO_BASE_ADDRESS+(0x143<<2), DRAMC_REG_SHUFFLE143, DRAMC_REG_THRD_SHUFFLE143},
    {DRAMC_AO_BASE_ADDRESS+(0x144<<2), DRAMC_REG_SHUFFLE144, DRAMC_REG_THRD_SHUFFLE144},
    {DRAMC_AO_BASE_ADDRESS+(0x145<<2), DRAMC_REG_SHUFFLE145, DRAMC_REG_THRD_SHUFFLE145},
    {DRAMC_AO_BASE_ADDRESS+(0x146<<2), DRAMC_REG_SHUFFLE146, DRAMC_REG_THRD_SHUFFLE146},
    {DRAMC_AO_BASE_ADDRESS+(0x147<<2), DRAMC_REG_SHUFFLE147, DRAMC_REG_THRD_SHUFFLE147},
    {DRAMC_AO_BASE_ADDRESS+(0x150<<2), DRAMC_REG_SHUFFLE150, DRAMC_REG_THRD_SHUFFLE150},
    {DRAMC_AO_BASE_ADDRESS+(0x151<<2), DRAMC_REG_SHUFFLE151, DRAMC_REG_THRD_SHUFFLE151},
    {DRAMC_AO_BASE_ADDRESS+(0x152<<2), DRAMC_REG_SHUFFLE152, DRAMC_REG_THRD_SHUFFLE152},
    {DRAMC_AO_BASE_ADDRESS+(0x153<<2), DRAMC_REG_SHUFFLE153, DRAMC_REG_THRD_SHUFFLE153},
    {DRAMC_AO_BASE_ADDRESS+(0x154<<2), DRAMC_REG_SHUFFLE154, DRAMC_REG_THRD_SHUFFLE154},
    {DRAMC_AO_BASE_ADDRESS+(0x155<<2), DRAMC_REG_SHUFFLE155, DRAMC_REG_THRD_SHUFFLE155},

    {DDRPHY_BASE_ADDR+(0x100<<2), DDRPHY_SHUFFLE100  , DDRPHY_THRD_SHUFFLE100 },
    {DDRPHY_BASE_ADDR+(0x102<<2), DDRPHY_SHUFFLE102  , DDRPHY_THRD_SHUFFLE102 },
    {DDRPHY_BASE_ADDR+(0x09f<<2), DDRPHY_SHUFFLE9F   , DDRPHY_THRD_SHUFFLE9F  },
    {DDRPHY_BASE_ADDR+(0x103<<2), DDRPHY_SHUFFLE103  , DDRPHY_THRD_SHUFFLE103 },
    {DDRPHY_BASE_ADDR+(0x105<<2), DDRPHY_SHUFFLE105  , DDRPHY_THRD_SHUFFLE105 },                 
    {DDRPHY_BASE_ADDR+(0x003<<2), DDRPHY_SHUFFLE03   , DDRPHY_THRD_SHUFFLE03  },
    {DDRPHY_BASE_ADDR+(0x004<<2), DDRPHY_SHUFFLE04   , DDRPHY_THRD_SHUFFLE04  },
    {DDRPHY_BASE_ADDR+(0x009<<2), DDRPHY_SHUFFLE09   , DDRPHY_THRD_SHUFFLE09  },
    {DDRPHY_BASE_ADDR+(0x00a<<2), DDRPHY_SHUFFLE0A   , DDRPHY_THRD_SHUFFLE0A  },
    {DDRPHY_BASE_ADDR+(0x00b<<2), DDRPHY_SHUFFLE0B   , DDRPHY_THRD_SHUFFLE0B  },
    {DDRPHY_BASE_ADDR+(0x00c<<2), DDRPHY_SHUFFLE0C   , DDRPHY_THRD_SHUFFLE0C  },
    {DDRPHY_BASE_ADDR+(0x00d<<2), DDRPHY_SHUFFLE0D   , DDRPHY_THRD_SHUFFLE0D  },
    {DDRPHY_BASE_ADDR+(0x00e<<2), DDRPHY_SHUFFLE0E   , DDRPHY_THRD_SHUFFLE0E  },
    {DDRPHY_BASE_ADDR+(0x00f<<2), DDRPHY_SHUFFLE0F   , DDRPHY_THRD_SHUFFLE0F  },
    {DDRPHY_BASE_ADDR+(0x010<<2), DDRPHY_SHUFFLE10   , DDRPHY_THRD_SHUFFLE10  },
    {DDRPHY_BASE_ADDR+(0x011<<2), DDRPHY_SHUFFLE11   , DDRPHY_THRD_SHUFFLE11  },
    {DDRPHY_BASE_ADDR+(0x012<<2), DDRPHY_SHUFFLE12   , DDRPHY_THRD_SHUFFLE12  },
    {DDRPHY_BASE_ADDR+(0x013<<2), DDRPHY_SHUFFLE13   , DDRPHY_THRD_SHUFFLE13  },
    {DDRPHY_BASE_ADDR+(0x014<<2), DDRPHY_SHUFFLE14   , DDRPHY_THRD_SHUFFLE14  },
    {DDRPHY_BASE_ADDR+(0x015<<2), DDRPHY_SHUFFLE15   , DDRPHY_THRD_SHUFFLE15  },
    {DDRPHY_BASE_ADDR+(0x016<<2), DDRPHY_SHUFFLE16   , DDRPHY_THRD_SHUFFLE16  },
    {DDRPHY_BASE_ADDR+(0x01b<<2), DDRPHY_SHUFFLE1B   , DDRPHY_THRD_SHUFFLE1B  },
    {DDRPHY_BASE_ADDR+(0x01c<<2), DDRPHY_SHUFFLE1C   , DDRPHY_THRD_SHUFFLE1C  },
    {DDRPHY_BASE_ADDR+(0x021<<2), DDRPHY_SHUFFLE21   , DDRPHY_THRD_SHUFFLE21  },
    {DDRPHY_BASE_ADDR+(0x022<<2), DDRPHY_SHUFFLE22   , DDRPHY_THRD_SHUFFLE22  },
    {DDRPHY_BASE_ADDR+(0x023<<2), DDRPHY_SHUFFLE23   , DDRPHY_THRD_SHUFFLE23  },
    {DDRPHY_BASE_ADDR+(0x024<<2), DDRPHY_SHUFFLE24   , DDRPHY_THRD_SHUFFLE24  },
    {DDRPHY_BASE_ADDR+(0x025<<2), DDRPHY_SHUFFLE25   , DDRPHY_THRD_SHUFFLE25  },
    {DDRPHY_BASE_ADDR+(0x026<<2), DDRPHY_SHUFFLE26   , DDRPHY_THRD_SHUFFLE26  },
    {DDRPHY_BASE_ADDR+(0x027<<2), DDRPHY_SHUFFLE27   , DDRPHY_THRD_SHUFFLE27  },
    {DDRPHY_BASE_ADDR+(0x028<<2), DDRPHY_SHUFFLE28   , DDRPHY_THRD_SHUFFLE28  },
    {DDRPHY_BASE_ADDR+(0x029<<2), DDRPHY_SHUFFLE29   , DDRPHY_THRD_SHUFFLE29  },
    {DDRPHY_BASE_ADDR+(0x02a<<2), DDRPHY_SHUFFLE2A   , DDRPHY_THRD_SHUFFLE2A  },
    {DDRPHY_BASE_ADDR+(0x02b<<2), DDRPHY_SHUFFLE2B   , DDRPHY_THRD_SHUFFLE2B  },
    {DDRPHY_BASE_ADDR+(0x02c<<2), DDRPHY_SHUFFLE2C   , DDRPHY_THRD_SHUFFLE2C  },
    {DDRPHY_BASE_ADDR+(0x02d<<2), DDRPHY_SHUFFLE2D   , DDRPHY_THRD_SHUFFLE2D  },
    {DDRPHY_BASE_ADDR+(0x02e<<2), DDRPHY_SHUFFLE2E   , DDRPHY_THRD_SHUFFLE2E  },
    {DDRPHY_BASE_ADDR+(0x000<<2), DDRPHY_SHUFFLE00   , DDRPHY_THRD_SHUFFLE00  },
    {DDRPHY_BASE_ADDR+(0x002<<2), DDRPHY_SHUFFLE02   , DDRPHY_THRD_SHUFFLE02  },
    {DDRPHY_BASE_ADDR+(0x018<<2), DDRPHY_SHUFFLE18   , DDRPHY_THRD_SHUFFLE18  },
    {DDRPHY_BASE_ADDR+(0x01a<<2), DDRPHY_SHUFFLE1A   , DDRPHY_THRD_SHUFFLE1A  },
    {DDRPHY_BASE_ADDR+(0x040<<2), DDRPHY_SHUFFLE40   , DDRPHY_THRD_SHUFFLE40  },
    {DDRPHY_BASE_ADDR+(0x041<<2), DDRPHY_SHUFFLE41   , DDRPHY_THRD_SHUFFLE41  },
    {DDRPHY_BASE_ADDR+(0x061<<2), DDRPHY_SHUFFLE61   , DDRPHY_THRD_SHUFFLE61  },
    {DDRPHY_BASE_ADDR+(0x062<<2), DDRPHY_SHUFFLE62   , DDRPHY_THRD_SHUFFLE62  },
    {DDRPHY_BASE_ADDR+(0x063<<2), DDRPHY_SHUFFLE63   , DDRPHY_THRD_SHUFFLE63  },
    {DDRPHY_BASE_ADDR+(0x064<<2), DDRPHY_SHUFFLE64   , DDRPHY_THRD_SHUFFLE64  },
    {DDRPHY_BASE_ADDR+(0x065<<2), DDRPHY_SHUFFLE65   , DDRPHY_THRD_SHUFFLE65  },
    {DDRPHY_BASE_ADDR+(0x066<<2), DDRPHY_SHUFFLE66   , DDRPHY_THRD_SHUFFLE66  },
    {DDRPHY_BASE_ADDR+(0x067<<2), DDRPHY_SHUFFLE67   , DDRPHY_THRD_SHUFFLE67  },
    {DDRPHY_BASE_ADDR+(0x06a<<2), DDRPHY_SHUFFLE6A   , DDRPHY_THRD_SHUFFLE6A  },
    {DDRPHY_BASE_ADDR+(0x06b<<2), DDRPHY_SHUFFLE6B   , DDRPHY_THRD_SHUFFLE6B  },
    {DDRPHY_BASE_ADDR+(0x106<<2), DDRPHY_SHUFFLE106  , DDRPHY_THRD_SHUFFLE106 },
    {DDRPHY_BASE_ADDR+(0x116<<2), DDRPHY_SHUFFLE116  , DDRPHY_THRD_SHUFFLE116 },
    {DDRPHY_BASE_ADDR+(0x117<<2), DDRPHY_SHUFFLE117  , DDRPHY_THRD_SHUFFLE117 },
    {DDRPHY_BASE_ADDR+(0x118<<2), DDRPHY_SHUFFLE118  , DDRPHY_THRD_SHUFFLE118 },
    {DDRPHY_BASE_ADDR+(0x119<<2), DDRPHY_SHUFFLE119  , DDRPHY_THRD_SHUFFLE119 },
    {DDRPHY_BASE_ADDR+(0x11a<<2), DDRPHY_SHUFFLE11A  , DDRPHY_THRD_SHUFFLE11A },
    {DDRPHY_BASE_ADDR+(0x13d<<2), DDRPHY_SHUFFLE13D  , DDRPHY_THRD_SHUFFLE13D },
    {DDRPHY_BASE_ADDR+(0x13e<<2), DDRPHY_SHUFFLE13E  , DDRPHY_THRD_SHUFFLE13E },
    {DDRPHY_BASE_ADDR+(0x13f<<2), DDRPHY_SHUFFLE13F  , DDRPHY_THRD_SHUFFLE13F },
    {DDRPHY_BASE_ADDR+(0x10b<<2), DDRPHY_PLL19       , DDRPHY_THRD_SHUFFLE10B },
};


#if REG_SHUFFLE_REG_CHECK
void ShuffleRegCheckProgram(U32 u4Addr)
{
	U32 Offset, TmpAddr;
	
	if((u4Addr >= DRAMC_AO_BASE_ADDRESS)  && (u4Addr < DDRPHY_BASE_ADDR))
	{
		TmpAddr = (DRAMC_AO_BASE_ADDRESS | (u4Addr &0xffff));
	}
	else
	{
		TmpAddr = (DDRPHY_BASE_ADDR | (u4Addr &0xffff));
	}

    	for (Offset = 0; Offset < FREQREG_SIZE; Offset++)
    	{
    		if(TmpAddr ==LowFreq_ShuffleReg[Offset].uiSourceAddr)
    		{
    		     	mcSHOW_DBG_MSG(("\n[ShuffleRegCheck]  OK 0x%x \n",u4Addr));
    			mcFPRINTF((fp_A60501, "\n[ShuffleRegCheck]  OK 0x%x\n",u4Addr));
    			break;
    		}
    	}

    	if(Offset ==FREQREG_SIZE)
    	{
       	mcSHOW_DBG_MSG(("\n[ShuffleRegCheck]  Not in shuffle 0x%x \n",u4Addr));
    		mcFPRINTF((fp_A60501, "\n[ShuffleRegCheck]  Not in shuffle 0x%x\n",u4Addr));
    	}		
}
#endif


#if __ETT__
DFS_SW_BACKUP_REG_T swDfsBackupReg, swDfsBackupRegTmp;

//static U32 FreqReg_Backup[FREQREG_SIZE];
//static U32 FreqReg_Backup_LP3_CHA[FREQREG_SIZE];
//static U32 FreqReg_Backup_LP3_CHB[FREQREG_SIZE];

//sourceRG: Normal=0, shuffle1=1, shuffle2=2
void DramcSaveToBackup(DRAMC_CTX_T *p, DRAM_DFS_SHUFFLE_TYPE_T sourceRG, DFS_SW_BACKUP_REG_T *dst)
{
    U32 Offset;
    U32 i;
    mcSHOW_DBG_MSG(("Save  frequency registers setting into backup. \n"));

    //DRAMC
    for(i=0; i<CHANNEL_MAX; i++)
    {
        for (Offset = 0; Offset < FREQREG_SIZE_DRAMC; Offset++)
        {
            U32 addr;
            if(sourceRG==DRAM_DFS_SHUFFLE_NORMAL)
                addr = ShuffleRegTable[Offset].uiNormalAddr + ((U32)i << POS_BANK_NUM);
            else if(sourceRG==DRAM_DFS_SHUFFLE_1)
                addr = ShuffleRegTable[Offset].uiShuffle1Addr + ((U32)i << POS_BANK_NUM);
            else
                addr = ShuffleRegTable[Offset].uiShuffle2Addr + ((U32)i << POS_BANK_NUM);

            dst->Dramc_Backup[i][Offset] = u4IO32Read4B(addr);
        }
    }
    
    //PHY
    for(i=0; i<DDRPHY_CONF_MAX; i++)
    {
        for (Offset = FREQREG_SIZE_DRAMC; Offset < FREQREG_SIZE; Offset++)
        {
            U32 addr;
            if(sourceRG==DRAM_DFS_SHUFFLE_NORMAL)
                addr = ShuffleRegTable[Offset].uiNormalAddr + ((U32)i << POS_BANK_NUM);
            else if(sourceRG==DRAM_DFS_SHUFFLE_1)
                addr = ShuffleRegTable[Offset].uiShuffle1Addr + ((U32)i << POS_BANK_NUM);
            else
                addr = ShuffleRegTable[Offset].uiShuffle2Addr + ((U32)i << POS_BANK_NUM);

            dst->PHY_Backup[i][Offset-FREQREG_SIZE_DRAMC] = u4IO32Read4B(addr);
        }
    }
}


void DramcRestoreBackup(DRAMC_CTX_T *p, DFS_SW_BACKUP_REG_T *source, DRAM_DFS_SHUFFLE_TYPE_T dstRG)
{
	U32 Offset;
    U32 i;
    mcSHOW_DBG_MSG(("Restore backup to frequency registers. \n"));

   //DRAMC
   for(i=0; i<CHANNEL_MAX; i++)
   {
       for (Offset = 0; Offset < FREQREG_SIZE_DRAMC; Offset++)
       {
           U32 addr;
           if(dstRG==DRAM_DFS_SHUFFLE_NORMAL)
               addr = ShuffleRegTable[Offset].uiNormalAddr + ((U32)i << POS_BANK_NUM);
           else if(dstRG==DRAM_DFS_SHUFFLE_1)
               addr = ShuffleRegTable[Offset].uiShuffle1Addr + ((U32)i << POS_BANK_NUM);
           else
               addr = ShuffleRegTable[Offset].uiShuffle2Addr + ((U32)i << POS_BANK_NUM);
   
           vIO32Write4B(addr, source->Dramc_Backup[i][Offset]);
       }
   }
   
   //PHY
   for(i=0; i<DDRPHY_CONF_MAX; i++)
   {
       for (Offset = FREQREG_SIZE_DRAMC; Offset < FREQREG_SIZE; Offset++)
       {
           U32 addr;
           if(dstRG==DRAM_DFS_SHUFFLE_NORMAL)
               addr = ShuffleRegTable[Offset].uiNormalAddr + ((U32)i << POS_BANK_NUM);
           else if(dstRG==DRAM_DFS_SHUFFLE_1)
               addr = ShuffleRegTable[Offset].uiShuffle1Addr + ((U32)i << POS_BANK_NUM);
           else
               addr = ShuffleRegTable[Offset].uiShuffle2Addr + ((U32)i << POS_BANK_NUM);
   
           vIO32Write4B(addr, source->PHY_Backup[i][Offset-FREQREG_SIZE_DRAMC]);
       }
   }
}

void DramcDumpFreqSetting(DRAMC_CTX_T *p)
{
	U32 Offset;
	U32 i;
	
    mcSHOW_DBG_MSG(("Dump shuffle registers...\n"));
    mcSHOW_DBG_MSG(("Shuffle registers number = %d\n", FREQREG_SIZE));

    //DRAMC
    for(i=0; i<CHANNEL_MAX; i++)
    {
        mcSHOW_DBG_MSG(("======  DRAMC CONF %d=======\n",i));
        for (Offset = 0; Offset < FREQREG_SIZE_DRAMC; Offset++)
        {
            U32 u4Shuffle1Addr, u4Shuffle2Addr, u4NormalAddr;
            u4NormalAddr = ShuffleRegTable[Offset].uiNormalAddr + ((U32)i << POS_BANK_NUM);
            u4Shuffle1Addr = ShuffleRegTable[Offset].uiShuffle1Addr + ((U32)i << POS_BANK_NUM);
            u4Shuffle2Addr = ShuffleRegTable[Offset].uiShuffle2Addr + ((U32)i << POS_BANK_NUM);

            #ifdef ETT_PRINT_FORMAT
            mcSHOW_DBG_MSG(("%d [Normal] Addr 0x%X = 0x%X [Shuffle1] Addr 0x%X = 0x%X [Shuffle2] Addr 0x%X = 0x%X\n",  Offset, 
                    u4NormalAddr, u4IO32Read4B(u4NormalAddr),
                    u4Shuffle1Addr, u4IO32Read4B(u4Shuffle1Addr),
                    u4Shuffle2Addr, u4IO32Read4B(u4Shuffle2Addr)));
            #else
            mcSHOW_DBG_MSG(("%2d [Normal] Addr 0x%8xh = 0x%8xh [Shuffle1] Addr 0x%8xh = 0x%8xh [Shuffle2] Addr 0x%8xh = 0x%8xh\n",  Offset, 
                    u4NormalAddr, u4IO32Read4B(u4NormalAddr),
                    u4Shuffle1Addr, u4IO32Read4B(u4Shuffle1Addr),
                    u4Shuffle2Addr, u4IO32Read4B(u4Shuffle2Addr)));
            #endif
        }
    }
    
    //PHY
    for(i=0; i<DDRPHY_CONF_MAX; i++)
    {
        mcSHOW_DBG_MSG(("======  DDRPHY CONF %d=======\n",i));
        for (Offset = FREQREG_SIZE_DRAMC; Offset < FREQREG_SIZE; Offset++)
        {
            U32 u4Shuffle1Addr, u4Shuffle2Addr, u4NormalAddr;
            u4NormalAddr = ShuffleRegTable[Offset].uiNormalAddr + ((U32)i << POS_BANK_NUM);
            u4Shuffle1Addr = ShuffleRegTable[Offset].uiShuffle1Addr + ((U32)i << POS_BANK_NUM);
            u4Shuffle2Addr = ShuffleRegTable[Offset].uiShuffle2Addr + ((U32)i << POS_BANK_NUM);

            #ifdef ETT_PRINT_FORMAT
            mcSHOW_DBG_MSG(("%d [Normal] Addr 0x%X = 0x%X [Shuffle1] Addr 0x%X = 0x%X [Shuffle2] Addr 0x%X = 0x%X\n",  Offset, 
                    u4NormalAddr, u4IO32Read4B(u4NormalAddr),
                    u4Shuffle1Addr, u4IO32Read4B(u4Shuffle1Addr),
                    u4Shuffle2Addr, u4IO32Read4B(u4Shuffle2Addr)));

            #else
            mcSHOW_DBG_MSG(("%2d [Normal] Addr 0x%8xh = 0x%8xh [Shuffle1] Addr 0x%8xh = 0x%8xh [Shuffle2] Addr 0x%8xh = 0x%8xh\n",  Offset, 
                    u4NormalAddr, u4IO32Read4B(u4NormalAddr),
                    u4Shuffle1Addr, u4IO32Read4B(u4Shuffle1Addr),
                    u4Shuffle2Addr, u4IO32Read4B(u4Shuffle2Addr)));

            #endif
        }

        {
            U32 u4DRVN_PRE, u4DRVP_PRE, u4PRE_EN, u4CAP_SEL;
            u4DRVN_PRE = u4IO32ReadFldAlign(DDRPHY_CMD10+((U32)i<<POS_BANK_NUM), CMD10_RG_TX_ARCMD_DRVN_PRE);
            u4DRVP_PRE = u4IO32ReadFldAlign(DDRPHY_CMD10+((U32)i<<POS_BANK_NUM), CMD10_RG_TX_ARCMD_DRVP_PRE);
            u4PRE_EN = u4IO32ReadFldAlign(DDRPHY_CMD10+((U32)i<<POS_BANK_NUM), CMD10_RG_TX_ARCMD_PRE_EN);
            u4CAP_SEL = u4IO32ReadFldAlign(DDRPHY_PLL8+((U32)i<<POS_BANK_NUM), PLL8_RG_RPI_CAP_SEL);
            mcSHOW_DBG_MSG(("%[Normal] DRVN_PRE=%d, DRVP_PRE=%d, PRE_EN=%d, CAP_SEL=%d \n", u4DRVN_PRE, u4DRVP_PRE, u4PRE_EN, u4CAP_SEL));

            u4DRVN_PRE = u4IO32ReadFldAlign(DDRPHY_SHU_R_CAP_SEL_THRD+((U32)i<<POS_BANK_NUM), SHU_RG_TX_ARCMD_DRVN_PRE_SEC);
            u4DRVP_PRE = u4IO32ReadFldAlign(DDRPHY_SHU_R_CAP_SEL_THRD+((U32)i<<POS_BANK_NUM), SHU_RG_TX_ARCMD_DRVP_PRE_SEC);
            u4PRE_EN = u4IO32ReadFldAlign(DDRPHY_SHU_R_CAP_SEL_THRD+((U32)i<<POS_BANK_NUM), SHU_RG_TX_ARCMD_PRE_EN_SEC);
            u4CAP_SEL = u4IO32ReadFldAlign(DDRPHY_SHU_R_CAP_SEL_SEC+((U32)i<<POS_BANK_NUM), SHU_RG_RPI_CAP_SEL_SEC);
            mcSHOW_DBG_MSG(("%[Shuffle1] DRVN_PRE=%d, DRVP_PRE=%d, PRE_EN=%d, CAP_SEL=%d \n", u4DRVN_PRE, u4DRVP_PRE, u4PRE_EN, u4CAP_SEL));

            u4DRVN_PRE = u4IO32ReadFldAlign(DDRPHY_SHU_R_CAP_SEL_THRD+((U32)i<<POS_BANK_NUM), SHU_RG_TX_ARCMD_DRVN_PRE_THRD);
            u4DRVP_PRE = u4IO32ReadFldAlign(DDRPHY_SHU_R_CAP_SEL_THRD+((U32)i<<POS_BANK_NUM), SHU_RG_TX_ARCMD_DRVP_PRE_THRD);
            u4PRE_EN = u4IO32ReadFldAlign(DDRPHY_SHU_R_CAP_SEL_THRD+((U32)i<<POS_BANK_NUM), SHU_RG_TX_ARCMD_PRE_EN_THRD);
            u4CAP_SEL = u4IO32ReadFldAlign(DDRPHY_SHU_R_CAP_SEL_THRD+((U32)i<<POS_BANK_NUM), SHU_RG_RPI_CAP_SEL_THRD);
            mcSHOW_DBG_MSG(("%[Shuffle2] DRVN_PRE=%d, DRVP_PRE=%d, PRE_EN=%d, CAP_SEL=%d \n", u4DRVN_PRE, u4DRVP_PRE, u4PRE_EN, u4CAP_SEL));
        }
    }
}

void DramcDFSTestCode(DRAMC_CTX_T *p)
{
    //shuffle 1
    vIO32WriteFldMulti_All(DDRPHY_PLL19, P_Fld(0x1,PLL12_RG_ARPI_MPDIV_IN_SEL)|P_Fld(0x1, PLL12_RG_BRPI_MPDIV_IN_SEL));    
}

#define DRAMC_NAO_BASE DRAMC0_NAO_BASE
#define CHA_RK0_DUMMY_ADDR  0x40000000
#define CHA_RK1_DUMMY_ADDR  0xA0000000
#define CHB_RK0_DUMMY_ADDR  0x40000100
#define CHB_RK1_DUMMY_ADDR  0xA0000100
void DVFS_gating_auto_save() // backup-restore gating PI/UI before DVFS
{
    unsigned int u4HWTrackPICH0R0, u4HWTrackPICH0R1;
    unsigned int u4HWTrackUICH0R0P0, u4HWTrackUICH0R1P0;
    unsigned int u4HWTrackUICH0R0P1, u4HWTrackUICH0R1P1;
    
    unsigned int bak_data0, bak_data1;
    unsigned int dfs_level;   
    U8 checkPIResult=DRAM_FAIL;
    U32 dummyValue[2];
    
        //ddrphy_conf shuffle_three
        /*TINFO="===  ddrphy shuffle_three reg start ==="*/

	  	  //*(UINT32P)DRAMC_WBR = 0x5;     
        dummyValue[0] = *((UINT32P)(CHA_RK0_DUMMY_ADDR));
        dummyValue[1] = *((UINT32P)(CHB_RK0_DUMMY_ADDR));
	  	  
	  	  dfs_level = (*((UINT32P)(DRAMC0_BASE   + 0x028 )) & 0x00003000) >> 12;
	  	   
        *((UINT32P)(DRAMC0_BASE   + 0x044 )) |= (0x1 << 12);           // MANUDLLFRZ = 1
        *((UINT32P)(DRAMC1_BASE   + 0x044 )) |= (0x1 << 12);           // MANUDLLFRZ = 1
        *((UINT32P)(DRAMC0_BASE   + 0x1c0 )) |= (0x1 << 20);           // STBSTATE_OPT = 1
        *((UINT32P)(DRAMC1_BASE   + 0x1c0 )) |= (0x1 << 20);           // STBSTATE_OPT = 1

        //dummy_write to CHA-RANK0 reserved memory address1;    
        //dummy_read from CHA-RANK0 address1;    
        //dummy_read from CHA-RANK1 address2;    
        *((UINT32P)(CHA_RK0_DUMMY_ADDR)) = dummyValue[0];
        dummyValue[0] = *((UINT32P)(CHA_RK0_DUMMY_ADDR));
        #ifdef DUAL_RANKS    
        dummyValue[0] = *((UINT32P)(CHA_RK1_DUMMY_ADDR));
        #endif

        *((UINT32P)(CHB_RK0_DUMMY_ADDR)) = dummyValue[1];
        dummyValue[1] = *((UINT32P)(CHB_RK0_DUMMY_ADDR));
        #ifdef DUAL_RANKS    
        dummyValue[1] = *((UINT32P)(CHB_RK1_DUMMY_ADDR));
        #endif

        do
        {
            U8 u1Dqs_pi[4];
            checkPIResult=DRAM_OK;
            u4HWTrackPICH0R0 = *((UINT32P)(DRAMC_NAO_BASE   + 0x374 ));    
            u1Dqs_pi[0]  = u4HWTrackPICH0R0 & 0xff;
            u1Dqs_pi[1]  = (u4HWTrackPICH0R0 >>8) & 0xff;
            u1Dqs_pi[2]  = (u4HWTrackPICH0R0 >>16) & 0xff;
            u1Dqs_pi[3]  = (u4HWTrackPICH0R0 >>24) & 0xff;
            if(u1Dqs_pi[0]>0x1F || u1Dqs_pi[1]>0x1F || u1Dqs_pi[2]>0x1F || u1Dqs_pi[3]>0x1F)
            {
                checkPIResult = DRAM_FAIL;
            }
            
            u4HWTrackPICH0R1 = *((UINT32P)(DRAMC_NAO_BASE   + 0x378 ));    
            u1Dqs_pi[0]  = u4HWTrackPICH0R1 & 0xff;
            u1Dqs_pi[1]  = (u4HWTrackPICH0R1 >>8) & 0xff;
            u1Dqs_pi[2]  = (u4HWTrackPICH0R1 >>16) & 0xff;
            u1Dqs_pi[3]  = (u4HWTrackPICH0R1 >>24) & 0xff;
            if(u1Dqs_pi[0]>0x1F || u1Dqs_pi[1]>0x1F || u1Dqs_pi[2]>0x1F || u1Dqs_pi[3]>0x1F)
            {
                checkPIResult = DRAM_FAIL;
            }
            if(checkPIResult == DRAM_FAIL)
            {   
                mcDELAY_US(1);
            }
        }while(checkPIResult==DRAM_FAIL);

    #if !DFS_GTDMW_SYNC
        mcSHOW_DBG_MSG(("SW Gating AutoSave CH_A. \n"));
        u4HWTrackPICH0R0 = *((UINT32P)(DRAMC_NAO_BASE   + 0x374 ));
        u4HWTrackPICH0R1 = *((UINT32P)(DRAMC_NAO_BASE   + 0x378 ));
        u4HWTrackUICH0R0P0 = *((UINT32P)(DRAMC_NAO_BASE   + 0x37c ));
        u4HWTrackUICH0R0P1 = *((UINT32P)(DRAMC_NAO_BASE   + 0x384 ));
        u4HWTrackUICH0R1P0 = *((UINT32P)(DRAMC_NAO_BASE   + 0x380 ));
        u4HWTrackUICH0R1P1 = *((UINT32P)(DRAMC_NAO_BASE   + 0x388 ));
        if (dfs_level == 0x0) {

        } else if (dfs_level == 0x1){

        } else { //dfs_level == 0x2

        }	
//Write 0x10004094=0x1020e374
//Write 0x10004098=0x1020e378
        if (dfs_level == 0x0) {
          *((UINT32P)(DRAMC0_BASE   + 0x094 )) = u4HWTrackPICH0R0;
          *((UINT32P)(DRAMC0_BASE   + 0x098 )) = u4HWTrackPICH0R1;
        } else if (dfs_level == 0x1){
          *((UINT32P)(DRAMC0_BASE   + 0x840 )) = u4HWTrackPICH0R0;
          *((UINT32P)(DRAMC0_BASE   + 0x844 )) = u4HWTrackPICH0R1;
        } else { //dfs_level == 0x2
          *((UINT32P)(DRAMC0_BASE   + 0xc40 )) = u4HWTrackPICH0R0;
          *((UINT32P)(DRAMC0_BASE   + 0xc44 )) = u4HWTrackPICH0R1;
        }	
//Write 0x10004404[22:20]=0x1020e384[5:3], 
//      0x10004404[14:12]=0x1020e37c[5:3]       
        if (dfs_level == 0x0) {
        bak_data0 = *((UINT32P)(DRAMC0_BASE   + 0x404 )) & 0xff8f8fff;
        } else if (dfs_level == 0x1){
        bak_data0 = *((UINT32P)(DRAMC0_BASE   + 0x864 )) & 0xff8f8fff;
        } else { //dfs_level == 0x2
        bak_data0 = *((UINT32P)(DRAMC0_BASE   + 0xc64 )) & 0xff8f8fff;
        }	
        bak_data1 = u4HWTrackUICH0R0P1 & 0x7 << 3;
        bak_data0 |= bak_data1 << 17;
        bak_data1 = u4HWTrackUICH0R0P0 & 0x7 << 3;                
        bak_data0 |= bak_data1 << 9;
        if (dfs_level == 0x0) {
        *((UINT32P)(DRAMC0_BASE   + 0x404 )) = bak_data0;
        } else if (dfs_level == 0x1){
        *((UINT32P)(DRAMC0_BASE   + 0x864 )) = bak_data0;
        } else { //dfs_level == 0x2
        *((UINT32P)(DRAMC0_BASE   + 0xc64 )) = bak_data0;
        }	
//Write 0x10004410[25:22]={0x1020e384[1:0], 0x1020e37c[1:0]}  
        if (dfs_level == 0x0) {
        bak_data0 = *((UINT32P)(DRAMC0_BASE   + 0x410 )) & 0xfc3fffff;
        } else if (dfs_level == 0x1){
        bak_data0 = *((UINT32P)(DRAMC0_BASE   + 0x870 )) & 0xfc3fffff;
        } else { //dfs_level == 0x2
        bak_data0 = *((UINT32P)(DRAMC0_BASE   + 0xc70 )) & 0xfc3fffff;
        }	
        bak_data1 = u4HWTrackUICH0R0P1 & 0x3;
        bak_data0 |= bak_data1 << 24;
        bak_data1 = u4HWTrackUICH0R0P0 & 0x3;                
        bak_data0 |= bak_data1 << 22;
        if (dfs_level == 0x0) {
        *((UINT32P)(DRAMC0_BASE   + 0x410 )) = bak_data0;
        } else if (dfs_level == 0x1){
        *((UINT32P)(DRAMC0_BASE   + 0x870 )) = bak_data0;
        } else { //dfs_level == 0x2
        *((UINT32P)(DRAMC0_BASE   + 0xc70 )) = bak_data0;
        }	
//Write 0x10004418[10:8]=0x1020e388[5:3]
//            0x10004418[6:4]=0x1020e380[5:3]
//            0x10004418[3:2]=0x1020e388[1:0] 
//            0x10004418[1:0]=0x1020e380[1:0]
        if (dfs_level == 0x0) {
        bak_data0 = *((UINT32P)(DRAMC0_BASE   + 0x418 )) & 0xfffff880;
        } else if (dfs_level == 0x1){
        bak_data0 = *((UINT32P)(DRAMC0_BASE   + 0x878 )) & 0xfffff880;
        } else { //dfs_level == 0x2
        bak_data0 = *((UINT32P)(DRAMC0_BASE   + 0xc78 )) & 0xfffff880;
        }	
        bak_data1 = u4HWTrackUICH0R1P1 & 0x7 << 3;
        bak_data0 |= bak_data1 << 5;
        bak_data1 = u4HWTrackUICH0R1P0 & 0x7 << 3;                
        bak_data0 |= bak_data1 << 1;
        bak_data1 = u4HWTrackUICH0R1P1 & 0x3;
        bak_data0 |= bak_data1 << 2;
        bak_data1 = u4HWTrackUICH0R1P0 & 0x3;                
        bak_data0 |= bak_data1;
        if (dfs_level == 0x0) {
        *((UINT32P)(DRAMC0_BASE   + 0x418 )) = bak_data0;
        } else if (dfs_level == 0x1){
        *((UINT32P)(DRAMC0_BASE   + 0x878 )) = bak_data0;
        } else { //dfs_level == 0x2
        *((UINT32P)(DRAMC0_BASE   + 0xc78 )) = bak_data0;
        }	
//Write 0x10004430[30:28]=0x1020e384[29:27] 
//            0x10004430[26:24]=0x1020e37c[29:27]
//            0x10004430[22:20]=0x1020e384[21:19] 
//            0x10004430[18:16]=0x1020e37c[21:19]
//            0x10004430[14:12]=0x1020e384[13:11] 
//            0x10004430[10:8]=0x1020e37c[13:11]
        if (dfs_level == 0x0) {
        bak_data0 = *((UINT32P)(DRAMC0_BASE   + 0x430 )) & 0x888888ff;
        } else if (dfs_level == 0x1){
        bak_data0 = *((UINT32P)(DRAMC0_BASE   + 0x8c0 )) & 0x888888ff;
        } else { //dfs_level == 0x2
        bak_data0 = *((UINT32P)(DRAMC0_BASE   + 0xcc0 )) & 0x888888ff;
        }	
        bak_data1 = u4HWTrackUICH0R0P1 & 0x7 << 27;
        bak_data0 |= bak_data1 << 1;
        bak_data1 = u4HWTrackUICH0R0P0 & 0x7 << 27;                
        bak_data0 |= bak_data1 >> 3;
        bak_data1 = u4HWTrackUICH0R0P1 & 0x7 << 19;
        bak_data0 |= bak_data1 << 1;
        bak_data1 = u4HWTrackUICH0R0P0 & 0x7 << 19;                
        bak_data0 |= bak_data1 >> 3;
        bak_data1 = u4HWTrackUICH0R0P1 & 0x7 << 11;
        bak_data0 |= bak_data1 << 1;
        bak_data1 = u4HWTrackUICH0R0P0 & 0x7 << 11;                
        bak_data0 |= bak_data1 >> 3;
        if (dfs_level == 0x0) {
        *((UINT32P)(DRAMC0_BASE   + 0x430 )) = bak_data0;
        } else if (dfs_level == 0x1){
        *((UINT32P)(DRAMC0_BASE   + 0x8c0 )) = bak_data0;
        } else { //dfs_level == 0x2
        *((UINT32P)(DRAMC0_BASE   + 0xcc0 )) = bak_data0;
        }	
//Write 0x10004434[30:28]=0x1020e384[26:24] 
//            0x10004434[26:24]=0x1020e37c[26:24]
//            0x10004434[22:20]=0x1020e384[18:16] 
//            0x10004434[18:16]=0x1020e37c[18:16]
//            0x10004434[14:12]=0x1020e384[10:8] 
//            0x10004434[10:8]=0x1020e37c[10:8]
        if (dfs_level == 0x0) {
        bak_data0 = *((UINT32P)(DRAMC0_BASE   + 0x434 )) & 0x888888ff;
        } else if (dfs_level == 0x1){
        bak_data0 = *((UINT32P)(DRAMC0_BASE   + 0x8c4 )) & 0x888888ff;
        } else { //dfs_level == 0x2
        bak_data0 = *((UINT32P)(DRAMC0_BASE   + 0xcc4 )) & 0x888888ff;
        }	
        bak_data1 = u4HWTrackUICH0R0P1 & 0x7 << 24;
        bak_data0 |= bak_data1 << 4;
        bak_data1 = u4HWTrackUICH0R0P0 & 0x7 << 24;                
        bak_data0 |= bak_data1;
        bak_data1 = u4HWTrackUICH0R0P1 & 0x7 << 16;
        bak_data0 |= bak_data1 << 4;
        bak_data1 = u4HWTrackUICH0R0P0 & 0x7 << 16;                
        bak_data0 |= bak_data1;
        bak_data1 = u4HWTrackUICH0R0P1 & 0x7 << 8;
        bak_data0 |= bak_data1 << 4;
        bak_data1 = u4HWTrackUICH0R0P0 & 0x7 << 8;                
        bak_data0 |= bak_data1;
        if (dfs_level == 0x0) {
        *((UINT32P)(DRAMC0_BASE   + 0x434 )) = bak_data0;
        } else if (dfs_level == 0x1){
        *((UINT32P)(DRAMC0_BASE   + 0x8c4 )) = bak_data0;
        } else { //dfs_level == 0x2
        *((UINT32P)(DRAMC0_BASE   + 0xcc4 )) = bak_data0;
        }	
//Write 0x10004438[30:28]=0x1020e388[29:27] 
//            0x10004438[26:24]=0x1020e380[29:27]
//            0x10004438[22:20]=0x1020e388[21:19] 
//            0x10004438[18:16]=0x1020e380[21:19]
//            0x10004438[14:12]=0x1020e388[13:11] 
//            0x10004438[10:8]=0x1020e380[13:11]
        if (dfs_level == 0x0) {
        bak_data0 = *((UINT32P)(DRAMC0_BASE   + 0x438 )) & 0x888888ff;
        } else if (dfs_level == 0x1){
        bak_data0 = *((UINT32P)(DRAMC0_BASE   + 0x8c8 )) & 0x888888ff;
        } else { //dfs_level == 0x2
        bak_data0 = *((UINT32P)(DRAMC0_BASE   + 0xcc8 )) & 0x888888ff;
        }	
        bak_data1 = u4HWTrackUICH0R1P1 & 0x7 << 27;
        bak_data0 |= bak_data1 << 1;
        bak_data1 = u4HWTrackUICH0R1P0 & 0x7 << 27;                
        bak_data0 |= bak_data1 >> 3;
        bak_data1 = u4HWTrackUICH0R1P1 & 0x7 << 19;
        bak_data0 |= bak_data1 << 1;
        bak_data1 = u4HWTrackUICH0R1P0 & 0x7 << 19;                
        bak_data0 |= bak_data1 >> 3;
        bak_data1 = u4HWTrackUICH0R1P1 & 0x7 << 11;
        bak_data0 |= bak_data1 << 1;
        bak_data1 = u4HWTrackUICH0R1P0 & 0x7 << 11;                
        bak_data0 |= bak_data1 >> 3;
        if (dfs_level == 0x0) {
        *((UINT32P)(DRAMC0_BASE   + 0x438 )) = bak_data0;
        } else if (dfs_level == 0x1){
        *((UINT32P)(DRAMC0_BASE   + 0x8c8 )) = bak_data0;
        } else { //dfs_level == 0x2
        *((UINT32P)(DRAMC0_BASE   + 0xcc8 )) = bak_data0;
        }	
//Write 0x1000443c[30:28]=0x1020e388[26:24] 
//            0x1000443c[26:24]=0x1020e380[26:24]
//            0x1000443c[22:20]=0x1020e388[18:16] 
//            0x1000443c[18:16]=0x1020e380[18:16]
//            0x1000443c[14:12]=0x1020e388[10:8] 
//            0x1000443c[10:8]=0x1020e380[10:8]
        if (dfs_level == 0x0) {
        bak_data0 = *((UINT32P)(DRAMC0_BASE   + 0x43c )) & 0x888888ff;
        } else if (dfs_level == 0x1){
        bak_data0 = *((UINT32P)(DRAMC0_BASE   + 0x8cc )) & 0x888888ff;
        } else { //dfs_level == 0x2
        bak_data0 = *((UINT32P)(DRAMC0_BASE   + 0xccc )) & 0x888888ff;
        }	
        bak_data1 = u4HWTrackUICH0R1P1 & 0x7 << 24;
        bak_data0 |= bak_data1 << 4;
        bak_data1 = u4HWTrackUICH0R1P0 & 0x7 << 24;                
        bak_data0 |= bak_data1;
        bak_data1 = u4HWTrackUICH0R1P1 & 0x7 << 16;
        bak_data0 |= bak_data1 << 4;
        bak_data1 = u4HWTrackUICH0R1P0 & 0x7 << 16;                
        bak_data0 |= bak_data1;
        bak_data1 = u4HWTrackUICH0R1P1 & 0x7 << 8;
        bak_data0 |= bak_data1 << 4;
        bak_data1 = u4HWTrackUICH0R1P0 & 0x7 << 8;                
        bak_data0 |= bak_data1;
        if (dfs_level == 0x0) {
        *((UINT32P)(DRAMC0_BASE   + 0x43c )) = bak_data0;
        } else if (dfs_level == 0x1){
        *((UINT32P)(DRAMC0_BASE   + 0x8cc )) = bak_data0;
        } else { //dfs_level == 0x2
        *((UINT32P)(DRAMC0_BASE   + 0xccc )) = bak_data0;
        }	
//Write 0x10004454[31:28]=
//            {0x1020e388[2], 0x1020e380[2], 0x1020e384[2], 0x1020e37c[2]}      
        if (dfs_level == 0x0) {
        bak_data0 = *((UINT32P)(DRAMC0_BASE   + 0x454 )) & 0x0fffffff;
        } else if (dfs_level == 0x1){
        bak_data0 = *((UINT32P)(DRAMC0_BASE   + 0x898 )) & 0x0fffffff;
        } else { //dfs_level == 0x2
        bak_data0 = *((UINT32P)(DRAMC0_BASE   + 0xc98 )) & 0x0fffffff;
        }	
        bak_data1 = u4HWTrackUICH0R1P1 & 0x1 << 2;
        bak_data0 |= bak_data1 << 29;
        bak_data1 = u4HWTrackUICH0R1P0 & 0x1 << 2;                
        bak_data0 |= bak_data1 << 28;
        bak_data1 = u4HWTrackUICH0R0P1 & 0x1 << 2;
        bak_data0 |= bak_data1 << 27;
        bak_data1 = u4HWTrackUICH0R0P0 & 0x1 << 2;                
        bak_data0 |= bak_data1 << 26;
        if (dfs_level == 0x0) {
        *((UINT32P)(DRAMC0_BASE   + 0x454 )) = bak_data0;
        } else if (dfs_level == 0x1){
        *((UINT32P)(DRAMC0_BASE   + 0x898 )) = bak_data0;
        } else { //dfs_level == 0x2
        *((UINT32P)(DRAMC0_BASE   + 0xc98 )) = bak_data0;
        }	

    #endif //DFS_GTDMW_SYNC

// ch1
        do
        {
            U8 u1Dqs_pi[4];
            checkPIResult=DRAM_OK;
            u4HWTrackPICH0R0 = *((UINT32P)(DRAMC1_NAO_BASE   + 0x374 ));    
            u1Dqs_pi[0]  = u4HWTrackPICH0R0 & 0xff;
            u1Dqs_pi[1]  = (u4HWTrackPICH0R0 >>8) & 0xff;
            u1Dqs_pi[2]  = (u4HWTrackPICH0R0 >>16) & 0xff;
            u1Dqs_pi[3]  = (u4HWTrackPICH0R0 >>24) & 0xff;
            if(u1Dqs_pi[0]>0x1F || u1Dqs_pi[1]>0x1F || u1Dqs_pi[2]>0x1F || u1Dqs_pi[3]>0x1F)
            {
                checkPIResult = DRAM_FAIL;
            }
            
            u4HWTrackPICH0R1 = *((UINT32P)(DRAMC1_NAO_BASE   + 0x378 ));    
            u1Dqs_pi[0]  = u4HWTrackPICH0R1 & 0xff;
            u1Dqs_pi[1]  = (u4HWTrackPICH0R1 >>8) & 0xff;
            u1Dqs_pi[2]  = (u4HWTrackPICH0R1 >>16) & 0xff;
            u1Dqs_pi[3]  = (u4HWTrackPICH0R1 >>24) & 0xff;
            if(u1Dqs_pi[0]>0x1F || u1Dqs_pi[1]>0x1F || u1Dqs_pi[2]>0x1F || u1Dqs_pi[3]>0x1F)
            {
                checkPIResult = DRAM_FAIL;
            }
            if(checkPIResult == DRAM_FAIL)
            {   
                mcDELAY_US(1);
            }
        }while(checkPIResult==DRAM_FAIL);

    #if !DFS_GTDMW_SYNC
        mcSHOW_DBG_MSG(("SW Gating AutoSave CH_B. \n"));
        u4HWTrackPICH0R0 = *((UINT32P)(DRAMC1_NAO_BASE   + 0x374 ));
        u4HWTrackPICH0R1 = *((UINT32P)(DRAMC1_NAO_BASE   + 0x378 ));
        u4HWTrackUICH0R0P0 = *((UINT32P)(DRAMC1_NAO_BASE   + 0x37c ));
        u4HWTrackUICH0R0P1 = *((UINT32P)(DRAMC1_NAO_BASE   + 0x384 ));
        u4HWTrackUICH0R1P0 = *((UINT32P)(DRAMC1_NAO_BASE   + 0x380 ));
        u4HWTrackUICH0R1P1 = *((UINT32P)(DRAMC1_NAO_BASE   + 0x388 ));
        if (dfs_level == 0x0) {

        } else if (dfs_level == 0x1){

        } else { //dfs_level == 0x2

        }	
//Write 0x10004094=0x1020e374
//Write 0x10004098=0x1020e378
        if (dfs_level == 0x0) {
          *((UINT32P)(DRAMC1_BASE   + 0x094 )) = u4HWTrackPICH0R0;
          *((UINT32P)(DRAMC1_BASE   + 0x098 )) = u4HWTrackPICH0R1;
        } else if (dfs_level == 0x1){
          *((UINT32P)(DRAMC1_BASE   + 0x840 )) = u4HWTrackPICH0R0;
          *((UINT32P)(DRAMC1_BASE   + 0x844 )) = u4HWTrackPICH0R1;
        } else { //dfs_level == 0x2
          *((UINT32P)(DRAMC1_BASE   + 0xc40 )) = u4HWTrackPICH0R0;
          *((UINT32P)(DRAMC1_BASE   + 0xc44 )) = u4HWTrackPICH0R1;
        }	
//Write 0x10004404[22:20]=0x1020e384[5:3], 
//      0x10004404[14:12]=0x1020e37c[5:3]       
        if (dfs_level == 0x0) {
        bak_data0 = *((UINT32P)(DRAMC1_BASE   + 0x404 )) & 0xff8f8fff;
        } else if (dfs_level == 0x1){
        bak_data0 = *((UINT32P)(DRAMC1_BASE   + 0x864 )) & 0xff8f8fff;
        } else { //dfs_level == 0x2
        bak_data0 = *((UINT32P)(DRAMC1_BASE   + 0xc64 )) & 0xff8f8fff;
        }	
        bak_data1 = u4HWTrackUICH0R0P1 & 0x7 << 3;
        bak_data0 |= bak_data1 << 17;
        bak_data1 = u4HWTrackUICH0R0P0 & 0x7 << 3;                
        bak_data0 |= bak_data1 << 9;
        if (dfs_level == 0x0) {
        *((UINT32P)(DRAMC1_BASE   + 0x404 )) = bak_data0;
        } else if (dfs_level == 0x1){
        *((UINT32P)(DRAMC1_BASE   + 0x864 )) = bak_data0;
        } else { //dfs_level == 0x2
        *((UINT32P)(DRAMC1_BASE   + 0xc64 )) = bak_data0;
        }	
//Write 0x10004410[25:22]={0x1020e384[1:0], 0x1020e37c[1:0]}  
        if (dfs_level == 0x0) {
        bak_data0 = *((UINT32P)(DRAMC1_BASE   + 0x410 )) & 0xfc3fffff;
        } else if (dfs_level == 0x1){
        bak_data0 = *((UINT32P)(DRAMC1_BASE   + 0x870 )) & 0xfc3fffff;
        } else { //dfs_level == 0x2
        bak_data0 = *((UINT32P)(DRAMC1_BASE   + 0xc70 )) & 0xfc3fffff;
        }	
        bak_data1 = u4HWTrackUICH0R0P1 & 0x3;
        bak_data0 |= bak_data1 << 24;
        bak_data1 = u4HWTrackUICH0R0P0 & 0x3;                
        bak_data0 |= bak_data1 << 22;
        if (dfs_level == 0x0) {
        *((UINT32P)(DRAMC1_BASE   + 0x410 )) = bak_data0;
        } else if (dfs_level == 0x1){
        *((UINT32P)(DRAMC1_BASE   + 0x870 )) = bak_data0;
        } else { //dfs_level == 0x2
        *((UINT32P)(DRAMC1_BASE   + 0xc70 )) = bak_data0;
        }	
//Write 0x10004418[10:8]=0x1020e388[5:3]
//            0x10004418[6:4]=0x1020e380[5:3]
//            0x10004418[3:2]=0x1020e388[1:0] 
//            0x10004418[1:0]=0x1020e380[1:0]
        if (dfs_level == 0x0) {
        bak_data0 = *((UINT32P)(DRAMC1_BASE   + 0x418 )) & 0xfffff880;
        } else if (dfs_level == 0x1){
        bak_data0 = *((UINT32P)(DRAMC1_BASE   + 0x878 )) & 0xfffff880;
        } else { //dfs_level == 0x2
        bak_data0 = *((UINT32P)(DRAMC1_BASE   + 0xc78 )) & 0xfffff880;
        }	
        bak_data1 = u4HWTrackUICH0R1P1 & 0x7 << 3;
        bak_data0 |= bak_data1 << 5;
        bak_data1 = u4HWTrackUICH0R1P0 & 0x7 << 3;                
        bak_data0 |= bak_data1 << 1;
        bak_data1 = u4HWTrackUICH0R1P1 & 0x3;
        bak_data0 |= bak_data1 << 2;
        bak_data1 = u4HWTrackUICH0R1P0 & 0x3;                
        bak_data0 |= bak_data1;
        if (dfs_level == 0x0) {
        *((UINT32P)(DRAMC1_BASE   + 0x418 )) = bak_data0;
        } else if (dfs_level == 0x1){
        *((UINT32P)(DRAMC1_BASE   + 0x878 )) = bak_data0;
        } else { //dfs_level == 0x2
        *((UINT32P)(DRAMC1_BASE   + 0xc78 )) = bak_data0;
        }	
//Write 0x10004430[30:28]=0x1020e384[29:27] 
//            0x10004430[26:24]=0x1020e37c[29:27]
//            0x10004430[22:20]=0x1020e384[21:19] 
//            0x10004430[18:16]=0x1020e37c[21:19]
//            0x10004430[14:12]=0x1020e384[13:11] 
//            0x10004430[10:8]=0x1020e37c[13:11]
        if (dfs_level == 0x0) {
        bak_data0 = *((UINT32P)(DRAMC1_BASE   + 0x430 )) & 0x888888ff;
        } else if (dfs_level == 0x1){
        bak_data0 = *((UINT32P)(DRAMC1_BASE   + 0x8c0 )) & 0x888888ff;
        } else { //dfs_level == 0x2
        bak_data0 = *((UINT32P)(DRAMC1_BASE   + 0xcc0 )) & 0x888888ff;
        }	
        bak_data1 = u4HWTrackUICH0R0P1 & 0x7 << 27;
        bak_data0 |= bak_data1 << 1;
        bak_data1 = u4HWTrackUICH0R0P0 & 0x7 << 27;                
        bak_data0 |= bak_data1 >> 3;
        bak_data1 = u4HWTrackUICH0R0P1 & 0x7 << 19;
        bak_data0 |= bak_data1 << 1;
        bak_data1 = u4HWTrackUICH0R0P0 & 0x7 << 19;                
        bak_data0 |= bak_data1 >> 3;
        bak_data1 = u4HWTrackUICH0R0P1 & 0x7 << 11;
        bak_data0 |= bak_data1 << 1;
        bak_data1 = u4HWTrackUICH0R0P0 & 0x7 << 11;                
        bak_data0 |= bak_data1 >> 3;
        if (dfs_level == 0x0) {
        *((UINT32P)(DRAMC1_BASE   + 0x430 )) = bak_data0;
        } else if (dfs_level == 0x1){
        *((UINT32P)(DRAMC1_BASE   + 0x8c0 )) = bak_data0;
        } else { //dfs_level == 0x2
        *((UINT32P)(DRAMC1_BASE   + 0xcc0 )) = bak_data0;
        }	
//Write 0x10004434[30:28]=0x1020e384[26:24] 
//            0x10004434[26:24]=0x1020e37c[26:24]
//            0x10004434[22:20]=0x1020e384[18:16] 
//            0x10004434[18:16]=0x1020e37c[18:16]
//            0x10004434[14:12]=0x1020e384[10:8] 
//            0x10004434[10:8]=0x1020e37c[10:8]
        if (dfs_level == 0x0) {
        bak_data0 = *((UINT32P)(DRAMC1_BASE   + 0x434 )) & 0x888888ff;
        } else if (dfs_level == 0x1){
        bak_data0 = *((UINT32P)(DRAMC1_BASE   + 0x8c4 )) & 0x888888ff;
        } else { //dfs_level == 0x2
        bak_data0 = *((UINT32P)(DRAMC1_BASE   + 0xcc4 )) & 0x888888ff;
        }	
        bak_data1 = u4HWTrackUICH0R0P1 & 0x7 << 24;
        bak_data0 |= bak_data1 << 4;
        bak_data1 = u4HWTrackUICH0R0P0 & 0x7 << 24;                
        bak_data0 |= bak_data1;
        bak_data1 = u4HWTrackUICH0R0P1 & 0x7 << 16;
        bak_data0 |= bak_data1 << 4;
        bak_data1 = u4HWTrackUICH0R0P0 & 0x7 << 16;                
        bak_data0 |= bak_data1;
        bak_data1 = u4HWTrackUICH0R0P1 & 0x7 << 8;
        bak_data0 |= bak_data1 << 4;
        bak_data1 = u4HWTrackUICH0R0P0 & 0x7 << 8;                
        bak_data0 |= bak_data1;
        if (dfs_level == 0x0) {
        *((UINT32P)(DRAMC1_BASE   + 0x434 )) = bak_data0;
        } else if (dfs_level == 0x1){
        *((UINT32P)(DRAMC1_BASE   + 0x8c4 )) = bak_data0;
        } else { //dfs_level == 0x2
        *((UINT32P)(DRAMC1_BASE   + 0xcc4 )) = bak_data0;
        }	
//Write 0x10004438[30:28]=0x1020e388[29:27] 
//            0x10004438[26:24]=0x1020e380[29:27]
//            0x10004438[22:20]=0x1020e388[21:19] 
//            0x10004438[18:16]=0x1020e380[21:19]
//            0x10004438[14:12]=0x1020e388[13:11] 
//            0x10004438[10:8]=0x1020e380[13:11]
        if (dfs_level == 0x0) {
        bak_data0 = *((UINT32P)(DRAMC1_BASE   + 0x438 )) & 0x888888ff;
        } else if (dfs_level == 0x1){
        bak_data0 = *((UINT32P)(DRAMC1_BASE   + 0x8c8 )) & 0x888888ff;
        } else { //dfs_level == 0x2
        bak_data0 = *((UINT32P)(DRAMC1_BASE   + 0xcc8 )) & 0x888888ff;
        }	
        bak_data1 = u4HWTrackUICH0R1P1 & 0x7 << 27;
        bak_data0 |= bak_data1 << 1;
        bak_data1 = u4HWTrackUICH0R1P0 & 0x7 << 27;                
        bak_data0 |= bak_data1 >> 3;
        bak_data1 = u4HWTrackUICH0R1P1 & 0x7 << 19;
        bak_data0 |= bak_data1 << 1;
        bak_data1 = u4HWTrackUICH0R1P0 & 0x7 << 19;                
        bak_data0 |= bak_data1 >> 3;
        bak_data1 = u4HWTrackUICH0R1P1 & 0x7 << 11;
        bak_data0 |= bak_data1 << 1;
        bak_data1 = u4HWTrackUICH0R1P0 & 0x7 << 11;                
        bak_data0 |= bak_data1 >> 3;
        if (dfs_level == 0x0) {
        *((UINT32P)(DRAMC1_BASE   + 0x438 )) = bak_data0;
        } else if (dfs_level == 0x1){
        *((UINT32P)(DRAMC1_BASE   + 0x8c8 )) = bak_data0;
        } else { //dfs_level == 0x2
        *((UINT32P)(DRAMC1_BASE   + 0xcc8 )) = bak_data0;
        }	
//Write 0x1000443c[30:28]=0x1020e388[26:24] 
//            0x1000443c[26:24]=0x1020e380[26:24]
//            0x1000443c[22:20]=0x1020e388[18:16] 
//            0x1000443c[18:16]=0x1020e380[18:16]
//            0x1000443c[14:12]=0x1020e388[10:8] 
//            0x1000443c[10:8]=0x1020e380[10:8]
        if (dfs_level == 0x0) {
        bak_data0 = *((UINT32P)(DRAMC1_BASE   + 0x43c )) & 0x888888ff;
        } else if (dfs_level == 0x1){
        bak_data0 = *((UINT32P)(DRAMC1_BASE   + 0x8cc )) & 0x888888ff;
        } else { //dfs_level == 0x2
        bak_data0 = *((UINT32P)(DRAMC1_BASE   + 0xccc )) & 0x888888ff;
        }	
        bak_data1 = u4HWTrackUICH0R1P1 & 0x7 << 24;
        bak_data0 |= bak_data1 << 4;
        bak_data1 = u4HWTrackUICH0R1P0 & 0x7 << 24;                
        bak_data0 |= bak_data1;
        bak_data1 = u4HWTrackUICH0R1P1 & 0x7 << 16;
        bak_data0 |= bak_data1 << 4;
        bak_data1 = u4HWTrackUICH0R1P0 & 0x7 << 16;                
        bak_data0 |= bak_data1;
        bak_data1 = u4HWTrackUICH0R1P1 & 0x7 << 8;
        bak_data0 |= bak_data1 << 4;
        bak_data1 = u4HWTrackUICH0R1P0 & 0x7 << 8;                
        bak_data0 |= bak_data1;
        if (dfs_level == 0x0) {
        *((UINT32P)(DRAMC1_BASE   + 0x43c )) = bak_data0;
        } else if (dfs_level == 0x1){
        *((UINT32P)(DRAMC1_BASE   + 0x8cc )) = bak_data0;
        } else { //dfs_level == 0x2
        *((UINT32P)(DRAMC1_BASE   + 0xccc )) = bak_data0;
        }	
//Write 0x10004454[31:28]=
//            {0x1020e388[2], 0x1020e380[2], 0x1020e384[2], 0x1020e37c[2]}      
        if (dfs_level == 0x0) {
        bak_data0 = *((UINT32P)(DRAMC1_BASE   + 0x454 )) & 0x0fffffff;
        } else if (dfs_level == 0x1){
        bak_data0 = *((UINT32P)(DRAMC1_BASE   + 0x898 )) & 0x0fffffff;
        } else { //dfs_level == 0x2
        bak_data0 = *((UINT32P)(DRAMC1_BASE   + 0xc98 )) & 0x0fffffff;
        }	
        bak_data1 = u4HWTrackUICH0R1P1 & 0x1 << 2;
        bak_data0 |= bak_data1 << 29;
        bak_data1 = u4HWTrackUICH0R1P0 & 0x1 << 2;                
        bak_data0 |= bak_data1 << 28;
        bak_data1 = u4HWTrackUICH0R0P1 & 0x1 << 2;
        bak_data0 |= bak_data1 << 27;
        bak_data1 = u4HWTrackUICH0R0P0 & 0x1 << 2;                
        bak_data0 |= bak_data1 << 26;
        if (dfs_level == 0x0) {
        *((UINT32P)(DRAMC1_BASE   + 0x454 )) = bak_data0;
        } else if (dfs_level == 0x1){
        *((UINT32P)(DRAMC1_BASE   + 0x898 )) = bak_data0;
        } else { //dfs_level == 0x2
        *((UINT32P)(DRAMC1_BASE   + 0xc98 )) = bak_data0;
        }	

        //*((UINT32P)(DRAMC0_BASE   + 0x1c0 )) &= ~(0x1 << 20);           // STBSTATE_OPT = 0
        //*((UINT32P)(DRAMC1_BASE   + 0x1c0 )) &= ~(0x1 << 20);           // STBSTATE_OPT = 0	  	  
        //*((UINT32P)(DRAMC0_BASE   + 0x044 )) &= ~(0x1 << 12);           // MANUDLLFRZ = 0
        //*((UINT32P)(DRAMC1_BASE   + 0x044 )) &= ~(0x1 << 12);           // MANUDLLFRZ = 0
        
    #endif //DFS_GTDMW_SYNC

    #if DFS_GTDMW_SYNC
    //if(u4IO32ReadFldAlign(DRAMC_REG_PADCTL7, PADCTL7_GTDMW_SYNC_MASK))
    {
        mcSHOW_DBG_MSG(("GTDMW_SYNC AutoSave . \n"));
        vIO32WriteFldAlign_All(DRAMC_REG_PADCTL7, 1, PADCTL7_GTDMW_SYNC_MASK);
        vIO32WriteFldAlign_All(DRAMC_REG_PADCTL7, 0, PADCTL7_GTDMW_SYNC_MASK);
    }
    mcDELAY_US(1);
    #endif
    

	  	  *(UINT32P)DRAMC_WBR = 0x0;          	  	  
        return;
}
//! DVFS_gating_auto_save


DRAM_STATUS_T DramcRestoreGatingTrackingToRG(DRAMC_CTX_T *p)
{
#ifdef HW_GATING
    U8 u1RankIdx, u1RankMax;
    U32 u4ResultDQS_PI, u4ResultDQS_UI, u4ResultDQS_UI_P1;
    U8 u1Dqs_pi[DQS_BIT_NUMBER], u1Dqs_ui[DQS_BIT_NUMBER],u1Dqs_ui_P1[DQS_BIT_NUMBER];
    U8 cali_shu_sel;

    if(u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DQSCAL0), DQSCAL0_STBCALEN)==0)
        return DRAM_OK;

    cali_shu_sel = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DLLCONF), DLLCONF_R_OTHER_SHU_GP);
    //mcSHOW_DBG_MSG(("[DramcRestoreGatingTrackingToRG] cali_shu_sel=%d\n", cali_shu_sel));
    #ifdef DUAL_RANKS
    if(uiDualRank)
        u1RankMax = RANK_MAX;
    else
    #endif
         u1RankMax =RANK_1;

    mcSHOW_DBG_MSG(("[DramcRestoreGatingTrackingToRG] DMA = %d\n", *(volatile unsigned int *)(0x10212c08)));
    for(u1RankIdx=0; u1RankIdx<u1RankMax; u1RankIdx++)
    {
        //mcSHOW_DBG_MSG(("[DramcRestoreGatingTrackingToRG] Channel=%d, Rank=%d\n", p->channel, u1RankIdx));
        u4ResultDQS_PI = u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_R0DQSIENDLY + (u1RankIdx *4)));
        u1Dqs_pi[0]  = u4ResultDQS_PI & 0xff;
        u1Dqs_pi[1]  = (u4ResultDQS_PI >>8) & 0xff;
        u1Dqs_pi[2]  = (u4ResultDQS_PI >>16) & 0xff;
        u1Dqs_pi[3]  = (u4ResultDQS_PI >>24) & 0xff;
        if(u1Dqs_pi[0]>0x1F || u1Dqs_pi[1]>0x1F || u1Dqs_pi[2]>0x1F || u1Dqs_pi[3]>0x1F)
        {
            mcSHOW_ERR_MSG(("[DramcRestoreGatingTrackingToRG] CH%d RK%d PI status (DQS0, DQS1, DQS2, DQS3) =(%d, %d, %d, %d)\n", p->channel, u1RankIdx, u1Dqs_pi[0], u1Dqs_pi[1], u1Dqs_pi[2], u1Dqs_pi[3]));
            return DRAM_FAIL;
        }
    }

    for(u1RankIdx=0; u1RankIdx<u1RankMax; u1RankIdx++)
    {
        //mcSHOW_DBG_MSG(("[DramcRestoreGatingTrackingToRG] Channel=%d, Rank=%d\n", p->channel, u1RankIdx));
        u4ResultDQS_PI = u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_R0DQSIENDLY + (u1RankIdx *4)));
        u1Dqs_pi[0]  = u4ResultDQS_PI & 0xff;
        u1Dqs_pi[1]  = (u4ResultDQS_PI >>8) & 0xff;
        u1Dqs_pi[2]  = (u4ResultDQS_PI >>16) & 0xff;
        u1Dqs_pi[3]  = (u4ResultDQS_PI >>24) & 0xff;
        mcSHOW_DBG_MSG(("[DramcRestoreGatingTrackingToRG] PI status (DQS0, DQS1, DQS2, DQS3) =(%d, %d, %d, %d)\n", u1Dqs_pi[0], u1Dqs_pi[1], u1Dqs_pi[2], u1Dqs_pi[3]));
        
        u4ResultDQS_UI = u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_R0DQSIENUIDLY + (u1RankIdx *4)));
        u1Dqs_ui[0]  = u4ResultDQS_UI & 0xff;
        u1Dqs_ui[1]  = (u4ResultDQS_UI >>8) & 0xff;
        u1Dqs_ui[2]  = (u4ResultDQS_UI >>16) & 0xff;
        u1Dqs_ui[3]  = (u4ResultDQS_UI >>24) & 0xff;
        mcSHOW_DBG_MSG(("[DramcRestoreGatingTrackingToRG] UI status (DQS0, DQS1, DQS2, DQS3) =(%d, %d, %d, %d)\n", u1Dqs_ui[0], u1Dqs_ui[1], u1Dqs_ui[2], u1Dqs_ui[3]));

        u4ResultDQS_UI_P1 = u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_R0DQSIENUIDLY_P1 + (u1RankIdx *4)));
        u1Dqs_ui_P1[0]  = u4ResultDQS_UI_P1 & 0xff;
        u1Dqs_ui_P1[1]  = (u4ResultDQS_UI_P1 >>8) & 0xff;
        u1Dqs_ui_P1[2]  = (u4ResultDQS_UI_P1 >>16) & 0xff;
        u1Dqs_ui_P1[3]  = (u4ResultDQS_UI_P1 >>24) & 0xff;        
        //mcSHOW_DBG_MSG(("[DramcRestoreGatingTrackingToRG]UI_Phase1 status (DQS0, DQS1, DQS2, DQS3) =(%d, %d, %d, %d)\n", u1Dqs_ui_P1[0], u1Dqs_ui_P1[1], u1Dqs_ui_P1[2], u1Dqs_ui_P1[3]));

        if(cali_shu_sel==0)
        {
            vIO32Write4B(DRAMC_REG_ADDR(DRAMC_REG_R0DQSIEN + (u1RankIdx *4)), u4ResultDQS_PI);
        }
        else if(cali_shu_sel==1)
        {
            vIO32Write4B(DRAMC_REG_ADDR(DRAMC_REG_SHUFFLE25 + (u1RankIdx *4)), u4ResultDQS_PI);
        }
        else
        {
            vIO32Write4B(DRAMC_REG_ADDR(DRAMC_REG_THRD_SHUFFLE25 + (u1RankIdx *4)), u4ResultDQS_PI);
        }

        U32 u4DRAMC_REG_SELPH2[3]   = {DRAMC_REG_SELPH2  , DRAMC_REG_SHUFFLE101, DRAMC_REG_THRD_SHUFFLE101};
        U32 u4DRAMC_REG_SELPH12[3]  = {DRAMC_REG_SELPH12 , DRAMC_REG_SHUFFLE10C, DRAMC_REG_THRD_SHUFFLE10C};
        U32 u4DRAMC_REG_SELPH5[3]   = {DRAMC_REG_SELPH5  , DRAMC_REG_SHUFFLE104, DRAMC_REG_THRD_SHUFFLE104};
        U32 u4DRAMC_REG_SELPH21[3]  = {DRAMC_REG_SELPH21 , DRAMC_REG_SHUFFLE115, DRAMC_REG_THRD_SHUFFLE115};
        U32 u4DRAMC_REG_SELPH13[3]  = {DRAMC_REG_SELPH13 , DRAMC_REG_SHUFFLE10D, DRAMC_REG_THRD_SHUFFLE10D};
        U32 u4DRAMC_REG_SELPH6_1[3] = {DRAMC_REG_SELPH6_1, DRAMC_REG_SHUFFLE106, DRAMC_REG_THRD_SHUFFLE106};
        U32 u4DRAMC_REG_SELPH14[3]  = {DRAMC_REG_SELPH14 , DRAMC_REG_SHUFFLE10E, DRAMC_REG_THRD_SHUFFLE10E};
        U32 u4DRAMC_REG_SELPH15[3]  = {DRAMC_REG_SELPH15 , DRAMC_REG_SHUFFLE10F, DRAMC_REG_THRD_SHUFFLE10F};
        
        
        if(u1RankIdx==0)
        {
            vIO32WriteFldAlign(DRAMC_REG_ADDR(u4DRAMC_REG_SELPH2[cali_shu_sel]), u1Dqs_ui[0]>>3, SELPH2_TXDLY_DQSGATE);  //DQS0_P0
            vIO32WriteFldMulti(DRAMC_REG_ADDR(u4DRAMC_REG_SELPH12[cali_shu_sel]), \
                                        P_Fld((U32) u1Dqs_ui[1]>>3, SELPH12_TX_DLY_DQS1_GATED)| \
                                        P_Fld((U32) u1Dqs_ui[2]>>3, SELPH12_TX_DLY_DQS2_GATED)| \
                                        P_Fld((U32) u1Dqs_ui[3]>>3, SELPH12_TX_DLY_DQS3_GATED));
        
            vIO32WriteFldAlign(DRAMC_REG_ADDR(u4DRAMC_REG_SELPH2[cali_shu_sel]), u1Dqs_ui_P1[0]>>3, SELPH2_TXDLY_DQSGATE_P1);//DQS0_P1
            vIO32WriteFldMulti(DRAMC_REG_ADDR(u4DRAMC_REG_SELPH12[cali_shu_sel]), \
                                        P_Fld((U32) u1Dqs_ui_P1[1]>>3, SELPH12_TX_DLY_DQS1_GATED_P1)| \
                                        P_Fld((U32) u1Dqs_ui_P1[2]>>3, SELPH12_TX_DLY_DQS2_GATED_P1)| \
                                        P_Fld((U32) u1Dqs_ui_P1[3]>>3, SELPH12_TX_DLY_DQS3_GATED_P1));
        
            // 0.5T coarse tune
            vIO32WriteFldAlign(DRAMC_REG_ADDR(u4DRAMC_REG_SELPH5[cali_shu_sel]), (u1Dqs_ui[0] & 0x3), SELPH5_DLY_DQSGATE); 
            vIO32WriteFldAlign(DRAMC_REG_ADDR(u4DRAMC_REG_SELPH21[cali_shu_sel]), ((u1Dqs_ui[0]&0x7)>>2), SELPH21_DLY_DQSGATE_2); 
            vIO32WriteFldMulti(DRAMC_REG_ADDR(u4DRAMC_REG_SELPH13[cali_shu_sel]), \
                                        P_Fld((U32) u1Dqs_ui[1]&0x7, SELPH13_REG_DLY_DQS1_GATED)| \
                                        P_Fld((U32) u1Dqs_ui[2]&0x7, SELPH13_REG_DLY_DQS2_GATED)| \
                                        P_Fld((U32) u1Dqs_ui[3]&0x7, SELPH13_REG_DLY_DQS3_GATED));
        
            vIO32WriteFldAlign(DRAMC_REG_ADDR(u4DRAMC_REG_SELPH5[cali_shu_sel]), (u1Dqs_ui_P1[0] & 0x3), SELPH5_DLY_DQSGATE_P1); 
            vIO32WriteFldAlign(DRAMC_REG_ADDR(u4DRAMC_REG_SELPH21[cali_shu_sel]), ((u1Dqs_ui_P1[0]&0x7)>>2), SELPH21_DLY_DQSGATE_P1_2); 
            vIO32WriteFldMulti(DRAMC_REG_ADDR(u4DRAMC_REG_SELPH13[cali_shu_sel]), \
                                        P_Fld((U32) u1Dqs_ui_P1[1]&0x7, SELPH13_REG_DLY_DQS1_GATED_P1)| \
                                        P_Fld((U32) u1Dqs_ui_P1[2]&0x7, SELPH13_REG_DLY_DQS2_GATED_P1)| \
                                        P_Fld((U32) u1Dqs_ui_P1[3]&0x7, SELPH13_REG_DLY_DQS3_GATED_P1));
        }
        else
        {
            // 4T or 2T coarse tune
            vIO32WriteFldAlign(DRAMC_REG_ADDR(u4DRAMC_REG_SELPH6_1[cali_shu_sel]), u1Dqs_ui[0]>>3, SELPH6_1_TXDLY_R1DQSGATE);  //DQS0_P0
            vIO32WriteFldMulti(DRAMC_REG_ADDR(u4DRAMC_REG_SELPH14[cali_shu_sel]), \
                                        P_Fld((U32) u1Dqs_ui[1]>>3, SELPH12_TX_DLY_DQS1_GATED)| \
                                        P_Fld((U32) u1Dqs_ui[2]>>3, SELPH12_TX_DLY_DQS2_GATED)| \
                                        P_Fld((U32) u1Dqs_ui[3]>>3, SELPH12_TX_DLY_DQS3_GATED));
    
            vIO32WriteFldAlign(DRAMC_REG_ADDR(u4DRAMC_REG_SELPH6_1[cali_shu_sel]), u1Dqs_ui_P1[0]>>3, SELPH6_1_TXDLY_R1DQSGATE_P1);//DQS0_P1
            vIO32WriteFldMulti(DRAMC_REG_ADDR(u4DRAMC_REG_SELPH14[cali_shu_sel]), \
                                        P_Fld((U32) u1Dqs_ui_P1[1]>>3, SELPH12_TX_DLY_DQS1_GATED_P1)| \
                                        P_Fld((U32) u1Dqs_ui_P1[2]>>3, SELPH12_TX_DLY_DQS2_GATED_P1)| \
                                        P_Fld((U32) u1Dqs_ui_P1[3]>>3, SELPH12_TX_DLY_DQS3_GATED_P1));
    
            // 0.5T coarse tune
            vIO32WriteFldAlign(DRAMC_REG_ADDR(u4DRAMC_REG_SELPH6_1[cali_shu_sel]), (u1Dqs_ui[0] & 0x3), SELPH6_1_DLY_R1DQSGATE); 
            vIO32WriteFldAlign(DRAMC_REG_ADDR(u4DRAMC_REG_SELPH21[cali_shu_sel]), ((u1Dqs_ui[0]&0x7)>>2), SELPH21_DLY_R1DQSGATE_2); 
            vIO32WriteFldMulti(DRAMC_REG_ADDR(u4DRAMC_REG_SELPH15[cali_shu_sel]), \
                                        P_Fld((U32) u1Dqs_ui[1]&0x7, SELPH13_REG_DLY_DQS1_GATED)| \
                                        P_Fld((U32) u1Dqs_ui[2]&0x7, SELPH13_REG_DLY_DQS2_GATED)| \
                                        P_Fld((U32) u1Dqs_ui[3]&0x7, SELPH13_REG_DLY_DQS3_GATED));
    
            vIO32WriteFldAlign(DRAMC_REG_ADDR(u4DRAMC_REG_SELPH6_1[cali_shu_sel]), (u1Dqs_ui_P1[0] & 0x3), SELPH6_1_DLY_R1DQSGATE_P1); 
            vIO32WriteFldAlign(DRAMC_REG_ADDR(u4DRAMC_REG_SELPH21[cali_shu_sel]), ((u1Dqs_ui_P1[0]&0x7)>>2), SELPH21_DLY_R1DQSGATE_P1_2); 
            vIO32WriteFldMulti(DRAMC_REG_ADDR(u4DRAMC_REG_SELPH15[cali_shu_sel]), \
                                        P_Fld((U32) u1Dqs_ui_P1[1]&0x7, SELPH13_REG_DLY_DQS1_GATED_P1)| \
                                        P_Fld((U32) u1Dqs_ui_P1[2]&0x7, SELPH13_REG_DLY_DQS2_GATED_P1)| \
                                        P_Fld((U32) u1Dqs_ui_P1[3]&0x7, SELPH13_REG_DLY_DQS3_GATED_P1));
        }
    }
    mcSHOW_DBG_MSG(("[DramcRestoreGatingTrackingToRG] DMA = %d\n", *(volatile unsigned int *)(0x10212c08)));
#endif
    return DRAM_OK;
}
#endif  //__ETT__

void DramcSaveToShuffleReg(DRAMC_CTX_T *p, DRAM_DFS_SHUFFLE_TYPE_T dstRG)
{
	U32 Offset;
	U32 u4RegValue;
    U32 i;

    mcSHOW_DBG_MSG(("Save  frequency registers setting into shuffle register. \n"));

    //DRAMC
    for(i=0; i<CHANNEL_MAX; i++)
    {
        for (Offset = 0; Offset < FREQREG_SIZE_DRAMC; Offset++)
        {
            U32 shuffleAddr;
            U32 normalAddr = ShuffleRegTable[Offset].uiNormalAddr+((U32)i << POS_BANK_NUM);
            U32 regValue = u4IO32Read4B(normalAddr);
            if(dstRG==1)
                shuffleAddr = ShuffleRegTable[Offset].uiShuffle1Addr + ((U32)i << POS_BANK_NUM);
            else
                shuffleAddr = ShuffleRegTable[Offset].uiShuffle2Addr + ((U32)i << POS_BANK_NUM);
    
            vIO32Write4B(shuffleAddr, regValue);
        }
    }
    
    //PHY
    for(i=0; i<DDRPHY_CONF_MAX; i++)
    {
        for (Offset = FREQREG_SIZE_DRAMC; Offset < FREQREG_SIZE; Offset++)
        {
            U32 shuffleAddr;
            U32 normalAddr = ShuffleRegTable[Offset].uiNormalAddr+((U32)i << POS_BANK_NUM);
            U32 regValue = u4IO32Read4B(normalAddr);
            if(dstRG==1)
                shuffleAddr = ShuffleRegTable[Offset].uiShuffle1Addr + ((U32)i << POS_BANK_NUM);
            else
                shuffleAddr = ShuffleRegTable[Offset].uiShuffle2Addr + ((U32)i << POS_BANK_NUM);
    
            vIO32Write4B(shuffleAddr, regValue);
        }
        /***
        0x1a8[31:28] -> 0x9fc[14:11] -> 0x9fc[23:20]
        0x1a8[27:24] -> 0x9fc[10:7]  -> 0x9fc[19:16]
        0x1a8[23]    -> 0x9fc[6]     -> 0x9fc[15]
        0x41c[25:24] -> 0x9f8[25:24] -> 0x9fc[25:24]
        ***/
        {
            U32 u4DRVN_PRE, u4DRVP_PRE, u4PRE_EN, u4CAP_SEL;
            u4DRVN_PRE = u4IO32ReadFldAlign(DDRPHY_CMD10+((U32)i<<POS_BANK_NUM), CMD10_RG_TX_ARCMD_DRVN_PRE);
            u4DRVP_PRE = u4IO32ReadFldAlign(DDRPHY_CMD10+((U32)i<<POS_BANK_NUM), CMD10_RG_TX_ARCMD_DRVP_PRE);
            u4PRE_EN = u4IO32ReadFldAlign(DDRPHY_CMD10+((U32)i<<POS_BANK_NUM), CMD10_RG_TX_ARCMD_PRE_EN);
            u4CAP_SEL = u4IO32ReadFldAlign(DDRPHY_PLL8+((U32)i<<POS_BANK_NUM), PLL8_RG_RPI_CAP_SEL);
            if(dstRG==1)//Shuffle 1
            {
                vIO32WriteFldAlign(DDRPHY_SHU_R_CAP_SEL_THRD+((U32)i<<POS_BANK_NUM), u4DRVN_PRE, SHU_RG_TX_ARCMD_DRVN_PRE_SEC);
                vIO32WriteFldAlign(DDRPHY_SHU_R_CAP_SEL_THRD+((U32)i<<POS_BANK_NUM), u4DRVP_PRE, SHU_RG_TX_ARCMD_DRVP_PRE_SEC);
                vIO32WriteFldAlign(DDRPHY_SHU_R_CAP_SEL_THRD+((U32)i<<POS_BANK_NUM), u4PRE_EN, SHU_RG_TX_ARCMD_PRE_EN_SEC);
                vIO32WriteFldAlign(DDRPHY_SHU_R_CAP_SEL_SEC+((U32)i<<POS_BANK_NUM), u4CAP_SEL, SHU_RG_RPI_CAP_SEL_SEC);
            }
            else
            {
                vIO32WriteFldAlign(DDRPHY_SHU_R_CAP_SEL_THRD+((U32)i<<POS_BANK_NUM), u4DRVN_PRE, SHU_RG_TX_ARCMD_DRVN_PRE_THRD);
                vIO32WriteFldAlign(DDRPHY_SHU_R_CAP_SEL_THRD+((U32)i<<POS_BANK_NUM), u4DRVP_PRE, SHU_RG_TX_ARCMD_DRVP_PRE_THRD);
                vIO32WriteFldAlign(DDRPHY_SHU_R_CAP_SEL_THRD+((U32)i<<POS_BANK_NUM), u4PRE_EN, SHU_RG_TX_ARCMD_PRE_EN_THRD);
                vIO32WriteFldAlign(DDRPHY_SHU_R_CAP_SEL_THRD+((U32)i<<POS_BANK_NUM), u4CAP_SEL, SHU_RG_RPI_CAP_SEL_THRD);
            }
        }            
    }
}

void DramcDFSDirectJump(DRAMC_CTX_T *p, U8 pll_shu_sel, U8 mpdiv_shu_sel, U8 cali_shu_sel)
{
    U32 u4Response[2];
    U32 u4TimeCnt;
    U8 mpdivInSel=0;

    mcSHOW_DBG_MSG(("[DramcDFS] Before DVFS, disable FB_CK divide\n"));
    mcFPRINTF((fp_A60501,"[DramcDFS] Before DVFS, disable FB_CK divide\n"));
    /*TINFO="===Before DVFS, disable FB_CK divider  ==="*/
    *((volatile unsigned int *)(CLK_MEM_DFS_CFG)) = 0x0000111F;

    ///*TINFO="===Before DVFS, disable M_CK free-run divider  ==="*/
    //*DFS_MEM_DCM_CTRL &=  ~(0x3 <<0);
    ////change rate enable
    //*DFS_MEM_DCM_CTRL |= (0x1<<2);
    ////change rate disable
    //*DFS_MEM_DCM_CTRL &= ~(0x1<<2);

    mcSHOW_DBG_MSG(("[DramcDFS] Before DVFS, disable M_CK free-run divider\n"));
    mcFPRINTF((fp_A60501,"[DramcDFS] Before DVFS, disable M_CK free-run divider\n"));
    /*TINFO="===Before DVFS, disable M_CK free-run divider  ==="*/
    *((volatile unsigned int *)(DFS_MEM_DCM_CTRL)) &=  ~(0x1 <<3); //ch0
    *((volatile unsigned int *)(DFS_MEM_DCM_CTRL)) &=  ~(0x1 <<4); //ch1

    //Disable free_run fb_ck cg
    *((volatile unsigned int *)(DFS_MEM_DCM_CTRL)) &=  ~(0x1 <<9); 

    mcSHOW_DBG_MSG(("[DramcDFS] DVFS to type(%d) flow start\n", p->frequency*2));
    mcFPRINTF((fp_A60501,"[DramcDFS] DVFS to type(%d) flow start\n", p->frequency*2));
    /*TINFO="===  dvfs 1600C to 1600P flow start ==="*/
    //set MR2
    #if SUPPORT_LP3_800
    if(p->frequency<=400)
    {
        u4MR2Value = 0x18;
    }
    else
    #endif
    if(p->frequency==533)
    {
        u4MR2Value = 0x16;
    }
    else if(p->frequency == 635)
    {
        u4MR2Value = 0x18;
    }
    else if(p->frequency == 800)
    {
        u4MR2Value = 0x1a;
    }
    else //1866
    {
        u4MR2Value = 0x1c;
    }
    vIO32Write4B_All(DRAMC_REG_MRS, ((u4MR2Value<<16) | 0x00000002));

    //RISCWriteDDRPHY(0x0448 , 0x00030000);   //MPDIV_IN_SEL

    // set PHYPLL & CLRPLL to SW control
    vIO32WriteFldAlign_All(DDRPHY_PHY_SPM_CTL0, 0, Fld(2,12,AC_MSKB1));
    vIO32WriteFldAlign_All(DDRPHY_PHY_SPM_CTL1, 0, Fld(2,12,AC_MSKB1));
    vIO32WriteFldAlign_All(DDRPHY_PHY_SPM_CTL2, 0, Fld(2,12,AC_MSKB1));


    #if 0
    //if(p->dfs_type==DRAM_DFS_LJ_HIGH || DRAM_DFS_LC_HIGH)
    //    vIO32WriteFldAlign_All(DDRPHY_PLL_SHU_GP, 0, PLL_SHU_GP_PLL_SHU_GP);
    if(p->dfs_type==DRAM_DFS_LC_LOW)
        vIO32WriteFldAlign_All(DDRPHY_PLL_SHU_GP, 1, PLL_SHU_GP_PLL_SHU_GP);
    else if(p->dfs_type==DRAM_DFS_LC_SLEEP)
        vIO32WriteFldAlign_All(DDRPHY_PLL_SHU_GP, 2, PLL_SHU_GP_PLL_SHU_GP);
    #endif

    //
    if(mpdiv_shu_sel==0)
        mpdivInSel = u4IO32ReadFldAlign(DDRPHY_PLL12, PLL12_RG_ARPI_MPDIV_IN_SEL);
    else if(mpdiv_shu_sel==1)    
        mpdivInSel = u4IO32ReadFldAlign(DDRPHY_PLL19, PLL12_RG_ARPI_MPDIV_IN_SEL);
    else
        mpdivInSel = u4IO32ReadFldAlign(DDRPHY_THRD_SHUFFLE10B, PLL12_RG_ARPI_MPDIV_IN_SEL);
        

    //PLL_shu_gp
    //1. PHYPLL: no shuffle, works while PLL_SHU_GP=0
    //2. CLRPLL->PHYPLL: cannot set PLL_SHU_GP=0
    if(mpdivInSel!=0)
        vIO32WriteFldAlign_All(DDRPHY_PLL_SHU_GP, pll_shu_sel, PLL_SHU_GP_PLL_SHU_GP);
    

    //PLL en [31]
    if(mpdivInSel==0)
    {
        vIO32WriteFldAlign_All(DDRPHY_PLL1, 1, PLL1_RG_RPHYPLL_EN);
        //mcDELAY_MS(5000);
    }
    else
    {
        vIO32WriteFldAlign_All(DDRPHY_PLL3, 1, PLL3_RG_RCLRPLL_EN);
    }
        
    // wait 20us for PLL
    //*MDM_TM_WAIT_US = 20; //WAIT 20us
    //while (*MDM_TM_WAIT_US>0); //
    mcDELAY_US(22);

    #if DFS_HW_SYNC_GATING_TRACKING
    vIO32WriteFldAlign_All(DRAMC_REG_DUMMY, 1, DUMMY_ALEBLOCK);
    do
    {
        u4Response[0] = u4IO32ReadFldAlign((DRAMC_REG_SPCMDRESP), SPCMDRESP_REQQ_EMPTY);
        u4Response[1] = u4IO32ReadFldAlign((DRAMC_REG_SPCMDRESP+(1<<POS_BANK_NUM)), SPCMDRESP_REQQ_EMPTY);
    }while(u4Response[0]==0 || u4Response[1]==0);
    #endif

    //Turn on DRAMC_SHU
    mcSHOW_DBG_MSG(("DramcDFSDirectJump(shuffle start) DMA = %d\n", *(volatile unsigned int *)(0x10212c08)));
    vIO32WriteFldMulti_All(DRAMC_REG_DLLCONF, P_Fld(1, DLLCONF_DMSHU_DRAMC) | \
                P_Fld(cali_shu_sel, DLLCONF_R_OTHER_SHU_GP) | P_Fld(mpdiv_shu_sel, DLLCONF_R_MPDIV_SHU_GP));


    #if DFS_HW_SYNC_GATING_TRACKING
    vIO32WriteFldAlign_All(DRAMC_REG_DUMMY, 0, DUMMY_ALEBLOCK);
    #endif
    
    #if 0
    if(p->dfs_type==DRAM_DFS_LJ_HIGH)
    {
        vIO32WriteFldMulti_All(DRAMC_REG_DLLCONF, P_Fld(1, DLLCONF_DMSHU_DRAMC) | \
                P_Fld(0, DLLCONF_R_OTHER_SHU_GP) | P_Fld(0, DLLCONF_R_MPDIV_SHU_GP));
    }
#if !DFS_COMBINATION_TYPE3
    else if(p->dfs_type==DRAM_DFS_LC_HIGH)
    {
        vIO32WriteFldMulti_All(DRAMC_REG_DLLCONF, P_Fld(1, DLLCONF_DMSHU_DRAMC) | \
                P_Fld(0, DLLCONF_R_OTHER_SHU_GP) | P_Fld(1, DLLCONF_R_MPDIV_SHU_GP));
    }
#endif
    else if(p->dfs_type==DRAM_DFS_LC_LOW)
    {
        vIO32WriteFldMulti_All(DRAMC_REG_DLLCONF, P_Fld(1, DLLCONF_DMSHU_DRAMC) | \
                P_Fld(1, DLLCONF_R_OTHER_SHU_GP) | P_Fld(1, DLLCONF_R_MPDIV_SHU_GP));
    }
    else
    {
        vIO32WriteFldMulti_All(DRAMC_REG_DLLCONF, P_Fld(1, DLLCONF_DMSHU_DRAMC) | \
                P_Fld(2, DLLCONF_R_OTHER_SHU_GP) | P_Fld(1, DLLCONF_R_MPDIV_SHU_GP));
    }
    #endif
        
    //wait 1st shuffle_done
    //for (i=0; i < 10 ; ++i)
    //    ;
    //    wait 1us
    //*MDM_TM_WAIT_US = 1; 
    //while (*MDM_TM_WAIT_US>0); 

    //Wait SHUFFLE_END =1
    u4TimeCnt = TIME_OUT_CNT;
    do
    {
        u4Response[0] = u4IO32ReadFldAlign((DRAMC_REG_SHUFFLE), SHUFFLE_SHUFFLE_END);
        u4Response[1] = u4IO32ReadFldAlign(((DRAMC_REG_SHUFFLE) + (1<<POS_BANK_NUM)), SHUFFLE_SHUFFLE_END);
        u4TimeCnt --;
        mcDELAY_US(1);
    }while((u4Response[0]==0 || u4Response[1]==0) && (u4TimeCnt>0));

    if(u4TimeCnt==0)//time out
    {
        mcSHOW_DBG_MSG(("[DramcDFS] SHUFFLE_END Response time out)\n"));
        mcFPRINTF((fp_A60501,"[DramcDFS] SHUFFLE_END Response time out)\n"));
    }

    //Turn off DRAMC_SHU
    vIO32WriteFldAlign_All(DRAMC_REG_DLLCONF, 0, DLLCONF_DMSHU_DRAMC);
    mcDELAY_US(1);
    #if !JADE_TRACKING_MODE
    vIO32WriteFldAlign_All(DRAMC_REG_DQSCAL0, 0 ,DQSCAL0_STBSTATE_OPT);
    #endif
    vIO32WriteFldAlign_All(DRAMC_REG_TEST2_3, 0, TEST2_3_MANUDLLFRZ);
    mcSHOW_DBG_MSG(("DramcDFSDirectJump(leave shuffle) DMA = %d\n", *(volatile unsigned int *)(0x10212c08)));
    
    mcSHOW_DBG_MSG(("[DramcDFS] DVFS end\n"));
    mcFPRINTF((fp_A60501,"[DramcDFS] DVFS end\n"));
    /*TINFO="===  dvfs test end ==="*/

    mcSHOW_DBG_MSG(("[DramcDFS] After DLL lock, enable FB_CK divider End\n"));
    mcFPRINTF((fp_A60501,"[DramcDFS] After DLL lock, enable FB_CK divider End\n"));
    /*TINFO="===After DLL lock, enable FB_CK divider End   ==="*/
    *((volatile unsigned int *)(CLK_MEM_DFS_CFG)) = 0x0000101F;

    ///*TINFO="===After DVFS enable M_CK free-run div   ==="*/
    //*DFS_MEM_DCM_CTRL |=  (0x1 <<1);
    //*DFS_MEM_DCM_CTRL &=  ~(0x1 <<0);
    ////change rate enable
    //*DFS_MEM_DCM_CTRL |= (0x1<<2);
    ////change rate disable
    //*DFS_MEM_DCM_CTRL &= ~(0x1<<2);

    mcSHOW_DBG_MSG(("[DramcDFS] After DVFS enable M_CK free-run div\n"));
    mcFPRINTF((fp_A60501,"[DramcDFS] After DVFS enable M_CK free-run div\n"));
    /*TINFO="===After DVFS enable M_CK free-run div   ==="*/
    *((volatile unsigned int *)(DFS_MEM_DCM_CTRL)) |=  (0x1 <<3); //ch0
    *((volatile unsigned int *)(DFS_MEM_DCM_CTRL)) |=  (0x1 <<4); //ch1
            
    //Enable free_run fb_ck cg
    *((volatile unsigned int *)(DFS_MEM_DCM_CTRL)) |=  (0x1 <<9); 

    //Turn off CLRPLL
    //PLL en [31] = 0
    if(mpdivInSel==0)
    {
        vIO32WriteFldAlign_All(DDRPHY_PLL3, 0, PLL3_RG_RCLRPLL_EN);
        //set PHYPLL to HW control
        vIO32WriteFldAlign_All(DDRPHY_PHY_SPM_CTL0, 0x1, Fld(2,12,AC_MSKB1));
        vIO32WriteFldAlign_All(DDRPHY_PHY_SPM_CTL1, 0x1, Fld(2,12,AC_MSKB1));
        vIO32WriteFldAlign_All(DDRPHY_PHY_SPM_CTL2, 0x1, Fld(2,12,AC_MSKB1));
    }
    else
    {
            #if CHECK_PLL_OK
            if(is_pll_good() == 1) //all pll is good, use auto k band
            #endif
            {
                vIO32WriteFldAlign_All(DDRPHY_PLL1, 0, PLL1_RG_RPHYPLL_EN);
            }        
        //set CLRPLL to HW control
        vIO32WriteFldAlign_All(DDRPHY_PHY_SPM_CTL0, 0x2, Fld(2,12,AC_MSKB1));
        vIO32WriteFldAlign_All(DDRPHY_PHY_SPM_CTL1, 0x2, Fld(2,12,AC_MSKB1));
        vIO32WriteFldAlign_All(DDRPHY_PHY_SPM_CTL2, 0x2, Fld(2,12,AC_MSKB1));
    }

    return;
}

#if DFS_SUPPORT_THIRD_SHUFFLE
void DFSPLLSettingForLP800(DRAMC_CTX_T *p)
{
    vIO32WriteFldMulti_All(DDRPHY_THRD_SHUFFLE10B, P_Fld(0x0,PLL12_RG_ARPI_MPDIV_IN_SEL)|P_Fld(0x0, PLL12_RG_BRPI_MPDIV_IN_SEL));
    vIO32WriteFldMulti_All(DDRPHY_THRD_SHUFFLE10B, P_Fld(0x3,PLL12_RG_ARPI_MPDIV_SEL)|P_Fld(0x3, PLL12_RG_BRPI_MPDIV_SEL));
    vIO32WriteFldMulti_All(DDRPHY_THRD_SHUFFLE10B, P_Fld(0x1,PLL12_RG_ARPI_MPDIV_OUT_SEL)|P_Fld(0x1, PLL12_RG_BRPI_MPDIV_OUT_SEL));
    vIO32WriteFldMulti_All(DDRPHY_THRD_SHUFFLE10B, P_Fld(0x1,PLL12_RG_ARPI_MPDIV_SEL)|P_Fld(0x1, PLL12_RG_BRPI_MPDIV_SEL));
}
#endif

void DFSSwitchFreq(DRAMC_CTX_T *p)
{
    //DramcEnterSelfRefresh(p, 1);
    DDRPhyPLLSetting(p);

    //MPDIV init
    if(p->freq_sel < LJ_MAX_SEL)
    {
        vIO32WriteFldMulti_All(DDRPHY_PLL12, P_Fld(0x0,PLL12_RG_ARPI_MPDIV_IN_SEL)|P_Fld(0x0, PLL12_RG_BRPI_MPDIV_IN_SEL));
    }
    else
    {
        vIO32WriteFldMulti_All(DDRPHY_PLL12, P_Fld(0x1,PLL12_RG_ARPI_MPDIV_IN_SEL)|P_Fld(0x1, PLL12_RG_BRPI_MPDIV_IN_SEL));
    }
    vIO32WriteFldMulti_All(DDRPHY_PLL12, P_Fld(0x3,PLL12_RG_ARPI_MPDIV_SEL)|P_Fld(0x3, PLL12_RG_BRPI_MPDIV_SEL));
    mcDELAY_US(1);
    if(p->freq_sel==LJ_DDR933 || p->freq_sel==LJ_DDR850 || p->freq_sel==LJ_DDR800)
    //|| p->freq_sel==LC_DDR533 || p->freq_sel==LC_DDR635 || p->freq_sel==LC_DDR800)
    {
        vIO32WriteFldMulti_All(DDRPHY_PLL12, P_Fld(0x1,PLL12_RG_ARPI_MPDIV_OUT_SEL)|P_Fld(0x1, PLL12_RG_BRPI_MPDIV_OUT_SEL));
        mcDELAY_US(1);
        vIO32WriteFldMulti_All(DDRPHY_PLL12, P_Fld(0x1,PLL12_RG_ARPI_MPDIV_SEL)|P_Fld(0x1, PLL12_RG_BRPI_MPDIV_SEL));
    }        
    else
    {
        vIO32WriteFldMulti_All(DDRPHY_PLL12, P_Fld(0x0,PLL12_RG_ARPI_MPDIV_OUT_SEL)|P_Fld(0x0, PLL12_RG_BRPI_MPDIV_OUT_SEL));
        mcDELAY_US(1);
        vIO32WriteFldMulti_All(DDRPHY_PLL12, P_Fld(0x0,PLL12_RG_ARPI_MPDIV_SEL)|P_Fld(0x0, PLL12_RG_BRPI_MPDIV_SEL));
    }
    
    //DLL init
    
	//DLL Init flow
	vIO32WriteFldAlign_All(DDRPHY_PLL13, 0, PLL13_RG_ARDLL_PHDET_EN);
    vIO32WriteFldAlign_All(DDRPHY_PLL13, 0, PLL14_RG_BRDLL_PHDET_EN);
	vIO32WriteFldAlign_All(DDRPHY_PLL16, 0, PLL16_RG_RPHYPLL_RESETB);
	mcDELAY_US(1);
	vIO32WriteFldAlign_All(DDRPHY_PLL16, 1, PLL16_RG_RPHYPLL_RESETB);
	mcDELAY_US(1);

	//vIO32WriteFldAlign_All(DDRPHY_PLL13, 1, PLL13_RG_ARDLL_PHDET_EN);
    vIO32WriteFldAlign(DDRPHY_PLL13, 1, PLL13_RG_ARDLL_PHDET_EN);
    mcDELAY_US(1);
    vIO32WriteFldAlign(DDRPHY_PLL13+(1<<POS_BANK_NUM), 1, PLL13_RG_ARDLL_PHDET_EN);
    vIO32WriteFldAlign(DDRPHY_PLL13+(2<<POS_BANK_NUM), 1, PLL13_RG_ARDLL_PHDET_EN);
    vIO32WriteFldAlign(DDRPHY_PLL13+(3<<POS_BANK_NUM), 1, PLL13_RG_ARDLL_PHDET_EN);


    
	mcDELAY_US(1);
	vIO32WriteFldAlign_All(DDRPHY_PLL14, 1, PLL14_RG_BRDLL_PHDET_EN);
	mcDELAY_US(1);	//DLL Ready
	
    //DramcEnterSelfRefresh(p, 0);
}

#if 0
void DFSSetupFlow(DRAMC_CTX_T *p)
{
    // calibration of LC_SLEEP is done. save to shuffle2
    DramcSaveToShuffleReg(p, DRAM_DFS_SHUFFLE_2);

    p->frequency = DFS_FREQ_LC_LOW;
    p->dfs_type = DRAM_DFS_LC_LOW;
    DFSSwitchFreq(p);
    DFSCalibration(p);
    DramcSaveToShuffleReg(p, DRAM_DFS_SHUFFLE_1);
  
    p->frequency = DFS_FREQ_LC_HIGH;
    p->dfs_type = DRAM_DFS_LC_HIGH;
    DFSSwitchFreq(p);
    DFSCalibration(p);


    //DramcRestoreBackup(p);
    //DramcDumpFreqSetting(p);
}
#endif
#endif  // DUAL_FREQ_K


// both "ENABLE_RANK_NUMBER_AUTO_DETECTION" and "DUAL_FREQ_K" use this API
void DFSInitForCalibration(DRAMC_CTX_T *p)
{
    if(p->dram_type == TYPE_LPDDR4)
    {
        //DramcModeRegInit_LP4(p);  //function removed
    }
    else
    {
        DdrPhySetting_Everest_LP3(p);
        DramcInit(p);	
    }
}


void DramcHWGatingInit(DRAMC_CTX_T *p)
{
    #ifdef HW_GATING
    #if JADE_TRACKING_MODE
    vIO32WriteFldMulti_All(DRAMC_REG_ADDR(DRAMC_REG_DQSCAL0), P_Fld(0, DQSCAL0_STBCALEN) | \
                                            P_Fld(0, DQSCAL0_R0DQSIENLLMTEN)| P_Fld(0, DQSCAL0_R0DQSIENHLMTEN) | \
                                            P_Fld(0, DQSCAL0_STBCAL2R)| P_Fld(1, DQSCAL0_STB_SELPHYCALEN) | \
                                            P_Fld(1, DQSCAL0_DQSG_MODE)| P_Fld(1, DQSCAL0_STBSTATE_OPT) | \
                                            P_Fld(0, DQSCAL0_RKCHGMASKDIS)| P_Fld(1, DQSCAL0_REFUICHG) | \
                                            P_Fld(1, DQSCAL0_PICGEN));
    #else    
    vIO32WriteFldMulti_All(DRAMC_REG_ADDR(DRAMC_REG_DQSCAL0), P_Fld(0, DQSCAL0_STBCALEN) | \
                                            P_Fld(0, DQSCAL0_R0DQSIENLLMTEN)| P_Fld(0, DQSCAL0_R0DQSIENHLMTEN) | \
                                            P_Fld(0, DQSCAL0_STBCAL2R)| P_Fld(1, DQSCAL0_STB_SELPHYCALEN) | \
                                            P_Fld(1, DQSCAL0_DQSG_MODE)| P_Fld(0, DQSCAL0_STBSTATE_OPT) | \
                                            P_Fld(0, DQSCAL0_RKCHGMASKDIS)| P_Fld(0, DQSCAL0_REFUICHG) | \
                                            P_Fld(1, DQSCAL0_PICGEN));

    #endif

    vIO32WriteFldMulti_All(DRAMC_REG_ADDR(DRAMC_REG_DUMMY),  \
                                            P_Fld(1, DUMMY_PICGLAT)| P_Fld(1, DUMMY_DQSIENCG_CHG_EN));
    
    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_RESERVED_DRAMC_1), 0, RESERVED_DRAMC_1_CG_RKEN);
    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_CLKCTRL), 1, CLKCTRL_DQSIENCG_NORMAL_EN);
    
    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DDRPHY_PINMUX), 1, PINMUX_R_DMDQSIENCG_EN);
    #endif
}


void DramcHWGatingOnOff(DRAMC_CTX_T *p, U8 u1OnOff)
{
    #ifdef HW_GATING
    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_DQSCAL0), u1OnOff, DQSCAL0_STBCALEN);
    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_DQSCAL0), u1OnOff, DQSCAL0_STB_SELPHYCALEN);
    #else
    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_DQSCAL0), 0, DQSCAL0_STBCALEN);   // PI tracking off = HW gating tracking off
    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_DQSCAL0), 0, DQSCAL0_STB_SELPHYCALEN);
    #endif
}

#if (__ETT__ || CPU_RW_TEST_AFTER_K)

U16 u2MaxGatingPos[CHANNEL_MAX][RANK_MAX][DQS_NUMBER]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
U16 u2MinGatingPos[CHANNEL_MAX][RANK_MAX][DQS_NUMBER]={0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

void DramcPrintHWGatingStatus(DRAMC_CTX_T *p, U8 u1Channel)
{
#ifdef HW_GATING
    U8 u1RankIdx, u1RankMax, u1ChannelBak, u1ByteIdx, ii;
    U32 u4ResultDQS_PI, u4ResultDQS_UI, u4ResultDQS_UI_P1;
    U8 u1Dqs_pi[DQS_BIT_NUMBER], u1Dqs_ui[DQS_BIT_NUMBER],u1Dqs_ui_P1[DQS_BIT_NUMBER];
    U16 u2TmpValue, u2TmpUI[DQS_NUMBER], u2TmpPI[DQS_NUMBER];
    U32 MANUDLLFRZ_bak, STBSTATE_OPT_bak;

    u1ChannelBak= p->channel;
    p->channel = u1Channel;
    vSetPHY2ChannelMapping(p, u1Channel);
    
    #ifdef DUAL_RANKS
    if(uiDualRank)
        u1RankMax = RANK_MAX;
    else
    #endif
         u1RankMax =RANK_1;

    MANUDLLFRZ_bak = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_TEST2_3), TEST2_3_MANUDLLFRZ);
    STBSTATE_OPT_bak = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DQSCAL0), DQSCAL0_STBSTATE_OPT);

    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_TEST2_3), 1, TEST2_3_MANUDLLFRZ);
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DQSCAL0), 1, DQSCAL0_STBSTATE_OPT);
    
    for(u1RankIdx=0; u1RankIdx<u1RankMax; u1RankIdx++)
    {
        mcSHOW_DBG_MSG(("[DramcHWGatingStatus] Channel=%d, Rank=%d\n", p->channel, u1RankIdx));
        u4ResultDQS_PI = u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_R0DQSIENDLY + (u1RankIdx *4)));
        u1Dqs_pi[0]  = u4ResultDQS_PI & 0xff;
        u1Dqs_pi[1]  = (u4ResultDQS_PI >>8) & 0xff;
        u1Dqs_pi[2]  = (u4ResultDQS_PI >>16) & 0xff;
        u1Dqs_pi[3]  = (u4ResultDQS_PI >>24) & 0xff;
        //mcSHOW_DBG_MSG(("[DramcHWGatingStatus] PI status (DQS0, DQS1, DQS2, DQS3) =(%d, %d, %d, %d)\n", u1Dqs_pi[0], u1Dqs_pi[1], u1Dqs_pi[2], u1Dqs_pi[3]));
        
        u4ResultDQS_UI = u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_R0DQSIENUIDLY + (u1RankIdx *4)));
        u1Dqs_ui[0]  = u4ResultDQS_UI & 0xff;
        u1Dqs_ui[1]  = (u4ResultDQS_UI >>8) & 0xff;
        u1Dqs_ui[2]  = (u4ResultDQS_UI >>16) & 0xff;
        u1Dqs_ui[3]  = (u4ResultDQS_UI >>24) & 0xff;
        //mcSHOW_DBG_MSG(("[DramcHWGatingStatus] UI status (DQS0, DQS1, DQS2, DQS3) =(%d, %d, %d, %d)\n", u1Dqs_ui[0], u1Dqs_ui[1], u1Dqs_ui[2], u1Dqs_ui[3]));
        mcSHOW_DBG_MSG((" Byte0(2T, 0.5T, PI) =(%d, %d, %d)\n Byte1(2T, 0.5T, PI) =(%d, %d, %d)\n Byte2(2T, 0.5T, PI) =(%d, %d, %d)\n Byte3(2T, 0.5T, PI) =(%d, %d, %d)\n", \
                                        u1Dqs_ui[0]/8, u1Dqs_ui[0]%8, u1Dqs_pi[0], u1Dqs_ui[1]/8, u1Dqs_ui[1]%8, u1Dqs_pi[1], \
                                        u1Dqs_ui[2]/8, u1Dqs_ui[2]%8, u1Dqs_pi[2], u1Dqs_ui[3]/8, u1Dqs_ui[3]%8, u1Dqs_pi[3]));

        u4ResultDQS_UI_P1 = u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_R0DQSIENUIDLY_P1 + (u1RankIdx *4)));
        u1Dqs_ui_P1[0]  = u4ResultDQS_UI_P1 & 0xff;
        u1Dqs_ui_P1[1]  = (u4ResultDQS_UI_P1 >>8) & 0xff;
        u1Dqs_ui_P1[2]  = (u4ResultDQS_UI_P1 >>16) & 0xff;
        u1Dqs_ui_P1[3]  = (u4ResultDQS_UI_P1 >>24) & 0xff;        
        mcSHOW_DBG_MSG((" UI_Phase1 (DQS0~3) =(%d, %d, %d, %d)\n\n", u1Dqs_ui_P1[0], u1Dqs_ui_P1[1], u1Dqs_ui_P1[2], u1Dqs_ui_P1[3]));

#if 0  // max and min gating position, still need test.
        for(u1ByteIdx=0; u1ByteIdx< DQS_NUMBER; u1ByteIdx++)
        {
            u2TmpValue = (u1Dqs_ui[u1ByteIdx]*32+ u1Dqs_pi[u1ByteIdx]);
            
            if(u2TmpValue > u2MaxGatingPos[p->channel][u1RankIdx][u1ByteIdx])
            {
                u2MaxGatingPos[p->channel][u1RankIdx][u1ByteIdx] = u2TmpValue;
           
                for(ii=0; ii< DQS_NUMBER; ii++)
                {
                    u2TmpUI[ii] = u2MaxGatingPos[p->channel][u1RankIdx][u1ByteIdx] /32;
                    u2TmpPI[ii] = u2MaxGatingPos[p->channel][u1RankIdx][u1ByteIdx] %32;
                }
                
                mcSHOW_DBG_MSG((" Gating udpate max Rank%d, Byte%d: \n Byte0(2T, 0.5T, PI) =(%d, %d, %d)\n Byte1(2T, 0.5T, PI) =(%d, %d, %d)\n Byte2(2T, 0.5T, PI) =(%d, %d, %d)\n Byte3(2T, 0.5T, PI) =(%d, %d, %d)\n", \
                                                u1RankIdx, u1ByteIdx, \
                                                u2TmpUI[0]/8, u2TmpUI[0]%8, u1Dqs_pi[0], u2TmpUI[1]/8, u2TmpUI[1]%8, u1Dqs_pi[1], \
                                                u2TmpUI[2]/8, u2TmpUI[2]%8, u1Dqs_pi[2], u2TmpUI[3]/8, u2TmpUI[3]%8, u1Dqs_pi[3]));
            }

            if(u2TmpValue < u2MinGatingPos[p->channel][u1RankIdx][u1ByteIdx])
            {
                u2MinGatingPos[p->channel][u1RankIdx][u1ByteIdx] = u2TmpValue;
                
                for(ii=0; ii< DQS_NUMBER; ii++)
                {
                    u2TmpUI[ii] = u2MinGatingPos[p->channel][u1RankIdx][u1ByteIdx] /32;
                    u2TmpPI[ii] = u2MinGatingPos[p->channel][u1RankIdx][u1ByteIdx] %32;
    }

                mcSHOW_DBG_MSG((" Gating udpate min Rank%d, Byte%d: \n Byte0(2T, 0.5T, PI) =(%d, %d, %d)\n Byte1(2T, 0.5T, PI) =(%d, %d, %d)\n Byte2(2T, 0.5T, PI) =(%d, %d, %d)\n Byte3(2T, 0.5T, PI) =(%d, %d, %d)\n", \
                                                u1RankIdx, u1ByteIdx, \
                                                u2TmpUI[0]/8, u2TmpUI[0]%8, u1Dqs_pi[0], u2TmpUI[1]/8, u2TmpUI[1]%8, u1Dqs_pi[1], \
                                                u2TmpUI[2]/8, u2TmpUI[2]%8, u1Dqs_pi[2], u2TmpUI[3]/8, u2TmpUI[3]%8, u1Dqs_pi[3]));
            }
        }           
#endif
    }
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DQSCAL0), STBSTATE_OPT_bak, DQSCAL0_STBSTATE_OPT);
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_TEST2_3), MANUDLLFRZ_bak, TEST2_3_MANUDLLFRZ);
   
    p->channel = u1ChannelBak;
    vSetPHY2ChannelMapping(p, u1ChannelBak);
#endif
}
#endif  //#if (__ETT__ || CPU_RW_TEST_AFTER_K)


U8 u1GetMR4RefreshRate(DRAM_CHANNEL_T channel)
{
    U8 u1Dummy, u1RefreshRate;
    
    //DramcModeRegRead(p, 4, &u1Dummy);
    //mcSHOW_DBG_MSG(("[u2GetRefreshRate] MR4 0x%x,  u1RefreshRate= 0x%x\n", u1Dummy, u1RefreshRate));
    u1RefreshRate = (U8)u4IO32ReadFldAlign(((channel << POS_BANK_NUM)+ DRAMC_REG_SPCMDRESP), SPCMDRESP_REFRESH_RATE);
    //mcSHOW_DBG_MSG(("[u2GetRefreshRate] channel = %d, u1RefreshRate= 0x%x\n", channel, u1RefreshRate));

    return u1RefreshRate;
}

#if RX_DLY_TRACKING_VERIFY
/***
1. set DQS*_DVS_DLY=0, then log calibration result 
2. set DQS*_DVS_DLY=4b'1xxxx, then log calibration result
3. compare 1 & 2 Rx delay
***/
void DramcRxInputDlyTrackingRG(DRAMC_CTX_T *p)
{
    DRAM_CHANNEL_T channel_bak = p->channel;
    vSetPHY2ChannelMapping(p, CHANNEL_A);

    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR1),0x0, RXDLY_TR1_R_RK0_DVS_MODE);
    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR11),0x0, RXDLY_TR11_R_RK1_DVS_MODE);

    //RG Mode : DQS*_DVS_DLY=0x7 -> DQ delay cell + 10 (experiment result)
    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE2),0xF, EYE2_RG_RX_ARDQS0_DVS_DLY);
    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1_2),0xF, EYEB1_2_RG_RX_ARDQS1_DVS_DLY);

    vSetPHY2ChannelMapping(p, channel_bak);
}

void DramcPrintRxInputDlyTrackingFlag(DRAMC_CTX_T *p)
{
    U8 u1RankIdx, u1RankMax;
    UINT8 dqsDly=0;
    UINT8 u1ByteIdx=0;

#ifdef DUAL_RANKS
    if(uiDualRank)
        u1RankMax = RANK_MAX;
    else
    #endif
        u1RankMax =RANK_1;

    for(u1RankIdx=0; u1RankIdx<u1RankMax; u1RankIdx++)
    {  
        for(u1ByteIdx = 0; u1ByteIdx < (p->data_width/DQS_BIT_NUMBER); u1ByteIdx++)
        {
            INT8 u1BitIdx=0;
            
            vIO32WriteFldAlign(DDRPHY_RXDLY_TR0+(aru1PhyMap2Channel[u1ByteIdx%2]<< POS_BANK_NUM), (u1RankIdx<<1)|(u1ByteIdx>>1),RXDLY_TR0_R_RX_DLY_TRACK_RO_SEL);
            mcSHOW_DBG_MSG(("  Rank%d Byte%d ro_sel=%d lead count bit(7~0) = ( ", u1RankIdx, u1ByteIdx, (u1RankIdx<<1)|(u1ByteIdx>>1)));
            for(u1BitIdx=7; u1BitIdx>=0; u1BitIdx--)
            {
                mcSHOW_DBG_MSG(("%H ", u4IO32ReadFldAlign(DDRPHY_RXDLY_TRRO0+u1BitIdx*4+(aru1PhyMap2Channel[u1ByteIdx%2]<< POS_BANK_NUM), RXDLY_TRRO0_DVS_RKX_BX_SW_LEAD_CNT_OUT_B0)));
            }
            mcSHOW_DBG_MSG((")\n"));
        }
    }
    for(u1RankIdx=0; u1RankIdx<u1RankMax; u1RankIdx++)
    {  
        for(u1ByteIdx = 0; u1ByteIdx < (p->data_width/DQS_BIT_NUMBER); u1ByteIdx++)
        {
            INT8 u1BitIdx=0;
            
            vIO32WriteFldAlign(DDRPHY_RXDLY_TR0+(aru1PhyMap2Channel[u1ByteIdx%2]<< POS_BANK_NUM), (u1RankIdx<<1)|(u1ByteIdx>>1),RXDLY_TR0_R_RX_DLY_TRACK_RO_SEL);
            mcSHOW_DBG_MSG(("  Rank%d Byte%d ro_sel=%d lag count bit(7~0) =  ( ", u1RankIdx, u1ByteIdx, (u1RankIdx<<1)|(u1ByteIdx>>1)));
            for(u1BitIdx=7; u1BitIdx>=0; u1BitIdx--)
            {
                mcSHOW_DBG_MSG(("%H ", u4IO32ReadFldAlign(DDRPHY_RXDLY_TRRO0+u1BitIdx*4+(aru1PhyMap2Channel[u1ByteIdx%2]<< POS_BANK_NUM), RXDLY_TRRO0_DVS_RKX_BX_SW_LAG_CNT_OUT_B0)));
            }
            mcSHOW_DBG_MSG((")\n"));
        }
    }
    for(u1RankIdx=0; u1RankIdx<u1RankMax; u1RankIdx++)
    {  
        for(u1ByteIdx = 0; u1ByteIdx < (p->data_width/DQS_BIT_NUMBER); u1ByteIdx++)
        {
            INT8 u1BitIdx=0;
            
            vIO32WriteFldAlign(DDRPHY_RXDLY_TR0+(aru1PhyMap2Channel[u1ByteIdx%2]<< POS_BANK_NUM), (u1RankIdx<<1)|(u1ByteIdx>>1),RXDLY_TR0_R_RX_DLY_TRACK_RO_SEL);
            mcSHOW_DBG_MSG(("  Rank%d Byte%d ro_sel=%d both count bit(7~0) =  ( ", u1RankIdx, u1ByteIdx, (u1RankIdx<<1)|(u1ByteIdx>>1)));
            for(u1BitIdx=7; u1BitIdx>=0; u1BitIdx-=2)
            {
                mcSHOW_DBG_MSG(("%H ", u4IO32ReadFldAlign(DDRPHY_RXDLY_TRRO9+(u1BitIdx/2)*4+(aru1PhyMap2Channel[u1ByteIdx%2]<< POS_BANK_NUM), RXDLY_TRRO9_DVS_RKX_BX_LEAD_LAG_CNT_OUT_B1)));
                mcSHOW_DBG_MSG(("%H ", u4IO32ReadFldAlign(DDRPHY_RXDLY_TRRO9+(u1BitIdx/2)*4+(aru1PhyMap2Channel[u1ByteIdx%2]<< POS_BANK_NUM), RXDLY_TRRO9_DVS_RKX_BX_LEAD_LAG_CNT_OUT_B0)));
            }
            mcSHOW_DBG_MSG((")\n"));
        }
    }
    #if 0
    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR7),0x1, RXDLY_TR7_R_RK0_B0_DVS_SW_CNT_CLR);
    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR9),0x1, RXDLY_TR9_R_RK0_B1_DVS_SW_CNT_CLR);
    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR17),0x1, RXDLY_TR17_R_RK1_B0_DVS_SW_CNT_CLR);
    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR19),0x1, RXDLY_TR19_R_RK1_B1_DVS_SW_CNT_CLR);
    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR7),0x0, RXDLY_TR7_R_RK0_B0_DVS_SW_CNT_CLR);
    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR9),0x0, RXDLY_TR9_R_RK0_B1_DVS_SW_CNT_CLR);
    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR17),0x0, RXDLY_TR17_R_RK1_B0_DVS_SW_CNT_CLR);
    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR19),0x0, RXDLY_TR19_R_RK1_B1_DVS_SW_CNT_CLR);
    #endif

    for(u1RankIdx=0; u1RankIdx<u1RankMax; u1RankIdx++)
    {  
        for(u1ByteIdx = 0; u1ByteIdx < (p->data_width/DQS_BIT_NUMBER); u1ByteIdx++)
        {
            INT8 u1BitIdx=0;
            
            vIO32WriteFldAlign(DDRPHY_RXDLY_TR0+(aru1PhyMap2Channel[u1ByteIdx%2]<< POS_BANK_NUM), (u1RankIdx<<1)|(u1ByteIdx>>1),RXDLY_TR0_R_RX_DLY_TRACK_RO_SEL);
            mcSHOW_DBG_MSG(("  Rank%d Byte%d ro_sel=%d th_count_out bit(7~0) =  ( ", u1RankIdx, u1ByteIdx, (u1RankIdx<<1)|(u1ByteIdx>>1)));
            for(u1BitIdx=7; u1BitIdx>=0; u1BitIdx-=2)
            {
                mcSHOW_DBG_MSG(("%H ", u4IO32ReadFldAlign(DDRPHY_RXDLY_TRRO15+(u1BitIdx/2)*4+(aru1PhyMap2Channel[u1ByteIdx%2]<< POS_BANK_NUM), RXDLY_TRRO15_DVS_RKX_BX_TH_CNT_OUT_B1)));
                mcSHOW_DBG_MSG(("%H ", u4IO32ReadFldAlign(DDRPHY_RXDLY_TRRO15+(u1BitIdx/2)*4+(aru1PhyMap2Channel[u1ByteIdx%2]<< POS_BANK_NUM), RXDLY_TRRO15_DVS_RKX_BX_TH_CNT_OUT_B0)));
            }
            mcSHOW_DBG_MSG((")\n"));
        }
    }
    
    vSetPHY2ChannelMapping(p, CHANNEL_A);
    //DramcGatingDebug(p, CHANNEL_A);
}

void DramcPrintRxInputDly(DRAMC_CTX_T *p)
{
    DRAM_CHANNEL_T channel_bak = p->channel;
    UINT8 dqsDly=0;
    UINT8 u1ByteIdx=0;

    vSetPHY2ChannelMapping(p, CHANNEL_A);

    mcSHOW_DBG_MSG(("RK0 DQS0-3 R(%d, %d, %d, %d) F(%d, %d, %d, %d)\n",
        u4IO32ReadFldAlign(DDRPHY_RXDQ5+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), RXDQ5_RK0_RX_ARDQS0_R_DLY),
        u4IO32ReadFldAlign(DDRPHY_RXDQ5+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), RXDQ5_RK0_RX_ARDQS0_R_DLY),
        u4IO32ReadFldAlign(DDRPHY_RXB1_5+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), RXB1_5_RK0_RX_ARDQS1_R_DLY),
        u4IO32ReadFldAlign(DDRPHY_RXB1_5+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), RXB1_5_RK0_RX_ARDQS1_R_DLY),
        u4IO32ReadFldAlign(DDRPHY_RXDQ5+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), RXDQ5_RK0_RX_ARDQS0_F_DLY),
        u4IO32ReadFldAlign(DDRPHY_RXDQ5+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), RXDQ5_RK0_RX_ARDQS0_F_DLY),
        u4IO32ReadFldAlign(DDRPHY_RXB1_5+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), RXB1_5_RK0_RX_ARDQS1_F_DLY),
        u4IO32ReadFldAlign(DDRPHY_RXB1_5+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), RXB1_5_RK0_RX_ARDQS1_F_DLY)));

    for(u1ByteIdx = 0; u1ByteIdx < (p->data_width/DQS_BIT_NUMBER); u1ByteIdx++)
    {
        U32 DQ_DLY[4];
        DQ_DLY[0] = u4IO32ReadFldAlign(DDRPHY_RXDQ1+0x60*(u1ByteIdx/2)+(aru1PhyMap2Channel[u1ByteIdx%2]<< POS_BANK_NUM), PHY_FLD_FULL),
        DQ_DLY[1] = u4IO32ReadFldAlign(DDRPHY_RXDQ2+0x60*(u1ByteIdx/2)+(aru1PhyMap2Channel[u1ByteIdx%2]<< POS_BANK_NUM), PHY_FLD_FULL),
        DQ_DLY[2] = u4IO32ReadFldAlign(DDRPHY_RXDQ3+0x60*(u1ByteIdx/2)+(aru1PhyMap2Channel[u1ByteIdx%2]<< POS_BANK_NUM), PHY_FLD_FULL),
        DQ_DLY[3] = u4IO32ReadFldAlign(DDRPHY_RXDQ4+0x60*(u1ByteIdx/2)+(aru1PhyMap2Channel[u1ByteIdx%2]<< POS_BANK_NUM), PHY_FLD_FULL),
        mcSHOW_DBG_MSG(("RK0 B%d R(7~0) (%d, %d, %d, %d, %d, %d, %d, %d)\n", u1ByteIdx,
            (DQ_DLY[3]>>8)&0xff, (DQ_DLY[3]>>24)&0xff, (DQ_DLY[2]>>8)&0xff, (DQ_DLY[2]>>24)&0xff, 
            (DQ_DLY[1]>>8)&0xff, (DQ_DLY[1]>>24)&0xff, (DQ_DLY[0]>>8)&0xff, (DQ_DLY[0]>>24)&0xff));
        mcSHOW_DBG_MSG(("RK0 B%d F(7~0) (%d, %d, %d, %d, %d, %d, %d, %d)\n", u1ByteIdx,
            (DQ_DLY[3]>>0)&0xff, (DQ_DLY[3]>>16)&0xff, (DQ_DLY[2]>>0)&0xff, (DQ_DLY[2]>>16)&0xff, 
            (DQ_DLY[1]>>0)&0xff, (DQ_DLY[1]>>16)&0xff, (DQ_DLY[0]>>0)&0xff, (DQ_DLY[0]>>16)&0xff));
    }
    
#ifdef DUAL_RANKS
    if(uiDualRank)
    {
        mcSHOW_DBG_MSG(("RK1 DQS0-3 R(%d, %d, %d, %d) F(%d, %d, %d, %d)\n",
        u4IO32ReadFldAlign(DDRPHY_RXDQ10+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), RXDQ10_RK1_RX_ARDQS0_R_DLY),
        u4IO32ReadFldAlign(DDRPHY_RXDQ10+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), RXDQ10_RK1_RX_ARDQS0_R_DLY),
        u4IO32ReadFldAlign(DDRPHY_RXRK1_B1_4+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), RXRK1_B1_4_RK1_RX_ARDQS1_R_DLY),
        u4IO32ReadFldAlign(DDRPHY_RXRK1_B1_4+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), RXRK1_B1_4_RK1_RX_ARDQS1_R_DLY),
        u4IO32ReadFldAlign(DDRPHY_RXDQ10+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), RXDQ10_RK1_RX_ARDQS0_F_DLY),
        u4IO32ReadFldAlign(DDRPHY_RXDQ10+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), RXDQ10_RK1_RX_ARDQS0_F_DLY),
        u4IO32ReadFldAlign(DDRPHY_RXRK1_B1_4+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), RXRK1_B1_4_RK1_RX_ARDQS1_F_DLY),
        u4IO32ReadFldAlign(DDRPHY_RXRK1_B1_4+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), RXRK1_B1_4_RK1_RX_ARDQS1_F_DLY)));

        for(u1ByteIdx = 0; u1ByteIdx < (p->data_width/DQS_BIT_NUMBER); u1ByteIdx++)
        {
            U32 DQ_DLY[4];
            DQ_DLY[0] = u4IO32ReadFldAlign(DDRPHY_RXDQ6+0x60*(u1ByteIdx/2)+(aru1PhyMap2Channel[u1ByteIdx%2]<< POS_BANK_NUM), PHY_FLD_FULL),
            DQ_DLY[1] = u4IO32ReadFldAlign(DDRPHY_RXDQ7+0x60*(u1ByteIdx/2)+(aru1PhyMap2Channel[u1ByteIdx%2]<< POS_BANK_NUM), PHY_FLD_FULL),
            DQ_DLY[2] = u4IO32ReadFldAlign(DDRPHY_RXDQ8+0x60*(u1ByteIdx/2)+(aru1PhyMap2Channel[u1ByteIdx%2]<< POS_BANK_NUM), PHY_FLD_FULL),
            DQ_DLY[3] = u4IO32ReadFldAlign(DDRPHY_RXDQ9+0x60*(u1ByteIdx/2)+(aru1PhyMap2Channel[u1ByteIdx%2]<< POS_BANK_NUM), PHY_FLD_FULL),
            mcSHOW_DBG_MSG(("RK1 B%d R(7~0) (%d, %d, %d, %d, %d, %d, %d, %d)\n", u1ByteIdx,
                (DQ_DLY[3]>>8)&0xff, (DQ_DLY[3]>>24)&0xff, (DQ_DLY[2]>>8)&0xff, (DQ_DLY[2]>>24)&0xff, 
                (DQ_DLY[1]>>8)&0xff, (DQ_DLY[1]>>24)&0xff, (DQ_DLY[0]>>8)&0xff, (DQ_DLY[0]>>24)&0xff));
            mcSHOW_DBG_MSG(("RK1 B%d F(7~0) (%d, %d, %d, %d, %d, %d, %d, %d)\n", u1ByteIdx,
                (DQ_DLY[3]>>0)&0xff, (DQ_DLY[3]>>16)&0xff, (DQ_DLY[2]>>0)&0xff, (DQ_DLY[2]>>16)&0xff, 
                (DQ_DLY[1]>>0)&0xff, (DQ_DLY[1]>>16)&0xff, (DQ_DLY[0]>>0)&0xff, (DQ_DLY[0]>>16)&0xff));
        }
    }
#endif            
    vSetPHY2ChannelMapping(p, channel_bak);
}

/***
1. set DQS*_DVS_DLY=4b'1xxxx, then log calibration result
2. move DQS delay from 0 to N, then check lead_count and lag_count result
3. move DQ delay from 0 to N, then check lead_count and lag_count result
***/
void DramcRxInputDlyTrackingSW(DRAMC_CTX_T *p)
{
    DRAM_CHANNEL_T channel_bak = p->channel;
    UINT8 dqsDly=0, dqDly=0;
    UINT8 u1ByteIdx=0, u1BitIdx=0;
    UINT8 updateDone=0;
    
    vSetPHY2ChannelMapping(p, CHANNEL_A);

    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR1),0x1, RXDLY_TR1_R_RK0_DVS_MODE);
    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR11),0x1, RXDLY_TR11_R_RK1_DVS_MODE);
    
    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE2),0xF, EYE2_RG_RX_ARDQS0_DVS_DLY);
    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1_2),0xF, EYEB1_2_RG_RX_ARDQS1_DVS_DLY);

    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR7),0x1, RXDLY_TR7_R_RK0_B0_DVS_SW_CNT_ENA);
    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR9),0x1, RXDLY_TR9_R_RK0_B1_DVS_SW_CNT_ENA);
    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR17),0x1, RXDLY_TR17_R_RK1_B0_DVS_SW_CNT_ENA);
    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR19),0x1, RXDLY_TR19_R_RK1_B1_DVS_SW_CNT_ENA);

    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR0),0x1, RXDLY_TR0_R_RX_DLY_RK_OPT);

#if 1//adjust DQS delay and monitor Lead/Lag count
    for(dqsDly=0; dqsDly<40; dqsDly++)
    {    
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ5), dqsDly, RXDQ5_RK0_RX_ARDQS0_R_DLY); 
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ5), dqsDly, RXDQ5_RK0_RX_ARDQS0_F_DLY);             
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXB1_5), dqsDly, RXB1_5_RK0_RX_ARDQS1_R_DLY); 
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXB1_5), dqsDly, RXB1_5_RK0_RX_ARDQS1_F_DLY);
        
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ10), dqsDly, RXDQ10_RK1_RX_ARDQS0_R_DLY); 
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ10), dqsDly, RXDQ10_RK1_RX_ARDQS0_F_DLY);             
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXRK1_B1_4), dqsDly, RXRK1_B1_4_RK1_RX_ARDQS1_R_DLY); 
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXRK1_B1_4), dqsDly, RXRK1_B1_4_RK1_RX_ARDQS1_R_DLY);
        
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR0), 1, RXDLY_TR0_R_DVS_SW_UP);
        while(!updateDone)
        {
            mcSHOW_DBG_MSG(("wait for DVS_SW_UP DONE=%H\n", updateDone));
            updateDone = u4IO32ReadFldAlign(DDRPHY_RXDLY_TRRO14+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), RXDLY_TRRO14_DVS_RK0_B0_SW_UP_DONE); updateDone<<1;
            updateDone &= u4IO32ReadFldAlign(DDRPHY_RXDLY_TRRO14+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), RXDLY_TRRO14_DVS_RK0_B1_SW_UP_DONE);updateDone<<1;            
            updateDone &= u4IO32ReadFldAlign(DDRPHY_RXDLY_TRRO14+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), RXDLY_TRRO14_DVS_RK0_B0_SW_UP_DONE);updateDone<<1;
            updateDone &= u4IO32ReadFldAlign(DDRPHY_RXDLY_TRRO14+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), RXDLY_TRRO14_DVS_RK0_B1_SW_UP_DONE);updateDone<<1;
            
            updateDone &= u4IO32ReadFldAlign(DDRPHY_RXDLY_TRRO14+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), RXDLY_TRRO14_DVS_RK1_B0_SW_UP_DONE);updateDone<<1;
            updateDone &= u4IO32ReadFldAlign(DDRPHY_RXDLY_TRRO14+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), RXDLY_TRRO14_DVS_RK1_B1_SW_UP_DONE);updateDone<<1;
            updateDone &= u4IO32ReadFldAlign(DDRPHY_RXDLY_TRRO14+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), RXDLY_TRRO14_DVS_RK1_B0_SW_UP_DONE);updateDone<<1;
            updateDone &= u4IO32ReadFldAlign(DDRPHY_RXDLY_TRRO14+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), RXDLY_TRRO14_DVS_RK1_B1_SW_UP_DONE);
        }
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR0), 0, RXDLY_TR0_R_DVS_SW_UP);
        mcSHOW_DBG_MSG(("DQS Delay=%d update done\n", dqsDly));

        DramPhyReset(p);

        DramcDFSTriggerDMA();
        while(*((volatile unsigned int *)(0x10212c08))) ;
        DramcPrintRxInputDlyTrackingFlag(p);

    }
#elif 0//ajdust DQ delay and monitor Lead/Lag count
    for(u1BitIdx=0; u1BitIdx<8; u1BitIdx++)
    {
        for(dqDly=12; dqDly<45; dqDly++)
        {    
            if(u1BitIdx%2==0)
            {
                //RK0
                vIO32WriteFldAlign_Phy_Byte(0, DRAMC_REG_ADDR(DDRPHY_RXDQ1+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ0_R_DLY); 
                vIO32WriteFldAlign_Phy_Byte(1, DRAMC_REG_ADDR(DDRPHY_RXDQ1+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ0_R_DLY); 
                vIO32WriteFldAlign_Phy_Byte(2, DRAMC_REG_ADDR(DDRPHY_RXDQ1+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ0_R_DLY); 
                vIO32WriteFldAlign_Phy_Byte(3, DRAMC_REG_ADDR(DDRPHY_RXDQ1+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ0_R_DLY); 
                vIO32WriteFldAlign_Phy_Byte(0, DRAMC_REG_ADDR(DDRPHY_RXDQ1+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ0_F_DLY); 
                vIO32WriteFldAlign_Phy_Byte(1, DRAMC_REG_ADDR(DDRPHY_RXDQ1+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ0_F_DLY); 
                vIO32WriteFldAlign_Phy_Byte(2, DRAMC_REG_ADDR(DDRPHY_RXDQ1+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ0_F_DLY); 
                vIO32WriteFldAlign_Phy_Byte(3, DRAMC_REG_ADDR(DDRPHY_RXDQ1+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ0_F_DLY);
                //RK1
                vIO32WriteFldAlign_Phy_Byte(0, DRAMC_REG_ADDR(DDRPHY_RXDQ6+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ0_R_DLY); 
                vIO32WriteFldAlign_Phy_Byte(1, DRAMC_REG_ADDR(DDRPHY_RXDQ6+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ0_R_DLY); 
                vIO32WriteFldAlign_Phy_Byte(2, DRAMC_REG_ADDR(DDRPHY_RXDQ6+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ0_R_DLY); 
                vIO32WriteFldAlign_Phy_Byte(3, DRAMC_REG_ADDR(DDRPHY_RXDQ6+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ0_R_DLY); 
                vIO32WriteFldAlign_Phy_Byte(0, DRAMC_REG_ADDR(DDRPHY_RXDQ6+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ0_F_DLY); 
                vIO32WriteFldAlign_Phy_Byte(1, DRAMC_REG_ADDR(DDRPHY_RXDQ6+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ0_F_DLY); 
                vIO32WriteFldAlign_Phy_Byte(2, DRAMC_REG_ADDR(DDRPHY_RXDQ6+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ0_F_DLY); 
                vIO32WriteFldAlign_Phy_Byte(3, DRAMC_REG_ADDR(DDRPHY_RXDQ6+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ0_F_DLY);

            }
            else
            {
                //RK0
                vIO32WriteFldAlign_Phy_Byte(0, DRAMC_REG_ADDR(DDRPHY_RXDQ1+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ1_R_DLY); 
                vIO32WriteFldAlign_Phy_Byte(1, DRAMC_REG_ADDR(DDRPHY_RXDQ1+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ1_R_DLY); 
                vIO32WriteFldAlign_Phy_Byte(2, DRAMC_REG_ADDR(DDRPHY_RXDQ1+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ1_R_DLY); 
                vIO32WriteFldAlign_Phy_Byte(3, DRAMC_REG_ADDR(DDRPHY_RXDQ1+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ1_R_DLY); 
                vIO32WriteFldAlign_Phy_Byte(0, DRAMC_REG_ADDR(DDRPHY_RXDQ1+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ1_F_DLY); 
                vIO32WriteFldAlign_Phy_Byte(1, DRAMC_REG_ADDR(DDRPHY_RXDQ1+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ1_F_DLY); 
                vIO32WriteFldAlign_Phy_Byte(2, DRAMC_REG_ADDR(DDRPHY_RXDQ1+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ1_F_DLY); 
                vIO32WriteFldAlign_Phy_Byte(3, DRAMC_REG_ADDR(DDRPHY_RXDQ1+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ1_F_DLY);
                //RK1
                vIO32WriteFldAlign_Phy_Byte(0, DRAMC_REG_ADDR(DDRPHY_RXDQ6+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ1_R_DLY); 
                vIO32WriteFldAlign_Phy_Byte(1, DRAMC_REG_ADDR(DDRPHY_RXDQ6+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ1_R_DLY); 
                vIO32WriteFldAlign_Phy_Byte(2, DRAMC_REG_ADDR(DDRPHY_RXDQ6+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ1_R_DLY); 
                vIO32WriteFldAlign_Phy_Byte(3, DRAMC_REG_ADDR(DDRPHY_RXDQ6+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ1_R_DLY); 
                vIO32WriteFldAlign_Phy_Byte(0, DRAMC_REG_ADDR(DDRPHY_RXDQ6+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ1_F_DLY); 
                vIO32WriteFldAlign_Phy_Byte(1, DRAMC_REG_ADDR(DDRPHY_RXDQ6+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ1_F_DLY); 
                vIO32WriteFldAlign_Phy_Byte(2, DRAMC_REG_ADDR(DDRPHY_RXDQ6+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ1_F_DLY); 
                vIO32WriteFldAlign_Phy_Byte(3, DRAMC_REG_ADDR(DDRPHY_RXDQ6+(u1BitIdx/2)*4), dqDly, RXDQ1_RK0_RX_ARDQ1_F_DLY);
            }
            
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR0), 1, RXDLY_TR0_R_DVS_SW_UP);
            while(!updateDone)
            {
                mcSHOW_DBG_MSG(("wait for DVS_SW_UP DONE=%H\n", updateDone));
                updateDone = u4IO32ReadFldAlign(DDRPHY_RXDLY_TRRO14+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), RXDLY_TRRO14_DVS_RK0_B0_SW_UP_DONE); updateDone<<1;
                updateDone &= u4IO32ReadFldAlign(DDRPHY_RXDLY_TRRO14+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), RXDLY_TRRO14_DVS_RK0_B1_SW_UP_DONE);updateDone<<1;            
                updateDone &= u4IO32ReadFldAlign(DDRPHY_RXDLY_TRRO14+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), RXDLY_TRRO14_DVS_RK0_B0_SW_UP_DONE);updateDone<<1;
                updateDone &= u4IO32ReadFldAlign(DDRPHY_RXDLY_TRRO14+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), RXDLY_TRRO14_DVS_RK0_B1_SW_UP_DONE);updateDone<<1;
                
                updateDone &= u4IO32ReadFldAlign(DDRPHY_RXDLY_TRRO14+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), RXDLY_TRRO14_DVS_RK1_B0_SW_UP_DONE);updateDone<<1;
                updateDone &= u4IO32ReadFldAlign(DDRPHY_RXDLY_TRRO14+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), RXDLY_TRRO14_DVS_RK1_B1_SW_UP_DONE);updateDone<<1;
                updateDone &= u4IO32ReadFldAlign(DDRPHY_RXDLY_TRRO14+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), RXDLY_TRRO14_DVS_RK1_B0_SW_UP_DONE);updateDone<<1;
                updateDone &= u4IO32ReadFldAlign(DDRPHY_RXDLY_TRRO14+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), RXDLY_TRRO14_DVS_RK1_B1_SW_UP_DONE);
            }
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR0), 0, RXDLY_TR0_R_DVS_SW_UP);
            mcSHOW_DBG_MSG(("DQ%d Delay=%d update done\n", u1BitIdx, dqDly));

            DramPhyReset(p);

            DramcDFSTriggerDMA();
            while(*((volatile unsigned int *)(0x10212c08))) ;
            DramcPrintRxInputDlyTrackingFlag(p);

        }
    }
#endif
        
    vSetPHY2ChannelMapping(p, channel_bak);
}

/***
1. set DQS*_DVS_DLY=4b'1xxxx, then log calibration result
LAG side:
2. change V/T and check lead/lag count
   ex. V from 1000->900mv (lag will be detected)
3. DFS (directly jump to itself) and check Rx dly
LEAD side
4. manually increase DQ_DLY(+10) after cali
5. when enabling HW tracking, lead will be detected.
6. DFS (directly jump to itself) and check Rx dly
***/
U32 testCnt=0;
void DramcRxInputDlyTrackingHW(DRAMC_CTX_T *p)
{
    DRAM_CHANNEL_T channel_bak = p->channel;
    UINT8 updateDone=0;
    U16 u2DVS_TH=0x0;
    U16 u2MinDly=0x14;
    U16 u2MaxDly=0x30;
    
    vSetPHY2ChannelMapping(p, CHANNEL_A);

    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR1),0x2, RXDLY_TR1_R_RK0_DVS_MODE);
    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR11),0x2, RXDLY_TR11_R_RK1_DVS_MODE);
    
    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE2),0xF, EYE2_RG_RX_ARDQS0_DVS_DLY);
    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1_2),0xF, EYEB1_2_RG_RX_ARDQS1_DVS_DLY);

    //err flag threshold
    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR8), u2DVS_TH, RXDLY_TR8_R_RK0_B0_DVS_TH_LEAD);
    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR8), u2DVS_TH, RXDLY_TR8_R_RK0_B0_DVS_TH_LAG);
    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR10), u2DVS_TH, RXDLY_TR10_R_RK0_B1_DVS_TH_LEAD);
    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR10), u2DVS_TH, RXDLY_TR10_R_RK0_B1_DVS_TH_LAG);
    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR18), u2DVS_TH, RXDLY_TR18_R_RK1_B0_DVS_TH_LEAD);
    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR18), u2DVS_TH, RXDLY_TR18_R_RK1_B0_DVS_TH_LAG);
    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR20), u2DVS_TH, RXDLY_TR20_R_RK1_B1_DVS_TH_LEAD);
    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR20), u2DVS_TH, RXDLY_TR20_R_RK1_B1_DVS_TH_LAG);

    //RK0-MIN-MAX boundary : default=0-0x3F
    vIO32WriteFldMulti_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR4), P_Fld(u2MinDly, RXDLY_TR4_RG_RK0_ARDQS0_MIN_DLY) | P_Fld(u2MinDly, RXDLY_TR4_RG_RK0_ARDQM0_MIN_DLY));
    vIO32WriteFldMulti_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR5), P_Fld(u2MinDly, RXDLY_TR5_RG_RK0_ARDQ0_MIN_DLY)
        | P_Fld(u2MinDly, RXDLY_TR5_RG_RK0_ARDQ1_MIN_DLY) | P_Fld(u2MinDly, RXDLY_TR5_RG_RK0_ARDQ2_MIN_DLY) |P_Fld(u2MinDly, RXDLY_TR5_RG_RK0_ARDQ3_MIN_DLY));
    vIO32WriteFldMulti_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR6), P_Fld(u2MinDly, RXDLY_TR6_RG_RK0_ARDQ4_MIN_DLY)
        | P_Fld(u2MinDly, RXDLY_TR6_RG_RK0_ARDQ5_MIN_DLY) | P_Fld(u2MinDly, RXDLY_TR6_RG_RK0_ARDQ6_MIN_DLY) |P_Fld(u2MinDly, RXDLY_TR6_RG_RK0_ARDQ7_MIN_DLY));

    vIO32WriteFldMulti_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR4), P_Fld(u2MaxDly, RXDLY_TR4_RG_RK0_ARDQS0_MAX_DLY) | P_Fld(u2MaxDly, RXDLY_TR4_RG_RK0_ARDQM0_MAX_DLY));
    vIO32WriteFldMulti_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR2), P_Fld(u2MaxDly, RXDLY_TR2_RG_RK0_ARDQ0_MAX_DLY)
        | P_Fld(u2MaxDly, RXDLY_TR2_RG_RK0_ARDQ1_MAX_DLY) | P_Fld(u2MaxDly, RXDLY_TR2_RG_RK0_ARDQ2_MAX_DLY) |P_Fld(u2MaxDly, RXDLY_TR2_RG_RK0_ARDQ3_MAX_DLY));
    vIO32WriteFldMulti_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR3), P_Fld(u2MaxDly, RXDLY_TR3_RG_RK0_ARDQ4_MAX_DLY)
        | P_Fld(u2MaxDly, RXDLY_TR3_RG_RK0_ARDQ5_MAX_DLY) | P_Fld(u2MaxDly, RXDLY_TR3_RG_RK0_ARDQ6_MAX_DLY) |P_Fld(u2MaxDly, RXDLY_TR3_RG_RK0_ARDQ7_MAX_DLY));

    //RK1-MIN-MAX boundary : default=0-0x3F
    vIO32WriteFldMulti_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR14), P_Fld(u2MinDly, RXDLY_TR14_RG_RK1_ARDQS0_MIN_DLY) | P_Fld(u2MinDly, RXDLY_TR14_RG_RK1_ARDQM0_MIN_DLY));
    vIO32WriteFldMulti_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR15), P_Fld(u2MinDly, RXDLY_TR15_RG_RK1_ARDQ0_MIN_DLY)
        | P_Fld(u2MinDly, RXDLY_TR15_RG_RK1_ARDQ1_MIN_DLY) | P_Fld(u2MinDly, RXDLY_TR15_RG_RK1_ARDQ2_MIN_DLY) |P_Fld(u2MinDly, RXDLY_TR15_RG_RK1_ARDQ3_MIN_DLY));
    vIO32WriteFldMulti_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR16), P_Fld(u2MinDly, RXDLY_TR16_RG_RK1_ARDQ4_MIN_DLY)
        | P_Fld(u2MinDly, RXDLY_TR16_RG_RK1_ARDQ5_MIN_DLY) | P_Fld(u2MinDly, RXDLY_TR16_RG_RK1_ARDQ6_MIN_DLY) |P_Fld(u2MinDly, RXDLY_TR16_RG_RK1_ARDQ7_MIN_DLY));
        
    vIO32WriteFldMulti_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR14), P_Fld(u2MaxDly, RXDLY_TR14_RG_RK1_ARDQS0_MAX_DLY) | P_Fld(u2MaxDly, RXDLY_TR14_RG_RK1_ARDQM0_MAX_DLY));
        vIO32WriteFldMulti_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR12), P_Fld(u2MaxDly, RXDLY_TR12_RG_RK1_ARDQ0_MAX_DLY)
            | P_Fld(u2MaxDly, RXDLY_TR12_RG_RK1_ARDQ1_MAX_DLY) | P_Fld(u2MaxDly, RXDLY_TR12_RG_RK1_ARDQ2_MAX_DLY) |P_Fld(u2MaxDly, RXDLY_TR12_RG_RK1_ARDQ3_MAX_DLY));
        vIO32WriteFldMulti_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR13), P_Fld(u2MaxDly, RXDLY_TR13_RG_RK1_ARDQ4_MAX_DLY)
            | P_Fld(u2MaxDly, RXDLY_TR13_RG_RK1_ARDQ5_MAX_DLY) | P_Fld(u2MaxDly, RXDLY_TR13_RG_RK1_ARDQ6_MAX_DLY) |P_Fld(u2MaxDly, RXDLY_TR13_RG_RK1_ARDQ7_MAX_DLY));

    //DQ/DQS update scale
    vIO32WriteFldMulti_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR1), P_Fld(0x1, RXDLY_TR1_R_RK0_RX_DLY_RIS_DQ_SCALE)
        | P_Fld(0x4, RXDLY_TR1_R_RK0_RX_DLY_RIS_DQS_SCALE) | P_Fld(0x1, RXDLY_TR1_R_RK0_RX_DLY_FAL_DQ_SCALE)
        | P_Fld(0x4, RXDLY_TR1_R_RK0_RX_DLY_FAL_DQS_SCALE));
    vIO32WriteFldMulti_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR11), P_Fld(0x1, RXDLY_TR11_R_RK1_RX_DLY_RIS_DQ_SCALE)
        | P_Fld(0x4, RXDLY_TR11_R_RK1_RX_DLY_RIS_DQS_SCALE) |  P_Fld(0x1, RXDLY_TR11_R_RK1_RX_DLY_FAL_DQ_SCALE)
        | P_Fld(0x4, RXDLY_TR11_R_RK1_RX_DLY_FAL_DQS_SCALE));

    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR0),0x1, RXDLY_TR0_R_RX_DLY_RK_OPT);

    //DramcDFSTriggerDMA();
    //enable HW tracking
    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR0),0x1, RXDLY_TR0_R_RX_DLY_TRACK_ENA);
    vIO32WriteFldMulti_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR1), P_Fld(0x1, RXDLY_TR1_R_RK0_RX_DLY_RIS_TRACK_GATE_ENA)
        | P_Fld(0x1, RXDLY_TR1_R_RK0_RX_DLY_FAL_TRACK_GATE_ENA));
    vIO32WriteFldMulti_All(DRAMC_REG_ADDR(DDRPHY_RXDLY_TR11), P_Fld(0x1, RXDLY_TR11_R_RK1_RX_DLY_RIS_TRACK_GATE_ENA)
        | P_Fld(0x1, RXDLY_TR11_R_RK1_RX_DLY_FAL_TRACK_GATE_ENA));

    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DDRPHY_PINMUX), 1, PINMUX_R_HWSAVE_MODE_ENA);
    vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_RESERVED_DRAMC_2), P_Fld(1, RESERVED_DRAMC_2_R_HWSAVE_DVFS_PHY_ENA)|P_Fld(1, RESERVED_DRAMC_2_R_HWSAVE_SREF_PHY_ENA));
    //while(1)
    testCnt++;
    //if(testCnt<=100)
    {
        //U32 voltage = 1000-testCnt;
        //PMIC_VCORE_ADJUST(voltage);
        #if DUAL_FREQ_K
        DramcDFSDirectJump(p, 0, 0, 0);
        #endif
        
        DramcPrintRxInputDly(p);
        DramcPrintRxInputDlyTrackingFlag(p);
    }

    vSetPHY2ChannelMapping(p, channel_bak);
}

#endif

void vGetImpedanceResult(DRAMC_CTX_T *p, U8 *drvp, U8* drvn)
{
    *drvp = (U8)u4IO32ReadFldAlign(DRAMC_REG_ADDR(DDRPHY_TXDQ9), TXDQ9_RG_TX_ARDQ_DRVP_B1);
    *drvn = (U8)u4IO32ReadFldAlign(DRAMC_REG_ADDR(DDRPHY_TXDQ9), TXDQ9_RG_TX_ARDQ_DRVN_B1); 
}

void DDRPhyFreqSel(DRAMC_CTX_T *p, DRAM_PLL_FREQ_SEL_T sel)
{
    p->freq_sel = sel;
    p->shu_type = DRAM_DFS_SHUFFLE_NORMAL;
    switch(p->freq_sel)
    {
        case LJ_DDR1866:
            p->frequency = 933; break;
        case LJ_DDR1700:
            p->frequency = 850; break;
        case LJ_DDR1600:
            p->frequency = 800; break;
        case LJ_DDR933:
            p->frequency = 466; break;
        case LJ_DDR850:
            p->frequency = 425; break;
        case LJ_DDR800:
            p->frequency = 400; break;
            
        case LC_DDR1600:
            #if DFS_COMBINATION_TYPE1
            p->shu_type = DRAM_DFS_SHUFFLE_1;
            #else
            p->shu_type = DRAM_DFS_SHUFFLE_NORMAL;
            #endif
            p->frequency = 800; break;
        case LC_DDR1270:
            #if DFS_COMBINATION_TYPE1
            p->shu_type = DRAM_DFS_SHUFFLE_2;
            #else
            p->shu_type = DRAM_DFS_SHUFFLE_1;
            #endif
            p->frequency = 635; break;
        case LC_DDR1066:
            #if DFS_COMBINATION_TYPE1
            p->shu_type = DRAM_DFS_SHUFFLE_SW;
            #else
            p->shu_type = DRAM_DFS_SHUFFLE_2;
            #endif
            p->frequency = 533; break;
        case LC_DDR800:
            p->frequency = 400; break;
        case LC_DDR635:
            p->frequency = 317; break;
        case LC_DDR533:
            p->frequency = 266; break;
        default:
            p->shu_type = DRAM_DFS_SHUFFLE_NORMAL;
            p->freq_sel = LJ_DDR1600;
            p->frequency = 800;
            break;
    }
}

#if ENABLE_DDRPHY_FREQ_METER
void DDRPhyFreqMeter(void)
{
    U8 i=0;
    U8 u1Value = 0x30;
    U32 u4Freq = 0;
        
    vIO32WriteFldAlign_All(DDRPHY_PLL2, 1 ,PLL2_RG_RPHYPLL_MONCK_EN);
    vIO32WriteFldAlign_All(DDRPHY_PLL4, 1 ,PLL4_RG_RCLRPLL_MONCK_EN);
	//vIO32WriteFldAlign_All(DDRPHY_PLL13, 0, PLL13_RG_ARDLL_PHDET_EN);

    if(u4IO32ReadFldAlign(DDRPHY_PLL1, PLL1_RG_RPHYPLL_EN))
    {    
        mcSHOW_DBG_MSG(("\n===LJPLL_FREQ_DEBUG_LOG ===\n"));
        for(i=0; i<3; i++)
        {
            u1Value = 0x30+i*2;
            u4Freq = 0;
            RISCWrite(0x1000010c, u1Value<<16);
            RISCWrite(0x10000220, 0x00001010);
            while((*((volatile unsigned int *)(0x10000220)) & 0x00000010)!=0) { mcDELAY_US(1); }
            u4Freq = (RISCRead(0x10000224)) & 0x0000FFFF;
            u4Freq = ((RISCRead(0x10000104)>>24)+1)*u4Freq*26*8/1024;
            
            mcSHOW_DBG_MSG(("FREQ PHYPLL_%d : %d\n", i, u4Freq));    
        }
    }
    
    if(u4IO32ReadFldAlign(DDRPHY_PLL3, PLL3_RG_RCLRPLL_EN))
    {
        mcSHOW_DBG_MSG(("\n===LCPLL_FREQ_DEBUG_LOG ===\n"));
        for(i=0; i<3; i++)
        {
            u1Value = 0x31+i*2;
            u4Freq = 0;
            RISCWrite(0x1000010c, u1Value<<16);
            RISCWrite(0x10000220, 0x00001010);
            while((*((volatile unsigned int *)(0x10000220)) & 0x00000010)!=0) { mcDELAY_US(1); }
            u4Freq = (RISCRead(0x10000224)) & 0x0000FFFF;
            u4Freq = ((RISCRead(0x10000104)>>24)+1)*u4Freq*26*2/1024;
            
            mcSHOW_DBG_MSG(("FREQ CLRPLL_%d : %d\n", i, u4Freq));    
        }
    }  
    RISCWrite(0x1000010c, 0x17<<16);
    RISCWrite(0x10000220, 0x00001010);
    while((*((volatile unsigned int *)(0x10000220)) & 0x00000010)!=0) { mcDELAY_US(1); }
    u4Freq = (RISCRead(0x10000224)) & 0x0000FFFF;
    u4Freq = ((RISCRead(0x10000104)>>24)+1)*u4Freq*26*4/1024;
    mcSHOW_DBG_MSG(("FREQ FMEM_CK(x4): %d\n\n", u4Freq));    
    #if (!PHY_MONCLK_SETTING_TEST)
    vIO32WriteFldAlign_All(DDRPHY_PLL2, 0 ,PLL2_RG_RPHYPLL_MONCK_EN);
    vIO32WriteFldAlign_All(DDRPHY_PLL4, 0 ,PLL4_RG_RCLRPLL_MONCK_EN);
    #endif
}
#endif


#if __ETT__
void GetPhyPllFrequency(DRAMC_CTX_T *p)
{
    U8 REF_CLK = 52;
    U16 u2real_freq=0;
    U8 mpdiv_shu_sel=0, pll_shu_sel=0;
    U32 u4MPDIV_IN_SEL = 0;
    U8 u1MPDIV_Sel = 0;
    U32 u4DDRPHY_PLL12_Addr[3] = {DDRPHY_BASE+0x42c, DDRPHY_BASE+0x448, DDRPHY_BASE+0x9c4};
    U32 u4DDRPHY_PLL1_Addr[3] =  {DDRPHY_BASE+0x400, DDRPHY_BASE+0x640, DDRPHY_BASE+0xa40};
    U32 u4DDRPHY_PLL3_Addr[3] =  {DDRPHY_BASE+0x408, DDRPHY_BASE+0x644, DDRPHY_BASE+0xa44};
    #if CHECK_PLL_OK
        if(is_pll_good() == 0) //not all pll is good, use 26M reference clock
            REF_CLK = 26;    
    #endif

    //DFSing
    if(*((volatile unsigned int *)(DRAMC0_BASE+0x028)) & (1<<17))
        return;
        
    mpdiv_shu_sel = (*((volatile unsigned int *)(DRAMC0_BASE+0x028))>>8)&0x7;
    pll_shu_sel = (*((volatile unsigned int *)(DDRPHY_BASE+0x63c)))&0x3;

    u4MPDIV_IN_SEL = *((volatile unsigned int *)(u4DDRPHY_PLL12_Addr[mpdiv_shu_sel])) & (1<<17) ;
    u1MPDIV_Sel = (*((volatile unsigned int *)(u4DDRPHY_PLL12_Addr[mpdiv_shu_sel]))>>10)&0x3;
    if(u4MPDIV_IN_SEL==0)//PHYPLL
    {
        //FREQ_LCPLL = = FREQ_XTAL*(RG_*_RPHYPLL_FBKDIV+1)*2^(RG_*_RPHYPLL_FBKSEL)/2^(RG_*_RPHYPLL_PREDIV)
        //U32 u4DDRPHY_PLL1 = *((volatile unsigned int *)(u4DDRPHY_PLL1_Addr[pll_shu_sel]));
        U32 u4DDRPHY_PLL1 = *((volatile unsigned int *)(u4DDRPHY_PLL1_Addr[0]));
        U32 u4FBKDIV = (u4DDRPHY_PLL1>>24)&0x7F;
        U8 u1FBKSEL = (u4DDRPHY_PLL1>>20)&0x3;
        U8 u1PREDIV = (u4DDRPHY_PLL1>>18)&0x3;
        if(u1PREDIV==3)
            u1PREDIV = 2;
        u2real_freq = (U16) (((u4FBKDIV+1)*REF_CLK)<<u1FBKSEL)>>(u1PREDIV);
    }
    else
    {
        //Freq(VCO clock) = FREQ_XTAL*(RG_RCLRPLL_SDM_PCW)/2^(RG_*_RCLRPLL_PREDIV)/2^(RG_*_RCLRPLL_POSDIV)
        //Freq(DRAM clock)= Freq(VCO clock)/4
        U32 u4DDRPHY_PLL3 = *((volatile unsigned int *)(u4DDRPHY_PLL3_Addr[pll_shu_sel]));
        U32 u4FBKDIV = (u4DDRPHY_PLL3>>24)&0x7F;
        U8 u1PREDIV = (u4DDRPHY_PLL3>>18)&0x3;
        U8 u1POSDIV = (u4DDRPHY_PLL3>>16)&0x3;
        U8 u1FBKSEL = (*((volatile unsigned int *)(DDRPHY_BASE+0x43c))>>12)&0x1;
        u2real_freq = (U16) ((((u4FBKDIV)*REF_CLK)<<u1FBKSEL>>u1PREDIV)>>(u1POSDIV))>>2;
    }
    u2real_freq = u2real_freq>> u1MPDIV_Sel;
    mcSHOW_DBG_MSG(("GetPhyFrequency: %d\n\n", u2real_freq));
}
#endif


void DDRPhyResetPLL(void)
{
    vIO32WriteFldAlign_All(DRAMC_REG_PADCTL4, 1, PADCTL4_CKEFIXOFF);

    vIO32WriteFldAlign_All(DDRPHY_PLL1, 0 ,PLL1_RG_RPHYPLL_EN);
    vIO32WriteFldAlign_All(DDRPHY_PLL3, 0 ,PLL3_RG_RCLRPLL_EN);
	vIO32WriteFldAlign_All(DDRPHY_PLL13, 0, PLL13_RG_ARDLL_PHDET_EN);
    vIO32WriteFldAlign_All(DDRPHY_PLL13, 0, PLL14_RG_BRDLL_PHDET_EN);
	vIO32WriteFldAlign_All(DDRPHY_PLL16, 0, PLL16_RG_RPHYPLL_RESETB);
    
    vIO32WriteFldAlign_All(DDRPHY_PLL1, 1 ,PLL1_RG_RPHYPLL_EN);
    vIO32WriteFldAlign_All(DDRPHY_PLL3, 1 ,PLL3_RG_RCLRPLL_EN);

    mcDELAY_US(22);
    
	vIO32WriteFldAlign_All(DDRPHY_PLL16, 1, PLL16_RG_RPHYPLL_RESETB);
	mcDELAY_US(1);
	vIO32WriteFldAlign_All(DDRPHY_PLL13, 1, PLL13_RG_ARDLL_PHDET_EN);
	mcDELAY_US(1);
	vIO32WriteFldAlign_All(DDRPHY_PLL14, 1, PLL14_RG_BRDLL_PHDET_EN);
	mcDELAY_US(1);	//DLL Ready

	vIO32WriteFldAlign_All(DRAMC_REG_PADCTL4, 0, PADCTL4_CKEFIXOFF);

	DDRPhyFreqMeter();
}

void DDRAllLowPowerSettings(DRAMC_CTX_T *p)
{       
    //PHY CG Control
    *((UINT32P)(DDRPHY_BASE    + 0x420 )) = 0x77F43FE1;
    *((UINT32P)(DDRPHY_BASE    + 0x424 )) = 0x000BC31E;
    *((UINT32P)(DDRPHY_BASE    + 0x42c )) &= ~((0x1 << 19) | (0x1 << 7));
    *((UINT32P)(DDRPHY_BASE    + 0x42c )) |= ((0x1 << 18) | (0x1 << 15));
    *((UINT32P)(DDRPHY_BASE    + 0x5b0 )) = 0xFFFFFFFF;
    *((UINT32P)(DDRPHY_BASE    + 0x5b4 )) = 0x0;
    *((UINT32P)(DDRPHY_BASE    + 0x5b8 )) &= ~((0x1 << 1) | (0x1 << 0));
    *((UINT32P)(DDRPHY_BASE    + 0x5bc )) &= ~((0x1 << 3) | (0x1 << 2) | (0x1 << 1) | (0x1 << 0));

    
    *((UINT32P)(DDRPHY1_BASE    + 0x420 )) = 0x77F5C31F;
    *((UINT32P)(DDRPHY1_BASE    + 0x424 )) = 0x000A3CE0;
    *((UINT32P)(DDRPHY1_BASE    + 0x42c )) &= ~((0x1 << 19) | (0x1 << 7));
    *((UINT32P)(DDRPHY1_BASE    + 0x42c )) |= ((0x1 << 18) | (0x1 << 15));
    *((UINT32P)(DDRPHY1_BASE    + 0x5b0 )) = 0xFFFFFFFF;
    *((UINT32P)(DDRPHY1_BASE    + 0x5b4 )) = 0x0;
    *((UINT32P)(DDRPHY1_BASE    + 0x5b8 )) &= ~((0x1 << 1) | (0x1 << 0));
    *((UINT32P)(DDRPHY1_BASE    + 0x5bc )) &= ~((0x1 << 3) | (0x1 << 2) | (0x1 << 1) | (0x1 << 0));

    
    *((UINT32P)(DDRPHY2_BASE    + 0x420 )) = 0x77F5C3E1;
    *((UINT32P)(DDRPHY2_BASE    + 0x424 )) = 0x000A3C1E;
    *((UINT32P)(DDRPHY2_BASE    + 0x42c )) &= ~((0x1 << 19) | (0x1 << 7));
    *((UINT32P)(DDRPHY2_BASE    + 0x42c )) |= ((0x1 << 18) | (0x1 << 15));
    *((UINT32P)(DDRPHY2_BASE    + 0x5b0 )) = 0xFFFFFFFF;
    *((UINT32P)(DDRPHY2_BASE    + 0x5b4 )) = 0x0;
    *((UINT32P)(DDRPHY2_BASE    + 0x5b8 )) &= ~((0x1 << 1) | (0x1 << 0));
    *((UINT32P)(DDRPHY2_BASE    + 0x5bc )) &= ~((0x1 << 3) | (0x1 << 2) | (0x1 << 1) | (0x1 << 0));

    
    *((UINT32P)(DDRPHY3_BASE    + 0x420 )) = 0x77F5C3E1;
    *((UINT32P)(DDRPHY3_BASE    + 0x424 )) = 0x000A3C1E;
    *((UINT32P)(DDRPHY3_BASE    + 0x42c )) &= ~((0x1 << 19) | (0x1 << 7));
    *((UINT32P)(DDRPHY3_BASE    + 0x42c )) |= ((0x1 << 18) | (0x1 << 15));
    *((UINT32P)(DDRPHY3_BASE    + 0x5b0 )) = 0xFFFFFFFF;
    *((UINT32P)(DDRPHY3_BASE    + 0x5b4 )) = 0x0;
    *((UINT32P)(DDRPHY3_BASE    + 0x5b8 )) &= ~((0x1 << 1) | (0x1 << 0));
    *((UINT32P)(DDRPHY3_BASE    + 0x5bc )) &= ~((0x1 << 3) | (0x1 << 2) | (0x1 << 1) | (0x1 << 0));
}


void SuspendResumeTest(DRAMC_CTX_T *p)
{
#if 0//Still debugging
    U32 u4err_value, ii;

    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DRAMC_PD_CTRL), 0x62, DRAMC_PD_CTRL_REFCNT_FR_CLK); 
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_CONF2), 0x61, CONF2_REFCNT); 

    u4err_value = DramcEngine2(p, TE_OP_WRITE_READ_CHECK, p->test2_1, p->test2_2, 2, 0, 0, 0);
    mcSHOW_DBG_MSG(("[SuspendResumeTest] Init write read check = 0x%x\n", u4err_value));

#if 0
    for(ii=0; ii <100; ii++)
    {
        mcDELAY_MS(1000);
        mcSHOW_DBG_MSG(("%d.", ii));
    }
    mcSHOW_DBG_MSG(("\n"));
     u4err_value = DramcEngine2(p, TE_OP_READ_CHECK, p->test2_1, p->test2_2, 2, 0, 0, 0);
     mcSHOW_DBG_MSG(("[SuspendResumeTest] Resume read check = 0x%x\n\n", u4err_value));

     u4err_value = DramcEngine2(p, TE_OP_READ_CHECK, p->test2_1, p->test2_2, 2, 0, 0, 0);
     mcSHOW_DBG_MSG(("[SuspendResumeTest] Resume read check = 0x%x\n\n", u4err_value));

     u4err_value = DramcEngine2(p, TE_OP_READ_CHECK, p->test2_1, p->test2_2, 2, 0, 0, 0);
     mcSHOW_DBG_MSG(("[SuspendResumeTest] Resume read check = 0x%x\n\n", u4err_value));
#endif

   // return; // for register dump
    // System reg. 0x0080  RG_128
              // 0            1  RW   DDRPHY_A_PWR_EN     Ch-A PWR_EN
              // 1            0  RW   DDRPHY_A_ISO_EN     Ch-A ISO_EN
              // 8            1  RW   DDRPHY_B_PWR_EN     Ch-B PWR_EN
              // 9            0  RW   DDRPHY_B_ISO_EN     Ch-B ISO_EN
             // 16            1  RW   DDRPHY_C_PWR_EN     Ch-C PWR_EN
             // 17            0  RW   DDRPHY_C_ISO_EN     Ch-C ISO_EN
             // 24            1  RW   DDRPHY_D_PWR_EN     Ch-D PWR_EN
             // 25            0  RW   DDRPHY_D_ISO_EN     Ch-D ISO_EN

    DramcEnterSelfRefresh(p, 1); // enter self refresh
#if 0
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DRAMC_PD_CTRL), 0, DRAMC_PD_CTRL_REFCNT_FR_CLK); 
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_CONF2), 0, CONF2_REFCNT); 
#endif
    mcSHOW_DBG_MSG(("[SuspendResumeTest] Please set  DM_SUS=1\n"));
    mcDELAY_MS(5000);
    //  GPIO DM_SUS =1 here
    mcSHOW_DBG_MSG(("[SuspendResumeTest] End Delay for DM_SUS =1\n"));
    
#if 0
    vIO32Write4B(mcSET_SYS_REG_ADDR(0x0080), 0x03030303);  // ISO_EN =1
    vIO32Write4B(mcSET_SYS_REG_ADDR(0x0080), 0x02020202);  // PWR_EN =0
    mcSHOW_DBG_MSG(("[SuspendResumeTest] PWR_EN= 0\n"));
#endif
    
    for(ii=0; ii <10; ii++)
    {
        mcDELAY_MS(1000);
        mcSHOW_DBG_MSG(("%d=", ii));
    }
    mcSHOW_DBG_MSG(("\n"));

#if 0
    mcSHOW_DBG_MSG(("[SuspendResumeTest] PWR_EN= 1\n"));
    vIO32Write4B(mcSET_SYS_REG_ADDR(0x0080), 0x03030303);  // PWR_EN =1
    vIO32Write4B(mcSET_SYS_REG_ADDR(0x0080), 0x01010101);  // ISO_EN =0

#endif
    mcSHOW_DBG_MSG(("[SuspendResumeTest] Please set  DM_SUS=0\n"));
    mcDELAY_MS(5000);
    mcSHOW_DBG_MSG(("[SuspendResumeTest] End Delay for DM_SUS =0\n"));
    //  GPIO DM_SUS =0 here

    #if 0
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DRAMC_PD_CTRL), 0x62, DRAMC_PD_CTRL_REFCNT_FR_CLK); 
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_CONF2), 0x61, CONF2_REFCNT); 
    #endif

    DramPhyReset(p);
    DramcEnterSelfRefresh(p, 0); // exit self refresh

    DramPhyReset(p);

//    return; // for register dump

     u4err_value = DramcEngine2(p, TE_OP_READ_CHECK, p->test2_1, p->test2_2, 2, 0, 0, 0);
     mcSHOW_DBG_MSG(("[SuspendResumeTest] Resume read check = 0x%x\n\n", u4err_value));

     u4err_value = DramcEngine2(p, TE_OP_READ_CHECK, p->test2_1, p->test2_2, 2, 0, 0, 0);
     mcSHOW_DBG_MSG(("[SuspendResumeTest] Resume read check = 0x%x\n\n", u4err_value));

     u4err_value = DramcEngine2(p, TE_OP_READ_CHECK, p->test2_1, p->test2_2, 2, 0, 0, 0);
     mcSHOW_DBG_MSG(("[SuspendResumeTest] Resume read check = 0x%x\n\n", u4err_value));
#endif
}

///TODO: EVEREST_PORTING_TODO
#ifdef LOOPBACK_TEST
DRAM_STATUS_T DramcLoopbackTest(DRAMC_CTX_T *p);
#endif



#ifdef DDR_INIT_TIME_PROFILING
//Kevin-CW Chen
//M0 ARMPLL div 1  = 897Mhz
//L1.mt6797.dev for smt ? 1.3Ghz
UINT32 temp;

void TimeProfileBegin(void)
{
    /* enable ARM CPU PMU */
    asm volatile(
        "MRC p15, 0, %0, c9, c12, 0\n"
        "BIC %0, %0, #1 << 0\n"   /* disable */
        "ORR %0, %0, #1 << 2\n"   /* reset cycle count */
        "BIC %0, %0, #1 << 3\n"   /* count every clock cycle */
        "MCR p15, 0, %0, c9, c12, 0\n"
        : "+r"(temp)
        :
        : "cc"
    );
    asm volatile(
        "MRC p15, 0, %0, c9, c12, 0\n"
        "ORR %0, %0, #1 << 0\n"   /* enable */
        "MCR p15, 0, %0, c9, c12, 0\n"
        "MRC p15, 0, %0, c9, c12, 1\n"
        "ORR %0, %0, #1 << 31\n"
        "MCR p15, 0, %0, c9, c12, 1\n"
        : "+r"(temp)
        :
        : "cc"
    );
}

UINT32 TimeProfileEnd(void)
{
    UINT32 div = 897*1024;
    asm volatile(
        "MRC p15, 0, %0, c9, c12, 0\n"
        "BIC %0, %0, #1 << 0\n"   /* disable */
        "MCR p15, 0, %0, c9, c12, 0\n"
        "MRC p15, 0, %0, c9, c13, 0\n"
        : "+r"(temp)
        :
        : "cc"
    );

    //mcSHOW_ERR_MSG((" TimeProfileEnd: %d CPU cycles(%d ms)\n\r", temp, temp/div));
    return (temp/div);
}

#endif

#if CHECK_PLL_OK
int is_pll_good(void)
{
  static int status = -1;
  unsigned int reg_val;
  
  if(status != -1)
  {
    //mcSHOW_DBG_MSG(("[is_pll_good] Already checked before, status = 0x%x\n", status));
    return status;
  }

  /*
    0x10206678[3:2]:
    0x0: 9/15 (PLL 100% good)
    0x1: 9/4, 9/5 (PLL is not good)
    0x2: 9/11, 9/12 (PLL shall be good)
  */  
  reg_val = *(volatile unsigned int*)(0x10206678);
  reg_val = (reg_val & 0xC) >> 2;
  switch (reg_val)
  {
    case 0:
      mcSHOW_DBG_MSG(("[is_pll_good] PLL 100\% good\n"));
      status = 1;
      break;
    case 1:
      mcSHOW_DBG_MSG(("[is_pll_good] PLL is not good\n"));
      status = 0;
      break;
    case 2:
      mcSHOW_DBG_MSG(("[is_pll_good] PLL shall be good\n"));
      status = 0;
      break;
    default:
      mcSHOW_DBG_MSG(("[is_pll_good] Not expected value (0x%x)\n", reg_val));
      status = 0;
      break;      
  }
  return status;  
}  
#endif

#if  EVEREST_MEM_C_TEST_CODE
//EVEREST TEST CODE
void mem_init_sim(void)
{
    DdrPhySetting_Everest_LP3(NULL);
    DramcSetting_Everest_LP3(NULL);
}
#endif
