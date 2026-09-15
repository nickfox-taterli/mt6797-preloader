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

//=============================================================================
//  Include Files                                                      
//=============================================================================
#if __ETT__
#include <common.h>
#include <ett_common.h>
#include <api.h>
#endif

#include "dramc_common.h"
//#include "dramc_register.h"
#include "dramc_pi_api.h"

#if ! __ETT__
#ifdef DDR_RESERVE_MODE  
#include "platform.h"
#include "wdt.h"
#endif
#endif
//=============================================================================
//  Definition                                                         
//=============================================================================

//=============================================================================
//  Global Variables                                                  
//=============================================================================
DRAMC_CTX_T *psCurrDramCtx;

DRAMC_CTX_T DramCtx_LPDDR3 =
{
  CHANNEL_A,          // DRAM_CHANNEL
#ifdef DUAL_RANKS
  RANK_DUAL,        //DRAM_RANK_NUMBER_T
#else
  RANK_SINGLE,
#endif
  RANK_0,               //DRAM_RANK_T
#if DUAL_FREQ_K && DFS_SUPPORT_THIRD_SHUFFLE
  DFS_THIRD_SHUFFLE_FREQ,
#elif DUAL_FREQ_K
  LC_DDR1270,
#else
  LJ_DDR1600,
#endif
  DRAM_DFS_SHUFFLE_NORMAL,
  TYPE_LPDDR3,        // DRAM_DRAM_TYPE_T
  PACKAGE_POP,        // DRAM_PACKAGE_T
  DATA_WIDTH_32BIT,     // DRAM_DATA_WIDTH_T
  DEFAULT_TEST2_1_CAL,    // test2_1;
  DEFAULT_TEST2_2_CAL,    // test2_2;
  TEST_XTALK_PATTERN,     // test_pattern;
  800,
  DISABLE,  // fglow_freq_write_en;
  DISABLE,  // ssc_en;
  DISABLE,   // en_4bitMux;
  
  DISABLE,  // enable_rx_scan_vref;
  DISABLE,   // enable_tx_scan_vref;

   //aru4CalResultFlag[CHANNEL_MAX][RANK_MAX][DRAM_DFS_MAX]
{{{0,0,0,0,},  {0,0,0,0}},
  {{0,0,0,0,}, {0,0,0,0}}}, 
//aru4CalExecuteFlag[CHANNEL_MAX][RANK_MAX][DRAM_DFS_MAX]
{{{0,0,0,0,},  {0,0,0,0}},
  {{0,0,0,0,}, {0,0,0,0}}}, 

#if fcFOR_CHIP_ID == fcA60501
  PCB_LOC_ASIDE,//DRAM_PCB_LOC_T pcb_loc;
  {0,0,0,0,}, //U32 ta43_result[4];   // for eye scan to check falling or rising error
#endif    
#if WRITE_LEVELING_MOVE_DQS_INSTEAD_OF_CLK
  {{0,0}, {0,0}}, //BOOL arfgWriteLevelingInitShif;
#endif
#if TX_PERBIT_INIT_FLOW_CONTROL
  {{FALSE, FALSE}, {FALSE, FALSE}},//BOOL fgTXPerbifInit;
#endif
};


//=============================================================================
//  External references                                                
//=============================================================================
#if  EVEREST_MEM_C_TEST_CODE
#define EMI_Init(x) 
#define print_DBG_info(x) 
#define Dump_EMIRegisters(x)
#else
extern void EMI_Init(DRAMC_CTX_T *p);
#endif

void Dramc_DDR_Reserved_Mode_setting(void)
{
	
    int temp1;
    int temp2;
    int temp3;
		
    mcSHOW_DBG_MSG(("--------------------------------------------\n"));
    mcSHOW_DBG_MSG(("DDR RESERVE MODE Release Flow BEGIN\n"));
    mcSHOW_DBG_MSG(("--------------------------------------------\n"));
    //*((UINT32P)(DDRPHY_BASE  + 0x43c )) |= (0x1 << 24);   //RG_RMCTLPLL_CK_SEL=1
    //change mem_ck to mempll
    //*((UINT32P)(CLK_CFG_0_SET))  = 0x00000100; 
    //*((UINT32P)(CLK_CFG_UPDATE)) = 0x00000002;

    //DDR RESERVE MODE triggered and RST triggered
    mcSHOW_DBG_MSG(("DDR RESERVE MODE enable and RST triggered\n"));
#if 0
    mcSHOW_DBG_MSG(("Disable DDR_CONF isolation\n"));
    *((UINT32P)(WDT_DRAMC_CTL)) = 0x59000000 | (*((UINT32P)(DEBUG_2_REG)) & ~0x400);  // disable DDR_CONF_ISO    

    mcSHOW_DBG_MSG(("Check DDR RESERVE MODE result\n"));
    if ((*((UINT32P)(WDT_DRAMC_CTL)) & 0x10000) == 1)
    {
        //Check DDR RESERVE Success or Not
        mcSHOW_ERR_MSG(("DDR RESERVE Before RST Fail\n"));
    }

    mcSHOW_DBG_MSG(("Wait DDR enter self-refreash mode\n"));
    while ((*((UINT32P)(WDT_DRAMC_CTL)) & 0x20000) == 0)
    {
        //Wait DDR Self Refreash Ready
        mcSHOW_DBG_MSG(("WDT_DRAMC_CTL & 0x20000 = %X\n", (*((UINT32P)(WDT_DRAMC_CTL)) & 0x20000))); 
        *((UINT32P)(MDM_TM_TPAR)) =  *((UINT32P)(WDT_DRAMC_CTL)) & 0x20000;
    }
#endif
    *((UINT32P)(DRAMC_WBR)) = 0x0;
    
    //disable DDRPHY dynamic clock gating
    *((UINT32P)(DRAMC0_BASE  + 0x1dc )) &= ~(0x1 << 30);   //R_DMPHYCLKDYNGEN=0
    *((UINT32P)(DRAMC1_BASE  + 0x1dc )) &= ~(0x1 << 30);   //R_DMPHYCLKDYNGEN=0

    //disable DDRPHY
    //change mcm_ck to 26M
    *((UINT32P)(CLK_CFG_0_CLR))  = 0x00000100; 
    *((UINT32P)(CLK_CFG_UPDATE)) = 0x00000002;

    *((UINT32P)(DDRPHY_BASE  + 0x400 )) &= ~(0x1 << 31);   //RG_*PLL_EN=0
    *((UINT32P)(DDRPHY_BASE  + 0x408 )) &= ~(0x1 << 31);
    *((UINT32P)(DDRPHY1_BASE + 0x400 )) &= ~(0x1 << 31);   //RG_*PLL_EN=0
    *((UINT32P)(DDRPHY1_BASE + 0x408 )) &= ~(0x1 << 31);
    *((UINT32P)(DDRPHY2_BASE + 0x400 )) &= ~(0x1 << 31);   //RG_*PLL_EN=0
    *((UINT32P)(DDRPHY2_BASE + 0x408 )) &= ~(0x1 << 31);
    *((UINT32P)(DDRPHY3_BASE + 0x400 )) &= ~(0x1 << 31);   //RG_*PLL_EN=0
    *((UINT32P)(DDRPHY3_BASE + 0x408 )) &= ~(0x1 << 31);

    *((UINT32P)(DDRPHY_BASE  + 0x01c )) &= ~(0x1 << 16);   //RG_*VREF_EN=0
    *((UINT32P)(DDRPHY_BASE  + 0x07c )) &= ~(0x1 << 16);
    *((UINT32P)(DDRPHY1_BASE + 0x01c )) &= ~(0x1 << 16);   //RG_*VREF_EN=0
    *((UINT32P)(DDRPHY1_BASE + 0x07c )) &= ~(0x1 << 16);
    *((UINT32P)(DDRPHY2_BASE + 0x01c )) &= ~(0x1 << 16);   //RG_*VREF_EN=0
    *((UINT32P)(DDRPHY2_BASE + 0x07c )) &= ~(0x1 << 16);
    *((UINT32P)(DDRPHY3_BASE + 0x01c )) &= ~(0x1 << 16);   //RG_*VREF_EN=0
    *((UINT32P)(DDRPHY3_BASE + 0x07c )) &= ~(0x1 << 16);
    *((UINT32P)(DDRPHY_BASE  + 0x42c )) &= ~((0x1 << 15)/* | (0x1 << 7)*/);   //RG_*MPDIV_EN=0
    *((UINT32P)(DDRPHY1_BASE + 0x42c )) &= ~((0x1 << 15)/* | (0x1 << 7)*/);   //RG_*MPDIV_EN=0
    *((UINT32P)(DDRPHY2_BASE + 0x42c )) &= ~((0x1 << 15)/* | (0x1 << 7)*/);   //RG_*MPDIV_EN=0
    *((UINT32P)(DDRPHY3_BASE + 0x42c )) &= ~((0x1 << 15)/* | (0x1 << 7)*/);   //RG_*MPDIV_EN=0

    *((UINT32P)(DDRPHY_BASE  + 0x420 )) |= ((0x1 << 31) | (0x1 << 27) | (0x1 << 23) | (0x1 << 22) | (0xfffff << 0));   //*PI_CG=1
    *((UINT32P)(DDRPHY_BASE  + 0x42c )) |= ((0x1 << 19)/* | (0x1 << 18)*/);   //RG_*MPDIV_CG=1
    *((UINT32P)(DDRPHY_BASE  + 0x020 )) &= ~(0x1 << 12);   //RG_*BIAS_EN=0
    *((UINT32P)(DDRPHY_BASE  + 0x080 )) &= ~(0x1 << 12);
    *((UINT32P)(DDRPHY1_BASE + 0x420 )) |= ((0x1 << 31) | (0x1 << 27) | (0x1 << 23) | (0x1 << 22) | (0xfffff << 0));   //*PI_CG=1
    *((UINT32P)(DDRPHY1_BASE + 0x42c )) |= ((0x1 << 19)/* | (0x1 << 18)*/);   //RG_*MPDIV_CG=1
    *((UINT32P)(DDRPHY1_BASE + 0x020 )) &= ~(0x1 << 12);   //RG_*BIAS_EN=0
    *((UINT32P)(DDRPHY1_BASE + 0x080 )) &= ~(0x1 << 12);
    *((UINT32P)(DDRPHY2_BASE + 0x420 )) |= ((0x1 << 31) | (0x1 << 27) | (0x1 << 23) | (0x1 << 22) | (0xfffff << 0));   //*PI_CG=1
    *((UINT32P)(DDRPHY2_BASE + 0x42c )) |= ((0x1 << 19)/* | (0x1 << 18)*/);   //RG_*MPDIV_CG=1
    *((UINT32P)(DDRPHY2_BASE + 0x020 )) &= ~(0x1 << 12);   //RG_*BIAS_EN=0
    *((UINT32P)(DDRPHY2_BASE + 0x080 )) &= ~(0x1 << 12);
    *((UINT32P)(DDRPHY3_BASE + 0x420 )) |= ((0x1 << 31) | (0x1 << 27) | (0x1 << 23) | (0x1 << 22) | (0xfffff << 0));   //*PI_CG=1
    *((UINT32P)(DDRPHY3_BASE + 0x42c )) |= ((0x1 << 19)/* | (0x1 << 18)*/);   //RG_*MPDIV_CG=1
    *((UINT32P)(DDRPHY3_BASE + 0x020 )) &= ~(0x1 << 12);   //RG_*BIAS_EN=0
    *((UINT32P)(DDRPHY3_BASE + 0x080 )) &= ~(0x1 << 12);

    *((UINT32P)(DDRPHY_BASE  + 0x430 )) &= ~(0x1 << 11); //RG_*PHDET_EN=0 (CA)
    *((UINT32P)(DDRPHY1_BASE + 0x430 )) &= ~(0x1 << 11); //RG_*PHDET_EN=0 (CB)
    *((UINT32P)(DDRPHY2_BASE + 0x430 )) &= ~(0x1 << 11); //RG_*PHDET_EN=0 (AB)
    *((UINT32P)(DDRPHY3_BASE + 0x430 )) &= ~(0x1 << 11); //RG_*PHDET_EN=0 (AB)


    *((UINT32P)(0x1000631c )) |= (0x1 << 1); //ddrphy_pwr_iso=1
    *((UINT32P)(0x100063b8 )) |= (0x1 << 1);

    *((UINT32P)(DDRPHY_BASE  + 0x014 )) &= ~(0x1 << 3);   //RG_*RESETB=0
    *((UINT32P)(DDRPHY_BASE  + 0x074 )) &= ~(0x1 << 3);
    *((UINT32P)(DDRPHY_BASE  + 0x188 )) &= ~(0x1 << 1);
    *((UINT32P)(DDRPHY_BASE  + 0x43c )) &= ~(0x1 << 16);
    *((UINT32P)(DDRPHY1_BASE + 0x014 )) &= ~(0x1 << 3);   //RG_*RESETB=0
    *((UINT32P)(DDRPHY1_BASE + 0x074 )) &= ~(0x1 << 3);
    *((UINT32P)(DDRPHY1_BASE + 0x188 )) &= ~(0x1 << 1);
    *((UINT32P)(DDRPHY1_BASE + 0x43c )) &= ~(0x1 << 16);
    *((UINT32P)(DDRPHY2_BASE + 0x014 )) &= ~(0x1 << 3);   //RG_*RESETB=0
    *((UINT32P)(DDRPHY2_BASE + 0x074 )) &= ~(0x1 << 3);
    *((UINT32P)(DDRPHY2_BASE + 0x188 )) &= ~(0x1 << 1);
    *((UINT32P)(DDRPHY2_BASE + 0x43c )) &= ~(0x1 << 16);
    *((UINT32P)(DDRPHY3_BASE + 0x014 )) &= ~(0x1 << 3);   //RG_*RESETB=0
    *((UINT32P)(DDRPHY3_BASE + 0x074 )) &= ~(0x1 << 3);
    *((UINT32P)(DDRPHY3_BASE + 0x188 )) &= ~(0x1 << 1);
    *((UINT32P)(DDRPHY3_BASE + 0x43c )) &= ~(0x1 << 16);

    //marked by kaihsin on May 7th, not necessary to PDN DDRPHY
    //*((UINT32P)(0x1000631c )) &= ~(0x1 << 2); //ddrphy_pwr_on=0
    //*((UINT32P)(0x100063b8 )) &= ~(0x1 << 2);

    //wait 1us
    mcDELAY_US(1);
    
    //enable DDRPHY by normal flow
    //marked by kaihsin on May 7th, not necessary to PDN DDRPHY
    //*((UINT32P)(0x1000631c )) |= (0x1 << 2); //ddrphy_pwr_on=1
    //*((UINT32P)(0x100063b8 )) |= (0x1 << 2);
    //wait 1us
    mcDELAY_US(1);

    *((UINT32P)(DDRPHY_BASE  + 0x014 )) |= (0x1 << 3);   //RG_*RESETB=1
    *((UINT32P)(DDRPHY_BASE  + 0x074 )) |= (0x1 << 3);
    *((UINT32P)(DDRPHY_BASE  + 0x188 )) |= (0x1 << 1);
    *((UINT32P)(DDRPHY_BASE  + 0x43c )) |= (0x1 << 16);
    *((UINT32P)(DDRPHY1_BASE + 0x014 )) |= (0x1 << 3);   //RG_*RESETB=1
    *((UINT32P)(DDRPHY1_BASE + 0x074 )) |= (0x1 << 3);
    *((UINT32P)(DDRPHY1_BASE + 0x188 )) |= (0x1 << 1);
    *((UINT32P)(DDRPHY1_BASE + 0x43c )) |= (0x1 << 16);
    *((UINT32P)(DDRPHY2_BASE + 0x014 )) |= (0x1 << 3);   //RG_*RESETB=1
    *((UINT32P)(DDRPHY2_BASE + 0x074 )) |= (0x1 << 3);
    *((UINT32P)(DDRPHY2_BASE + 0x188 )) |= (0x1 << 1);
    *((UINT32P)(DDRPHY2_BASE + 0x43c )) |= (0x1 << 16);
    *((UINT32P)(DDRPHY3_BASE + 0x014 )) |= (0x1 << 3);   //RG_*RESETB=1
    *((UINT32P)(DDRPHY3_BASE + 0x074 )) |= (0x1 << 3);
    *((UINT32P)(DDRPHY3_BASE + 0x188 )) |= (0x1 << 1);
    *((UINT32P)(DDRPHY3_BASE + 0x43c )) |= (0x1 << 16);

    *((UINT32P)(0x1000631c )) &= ~(0x1 << 1); //ddrphy_pwr_iso=0
    *((UINT32P)(0x100063b8 )) &= ~(0x1 << 1);

    //MPLL enable -- TBD
    /*TINFO="=== SPM setting start ==="*/
    *((volatile unsigned int *)(DPY_PWR_CON)) = 0xd ;
    *((volatile unsigned int *)(DPY_CH1_PWR_CON)) = 0xd ;
    *((volatile unsigned int *)(SPM_PLL_CON)) = (*((volatile unsigned int *)(SPM_PLL_CON))) & ~(1<<4);
    *((volatile unsigned int *)(PCM_PWR_IO_EN)) = (*((volatile unsigned int *)(PCM_PWR_IO_EN))) & ~(1<<0) ;
    *((volatile unsigned int *)(SPM_POWER_ON_VAL0)) = (*((volatile unsigned int *)(SPM_POWER_ON_VAL0))) & ~(1<<2);
    /*TINFO="=== SPM setting end ==="*/
              
    /*TINFO="=== backup SPM setting ==="*/              
    temp1 = *((UINT32P)(DDRPHY_BASE  + 0x580 ));
    temp2 = *((UINT32P)(DDRPHY_BASE  + 0x584 ));
    temp3 = *((UINT32P)(DDRPHY_BASE  + 0x588 ));
    /*TINFO="=== backup SPM setting end==="*/     	
              
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
    //wait 20us for MPLL
    mcDELAY_US(20);

    *((UINT32P)(DDRPHY_BASE  + 0x400 )) |= (0x1 << 31);   //RG_*PLL_EN=1
    *((UINT32P)(DDRPHY_BASE  + 0x408 )) |= (0x1 << 31);
    *((UINT32P)(DDRPHY1_BASE + 0x400 )) |= (0x1 << 31);   //RG_*PLL_EN=1
    *((UINT32P)(DDRPHY1_BASE + 0x408 )) |= (0x1 << 31);
    *((UINT32P)(DDRPHY2_BASE + 0x400 )) |= (0x1 << 31);   //RG_*PLL_EN=1
    *((UINT32P)(DDRPHY2_BASE + 0x408 )) |= (0x1 << 31);
    *((UINT32P)(DDRPHY3_BASE + 0x400 )) |= (0x1 << 31);   //RG_*PLL_EN=1
    *((UINT32P)(DDRPHY3_BASE + 0x408 )) |= (0x1 << 31);
    //wait 20us for MEMPLL
    mcDELAY_US(20);

    *((UINT32P)(DDRPHY_BASE  + 0x01c )) |= (0x1 << 16);   //RG_*VREF_EN=1
    *((UINT32P)(DDRPHY_BASE  + 0x07c )) |= (0x1 << 16);
    *((UINT32P)(DDRPHY1_BASE + 0x01c )) |= (0x1 << 16);   //RG_*VREF_EN=1
    *((UINT32P)(DDRPHY1_BASE + 0x07c )) |= (0x1 << 16);
    *((UINT32P)(DDRPHY2_BASE + 0x01c )) |= (0x1 << 16);   //RG_*VREF_EN=1
    *((UINT32P)(DDRPHY2_BASE + 0x07c )) |= (0x1 << 16);
    *((UINT32P)(DDRPHY3_BASE + 0x01c )) |= (0x1 << 16);   //RG_*VREF_EN=1
    *((UINT32P)(DDRPHY3_BASE + 0x07c )) |= (0x1 << 16);
    *((UINT32P)(DDRPHY_BASE  + 0x42c )) |= ((0x1 << 15)/* | (0x1 << 7)*/);   //RG_*MPDIV_EN=1
    *((UINT32P)(DDRPHY1_BASE + 0x42c )) |= ((0x1 << 15)/* | (0x1 << 7)*/);   //RG_*MPDIV_EN=1
    *((UINT32P)(DDRPHY2_BASE + 0x42c )) |= ((0x1 << 15)/* | (0x1 << 7)*/);   //RG_*MPDIV_EN=1
    *((UINT32P)(DDRPHY3_BASE + 0x42c )) |= ((0x1 << 15)/* | (0x1 << 7)*/);   //RG_*MPDIV_EN=1
    //wait 1us
    mcDELAY_US(1);

    //*((UINT32P)(DDRPHY_BASE  + 0x420 )) &= ~((0x1 << 31) | (0x1 << 27) | (0x1 << 23) | (0x1 << 22) | (0xfffff << 0));   //*PI_CG=0
    *((UINT32P)(DDRPHY_BASE  + 0x420 )) = 0x77F43FE1;   //*PI_CG=0
    *((UINT32P)(DDRPHY_BASE  + 0x42c )) &= ~((0x1 << 19)/* | (0x1 << 18)*/);   //RG_*MPDIV_CG=0
    //*((UINT32P)(DDRPHY_BASE  + 0x020 )) |= (0x1 << 12);   //RG_*BIAS_EN=1
    *((UINT32P)(DDRPHY_BASE  + 0x080 )) |= (0x1 << 12);
    //*((UINT32P)(DDRPHY1_BASE + 0x420 )) &= ~((0x1 << 31) | (0x1 << 27) | (0x1 << 23) | (0x1 << 22) | (0xfffff << 0));   //*PI_CG=0
    *((UINT32P)(DDRPHY1_BASE  + 0x420 )) = 0x77F5C31F;   //*PI_CG=0
    *((UINT32P)(DDRPHY1_BASE + 0x42c )) &= ~((0x1 << 19)/* | (0x1 << 18)*/);   //RG_*MPDIV_CG=0
    //*((UINT32P)(DDRPHY1_BASE + 0x020 )) |= (0x1 << 12);   //RG_*BIAS_EN=1
    *((UINT32P)(DDRPHY1_BASE + 0x080 )) |= (0x1 << 12);
    //*((UINT32P)(DDRPHY2_BASE + 0x420 )) &= ~((0x1 << 31) | (0x1 << 27) | (0x1 << 23) | (0x1 << 22) | (0xfffff << 0));   //*PI_CG=0
    *((UINT32P)(DDRPHY2_BASE  + 0x420 )) = 0x77F5C3E1;   //*PI_CG=0
    *((UINT32P)(DDRPHY2_BASE + 0x42c )) &= ~((0x1 << 19)/* | (0x1 << 18)*/);   //RG_*MPDIV_CG=0
    //*((UINT32P)(DDRPHY2_BASE + 0x020 )) |= (0x1 << 12);   //RG_*BIAS_EN=1
    *((UINT32P)(DDRPHY2_BASE + 0x080 )) |= (0x1 << 12);
    //*((UINT32P)(DDRPHY3_BASE + 0x420 )) &= ~((0x1 << 31) | (0x1 << 27) | (0x1 << 23) | (0x1 << 22) | (0xfffff << 0));   //*PI_CG=0
    *((UINT32P)(DDRPHY3_BASE  + 0x420 )) = 0x77F5C3E1;   //*PI_CG=0
    *((UINT32P)(DDRPHY3_BASE + 0x42c )) &= ~((0x1 << 19)/* | (0x1 << 18)*/);   //RG_*MPDIV_CG=0
    //*((UINT32P)(DDRPHY3_BASE + 0x020 )) |= (0x1 << 12);   //RG_*BIAS_EN=1
    *((UINT32P)(DDRPHY3_BASE + 0x080 )) |= (0x1 << 12);

    //wait 1us
    mcDELAY_US(1);  
     //*((UINT32P)(DDRPHY_BASE  + 0x43c )) &= ~(0x1 << 24);   //RG_RMCTLPLL_CK_SEL=0
    //change mem_ck to mempll
    *((UINT32P)(CLK_CFG_0_SET))  = 0x00000100;  
    *((UINT32P)(CLK_CFG_UPDATE)) = 0x00000002;

    //force top feedback MCK not divide
    //rg_ddrphy_fb_ck_force_en = 1 -- TBD
    //*((UINT32P)(CLK_MEM_DFS_CFG)) |= (0x1 << 8); //rg_ddrphy_fb_ck_force_en = 1
    //disable FB_CK divider
    *((UINT32P)(CLK_MEM_DFS_CFG)) = 0x0000111F;

    //disable M_CK free-run divider & free_run fb_ck cg
    *((UINT32P)(DFS_MEM_DCM_CTRL)) &=  ~((0x1 <<9) | (0x1 <<4) | (0x1 <<3)); //[3]: ch0 M_CK free-run divider, [4]: ch1 M_CK free-run divider, [9] free_run fb_ck cg

    //*((UINT32P)DRAMC_WBR)) = 0x3;

    //DLL enable
    *((UINT32P)(DDRPHY_BASE  + 0x430 )) |= (0x1 << 11); //RG_*PHDET_EN=1 (CA)
    // wait 1us
    mcDELAY_US(1);
    
    *((UINT32P)(DDRPHY1_BASE + 0x430 )) |= (0x1 << 11); //RG_*PHDET_EN=1 (CB)
    //*((UINT32P)(DDRPHY2_BASE + 0x430 )) |= 0x00660e00; //RG_*PHDET_EN=1 (AB) //temp workaround
    //*((UINT32P)(DDRPHY3_BASE + 0x430 )) |= 0x00660e00; //RG_*PHDET_EN=1 (AB) //temp workaround
    *((UINT32P)(DDRPHY2_BASE + 0x430 )) |= (0x1 << 11); //RG_*PHDET_EN=1 (AB)
    *((UINT32P)(DDRPHY3_BASE + 0x430 )) |= (0x1 << 11); //RG_*PHDET_EN=1 (AB)
    //wait 100us
    mcDELAY_US(100);

    //top feedback MCK to divided frequency
    //*((UINT32P)(CLK_MEM_DFS_CFG)) &= ~(0x1 << 8); //rg_ddrphy_fb_ck_force_en = 0
    //After DLL lock, enable FB_CK divider End
    *((UINT32P)(CLK_MEM_DFS_CFG)) = 0x0000101F;

    //enable M_CK free-run divider & free_run fb_ck cg
    *((UINT32P)(DFS_MEM_DCM_CTRL)) |=  ((0x1 <<9) | (0x1 <<4) | (0x1 <<3)); //[3]: ch0 M_CK free-run divider, [4]: ch1 M_CK free-run divider, [9] free_run fb_ck cg

    //*((UINT32P)(DRAMC0_BASE  + 0x0f4 )) |= (0x1 << 25);   //R_DMRDATRST=1
    //*((UINT32P)(DRAMC1_BASE  + 0x0f4 )) |= (0x1 << 25);   //R_DMRDATRST=1
    //*((UINT32P)(DRAMC0_BASE  + 0x0f4 )) &= ~(0x1 << 25);   //R_DMRDATRST=0
    //*((UINT32P)(DRAMC1_BASE  + 0x0f4 )) &= ~(0x1 << 25);   //R_DMRDATRST=0
    
    //*((UINT32P)(DRAMC_WBR)) = 0x7;

    //enable DDRPHY dynamic clock gating
    *((UINT32P)(DRAMC0_BASE  + 0x1dc )) |= (0x1 << 30);   //R_DMPHYCLKDYNGEN=1
    *((UINT32P)(DRAMC1_BASE  + 0x1dc )) |= (0x1 << 30);   //R_DMPHYCLKDYNGEN=1

    mcSHOW_DBG_MSG(("enable DDRPHY by normal flow end\n"));
#if 0
    //disable DRAMC_ISO and DRAMC Self Refresh Control
    *((UINT32P)(WDT_DRAMC_CTL)) =  0x59000000 | (*((UINT32P)(DEBUG_2_REG)) & ~0x300);

    ///for (i=0; i<10; i++);
    mcDELAY_US(1);

    mcSHOW_DBG_MSG(("Wait DDR exit self-refreash mode\n"));
    while (*((UINT32P)(WDT_DRAMC_CTL)) & 0x20000)
    {
        //Wait DDR Self Refreash Release
        mcSHOW_DBG_MSG(("WDT_DRAMC_CTL & 0x20000 = %X\n", (*((UINT32P)(WDT_DRAMC_CTL)) & 0x20000)));
    }

    *(UINT32P)(0x40000000) = 0x12345678;
    *(UINT32P)(0x40000004) = *(UINT32P)(0x40000000) ;

    ///for (i=0; i<10; i++);
    mcDELAY_US(1);

    *((UINT32P)(EMI_MPUP)) = 0x200; // In order to sync emi_reg setting to emi

    ///for (i=0; i<10; i++);
    mcDELAY_US(1);
#endif    
    *((UINT32P)(DRAMC_WBR)) = 0x7;
    

  /*TINFO="=== set ddrphy peri to spm control ==="*/
    RISCWriteDDRPHY(0x0580 , temp1);
    RISCWriteDDRPHY(0x0584 , temp2);
    RISCWriteDDRPHY(0x0588 , temp3);
  /*TINFO="=== set ddrphy peri to spm control end==="*/

    *((UINT32P)(DRAMC_WBR)) = 0x0;
    
    mcSHOW_DBG_MSG(("--------------------------------------------\n"));
    mcSHOW_DBG_MSG(("End of DDR RESERVE MODE Release END\n"));
    mcSHOW_DBG_MSG(("--------------------------------------------\n"));
}

void Dramc_DDR_Reserved_Mode_JumpToHigh(void)
{
#if DUAL_FREQ_K
#if DFS_COMBINATION_TYPE1
    if(get_chip_id_by_efuse() == CHIP_MT6797M)
    {
        DDRPhyFreqSel(&DramCtx_LPDDR3, LJ_DDR1600);
        DramcDFSDirectJump(&DramCtx_LPDDR3, 0, 0, 0);
        DDRPhyFreqSel(&DramCtx_LPDDR3, LC_DDR1600);
        DramcDFSDirectJump(&DramCtx_LPDDR3, 0, 1, 0);
        
    }
    else
    {
        DDRPhyFreqSel(&DramCtx_LPDDR3, LJ_DDR1866);
        DramcDFSDirectJump(&DramCtx_LPDDR3, 0, 0, 0);
    }
#elif DFS_COMBINATION_TYPE3    
    if(get_chip_id_by_efuse() == CHIP_MT6797M)
    {
        DDRPhyFreqSel(&DramCtx_LPDDR3, LJ_DDR1600);
        DramcDFSDirectJump(&DramCtx_LPDDR3, 0, 0, 0);
        DDRPhyFreqSel(&DramCtx_LPDDR3, LC_DDR1600);
        DramcDFSDirectJump(&DramCtx_LPDDR3, 0, 1, 0);
        
    }
    else
    {
        DDRPhyFreqSel(&DramCtx_LPDDR3, LJ_DDR1700);
        DramcDFSDirectJump(&DramCtx_LPDDR3, 0, 0, 0);
    }
#else
    DDRPhyFreqSel(&DramCtx_LPDDR3, LJ_DDR1600);
    DramcDFSDirectJump(&DramCtx_LPDDR3, 0, 0, 0);
    DDRPhyFreqSel(&DramCtx_LPDDR3, LC_DDR1600);
    DramcDFSDirectJump(&DramCtx_LPDDR3, 0, 1, 0);
#endif
#endif
}

void Dramc_DDR_Reserved_Mode_ResetShuffle(void)
{
#if DUAL_FREQ_K
	DRAM_PLL_FREQ_SEL_T dft_freq_sel = DramCtx_LPDDR3.freq_sel;

	*((UINT32P)(DDRPHY_BASE + 0x63c )) &= ~(0x3);
	*((UINT32P)(DDRPHY1_BASE + 0x63c )) &= ~(0x3);
	*((UINT32P)(DDRPHY2_BASE + 0x63c )) &= ~(0x3);
	*((UINT32P)(DDRPHY3_BASE + 0x63c )) &= ~(0x3);

#if DFS_COMBINATION_TYPE1
    if(get_chip_id_by_efuse() == CHIP_MT6797M)
    {
        DDRPhyFreqSel(&DramCtx_LPDDR3, LJ_DDR1600);
        DramcDFSDirectJump(&DramCtx_LPDDR3, 0, 0, 0);
    }
    else
    {
        DDRPhyFreqSel(&DramCtx_LPDDR3, LJ_DDR1866);
        DramcDFSDirectJump(&DramCtx_LPDDR3, 0, 0, 0);
    }
#elif DFS_COMBINATION_TYPE3    
    if(get_chip_id_by_efuse() == CHIP_MT6797M)
    {
        DDRPhyFreqSel(&DramCtx_LPDDR3, LJ_DDR1600);
        DramcDFSDirectJump(&DramCtx_LPDDR3, 0, 0, 0);
    }
    else
    {
        DDRPhyFreqSel(&DramCtx_LPDDR3, LJ_DDR1700);
        DramcDFSDirectJump(&DramCtx_LPDDR3, 0, 0, 0);
    }
#else
    DDRPhyFreqSel(&DramCtx_LPDDR3, LJ_DDR1600);
    DramcDFSDirectJump(&DramCtx_LPDDR3, 0, 0, 0);
#endif
	DDRPhyFreqSel(&DramCtx_LPDDR3, dft_freq_sel);
#endif
}

#if ENABLE_LCPLL_IC_SCAN
void TA2_Stress_Test(DRAMC_CTX_T *p)
{
    U32 err_count = 0;
    U32 pass_count = 0; 
    U32 u4ErrorValue;
    U8 u1ChannelIdx, u1RankIdx;
    U32 u4TestCount;
    U8 u1RankMax;

    for(u4TestCount=0; u4TestCount<10000; u4TestCount++)
    {

            for(u1ChannelIdx =0; u1ChannelIdx< CHANNEL_MAX; u1ChannelIdx++)
            {
                #if SINGLE_CHANNEL_ENABLE
                if(u1ChannelIdx>0)
                    break; //Skip channel B
                #endif
                
                p->channel = u1ChannelIdx;

                #ifdef DUAL_RANKS
                if(uiDualRank)
                    u1RankMax = RANK_MAX;
                else
                #endif
                     u1RankMax =RANK_1;

                for(u1RankIdx =0 ; u1RankIdx< u1RankMax; u1RankIdx++)
                {
                    DramcSetRankEngine2(p, u1RankIdx);
                    //u4ErrorValue = DramcEngine2(p, TE_OP_WRITE_READ_CHECK, p->test2_1, 0xaaffffff, 2, 0, 0, 0);
                    //u4ErrorValue = DramcEngine2(p, TE_OP_WRITE_READ_CHECK, p->test2_1, 0xaa7fffff, 2, 0, 0, 0);
                    u4ErrorValue = DramcEngine2(p, TE_OP_WRITE_READ_CHECK, p->test2_1, 0xaa000400, 2, 0, 0, 0);
                    
                    if (u4ErrorValue > 0)
                    {
                        err_count++;
                        mcSHOW_DBG_MSG(("channel(%d) Rank(%d), TA2 failed, pass_cnt:%d,err_cnt:%d, u4ErrorValue 0x%x\n", u1ChannelIdx, u1RankIdx, pass_count,err_count, u4ErrorValue));
                    }
                    else
                    {
                        pass_count++;
                        mcSHOW_DBG_MSG(("channel (%d) Rank(%d),TA2 pass, pass_cnt:%d,err_cnt:%d\n", u1ChannelIdx, u1RankIdx, pass_count,err_count));
                    }     
                    
                    #if LJPLL_FREQ_DEBUG_LOG
                    DDRPhyFreqMeter();
                    #endif
                    mcDELAY_US(100); 
                }
            }
        }
}

static void vPhypll_IC_Scan(DRAMC_CTX_T *p)
{
    U8 ucExecute[CHANNEL_MAX][RANK_MAX], ucResult[CHANNEL_MAX][RANK_MAX];
    
    mcSHOW_DBG_MSG(("\n\n=====================\n"));
    mcSHOW_DBG_MSG(("[LCPLL_IC_SCAN] Channel = %d, Rank= %d, Freq.= 400Mhz, (ucCalExecute_All 0x%x, ucCalResult_All 0x%x)\n", \
                                        CHANNEL_A, p->rank, \
                                        p->aru4CalExecuteFlag[p->channel][p->rank][p->shu_type], \
                                        p->aru4CalResultFlag[p->channel][p->rank][p->shu_type]));
    
    mcSHOW_DBG_MSG(("[LCPLL_IC_SCAN] Channel = %d, Rank= %d, Freq.= 400Mhz, (ucCalExecute_All 0x%x, ucCalResult_All 0x%x)\n", \
                                        CHANNEL_B, p->rank, \
                                        p->aru4CalExecuteFlag[p->channel][p->rank][p->shu_type], \
                                        p->aru4CalResultFlag[p->channel][p->rank][p->shu_type]));
    
    if((p->aru4CalResultFlag[CHANNEL_A][p->rank][p->shu_type]==DRAM_OK) && \
        (p->aru4CalResultFlag[CHANNEL_B][p->rank][p->shu_type]==DRAM_OK))
        mcSHOW_DBG_MSG(("LCPLL PASS\n"));
    else
        mcSHOW_DBG_MSG(("LCPLL FAIL !!!!!!\n"));
    
    mcSHOW_DBG_MSG(("=====================\n\n"));
    
        TA2_Stress_Test(p);
        while(1);

}
#endif  //ENABLE_LCPLL_IC_SCAN


#if CPU_RW_TEST_AFTER_K
void vDramCPUReadWriteTestAfterCalibration(DRAMC_CTX_T *p)
{
    U8 u1DumpInfo=0;
    U32 uiLen, count, uiFixedAddr;
    U32 pass_count, err_count;
    uiLen = 0xff00;

    err_count=0;
    pass_count=0;
    uiFixedAddr = DDR_BASE+0x10000000;
    
#if GATING_ONLY_FOR_DEBUG
    DramcGatingDebugInit(p);
    DramcGatingDebugRankSel(p, RANK_0);
#endif
    
    for (count=uiFixedAddr; count<uiFixedAddr+uiLen; count+=4)
    {
        *(volatile unsigned int   *)(count) = count;
    }

    for (count=uiFixedAddr; count<uiFixedAddr+uiLen; count+=4)
    {
        if (*(volatile unsigned int   *)(count) != count)
        {
            //mcSHOW_DBG_MSG(("[Fail] Addr %xh = %xh\n",count, *(volatile unsigned int   *)(count)));
            err_count++;
        }
        else
            pass_count ++;
    }     
    
    if(err_count)
    {       
        mcSHOW_DBG_MSG(("[MEM_TEST] Rank0 Fail. (uiFixedAddr 0x%X, Pass count =%d, Fail count =%d)\n", uiFixedAddr, pass_count, err_count));
        u1DumpInfo =1;
    }
    else
    {
        mcSHOW_DBG_MSG(("[MEM_TEST] Rank0 OK. (uiFixedAddr 0x%X, Pass count =%d, Fail count =%d)\n", uiFixedAddr, pass_count, err_count));
    }

    #ifdef DUAL_RANKS 
    if (uiDualRank) 
    {
        #if GATING_ONLY_FOR_DEBUG
        DramcGatingDebugRankSel(p, RANK_1);
        #endif
        
        err_count=0;
        pass_count=0;
        
        #if SINGLE_CHANNEL_ENABLE
            uiFixedAddr =DDR_BASE+0x30000000;
        #else
            uiFixedAddr =DDR_BASE+0x60000000;
        #endif   
        
        for (count=uiFixedAddr; count<uiFixedAddr+uiLen; count+=4)
        {
            *(volatile unsigned int   *)(count) = count;
        }

        for (count=uiFixedAddr; count<uiFixedAddr+uiLen; count+=4)
        {
            if (*(volatile unsigned int   *)(count) != count)
            {
                //mcSHOW_DBG_MSG(("[Fail] Addr %xh = %xh\n",count, *(volatile unsigned int   *)(count)));
                err_count++;
            }
            else
                pass_count ++;
        }       
        
        if(err_count)
        {       
            mcSHOW_DBG_MSG(("[MEM_TEST] Rank1 Fail. (uiFixedAddr 0x%X, Pass count =%d, Fail count =%d)\n", uiFixedAddr, pass_count, err_count));
            u1DumpInfo =1;
        }
        else
        {
            mcSHOW_DBG_MSG(("[MEM_TEST] Rank1 OK. (uiFixedAddr 0x%X, Pass count =%d, Fail count =%d)\n", uiFixedAddr, pass_count, err_count));
        }
    }
    #endif

    if(u1DumpInfo)
    {
        // Read gating error flag
        DramcDumpDebugInfo(p); 
    }
    #if GATING_ONLY_FOR_DEBUG
    DramcGatingDebugExit(p);
    #endif
}
#endif



#define TIME_RROFILE_BREAK_DOWN 0
static void vDramCalibrationSingleChannel(DRAMC_CTX_T *p)
{
    U8 u1RankMax;
    S8 s1RankIdx;
#if TIME_RROFILE_BREAK_DOWN
    UINT32 CPU_Cycle, CPUCycleSum=0;
    TimeProfileBegin();
#endif    

#if ENABLE_CA_TRAINING  // skip when bring up
    DramcRankSwap(p, RANK_0);
    DramcCATraining((DRAMC_CTX_T *) p);   //Dram will be reset when finish CA training.
#endif

#if TIME_RROFILE_BREAK_DOWN
    CPU_Cycle=TimeProfileEnd();    
    CPUCycleSum += CPU_Cycle;
    mcSHOW_ERR_MSG(("DRAMC CA train takes %d ms\n\r", CPU_Cycle));
    TimeProfileBegin();
#endif

#if GATING_ADJUST_TXDLY_FOR_TRACKING
    DramcRxdqsGatingPreProcess(p);
#endif

#ifdef DUAL_RANKS
if(uiDualRank)
    u1RankMax = RANK_MAX;
else
#endif
     u1RankMax =RANK_1;

    for(s1RankIdx=(u1RankMax-1); s1RankIdx>=0; s1RankIdx--)   
    {
        DramcRankSwap(p, s1RankIdx);

        #ifdef DUAL_RANKS
        if((uiDualRank==0)|| ((uiDualRank==1) && (s1RankIdx == RANK_1)))
        #endif
        {
            #if ENABLE_WRITE_LEVELING
            mcSHOW_DBG_MSG(("Rank %d write leveling calibration\n", s1RankIdx));
            DramcWriteLeveling((DRAMC_CTX_T *) p);//Dram will be reset when finish write leveling
            #endif
        
            #if TIME_RROFILE_BREAK_DOWN
            CPU_Cycle=TimeProfileEnd();    
            CPUCycleSum += CPU_Cycle;
            mcSHOW_ERR_MSG(("Rank %d Write leveling takes %d ms\n\r", s1RankIdx, CPU_Cycle));
            TimeProfileBegin();
            #endif
        }

        #if LJPLL_FREQ_DEBUG_LOG
        DDRPhyFreqMeter();
        #endif
        
        DramcRxdqsGatingCal(p);
        
        #if TIME_RROFILE_BREAK_DOWN
        CPU_Cycle=TimeProfileEnd();    
        CPUCycleSum += CPU_Cycle;
        mcSHOW_ERR_MSG(("Rank %d Gating takes %d ms\n\r", s1RankIdx, CPU_Cycle));
        TimeProfileBegin();
        #endif

        #if LJPLL_FREQ_DEBUG_LOG
        DDRPhyFreqMeter();
        #endif

        DramcRxdatlatCal((DRAMC_CTX_T *) p);

        #if TIME_RROFILE_BREAK_DOWN
        CPU_Cycle=TimeProfileEnd();    
        CPUCycleSum += CPU_Cycle;
        mcSHOW_ERR_MSG(("Rank %d Datlat takes %d ms\n\r", s1RankIdx CPU_Cycle));
        TimeProfileBegin();
        #endif

        #if LJPLL_FREQ_DEBUG_LOG
        DDRPhyFreqMeter();
        #endif

        #ifdef DUAL_RANKS
        #ifndef DUAL_RANK_RX_K
        if(s1RankIdx==RANK_1)
        #endif
        #endif
        {
            mcSHOW_DBG_MSG(("Rank %d RX calibration\n", s1RankIdx));
            DramcRxWindowPerbitCal((DRAMC_CTX_T *) p, 1);
            
            #if TIME_RROFILE_BREAK_DOWN
            CPU_Cycle=TimeProfileEnd();    
            CPUCycleSum += CPU_Cycle;
            mcSHOW_ERR_MSG(("Rnak %d RX takes %d ms\n\r", s1RankIdx CPU_Cycle));      
            TimeProfileBegin();
            #endif
        }

        #ifdef DUAL_RANKS
        #ifndef DUAL_RANK_TX_K
        if(s1RankIdx==RANK_1)
        #endif
        #endif
        {
            mcSHOW_DBG_MSG(("Rank %d TX calibration\n", s1RankIdx));
            DramcTxWindowPerbitCal((DRAMC_CTX_T *) p);   
        }
    }    

    #if GATING_ADJUST_TXDLY_FOR_TRACKING 
        DramcRxdqsGatingPostProcess(p);
    #endif

    #ifdef DUAL_RANKS
    if (uiDualRank) 
    {    
        DramcDualRankRxdatlatCal(p);
    }
    #endif

#if LJPLL_FREQ_DEBUG_LOG
    DDRPhyFreqMeter();
#endif
}

static void vDramCalibrationAllChannel(DRAMC_CTX_T *p)
{
    vSetPHY2ChannelMapping(p, CHANNEL_A); // when switching channel, must update PHY to Channel Mapping
    vDramCalibrationSingleChannel(p);

    #if !SINGLE_CHANNEL_ENABLE
    vSetPHY2ChannelMapping(p, CHANNEL_B);// when switching channel, must update PHY to Channel Mapping
    vDramCalibrationSingleChannel(p);
    #endif
}

#if ENABLE_MRR_AFTER_FIRST_K
int GetDramInforAfterCalByMRR(DRAMC_CTX_T *p, DRAM_INFO_BY_MRR_T *DramInfo)
{
    U8 u1ChannelIdx, u1RankIdx, u1RankMax, u1Density;
    U32 u4Size;
    vSetPHY2ChannelMapping(p, CHANNEL_A);
    DramcModeRegReadByRank(p, RANK_0, 5, &(DramInfo->u1MR5VendorID));
    mcSHOW_DBG_MSG(("[GetDramInforAfterCalByMRR] Vendor %x.\n", DramInfo->u1MR5VendorID));

    for(u1ChannelIdx=0; u1ChannelIdx<CHANNEL_MAX; u1ChannelIdx++)
        for(u1RankIdx =0; u1RankIdx<RANK_MAX; u1RankIdx++)
        {
            #ifdef DUAL_RANKS
            if((uiDualRank==0) && (u1RankIdx==RANK_1))
            {
                //Single rank, set rank 1 desity as 0
                DramInfo->u4MR8Density[u1ChannelIdx][u1RankIdx] =0;
            }
            else
            #endif// end of DUAL_RANKS
            {
            	if((p->aru4CalExecuteFlag[u1ChannelIdx][u1RankIdx][p->shu_type] !=0)  && \
            	    (p->aru4CalResultFlag[u1ChannelIdx][u1RankIdx][p->shu_type]==0))
            	{         
            	    vSetPHY2ChannelMapping(p, u1ChannelIdx);
            	    DramcModeRegReadByRank(p, u1RankIdx, 8, &u1Density);
            	    u1Density = (u1Density>>2)&0xf;
            	    switch(u1Density)
            	    {
            	        case 0x6:
            	            u4Size = 0x20000000;  //4Gb
            	            //DBG_MSG("[EMI]DRAM density = 4Gb\n");
            	            break;
            	        case 0xE:
            	            u4Size = 0x30000000;  //6Gb
            	            //DBG_MSG("[EMI]DRAM density = 6Gb\n");
            	            break;
            	        case 0x7:
            	            u4Size = 0x40000000;  //8Gb
            	            //DBG_MSG("[EMI]DRAM density = 8Gb\n");
            	            break;
            	        case 0xD:
            	            u4Size = 0x60000000;  //12Gb
            	            //DBG_MSG("[EMI]DRAM density = 12Gb\n");
            	            break;
            	        case 0x8:
            	            u4Size = 0x80000000;  //16Gb
            	            //DBG_MSG("[EMI]DRAM density = 16Gb\n");
            	            break;
            	        //case 0x9:
            	            //u4Size = 0x100000000L; //32Gb
            	            //DBG_MSG("[EMI]DRAM density = 32Gb\n");
            	            //break;
            	        default:
            	            u4Size = 0; //reserved
            	    }  
            	    DramInfo->u4MR8Density[u1ChannelIdx][u1RankIdx] = u4Size;
            	}
            	else
            	    DramInfo->u4MR8Density[u1ChannelIdx][u1RankIdx] =0;
            	
            	mcSHOW_DBG_MSG(("[GetDramInforAfterCalByMRR] Channel %d, Rank %d, Desity %x.\n", u1ChannelIdx,u1RankIdx, DramInfo->u4MR8Density[u1ChannelIdx][u1RankIdx]));
            }
        }

        //while(1);
    return 0;
}
#endif

#if ENABLE_RANK_NUMBER_AUTO_DETECTION
static U8 DramRankNumberDetection(DRAMC_CTX_T *p)
{
    U8 u1RankBak;

    u1RankBak= p->rank;  // backup current rank setting
    
    vSetPHY2ChannelMapping(p, CHANNEL_A); // when switching channel, must update PHY to Channel Mapping
    DramcRankSwap(p, RANK_1); 
    DramcWriteLeveling((DRAMC_CTX_T *) p);//Dram will be reset when finish write leveling
    if(((p->aru4CalResultFlag[p->channel][p->rank][p->shu_type] >>DRAM_CALIBRATION_WRITE_LEVEL) &0x1) ==0)
    {
        p->support_rank_num = 2;
    }
    else
    {
        p->support_rank_num = 1;
    }
    mcSHOW_DBG_MSG(("[DramRankNumberDetection] %d, %X\n", p->support_rank_num, p->aru4CalResultFlag[p->channel][p->rank][p->shu_type]));
    
    DramcRankSwap(p, u1RankBak);  // restore rank setting

    return (p->support_rank_num-1);
}
#endif


int Init_DRAM(DRAM_INFO_BY_MRR_T *DramInfo)
{
    #ifdef DDR_INIT_TIME_PROFILING
    UINT32 CPU_Cycle, CPUCycleSum=0;
    #endif
    int mem_start,len;
    DRAMC_CTX_T * p;
    U8 ucstatus = 0;
    U32 u4value;

    psCurrDramCtx = &DramCtx_LPDDR3;
    default_emi_setting = &emi_setting_default_lpddr3;      
    p = psCurrDramCtx;  

    EMI_Init(p);    

#if ENABLE_MRR_AFTER_FIRST_K
   if(DramInfo != NULL)
#endif
    {
        mcSHOW_DBG_MSG(("\n\n[Everest] ETT version 0.0.9.7\n\n"));

#ifdef LOOPBACK_TEST
        mcSHOW_DBG_MSG(("\n\nChannel A loop-back test...n\n"));
        DdrPhyInit((DRAMC_CTX_T *) p);
        DramcSwImpedanceCal((DRAMC_CTX_T *) p, 1);    
        p->channel = CHANNEL_A;
        DramcLoopbackTest(p);
        while (1);
#endif

#if defined(DDR_INIT_TIME_PROFILING)
          TimeProfileBegin();
#endif

#if 0//defined(DDR_INIT_TIME_PROFILING)
          TimeProfileBegin();
          mcDELAY_US(1000000);
          CPU_Cycle=TimeProfileEnd();    
          mcSHOW_ERR_MSG((" mcDELAY_US(1000) takes %d us\n\r", CPU_Cycle));  
#endif
      
#if 0//def Enable_GP6_TIMER
      U32 l_low_tick0, l_high_tick0;
          GPT_Start(GPT6);
          l_low_tick0 = GPT_GetTickCount(&l_high_tick0);
          must_print("Before GPT_GetTickCount = 0x%x 0x%x\n", l_high_tick0, l_low_tick0);
          delay_a_while (6500000);
          l_low_tick0 = GPT_GetTickCount(&l_high_tick0);
          must_print("After GPT_GetTickCount = 0x%x 0x%x\n", l_high_tick0, l_low_tick0);
          //while(1);
#endif

      DramcSwImpedanceCal((DRAMC_CTX_T *) p, 1);  //BringUp : Maybe bypass.
      
      // DramC & PHY init for all channels  
      DDRPhyFreqSel(p, p->freq_sel);

#if CHECK_PLL_OK
      if(is_pll_good() == 0) //not all pll is good, don't use LCPLL to do calibration
      {
            DDRPhyFreqSel(p, LJ_DDR1600);
      }
#endif

      DdrPhyInit((DRAMC_CTX_T *) p);
      DramcInit((DRAMC_CTX_T *) p);   

#ifdef FTTEST_ZQONLY
      while (1)
      {
          ucstatus |= ucDram_Register_Write(mcSET_DRAMC_REG_ADDR(0x88), LPDDR3_MODE_REG_10);
          ucstatus |= ucDram_Register_Write(mcSET_DRAMC_REG_ADDR(0x1e4), 0x00000001);
          mcDELAY_US(1);    // tZQINIT>=1us
          ucstatus |= ucDram_Register_Write(mcSET_DRAMC_REG_ADDR(0x1e4), 0x00000000);
      }
#endif

#ifdef ENABLE_MIOCK_JMETER
        vSetPHY2ChannelMapping(p, CHANNEL_A);
        DramcMiockJmeter(p);
        #if !SINGLE_CHANNEL_ENABLE
        vSetPHY2ChannelMapping(p, CHANNEL_B);
        DramcMiockJmeter(p);
        #endif
#else
        #if CHECK_PLL_OK
            if(is_pll_good() == 0) //not all pll is good, wait a while for PLL stable
            {
                  mcDELAY_MS(1000);  
            }        
        #endif
#endif

#if ENABLE_RANK_NUMBER_AUTO_DETECTION  // only need to do this when #define DUAL_RANKS
        uiDualRank=DramRankNumberDetection(p);
        DFSInitForCalibration(p);  // Restore setting after rank dection (especially DQ= DQS+16)
#endif

        vDramCalibrationAllChannel(p);

#if ENABLE_MRR_AFTER_FIRST_K
        GetDramInforAfterCalByMRR(p, DramInfo);
        return 0;
#endif
    }
  
#if ENABLE_LCPLL_IC_SCAN 
    vPhypll_IC_Scan(p);
#endif
  
#if DUAL_FREQ_K
    #if CHECK_PLL_OK
    if(is_pll_good() == 1) //not all pll is good, don't use LCPLL to do calibration
    #endif
    {
        ///// COPY from DFSSetupFlow()//////////
        #if DFS_COMBINATION_TYPE3    
            #if DFS_SUPPORT_THIRD_SHUFFLE
            // calibration of 1066C is done. save to shuffle2
            DramcSaveToShuffleReg(p, DRAM_DFS_SHUFFLE_2);
            DFSPLLSettingForLP800(p);//LP800 MPDIV

            //1270C
            DDRPhyFreqSel(p, LC_DDR1270);
            DFSInitForCalibration(p);
            vDramCalibrationAllChannel(p);
            #endif
            DramcSaveToShuffleReg(p, DRAM_DFS_SHUFFLE_1);

            if(get_chip_id_by_efuse() == CHIP_MT6797M)
            {
                //1600C
                DDRPhyFreqSel(p, LC_DDR1600);
                DFSInitForCalibration(p);
                vDramCalibrationAllChannel(p);

                #if CPU_RW_TEST_AFTER_K
                mcSHOW_DBG_MSG(("\n[MEM_TEST] 00: After calibration, before DFS\n"));
                vDramCPUReadWriteTestAfterCalibration(p);
                #endif

                //1600P
                DDRPhyFreqSel(p, LJ_DDR1600);
                DFSSwitchFreq(p);
                p->channel = CHANNEL_A;
                Dram_Reset(p);
                p->channel = CHANNEL_B;
                Dram_Reset(p);

                #if CPU_RW_TEST_AFTER_K
                mcSHOW_DBG_MSG(("\n[MEM_TEST] 01: After calibration, before DFS\n"));
                vDramCPUReadWriteTestAfterCalibration(p);
                #endif

                DDRPhyFreqSel(p, LC_DDR1600);
                DramcDFSDirectJump(p, 0, 1, 0);
            }
            else
            {
                #if DFS_SUPPORT_THIRD_SHUFFLE
                DDRPhyFreqSel(p, LJ_DDR1600);
                DFSSwitchFreq(p);
                #endif
                //1700P
                DDRPhyFreqSel(p, LJ_DDR1700);
                DFSInitForCalibration(p);
                vDramCalibrationAllChannel(p);
            }
        #elif DFS_COMBINATION_TYPE1
            #if DFS_SUPPORT_THIRD_SHUFFLE
            // calibration of 1066C is done. save to shuffle2
            DramcSaveToShuffleReg(p, DRAM_DFS_SHUFFLE_2);
            DFSPLLSettingForLP800(p);//LP800 MPDIV

            //1270C
            DDRPhyFreqSel(p, LC_DDR1270);
            DFSInitForCalibration(p);
            vDramCalibrationAllChannel(p);
            #endif
            DramcSaveToShuffleReg(p, DRAM_DFS_SHUFFLE_1);
            
            if(get_chip_id_by_efuse() == CHIP_MT6797M)
            {
                //1600C
                DDRPhyFreqSel(p, LC_DDR1600);
                DFSInitForCalibration(p);
                vDramCalibrationAllChannel(p);

                #if CPU_RW_TEST_AFTER_K
                mcSHOW_DBG_MSG(("\n[MEM_TEST] 00: After calibration, before DFS\n"));
                vDramCPUReadWriteTestAfterCalibration(p);
                #endif

                //1600P
                DDRPhyFreqSel(p, LJ_DDR1600);
                DFSSwitchFreq(p);
                p->channel = CHANNEL_A;
                Dram_Reset(p);
                p->channel = CHANNEL_B;
                Dram_Reset(p);

                #if CPU_RW_TEST_AFTER_K
                mcSHOW_DBG_MSG(("\n[MEM_TEST] 01: After calibration, before DFS\n"));
                vDramCPUReadWriteTestAfterCalibration(p);
                #endif

                DDRPhyFreqSel(p, LC_DDR1600);
                DramcDFSDirectJump(p, 0, 1, 0);
            }
            else
            {
                #if DFS_SUPPORT_THIRD_SHUFFLE
                DDRPhyFreqSel(p, LJ_DDR1600);
                DFSSwitchFreq(p);
                #endif
                //1866P
                DDRPhyFreqSel(p, LJ_DDR1866);
                DFSInitForCalibration(p);
                vDramCalibrationAllChannel(p);
            }
        #else
            #if DFS_SUPPORT_THIRD_SHUFFLE
            // calibration of 1066C is done. save to shuffle2
            DramcSaveToShuffleReg(p, DRAM_DFS_SHUFFLE_2);
            DFSPLLSettingForLP800(p);//LP800 MPDIV
            
            //1270C
            DDRPhyFreqSel(p, LC_DDR1270);
            DFSInitForCalibration(p);
            vDramCalibrationAllChannel(p);
            #endif
            DramcSaveToShuffleReg(p, DRAM_DFS_SHUFFLE_1);

            //1600C
            DDRPhyFreqSel(p, LC_DDR1600);
            DFSInitForCalibration(p);
            vDramCalibrationAllChannel(p);

            #if CPU_RW_TEST_AFTER_K
            mcSHOW_DBG_MSG(("\n[MEM_TEST] 00: After calibration, before DFS\n"));
            vDramCPUReadWriteTestAfterCalibration(p);
            #endif

            //1600P
            DDRPhyFreqSel(p, LJ_DDR1600);
            DFSSwitchFreq(p);
            p->channel = CHANNEL_A;
            Dram_Reset(p);
            p->channel = CHANNEL_B;
            Dram_Reset(p);

            #if CPU_RW_TEST_AFTER_K
            mcSHOW_DBG_MSG(("\n[MEM_TEST] 01: After calibration, before DFS\n"));
            vDramCPUReadWriteTestAfterCalibration(p);
            #endif

            #ifdef SLT
            DDRPhyFreqSel(p, LC_DDR1600);
            DramcDFSDirectJump(p, 0, 1, 0);
            #endif            
        #endif///// end of DFSSetupFlow()/////////

	#ifndef SLT
        DDRPhyFreqSel(p, LC_DDR1270); /* Set DRAM frequency to 1270 */
        DramcDFSDirectJump(p, 1, 1, 1);
        pmic_config_interface(0x608,0x30,0x7F,0); /* Set Vcore to 0.9V */
	#endif   
    }
#endif

#if defined(DDR_INIT_TIME_PROFILING)
    CPU_Cycle=TimeProfileEnd();    
    mcSHOW_ERR_MSG((" Total calibrations take %d ms\n\r", CPU_Cycle));  
#endif

#if !LCPLL_IC_SCAN
    print_DBG_info(p);
    Dump_EMIRegisters(p); 
    #endif

#if 0
    DramcRegDump(p);
#endif

#ifdef RX_EYE_SCAN
    int i;

    mcSHOW_DBG_MSG(("Channel A eye scan.\n\n"));
    vSetPHY2ChannelMapping(p, CHANNEL_A);
    DramcRxEyeScanInit(p);
    for(i=0; i<8; i++)
    {
        #if 1
        DramcRxEyeScanRun(p, 0, i); // boot time
        #else
        DramcRxEyeScanRun(p, 1, i);  // run time
        #endif
    }

    mcSHOW_DBG_MSG(("Channel B eye scan.\n\n"));
    vSetPHY2ChannelMapping(p, CHANNEL_B);
    DramcRxEyeScanInit(p);
    for(i=0; i<8; i++)
    {
        #if 1
        DramcRxEyeScanRun(p, 0, i); // boot time
        #else
        DramcRxEyeScanRun(p, 1, i);  // run time
        #endif
    }
    //while(1);
#endif
  

#ifdef TX_EYE_SCAN
DramcTxEyeScanInit(p);

mcSHOW_DBG_MSG(("Channel A TX eye scan.\n\n"));
vSetPHY2ChannelMapping(p, CHANNEL_A);
{
    #if 1
    DramcTxEyeScanRun(p, 0); //use test engine 2
    #else
    DramcTxEyeScanRun(p, 1);  // use DMA
    #endif
}

mcSHOW_DBG_MSG(("Channel B TX eye scan.\n\n"));
vSetPHY2ChannelMapping(p, CHANNEL_B);
{
    #if 1
    DramcTxEyeScanRun(p, 0); //use test engine 2
    #else
    DramcTxEyeScanRun(p, 1);  // use DMA
    #endif
}
while(1);
#endif

#ifndef FIRST_BRING_UP
    #if CPU_RW_TEST_AFTER_K
    mcSHOW_DBG_MSG(("\n[MEM_TEST] 02: After DFS, before run time config\n"));
    vDramCPUReadWriteTestAfterCalibration(p);
    #endif
    
    mcSHOW_DBG_MSG(("\n\nSettings after calibration ...\n\n"));
    DramcRunTimeConfig(p);
    
    #if CPU_RW_TEST_AFTER_K
    mcSHOW_DBG_MSG(("\n[MEM_TEST] 03: After run time config\n"));
    vDramCPUReadWriteTestAfterCalibration(p);
    #endif
#endif

    return 0;
}

#if  EVEREST_MEM_C_TEST_CODE
//EVEREST TEST CODE
void main(void)
{
    printf("DdrPhySetting_Everest_LP3()+DramcSetting_Everest_LP3()\n");
    Init_DRAM();
}
#endif

