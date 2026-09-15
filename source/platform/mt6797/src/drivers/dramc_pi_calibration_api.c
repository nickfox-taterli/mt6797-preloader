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
 * $RCSfile: pi_calibration_api.c,v $
 * $Revision: #5 $
 *
 *---------------------------------------------------------------------------*/

/** @file pi_calibration_api.c
 *  Basic DRAMC calibration API implementation
 */

//-----------------------------------------------------------------------------
// Include files
//-----------------------------------------------------------------------------
//#include "..\Common\pd_common.h"
#include "dramc_common.h"
#include "x_hal_io.h"
//#include "DramC_reg.h"
//#include "System_reg.h"


#if 1//REG_ACCESS_PORTING_DGB
U8 RegLogEnable=0;
#endif

#if REG_SHUFFLE_REG_CHECK
U8 ShuffleRegCheck =0;
#endif

#if SUPPORT_LP4_DBI
extern U8 u1WriteDBIEnable;
extern U8 u1ReadDBIEnable;
#endif
//#define fcFOR_DQSGDUALP_ENABLE   // Dual-phase DQS clock gating control enabling (new gating, not define as MT6589)
#define CBT_SEL_MEDIAN 0

#define CATRAINING_NUM_LP4      6
//#define MAX_CA_PI_DELAY         31   
#define MAX_CA_PI_DELAY         63  
#define MAX_CS_PI_DELAY         63 
#define MAX_CLK_PI_DELAY        31

#define CBT_VREF_RANGE_SEL              0  //MR12,OP[6]
#define CBT_VREF_RANGE_BEGIN        0
#define CBT_VREF_RANGE_END          50 // binary 110010
#define CBT_VREF_RANGE_STEP         2

#define ENABLE_RX_INPUT_BUF_CALIBRATION 0  // Will not enable in Everest.

#define PASS_RANGE_NA   0x7fff
#define HIGH_FREQ 2130  // for LP4 only.

#define EVEREST_CHANGE_OF_PHY_PBYTE    1 // since Everest, PByte is only move PI delay of DQS/DQS_OEN, not including DQ/DQM/DQ_OEN/DQM_OEN

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
static U8 fgwrlevel_done = 0;

#if PHY_MAPPING_TO_CHANNEL
U8 aru1PhyMap2Channel[2];
#endif

#ifdef TX_EYE_SCAN
U16 aru2TXCaliDelay[CHANNEL_MAX][DQS_NUMBER];
U16 aru2TXCaliDelay_OEN[CHANNEL_MAX][DQS_NUMBER];
#endif

#if SW_CHANGE_FOR_SIMULATION
#define vSetCalibrationResult(x,y,z) 
#define DramcRxWinRDDQC(x) 0xffffffff
#endif

#define CBT_WORKAROUND_O1_SWAP 0 // set default clk and ca value
#define HW_WORKAROUND_RX_EYE_LP3_BYTE3_DELAY_READY 0

static U32 CATrain_CmdDelay[CHANNEL_MAX];
static U32 CATrain_CsDelay[CHANNEL_MAX];
static S32 CATrain_ClkDelay[CHANNEL_MAX];
static S32 wrlevel_dqs_final_delay[CHANNEL_MAX][DQS_NUMBER]; // 3 is channel number
static U16 u2rx_window_sum;

static U8 ucg_num_dlycell_perT = 49; //from 60807

static void dle_factor_handler(DRAMC_CTX_T *p, U8 curr_val, U8 pip_num);

#if PHY_MAPPING_TO_CHANNEL
void vSetPHY2ChannelMapping(DRAMC_CTX_T *p, U8 u1Channel)
{
    p->channel =u1Channel;
    if(p->channel == CHANNEL_A)
    {
        aru1PhyMap2Channel[0] = 1;  // channel A, first PHY is PHYB
        aru1PhyMap2Channel[1] = 2; // channel A, second PHY is PHYC
    }
    else
    {
        aru1PhyMap2Channel[0] = 3;// channel B, first PHY is PHYD
        aru1PhyMap2Channel[1] = 0;// channel B, second PHY is PHYA
    }       
}
#endif


void vSetCalibrationResult(DRAMC_CTX_T *p, U8 ucCalType, U8 ucResult)
{
    p->aru4CalExecuteFlag[p->channel][p->rank][p->shu_type] |= (1<<ucCalType); // ececution done
    if (ucResult == DRAM_OK)  // Calibration OK
    {
        p->aru4CalResultFlag[p->channel][p->rank][p->shu_type] &= (~(1<<ucCalType)); 
    }   
    else  //Calibration fail
    {
        p->aru4CalResultFlag[p->channel][p->rank][p->shu_type] |= (1<<ucCalType);
    }
}

void vGetCalibrationResult_All(DRAMC_CTX_T *p, U8 u1Channel, U8 u1Rank, U8 u1FreqType, U32 *u4CalExecute, U32 *u4CalResult)
{
    *u4CalExecute = p->aru4CalExecuteFlag[u1Channel][u1Rank][u1FreqType];
    *u4CalResult = p->aru4CalResultFlag[u1Channel][u1Rank][u1FreqType];
}

#if 0  //no use now, disable for saving code size.
void vGetCalibrationResult(DRAMC_CTX_T *p, U8 ucCalType, U8 *ucCalExecute, U8 *ucCalResult)
{
    U32 ucCalResult_All, ucCalExecute_All;

    ucCalExecute_All = p->aru4CalExecuteFlag[p->channel][p->rank][p->shu_type];
    ucCalResult_All = p->aru4CalResultFlag[p->channel][p->rank][p->shu_type];

    *ucCalExecute = (U8)((ucCalExecute_All >>ucCalType) & 0x1);
    *ucCalResult =  (U8)((ucCalResult_All >>ucCalType) & 0x1);
}
#endif

const char *szCalibStatusName[DRAM_CALIBRATION_MAX]=
{
	"ZQ Calibration",
	"SW Impedance",
	"CA Training",
	"Write leveling",
	"RX DQS gating",
        "RX DATLAT",
	"RX DQ/DQS(RDDQC)",
	"RX DQ/DQS(Engine)",
	"TX DQ/DQS",	
};

void vPrintCalibrationResult(DRAMC_CTX_T *p)
{
    U8 ucCHIdx, ucRankIdx, ucFreqIdx, ucCalIdx;
    U32 ucCalResult_All, ucCalExecute_All;
    U8 ucCalResult, ucCalExecute;
    
    for(ucCHIdx=0; ucCHIdx<CHANNEL_MAX; ucCHIdx++)
    {
        for(ucRankIdx=0; ucRankIdx<RANK_MAX; ucRankIdx++)
        {
            for(ucFreqIdx=0; ucFreqIdx<DRAM_DFS_SHUFFLE_MAX; ucFreqIdx++)
            {
                ucCalExecute_All = p->aru4CalExecuteFlag[ucCHIdx][ucRankIdx][ucFreqIdx];
                ucCalResult_All = p->aru4CalResultFlag[ucCHIdx][ucRankIdx][ucFreqIdx];
                mcSHOW_DBG_MSG(("[vPrintCalibrationResult] Channel = %d, Rank= %d, Freq.= %d, (ucCalExecute_All 0x%x, ucCalResult_All 0x%x)\n", ucCHIdx, ucRankIdx, ucFreqIdx, ucCalExecute_All, ucCalResult_All));
                
                for(ucCalIdx =0; ucCalIdx<DRAM_CALIBRATION_MAX; ucCalIdx++)
                {
                    ucCalExecute = (U8)((ucCalExecute_All >>ucCalIdx) & 0x1);
                    ucCalResult =  (U8)((ucCalResult_All >>ucCalIdx) & 0x1);
                    #ifdef ETT_PRINT_FORMAT
                    mcSHOW_DBG_MSG(("    %s : Execute= %d, Result= %d (0: Ok, 1:Fail)\n", szCalibStatusName[ucCalIdx], ucCalExecute, ucCalResult));
                    #else
                    mcSHOW_DBG_MSG(("    %20s : Execute= %d, Result= %d (0: Ok, 1:Fail)\n", szCalibStatusName[ucCalIdx], ucCalExecute, ucCalResult));
                    #endif
                }
            }
        }
    }
}


//for LP3 read register in different byte
U32 u4IO32ReadFldAlign_Phy_Byte(U8 ucByteIdx, U32 reg32, U32 fld)
{
       U8 ucGetChannel ;
       
       if(reg32<PHY_A_BASE_VIRTUAL)
       {
           mcSHOW_DBG_MSG(("\n[u4IO32ReadFldAlign_Phy_Byte] wrong address %d\n", reg32));
           return;
       }
       
       ucGetChannel = ((reg32-PHY_A_BASE_VIRTUAL) >> POS_BANK_NUM) & 0xf;
       // remove original channel information
       reg32 &= 0xffff;  
       reg32 += PHY_A_BASE_VIRTUAL;

/*
          if(p->channel == CHANNEL_A)
          {
              aru1PhyMap2Channel[0] = 1;  // channel A, first PHY is PHYB
              aru1PhyMap2Channel[1] = 2; // channel A, second PHY is PHYC
          }
          else
          {
              aru1PhyMap2Channel[0] = 3;// channel B, first PHY is PHYD
              aru1PhyMap2Channel[1] = 0;// channel B, second PHY is PHYA
          }            
*/
          switch(ucByteIdx)
          {
            // CHANNEL A ===
            case 0:            
                return u4IO32ReadFldAlign(reg32+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), fld);
                break;
            case 1:            
                return u4IO32ReadFldAlign(reg32+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), fld);
                break;
            case 2:            
                return u4IO32ReadFldAlign(reg32+(aru1PhyMap2Channel[0]<< POS_BANK_NUM)+0x60, fld);
                break;
            case 3:            
                return u4IO32ReadFldAlign(reg32+(aru1PhyMap2Channel[1]<< POS_BANK_NUM)+0x60, fld);
                break;
            }
}

//for LP3 write register in different byte
void vIO32WriteFldAlign_Phy_Byte(U8 ucByteIdx, U32 reg32, U32 val32, U32 fld)
{
    U8 ucGetChannel ;
    
    if(reg32<PHY_A_BASE_VIRTUAL)
    {
        mcSHOW_DBG_MSG(("\n[vIO32WriteFldAlign_Phy_Byte] wrong address %d\n", reg32));
        return;
    }
    
    ucGetChannel = ((reg32-PHY_A_BASE_VIRTUAL) >> POS_BANK_NUM) & 0xf;
    // remove original channel information
    reg32 &= 0xffff;  
    reg32 += PHY_A_BASE_VIRTUAL;

    /*
              if(p->channel == CHANNEL_A)
              {
                  aru1PhyMap2Channel[0] = 1;  // channel A, first PHY is PHYB
                  aru1PhyMap2Channel[1] = 2; // channel A, second PHY is PHYC
              }
              else
              {
                  aru1PhyMap2Channel[0] = 3;// channel B, first PHY is PHYD
                  aru1PhyMap2Channel[1] = 0;// channel B, second PHY is PHYA
              }            
    */
    switch(ucByteIdx)
    {
        case 0:            
            vIO32WriteFldAlign(reg32+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), val32, fld);
            break;
        case 1:            
            vIO32WriteFldAlign(reg32+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), val32, fld);
            break;
        case 2:            
            vIO32WriteFldAlign(reg32+(aru1PhyMap2Channel[0]<< POS_BANK_NUM)+0x60, val32, fld);
            break;
        case 3:            
            vIO32WriteFldAlign(reg32+(aru1PhyMap2Channel[1]<< POS_BANK_NUM)+0x60, val32, fld);
            break;
    }
}

// for LP3 to control all PHY of single channel
void vIO32WriteFldAlign_Phy_All(U32 reg32, U32 val32, U32 fld)
{
    U8 ucGetChannel ;
    
    if(reg32<PHY_A_BASE_VIRTUAL)
    {
        mcSHOW_DBG_MSG(("\n[vIO32WriteFldAlign_Phy_All] wrong address %d\n", reg32));
        return;
    }
    
    ucGetChannel = ((reg32-PHY_A_BASE_VIRTUAL) >> POS_BANK_NUM) & 0xf;
    // remove original channel information
    reg32 &= 0xffff;  
    reg32 += PHY_A_BASE_VIRTUAL;

    //mcSHOW_DBG_MSG(("\n[vIO32WriteFldAlign_Phy_All] ucGetChannel %d, reg32 0x%x\n", ucGetChannel, reg32));

    /*
              if(p->channel == CHANNEL_A)
              {
                  aru1PhyMap2Channel[0] = 1;  // channel A, first PHY is PHYB
                  aru1PhyMap2Channel[1] = 2; // channel A, second PHY is PHYC
              }
              else
              {
                  aru1PhyMap2Channel[0] = 3;// channel B, first PHY is PHYD
                  aru1PhyMap2Channel[1] = 0;// channel B, second PHY is PHYA
              }            
    */
    vIO32WriteFldAlign(reg32+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), val32, fld);
    vIO32WriteFldAlign(reg32+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), val32, fld);
}

//Reset PHY to prevent glitch when change DQS gating delay or RX DQS input delay
// [Lynx] Everest : cannot reset single channel. All DramC and All Phy have to reset together.
void DramPhyReset(DRAMC_CTX_T *p)
{
    // Everest change reset order : reset DQS before DQ, move PHY reset to final.
    #if ENABLE_LP4_SW
    if(p->dram_type == TYPE_LPDDR4)
    {
        ///TODO: Elbrus , reset all channel & PHY. Whiteney doesn't need.
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_GDDR3CTL1), 1, GDDR3CTL1_RDATRST);// read data counter reset
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_PINMUX), 1, PINMUX_R_DMPHYRST);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_TXDQ3), 0, TXDQ3_RG_RX_ARDQS0_STBEN_RESETB);         
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_RXDQ13), 0, RXDQ13_RG_RX_ARDQS1_STBEN_RESETB);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_TXDQ3), 0, TXDQ3_RG_ARDQ_RESETB_B0);//add for test
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_RXDQ13), 0, RXDQ13_RG_ARDQ_RESETB_B1);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_TXDQ3), 0, TXDQ3_RG_RX_ARDQ_STBEN_RESETB_B0);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_RXDQ13), 0, RXDQ13_RG_RX_ARDQ_STBEN_RESETB_B1);
        
        mcDELAY_US(1);//delay 10ns
        
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_TXDQ3), 1, TXDQ3_RG_RX_ARDQS0_STBEN_RESETB);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_RXDQ13), 1, RXDQ13_RG_RX_ARDQS1_STBEN_RESETB);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_TXDQ3), 1, TXDQ3_RG_ARDQ_RESETB_B0);//add for test
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_RXDQ13), 1, RXDQ13_RG_ARDQ_RESETB_B1);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_TXDQ3), 1, TXDQ3_RG_RX_ARDQ_STBEN_RESETB_B0);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_RXDQ13), 1, RXDQ13_RG_RX_ARDQ_STBEN_RESETB_B1);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_PINMUX), 0, PINMUX_R_DMPHYRST);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_GDDR3CTL1), 0, GDDR3CTL1_RDATRST);
    }
    else //LPDDR3
    #endif
    {
        // Everest change : must reset all dramC and PHY together.
        vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_GDDR3CTL1), 1, GDDR3CTL1_RDATRST);// read data counter reset
        vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DDRPHY_PINMUX), 1, PINMUX_R_DMPHYRST);
        vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DDRPHY_TXDQ3), 0, TXDQ3_RG_RX_ARDQS0_STBEN_RESETB);         
        vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DDRPHY_RXDQ13), 0, RXDQ13_RG_RX_ARDQS1_STBEN_RESETB);
        vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DDRPHY_TXDQ3), 0, TXDQ3_RG_ARDQ_RESETB_B0);//add for test
        vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DDRPHY_RXDQ13), 0, RXDQ13_RG_ARDQ_RESETB_B1);
        vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DDRPHY_TXDQ3), 0, TXDQ3_RG_RX_ARDQ_STBEN_RESETB_B0);
        vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DDRPHY_RXDQ13), 0, RXDQ13_RG_RX_ARDQ_STBEN_RESETB_B1);

        mcDELAY_US(1);//delay 10ns

        vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DDRPHY_TXDQ3), 1, TXDQ3_RG_RX_ARDQS0_STBEN_RESETB);
        vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DDRPHY_RXDQ13), 1, RXDQ13_RG_RX_ARDQS1_STBEN_RESETB);
        vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DDRPHY_TXDQ3), 1, TXDQ3_RG_ARDQ_RESETB_B0);  //add for test
        vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DDRPHY_RXDQ13), 1, RXDQ13_RG_ARDQ_RESETB_B1);
        vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DDRPHY_TXDQ3), 1, TXDQ3_RG_RX_ARDQ_STBEN_RESETB_B0);
        vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DDRPHY_RXDQ13), 1, RXDQ13_RG_RX_ARDQ_STBEN_RESETB_B1);
        vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DDRPHY_PINMUX), 0, PINMUX_R_DMPHYRST);
        vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_GDDR3CTL1), 0, GDDR3CTL1_RDATRST);
    }
}


void DramEyeStbenReset(DRAMC_CTX_T *p)
{
    //vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_GDDR3CTL1), 1, GDDR3CTL1_RDATRST);// read data counter reset

    #if ENABLE_LP4_SW
    if(p->dram_type == TYPE_LPDDR4)
    {
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE2), 0, EYE2_RG_RX_ARDQ_EYE_STBEN_RESETB_B0);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), 0, EYEB1_2_RG_RX_ARDQ_EYE_STBEN_RESETB_B1);
                
        mcDELAY_US(1);//delay 10ns

        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE2), 1, EYE2_RG_RX_ARDQ_EYE_STBEN_RESETB_B0);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), 1, EYEB1_2_RG_RX_ARDQ_EYE_STBEN_RESETB_B1);
    }
    else //LPDDR3
    #endif
    {
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE2), 0, EYE2_RG_RX_ARDQ_EYE_STBEN_RESETB_B0);
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), 0, EYEB1_2_RG_RX_ARDQ_EYE_STBEN_RESETB_B1);

        mcDELAY_US(1);//delay 10ns
        
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE2), 1, EYE2_RG_RX_ARDQ_EYE_STBEN_RESETB_B0);
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), 1, EYEB1_2_RG_RX_ARDQ_EYE_STBEN_RESETB_B1);
    }
}


//PHY Clock Gated and Reset
static void DramPhyCGReset(DRAMC_CTX_T *p, BOOL bOnOff)
{
#if 0
RESETB:
RG_RX_ARCLK_STBEN_RESETB
RG_RX_ARDQS0_STBEN_RESETB
RG_RX_ARDQS1_STBEN_RESETB
RG_RX_BRCLK_STBEN_RESETB
RG_RX_BRDQS0_STBEN_RESETB
RG_RX_BRDQS1_STBEN_RESETB

CG:
RG_ARPI_CG_DQSIEN_CA
RG_ARPI_CG_DQSIEN_B0
RG_ARPI_CG_DQSIEN_B1
RG_BRPI_CG_DQSIEN_CA
RG_BRPI_CG_DQSIEN_B0
RG_BRPI_CG_DQSIEN_B1
#endif
        
     // Reset ARDQS when clock off and on or gating PI >1
    #if ENABLE_LP4_SW
    if(p->dram_type == TYPE_LPDDR4)
    {
        ////vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_TXDQ3), !bOnOff, TXDQ3_RG_RX_ARDQS0_STBEN_RESETB);
        ////vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_RXDQ13), !bOnOff, RXDQ13_RG_RX_ARDQS1_STBEN_RESETB);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_PLL9), bOnOff, PLL9_RG_ARPI_CG_DQSIEN_B0);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_PLL9), bOnOff, PLL9_RG_ARPI_CG_DQSIEN_B1);
    } 
    else //LPDDR3
    #endif
    {
        ////vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_TXDQ3), !bOnOff, TXDQ3_RG_RX_ARDQS0_STBEN_RESETB);
        ////vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ13), !bOnOff, RXDQ13_RG_RX_ARDQS1_STBEN_RESETB);
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_PLL9), bOnOff, PLL9_RG_ARPI_CG_DQSIEN_B0);
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_PLL9), bOnOff, PLL9_RG_ARPI_CG_DQSIEN_B1);
    }
}


DRAM_STATUS_T DramcRankSwap(DRAMC_CTX_T *p, U8 u1Rank)
{    
    U8 u1Multi;

    p->rank = u1Rank;

    u1Multi = (p->support_rank_num >1) ? 1 : 0;
    
    mcSHOW_DBG_MSG(("[DramcRankSwap] Rank number %d, (u1Multi %d), Rank %d\n", p->support_rank_num, u1Multi, u1Rank));
    //mcFPRINTF((fp_A60501, "[DramcRankSwap] Rank number %d, (u1Multi %d), Rank %d\n", p->support_rank_num, u1Multi, u1Rank));

    //Set to non-zero for multi-rank
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_RKCFG), u1Multi, RKCFG_RKMODE);
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_RKCFG), u1Rank, RKCFG_RKSWAP);

    return DRAM_OK;
}


#if ENABLE_RX_INPUT_BUF_CALIBRATION
//-------------------------------------------------------------------------
/** DramcSwImpedanceCal
 *  start TX OCD impedance calibration.
 *  @param p                Pointer of context created by DramcCtxCreate.
 *  @param  apply           (U8): 0 don't apply the register we set  1 apply the register we set ,default don't apply.
 *  @retval status          (DRAM_STATUS_T): DRAM_OK or DRAM_FAIL 
 */
//-------------------------------------------------------------------------
static U8 RXInputBuf_DelayExchange(S8 iOfst)
{
    U8 u1Value;
    
    if(iOfst <0)
    {
        u1Value = 0x8 | (-iOfst);
    }
    else
    {
        u1Value = iOfst;
    }

    return u1Value;
}

static DRAM_STATUS_T RXInputBufCal_SingleByte(DRAMC_CTX_T *p, U8 u1ByteIdx)
{
    U32 u4BaklReg_DDRPHY_EYE3, u4BaklReg_DDRPHY_EYE2, u4BaklReg_DRAMC_MCKDLY, u4BaklReg_DDRPHY_TXDQ3;

    U32 u4AddrOfst;
    S8 iOffset, iDQFlagChange[DQS_BIT_NUMBER], iDQMFlagChange;
    U32 u4Value, u4RestltDQ, u4RestltDQM;
    U8 u1BitIdx, u1FinishCount, u1DQFinalFlagChange[DQS_BIT_NUMBER], u1DQMFinalFlagChange;

    mcSHOW_DBG_MSG(("\n[DramcRXInputBufferOffsetCal] Start byte %d\n", u1ByteIdx));
    mcFPRINTF((fp_A60501, "\n[DramcRXInputBufferOffsetCal] Start byte %d\n", u1ByteIdx));

    if(u1ByteIdx >3)  //Byte 0~3
    {
        mcSHOW_DBG_MSG(("Wrong byte index %d, should be 0~3\n", u1ByteIdx));
        mcFPRINTF((fp_A60501, "Wrong byte index %d, should be 0~3\n", u1ByteIdx));
        return DRAM_FAIL;
    }

     // for LP4 only
    if(u1ByteIdx==0) //byte 0
    {
        u4AddrOfst =0;
    }
    else //Byte 1
    {
        u4AddrOfst =0x60;
    }

    //Back up dramC register
    u4BaklReg_DRAMC_MCKDLY = u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_MCKDLY));

    if(p->dram_type == TYPE_LPDDR3)
    {       
        u4BaklReg_DDRPHY_EYE3    = u4IO32ReadFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_EYE3),   PHY_FLD_FULL);
        u4BaklReg_DDRPHY_EYE2    = u4IO32ReadFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_EYE2),   PHY_FLD_FULL);
        u4BaklReg_DDRPHY_TXDQ3 = u4IO32ReadFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_TXDQ3), PHY_FLD_FULL);
      
        //Enable BIAS (RG_RX_*DQ_BIAS_EN=1)
        vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_EYE3), 1, EYE3_RG_RX_ARDQ_BIAS_EN_B0);

        //Enable VREF, (RG_RX_*DQ_VREF_EN_* =1)
        vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_EYE2), 1, EYE2_RG_RX_ARDQ_VREF_EN_B0);

        //Select VREF, For LPDDR3, set RG_RX_*DQ_DDR4_SEL_* =0, RG_RX_*DQ_DDR3_SEL_* =1
        vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_EYE3), 1, EYE3_RG_RX_ARDQ_DDR3_SEL_B0);
        vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_EYE3), 0, EYE3_RG_RX_ARDQ_DDR4_SEL_B0);

        // use default Vref, no register backup
        //Set Vref voltage: LP3 without termination  SEL[4:0] = 01110 (0xe) (refer to Vref default table)
        //vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE2), 0xe, EYE2_RG_RX_ARDQ_VREF_SEL_B0);
        //vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), 0xe, EYEB1_2_RG_RX_ARDQ_VREF_SEL_B1);

    }
    #if ENABLE_LP4_SW
    else //LPDDR4 
    {
        //Backup setting registers
        u4BaklReg_DDRPHY_EYE3 = u4IO32Read4B(DRAMC_REG_ADDR(DDRPHY_EYE3+u4AddrOfst));
        u4BaklReg_DDRPHY_EYE2 = u4IO32Read4B(DRAMC_REG_ADDR(DDRPHY_EYE2+u4AddrOfst));
        u4BaklReg_DDRPHY_TXDQ3 = u4IO32Read4B(DRAMC_REG_ADDR(DDRPHY_TXDQ3+u4AddrOfst));

        //Enable BIAS (RG_RX_*DQ_BIAS_EN=1)
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE3+u4AddrOfst), 1, EYE3_RG_RX_ARDQ_BIAS_EN_B0);

        //Enable VREF, (RG_RX_*DQ_VREF_EN_* =1)
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE2+u4AddrOfst), 1, EYE2_RG_RX_ARDQ_VREF_EN_B0);

        //Select VREF, for LPDDR4 set RG_RX_*DQ_DDR4_SEL_* =1, RG_RX_*DQ_DDR3_SEL_* =0
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE3+u4AddrOfst), 0, EYE3_RG_RX_ARDQ_DDR3_SEL_B0);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE3+u4AddrOfst), 1, EYE3_RG_RX_ARDQ_DDR4_SEL_B0);

        //Set Vref voltage: LP4 with termination  SEL[4:0] = 01110 (0xe) (refer to Vref default table)
        ///TODO: Need to set according to ODT on/off.
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE2+u4AddrOfst), 0xe, EYE2_RG_RX_ARDQ_VREF_SEL_B0);
    }
    #endif

    // Wait 1us.
    mcDELAY_US(1);

    // Enable RX input buffer (RG_RX_*DQ_IN_BUFF_EN_* =1, DA_RX_*DQ_IN_GATE_EN_* =1)
    //Enable RX input buffer offset calibration (RG_RX_*DQ_OFFC_EN_*=1)
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_MCKDLY), 0xf, MCKDLY_FIXDQIEN); // Keep DQ input always ON.

    #if ENABLE_LP4_SW
    if(p->dram_type == TYPE_LPDDR4)
    {
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_TXDQ3+u4AddrOfst), 1, TXDQ3_RG_RX_ARDQ_IN_BUFF_EN_B0);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_TXDQ3+u4AddrOfst), 1, TXDQ3_RG_RX_ARDQ_OFFC_EN_B0);
    }
    else //LPDDDR3
    #endif
    {
        vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_TXDQ3), 1, TXDQ3_RG_RX_ARDQ_IN_BUFF_EN_B0);
        vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_TXDQ3), 1, TXDQ3_RG_RX_ARDQ_OFFC_EN_B0);
    }
    
    // SW parameter initialization
    u1FinishCount =0;
    iDQMFlagChange = 0x7f;
    
    for(u1BitIdx=0; u1BitIdx< DQS_BIT_NUMBER; u1BitIdx++)
    {
        iDQFlagChange[u1BitIdx] = 0x7f; //initial as invalid
    }

    //Sweep RX offset calibration code (RG_RX_*DQ*_OFFC<3:0>), the MSB is sign bit, sweep the code from -7(1111) to +7(0111)
    for(iOffset=-7; iOffset<7; iOffset++)
    {
        u4Value = RXInputBuf_DelayExchange(iOffset);

        #ifdef ETT_PRINT_FORMAT
        mcSHOW_DBG_MSG(("iOffset= %d, u4Value=%d,", iOffset, u4Value));
        #else
        mcSHOW_DBG_MSG(("iOffset= %2d, u4Value=%2d,", iOffset, u4Value));
        #endif
        mcFPRINTF((fp_A60501, "iOffset= %2d, u4Value=%2d,", iOffset, u4Value));

        #if ENABLE_LP4_SW
        if(p->dram_type == TYPE_LPDDR4)
        {
             //u4AddrOfst==0, Delay of DQ0.          
             //u4AddrOfst==0x60, Delay of DQ8
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_RXDQ0OFFC+u4AddrOfst), u4Value, RXDQ0OFFC_RG_RX_ARDQ0_OFFC);

             //u4AddrOfst==0, Delay of DQ1~DQ7, DQM0.     
             //u4AddrOfst==0x60, Delay of DQ9~DQ15, DQM1
            u4Value = u4Value |(u4Value<<4) |(u4Value<<8) |(u4Value<<12) |(u4Value<<16) |(u4Value<<20) |(u4Value<<24) |(u4Value<<28);
            vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_RXDQ1OFFC+u4AddrOfst), u4Value); 

            //For each code sweep, wait 0.1us to check the flag.
            mcDELAY_US(1);
            
            //Check offset flag of DQ (RGS_*DQ*_OFFSET_FLAG_*), the value will be from 1(-7) to 0(+7). Record the value when the flag becomes "0".
            //Flag bit0 is for DQ0,  Flag bit15 for DQ15
            //#ifdef A60501_BUILD_ERROR_TODO
            u4RestltDQ = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DDRPHY_PHY_RO_1), PHY_RO_1_RGS_ARDQ_OFFSET_FLAG);   
			//#endif

			//A60817 needs to swap CH_A byte0/byte1
            //if(((p->channel == CHANNEL_A) &&(u1ByteIdx!=0)) || \
            //    ((p->channel == CHANNEL_B) &&(u1ByteIdx==0)))
			if(u1ByteIdx==0)
            {
                u4RestltDQ = u4RestltDQ & 0xff;// Get result flag of DQ0~DQ7
            }
            else
            {
                u4RestltDQ = u4RestltDQ >> 8;// Get refult flag of DQ8~DQ15
            }
        }
        else //LPDDR3
        #endif// #if ENABLE_LP4_SW
        {
            vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_RXDQ0OFFC), u4Value, RXDQ0OFFC_RG_RX_ARDQ0_OFFC);
             u4Value = u4Value |(u4Value<<4) |(u4Value<<8) |(u4Value<<12) |(u4Value<<16) |(u4Value<<20) |(u4Value<<24) |(u4Value<<28);
            vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_RXDQ1OFFC), u4Value, PHY_FLD_FULL); 

            //For each code sweep, wait 0.1us to check the flag.
            mcDELAY_US(1);
            
            //Check offset flag of DQ (RGS_*DQ*_OFFSET_FLAG_*), the value will be from 1(-7) to 0(+7). Record the value when the flag becomes "0".
            //Flag bit0 is for DQ0,  Flag bit15 for DQ15
            switch(u1ByteIdx)
            {	
                case 0://byte 0
                    u4RestltDQ = u4IO32ReadFldAlign(DDRPHY_PHY_RO_1+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), PHY_RO_1_RGS_ARDQ_OFFSET_FLAG) & 0xff;
                    break;
                case 1://byte 1  ==> channel 1, Byte0
                    u4RestltDQ = u4IO32ReadFldAlign(DDRPHY_PHY_RO_1+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), PHY_RO_1_RGS_ARDQ_OFFSET_FLAG) >> 8;
                    break;
                case 2://byte 2  ==> channel 2, Byte1
                    u4RestltDQ = u4IO32ReadFldAlign(DDRPHY_PHY_RO_1+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), PHY_RO_1_RGS_ARDQ_OFFSET_FLAG) & 0xff;
                    break;
                case 3://byte 3  ==> channel 0, Byte1 (channel 0, byte swap, need to read ch0 byte0)
                    u4RestltDQ = u4IO32ReadFldAlign(DDRPHY_PHY_RO_1+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), PHY_RO_1_RGS_ARDQ_OFFSET_FLAG) >> 8;
                    break;
                default:
                    //won't happen
                    break;
            }
        }
        mcSHOW_DBG_MSG(("u4RestltDQ= 0x%x,", u4RestltDQ));
        mcFPRINTF((fp_A60501, "u4RestltDQ= 0x%x,", u4RestltDQ));


        for(u1BitIdx= 0; u1BitIdx <DQS_BIT_NUMBER; u1BitIdx++)
        {
            if(iDQFlagChange[u1BitIdx] == 0x7f) //invalid
            {
                u4Value = (u4RestltDQ >> u1BitIdx) & 0x1;
                
                if(u4Value ==0) // 1 -> 0
                {
                    iDQFlagChange[u1BitIdx] = iOffset;
                    u1FinishCount ++;
                }
            }
        }

        //Check offset flag of DQM (RGS_*DQ*_OFFSET_FLAG_*), the value will be from 1(-7) to 0(+7). Record the value when the flag becomes "0".
        #if ENABLE_LP4_SW
        if(p->dram_type == TYPE_LPDDR4)
        {
        	//#ifdef A60501_BUILD_ERROR_TODO
			//A60817 needs to swap CH_A byte0/byte1
            //if(((p->channel == CHANNEL_A) &&(u1ByteIdx!=0)) || \
            //    ((p->channel == CHANNEL_B) &&(u1ByteIdx==0)))
			if(u1ByteIdx==0)
            {
                u4RestltDQM = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DDRPHY_PHY_RO_1), PHY_RO_1_RGS_ARDQM0_OFFSET_FLAG); 
            }
            else
            {
                u4RestltDQM = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DDRPHY_PHY_RO_1), PHY_RO_1_RGS_ARDQM1_OFFSET_FLAG); 
            }
            //#endif
        }
        else  //LPDDR3, 
        #endif
        {
        	//#ifdef A60501_BUILD_ERROR_TODO
            switch(u1ByteIdx)
            {                
                case 0://byte 0
                    u4RestltDQ = u4IO32ReadFldAlign(DDRPHY_PHY_RO_1+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), PHY_RO_1_RGS_ARDQM0_OFFSET_FLAG);
                    break;
                case 1://byte 1 
                    u4RestltDQ = u4IO32ReadFldAlign(DDRPHY_PHY_RO_1+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), PHY_RO_1_RGS_ARDQM1_OFFSET_FLAG);
                    break;
                case 2://byte 2
                    u4RestltDQ = u4IO32ReadFldAlign(DDRPHY_PHY_RO_1+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), PHY_RO_1_RGS_ARDQM0_OFFSET_FLAG);
                    break;
                case 3://byte 3  
                    u4RestltDQ = u4IO32ReadFldAlign(DDRPHY_PHY_RO_1+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), PHY_RO_1_RGS_ARDQM1_OFFSET_FLAG);
                    break;
                default:
                    //won't happen
                    break;
            }
        }

        mcSHOW_DBG_MSG(("u4RestltDQM= %d\n", u4RestltDQM));
        mcFPRINTF((fp_A60501, "u4RestltDQM= %d\n", u4RestltDQM));
        
        if(iDQMFlagChange== 0x7f) //invalid
        {
            if(u4RestltDQM==0)// 1 -> 0
            {
                iDQMFlagChange= iOffset;
                u1FinishCount++;
            }
        }

        if(u1FinishCount==9) //DQ8 bits, DQM 1bit, total 9 bits.
        {
            break; //all bits done, early break
        }
    }

    mcSHOW_DBG_MSG(("Result DQ [0]%d [1]%d [2]%d [3]%d [4]%d [5]%d [6]%d [7]%d\n", \
                                        iDQFlagChange[0], iDQFlagChange[1], iDQFlagChange[2], iDQFlagChange[3], \
                                        iDQFlagChange[4], iDQFlagChange[5], iDQFlagChange[6],iDQFlagChange[7]));
    mcFPRINTF((fp_A60501,"Result DQ [0]%d [1]%d [2]%d [3]%d [4]%d [5]%d [6]%d [7]%d\n", \
                                        iDQFlagChange[0], iDQFlagChange[1], iDQFlagChange[2], iDQFlagChange[3], \
                                        iDQFlagChange[4], iDQFlagChange[5], iDQFlagChange[6],iDQFlagChange[7]));

    mcSHOW_DBG_MSG(("Result DQM %d\n", iDQMFlagChange));
    mcFPRINTF((fp_A60501, "Result DQM %d\n", iDQMFlagChange));

    //Restore setting registers
    vIO32Write4B(DRAMC_REG_ADDR(DRAMC_REG_MCKDLY), u4BaklReg_DRAMC_MCKDLY);

    #if ENABLE_LP4_SW
    if(p->dram_type == TYPE_LPDDR4)
    {
        vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_EYE3+u4AddrOfst), u4BaklReg_DDRPHY_EYE3);
        vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_EYE2+u4AddrOfst), u4BaklReg_DDRPHY_EYE2);
        vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_TXDQ3+u4AddrOfst), u4BaklReg_DDRPHY_TXDQ3);
    }
    else
    #endif
    {
        vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_EYE3), u4BaklReg_DDRPHY_EYE3, PHY_FLD_FULL);
        vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_EYE2), u4BaklReg_DDRPHY_EYE2, PHY_FLD_FULL);
        vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_TXDQ3), u4BaklReg_DDRPHY_TXDQ3, PHY_FLD_FULL);
    }

    // Change the offset value according to register format 
    for(u1BitIdx= 0; u1BitIdx <DQS_BIT_NUMBER; u1BitIdx++)
    {
        u1DQFinalFlagChange[u1BitIdx] = RXInputBuf_DelayExchange(iDQFlagChange[u1BitIdx]);
    }
    u1DQMFinalFlagChange= RXInputBuf_DelayExchange(iDQMFlagChange);

    #if ENABLE_LP4_SW
    if(p->dram_type == TYPE_LPDDR4)
    {
        //Apply the code recorded.
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_RXDQ0OFFC+u4AddrOfst), u1DQFinalFlagChange[0], RXDQ0OFFC_RG_RX_ARDQ0_OFFC);    //Delay of DQ0 or DQ8

        u4Value = u1DQMFinalFlagChange |(u1DQFinalFlagChange[7]<<4) |(u1DQFinalFlagChange[6]<<8) |(u1DQFinalFlagChange[5]<<12) | \
                        (u1DQFinalFlagChange[4]<<16) |(u1DQFinalFlagChange[3]<<20) |(u1DQFinalFlagChange[2]<<24) |(u1DQFinalFlagChange[1]<<28);      
        vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_RXDQ1OFFC+u4AddrOfst), u4Value);   //Delay of DQ1~DQ7, DQM0  or    Delay of DQ9~DQ15, DQM1

        // Disable RX input buffer offset calibration (RG_RX_*DQ_OFFC_EN_*=0)
        //vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_TXDQ3+u4AddrOfst), 0, TXDQ3_RG_RX_ARDQ_OFFC_EN_B0);
    }
    else //LPDDR3
    #endif
    {
        //Apply the code recorded.
        vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_RXDQ0OFFC), u1DQFinalFlagChange[0], RXDQ0OFFC_RG_RX_ARDQ0_OFFC);    //Delay of DQ0 or DQ8

        u4Value = u1DQMFinalFlagChange |(u1DQFinalFlagChange[7]<<4) |(u1DQFinalFlagChange[6]<<8) |(u1DQFinalFlagChange[5]<<12) | \
                        (u1DQFinalFlagChange[4]<<16) |(u1DQFinalFlagChange[3]<<20) |(u1DQFinalFlagChange[2]<<24) |(u1DQFinalFlagChange[1]<<28);      
        vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_RXDQ1OFFC), u4Value, PHY_FLD_FULL);   //Delay of DQ1~DQ7, DQM0  or    Delay of DQ9~DQ15, DQM1

        // Disable RX input buffer offset calibration (RG_RX_*DQ_OFFC_EN_*=0)
        //vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_TXDQ3), 0, TXDQ3_RG_RX_ARDQ_OFFC_EN_B0);    //Delay of DQ0 or DQ8
    }

    #if 0
    mcSHOW_DBG_MSG(("Register DQ [0]%d [1]%d [2]%d [3]%d [4]%d [5]%d [6]%d [7]%d\n", \
                                        u1DQFinalFlagChange[0], u1DQFinalFlagChange[1], u1DQFinalFlagChange[2], u1DQFinalFlagChange[3], \
                                        u1DQFinalFlagChange[4], u1DQFinalFlagChange[5], u1DQFinalFlagChange[6],u1DQFinalFlagChange[7]));
    mcFPRINTF((fp_A60501,"Register DQ [0]%d [1]%d [2]%d [3]%d [4]%d [5]%d [6]%d [7]%d\n", \
                                        u1DQFinalFlagChange[0], u1DQFinalFlagChange[1], u1DQFinalFlagChange[2], u1DQFinalFlagChange[3], \
                                        u1DQFinalFlagChange[4], u1DQFinalFlagChange[5], u1DQFinalFlagChange[6],u1DQFinalFlagChange[7]));
    mcSHOW_DBG_MSG(("Register DQM %d\n", u1DQMFinalFlagChange));
    mcFPRINTF((fp_A60501, "Register DQM %d\n", u1DQMFinalFlagChange));
    #endif

    return DRAM_OK;
}



DRAM_STATUS_T DramcRXInputBufferOffsetCal(DRAMC_CTX_T *p)
{
    U8 ucByteIndx;

    for(ucByteIndx =0; ucByteIndx < p->data_width /DQS_BIT_NUMBER; ucByteIndx++)
    {
        RXInputBufCal_SingleByte(p, ucByteIndx);
    }

    mcSHOW_DBG_MSG(("[DramcRXInputBufferOffsetCal] ====Done====\n"));
    mcFPRINTF((fp_A60501, "[DramcRXInputBufferOffsetCal] ====Done====\n"));    

    return DRAM_OK;
}

#endif

//-------------------------------------------------------------------------
/** DramcSwImpedanceCal
 *  start TX OCD impedance calibration.
 *  @param p                Pointer of context created by DramcCtxCreate.
 *  @param  apply           (U8): 0 don't apply the register we set  1 apply the register we set ,default don't apply.
 *  @retval status          (DRAM_STATUS_T): DRAM_OK or DRAM_FAIL 
 */
//-------------------------------------------------------------------------
#define HYNIX_IMPX_ADJUST 0
#if HYNIX_IMPX_ADJUST
U32 ImpedanceAdjustment_Hynix(U32 u4OriValue, U8 u1Div)
{
    U32 u4RODT;
    U32 u4AdjustValue;

    if(u1Div >0)
    {
        u4RODT = 240/u1Div;
        u4AdjustValue = 60 *(u4OriValue+2)/u4RODT -2;
    }
    else
    {
        u4RODT =0;
        u4AdjustValue = u4OriValue;
    }

    mcSHOW_DBG_MSG(("ODTN Change by Tool, 240 div %d =%d, After adjustment ODTN=%d\n\n", u1Div, u4RODT, u4AdjustValue));

    return u4AdjustValue;
}
#endif

DRAM_STATUS_T DramcSwImpedanceCal(DRAMC_CTX_T *p, U8 u1Para)
{
    U32 u4ImpxDrv, u4ImpCalResult;
    U32 u4DRVP_Result =0xff,u4ODTN_Result =0xff, u4DRVN_Result =0xff;
    U32 u4BaklReg_DDRPHY_PLL20, u4BaklReg_DDRPHY_IMPA1;
    U8 u1ByteIdx;

    mcSHOW_DBG_MSG2(("[DramcSwImpedanceCal] Start \n"));
    mcFPRINTF((fp_A60501, "[DramcSwImpedanceCal] Start\n"));

    // DDRPHY_PLL15: only need to set reg in channelA.   Same reg in Channel B is for other purpose. 
    vIO32WriteFldMulti((DDRPHY_PLL15), P_Fld(0, PLL15_RG_RPHYPLL_TSTLVROD_EN) | P_Fld(0, PLL15_RG_RPHYPLL_TST_EN) | \
                            P_Fld(0, PLL15_RG_RPHYPLL_TSTCK_EN) | P_Fld(0, PLL15_RG_RPHYPLL_TSTFM_EN) | \
                            P_Fld(0, PLL15_RG_RPHYPLL_TSTOD_EN) | P_Fld(0, PLL15_RG_RPHYPLL_TSTOP_EN));
    vIO32Write4B_All((DDRPHY_PHY_SPM_CTL3), 0);

	//HW calibration setting will affect sw calibration. so need to set IMPCAL_IMP_CALI_ENP=1
	vIO32WriteFldAlign((DRAMC_REG_IMPCAL), 1, IMPCAL_IMP_CALI_ENP);
	
    //Register backup 
    u4BaklReg_DDRPHY_PLL20 = u4IO32Read4B((DDRPHY_PLL20));
    u4BaklReg_DDRPHY_IMPA1 = u4IO32Read4B((DDRPHY_IMPA1));

    #if ENABLE_LP4_SW
    if(p->dram_type == TYPE_LPDDR4)
    {
    	//RG_IMPCAL_VREF_SEL=6'h33
    	//RG_IMPCAL_LP3_EN=0, RG_IMPCAL_LP4_EN=1
        vIO32WriteFldMulti( (DDRPHY_IMPA1), P_Fld(1, IMPA1_RG_RIMP_BIAS_EN) | \
        					P_Fld(0, IMPA1_RG_RIMP_PRE_EN) | P_Fld(1, IMPA1_RG_RIMP_VREF_EN));
    	vIO32WriteFldMulti( (DDRPHY_PLL20), P_Fld(1, PLL20_RG_IMPA_EN) | \
    						P_Fld(0, PLL20_RG_IMPA_DDR3_SEL) | P_Fld(1, PLL20_RG_IMPA_DDR4_SEL) | \
    						P_Fld(0x33, PLL20_RG_IMPA_VREF_SEL));
    }
    else //LPDDR3
    #endif
    {
    	//RG_IMPCAL_VREF_SEL=6'h0f
    	//RG_IMPCAL_LP3_EN=0, RG_IMPCAL_LP4_EN=1
        //RG_IMPCAL_ODT_EN=0
        vIO32WriteFldMulti((DDRPHY_IMPA1), P_Fld(0, IMPA1_RG_RIMP_ODT_EN)| P_Fld(1, IMPA1_RG_RIMP_BIAS_EN) | \
        					P_Fld(0, IMPA1_RG_RIMP_PRE_EN) | P_Fld(1, IMPA1_RG_RIMP_VREF_EN));
    	vIO32WriteFldMulti((DDRPHY_PLL20), P_Fld(1, PLL20_RG_IMPA_EN) | \
    						P_Fld(1, PLL20_RG_IMPA_DDR3_SEL) | P_Fld(0, PLL20_RG_IMPA_DDR4_SEL) | \
    						P_Fld(0x0f, PLL20_RG_IMPA_VREF_SEL));
        #ifdef ETT_PRINT_FORMAT        
    	mcSHOW_DBG_MSG3(("0x%X=0x%X\n", DRAMC_REG_ADDR(DDRPHY_IMPA1), u4IO32Read4B(DRAMC_REG_ADDR(DDRPHY_IMPA1))));
    	mcSHOW_DBG_MSG3(("0x%X=0x%X\n", DRAMC_REG_ADDR(DDRPHY_PLL20), u4IO32Read4B(DRAMC_REG_ADDR(DDRPHY_PLL20))));
        #else
    	mcSHOW_DBG_MSG3(("0x%8x=0x%8x\n", DRAMC_REG_ADDR(DDRPHY_IMPA1), u4IO32Read4B(DRAMC_REG_ADDR(DDRPHY_IMPA1))));
    	mcSHOW_DBG_MSG3(("0x%8x=0x%8x\n", DRAMC_REG_ADDR(DDRPHY_PLL20), u4IO32Read4B(DRAMC_REG_ADDR(DDRPHY_PLL20))));
        #endif
    }
	mcDELAY_US(1);

    // K pull up 
    mcSHOW_DBG_MSG2(("======= K DRVP=====================\n"));
    mcFPRINTF((fp_A60501, "======= K DRVP=====================\n"));    
    //PUCMP_EN=1
    //ODT_EN=0
	vIO32WriteFldAlign((DDRPHY_PLL20), 1, PLL20_RG_IMPA_OCD_PUCMP_EN);
    #if ENABLE_LP4_SW
	if(p->dram_type == TYPE_LPDDR4)
	{
    	vIO32WriteFldAlign((DDRPHY_IMPA1), 0, IMPA1_RG_RIMP_ODT_EN);
	}
    #endif
	//DRVP=0
    //DRV05=1 
    vIO32WriteFldMulti((DDRPHY_IMPA1), P_Fld(0, IMPA1_RG_IMPA_ODTN_SW)|P_Fld(0, IMPA1_RG_IMPA_ODTP_SW)|P_Fld(1, IMPA1_RG_RIMP_REV));
    
    //If RGS_TX_OCD_IMPCALOUTX=0
    //RG_IMPX_DRVP++;
    //Else save and keep RG_IMPX_DRVP value, and assign to DRVP
    for(u4ImpxDrv=0; u4ImpxDrv<32; u4ImpxDrv++)
    {
        if(u4ImpxDrv==16) //0~15, 29~31
            u4ImpxDrv = 29;
        vIO32WriteFldAlign((DDRPHY_IMPA1), u4ImpxDrv, IMPA1_RG_IMPA_ODTP_SW);
        mcDELAY_US(1);
        u4ImpCalResult = u4IO32ReadFldAlign((DDRPHY_PHY_RO_2), PHY_RO_2_RGS_RIMPCALOUT);
        mcSHOW_DBG_MSG3(("1. OCD DRVP=%d CALOUT=%d\n", u4ImpxDrv, u4ImpCalResult));
        mcFPRINTF((fp_A60501, "1. OCD DRVP=%d CALOUT=%d\n", u4ImpxDrv, u4ImpCalResult));   
        
        if((u4ImpCalResult ==1) && (u4DRVP_Result == 0xff))//first found
        {
            u4DRVP_Result = u4ImpxDrv;
            mcSHOW_DBG_MSG(("\n1. OCD DRVP calibration OK! DRVP=%d\n\n", u4DRVP_Result));
            mcFPRINTF((fp_A60501, "\n1. OCD DRVP calibration OK! DRVP=%d\n\n", u4DRVP_Result));   
            break;
        }
    }
	
    #if 0//ENABLE_LP4_SW
    if(p->dram_type == TYPE_LPDDR4)
    {
        mcSHOW_DBG_MSG(("======= K ODT =====================\n"));
        mcFPRINTF((fp_A60501, "======= K ODT=====================\n"));    

        //RG_IMPX_OCD_PUCMP_EN=0
        //RG_IMPX_DDR4_ODT_EN=1
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_PLL20), 0, PLL20_RG_IMPA_OCD_PUCMP_EN);
        //#if A60501_BUILD_ERROR_TODO
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_PLL20), 1, PLL20_RG_IMPA_DDR4_ODT_EN);
		//#endif

        //If RGS_TX_OCD_IMPCALOUTX=1
        //RG_IMPX_DRVN++;
        //Else save RG_IMPX_DRVN value and assign to ODTN
        for(u4ImpxDrv=0; u4ImpxDrv<32 ; u4ImpxDrv++)
        {
            if(u4ImpxDrv==16) //0~15, 28~31
                u4ImpxDrv = 28;

			//#if A60501_BUILD_ERROR_TODO
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_IMPA1), u4ImpxDrv, IMPA1_RG_IMPA_DRVN);
            mcDELAY_US(1);
            u4ImpCalResult = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DDRPHY_PHYSTATUS1), PHYSTATUS1_RGS_TX_OCD_A_IMPCALOUT);
            mcSHOW_DBG_MSG3(("2. ODT DRVN=%2d CALOUT=%d\n", u4ImpxDrv, u4ImpCalResult));
			//#endif
            
            if((u4ImpCalResult ==0) &&(u4ODTN_Result == 0xff)) //first found
            {
                u4ODTN_Result = u4ImpxDrv;
                mcSHOW_DBG_MSG(("\n2. ODT DRVN calibration OK! ODTN=%2d\n\n", u4ODTN_Result));
                mcFPRINTF((fp_A60501, "\n2. ODT DRVN calibration OK! ODTN=%2d\n\n", u4ODTN_Result)); 
                break;
            }
        }
    }
	#endif

    //LP4: ODTN calibration, LP3: DRVN calibration
    mcSHOW_DBG_MSG2(("======= K ODTN=====================\n"));
    mcFPRINTF((fp_A60501, "======= K ODTN=====================\n"));    
	//PUCMP_EN=0
	//LPDDR4 : ODT_EN=1
	vIO32WriteFldAlign((DDRPHY_PLL20), 0, PLL20_RG_IMPA_OCD_PUCMP_EN);

    #if ENABLE_LP4_SW
    if(p->dram_type == TYPE_LPDDR4)
    {    	
		vIO32WriteFldAlign((DDRPHY_IMPA1), 1, IMPA1_RG_RIMP_ODT_EN);
    }
    #endif
	//DRVP=DRVP_FINAL
	//DRVN=0
	//DRV05=1 
	vIO32WriteFldMulti((DDRPHY_IMPA1), P_Fld(u4DRVP_Result, IMPA1_RG_IMPA_ODTP_SW) | \
						P_Fld(0, IMPA1_RG_IMPA_ODTN_SW) | P_Fld(1, IMPA1_RG_RIMP_REV));

    //If RGS_TX_OCD_IMPCALOUTX=1
    //RG_IMPX_DRVN++;
    //Else save RG_IMPX_DRVN value and assign to DRVN
    for(u4ImpxDrv=0; u4ImpxDrv<32 ; u4ImpxDrv++)
    {
        if(u4ImpxDrv==16) //0~15, 29~31
            u4ImpxDrv = 29;

        vIO32WriteFldAlign((DDRPHY_IMPA1), u4ImpxDrv, IMPA1_RG_IMPA_ODTN_SW);
        mcDELAY_US(1);
        u4ImpCalResult = u4IO32ReadFldAlign((DDRPHY_PHY_RO_2), PHY_RO_2_RGS_RIMPCALOUT);
        mcSHOW_DBG_MSG3(("3. OCD ODTN=%d ,CALOUT=%d\n", u4ImpxDrv, u4ImpCalResult));
        mcFPRINTF((fp_A60501, "3. OCD ODTN=%d ,CALOUT=%d\n", u4ImpxDrv, u4ImpCalResult));
        
        if((u4ImpCalResult ==0) &&(u4ODTN_Result == 0xff))//first found
        {
            u4ODTN_Result = u4ImpxDrv;
            mcSHOW_DBG_MSG(("\n3. OCD ODTN calibration OK! ODTN=%d\n\n", u4ODTN_Result));
            mcFPRINTF((fp_A60501, "\n3. OCD ODTN calibration OK! ODTN=%d\n\n", u4ODTN_Result)); 
            break;
        }
    }

    //Register Restore 
    vIO32Write4B((DDRPHY_PLL20), u4BaklReg_DDRPHY_PLL20);
    vIO32Write4B((DDRPHY_IMPA1), u4BaklReg_DDRPHY_IMPA1);
    vIO32WriteFldAlign((DRAMC_REG_IMPCAL), 0, IMPCAL_IMP_CALI_ENP);

    #if ENABLE_LP4_SW
    if(p->dram_type == TYPE_LPDDR4)
    {
    	#if 1
        if(u4DRVP_Result==0xff)
        {
        	mcFPRINTF((fp_A60501, "\n[CHIP_SCAN]1. OCD DRVP calibration FAIL! \n\n"));
        	u4DRVP_Result = 9;
		}
        if(u4ODTN_Result==0xff || u4DRVP_Result==0xff)
        {
        	mcFPRINTF((fp_A60501, "\n[CHIP_SCAN]3. OCD ODTN calibration FAIL! \n\n")); 
        	u4ODTN_Result = 15;
		}
		u4DRVN_Result = 11; //fixed value from DE
		#else//A60501_TEST_CODE
		{
			char s[256];
			U32 u4DRVPInit=0xff, u4DRVNInit=0xff;
			FILE *pFile = fopen("a60501.ini", "r");
			if(pFile==NULL)
			{
				mcSHOW_DBG_MSG(("[ERROR] open a60501.ini FAIL \n\n"));
				while(1);
			}
			while(fgets(s, 255, pFile)!=NULL)
			{
				char *ptr1 = NULL;
				
				if(strstr(s, "DRVP="))
				{
					ptr1 = s+5;
					u4DRVPInit = atoi(ptr1);
				}
				if(strstr(s, "DRVN="))
				{
					ptr1 = s+5;
					u4DRVNInit = atoi(ptr1);
				}
			}
			fclose(pFile);
			if(u4DRVPInit==0xff || u4DRVNInit==0xff)
			{
				mcSHOW_DBG_MSG(("[ERROR] a60501.ini no DRVP/DRVN setting \n\n"));
				while(1);
			}
			if(u4ODTN_Result==0xff || u4DRVP_Result==0xff)
	        {
	        	u4ODTN_Result = 15;
			}
			u4DRVP_Result = u4DRVPInit;
			u4DRVN_Result = u4DRVNInit; //fixed value from
		}
		#endif

		mcSHOW_DBG_MSG(("DramcSwImpedanceCal : DRVP=%d , DRVN=%d , ODTN=%d\n\n", u4DRVP_Result, u4DRVN_Result, u4ODTN_Result));
        mcFPRINTF((fp_A60501, "DramcSwImpedanceCal : DRVP=%d , DRVN=%d , ODTN=%d\n\n", u4DRVP_Result, u4DRVN_Result, u4ODTN_Result)); 

        #if 0//HYNIX_IMPX_ADJUST
        if(u1Para)
        {
            u4ODTN_Result= ImpedanceAdjustment_Hynix(u4ODTN_Result, u1Para);
        }
        #endif
        
        //DQ
        vIO32WriteFldAlign_All((DDRPHY_TXDQ1), u4DRVP_Result, TXDQ1_RG_TX_ARDQ_DRVP_B0);
        vIO32WriteFldAlign_All((DDRPHY_TXDQ1), u4DRVN_Result, TXDQ1_RG_TX_ARDQ_DRVN_B0);
        vIO32WriteFldAlign_All((DDRPHY_TXDQ1), u4ODTN_Result, TXDQ1_RG_TX_ARDQ_ODTN_B0);
        vIO32WriteFldAlign_All((DDRPHY_TXDQ1)+0x60, u4DRVP_Result, TXDQ1_RG_TX_ARDQ_DRVP_B0);
        vIO32WriteFldAlign_All((DDRPHY_TXDQ1)+0x60, u4DRVN_Result, TXDQ1_RG_TX_ARDQ_DRVN_B0);
        vIO32WriteFldAlign_All((DDRPHY_TXDQ1)+0x60, u4ODTN_Result, TXDQ1_RG_TX_ARDQ_ODTN_B0);
        
        //DQS
        vIO32WriteFldAlign_All((DDRPHY_TXDQ2), u4DRVP_Result, TXDQ2_RG_TX_ARDQS0_DRVP);
        vIO32WriteFldAlign_All((DDRPHY_TXDQ2), u4DRVN_Result, TXDQ2_RG_TX_ARDQS0_DRVN);
        vIO32WriteFldAlign_All((DDRPHY_TXDQ2), u4ODTN_Result, TXDQ2_RG_TX_ARDQS0_ODTN);
        vIO32WriteFldAlign_All((DDRPHY_TXDQ2)+0x60, u4DRVP_Result, TXDQ2_RG_TX_ARDQS0_DRVP);
        vIO32WriteFldAlign_All((DDRPHY_TXDQ2)+0x60, u4DRVN_Result, TXDQ2_RG_TX_ARDQS0_DRVN);
        vIO32WriteFldAlign_All((DDRPHY_TXDQ2)+0x60, u4ODTN_Result, TXDQ2_RG_TX_ARDQS0_ODTN);
        
       //CMD 
        vIO32WriteFldAlign_All((DDRPHY_CMD4), u4DRVP_Result, CMD4_RG_TX_ARCMD_DRVP);
        vIO32WriteFldAlign_All((DDRPHY_CMD4), u4DRVN_Result, CMD4_RG_TX_ARCMD_DRVN);

        //CLK
        vIO32WriteFldAlign_All((DDRPHY_CMD4), u4DRVP_Result, CMD4_RG_TX_ARCLK_DRVP);
        vIO32WriteFldAlign_All((DDRPHY_CMD4), u4DRVN_Result, CMD4_RG_TX_ARCLK_DRVN);
    }
    else  //LPDDR3
    #endif
    {      
        u4DRVN_Result = u4ODTN_Result;
        if(u4DRVN_Result==0xff || u4DRVP_Result==0xff)
        {
            u4DRVP_Result = 8;
        	u4DRVN_Result = 9;
		}

        //DQ
        vIO32WriteFldAlign_All((DDRPHY_TXDQ1), u4DRVP_Result, TXDQ1_RG_TX_ARDQ_DRVP_B0);
        vIO32WriteFldAlign_All((DDRPHY_TXDQ1), u4DRVN_Result, TXDQ1_RG_TX_ARDQ_DRVN_B0);
        vIO32WriteFldAlign_All((DDRPHY_TXDQ9), u4DRVP_Result, TXDQ9_RG_TX_ARDQ_DRVP_B1);
        vIO32WriteFldAlign_All((DDRPHY_TXDQ9), u4DRVN_Result, TXDQ9_RG_TX_ARDQ_DRVN_B1);
        
        //DQS
        vIO32WriteFldAlign_All((DDRPHY_TXDQ2), u4DRVP_Result, TXDQ2_RG_TX_ARDQS0_DRVP);
        vIO32WriteFldAlign_All((DDRPHY_TXDQ2), u4DRVN_Result, TXDQ2_RG_TX_ARDQS0_DRVN);
        vIO32WriteFldAlign_All((DDRPHY_TXDQ10), u4DRVP_Result, TXDQ10_RG_TX_ARDQS1_DRVP);
        vIO32WriteFldAlign_All((DDRPHY_TXDQ10), u4DRVN_Result, TXDQ10_RG_TX_ARDQS1_DRVN);
        
       //CMD 
        vIO32WriteFldAlign_All((DDRPHY_CMD4), u4DRVP_Result, CMD4_RG_TX_ARCMD_DRVP);
        vIO32WriteFldAlign_All((DDRPHY_CMD4), u4DRVN_Result, CMD4_RG_TX_ARCMD_DRVN);

        //CLK
        vIO32WriteFldAlign_All((DDRPHY_CMD4), u4DRVP_Result, CMD4_RG_TX_ARCLK_DRVP);
        vIO32WriteFldAlign_All((DDRPHY_CMD4), u4DRVN_Result, CMD4_RG_TX_ARCLK_DRVN);
    }

    vSetCalibrationResult(p, DRAM_CALIBRATION_SW_IMPEDANCE, DRAM_OK); 
    mcSHOW_DBG_MSG2(("[DramcSwImpedanceCal] Done \n\n"));
    mcFPRINTF((fp_A60501, "[DramcSwImpedanceCal] Done \n\n"));
    
    return DRAM_OK;
}

//-------------------------------------------------------------------------
/** DramcHwImpedanceCal
 *  start TX OCD impedance calibration.
 *  @param p                Pointer of context created by DramcCtxCreate.
 *  @param  apply           (U8): 0 don't apply the register we set  1 apply the register we set ,default don't apply.
 *  @retval status          (DRAM_STATUS_T): DRAM_OK or DRAM_FAIL 
 */
//-------------------------------------------------------------------------
// only for ES validation
/////  NOT SUPPORT in A60501 !!!!!!
DRAM_STATUS_T DramcHwImpedanceCal(DRAMC_CTX_T *p)
{
#if 0
    #if TODO_PORTING_TO_60501
    U32 u4value;
    #endif       
    mcSHOW_DBG_MSG(("[HW Imp Calibr] Start HW impedance calibration...\n"));
    mcFPRINTF((fp_A60501, "[HW Imp Calibr] Start HW impedance calibration...\n"));
            
 #if REG_ACCESS_PORTING_DGB
    RegLogEnable =1;
    mcSHOW_DBG_MSG(("\n[REG_ACCESS_PORTING_FUNC]   DramcHwImpedanceCal\n"));
    mcFPRINTF((fp_A60501, "\n[REG_ACCESS_PORTING_FUNC]   DramcHwImpedanceCal\n"));
#endif
            
    // DRVREF (REG.100[24]) = 0: change will be apply directly 
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_OCDK), 0, OCDK_DRVREF);

    // Set R_DMIMPCAL_HW (REG.1C8[1]) = 1 (HW)
    // Set R_DMIMPCAL_CHKCYCLE (REG.1C8[19][3:2]) = 3
    // Set R_DMIMPCAL_CALEN_CYCLE (REG.1C8[18:16]) = 3
    // Set R_DMIMPCAL_CALICNT (REG.1C8[31:28]) = 7
    // Set R_DMIMPCALCNT (REG.1C8[27:20]) = 0 @ init; 0xf @ run-time
    vIO32WriteFldMulti_C(DRAMC_REG_ADDR(DRAMC_REG_IMPCAL), \
    						P_Fld(1, IMPCAL_IMPCAL_HW)|P_Fld(3, IMPCAL_IMPCAL_CJKCYCLE)| \
    						P_Fld(3, IMPCAL_IMPCAL_CALEN_CYCLE)|P_Fld(7, IMPCAL_IMPCAL_CALICNT)| \
    						P_Fld(0, IMPCAL_IMPCALCNT));
    	
    // Set R_DMIMPCALI_EN (REG.1C8[0]) = 1
    vIO32WriteFldAlign_C(DRAMC_REG_ADDR(DRAMC_REG_IMPCAL), 1, IMPCAL_IMPCALI_EN);
         
    // Check calibrated result
    // only show in drvp_status in A60501, not sure if HW has filled into driving (Rome will have registers to read)
    // ~1.5us
    mcDELAY_US(5);

#if TODO_PORTING_TO_60501  //read the result for check.
    u4value= u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DLLCNT0), DLLCNT0_DRVP_SAVE);
    mcSHOW_DBG_MSG(("[HW Imp Calibr] drvp_save  : %2d\n",u4value));
    mcFPRINTF((fp_A60501, "[HW Imp Calibr] drvp_save  : %2d\n",u4value));

    u4value= u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DLLCNT0), DLLCNT0_DRVN_SAVE);
    mcSHOW_DBG_MSG(("[HW Imp Calibr] drvn_save  : %2d\n",u4value));
    mcFPRINTF((fp_A60501, "[HW Imp Calibr] drvn_save  : %2d\n",u4value));

    u4value= u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DLLCNT0), DLLCNT0_DRVP_SAVE_2);
    mcSHOW_DBG_MSG(("[HW Imp Calibr] drvp_save_2: %2d\n",u4value));
    mcFPRINTF((fp_A60501, "[HW Imp Calibr] drvp_save_2: %2d\n",u4value));

    u4value= u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DLLCNT0), DLLCNT0_DRVN_SAVE_2);
    mcSHOW_DBG_MSG(("[HW Imp Calibr] drvp_save_2: %2d\n",u4value));
    mcFPRINTF((fp_A60501, "[HW Imp Calibr] drvp_save_2: %2d\n",u4value));
#endif
        
    // restore settings
    // R_DMIMPCALI_EN (REG.1C8[0]) = 0
    vIO32WriteFldAlign_C(DRAMC_REG_ADDR(DRAMC_REG_IMPCAL), 0, IMPCAL_IMPCALI_EN);

    // DRVREF (REG.100[24]) = 1: change will be apply during refresh
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_OCDK), 1, OCDK_DRVREF);

#if REG_ACCESS_PORTING_DGB
    RegLogEnable =0;
#endif

#endif  //end of #if 0
    return DRAM_OK;
}


void Dram_Reset(DRAMC_CTX_T *p)
{
    // This function is implemented based on DE's bring up flow for DRAMC
    DramcModeRegInit_Everest_LP3(p);

    ///TODO: update MR2(WL& RL) from AC timing table
}

static void O1PathOnOff(DRAMC_CTX_T *p, U8 u1OnOff)
{
    if(u1OnOff)
    {
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_MCKDLY), 0xf, MCKDLY_FIXDQIEN);
    }
    else
    {
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_MCKDLY), 0, MCKDLY_FIXDQIEN);
    }

    #if ENABLE_LP4_SW
    if(p->dram_type == TYPE_LPDDR4)
    {
        if(u1OnOff)
        {
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE2), 0xd, EYE2_RG_RX_ARDQ_EYE_VREF_SEL_B0);
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), 0xd, EYEB1_2_RG_RX_ARDQ_EYE_VREF_SEL_B1);
        }
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_TXDQ3), u1OnOff, TXDQ3_RG_RX_ARDQ_SMT_EN_B0); 
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_RXDQ13), u1OnOff, RXDQ13_RG_RX_ARDQ_SMT_EN_B1); 
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE2), u1OnOff, EYE2_RG_RX_ARDQ_EYE_VREF_EN_B0);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), u1OnOff, EYEB1_2_RG_RX_ARDQ_EYE_VREF_EN_B1);
    }
    else //LPDDR3
    #endif
    {
        if(u1OnOff)
        {
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE2), 0xe, EYE2_RG_RX_ARDQ_EYE_VREF_SEL_B0);
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), 0xe, EYEB1_2_RG_RX_ARDQ_EYE_VREF_SEL_B1);
        }
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_TXDQ3), u1OnOff, TXDQ3_RG_RX_ARDQ_SMT_EN_B0);
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ13), u1OnOff, RXDQ13_RG_RX_ARDQ_SMT_EN_B1);
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE2), u1OnOff, EYE2_RG_RX_ARDQ_EYE_VREF_EN_B0);
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), u1OnOff, EYEB1_2_RG_RX_ARDQ_EYE_VREF_EN_B1);
    }
}

// LPDDR DQ -> PHY DQ mapping
// in A60501, it's 1-1 mapping
#if fcFOR_CHIP_ID == fcA60501
const U32 uiLPDDR_PHY_Mapping_SBS[32] = {20, 17, 19, 16, 21, 22, 23, 18,
                                      13, 15, 9, 11, 12, 10, 14, 8,
                                      31, 26, 29, 27, 28, 24, 30, 25, 
                                      3, 1, 7, 2, 0, 4, 5, 6};

const U32 uiLPDDR_PHY_Mapping_POP[32] = {14, 10, 8, 11, 12, 15, 9, 13,
                                      22, 18, 21, 23, 16, 19, 20, 17,
                                      5, 7, 6, 4, 3, 2, 1, 0, 
                                      30, 29, 31, 27, 28, 26, 25, 24};

// DQ16~31 not modify yet.
const U32 uiLPDDR_PHY_Mapping_POP_LP3[32] = {0, 1, 3, 2, 4, 5, 6, 7,
                                      15, 14, 13, 12, 11, 10, 9, 8,
                                      5, 7, 6, 4, 3, 2, 1, 0, 
                                      30, 29, 31, 27, 28, 26, 25, 24};

#elif fcFOR_CHIP_ID == fcEverest
const U32 uiLPDDR_PHY_Mapping_POP_CHA[32] = 
{
    1, 0, 2, 3, 4, 5, 7, 6, 
    13, 14 , 15,  10, 8, 12, 11, 9,
    17, 16, 19, 18, 20, 21, 22, 23,  
    30, 31,  29, 28, 26, 27, 24, 25
};

const U32 uiLPDDR_PHY_Mapping_POP_CHB[32] = 
{
    1, 0, 2, 3, 4, 5, 7, 6, 
    14, 15, 13, 12, 10, 11, 9, 8,
    17, 16, 19, 18, 20, 21, 23, 22, 
    31, 30, 29, 28, 26, 27, 25, 24
    };
#endif

//-------------------------------------------------------------------------
/** DramcCATraining
 *  start the calibrate the skew between Clk pin and CAx pins.
 *  @param p                Pointer of context created by DramcCtxCreate.
 *  @retval status          (DRAM_STATUS_T): DRAM_OK or DRAM_FAIL 
 */
//-------------------------------------------------------------------------
#define MAX_CLKO_DELAY         31
#define CATRAINING_NUM        10

static DRAM_STATUS_T CATrainingEntry(DRAMC_CTX_T *p, U32 uiMR41, U32 u4GoldenPattern)
{     
    //CA_TRAINING_BEGIN:

    // Set CA0~CA3, CA5~CA8 output delay to 0.
    //vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_CMD5), 0, PHY_FLD_FULL);// CA0~CA7
    //vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_CMD6), 0, CMD6_DA_TX_ARCA8_DLY);
    //vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_CMD6), 0, CMD6_DA_TX_ARCA9_DLY);

    // CS extent enable (need DRAM to support)
    // for testing
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_MISC), 1, MISC_CATRAINCSEXT); 

    // CKE high, CKE must be driven HIGH prior to issuance of the MRW41 and MRW48 command
    vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_PADCTL4), P_Fld(0, PADCTL4_CKEFIXOFF)|P_Fld(1, PADCTL4_CKEFIXON));

    // Enter MR 41/MR48
    // Set MA & OP.
    if (uiMR41) 
    {
    	 vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_MRS), P_Fld(0xa4, MRS_MRSOP)|P_Fld(0,MRS_MRSBA)|P_Fld(41, MRS_MRSMA));
    } 
    else     
    {
        vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_MRS), P_Fld(0xc0, MRS_MRSOP)|P_Fld(0,MRS_MRSBA)|P_Fld(48, MRS_MRSMA));
    }
    // Hold the CA bus stable for at least one cycle.
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_CATRAINING), 1, CATRAINING_CATRAINMRS);

    // MRW
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 1, SPCMD_MRWEN);
    mcDELAY_US(1);
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 0, SPCMD_MRWEN);

    // Disable CA bus stable.
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_CATRAINING), 0, CATRAINING_CATRAINMRS);

    // Wait tCACKEL(10 tck) before CKE low
    mcDELAY_US(1);

    // CKE low
    vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_PADCTL4), P_Fld(1, PADCTL4_CKEFIXOFF)|P_Fld(0,PADCTL4_CKEFIXON));

    // Set CA0~CA3, CA5~CA8 rising/falling golden value.
    vIO32Write4B(DRAMC_REG_ADDR(DRAMC_REG_CATRAINING2), u4GoldenPattern);

    // Wait tCAENT(10 tck) before pattern output
    mcDELAY_US(1);

    return DRAM_OK;
}


static S32 CheckCATrainingTransition(U32 uiCA, U32 capattern, U32 uiRisingEdge, U32 uiFallingEdge)
{
    S32 iPass=0;
    U32 iii;

    if (capattern == 0x55555555)
    {
         if ((uiRisingEdge!=0) && (uiFallingEdge==0)) 
         {
             iPass = 1;
         } 
         else
         {
             iPass = 0;
         }                    
    }
    else if (capattern == 0xaaaaaaaa)
    {
        if ((uiRisingEdge==0) && (uiFallingEdge!=0)) 
        {
            iPass = 1;
        } 
        else
        {
            iPass = 0;
        }
    }
    else if (capattern == 0x99999999)
    {
        iii = uiCA;
        if (iii>=5) iii-=5;

        if ((iii & 1) == 0)
        {
            if ((uiRisingEdge!=0) && (uiFallingEdge==0)) 
            {
                iPass = 1;
            } 
            else
            {
                iPass = 0;
            }                                                
        }
        else
        {
            if ((uiRisingEdge==0) && (uiFallingEdge!=0)) 
            {
                iPass = 1;
            } 
            else
            {
                iPass = 0;
            }                            
        }
    }                    
    else if (capattern == 0x66666666)
    {
        iii = uiCA;
        if (iii>=5) iii-=5;

        if ((iii & 1) == 0)
        {
            if ((uiRisingEdge==0) && (uiFallingEdge!=0)) 
            {
                iPass = 1;
            } 
            else
            {
                iPass = 0;
            }                        
        }
        else
        {
            if ((uiRisingEdge!=0) && (uiFallingEdge==0)) 
            {
                iPass = 1;
            } 
            else
            {
                iPass = 0;
            }                                                                            
        }
    } 
    
    return iPass;                    
}


static S32 CATrainingDelayCompare(DRAMC_CTX_T *p, U32 uiMR41, U32 u4GoldenPattern)
{
    U32 *uiLPDDR_PHY_Mapping;
//    U32 u4O1Data[DQS_NUMBER];
    U32 u4dq_o1_tmp[DQS_NUMBER];
    U32 uiTemp, uiCA, uiRisingEdge, uiFallingEdge, uiFinishCount;
    S32 iFirstCAPass[CATRAINING_NUM], iLastCAPass[CATRAINING_NUM], iPass, iDelay;
    S32 iCenter[CATRAINING_NUM], iCenterSum=0;

#if fcFOR_CHIP_ID == fcA60501
    if (p->package==PACKAGE_POP)
    {
        uiLPDDR_PHY_Mapping = (U32 *)uiLPDDR_PHY_Mapping_POP_LP3;
    }
    else
    {
        // Should be SBS.
        uiLPDDR_PHY_Mapping = (U32 *)uiLPDDR_PHY_Mapping_SBS;
    }
#elif fcFOR_CHIP_ID == fcEverest
    if (p->channel == CHANNEL_A)
    {
        uiLPDDR_PHY_Mapping = (U32 *)uiLPDDR_PHY_Mapping_POP_CHA;
    }
    else
    {
        uiLPDDR_PHY_Mapping = (U32 *)uiLPDDR_PHY_Mapping_POP_CHB;
    }
#endif

    // Calculate the middle range & max middle.
    mcSHOW_DBG_MSG(("=========================================\n"));
    mcSHOW_DBG_MSG(("[CA Training] Frequency=%d, Channel=%d, Rank=%d\n", p->frequency, p->channel, p->rank));
    mcSHOW_DBG_MSG(("x=Pass window CA(max~min) Clk(min~max) center. \n"));
    if(uiMR41)
    {
        mcSHOW_DBG_MSG(("y=CA0~CA3, CA5~8\n"));
    }
    else//MR48
    {
        mcSHOW_DBG_MSG(("y=CA4 CA9\n"));
    }
    mcSHOW_DBG_MSG(("=========================================\n"));

    mcFPRINTF((fp_A60501, "=========================================\n"));
    mcFPRINTF((fp_A60501, "1. CA training window before adjustment.\n"));
    mcFPRINTF((fp_A60501, "x=Pass window CA(max~min) Clk(min~max) center. \n"));
    if(uiMR41)
    {
        mcFPRINTF((fp_A60501, "y=CA0~CA3, CA5~8\n"));
    }
    else//MR48
    {
        mcFPRINTF((fp_A60501, "y=CA4 CA9\n"));
    }
    mcFPRINTF((fp_A60501, "=========================================\n"));

    uiFinishCount = 0;
    for (uiCA=0; uiCA<CATRAINING_NUM; uiCA++) 
    {
         iLastCAPass[uiCA] = PASS_RANGE_NA;
         iFirstCAPass[uiCA] = PASS_RANGE_NA;
    }

    vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_ARPI_CMD), 0, ARPI_CMD_DA_ARPI_CMD);
  
    // Delay clock output delay to do CA training in order to get the pass window.
    for (iDelay= -(MAX_CLK_PI_DELAY/2); iDelay <= MAX_CA_PI_DELAY; iDelay++)
    {     
        if(iDelay <=0) 
        {    //Set CLK delay
              vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_ARPI_CMD), -iDelay, ARPI_CMD_DA_ARPI_CK);
        }
        else 
        {    // Set CA output delay
              vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_ARPI_CMD), iDelay, ARPI_CMD_DA_ARPI_CMD);
        }

        // CA training pattern output enable
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_CATRAINING), 1, CATRAINING_CATRAINEN);
        // delay 2 DRAM clock cycle
        mcDELAY_US(1);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_CATRAINING), 0, CATRAINING_CATRAINEN);
        
        // Wait tADR(20ns) before CA sampled values available in DQ.
        mcDELAY_US(1);

        // Get DQ value.
        u4dq_o1_tmp[0] = u4IO32ReadFldAlign(DDRPHY_NEW_DQ_O1_RO +(aru1PhyMap2Channel[0]<< POS_BANK_NUM), NEW_DQ_O1_RO_NEW_DQ_O1); 
        mcSHOW_DBG_MSG3(("DQ_O1_AA 0x%X  ", u4dq_o1_tmp[0]));
        u4dq_o1_tmp[2] = ((u4dq_o1_tmp[0] >>8) & 0xff);
        u4dq_o1_tmp[0] &= 0xff;

        u4dq_o1_tmp[1] = u4IO32ReadFldAlign(DDRPHY_NEW_DQ_O1_RO +(aru1PhyMap2Channel[1]<< POS_BANK_NUM), NEW_DQ_O1_RO_NEW_DQ_O1);         
        mcSHOW_DBG_MSG3(("DQ_O1_BB 0x%X  ", u4dq_o1_tmp[1]));
        u4dq_o1_tmp[3] = ((u4dq_o1_tmp[1] >>8) & 0xff);
        u4dq_o1_tmp[1] &= 0xff;
        
        uiTemp = (u4dq_o1_tmp[0]) | (u4dq_o1_tmp[1] <<8) | (u4dq_o1_tmp[2]  <<16) | (u4dq_o1_tmp[3] <<24);
        
        #ifdef ETT_PRINT_FORMAT
        mcSHOW_DBG_MSG2(("%d, | ", iDelay));
        #else
        mcSHOW_DBG_MSG2(("delay %3d, DQ_O1 0x%8x| Pass ", iDelay, uiTemp));
        #endif
        mcFPRINTF((fp_A60501, "delay %3d, DQ_O1 0x%8x| Pass ", iDelay, uiTemp));

        // Compare with golden value.
        for (uiCA=0; uiCA<CATRAINING_NUM; uiCA++) 
        {
            if(uiMR41 && ((uiCA==4) || (uiCA==9)))  // MR41 is for CA0~3, CA5~8
            {
                continue;
            }
            else if((uiMR41==0) && ((uiCA!=4) && (uiCA!=9)))// MR48 is for CA4, CA8
            {
                continue;
            }

           // if ( (iFirstCAPass[uiCA]==PASS_RANGE_NA) || (iLastCAPass[uiCA]==PASS_RANGE_NA)) //marked fo debug
            {
                if (uiCA<4)   //CA0~3
                {
                    uiRisingEdge = uiTemp & (0x01 << uiLPDDR_PHY_Mapping[uiCA<<1]);
                    uiFallingEdge = uiTemp & (0x01 << uiLPDDR_PHY_Mapping[(uiCA<<1)+1]); 
                }
                else if((uiCA==4) || (uiCA==9))
                {
                    uiRisingEdge = uiTemp & (0x01 << uiLPDDR_PHY_Mapping[(uiCA==4) ? 0 : 8]);
                    uiFallingEdge = uiTemp & (0x01 << uiLPDDR_PHY_Mapping[(uiCA==4) ? 1 : 9]); 
                }
                else//CA5~8
                {
                    uiRisingEdge = uiTemp & (0x01 << uiLPDDR_PHY_Mapping[(uiCA-1)<<1]);
                    uiFallingEdge = uiTemp & (0x01 << uiLPDDR_PHY_Mapping[((uiCA-1)<<1)+1]); 
                }

                iPass = CheckCATrainingTransition(uiCA, u4GoldenPattern, uiRisingEdge, uiFallingEdge);
                mcSHOW_DBG_MSG2(("%d ", iPass));
                mcFPRINTF((fp_A60501, "%d ", iPass));
                
                if (iFirstCAPass[uiCA]==PASS_RANGE_NA)
                {
                    if (iPass == 1) 
                    {
                        iFirstCAPass[uiCA] = iDelay;
                    }
                }
                else
                {
                    if (iLastCAPass[uiCA]==PASS_RANGE_NA)
                    {                    
                        if (iPass == 0) 
                        {
                            if((iDelay-iFirstCAPass[uiCA]) <5)  // prevent glitch 
                            {
                                iFirstCAPass[uiCA]=PASS_RANGE_NA;                      
                                continue;
                            }
                         
                            uiFinishCount++;
                            iLastCAPass[uiCA] = iDelay-1;                               
                            
                            iCenter[uiCA] = (iLastCAPass[uiCA] + iFirstCAPass[uiCA]) >>1;
                            iCenterSum += iCenter[uiCA];
                        }
                        else
                        {
                            if (iDelay==MAX_CA_PI_DELAY)
                            {
                                uiFinishCount++;
                                iLastCAPass[uiCA] = iDelay;
                                
                                iCenter[uiCA] = (iLastCAPass[uiCA] + iFirstCAPass[uiCA]) >>1;
                                iCenterSum += iCenter[uiCA];
                            }
                        }
                    }
                }
            }
        }

        // Wait tCACD(22clk) before output CA pattern to DDR again..
        mcDELAY_US(1);   

        mcSHOW_DBG_MSG2(("\n"));
        mcFPRINTF((fp_A60501, "\n"));

        if((uiMR41 && (uiFinishCount==8)) || ((uiMR41==0) && (uiFinishCount==2)))
        {
            mcSHOW_DBG_MSG(("[CATrainingDelayCompare] Early break, uiMR41=%d, uiFinishCount=%d\n", uiMR41, uiFinishCount));
            mcFPRINTF((fp_A60501, "[CATrainingDelayCompare] Early break, uiMR41=%d\n", uiMR41));
            break;
        }
    }


    vSetCalibrationResult(p, DRAM_CALIBRATION_CA_TRAIN, DRAM_OK); // set default result OK, udpate status when per bit fail
    
    for (uiCA=0; uiCA<CATRAINING_NUM; uiCA++) 
    {
        if(uiMR41 && ((uiCA==4) || (uiCA==9)))  // MR41 is for CA0~3, CA5~8
        {
            continue;
        }
        else if((uiMR41==0) && ((uiCA!=4) && (uiCA!=9)))// MR48 is for CA4, CA8
        {
            continue;
        }

        #ifdef ETT_PRINT_FORMAT
        mcSHOW_DBG_MSG(("CA%d  (%d~%d) %d,\n", uiCA, iFirstCAPass[uiCA], iLastCAPass[uiCA], iCenter[uiCA]));
        #else
        mcSHOW_DBG_MSG(("CA%3d   Pass(%3d~%3d)  Center %3d,\n", uiCA, iFirstCAPass[uiCA], iLastCAPass[uiCA], iCenter[uiCA]));
        #endif
        mcFPRINTF((fp_A60501, "CA%4d Pass(%3d~%3d) Center %3d,\n", uiCA, iFirstCAPass[uiCA], iLastCAPass[uiCA], iCenter[uiCA]));

        if(iLastCAPass[uiCA]==PASS_RANGE_NA)  // no CA window found
        { 
            vSetCalibrationResult(p, DRAM_CALIBRATION_CA_TRAIN, DRAM_FAIL);
        }
    }

    // CS extent disable
    // for testing
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_MISC), 0, MISC_CATRAINCSEXT);

    // Wait tCACKEN (10ck)
    mcDELAY_US(1);        

    if((uiMR41 && (uiFinishCount<8)) || ((uiMR41==0) && (uiFinishCount<2)))
    {
        mcSHOW_DBG_MSG(("[CATrainingDelayCompare] Error: some bits has abnormal window, uiMR41=%d, uiFinishCount=%d\n", uiMR41, uiFinishCount));
        mcFPRINTF((fp_A60501, "[CATrainingDelayCompare] Error: some bits has abnormal window, uiMR41=%d, uiFinishCount=%d\n", uiMR41, uiFinishCount));
    }
    
    return iCenterSum;
}


static void CATrainingExit(DRAMC_CTX_T *p)
{

    vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_PADCTL4), P_Fld(0, PADCTL4_CKEFIXOFF)|P_Fld(1,PADCTL4_CKEFIXON));
    mcDELAY_US(1);

    // CS extent enable
    // for testing
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_MISC), 1, MISC_CATRAINCSEXT); 

    // MR42 to leave CA training.
    vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_MRS), P_Fld(0xa8, MRS_MRSOP)|P_Fld(0,MRS_MRSBA)|P_Fld(0x2a, MRS_MRSMA));

    // Hold the CA bus stable for at least one cycle.
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_CATRAINING), 1, CATRAINING_CATRAINMRS);
    
    // MRW
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 1, SPCMD_MRWEN);
    mcDELAY_US(1);
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 0, SPCMD_MRWEN);
    
    // Disable the hold the CA bus stable for at least one cycle.
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_CATRAINING), 0, CATRAINING_CATRAINMRS);

    // CS extent disable
    // for testing
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_MISC), 0, MISC_CATRAINCSEXT); 
}

static DRAM_STATUS_T CATrainingLP3(DRAMC_CTX_T *p)
{
    U32 uiMR41;
    U32 uiReg008h; 
    U32 uiReg1DCh;
    S32 iFinalCACLK;
    S32 iCenterSum=0;
    
    //  01010101b -> 10101010b : Golden value = 1001100110011001b=0x9999
    //  11111111b -> 00000000b : Golden value = 0101010101010101b=0x5555
    U32 u4GoldenPattern =0x55555555;
    //U32 u4GoldenPattern =0xaaaaaaaa;

    mcSHOW_DBG_MSG(("\n[CATrainingLP3]   Begin\n"));
    mcFPRINTF((fp_A60501, "\n[CATrainingLP3]   Begin\n"));

    // Fix ODT off. A60501 disable ODT in the init code. So no need to do the following code.
    //uiReg54h=u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_WODT), WODT_WODTFIXOFF);
    //vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_WODT), 1, WODT_WODTFIXOFF);// According to Derping, should set to 1 to disable ODT.

    // Let MIO_CK always ON.
    #if 1
    // Disable auto refresh: REFCNT_FR_CLK = 0 (0x1dc[23:16])
    uiReg1DCh=u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_DRAMC_PD_CTRL));
    uiReg008h = u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_CONF2));

    vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_DRAMC_PD_CTRL), P_Fld(1, DRAMC_PD_CTRL_MIOCKCTRLOFF) |P_Fld(0, DRAMC_PD_CTRL_REFCNT_FR_CLK));
    vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_CONF2), P_Fld(1, CONF2_REFDIS) |P_Fld(0, CONF2_REFCNT));
    #else
    //disable auto refresh, REFCNT_FR_CLK = 0 (0x1dc[23:16]), ADVREFEN = 0 (0x44[30]), (CONF2_REFCNT =0)
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DRAMC_PD_CTRL), 0, DRAMC_PD_CTRL_REFCNT_FR_CLK);   
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_TEST2_3), 0, TEST2_3_ADVREFEN);
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_CONF2), 0, CONF2_REFCNT);// new in 60817  Yulia change
    #endif

    // Step 1.1 : let IO to O1 path valid
    O1PathOnOff(p, 1);

    // ----- MR41, CA0~3, CA5~8 ------- 
    uiMR41 = 1;
    CATrainingEntry(p, uiMR41, u4GoldenPattern);  //MR41
    iCenterSum += CATrainingDelayCompare(p, uiMR41, u4GoldenPattern); 

    // ----- MR48, CA4 and 9 ------- 
    uiMR41 = 0;
    CATrainingEntry(p, uiMR41, u4GoldenPattern);  //MR48
    iCenterSum += CATrainingDelayCompare(p, uiMR41, u4GoldenPattern); 
    
    iFinalCACLK =iCenterSum/CATRAINING_NUM;
    
    mcSHOW_DBG_MSG(("=========================================\n"));
    #ifdef ETT_PRINT_FORMAT    
    mcSHOW_DBG_MSG(("u4GoldenPattern 0x%X, iFinalCACLK = %d\n\n", u4GoldenPattern, iFinalCACLK));
    #else    
    mcSHOW_DBG_MSG(("u4GoldenPattern 0x%x, iFinalCACLK = %d\n\n", u4GoldenPattern, iFinalCACLK));
    #endif
    
    mcFPRINTF((fp_A60501, "=========================================\n"));
    mcFPRINTF((fp_A60501, "iFinalCACLK = %d\n\n", iFinalCACLK));

    #if REG_SHUFFLE_REG_CHECK
    ShuffleRegCheck =1;
    #endif

    if(iFinalCACLK <0) 
    {    //SEt CLK delay
    #if CA_TRAIN_RESULT_DO_NOT_MOVE_CLK
          CATrain_ClkDelay[p->channel] =0;  /// should not happen.
          vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_ARPI_CMD), 0x10, ARPI_CMD_DA_ARPI_CMD);  // set to defult
          
          vSetCalibrationResult(p, DRAM_CALIBRATION_CA_TRAIN, DRAM_FAIL);  
          mcSHOW_ERR_MSG(("Error : CLK delay is not 0 (%d), Set as 0.  CA set as 0x10\n\n", -iFinalCACLK));
          mcFPRINTF((fp_A60501, "Error : CLK delay is not 0 (%d), Set as 0.  CA set as 0x10\n\n", -iFinalCACLK));
          //while(1){};
    #else
          CATrain_ClkDelay[p->channel] = -iFinalCACLK;
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_ARPI_CMD), 0, ARPI_CMD_DA_ARPI_CMD);
        mcSHOW_DBG_MSG(("Clk Dealy is = %d, CA delay is 0\n\n", -iFinalCACLK));
        mcFPRINTF((fp_A60501, "Clk Dealy is = %d, CA delay is 0\n\n", -iFinalCACLK));
    #endif  
    }
    else 
    {    // Set CA output delay
          CATrain_ClkDelay[p->channel] = 0;

#if LP3_JV_WORKAROUND
        if((p->frequency >=800) && (iFinalCACLK>2))
        {
            mcSHOW_DBG_MSG(("LP3_JV_WORKAROUND: sepcial setting, CA delay fine-tune (%d->%d)n\n", iFinalCACLK, iFinalCACLK-2));
            iFinalCACLK -=2;
        }
#endif
          vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_ARPI_CMD), iFinalCACLK, ARPI_CMD_DA_ARPI_CMD);
          mcSHOW_DBG_MSG(("Clk Dealy is 0, CA delay is %d\n\n", iFinalCACLK));
          mcFPRINTF((fp_A60501, "Clk Dealy is 0, CA delay is %d\n\n", iFinalCACLK));
    }
    // no need to enter self refresh before setting CLK under CA training mode
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_ARPI_CMD), CATrain_ClkDelay[p->channel] , ARPI_CMD_DA_ARPI_CK);
    mcSHOW_DBG_MSG(("=========================================\n"));    
    mcFPRINTF((fp_A60501, "=========================================\n"));    

    #if REG_SHUFFLE_REG_CHECK
    ShuffleRegCheck =0;
    #endif

    CATrainingExit(p);

    // Disable fix DQ input enable.  Disable IO to O1 path
    O1PathOnOff(p, 0);
    
    // Disable CKE high, back to dynamic
    vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_PADCTL4), P_Fld(0, PADCTL4_CKEFIXOFF)|P_Fld(0,PADCTL4_CKEFIXON));

     // Restore the registers' values.
    //vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_WODT), uiReg54h, WODT_WODTFIXOFF);
    vIO32Write4B(DRAMC_REG_ADDR(DRAMC_REG_DRAMC_PD_CTRL), uiReg1DCh);
    vIO32Write4B(DRAMC_REG_ADDR(DRAMC_REG_CONF2), uiReg008h);
                  
    return DRAM_OK;
}

DRAM_STATUS_T DramcCATraining(DRAMC_CTX_T *p)
{
    DRAM_STATUS_T RetStatus = DRAM_OK;

#if REG_ACCESS_PORTING_DGB
    RegLogEnable =1;
    mcSHOW_DBG_MSG(("\n[REG_ACCESS_PORTING_FUNC]   DramcCATraining\n"));
    mcFPRINTF((fp_A60501, "\n[REG_ACCESS_PORTING_FUNC]   DramcCATraining\n"));
#endif

    // error handling
    if (!p)
    {
        mcSHOW_ERR_MSG(("context is NULL\n"));
        RetStatus= DRAM_FAIL;
    }

    if(p->dram_type == TYPE_LPDDR3)
    {
        RetStatus = CATrainingLP3(p);
    }
    #if 0//ENABLE_LP4_SW
    else if(p->dram_type == TYPE_LPDDR4)
    {
        RetStatus = CmdBusTrainingLP4(p);
        //RetStatus = CmdBusTrainingLP4_Test(p);
    }
    #endif
    else 
    {
        mcSHOW_ERR_MSG(("Wrong DRAM TYPE. Only support LPDDR3/LPDDR4 in CA training!!\n"));
        RetStatus = DRAM_FAIL;
    }
    
    //After CA training, since we only let rank0 enter ca training mode, rank1 may failed due to unexpected CLK jitter¡K
    //So we reset DRAM and re-issue all MR's (@ DramcInit) after CA training¡K
    Dram_Reset(p);

#if REG_ACCESS_PORTING_DGB
    RegLogEnable =0;
#endif

return RetStatus;
}

//-------------------------------------------------------------------------
/** DramcWriteLeveling
 *  start Write Leveling Calibration.
 *  @param p                Pointer of context created by DramcCtxCreate.
 *  @param  apply           (U8): 0 don't apply the register we set  1 apply the register we set ,default don't apply.
 *  @retval status          (DRAM_STATUS_T): DRAM_OK or DRAM_FAIL 
 */
//-------------------------------------------------------------------------
#define WRITE_LEVELING_MOVD_DQS 1//UI

typedef struct _REG_TRANSFER
{
    U32 u4Addr;
    U32 u4Fld;
} REG_TRANSFER_T;


// NOT suitable for Gating delay
static DRAM_STATUS_T ExecuteMoveDramCDelay(DRAMC_CTX_T *p, REG_TRANSFER_T regs[], S8 iShiftUI)
{
    S32 s4HighLevelDelay, s4DelaySum;
    U32 u4Tmp0p5T, u4Tmp2T;
    U8 ucDataRateDiv;
    DRAM_STATUS_T MoveResult;

    if(p->dram_type == TYPE_LPDDR4)
            ucDataRateDiv= 8;
    else //LPDDR3
        ucDataRateDiv =4;

    u4Tmp0p5T = u4IO32ReadFldAlign(DRAMC_REG_ADDR(regs[0].u4Addr), regs[0].u4Fld);
    u4Tmp0p5T |= (u4IO32ReadFldAlign(DRAMC_REG_ADDR(regs[1].u4Addr), regs[1].u4Fld) <<2);
    u4Tmp2T = u4IO32ReadFldAlign(DRAMC_REG_ADDR(regs[2].u4Addr), regs[2].u4Fld);
    //mcSHOW_DBG_MSG(("\n[MoveDramC_Orz]  u4Tmp2T:%d,  u4Tmp0p5T: %d,\n",  u4Tmp2T, u4Tmp0p5T));
    //mcFPRINTF((fp_A60501, "\n[MoveDramC_Orz]  u4Tmp2T:%d,  u4Tmp0p5T: %d,\n",  u4Tmp2T, u4Tmp0p5T));

    s4HighLevelDelay = u4Tmp2T *ucDataRateDiv + u4Tmp0p5T;
    s4DelaySum = (s4HighLevelDelay + iShiftUI);
    //mcSHOW_DBG_MSG(("\n[MoveDramC_Orz]  s4HighLevelDealy(%d) +  iShiftUI(%d) = %d\n",  s4HighLevelDelay, iShiftUI, s4DelaySum));

    if(s4DelaySum < 0)
    {
        u4Tmp0p5T =0;
        u4Tmp2T=0;
        MoveResult =  DRAM_FAIL;
        //mcSHOW_ERR_MSG(("\n[MoveDramC_Orz]  s4HighLevelDealy(%d) +  iShiftUI(%d) is small than 0!!\n",  s4HighLevelDelay, iShiftUI));
    }
    else
    {
        u4Tmp0p5T = s4DelaySum % ucDataRateDiv;
        u4Tmp2T = s4DelaySum / ucDataRateDiv;
        MoveResult = DRAM_OK;
    }
    
    vIO32WriteFldAlign(DRAMC_REG_ADDR(regs[0].u4Addr), u4Tmp0p5T, regs[0].u4Fld);
    vIO32WriteFldAlign(DRAMC_REG_ADDR(regs[1].u4Addr), u4Tmp0p5T>>2, regs[1].u4Fld);
    vIO32WriteFldAlign(DRAMC_REG_ADDR(regs[2].u4Addr), u4Tmp2T, regs[2].u4Fld);
    //mcSHOW_DBG_MSG(("\n[MoveDramC_Orz]  Final ==> u4Tmp2T:%d,  u4Tmp0p5T: %d,\n",  u4Tmp2T, u4Tmp0p5T));
    //mcFPRINTF((fp_A60501, "\n[MoveDramC_Orz]  Final ==> u4Tmp2T:%d,  u4Tmp0p5T: %d,\n",  u4Tmp2T, u4Tmp0p5T));

    return MoveResult;
}

#if WRITE_LEVELING_MOVE_DQS_INSTEAD_OF_CLK
static void MoveDramC_TX_DQS(DRAMC_CTX_T *p, U8 u1ByteIdx, S8 iShiftUI)
{
    REG_TRANSFER_T TransferReg[3];

    switch(u1ByteIdx)
    {
        case 0:
            // DQS0
            TransferReg[0].u4Addr = DRAMC_REG_SELPH11;
            TransferReg[0].u4Fld =SELPH11_DLY_DQS0;
            TransferReg[1].u4Addr = DRAMC_REG_SELPH22;
            TransferReg[1].u4Fld =SELPH22_DLY_DQS0_2;
            TransferReg[2].u4Addr = DRAMC_REG_SELPH9;
            TransferReg[2].u4Fld =SELPH9_TXDLY_DQS0;
            ExecuteMoveDramCDelay(p, TransferReg, iShiftUI);

            // DQS_OEN_0
            TransferReg[0].u4Addr = DRAMC_REG_SELPH11;
            TransferReg[0].u4Fld =SELPH11_DLY_OEN_DQS0;
            TransferReg[1].u4Addr = DRAMC_REG_SELPH22;
            TransferReg[1].u4Fld =SELPH22_DLY_OEN_DQS0_2;
            TransferReg[2].u4Addr = DRAMC_REG_SELPH9;
            TransferReg[2].u4Fld =SELPH9_TXDLY_OEN_DQS0;
            ExecuteMoveDramCDelay(p, TransferReg, iShiftUI);
            break;
            
        case 1:
            // DQS1
            TransferReg[0].u4Addr = DRAMC_REG_SELPH11;
            TransferReg[0].u4Fld =SELPH11_DLY_DQS1;
            TransferReg[1].u4Addr = DRAMC_REG_SELPH22;
            TransferReg[1].u4Fld =SELPH22_DLY_DQS1_2;
            TransferReg[2].u4Addr = DRAMC_REG_SELPH9;
            TransferReg[2].u4Fld =SELPH9_TXDLY_DQS1;
            ExecuteMoveDramCDelay(p, TransferReg, iShiftUI);
            
            // DQS_OEN_1
            TransferReg[0].u4Addr = DRAMC_REG_SELPH11;
            TransferReg[0].u4Fld =SELPH11_DLY_OEN_DQS1;
            TransferReg[1].u4Addr = DRAMC_REG_SELPH22;
            TransferReg[1].u4Fld =SELPH22_DLY_OEN_DQS1_2;
            TransferReg[2].u4Addr = DRAMC_REG_SELPH9;
            TransferReg[2].u4Fld =SELPH9_TXDLY_OEN_DQS1;
            ExecuteMoveDramCDelay(p, TransferReg, iShiftUI);
            break;
            
        case 2:
            // DQS2
            TransferReg[0].u4Addr = DRAMC_REG_SELPH11;
            TransferReg[0].u4Fld =SELPH11_DLY_DQS2;
            TransferReg[1].u4Addr = DRAMC_REG_SELPH22;
            TransferReg[1].u4Fld =SELPH22_DLY_DQS2_2;
            TransferReg[2].u4Addr = DRAMC_REG_SELPH9;
            TransferReg[2].u4Fld =SELPH9_TXDLY_DQS2;
            ExecuteMoveDramCDelay(p, TransferReg, iShiftUI);
            
            // DQS_OEN_2
            TransferReg[0].u4Addr = DRAMC_REG_SELPH11;
            TransferReg[0].u4Fld =SELPH11_DLY_OEN_DQS2;
            TransferReg[1].u4Addr = DRAMC_REG_SELPH22;
            TransferReg[1].u4Fld =SELPH22_DLY_OEN_DQS2_2;
            TransferReg[2].u4Addr = DRAMC_REG_SELPH9;
            TransferReg[2].u4Fld =SELPH9_TXDLY_OEN_DQS2;
            ExecuteMoveDramCDelay(p, TransferReg, iShiftUI);
            break;
            
        case 3:
            // DQS3
            TransferReg[0].u4Addr = DRAMC_REG_SELPH11;
            TransferReg[0].u4Fld =SELPH11_DLY_DQS3;
            TransferReg[1].u4Addr = DRAMC_REG_SELPH22;
            TransferReg[1].u4Fld =SELPH22_DLY_DQS3_2;
            TransferReg[2].u4Addr = DRAMC_REG_SELPH9;
            TransferReg[2].u4Fld =SELPH9_TXDLY_DQS3;
            ExecuteMoveDramCDelay(p, TransferReg, iShiftUI);
            
            // DQS_OEN_3
            TransferReg[0].u4Addr = DRAMC_REG_SELPH11;
            TransferReg[0].u4Fld =SELPH11_DLY_OEN_DQS3;
            TransferReg[1].u4Addr = DRAMC_REG_SELPH22;
            TransferReg[1].u4Fld =SELPH22_DLY_OEN_DQS3_2;
            TransferReg[2].u4Addr = DRAMC_REG_SELPH9;
            TransferReg[2].u4Fld =SELPH9_TXDLY_OEN_DQS3;
            ExecuteMoveDramCDelay(p, TransferReg, iShiftUI);
            break;

            default: 
                break;
    }
}
#endif


static void MoveDramC_TX_DQ(DRAMC_CTX_T *p, U8 u1ByteIdx, S8 iShiftUI)
{
    REG_TRANSFER_T TransferReg[3];

    switch(u1ByteIdx)
    {
        case 0:
            // DQM0
            TransferReg[0].u4Addr = DRAMC_REG_SELPH10;
            TransferReg[0].u4Fld =SELPH10_DLY_DQM0;
            TransferReg[1].u4Addr = DRAMC_REG_SELPH22;
            TransferReg[1].u4Fld =SELPH22_DLY_DQM0_2;
            TransferReg[2].u4Addr = DRAMC_REG_SELPH8;
            TransferReg[2].u4Fld =SELPH8_TXDLY_DQM0;
            ExecuteMoveDramCDelay(p, TransferReg, iShiftUI);
            // DQM_OEN_0
            TransferReg[0].u4Addr = DRAMC_REG_SELPH10;
            TransferReg[0].u4Fld =SELPH10_DLY_OEN_DQM0;
            TransferReg[1].u4Addr = DRAMC_REG_SELPH22;
            TransferReg[1].u4Fld =SELPH22_DLY_OEN_DQM0_2;
            TransferReg[2].u4Addr = DRAMC_REG_SELPH8;
            TransferReg[2].u4Fld =SELPH8_TXDLY_OEN_DQM0;
            ExecuteMoveDramCDelay(p, TransferReg, iShiftUI);
            // DQ0
            TransferReg[0].u4Addr = DRAMC_REG_SELPH10;
            TransferReg[0].u4Fld =SELPH10_DLY_DQ0;
            TransferReg[1].u4Addr = DRAMC_REG_SELPH22;
            TransferReg[1].u4Fld =SELPH22_DLY_DQ0_2;
            TransferReg[2].u4Addr = DRAMC_REG_SELPH7;
            TransferReg[2].u4Fld =SELPH7_TXDLY_DQ0;
            ExecuteMoveDramCDelay(p, TransferReg, iShiftUI);
            // DQ_OEN_0
            TransferReg[0].u4Addr = DRAMC_REG_SELPH10;
            TransferReg[0].u4Fld =SELPH10_DLY_OEN_DQ0;
            TransferReg[1].u4Addr = DRAMC_REG_SELPH22;
            TransferReg[1].u4Fld =SELPH22_DLY_OEN_DQ0_2;
            TransferReg[2].u4Addr = DRAMC_REG_SELPH7;
            TransferReg[2].u4Fld =SELPH7_TXDLY_OEN_DQ0;
            ExecuteMoveDramCDelay(p, TransferReg, iShiftUI);
        break;

        case 1:
            // DQM1
            TransferReg[0].u4Addr = DRAMC_REG_SELPH10;
            TransferReg[0].u4Fld =SELPH10_DLY_DQM1;
            TransferReg[1].u4Addr = DRAMC_REG_SELPH22;
            TransferReg[1].u4Fld =SELPH22_DLY_DQM1_2;
            TransferReg[2].u4Addr = DRAMC_REG_SELPH8;
            TransferReg[2].u4Fld =SELPH8_TXDLY_DQM1;
            ExecuteMoveDramCDelay(p, TransferReg, iShiftUI);
            // DQM_OEN_1
            TransferReg[0].u4Addr = DRAMC_REG_SELPH10;
            TransferReg[0].u4Fld =SELPH10_DLY_OEN_DQM1;
            TransferReg[1].u4Addr = DRAMC_REG_SELPH22;
            TransferReg[1].u4Fld =SELPH22_DLY_OEN_DQM1_2;
            TransferReg[2].u4Addr = DRAMC_REG_SELPH8;
            TransferReg[2].u4Fld =SELPH8_TXDLY_OEN_DQM1;
            ExecuteMoveDramCDelay(p, TransferReg, iShiftUI);
            // DQ1
            TransferReg[0].u4Addr = DRAMC_REG_SELPH10;
            TransferReg[0].u4Fld =SELPH10_DLY_DQ1;
            TransferReg[1].u4Addr = DRAMC_REG_SELPH22;
            TransferReg[1].u4Fld =SELPH22_DLY_DQ1_2;
            TransferReg[2].u4Addr = DRAMC_REG_SELPH7;
            TransferReg[2].u4Fld =SELPH7_TXDLY_DQ1;
            ExecuteMoveDramCDelay(p, TransferReg, iShiftUI);    
             // DQ_OEN_1
             TransferReg[0].u4Addr = DRAMC_REG_SELPH10;
             TransferReg[0].u4Fld =SELPH10_DLY_OEN_DQ1;
             TransferReg[1].u4Addr = DRAMC_REG_SELPH22;
             TransferReg[1].u4Fld =SELPH22_DLY_OEN_DQ1_2;
             TransferReg[2].u4Addr = DRAMC_REG_SELPH7;
             TransferReg[2].u4Fld =SELPH7_TXDLY_OEN_DQ1;
             ExecuteMoveDramCDelay(p, TransferReg, iShiftUI);
        break;

        case 2:
                // DQM2
            TransferReg[0].u4Addr = DRAMC_REG_SELPH10;
            TransferReg[0].u4Fld =SELPH10_DLY_DQM2;
            TransferReg[1].u4Addr = DRAMC_REG_SELPH22;
            TransferReg[1].u4Fld =SELPH22_DLY_DQM2_2;
            TransferReg[2].u4Addr = DRAMC_REG_SELPH8;
            TransferReg[2].u4Fld =SELPH8_TXDLY_DQM2;
            ExecuteMoveDramCDelay(p, TransferReg, iShiftUI);
            // DQM_OEN_2
            TransferReg[0].u4Addr = DRAMC_REG_SELPH10;
            TransferReg[0].u4Fld =SELPH10_DLY_OEN_DQM2;
            TransferReg[1].u4Addr = DRAMC_REG_SELPH22;
            TransferReg[1].u4Fld =SELPH22_DLY_OEN_DQM2_2;
            TransferReg[2].u4Addr = DRAMC_REG_SELPH8;
            TransferReg[2].u4Fld =SELPH8_TXDLY_OEN_DQM2;
            ExecuteMoveDramCDelay(p, TransferReg, iShiftUI);
            // DQ2
            TransferReg[0].u4Addr = DRAMC_REG_SELPH10;
            TransferReg[0].u4Fld =SELPH10_DLY_DQ2;
            TransferReg[1].u4Addr = DRAMC_REG_SELPH22;
            TransferReg[1].u4Fld =SELPH22_DLY_DQ2_2;
            TransferReg[2].u4Addr = DRAMC_REG_SELPH7;
            TransferReg[2].u4Fld =SELPH7_TXDLY_DQ2;
            ExecuteMoveDramCDelay(p, TransferReg, iShiftUI);
            // DQ_OEN_2
            TransferReg[0].u4Addr = DRAMC_REG_SELPH10;
            TransferReg[0].u4Fld =SELPH10_DLY_OEN_DQ2;
            TransferReg[1].u4Addr = DRAMC_REG_SELPH22;
            TransferReg[1].u4Fld =SELPH22_DLY_OEN_DQ2_2;
            TransferReg[2].u4Addr = DRAMC_REG_SELPH7;
            TransferReg[2].u4Fld =SELPH7_TXDLY_OEN_DQ2;
            ExecuteMoveDramCDelay(p, TransferReg, iShiftUI);
        break;

        case 3:
                // DQM3
            TransferReg[0].u4Addr = DRAMC_REG_SELPH10;
            TransferReg[0].u4Fld =SELPH10_DLY_DQM3;
            TransferReg[1].u4Addr = DRAMC_REG_SELPH22;
            TransferReg[1].u4Fld =SELPH22_DLY_DQM3_2;
            TransferReg[2].u4Addr = DRAMC_REG_SELPH8;
            TransferReg[2].u4Fld =SELPH8_TXDLY_DQM3;
            ExecuteMoveDramCDelay(p, TransferReg, iShiftUI);
            // DQM_OEN_3
            TransferReg[0].u4Addr = DRAMC_REG_SELPH10;
            TransferReg[0].u4Fld =SELPH10_DLY_OEN_DQM3;
            TransferReg[1].u4Addr = DRAMC_REG_SELPH22;
            TransferReg[1].u4Fld =SELPH22_DLY_OEN_DQM3_2;
            TransferReg[2].u4Addr = DRAMC_REG_SELPH8;
            TransferReg[2].u4Fld =SELPH8_TXDLY_OEN_DQM3;
            ExecuteMoveDramCDelay(p, TransferReg, iShiftUI);
            // DQ3
            TransferReg[0].u4Addr = DRAMC_REG_SELPH10;
            TransferReg[0].u4Fld =SELPH10_DLY_DQ3;
            TransferReg[1].u4Addr = DRAMC_REG_SELPH22;
            TransferReg[1].u4Fld =SELPH22_DLY_DQ3_2;
            TransferReg[2].u4Addr = DRAMC_REG_SELPH7;
            TransferReg[2].u4Fld =SELPH7_TXDLY_DQ3;
            ExecuteMoveDramCDelay(p, TransferReg, iShiftUI);
            // DQ_OEN_3
            TransferReg[0].u4Addr = DRAMC_REG_SELPH10;
            TransferReg[0].u4Fld =SELPH10_DLY_OEN_DQ3;
            TransferReg[1].u4Addr = DRAMC_REG_SELPH22;
            TransferReg[1].u4Fld =SELPH22_DLY_OEN_DQ3_2;
            TransferReg[2].u4Addr = DRAMC_REG_SELPH7;
            TransferReg[2].u4Fld =SELPH7_TXDLY_OEN_DQ3;
            ExecuteMoveDramCDelay(p, TransferReg, iShiftUI);
        break;
    }
}

#if WRITE_LEVELING_MOVE_DQS_INSTEAD_OF_CLK
static void WriteLevelingMoveDQSInsteadOfCLK(DRAMC_CTX_T *p)
{
    U8 u1ByteIdx;
    
    for(u1ByteIdx =0 ; u1ByteIdx<(p->data_width/DQS_BIT_NUMBER); u1ByteIdx++)
    {
        MoveDramC_TX_DQS(p, u1ByteIdx, -WRITE_LEVELING_MOVD_DQS);
        MoveDramC_TX_DQ(p, u1ByteIdx, -WRITE_LEVELING_MOVD_DQS);
    }
}
#endif


DRAM_STATUS_T DramcWriteLeveling(DRAMC_CTX_T *p)
{
// Note that below procedure is based on "ODT off"
    U32 u4value, u4dq_o1, u4dq_o1_tmp[DQS_NUMBER];
    U8 byte_i, ucsample_count;
    S32 ii, ClockDelayMax;
    U8 ucsample_status[DQS_NUMBER], ucdq_o1_perbyte[DQS_NUMBER], ucdq_o1_index[DQS_NUMBER];
    U32 u4prv_register_1dc, u4prv_register_044, u4prv_register_0e4, u4prv_register_13c, u4prv_register_008;
    U32 u4prv_register_phy_01c, u4prv_register_phy_07c, u4prv_register_phy_014, u4prv_register_phy_074;

    S32 wrlevel_dq_delay[DQS_NUMBER]; // 3 is channel number
    S32 wrlevel_dqs_delay[DQS_NUMBER]; // 3 is channel number

    #if WRITE_LEVELING_MOVE_DQS_INSTEAD_OF_CLK
    S32 i4PIBegin, i4PIEnd;
    #endif

    fgwrlevel_done = 0;

    #if CBT_WORKAROUND_O1_SWAP
    if(p->dram_type == TYPE_LPDDR4)
    {
        CATrain_CmdDelay[p->channel] = 0;//u4IO32ReadFldAlign(DRAMC_REG_ADDR(DDRPHY_ARPI_CMD), ARPI_CMD_DA_ARPI_CMD);
        CATrain_CsDelay[p->channel] = 0;//u4IO32ReadFldAlign(DRAMC_REG_ADDR(DDRPHY_ARPI_CMD), ARPI_CMD_DA_ARPI_CS);
        CATrain_ClkDelay[p->channel] = 0;//u4IO32ReadFldAlign(DRAMC_REG_ADDR(DDRPHY_ARPI_CMD), ARPI_CMD_DA_ARPI_CK);
    }
    #endif
    
    // error handling
    if (!p)
    {
        mcSHOW_ERR_MSG(("context is NULL\n"));
        return DRAM_FAIL;
    }

    if ((p->dram_type != TYPE_LPDDR3) && (p->dram_type != TYPE_LPDDR4))
    {
        mcSHOW_ERR_MSG(("Wrong DRAM TYPE. Only support LPDDR3/LPDDR4 in write leveling!!\n"));
        return DRAM_FAIL;
    }
    
    // DQ mapping
    ///!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    /// Note : uiLPDDR_PHY_Mapping_POP, need to take care mapping in real chip, but not in test chip.
    /// Everest : there is bit swap inside single byte. PHY & DRAM is 1-1 byte mapping, no swap.
    for (byte_i=0; byte_i<DQS_NUMBER; byte_i++)
    {
        ucdq_o1_index[byte_i] = byte_i*8;
    }
    
#if REG_ACCESS_PORTING_DGB
    RegLogEnable =1;
    mcSHOW_DBG_MSG(("\n[REG_ACCESS_PORTING_FUNC]   DramcWriteLeveling\n"));
    mcFPRINTF((fp_A60501, "\n[REG_ACCESS_PORTING_FUNC]   DramcWriteLeveling\n"));
#endif

    mcSHOW_DBG_MSG(("\n[DramcWriteLeveling]   Begin\n"));
    mcFPRINTF((fp_A60501, "\n[DramcWriteLeveling]   Begin\n"));

    // backup mode settings
    u4prv_register_1dc = u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_DRAMC_PD_CTRL));
    u4prv_register_044 = u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_TEST2_3));
    u4prv_register_0e4 = u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_PADCTL4));
    u4prv_register_13c = u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_WRITE_LEVELING));
    u4prv_register_008 = u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_CONF2));

   
    //write leveling mode initialization
          																			    	
    //Make CKE fixed at 1 (Don't enter power down, Put this before issuing MRS): CKEFIXON = 1 (0xe4[2])
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_PADCTL4), 1, PADCTL4_CKEFIXON);
    
    //disable auto refresh, REFCNT_FR_CLK = 0 (0x1dc[23:16]), ADVREFEN = 0 (0x44[30]), (CONF2_REFCNT =0)
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DRAMC_PD_CTRL), 0, DRAMC_PD_CTRL_REFCNT_FR_CLK);   
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_TEST2_3), 0, TEST2_3_ADVREFEN);
    vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_CONF2), P_Fld(1, CONF2_REFDIS) |P_Fld(0, CONF2_REFCNT));
        
    //ODT, DQIEN fixed at 1; FIXODT = 1 (0xd8[23]), FIXDQIEN = 1111 (0xd8[15:12])
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_WODT), 1, WODT_WODTFIXOFF);

    //PHY RX Setting for Write Leveling
    //Let IO toO1 path valid, Enable SMT_EN
    O1PathOnOff(p, 1);
    
    // enable DDR write leveling mode:  issue MR2[7] to enable write leveling (refer to DEFAULT MR2 value)
    u4MR2Value |= 0x80;  // OP[7] WR LEV =1
    vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_MRS), P_Fld(u4MR2Value, MRS_MRSOP)|P_Fld(2, MRS_MRSMA));

    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 1, SPCMD_MRWEN);
    mcDELAY_US(1);
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 0, SPCMD_MRWEN);
    //wait tWLDQSEN (25 nCK / 25ns) after enabling write leveling mode (DDR3 / LPDDDR3)
    mcDELAY_US(1);    

    //Set {R_DQS_B3_G R_DQS_B2_G R_DQS_B1_G R_DQS_B0_G}=1010: 0x13c[4:1] (this depends on sel_ph setting)
    //Enable Write leveling: 0x13c[0]
    //vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_WRITE_LEVELING), P_Fld(0xa, WRITE_LEVELING_DQSBX_G)|P_Fld(1, WRITE_LEVELING_WRITE_LEVEL_EN));
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_WRITE_LEVELING), 0xa, WRITE_LEVELING_DQSBX_G);
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_WRITE_LEVELING), 1, WRITE_LEVELING_WRITE_LEVEL_EN);
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_WRITE_LEVELING), 1, WRITE_LEVELING_CBTMASKDQSOE); 
    // select DQS
    if(p->dram_type == TYPE_LPDDR3)
    {
        u4value = 0xf;//select byte 0.1.2.3
    }
    else  //LPDDR4
    {
        u4value = 0x3;//select byte 0.1
    }
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_WRITE_LEVELING), u4value, WRITE_LEVELING_DQS_SEL); 

    // wait tWLMRD (40 nCL / 40 ns) before DQS pulse (DDR3 / LPDDR3)
    mcDELAY_US(1);    

    //Proceed write leveling...
    //Initilize sw parameters
    ClockDelayMax = MAX_TX_DQSDLY_TAPS;
    for (ii=0; ii < (S32)(p->data_width/DQS_BIT_NUMBER); ii++)
    {
        ucsample_status[ii] = 0;
        wrlevel_dqs_final_delay[p->channel][ii] = 0;
    }

    //used for WL done status
    // each bit of sample_cnt represents one-byte WL status
    // 1: done or N/A. 0: NOK
    if ((p->data_width == DATA_WIDTH_16BIT))
    {
        ucsample_count = 0xfc;
    }
    else
    {
        ucsample_count = 0xf0;
    }

    mcSHOW_DBG_MSG(("===============================================================================\n"));    
    mcSHOW_DBG_MSG(("[Write Leveling] Frequency=%d, Channel=%d, Rank=%d\n", p->frequency, p->channel, p->rank));
    mcSHOW_DBG_MSG(("===============================================================================\n"));
    mcSHOW_DBG_MSG(("delay  byte0  byte1  byte2  byte3\n"));
    mcSHOW_DBG_MSG(("-----------------------------\n"));

    mcFPRINTF((fp_A60501, "===============================================================================\n"));
    mcFPRINTF((fp_A60501, "\n        dramc_write_leveling_swcal\n"));
    mcFPRINTF((fp_A60501, "===============================================================================\n"));
    mcFPRINTF((fp_A60501, "delay  byte0  byte1  byte2  byte3\n"));
    mcFPRINTF((fp_A60501, "-----------------------------\n"));

    #if WRITE_LEVELING_MOVE_DQS_INSTEAD_OF_CLK
    if(p->arfgWriteLevelingInitShif[p->channel][p->rank] ==FALSE)
    {
        WriteLevelingMoveDQSInsteadOfCLK(p);
        //p->arfgWriteLevelingInitShif[p->channel][p->rank] =TRUE;
        p->arfgWriteLevelingInitShif[p->channel][RANK_0] =TRUE;
        p->arfgWriteLevelingInitShif[p->channel][RANK_1] =TRUE;
        #if TX_PERBIT_INIT_FLOW_CONTROL
        // both 2 rank use one write leveling result, TX need to udpate.
        p->fgTXPerbifInit[p->channel][RANK_0]= FALSE;
        p->fgTXPerbifInit[p->channel][RANK_1]= FALSE;
        #endif

        mcSHOW_DBG_MSG(("WriteLevelingMoveDQSInsteadOfCLK\n"));
        mcFPRINTF((fp_A60501, "WriteLevelingMoveDQSInsteadOfCLK\n"));
    }
    #endif

    // Set DQS output delay to 0
    #if ENABLE_LP4_SW
    if(p->dram_type == TYPE_LPDDR4)
    {
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_ARPI_DQ), 0, ARPI_DQ_RK0_ARPI_PBYTE_B0);  //rank0, byte0, DQS delay
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_ARPI_DQ), 0, ARPI_DQ_RK0_ARPI_PBYTE_B1);  //rank0, byte1, DQS delay
    }
    else
    #endif
    {
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_ARPI_DQ), 0, ARPI_DQ_RK0_ARPI_PBYTE_B0);  //rank0, byte0, DQS delay
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_ARPI_DQ), 0, ARPI_DQ_RK0_ARPI_PBYTE_B1);  //rank0, byte1, DQS delay
    }
    #if WRITE_LEVELING_MOVE_DQS_INSTEAD_OF_CLK
    i4PIBegin = WRITE_LEVELING_MOVD_DQS*32 -MAX_CLK_PI_DELAY-1;
    i4PIEnd = i4PIBegin + 64;
    #endif

    #if fcFOR_CHIP_ID == fcEverest
    // in Everest, for coner IC of channel A, Byte 0/2 may be around DQS PI=64~74.
    // Therefore, fine tue the scan range to prevent out of boundary.
    if(p->channel == CHANNEL_A)
    {
        // scan from PI = 10 to 84
        i4PIBegin +=10;
        i4PIEnd +=20;
    }
    #endif

    #if  WRITE_LEVELING_MOVE_DQS_INSTEAD_OF_CLK
    for (ii=i4PIBegin; ii<i4PIEnd; ii++)
    #else
    for (ii=(-MAX_CLK_PI_DELAY); ii<=MAX_TX_DQSDLY_TAPS; ii++)
    #endif
    {
        if (ii <= 0)
        {
            // Adjust Clk output delay.
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_ARPI_CMD), -ii, ARPI_CMD_DA_ARPI_CK); 
        }
        else
        {
            // Adjust DQS output delay.
            // PI (TX DQ/DQS adjust at the same time)
            #if ENABLE_LP4_SW
            if(p->dram_type == TYPE_LPDDR4)
            {
                vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_ARPI_DQ), ii, ARPI_DQ_RK0_ARPI_PBYTE_B0);  //rank0, byte0, DQS delay
                vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_ARPI_DQ), ii, ARPI_DQ_RK0_ARPI_PBYTE_B1);  //rank0, byte1, DQS delay
            }
            else
            #endif
            {
                vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_ARPI_DQ), ii, ARPI_DQ_RK0_ARPI_PBYTE_B0);  //rank0, byte0, DQS delay
                vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_ARPI_DQ), ii, ARPI_DQ_RK0_ARPI_PBYTE_B1);  //rank0, byte1, DQS delay
            }
        }
        //Trigger DQS pulse, R_DQS_WLEV: 0x13c[8] from 1 to 0
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_WRITE_LEVELING), 1, WRITE_LEVELING_DQS_WLEV);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_WRITE_LEVELING), 0, WRITE_LEVELING_DQS_WLEV);

        //wait tWLO (7.5ns / 20ns) before output (DDR3 / LPDDR3)
        mcDELAY_US(1);

        //Read DQ_O1 from register
        #if ENABLE_LP4_SW
        if(p->dram_type == TYPE_LPDDR4)
        {
            u4dq_o1 = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DDRPHY_NEW_DQ_O1_RO), DQO1_RO_DQO1_RO);

            #if 0 //A60501 no need 01 byte swap
            if(p->channel ==CHANNEL_A) //O1 byte swap
            {
                u4dq_o1_tmp[0] = u4dq_o1 &0xff;
                u4dq_o1_tmp[1] = (u4dq_o1>>8) &0xff;

                u4dq_o1 = (u4dq_o1_tmp[0] <<8) | u4dq_o1_tmp[1];
            }
            #endif
        }
        else //LPRRR3
        #endif
        {       
            // Get DQ value.
            u4dq_o1_tmp[0] = u4IO32ReadFldAlign(DDRPHY_NEW_DQ_O1_RO +(aru1PhyMap2Channel[0]<< POS_BANK_NUM), NEW_DQ_O1_RO_NEW_DQ_O1); 
            u4dq_o1_tmp[2] = ((u4dq_o1_tmp[0] >>8) & 0xff);
            u4dq_o1_tmp[0] &= 0xff;
            
            u4dq_o1_tmp[1] = u4IO32ReadFldAlign(DDRPHY_NEW_DQ_O1_RO +(aru1PhyMap2Channel[1]<< POS_BANK_NUM), NEW_DQ_O1_RO_NEW_DQ_O1); 
            u4dq_o1_tmp[3] = ((u4dq_o1_tmp[1] >>8) & 0xff);
            u4dq_o1_tmp[1] &= 0xff;

            u4dq_o1 = (u4dq_o1_tmp[0]) | (u4dq_o1_tmp[1] <<8) | (u4dq_o1_tmp[2]  <<16) | (u4dq_o1_tmp[3] <<24);
            //mcSHOW_DBG_MSG2(("DQ_O1: 0x%x\n", u4dq_o1));
         }

        #ifdef ETT_PRINT_FORMAT
        mcSHOW_DBG_MSG(("%d    ", ii));
        #else
        mcSHOW_DBG_MSG(("%2d    ", ii));
        #endif
        mcFPRINTF((fp_A60501, "%2d    ", ii));
        
        //mcSHOW_DBG_MSG(("0x%x    ", u4dq_o1));
        //mcFPRINTF((fp_A60501, "0x%x    ", u4dq_o1));

        for (byte_i = 0; byte_i < (p->data_width/DQS_BIT_NUMBER);  byte_i++)
        {
            ucdq_o1_perbyte[byte_i] = (U8)((u4dq_o1>>ucdq_o1_index[byte_i]) & 0xff);         // ==> TOBEREVIEW

            mcSHOW_DBG_MSG(("%x   ", ucdq_o1_perbyte[byte_i]));
            mcFPRINTF((fp_A60501, "%x    ", ucdq_o1_perbyte[byte_i]));

#if fcFOR_CHIP_ID == fcEverest
            // in Everest, for coner IC of channel A, Byte 0/2 may be around CA PI=64.
            // Therefore, fine tue the scan range to prevent out of boundary.
            if(p->channel == CHANNEL_A)
            {
                if((byte_i==0 || byte_i==2) && (ii<20))
                {
                    // for byte 0/2, get result of DQS 20~84
                    // for byte 1/3, get result of DQS 10~64
                    continue;
                }
            }
#endif

            if ((ucsample_status[byte_i]==0) && (ucdq_o1_perbyte[byte_i]==0))
            {
                ucsample_status[byte_i] = 1;
            }
            else if ((ucsample_status[byte_i]>=1) && (ucdq_o1_perbyte[byte_i] ==0))
            {
                ucsample_status[byte_i] = 1;
            }
            else if ((ucsample_status[byte_i]>=1) && (ucdq_o1_perbyte[byte_i] !=0))
            {
                ucsample_status[byte_i]++;
            }
			//mcSHOW_DBG_MSG(("(%x)	", ucsample_status[byte_i]));

            if((ucsample_count &(0x01 << byte_i))==0)// result not found of byte yet
            {            
                #if  WRITE_LEVELING_MOVE_DQS_INSTEAD_OF_CLK
                if((ucsample_status[byte_i] ==8) || ((ii==i4PIEnd-1) && (ucsample_status[byte_i] >1))) 
                #else
                if((ucsample_status[byte_i] ==8) || ((ii==MAX_TX_DQSDLY_TAPS)&& (ucsample_status[byte_i] >1))) 
                #endif
                {
                    wrlevel_dqs_final_delay[p->channel][byte_i] = ii -ucsample_status[byte_i] +2;
                    ucsample_count |= (0x01 << byte_i);
                    //mcSHOW_DBG_MSG(("(record %d) ", wrlevel_dqs_final_delay[p->channel][byte_i]));
                }
            }
        }
        mcSHOW_DBG_MSG(("\n"));
        mcFPRINTF((fp_A60501, "\n"));      
        
        if (ucsample_count == 0xff)
            break;  // all byte found, early break.
    }

    if (ucsample_count == 0xff)
    {
        // all bytes are done
        fgwrlevel_done= 1;     
        vSetCalibrationResult(p, DRAM_CALIBRATION_WRITE_LEVEL, DRAM_OK);
    }   
    else
    {
        vSetCalibrationResult(p, DRAM_CALIBRATION_WRITE_LEVEL, DRAM_FAIL);
    }

    mcSHOW_DBG_MSG2(("pass bytecount = 0x%x (0xff means all bytes pass) \n\n", ucsample_count));
    mcFPRINTF((fp_A60501, "pass bytecount = 0x%x (0xff means all bytes pass)\n\n", ucsample_count));
    
    for (byte_i = 0; byte_i < (p->data_width/DQS_BIT_NUMBER);  byte_i++)
    {
        if (ClockDelayMax > wrlevel_dqs_final_delay[p->channel][byte_i])
        {
        	ClockDelayMax = wrlevel_dqs_final_delay[p->channel][byte_i];
        }
    }    
    mcSHOW_DBG_MSG(("========================================\n"));
    mcFPRINTF((fp_A60501, "========================================\n"));

    if (ClockDelayMax > 0)
    {
    	ClockDelayMax = 0;
    }
    else
    {
    	ClockDelayMax = -ClockDelayMax;
    }

    mcFPRINTF((fp_A60501, "WL Clk delay = %d,  CA CLK delay = %d\n", ClockDelayMax, CATrain_ClkDelay[p->channel]));    
    mcSHOW_DBG_MSG(("WL Clk delay = %d, CA CLK delay = %d\n", ClockDelayMax, CATrain_ClkDelay[p->channel]));

    // Adjust Clk & CA if needed
    if (CATrain_ClkDelay[p->channel] < ClockDelayMax)
    {
    	S32 Diff = ClockDelayMax - CATrain_ClkDelay[p->channel];
	mcSHOW_DBG_MSG(("CA adjust %d taps... \n", Diff));

    	// Write shift value into CA output delay.  
    	#if ENABLE_LP4_SW
        if(p->dram_type == TYPE_LPDDR4)
        {
            u4value = CATrain_CmdDelay[p->channel];
            u4value += Diff;
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_ARPI_CMD), u4value, ARPI_CMD_DA_ARPI_CMD);
        }
        else
        #endif
        {
            u4value = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DDRPHY_ARPI_CMD), ARPI_CMD_DA_ARPI_CMD);
            u4value += Diff;

            //default value in init() is 0x10 , vIO32Write4B(mcSET_DDRPHY_REG_ADDR_CHC(0x0458), 0x00100000);   // Partickl
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_ARPI_CMD), u4value, ARPI_CMD_DA_ARPI_CMD);
        }
        mcSHOW_DBG_MSG(("[DramcWriteLeveling] Update CA Delay = %d\n", u4value));
        mcFPRINTF((fp_A60501, "[DramcWriteLeveling] Update CA Delay = %d\n", u4value));

        // Write shift value into CS output delay.
        #if ENABLE_LP4_SW
        if(p->dram_type == TYPE_LPDDR4)
        {
            //u4value = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DDRPHY_ARPI_CMD), ARPI_CMD_DA_ARPI_CS);
            u4value = CATrain_CsDelay[p->channel];
            u4value += Diff;
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_ARPI_CMD), u4value, ARPI_CMD_DA_ARPI_CS);
            mcSHOW_DBG_MSG(("[DramcWriteLeveling] Update CS Delay = %d\n", u4value));
            mcFPRINTF((fp_A60501, "[DramcWriteLeveling] Update CS Delay = %d\n", u4value));
        }
        else
        #endif
        {
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_ARPI_CMD), ClockDelayMax, ARPI_CMD_DA_ARPI_CS);
            mcSHOW_DBG_MSG(("[DramcWriteLeveling] Update CS Delay = %d\n", ClockDelayMax));
            mcFPRINTF((fp_A60501, "[DramcWriteLeveling] Update CS Delay = %d\n", ClockDelayMax));
        }
    }
    else
    {
    	mcSHOW_DBG_MSG(("No need to update CA/CS delay because the CLK delay is small than CA training.\n"));
    	ClockDelayMax = CATrain_ClkDelay[p->channel];
    }    

    //DramcEnterSelfRefresh(p, 1);  //enter self refresh mode when changing CLK
    // Write max center value into Clk output delay.
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_ARPI_CMD), ClockDelayMax, ARPI_CMD_DA_ARPI_CK); 
    //DramcEnterSelfRefresh(p, 0); 

    mcFPRINTF((fp_A60501, "Final Clk output delay = %d\n", ClockDelayMax));    
    mcSHOW_DBG_MSG(("Final Clk output delay = %d\n", ClockDelayMax));
    //mcSHOW_DBG_MSG(("After adjustment...\n"));
    
    for (byte_i = 0; byte_i < (p->data_width/DQS_BIT_NUMBER);  byte_i++)
    {
    	wrlevel_dqs_final_delay[p->channel][byte_i] += (ClockDelayMax);
    	#if 0
    	if (wrlevel_dqs_final_delay[p->channel][byte_i] > MAX_TX_DQSDLY_TAPS)
    	{
    		wrlevel_dqs_final_delay[p->channel][byte_i] = MAX_TX_DQSDLY_TAPS;
    	}
    	#endif
    	mcSHOW_DBG_MSG(("DQS%d delay =  %d\n", byte_i, wrlevel_dqs_final_delay[p->channel][byte_i]));
    	mcFPRINTF((fp_A60501, "DQS%d delay =  %d\n", byte_i, wrlevel_dqs_final_delay[p->channel][byte_i]));
    } 

    // write leveling done, mode settings recovery if necessary
    // recover mode registers : issue MR2[7] to disable write leveling (refer to DEFAULT MR2 value)
    u4MR2Value &= 0x7f;  // OP[7] WR LEV =0
    vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_MRS), P_Fld(u4MR2Value, MRS_MRSOP)|P_Fld(2, MRS_MRSMA));
  
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 1, SPCMD_MRWEN);
    mcDELAY_US(1);
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 0, SPCMD_MRWEN);

    // restore registers.
    vIO32Write4B(DRAMC_REG_ADDR(DRAMC_REG_DRAMC_PD_CTRL), u4prv_register_1dc);
    vIO32Write4B(DRAMC_REG_ADDR(DRAMC_REG_TEST2_3), u4prv_register_044);
    vIO32Write4B(DRAMC_REG_ADDR(DRAMC_REG_PADCTL4), u4prv_register_0e4);
    vIO32Write4B(DRAMC_REG_ADDR(DRAMC_REG_WRITE_LEVELING), u4prv_register_13c);
    vIO32Write4B(DRAMC_REG_ADDR(DRAMC_REG_CONF2), u4prv_register_008);

    //Disable DQ_O1, SELO1ASO=0 for power saving
    O1PathOnOff(p, 0);

    #if ENABLE_LP4_SW
    if(p->dram_type == TYPE_LPDDR4)
    { 
        #if REG_SHUFFLE_REG_CHECK
        ShuffleRegCheck =1;
        #endif
        // set to best values for  DQS, DQM, DQ
        vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_ARPI_DQ), \
                                    P_Fld((U32) wrlevel_dqs_final_delay[p->channel][0], ARPI_DQ_RK0_ARPI_PBYTE_B0)| \
                                    P_Fld((U32) wrlevel_dqs_final_delay[p->channel][1], ARPI_DQ_RK0_ARPI_PBYTE_B1));

        if(p->rank == RANK_1)
        {
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_ARPI_DQ2), \
                                P_Fld((U32) wrlevel_dqs_final_delay[p->channel][0], ARPI_DQ_RK0_ARPI_PBYTE_B0)| \
                                P_Fld((U32) wrlevel_dqs_final_delay[p->channel][1], ARPI_DQ_RK0_ARPI_PBYTE_B1));
        }
        #if REG_SHUFFLE_REG_CHECK
        ShuffleRegCheck =0;
        #endif
    }
    else //LPDDR3
    #endif
    { 
        #if REG_SHUFFLE_REG_CHECK
        ShuffleRegCheck =1;
        #endif

        
#if 0
        // set to best values for  DQS
        vIO32WriteFldMulti(DDRPHY_ARPI_DQ+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), \
                                    P_Fld((U32) wrlevel_dqs_final_delay[p->channel][0], ARPI_DQ_RK0_ARPI_PBYTE_B0)| \
                                    P_Fld((U32) wrlevel_dqs_final_delay[p->channel][2], ARPI_DQ_RK0_ARPI_PBYTE_B1));

        
        vIO32WriteFldMulti(DDRPHY_ARPI_DQ+(aru1PhyMap2Channel[1]<< POS_BANK_NUM),  \
                                    P_Fld((U32) wrlevel_dqs_final_delay[p->channel][1], ARPI_DQ_RK0_ARPI_PBYTE_B0)| \
                                    P_Fld((U32) wrlevel_dqs_final_delay[p->channel][3], ARPI_DQ_RK0_ARPI_PBYTE_B1));

        if(p->rank == RANK_1)
        {
            vIO32WriteFldMulti(DDRPHY_ARPI_DQ2+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), \
                                        P_Fld((U32) wrlevel_dqs_final_delay[p->channel][0], ARPI_DQ_RK0_ARPI_PBYTE_B0)| \
                                        P_Fld((U32) wrlevel_dqs_final_delay[p->channel][2], ARPI_DQ_RK0_ARPI_PBYTE_B1));
            
            
            vIO32WriteFldMulti(DDRPHY_ARPI_DQ2+(aru1PhyMap2Channel[1]<< POS_BANK_NUM),  \
                                        P_Fld((U32) wrlevel_dqs_final_delay[p->channel][1], ARPI_DQ_RK0_ARPI_PBYTE_B0)| \
                                        P_Fld((U32) wrlevel_dqs_final_delay[p->channel][3], ARPI_DQ_RK0_ARPI_PBYTE_B1));
        }

#else
    for(byte_i=0; byte_i<DQS_NUMBER; byte_i++)
    {
        if(wrlevel_dqs_final_delay [p->channel][byte_i] >= 0x40) //ARPI_PBYTE_B* is 6 bits, max 0x40
        {
            wrlevel_dqs_delay[byte_i] = wrlevel_dqs_final_delay [p->channel][byte_i] - 0x40;
            MoveDramC_TX_DQS(p, byte_i, 2);
        }
        else
            wrlevel_dqs_delay[byte_i] = wrlevel_dqs_final_delay [p->channel][byte_i];
    }

    
    // set to best values for  DQS
    vIO32WriteFldMulti(DDRPHY_ARPI_DQ+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), \
                                P_Fld((U32) wrlevel_dqs_delay[0], ARPI_DQ_RK0_ARPI_PBYTE_B0)| \
                                P_Fld((U32) wrlevel_dqs_delay[2], ARPI_DQ_RK0_ARPI_PBYTE_B1));
    
    
    vIO32WriteFldMulti(DDRPHY_ARPI_DQ+(aru1PhyMap2Channel[1]<< POS_BANK_NUM),  \
                                P_Fld((U32) wrlevel_dqs_delay[1], ARPI_DQ_RK0_ARPI_PBYTE_B0)| \
                                P_Fld((U32) wrlevel_dqs_delay[3], ARPI_DQ_RK0_ARPI_PBYTE_B1));
    
    if(p->rank == RANK_1)
    {
        vIO32WriteFldMulti(DDRPHY_ARPI_DQ2+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), \
                                    P_Fld((U32) wrlevel_dqs_delay[0], ARPI_DQ_RK0_ARPI_PBYTE_B0)| \
                                    P_Fld((U32) wrlevel_dqs_delay[2], ARPI_DQ_RK0_ARPI_PBYTE_B1));
        
        
        vIO32WriteFldMulti(DDRPHY_ARPI_DQ2+(aru1PhyMap2Channel[1]<< POS_BANK_NUM),  \
                                    P_Fld((U32) wrlevel_dqs_delay[1], ARPI_DQ_RK0_ARPI_PBYTE_B0)| \
                                    P_Fld((U32) wrlevel_dqs_delay[3], ARPI_DQ_RK0_ARPI_PBYTE_B1));
    }
#endif

        #if EVEREST_CHANGE_OF_PHY_PBYTE
        //Evereest new change, ARPI_DQ_RK0_ARPI_PBYTE_B* only move DQS, not including of DQM&DQ anymore. 
        //Add move DQ, DQ= DQS+0x10, after cali.  take care diff. UI. with DQS
        for(byte_i=0; byte_i<DQS_NUMBER; byte_i++)
        {
            wrlevel_dq_delay[byte_i] = wrlevel_dqs_final_delay [p->channel][byte_i] + 0x10;
            if(wrlevel_dq_delay[byte_i] >= 0x40) //ARPI_DQ_B* is 6 bits, max 0x40
            {
                wrlevel_dq_delay[byte_i] -= 0x40;
                MoveDramC_TX_DQ(p, byte_i, 2);
            }
        }
        
        // set to best values for  DQ/DQM/DQ_OEN/DQM_OEN
        vIO32WriteFldMulti(DDRPHY_ARPI_DQ+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), \
                                    P_Fld((U32) wrlevel_dq_delay[0], ARPI_DQ_RK0_ARPI_DQ_B0)| \
                                    P_Fld((U32) wrlevel_dq_delay[2], ARPI_DQ_RK0_ARPI_DQ_B1));

        
        vIO32WriteFldMulti(DDRPHY_ARPI_DQ+(aru1PhyMap2Channel[1]<< POS_BANK_NUM),  \
                                    P_Fld((U32) wrlevel_dq_delay[1], ARPI_DQ_RK0_ARPI_DQ_B0)| \
                                    P_Fld((U32) wrlevel_dq_delay[3], ARPI_DQ_RK0_ARPI_DQ_B1));

        if(p->rank == RANK_1)
        {
            vIO32WriteFldMulti(DDRPHY_ARPI_DQ2+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), \
                                        P_Fld((U32) wrlevel_dq_delay[0], ARPI_DQ_RK0_ARPI_DQ_B0)| \
                                        P_Fld((U32) wrlevel_dq_delay[2], ARPI_DQ_RK0_ARPI_DQ_B1));
            
            
            vIO32WriteFldMulti(DDRPHY_ARPI_DQ2+(aru1PhyMap2Channel[1]<< POS_BANK_NUM),  \
                                        P_Fld((U32) wrlevel_dq_delay[1], ARPI_DQ_RK0_ARPI_DQ_B0)| \
                                        P_Fld((U32) wrlevel_dq_delay[3], ARPI_DQ_RK0_ARPI_DQ_B1));
        }
        #endif
        
        #if REG_SHUFFLE_REG_CHECK
        ShuffleRegCheck =0;
        #endif
    }

//A60501 can work without dramC and PHY reset , remove this reset.
#if 0  // SW work around : Dramc need sw reset after enable WRITE_LEVELING_WRITE_LEVEL_EN.
    // Need to check if it OK for LP4?
   // if(p->dram_type == TYPE_LPDDR3)
    {
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_PADCTL4), 1, PADCTL4_DMSW_RST); 
        mcDELAY_US(1);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_PADCTL4), 0, PADCTL4_DMSW_RST); 

        DramPhyReset(p);
    }
#endif

    //After write leveling, since we only let rank0 enter write leveling mode, rank1 may failed due to unexpected CLK jitter¡K
    //So we reset DRAM and re-issue all MR's (@ DramcInit) after write leveling¡K
    Dram_Reset(p);

#if REG_ACCESS_PORTING_DGB
    RegLogEnable =0;
#endif
    mcSHOW_DBG_MSG(("[DramcWriteLeveling] ====Done====\n\n"));
    mcFPRINTF((fp_A60501, "[DramcWriteLeveling] ====Done====\n\n"));    

    return DRAM_OK;
    // log example
/*
===============================================================================
        dramc_write_leveling_swcal
===============================================================================
delay  byte0  byte1  byte2  byte3
-----------------------------
WriteLevelingMoveDQSInsteadOfCLK
33    0    0    0    0    
34    0    0    0    0    
35    0    0    0    0    
36    0    0    0    0    
37    ff    0    0    0    
38    0    0    0    0    
39    0    0    0    0    
40    ff    0    0    ff    
41    ff    0    0    ff    
42    ff    0    0    ff    
43    ff    0    0    ff    
44    ff    ff    0    ff    
45    ff    ff    ff    ff    
46    ff    ff    ff    ff    
47    ff    ff    ff    ff    
48    ff    ff    ff    ff    
49    ff    ff    ff    ff    
50    ff    ff    ff    ff    
51    ff    ff    ff    ff    
52    ff    ff    ff    ff    
53    ff    ff    ff    ff    
54    ff    ff    ff    ff    
55    ff    ff    ff    ff    
56    ff    ff    ff    ff    
57    ff    ff    ff    ff    
58    ff    ff    ff    ff    
59    ff    ff    ff    ff    
60    ff    ff    ff    ff    
61    ff    ff    ff    ff    
62    ff    ff    ff    ff    
63    ff    ff    ff    ff    
64    ff    ff    ff    ff    
65    ff    ff    ff    ff    
66    ff    ff    ff    ff    
67    ff    ff    ff    ff    
68    ff    ff    ff    ff    
69    ff    ff    ff    ff    
70    ff    ff    ff    0    
71    0    ff    ff    0    
72    0    ff    ff    ff    
73    0    ff    ff    ff    
74    0    ff    ff    0    
75    0    0    ff    0    
76    0    0    ff    0    
77    0    0    0    0    
78    0    0    0    0    
79    0    0    0    0    
80    0    0    0    0    
81    0    0    0    0    
82    0    0    0    0    
83    0    0    0    0    
84    0    0    0    0    
85    0    0    0    0    
86    0    0    0    0    
87    0    0    0    0    
88    0    0    0    0    
89    0    0    0    0    
90    0    0    0    0    
91    0    0    0    0    
92    0    0    0    0    
93    0    0    0    0    
94    0    0    0    0    
95    0    0    0    0    
96    0    0    0    0    
pass bytecount = 0xff (0xff means all bytes pass)

byte_i    status    best delay
   0       1      40
   1       1      44
   2       1      45
   3       1      40
========================================
WL Clk delay = 0,  CA CLK delay = 0
Final Clk output delay = 0
DQS0 output delay =  40
DQS1 output delay =  44
DQS2 output delay =  45
DQS3 output delay =  40
[DramcWriteLeveling] ====Done====

*/
}


#define GATING_DQS_COUNTER_BYTE3_FAIL_WORKAROUND 0  //The design will be fixed in Everest, disable work around
#define GATING_RODT_LATANCY_EN 0  //LP3 RODT is not enable, don't need to set the RODT settings.

#define GATING_PATTERN_NUM_LP4 0x23
#define GATING_GOLDEND_DQSCNT_LP4 0x4646

#define GATING_PATTERN_NUM_LP3 0x46
#define GATING_GOLDEND_DQSCNT_LP3 0x2323

#define GATING_TXDLY_CHNAGE  1 //Gating txdly chcange & RANKINCTL setting

#if SW_CHANGE_FOR_SIMULATION
#define ucRX_DLY_DQSIENSTB_LOOP 32
#define ucRX_DQS_CTL_LOOP 8
#endif

#if GATING_ADJUST_TXDLY_FOR_TRACKING
U8 u1TXDLY_Cal_min =0xff, u1TXDLY_Cal_max=0;
U8 ucbest_coarse_tune2T_backup[RANK_MAX][DQS_NUMBER];
U8 ucbest_coarse_tune0p5T_backup[RANK_MAX][DQS_NUMBER];
U8 ucbest_coarse_tune2T_P1_backup[RANK_MAX][DQS_NUMBER];
U8 ucbest_coarse_tune0p5T_P1_backup[RANK_MAX][DQS_NUMBER];
#endif

DRAM_STATUS_T DramcRxdqsGatingCal(DRAMC_CTX_T *p)
{
        #if !SW_CHANGE_FOR_SIMULATION
        U8 ucRX_DLY_DQSIENSTB_LOOP,ucRX_DQS_CTL_LOOP;
        #endif
        U32 u4value, u4all_result_R, u4all_result_F, u4err_value;
        U8 ucpass_begin[DQS_NUMBER], ucpass_count[DQS_NUMBER], ucCurrentPass;
        U8 ucmin_coarse_tune2T[DQS_NUMBER], ucmin_coarse_tune0p5T[DQS_NUMBER], ucmin_fine_tune[DQS_NUMBER];
        U8 ucpass_count_1[DQS_NUMBER], ucmin_coarse_tune2T_1[DQS_NUMBER], ucmin_coarse_tune0p5T_1[DQS_NUMBER], ucmin_fine_tune_1[DQS_NUMBER];
        U8 dqs_i,  ucdly_coarse_large, ucdly_coarse_0p5T, ucdly_fine_xT;
        U8 ucdqs_result_R, ucdqs_result_F, uctmp_offset, uctmp_value;
        U8 ucbest_fine_tune[DQS_NUMBER], ucbest_coarse_tune0p5T[DQS_NUMBER], ucbest_coarse_tune2T[DQS_NUMBER];
        U8 ucbest_fine_tune_P1[DQS_NUMBER], ucbest_coarse_tune0p5T_P1[DQS_NUMBER], ucbest_coarse_tune2T_P1[DQS_NUMBER];
    
        U8 ucFreqDiv;
        U8 ucdly_coarse_large_P1, ucdly_coarse_0p5T_P1;
        
    #if GATING_ADJUST_TXDLY_FOR_TRACKING
        U8 u1TX_dly_DQSgated;
    #endif
    #if GATING_TXDLY_CHNAGE
        U32 u4ReadDQSINCTL, u4ReadTXDLY, u4RankINCTL_ROOT, u4Tmp2T, u4Tmp0p5T;
    #endif
    #if GATING_RODT_LATANCY_EN
        U8 ucdly_coarse_large_RODT, ucdly_coarse_0p5T_RODT;
        U8 ucbest_coarse_large_RODT, ucbest_coarse_0p5T_RODT;
    #endif
        U8 ucCoarseTune, ucCoarseStart, ucCoarseEnd;
        U32 LP3_DataPerByte[DQS_NUMBER];
        U32 u4DebugCnt[DQS_NUMBER];
        U16 u2DebugCntPerByte;
    
        U32 u4BakReg_DRAMC_DQSCAL0, u4BakReg_DRAMC_STBCAL_F;
        U32 u4BakReg_DRAMC_WODT, u4BakReg_DRAMC_SPCMD;
    
        U8 u1PassByteCount=0;
    
#if ENABLE_CLAIBTAION_WINDOW_LOG_FOR_FT
        U16 u2MinWinSize = 0xffff;
        U8 u1MinWinSizeByteidx;
#endif
        // error handling
        if (!p)
        {
            mcSHOW_ERR_MSG(("context is NULL\n"));
            return DRAM_FAIL;
        }
    
        //Register backup 
        u4BakReg_DRAMC_DQSCAL0 = u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_DQSCAL0));
        u4BakReg_DRAMC_STBCAL_F = u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_STBCAL_F));
        u4BakReg_DRAMC_WODT = u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_WODT));
        u4BakReg_DRAMC_SPCMD = u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_SPCMD));

        //Justin: DQ_REV_B*[5] =1
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_TXDQ3),  1, TXDQ8_RG_TX_ARDQ_OE_DIS_B1);
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ13),  1, TXDQ8_RG_TX_ARDQ_OE_DIS_B1);  
     
        // Disable HW gating first, 0x1c0[31]
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DQSCAL0), 0, DQSCAL0_STBCALEN);
    
        //If DQS ring counter is different as our expectation, error flag is asserted and the status is in ddrphycfg 0xFC0 ~ 0xFCC
        //Enable this function by R_DMSTBENCMPEN=1 (0x348[18])
        //Set R_DMSTBCNT_LATCH_EN=1, 0x348[11]
        //Set R_DM4TO1MODE=0, 0x54[11]
        //Clear error flag by ddrphycfg 0x5c0[1] R_DMPHYRST
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_STBCAL_F), 1, STBCAL_F_STBENCMPEN);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_STBCAL_F), 1, STBCAL_F_STBCNT_LATCH_EN);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_WODT), 0, WODT_DM4TO1MODE);
    
        //enable &reset DQS counter
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 1, SPCMD_DQSGCNTEN); 
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 1, SPCMD_DQSGCNTRST);
        mcDELAY_US(1);//delay 2T
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 0, SPCMD_DQSGCNTRST);
    
        //Initialize variables
        for (dqs_i=0; dqs_i<DQS_NUMBER; dqs_i++)
        {
            ucpass_begin[dqs_i] = 0;
            ucpass_count[dqs_i] = 0;
        }

        #if !SW_CHANGE_FOR_SIMULATION
        ucRX_DLY_DQSIENSTB_LOOP= 32;// PI fine tune 0->31
        #endif
    
        mcSHOW_DBG_MSG2(("===============================================================================\n"));        
        mcSHOW_DBG_MSG(("[Gating] Frequency=%d, Channel=%d, Rank=%d\n", p->frequency, p->channel, p->rank));
        mcSHOW_DBG_MSG2(("x = dqs result \ny = coarse_2T  coarse_0.5T  finetune\n"));
        mcSHOW_DBG_MSG2(("-------------------------------------------------------------------------------\n")); 
        mcSHOW_DBG_MSG2(("y  |  dqs0f   dqs0r   dqs1f   dqs1r   dqs2f   dqs2r   dqs3f   dqs3r\n"));
        mcSHOW_DBG_MSG2(("-------------------------------------------------------------------------------\n"));

    #if ENABLE_LP4_SW
        if(p->dram_type == TYPE_LPDDR4)
        {
            ucFreqDiv= 4;
    
            if(p->frequency >= 1600)
            {
            #if SUPPORT_LP4_DBI
                if(u1ReadDBIEnable)
                {
                    //for 1600MHz, DBI
                    ucCoarseStart = 28;
                    ucCoarseEnd = 48;
                }
                else
            #endif
                {
                    ucCoarseStart = 21;
                    ucCoarseEnd = 32;
                }
            }
            else
            {
                ucCoarseStart = DQS_GW_COARSE_START_LP4;
                ucCoarseEnd = DQS_GW_COARSE_END_LP4;
            }
        }
        else //LPDDR3
        #endif
        {
            ucFreqDiv= 2;
      
            if(p->frequency >= 800)     //1600
                ucCoarseStart = 12;
            else if(p->frequency >= 600)  //1270
                ucCoarseStart = 7;
            else if(p->frequency >= 533)  //1066
                ucCoarseStart = 6;
            else // 800
                ucCoarseStart = 9;
            
            ucCoarseEnd = DQS_GW_COARSE_END_LP3;
        }

#if !SW_CHANGE_FOR_SIMULATION
        #if fcFOR_CHIP_ID == fcA60501
        ucRX_DQS_CTL_LOOP = ucFreqDiv*2;
        #else//GATING_TXDLY_CHNAGE  //Everest new change
        ucRX_DQS_CTL_LOOP = 8; // Since Everest, no matter LP3 or LP4. ucRX_DQS_CTL_LOOP is 8.
        #endif
#endif
    
    #if GATING_RODT_LATANCY_EN  //LP3 RODT is not enable, don't need to set the RODT settings.
        // 1.   DQSG latency = 
        // (1)   R_DMR*DQSINCTL[3:0] (MCK) + 
        // (2)   selph_TX_DLY[2:0] (MCK) + 
        // (3)   selph_dly[2:0] (UI)
    
        // 2.   RODT latency = 
        // (1)   R_DMTRODT[3:0] (MCK) + 
        // (2)   selph_TX_DLY[2:0] (MCK) + 
        // (3)   selph_dly[2:0] (UI) 

        if(p->dram_type == TYPE_LPDDR4)
        {
            //R_DMTRODT[3:0] (MCK)  = R_DMR*DQSINCTL[3:0] (MCK)
            if(p->rank == RANK_0)
            {   
                u4value = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DQSCTL1), DQSCTL1_DQSINCTL);
                vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DDR2CTL), u4value & 0x7, DDR2CTL_RODT);//R_DMTRODT[2:0]
                vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DDR2CTL), (u4value >>3), DDR2CTL_RODT3);//R_DMTRODT[3]
            }
            #if 0 //R1DQSINCTL and R2DQSINCTL is useless on A-PHY. Only need to set RODT once. 
            else //rank1
            {
                u4value = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DQSCTL2), DQSCTL2_R1DQSINCTL);
                vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DDR2CTL), u4value & 0x7, DDR2CTL_RODT);//R_DMTRODT[2:0]
                vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DDR2CTL), (u4value >>3), DDR2CTL_RODT3);//R_DMTRODT[3]
            }
            #endif
        }
    #endif
    
        for (ucCoarseTune = ucCoarseStart; ucCoarseTune < ucCoarseEnd; ucCoarseTune += DQS_GW_COARSE_STEP)
        {
            ucdly_coarse_large      = ucCoarseTune / ucRX_DQS_CTL_LOOP;
            ucdly_coarse_0p5T      = ucCoarseTune % ucRX_DQS_CTL_LOOP;
            
            ucdly_coarse_large_P1 = (ucCoarseTune + ucFreqDiv) / ucRX_DQS_CTL_LOOP;
            ucdly_coarse_0p5T_P1 =(ucCoarseTune + ucFreqDiv) % ucRX_DQS_CTL_LOOP;
    
        #if GATING_RODT_LATANCY_EN  //LP3 RODT is not enable, don't need to set the RODT settings.
            if(p->dram_type == TYPE_LPDDR4)
            {
                ucdly_coarse_large_RODT     = ucdly_coarse_large-1;
                ucdly_coarse_0p5T_RODT     = ucdly_coarse_0p5T;
            }
        #endif
            
            DramPhyCGReset(p, 1);// need to reset when UI update or PI change >=2
    
            // 4T or 2T coarse tune
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH2), ucdly_coarse_large, SELPH2_TXDLY_DQSGATE);  //DQS0_P0
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH12), \
                                        P_Fld((U32) ucdly_coarse_large, SELPH12_TX_DLY_DQS1_GATED)| \
                                        P_Fld((U32) ucdly_coarse_large, SELPH12_TX_DLY_DQS2_GATED)| \
                                        P_Fld((U32) ucdly_coarse_large, SELPH12_TX_DLY_DQS3_GATED));
    
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH2), ucdly_coarse_large_P1, SELPH2_TXDLY_DQSGATE_P1);//DQS0_P1
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH12), \
                                        P_Fld((U32) ucdly_coarse_large_P1, SELPH12_TX_DLY_DQS1_GATED_P1)| \
                                        P_Fld((U32) ucdly_coarse_large_P1, SELPH12_TX_DLY_DQS2_GATED_P1)| \
                                        P_Fld((U32) ucdly_coarse_large_P1, SELPH12_TX_DLY_DQS3_GATED_P1));
    
            // 0.5T coarse tune
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH5), (ucdly_coarse_0p5T & 0x3), SELPH5_DLY_DQSGATE); 
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH21), (ucdly_coarse_0p5T>>2), SELPH21_DLY_DQSGATE_2); 
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH13), \
                                        P_Fld((U32) ucdly_coarse_0p5T, SELPH13_REG_DLY_DQS1_GATED)| \
                                        P_Fld((U32) ucdly_coarse_0p5T, SELPH13_REG_DLY_DQS2_GATED)| \
                                        P_Fld((U32) ucdly_coarse_0p5T, SELPH13_REG_DLY_DQS3_GATED));
    
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH5), (ucdly_coarse_0p5T_P1 & 0x3), SELPH5_DLY_DQSGATE_P1);
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH21), (ucdly_coarse_0p5T_P1>>2), SELPH21_DLY_DQSGATE_P1_2); 
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH13), \
                                        P_Fld((U32) ucdly_coarse_0p5T_P1, SELPH13_REG_DLY_DQS1_GATED_P1)| \
                                        P_Fld((U32) ucdly_coarse_0p5T_P1, SELPH13_REG_DLY_DQS2_GATED_P1)| \
                                        P_Fld((U32) ucdly_coarse_0p5T_P1, SELPH13_REG_DLY_DQS3_GATED_P1));
    
    
        #if GATING_RODT_LATANCY_EN  //LP3 RODT is not enable, don't need to set the RODT settings.
            if(p->dram_type == TYPE_LPDDR4)
            {
                vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH6_1), ucdly_coarse_large_RODT, SELPH6_1_TXDLY_RODTEN);  
                vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH6_1), ucdly_coarse_0p5T_RODT, SELPH6_1_DLY_RODTEN); 
                //mcSHOW_DBG_MSG2(("RODT delay(2T, 0.5T) = (%d, %d)\n", ucdly_coarse_large_RODT, ucdly_coarse_0p5T_RODT));
                //mcFPRINTF((fp_A60501,"RODT delay(2T, 0.5T) = (%d, %d)\n", ucdly_coarse_large_RODT, ucdly_coarse_0p5T_RODT));
            }
        #endif
    
            DramPhyCGReset(p, 0);  
            mcDELAY_US(1);//delay 2T
    
            for (ucdly_fine_xT=DQS_GW_FINE_START; ucdly_fine_xT<DQS_GW_FINE_END; ucdly_fine_xT+=DQS_GW_FINE_STEP)
            {
                DramPhyCGReset(p, 1);  
                u4value = ucdly_fine_xT | (ucdly_fine_xT<<8) | (ucdly_fine_xT<<16) | (ucdly_fine_xT<<24);
                vIO32Write4B(DRAMC_REG_ADDR(DRAMC_REG_R0DQSIEN), u4value);  
    
                //ok we set a coarse/fine tune value already
                //reset the read counters in both DRAMC and DDRPHY (R_DMPHYRST: 0x0f0[28])
                //enable test engine
                //record the counter value
    
                DramPhyCGReset(p, 0);  
                mcDELAY_US(1);//delay 2T
                
                //reset phy, reset read data counter 
                DramPhyReset(p);  

                //reset DQS counter
                vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 1, SPCMD_DQSGCNTRST);
                mcDELAY_US(1);//delay 2T
                vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 0, SPCMD_DQSGCNTRST);

                #if ENABLE_LP4_SW
                if(p->dram_type == TYPE_LPDDR4)
                {
                    // enable TE2, audio pattern
                    DramcEngine2(p, TE_OP_READ_CHECK, 0x55000000, 0xaa000000 |GATING_PATTERN_NUM_LP4, 1, 0, 0, 0);
                    //#ifdef A60501_BUILD_ERROR_TODO
                    u4all_result_R = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DDRPHY_STBERR_RK0_R), STBERR_RK0_R_STBERR_RK0_R);
                    u4all_result_F = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DDRPHY_STBERR_RK0_F), STBERR_RK0_F_STBERR_RK0_F);              
                    //#endif
               }
                else //LPDDR3
                #endif
                {
                    // enable TE2, audio pattern
                    DramcEngine2(p, TE_OP_READ_CHECK, 0x55000000, 0xaa000000 |GATING_PATTERN_NUM_LP3, 1, 0, 0, 0);
                #if fcFOR_CHIP_ID == fcA60501
                    ///TODO: NOTICE , DramC and PHY byte 1 and 2 inverse work around.       
                    LP3_DataPerByte[0] = (u4IO32ReadFldAlign(DDRPHY_STBERR_RK0_R+(1<< POS_BANK_NUM), STBERR_RK0_R_STBERR_RK0_R) >> 8) & 0xff;
                    LP3_DataPerByte[2] =  u4IO32ReadFldAlign(DDRPHY_STBERR_RK0_R+(1<< POS_BANK_NUM), STBERR_RK0_R_STBERR_RK0_R) & 0xff;
                    LP3_DataPerByte[1] = (u4IO32ReadFldAlign(DDRPHY_STBERR_RK0_R+(2<< POS_BANK_NUM), STBERR_RK0_R_STBERR_RK0_R) >> 8) & 0xff;
                    LP3_DataPerByte[3] = (u4IO32ReadFldAlign(DDRPHY_STBERR_RK0_R, STBERR_RK0_R_STBERR_RK0_R) >> 8) & 0xff;
                    u4all_result_R = LP3_DataPerByte[0] | (LP3_DataPerByte[1] <<8) |(LP3_DataPerByte[2] <<16) |(LP3_DataPerByte[3] <<24);
    
                    LP3_DataPerByte[0] = (u4IO32ReadFldAlign(DDRPHY_STBERR_RK0_F+(1<< POS_BANK_NUM), STBERR_RK0_F_STBERR_RK0_F) >> 8) & 0xff;
                    LP3_DataPerByte[2] =  u4IO32ReadFldAlign(DDRPHY_STBERR_RK0_F+(1<< POS_BANK_NUM), STBERR_RK0_F_STBERR_RK0_F) & 0xff;
                    LP3_DataPerByte[1] = (u4IO32ReadFldAlign(DDRPHY_STBERR_RK0_F+(2<< POS_BANK_NUM), STBERR_RK0_F_STBERR_RK0_F) >> 8) & 0xff;
                    LP3_DataPerByte[3] = (u4IO32ReadFldAlign(DDRPHY_STBERR_RK0_F, STBERR_RK0_F_STBERR_RK0_F) >> 8) & 0xff;
                    u4all_result_F = LP3_DataPerByte[0] | (LP3_DataPerByte[1] <<8) |(LP3_DataPerByte[2] <<16) |(LP3_DataPerByte[3] <<24);
                #else //Everest
                    // rising
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
                #endif
                }
                
                // ring counter for debug
                #if ENABLE_LP4_SW
                if(p->dram_type == TYPE_LPDDR4)
                {
                    //read DQS counter
                    u4DebugCnt[0]= u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_DQSGNWCNT0));
                    u4DebugCnt[1] = (u4DebugCnt[0] >> 16) & 0xffff;
                    u4DebugCnt[0] &= 0xffff;
                    #if 0
                    u4DebugCnt[2] = u4IO32Read4B(DRAMC_REG_ADDR(DDRPHY_PHY_RO_3));
                    u4DebugCnt[3] = u4IO32Read4B(DRAMC_REG_ADDR(DDRPHY_PHY_RO_4));
                    #endif
                }else//LPDDR3
                #endif
                {
                     //read DQS counter
                    u4DebugCnt[0] = u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_DQSGNWCNT0));
                    u4DebugCnt[1] = (u4DebugCnt[0] >> 16) & 0xffff;
                    u4DebugCnt[0] &= 0xffff;
                        
                    u4DebugCnt[2] = u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_DQSGNWCNT1));
                    u4DebugCnt[3] = (u4DebugCnt[2] >> 16) & 0xffff;
                    u4DebugCnt[2] &= 0xffff;
    
                #if 0
                    u4DebugCnt2 = u4IO32Read4B((DDRPHY_STBENSTATUS1));
                    u4DebugCnt3 = u4IO32Read4B((DDRPHY_STBENSTATUS2));
                    mcSHOW_ERR_MSG(("CHA (%x, %x) ", u4DebugCnt2,u4DebugCnt3));
    
                    u4DebugCnt2 = u4IO32Read4B((DDRPHY_STBENSTATUS1) + (1<< POS_BANK_NUM));
                    u4DebugCnt3 = u4IO32Read4B((DDRPHY_STBENSTATUS2) + (1<< POS_BANK_NUM));
                    mcSHOW_ERR_MSG(("CHB (%x, %x) ", u4DebugCnt2,u4DebugCnt3));
    
                    u4DebugCnt2 = u4IO32Read4B((DDRPHY_STBENSTATUS1)+ (2<< POS_BANK_NUM));
                    u4DebugCnt3 = u4IO32Read4B((DDRPHY_STBENSTATUS2)+ (2<< POS_BANK_NUM));
                    mcSHOW_ERR_MSG(("CHC (%x, %x) \n", u4DebugCnt2,u4DebugCnt3));
                #endif
                }
                
                 u4err_value =0;
            #if fcFOR_CHIP_ID == fcA60501        //test only
                 u4err_value = TestEngineCompare(p);
             #endif             
    
                 /*TINFO="%2d  %2d  %2d |(B3->B0) 0x%4x, 0x%4x, 0x%4x, 0x%4x | %2x %2x  %2x %2x  %2x %2x  %2x %2x | 0x%8x\n", ucdly_coarse_large, ucdly_coarse_0p5T, ucdly_fine_xT, \
                                                      u4DebugCnt[3], u4DebugCnt[2], u4DebugCnt[1], u4DebugCnt[0], \
                                                       (u4all_result_F>>24)&0xff, (u4all_result_R>>24)&0xff, \
                                                       (u4all_result_F>>16)&0xff, (u4all_result_R>>16)&0xff, \
                                                       (u4all_result_F>>8)&0xff,   (u4all_result_R>>8)&0xff, \
                                                       (u4all_result_F)&0xff,         (u4all_result_R)&0xff,    u4err_value*/
    
                #ifdef ETT_PRINT_FORMAT
                mcSHOW_DBG_MSG2(("%d  %d  %d |(B3->B0) 0x%H, 0x%H, 0x%H, 0x%H | %B %B  %B %B  %B %B  %B %B | 0x%X\n", ucdly_coarse_large, ucdly_coarse_0p5T, ucdly_fine_xT, \
                                                      u4DebugCnt[3], u4DebugCnt[2], u4DebugCnt[1], u4DebugCnt[0], \
                                                       (u4all_result_F>>24)&0xff, (u4all_result_R>>24)&0xff, \
                                                       (u4all_result_F>>16)&0xff, (u4all_result_R>>16)&0xff, \
                                                       (u4all_result_F>>8)&0xff,   (u4all_result_R>>8)&0xff, \
                                                       (u4all_result_F)&0xff,         (u4all_result_R)&0xff,    u4err_value));                    
                 
                #else
                mcSHOW_DBG_MSG2(("%2d  %2d  %2d |(B3->B0) 0x%4x, 0x%4x, 0x%4x, 0x%4x | %2x %2x  %2x %2x  %2x %2x  %2x %2x | 0x%8x\n", ucdly_coarse_large, ucdly_coarse_0p5T, ucdly_fine_xT, \
                                                      u4DebugCnt[3], u4DebugCnt[2], u4DebugCnt[1], u4DebugCnt[0], \
                                                       (u4all_result_F>>24)&0xff, (u4all_result_R>>24)&0xff, \
                                                       (u4all_result_F>>16)&0xff, (u4all_result_R>>16)&0xff, \
                                                       (u4all_result_F>>8)&0xff,   (u4all_result_R>>8)&0xff, \
                                                       (u4all_result_F)&0xff,         (u4all_result_R)&0xff,    u4err_value));                    
                 
                #endif
                 
                 mcFPRINTF((fp_A60501,"%2d  %2d  %2d |(B3->B0) 0x%4x, 0x%4x, 0x%4x, 0x%4x | (B3->B0) %2x %2x  %2x %2x  %2x %2x  %2x %2x | 0x%8x\n", ucdly_coarse_large, ucdly_coarse_0p5T, ucdly_fine_xT, \
                                                    u4DebugCnt[3], u4DebugCnt[2], u4DebugCnt[1], u4DebugCnt[0], \
                                                     (u4all_result_F>>24)&0xff, (u4all_result_R>>24)&0xff, \
                                                     (u4all_result_F>>16)&0xff, (u4all_result_R>>16)&0xff, \
                                                     (u4all_result_F>>8)&0xff,   (u4all_result_R>>8)&0xff, \
                                                     (u4all_result_F)&0xff,         (u4all_result_R)&0xff,    u4err_value));                    
    
                //find gating window pass range per DQS separately
                for (dqs_i=0; dqs_i<(p->data_width/DQS_BIT_NUMBER); dqs_i++)
                {
                    //get dqs error result
                    ucdqs_result_R = (U8)((u4all_result_R>>(8*dqs_i))&0x000000ff);
                    ucdqs_result_F = (U8)((u4all_result_F>>(8*dqs_i))&0x000000ff);
    
                    u2DebugCntPerByte =(U16) u4DebugCnt[dqs_i];
    
                #if 0
                    if(p->dram_type == TYPE_LPDDR4)
                    {
                        u2DebugCntPerByte = (U16)((u4DebugCnt>>(16*dqs_i))&0xffff);
                    }
                    else //LPDDR3
                    {
                        if(dqs_i<2)
                        {
                            u2DebugCntPerByte = (U16)((u4DebugCnt>>(16*dqs_i))&0xffff);
                        }
                        else
                        {
                            u2DebugCntPerByte = (U16)((u4DebugCnt2>>(16*(dqs_i-2)))&0xffff);
                        }
                    }
                #endif
    
                    // check if current tap is pass
                    ucCurrentPass =0;
                    if(p->dram_type == TYPE_LPDDR4)
                    {
                        if(((p->frequency >2130) &&( u2DebugCntPerByte==GATING_GOLDEND_DQSCNT_LP4)) ||
                        ((ucdqs_result_R==0) && (ucdqs_result_F==0) && (u2DebugCntPerByte==GATING_GOLDEND_DQSCNT_LP4)))
                            ucCurrentPass =1;
                    }
                    else //LPDDR3
                    {
                        if(((p->frequency >1060) &&( u2DebugCntPerByte==GATING_GOLDEND_DQSCNT_LP3)) ||
                            ((ucdqs_result_R==0) && (ucdqs_result_F==0)))
                        {
                        #if GATING_DQS_COUNTER_BYTE3_FAIL_WORKAROUND
                            if(dqs_i ==3) //byte3 counter is always 0 due to hw problem, work around : skip check byte3 counter
                                ucCurrentPass =1;
                            else //Byte 0~2
                        #endif
                            if(u2DebugCntPerByte==GATING_GOLDEND_DQSCNT_LP3)
                                ucCurrentPass =1;
                        }
                    }
    
                    //if current tap is pass 
                    if (ucCurrentPass)
                    {
                        if (ucpass_begin[dqs_i]==0)
                        {
                            //no pass tap before , so it is the begining of pass range
                            ucpass_begin[dqs_i] = 1;
                            ucpass_count_1[dqs_i] = 0;
                            ucmin_coarse_tune2T_1[dqs_i] = ucdly_coarse_large;
                            ucmin_coarse_tune0p5T_1[dqs_i] = ucdly_coarse_0p5T;
                            ucmin_fine_tune_1[dqs_i] = ucdly_fine_xT;           
    
                            /*TINFO="[Byte %d]First pass (%d, %d, %d)\n", dqs_i,ucdly_coarse_large, ucdly_coarse_0p5T, ucdly_fine_xT*/
                            mcSHOW_DBG_MSG2(("[Byte %d]First pass (%d, %d, %d)\n", dqs_i,ucdly_coarse_large, ucdly_coarse_0p5T, ucdly_fine_xT));
                        }
    
                        if (ucpass_begin[dqs_i]==1)
                        {
                            //incr pass tap number
                            ucpass_count_1[dqs_i]++;
                        }
                    }
                    else // current tap is fail
                    {
                        if (ucpass_begin[dqs_i]==1)
                        {
                            //at the end of pass range
                            ucpass_begin[dqs_i] = 0;
    
                            //save the max range settings, to avoid glitch
                            if (ucpass_count_1[dqs_i] > ucpass_count[dqs_i])
                            {
                                ucmin_coarse_tune2T[dqs_i] = ucmin_coarse_tune2T_1[dqs_i];
                                ucmin_coarse_tune0p5T[dqs_i] = ucmin_coarse_tune0p5T_1[dqs_i];
                                ucmin_fine_tune[dqs_i] = ucmin_fine_tune_1[dqs_i];
                                ucpass_count[dqs_i] = ucpass_count_1[dqs_i];
                                /*TINFO="[Byte %d]Bigger pass win(%d, %d, %d)  Pass tap=%d\n", \
                                    dqs_i, ucmin_coarse_tune2T_1[dqs_i], ucmin_coarse_tune0p5T_1[dqs_i], ucmin_fine_tune_1[dqs_i], ucpass_count_1[dqs_i]*/
                                mcSHOW_DBG_MSG(("[Byte %d]Bigger pass win(%d, %d, %d)  Pass tap=%d\n", \
                                    dqs_i, ucmin_coarse_tune2T_1[dqs_i], ucmin_coarse_tune0p5T_1[dqs_i], ucmin_fine_tune_1[dqs_i], ucpass_count_1[dqs_i]));
    
                                if((ucpass_count_1[dqs_i] > (32/DQS_GW_FINE_STEP)) && (ucpass_count_1[dqs_i] < (96/DQS_GW_FINE_STEP)))
                                {
                                    u1PassByteCount  |= (1<<dqs_i);
                                }
    
                                if(((p->dram_type==TYPE_LPDDR4) && (u1PassByteCount==0x3)) || \
                                    ((p->dram_type==TYPE_LPDDR3) && (u1PassByteCount==0xf)))
                                {
                                    mcSHOW_DBG_MSG2(("All bytes gating window pass, Done, Early break!\n"));
                                    mcFPRINTF((fp_A60501, "All bytes gating window pass, Done, Early break!\n"));
                                    ucdly_fine_xT = DQS_GW_FINE_END;//break loop
                                    ucCoarseTune = ucCoarseEnd;      //break loop
                                }
                            }
                        }
                    }
                }                                 
            }
        }

#if !SW_CHANGE_FOR_SIMULATION
            vSetCalibrationResult(p, DRAM_CALIBRATION_GATING, DRAM_OK);
#endif
        //check if there is no pass taps for each DQS
        for (dqs_i=0; dqs_i<(p->data_width/DQS_BIT_NUMBER); dqs_i++)
        {
            if (ucpass_count[dqs_i]==0)
            {
                /*TINFO="error, no pass taps in DQS_%d !!!\n", dqs_i*/
                mcSHOW_ERR_MSG(("error, no pass taps in DQS_%d !!!\n", dqs_i));
                mcFPRINTF((fp_A60501, "error, no pass taps in DQS_%d !!!\n", dqs_i));
                #if !SW_CHANGE_FOR_SIMULATION
                vSetCalibrationResult(p, DRAM_CALIBRATION_GATING, DRAM_FAIL);
                #endif
            }
            
            #if ENABLE_CLAIBTAION_WINDOW_LOG_FOR_FT
            if(ucpass_count[dqs_i] < u2MinWinSize)
            {
                u2MinWinSize = ucpass_count[dqs_i];
                u1MinWinSizeByteidx= dqs_i;
            }
            #endif
        }

        #if ENABLE_CLAIBTAION_WINDOW_LOG_FOR_FT
        mcSHOW_DBG_MSG2(("FT log: Gating min window : byte %d, size %d\n", u1MinWinSizeByteidx, u2MinWinSize*DQS_GW_FINE_STEP));
        #endif    
            
        //find center of each byte
        for (dqs_i=0; dqs_i<(p->data_width/DQS_BIT_NUMBER); dqs_i++)
        {
#if SW_CHANGE_FOR_SIMULATION  // simulation cannot support %
        // -- PI for Phase0 & Phase1 --
            uctmp_offset = ucpass_count[dqs_i]*DQS_GW_FINE_STEP/2;
            uctmp_value = ucmin_fine_tune[dqs_i]+uctmp_offset;
            ucbest_fine_tune[dqs_i] = uctmp_value - (uctmp_value/ucRX_DLY_DQSIENSTB_LOOP) * ucRX_DLY_DQSIENSTB_LOOP;
            ucbest_fine_tune_P1[dqs_i] = ucbest_fine_tune[dqs_i];
    
        // coarse tune 0.5T for Phase 0
            uctmp_offset = uctmp_value / ucRX_DLY_DQSIENSTB_LOOP;
            uctmp_value = ucmin_coarse_tune0p5T[dqs_i]+uctmp_offset;
            ucbest_coarse_tune0p5T[dqs_i] = uctmp_value - (uctmp_value/ucRX_DQS_CTL_LOOP) * ucRX_DQS_CTL_LOOP;
             
        // coarse tune 2T for Phase 0
            uctmp_offset = uctmp_value/ucRX_DQS_CTL_LOOP;
            ucbest_coarse_tune2T[dqs_i] = ucmin_coarse_tune2T[dqs_i]+uctmp_offset;
    
        // coarse tune 0.5T for Phase 1
            uctmp_value = ucbest_coarse_tune0p5T[dqs_i]+ ucFreqDiv;
            ucbest_coarse_tune0p5T_P1[dqs_i] = uctmp_value - (uctmp_value/ ucRX_DQS_CTL_LOOP) *ucRX_DQS_CTL_LOOP;
        
        // coarse tune 2T for Phase 1
            uctmp_offset = uctmp_value/ucRX_DQS_CTL_LOOP;
            ucbest_coarse_tune2T_P1[dqs_i] = ucbest_coarse_tune2T[dqs_i]+uctmp_offset;
#else
        // -- PI for Phase0 & Phase1 --
            uctmp_offset = ucpass_count[dqs_i]*DQS_GW_FINE_STEP/2;
            uctmp_value = ucmin_fine_tune[dqs_i]+uctmp_offset;
            ucbest_fine_tune[dqs_i] = uctmp_value% ucRX_DLY_DQSIENSTB_LOOP;
            ucbest_fine_tune_P1[dqs_i] = ucbest_fine_tune[dqs_i];
    
        // coarse tune 0.5T for Phase 0
            uctmp_offset = uctmp_value / ucRX_DLY_DQSIENSTB_LOOP;
            uctmp_value = ucmin_coarse_tune0p5T[dqs_i]+uctmp_offset;
            ucbest_coarse_tune0p5T[dqs_i] = uctmp_value% ucRX_DQS_CTL_LOOP;
            
        // coarse tune 2T for Phase 0
            uctmp_offset = uctmp_value/ucRX_DQS_CTL_LOOP;
            ucbest_coarse_tune2T[dqs_i] = ucmin_coarse_tune2T[dqs_i]+uctmp_offset;
    
    // coarse tune 0.5T for Phase 1
    #if fcFOR_CHIP_ID == fcA60501
        uctmp_value = ucbest_coarse_tune0p5T[dqs_i]+ (ucRX_DQS_CTL_LOOP/2);
    #else //Everest new change
        uctmp_value = ucbest_coarse_tune0p5T[dqs_i]+ ucFreqDiv;
    #endif
        ucbest_coarse_tune0p5T_P1[dqs_i] = uctmp_value% ucRX_DQS_CTL_LOOP;
        
        // coarse tune 2T for Phase 1
            uctmp_offset = uctmp_value/ucRX_DQS_CTL_LOOP;
            ucbest_coarse_tune2T_P1[dqs_i] = ucbest_coarse_tune2T[dqs_i]+uctmp_offset;
#endif

        }
    
        mcSHOW_DBG_MSG(("===============================================================================\n"));
        mcSHOW_DBG_MSG(("    dqs input gating widnow, final delay value\n    channel=%d(2:cha, 3:chb)  rank=%d\n", p->channel, p->rank));
        mcSHOW_DBG_MSG(("===============================================================================\n"));
        mcSHOW_DBG_MSG(("test2_1: 0x%x, test2_2: 0x%x, test pattern: %d\n",  p->test2_1,p->test2_2, p->test_pattern));
        mcSHOW_DBG_MSG2(("dqs input gating widnow, best delay value\n"));
        mcSHOW_DBG_MSG2(("===============================================================================\n")); 
        mcFPRINTF((fp_A60501, "===============================================================================\n"));
        mcFPRINTF((fp_A60501, "    dqs input gating widnow, final delay value\n    channel=%d(2:cha, 3:chb)  \n", p->channel));
        mcFPRINTF((fp_A60501, "===============================================================================\n"));
        mcFPRINTF((fp_A60501, "test2_1: 0x%x, test2_2: 0x%x, test pattern: %d\n",  p->test2_1,p->test2_2, p->test_pattern));
        mcFPRINTF((fp_A60501, "dqs input gating widnow, best delay value\n"));
        mcFPRINTF((fp_A60501, "===============================================================================\n")); 
    
        for (dqs_i=0; dqs_i<(p->data_width/DQS_BIT_NUMBER); dqs_i++)
        {
            /*TINFO="best DQS%d delay(2T, 0.5T, PI) = (%d, %d, %d)\n", dqs_i, ucbest_coarse_tune2T[dqs_i], ucbest_coarse_tune0p5T[dqs_i], ucbest_fine_tune[dqs_i]*/
            mcSHOW_DBG_MSG(("best DQS%d delay(2T, 0.5T, PI) = (%d, %d, %d)\n", dqs_i, ucbest_coarse_tune2T[dqs_i], ucbest_coarse_tune0p5T[dqs_i], ucbest_fine_tune[dqs_i]));
            mcFPRINTF((fp_A60501,"best DQS%d delay(2T, 0.5T, PI) = (%d, %d, %d)\n", dqs_i, ucbest_coarse_tune2T[dqs_i], ucbest_coarse_tune0p5T[dqs_i], ucbest_fine_tune[dqs_i]));
            #if GATING_ADJUST_TXDLY_FOR_TRACKING         
            // find min gating TXDLY (should be in P0)
            u1TX_dly_DQSgated  = ((ucbest_coarse_tune2T[dqs_i] <<1)|((ucbest_coarse_tune0p5T[dqs_i] >>2)&0x1));      
            if(u1TX_dly_DQSgated < u1TXDLY_Cal_min)
                u1TXDLY_Cal_min = u1TX_dly_DQSgated;
            
            ucbest_coarse_tune0p5T_backup[p->rank][dqs_i] = ucbest_coarse_tune0p5T[dqs_i];
            ucbest_coarse_tune2T_backup[p->rank][dqs_i] = ucbest_coarse_tune2T[dqs_i];
            #endif
        }
        mcSHOW_DBG_MSG2(("===============================================================================\n")); 
        mcFPRINTF((fp_A60501,"===============================================================================\n")); 
    
        for (dqs_i=0; dqs_i<(p->data_width/DQS_BIT_NUMBER); dqs_i++)
        {
            /*TINFO="best DQS%d P1 delay(2T, 0.5T, PI) = (%d, %d, %d)\n", dqs_i, ucbest_coarse_tune2T_P1[dqs_i], ucbest_coarse_tune0p5T_P1[dqs_i], ucbest_fine_tune[dqs_i]*/
            mcSHOW_DBG_MSG(("best DQS%d P1 delay(2T, 0.5T, PI) = (%d, %d, %d)\n", dqs_i, ucbest_coarse_tune2T_P1[dqs_i], ucbest_coarse_tune0p5T_P1[dqs_i], ucbest_fine_tune[dqs_i]));
            mcFPRINTF((fp_A60501,"best DQS%d P1 delay(2T, 0.5T, PI) = (%d, %d, %d)\n", dqs_i, ucbest_coarse_tune2T_P1[dqs_i], ucbest_coarse_tune0p5T_P1[dqs_i], ucbest_fine_tune[dqs_i]));

            #if GATING_ADJUST_TXDLY_FOR_TRACKING         
            // find max gating TXDLY (should be in P1)
            u1TX_dly_DQSgated  = ((ucbest_coarse_tune2T_P1[dqs_i] <<1)|((ucbest_coarse_tune0p5T_P1[dqs_i] >>2)&0x1));
            if(u1TX_dly_DQSgated > u1TXDLY_Cal_max)
                u1TXDLY_Cal_max = u1TX_dly_DQSgated;
   
            ucbest_coarse_tune0p5T_P1_backup[p->rank][dqs_i] = ucbest_coarse_tune0p5T_P1[dqs_i];
            ucbest_coarse_tune2T_P1_backup[p->rank][dqs_i] = ucbest_coarse_tune2T_P1[dqs_i];
            #endif
        }
        
        mcSHOW_DBG_MSG2(("===============================================================================\n")); 
        mcFPRINTF((fp_A60501,"===============================================================================\n")); 
    
        //Restore registers 
        vIO32Write4B(DRAMC_REG_ADDR(DRAMC_REG_DQSCAL0), u4BakReg_DRAMC_DQSCAL0);
        vIO32Write4B(DRAMC_REG_ADDR(DRAMC_REG_STBCAL_F), u4BakReg_DRAMC_STBCAL_F);
        vIO32Write4B(DRAMC_REG_ADDR(DRAMC_REG_WODT), u4BakReg_DRAMC_WODT);
        vIO32Write4B(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), u4BakReg_DRAMC_SPCMD);
    
        // Set Coarse Tune Value to registers
        DramPhyCGReset(p, 1);// need to reset when UI update or PI change >=2
    
    #if REG_SHUFFLE_REG_CHECK
        ShuffleRegCheck =1;
    #endif
    
        // 4T or 2T coarse tune
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH2), ucbest_coarse_tune2T[0], SELPH2_TXDLY_DQSGATE);  //DQS0_P0
        vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH12), \
                                    P_Fld((U32) ucbest_coarse_tune2T[1], SELPH12_TX_DLY_DQS1_GATED)| \
                                    P_Fld((U32) ucbest_coarse_tune2T[2], SELPH12_TX_DLY_DQS2_GATED)| \
                                    P_Fld((U32) ucbest_coarse_tune2T[3], SELPH12_TX_DLY_DQS3_GATED));
    
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH2), ucbest_coarse_tune2T_P1[0], SELPH2_TXDLY_DQSGATE_P1);//DQS0_P1
        vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH12), \
                                    P_Fld((U32) ucbest_coarse_tune2T_P1[1], SELPH12_TX_DLY_DQS1_GATED_P1)| \
                                    P_Fld((U32) ucbest_coarse_tune2T_P1[2], SELPH12_TX_DLY_DQS2_GATED_P1)| \
                                    P_Fld((U32) ucbest_coarse_tune2T_P1[3], SELPH12_TX_DLY_DQS3_GATED_P1));
    
        // 0.5T coarse tune
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH5), (ucbest_coarse_tune0p5T[0] & 0x3), SELPH5_DLY_DQSGATE); 
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH21), (ucbest_coarse_tune0p5T[0]>>2), SELPH21_DLY_DQSGATE_2); 
        vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH13), \
                                    P_Fld((U32) ucbest_coarse_tune0p5T[1], SELPH13_REG_DLY_DQS1_GATED)| \
                                    P_Fld((U32) ucbest_coarse_tune0p5T[2], SELPH13_REG_DLY_DQS2_GATED)| \
                                    P_Fld((U32) ucbest_coarse_tune0p5T[3], SELPH13_REG_DLY_DQS3_GATED));
    
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH5), (ucbest_coarse_tune0p5T_P1[0] & 0x3), SELPH5_DLY_DQSGATE_P1); 
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH21), (ucbest_coarse_tune0p5T_P1[0]>>2), SELPH21_DLY_DQSGATE_P1_2); 
        vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH13), \
                                    P_Fld((U32) ucbest_coarse_tune0p5T_P1[1], SELPH13_REG_DLY_DQS1_GATED_P1)| \
                                    P_Fld((U32) ucbest_coarse_tune0p5T_P1[2], SELPH13_REG_DLY_DQS2_GATED_P1)| \
                                    P_Fld((U32) ucbest_coarse_tune0p5T_P1[3], SELPH13_REG_DLY_DQS3_GATED_P1));
    
        if(p->rank == RANK_1)
        {
            // 4T or 2T coarse tune
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH6_1), ucbest_coarse_tune2T[0], SELPH6_1_TXDLY_R1DQSGATE);  //DQS0_P0
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH14), \
                                        P_Fld((U32) ucbest_coarse_tune2T[1], SELPH12_TX_DLY_DQS1_GATED)| \
                                        P_Fld((U32) ucbest_coarse_tune2T[2], SELPH12_TX_DLY_DQS2_GATED)| \
                                        P_Fld((U32) ucbest_coarse_tune2T[3], SELPH12_TX_DLY_DQS3_GATED));
    
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH6_1), ucbest_coarse_tune2T_P1[0], SELPH6_1_TXDLY_R1DQSGATE_P1);//DQS0_P1
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH14), \
                                        P_Fld((U32) ucbest_coarse_tune2T_P1[1], SELPH12_TX_DLY_DQS1_GATED_P1)| \
                                        P_Fld((U32) ucbest_coarse_tune2T_P1[2], SELPH12_TX_DLY_DQS2_GATED_P1)| \
                                        P_Fld((U32) ucbest_coarse_tune2T_P1[3], SELPH12_TX_DLY_DQS3_GATED_P1));
    
            // 0.5T coarse tune
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH6_1), (ucbest_coarse_tune0p5T[0] & 0x3), SELPH6_1_DLY_R1DQSGATE); 
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH21), (ucbest_coarse_tune0p5T[0]>>2), SELPH21_DLY_R1DQSGATE_2); 
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH15), \
                                        P_Fld((U32) ucbest_coarse_tune0p5T[1], SELPH13_REG_DLY_DQS1_GATED)| \
                                        P_Fld((U32) ucbest_coarse_tune0p5T[2], SELPH13_REG_DLY_DQS2_GATED)| \
                                        P_Fld((U32) ucbest_coarse_tune0p5T[3], SELPH13_REG_DLY_DQS3_GATED));
    
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH6_1), (ucbest_coarse_tune0p5T_P1[0] & 0x3), SELPH6_1_DLY_R1DQSGATE_P1); 
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH21), (ucbest_coarse_tune0p5T_P1[0]>>2), SELPH21_DLY_R1DQSGATE_P1_2); 
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH15), \
                                        P_Fld((U32) ucbest_coarse_tune0p5T_P1[1], SELPH13_REG_DLY_DQS1_GATED_P1)| \
                                        P_Fld((U32) ucbest_coarse_tune0p5T_P1[2], SELPH13_REG_DLY_DQS2_GATED_P1)| \
                                        P_Fld((U32) ucbest_coarse_tune0p5T_P1[3], SELPH13_REG_DLY_DQS3_GATED_P1));
    
        }
    
    #if GATING_RODT_LATANCY_EN    //LP3 RODT is not enable, don't need to set the RODT settings.
        // RODT = Byte 0 Gating - 8UI,
        ///TODO: Fix me.   Only one RODT, but 2 ranks 2 gating.  Need to find out the earliest gating of each byte of each rank.
        if(p->dram_type == TYPE_LPDDR4)
        {
            if(ucbest_coarse_tune2T[0] >0)
            {
                ucbest_coarse_large_RODT =ucbest_coarse_tune2T[0] -1;
                ucbest_coarse_0p5T_RODT = ucbest_coarse_tune0p5T[0];
            }
            else //if(ucbest_coarse_tune2T[0] ==0)  //shouble not happen,  just only protect this happen
            {
                ucbest_coarse_large_RODT =0;
                ucbest_coarse_0p5T_RODT = 0;
                mcSHOW_DBG_MSG(("[DramcRxdqsGatingCal] Error: ucbest_coarse_tune2T is already 0. RODT cannot be -1 UI\n"));
                mcFPRINTF((fp_A60501, "[DramcRxdqsGatingCal] Error: ucbest_coarse_tune2T is already 0. RODT cannot be -1 UI\n"));    
            }
    
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH6_1), ucbest_coarse_large_RODT, SELPH6_1_TXDLY_RODTEN);  
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH6_1), ucbest_coarse_0p5T_RODT, SELPH6_1_DLY_RODTEN); 
            mcSHOW_DBG_MSG2(("best RODT delay(2T, 0.5T) = (%d, %d)\n", ucbest_coarse_large_RODT, ucbest_coarse_0p5T_RODT));
            mcFPRINTF((fp_A60501,"best RODT delay(2T, 0.5T) = (%d, %d)\n", ucbest_coarse_large_RODT, ucbest_coarse_0p5T_RODT));
        }
    #endif

     #if (GATING_TXDLY_CHNAGE &(!GATING_ADJUST_TXDLY_FOR_TRACKING))       
        u4ReadDQSINCTL = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DQSCTL1), DQSCTL1_DQSINCTL);

        u4ReadTXDLY= (((ucbest_coarse_tune2T[0]<<1) + ((ucbest_coarse_tune0p5T[0]>>2) & 0x1))+
                                ((ucbest_coarse_tune2T[1]<<1) + ((ucbest_coarse_tune0p5T[1]>>2) & 0x1))+
                                ((ucbest_coarse_tune2T[2]<<1) + ((ucbest_coarse_tune0p5T[2]>>2) & 0x1))+
                                ((ucbest_coarse_tune2T[3]<<1) + ((ucbest_coarse_tune0p5T[3]>>2) & 0x1)))/4;

        if(u4ReadDQSINCTL + u4ReadTXDLY >= 4)
            u4RankINCTL_ROOT = u4ReadDQSINCTL + u4ReadTXDLY -4;
        else
            u4RankINCTL_ROOT = 0;

        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DUMMY), u4RankINCTL_ROOT, DUMMY_RANKINCTL_ROOT1);  
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DQSCAL1), u4RankINCTL_ROOT, DQSCAL1_RANKINCTL);  
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DQSCAL1), u4RankINCTL_ROOT, DQSCAL1_RANKINCTL_PHY);  

         mcSHOW_DBG_MSG(("DQSINCTL=%d, TXDLY=%d, RANKINCTL_ROOT=%d\n", u4ReadDQSINCTL, u4ReadTXDLY, u4RankINCTL_ROOT));
         mcFPRINTF((fp_A60501,"DQSINCTL=%d, TXDLY=%d, RANKINCTL_ROOT=%d\n", u4ReadDQSINCTL, u4ReadTXDLY, u4RankINCTL_ROOT));
    #endif
    
        // Set Fine Tune Value to registers
        u4value = ucbest_fine_tune[0] | (ucbest_fine_tune[1]<<8) | (ucbest_fine_tune[2]<<16) | (ucbest_fine_tune[3]<<24);
        vIO32Write4B(DRAMC_REG_ADDR(DRAMC_REG_R0DQSIEN), u4value);  
    
        if(p->rank == RANK_1)
        {
            vIO32Write4B(DRAMC_REG_ADDR(DRAMC_REG_R1DQSIEN), u4value);  
        }
        
    #if REG_SHUFFLE_REG_CHECK
        ShuffleRegCheck =0;
    #endif
        
        mcDELAY_US(1);//delay 2T
        DramPhyCGReset(p, 0);  
        DramPhyReset(p);   //reset phy, reset read data counter 
    
        /*TINFO="[DramcRxdqsGatingCal] ====Done====\n"*/
        mcSHOW_DBG_MSG2(("[DramcRxdqsGatingCal] ====Done====\n"));
        mcFPRINTF((fp_A60501, "[DramcRxdqsGatingCal] ====Done====\n"));    
    
        return DRAM_OK;
        // log example
        /*
    0   1  12 |(B3->B0) 0x   0, 0x1211, 0x1212, 0x1211 | (B3->B0) 11 11  11 11  11 11  11 11 | 0xffffffff
    0   1  16 |(B3->B0) 0x   0, 0x1211, 0x1212, 0x1211 | (B3->B0) 11 11  11 11  11 11  11 11 | 0xffffffff
    0   1  20 |(B3->B0) 0x   0, 0x1211, 0x1212, 0x1211 | (B3->B0) 11 11  11 11  11 11  11 11 | 0xffffffff
    0   1  24 |(B3->B0) 0x   0, 0x1211, 0x1212, 0x1211 | (B3->B0) 11 11  11 11  11 11  11 11 | 0xffffffff
    0   1  28 |(B3->B0) 0x   0, 0x1211, 0x1212, 0x1211 | (B3->B0) 11 11  11 11  11 11  11 11 | 0xffffffff
    0   2   0 |(B3->B0) 0x   0, 0x1d1c, 0x1212, 0x1211 | (B3->B0) 11 11  11 11  11 11  11 11 | 0xffffffff
    0   2   4 |(B3->B0) 0x   0, 0x2324, 0x1212, 0x1413 | (B3->B0) 11 11   0  0  11 11  11 11 | 0xffffffff
    0   2   8 |(B3->B0) 0x   0, 0x2323, 0x1212, 0x2222 | (B3->B0)  0  0   0  0  11 11  11 11 | 0xff00ffff
    0   2  12 |(B3->B0) 0x   0, 0x2323, 0x1211, 0x2323 | (B3->B0)  0  0   0  0  11 11   0  0 | 0x    ffff
    0   2  16 |(B3->B0) 0x   0, 0x2323, 0x 504, 0x2324 | (B3->B0)  0  0   0  0  11 11   0  0 | 0x    ff00
    0   2  20 |(B3->B0) 0x   0, 0x2323, 0x 303, 0x2323 | (B3->B0)  0  0   0  0   1  1   0  0 | 0x    ff00
    0   2  24 |(B3->B0) 0x   0, 0x2323, 0x2324, 0x2323 | (B3->B0)  0  0   0  0   0  0   0  0 | 0x       0
    0   2  28 |(B3->B0) 0x   0, 0x2323, 0x2323, 0x2323 | (B3->B0)  0  0   0  0   0  0   0  0 | 0x       0
    0   3   0 |(B3->B0) 0x   0, 0x2323, 0x2323, 0x2323 | (B3->B0)  0  0   0  0   0  0   0  0 | 0x       0
    0   3   4 |(B3->B0) 0x   0, 0x2323, 0x2323, 0x2323 | (B3->B0)  0  0   0  0   0  0   0  0 | 0x       0
    0   3   8 |(B3->B0) 0x   0, 0x2323, 0x2323, 0x2323 | (B3->B0)  0  0   0  0   0  0   0  0 | 0x       0
    0   3  12 |(B3->B0) 0x   0, 0x2323, 0x2323, 0x2323 | (B3->B0)  0  0   0  0   0  0   0  0 | 0x       0
    0   3  16 |(B3->B0) 0x   0, 0x2323, 0x2323, 0x2323 | (B3->B0)  0  0   0  0   0  0   0  0 | 0x       0
    0   3  20 |(B3->B0) 0x   0, 0x2323, 0x2323, 0x2323 | (B3->B0)  0  0   0  0   0  0   0  0 | 0x       0
    0   3  24 |(B3->B0) 0x   0, 0x2323, 0x2323, 0x2323 | (B3->B0)  0  0   0  0   0  0   0  0 | 0x       0
    0   3  28 |(B3->B0) 0x   0, 0x2323, 0x2323, 0x2323 | (B3->B0)  0  0   0  0   0  0   0  0 | 0x       0
    1   0   0 |(B3->B0) 0x   0, 0x2120, 0x2323, 0x2323 | (B3->B0) 11 11  11 11   0  0   0  0 | 0xffff0000
    1   0   4 |(B3->B0) 0x   0, 0x2323, 0x2323, 0x1212 | (B3->B0) 11 11   0  0   0  0  11 11 | 0xffff00ff
    1   0   8 |(B3->B0) 0x   0, 0x2324, 0x2323, 0x2323 | (B3->B0)  0  0   0  0   0  0   0  0 | 0xffff00ff
    1   0  12 |(B3->B0) 0x   0, 0x2323, 0x2323, 0x2323 | (B3->B0)  0  0   0  0   0  0   0  0 | 0xffff00ff
    1   0  16 |(B3->B0) 0x   0, 0x2323, 0x1f1f, 0x2323 | (B3->B0)  0  0   0  0  11 11   0  0 | 0xffffffff
    1   0  20 |(B3->B0) 0x   0, 0x2323, 0x2324, 0x2323 | (B3->B0)  0  0   0  0   0  0   0  0 | 0xffffffff
    1   0  24 |(B3->B0) 0x   0, 0x2323, 0x2323, 0x2323 | (B3->B0)  0  0   0  0   0  0   0  0 | 0xffffffff
    1   0  28 |(B3->B0) 0x   0, 0x2322, 0x2324, 0x2323 | (B3->B0) 11 11  11 11   0  0   0  0 | 0x    ffff
    1   1   0 |(B3->B0) 0x   0, 0x2322, 0x2324, 0x2323 | (B3->B0) 11 11  11 11   0  0   0  0 | 0xffffffff
    1   1   4 |(B3->B0) 0x   0, 0x2323, 0x2324, 0x2322 | (B3->B0) 11 11  11 11   0  0  11 11 | 0xffffff00
    1   1   8 |(B3->B0) 0x   0, 0x2323, 0x2324, 0x2322 | (B3->B0) 11 11  11 11  11 11  11 11 | 0xffffffff
    1   1  12 |(B3->B0) 0x   0, 0x2323, 0x2323, 0x2323 | (B3->B0) 11 11  11 11  11 11  11 11 | 0xffffffff
    1   1  16 |(B3->B0) 0x   0, 0x2323, 0x2322, 0x2323 | (B3->B0) 11 11  11 11  11 11  11 11 | 0xffffffff
    1   1  20 |(B3->B0) 0x   0, 0x2323, 0x2323, 0x2323 | (B3->B0) 11 11  11 11  11 11  11 11 | 0xffffffff
    1   1  24 |(B3->B0) 0x   0, 0x2323, 0x2323, 0x2323 | (B3->B0) 11 11  11 11  11 11  11 11 | 0xffffffff
    1   1  28 |(B3->B0) 0x   0, 0x2323, 0x2323, 0x2323 | (B3->B0) 11 11  11 11  11 11  11 11 | 0xffffffff
    1   2   0 |(B3->B0) 0x   0, 0x1a1b, 0x2323, 0x2323 | (B3->B0) 11 11  11 11  11 11  11 11 | 0xffffffff
    1   2   4 |(B3->B0) 0x   0, 0x1a1b, 0x2323, 0x1e1f | (B3->B0)  0  0  11 11  11 11   0  0 | 0xffffffff

   ===============================================================================
       dqs input gating widnow, final delay value
       channel=2(2:cha, 3:chb)  
   ===============================================================================
   test2_1: 0x55000000, test2_2: 0xaa000400, test pattern: 5
   dqs input gating widnow, best delay value
   ===============================================================================
   best DQS0 delay(2T, 0.5T, PI) = (0, 3, 12)
   best DQS1 delay(2T, 0.5T, PI) = (0, 3, 22)
   best DQS2 delay(2T, 0.5T, PI) = (0, 3, 4)
   best DQS3 delay(2T, 0.5T, PI) = (0, 3, 4)
   ===============================================================================
   best DQS0 P1 delay(2T, 0.5T, PI) = (1, 1, 12)
   best DQS1 P1 delay(2T, 0.5T, PI) = (1, 1, 22)
   best DQS2 P1 delay(2T, 0.5T, PI) = (1, 1, 4)
   best DQS3 P1 delay(2T, 0.5T, PI) = (1, 1, 4)
   ===============================================================================
   [DramcRxdqsGatingCal] ====Done====

*/
}

#if GATING_ADJUST_TXDLY_FOR_TRACKING
DRAM_STATUS_T DramcRxdqsGatingPostProcess(DRAMC_CTX_T *p)
{
     U8 dqs_i;
     U8 u1RankIdx, u1RankMax;
     S8 s1ChangeDQSINCTL;

     U32 u4ReadDQSINCTL, u4ReadTXDLY[RANK_MAX][DQS_NUMBER], u4ReadTXDLY_P1[RANK_MAX][DQS_NUMBER], u4RankINCTL_ROOT, u4XRTR2R, reg_TX_dly_DQSgated_min;
 
     // 800  : reg_TX_dly_DQSgated (min) =2
     // 1066 : reg_TX_dly_DQSgated (min) =2
     // 1270 : reg_TX_dly_DQSgated (min) =2
     // 1600 : reg_TX_dly_DQSgated (min) =3
     // 1866 : reg_TX_dly_DQSgated (min) =3
    if(p->frequency < 700)
         reg_TX_dly_DQSgated_min = 2;
     else
         reg_TX_dly_DQSgated_min= 3;
    
     s1ChangeDQSINCTL = reg_TX_dly_DQSgated_min- u1TXDLY_Cal_min;
     
     mcSHOW_DBG_MSG(("[DramcRxdqsGatingPostProcess] p->frequency %d\n", p->frequency));
     mcSHOW_DBG_MSG(("[DramcRxdqsGatingPostProcess] s1ChangeDQSINCTL %d, reg_TX_dly_DQSgated_min %d, u1TXDLY_Cal_min %d\n", s1ChangeDQSINCTL, reg_TX_dly_DQSgated_min, u1TXDLY_Cal_min));
     if(s1ChangeDQSINCTL!=0)  // need to change DQSINCTL and TXDLY of each byte
     {        
         u1TXDLY_Cal_min += s1ChangeDQSINCTL;
         u1TXDLY_Cal_max += s1ChangeDQSINCTL;
     
        #ifdef DUAL_RANKS
        if(uiDualRank)
            u1RankMax = RANK_MAX;
        else
        #endif
             u1RankMax =RANK_1;
        
        for(u1RankIdx=0; u1RankIdx<u1RankMax; u1RankIdx++)   
        {
             mcSHOW_DBG_MSG2(("====DramcRxdqsGatingPostProcess (Rank = %d) ========================================\n", u1RankIdx)); 
             
             for (dqs_i=0; dqs_i<(p->data_width/DQS_BIT_NUMBER); dqs_i++)
             {
                 u4ReadTXDLY[u1RankIdx][dqs_i]= ((ucbest_coarse_tune2T_backup[u1RankIdx][dqs_i]<<1) + ((ucbest_coarse_tune0p5T_backup[u1RankIdx][dqs_i]>>2) & 0x1));
                 u4ReadTXDLY_P1[u1RankIdx][dqs_i]= ((ucbest_coarse_tune2T_P1_backup[u1RankIdx][dqs_i]<<1) + ((ucbest_coarse_tune0p5T_P1_backup[u1RankIdx][dqs_i]>>2) & 0x1));          

                 u4ReadTXDLY[u1RankIdx][dqs_i] += s1ChangeDQSINCTL;
                 u4ReadTXDLY_P1[u1RankIdx][dqs_i] += s1ChangeDQSINCTL;

                 ucbest_coarse_tune2T_backup[u1RankIdx][dqs_i] = (u4ReadTXDLY[u1RankIdx][dqs_i] >>1);
                 ucbest_coarse_tune0p5T_backup[u1RankIdx][dqs_i] = ((u4ReadTXDLY[u1RankIdx][dqs_i] & 0x1) <<2)+(ucbest_coarse_tune0p5T_backup[u1RankIdx][dqs_i] & 0x3);

                 ucbest_coarse_tune2T_P1_backup[u1RankIdx][dqs_i] = (u4ReadTXDLY_P1[u1RankIdx][dqs_i] >>1);
                 ucbest_coarse_tune0p5T_P1_backup[u1RankIdx][dqs_i] = ((u4ReadTXDLY_P1[u1RankIdx][dqs_i] & 0x1)<<2) +(ucbest_coarse_tune0p5T_P1_backup[u1RankIdx][dqs_i] & 0x3);
                 
                 mcSHOW_DBG_MSG(("best DQS%d delay(2T, 0.5T) = (%d, %d)\n", dqs_i, ucbest_coarse_tune2T_backup[u1RankIdx][dqs_i], ucbest_coarse_tune0p5T_backup[u1RankIdx][dqs_i]));
             }

             for (dqs_i=0; dqs_i<(p->data_width/DQS_BIT_NUMBER); dqs_i++)
             {
                mcSHOW_DBG_MSG(("best DQS%d P1 delay(2T, 0.5T) = (%d, %d)\n", dqs_i, ucbest_coarse_tune2T_P1_backup[u1RankIdx][dqs_i], ucbest_coarse_tune0p5T_P1_backup[u1RankIdx][dqs_i]));
             }
        }

        // 4T or 2T coarse tune
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH2), ucbest_coarse_tune2T_backup[0][0], SELPH2_TXDLY_DQSGATE);  //DQS0_P0
        vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH12), \
                                    P_Fld((U32) ucbest_coarse_tune2T_backup[0][1], SELPH12_TX_DLY_DQS1_GATED)| \
                                    P_Fld((U32) ucbest_coarse_tune2T_backup[0][2], SELPH12_TX_DLY_DQS2_GATED)| \
                                    P_Fld((U32) ucbest_coarse_tune2T_backup[0][3], SELPH12_TX_DLY_DQS3_GATED));
    
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH2), ucbest_coarse_tune2T_P1_backup[0][0], SELPH2_TXDLY_DQSGATE_P1);//DQS0_P1
        vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH12), \
                                    P_Fld((U32) ucbest_coarse_tune2T_P1_backup[0][1], SELPH12_TX_DLY_DQS1_GATED_P1)| \
                                    P_Fld((U32) ucbest_coarse_tune2T_P1_backup[0][2], SELPH12_TX_DLY_DQS2_GATED_P1)| \
                                    P_Fld((U32) ucbest_coarse_tune2T_P1_backup[0][3], SELPH12_TX_DLY_DQS3_GATED_P1));
    
        // 0.5T coarse tune
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH5), (ucbest_coarse_tune0p5T_backup[0][0] & 0x3), SELPH5_DLY_DQSGATE); 
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH21), (ucbest_coarse_tune0p5T_backup[0][0]>>2), SELPH21_DLY_DQSGATE_2); 
        vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH13), \
                                    P_Fld((U32) ucbest_coarse_tune0p5T_backup[0][1], SELPH13_REG_DLY_DQS1_GATED)| \
                                    P_Fld((U32) ucbest_coarse_tune0p5T_backup[0][2], SELPH13_REG_DLY_DQS2_GATED)| \
                                    P_Fld((U32) ucbest_coarse_tune0p5T_backup[0][3], SELPH13_REG_DLY_DQS3_GATED));
    
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH5), (ucbest_coarse_tune0p5T_P1_backup[0][0] & 0x3), SELPH5_DLY_DQSGATE_P1); 
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH21), (ucbest_coarse_tune0p5T_P1_backup[0][0]>>2), SELPH21_DLY_DQSGATE_P1_2); 
        vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH13), \
                                    P_Fld((U32) ucbest_coarse_tune0p5T_P1_backup[0][1], SELPH13_REG_DLY_DQS1_GATED_P1)| \
                                    P_Fld((U32) ucbest_coarse_tune0p5T_P1_backup[0][2], SELPH13_REG_DLY_DQS2_GATED_P1)| \
                                    P_Fld((U32) ucbest_coarse_tune0p5T_P1_backup[0][3], SELPH13_REG_DLY_DQS3_GATED_P1));

    
    #ifdef DUAL_RANKS
     if(uiDualRank)
        {
            // 4T or 2T coarse tune
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH6_1), ucbest_coarse_tune2T_backup[1][0], SELPH6_1_TXDLY_R1DQSGATE);  //DQS0_P0
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH14), \
                                        P_Fld((U32) ucbest_coarse_tune2T_backup[1][1], SELPH12_TX_DLY_DQS1_GATED)| \
                                        P_Fld((U32) ucbest_coarse_tune2T_backup[1][2], SELPH12_TX_DLY_DQS2_GATED)| \
                                        P_Fld((U32) ucbest_coarse_tune2T_backup[1][3], SELPH12_TX_DLY_DQS3_GATED));
    
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH6_1), ucbest_coarse_tune2T_P1_backup[1][0], SELPH6_1_TXDLY_R1DQSGATE_P1);//DQS0_P1
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH14), \
                                        P_Fld((U32) ucbest_coarse_tune2T_P1_backup[1][1], SELPH12_TX_DLY_DQS1_GATED_P1)| \
                                        P_Fld((U32) ucbest_coarse_tune2T_P1_backup[1][2], SELPH12_TX_DLY_DQS2_GATED_P1)| \
                                        P_Fld((U32) ucbest_coarse_tune2T_P1_backup[1][3], SELPH12_TX_DLY_DQS3_GATED_P1));
    
            // 0.5T coarse tune
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH6_1), (ucbest_coarse_tune0p5T_backup[1][0] & 0x3), SELPH6_1_DLY_R1DQSGATE); 
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH21), (ucbest_coarse_tune0p5T_backup[1][0]>>2), SELPH21_DLY_R1DQSGATE_2); 
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH15), \
                                        P_Fld((U32) ucbest_coarse_tune0p5T_backup[1][1], SELPH13_REG_DLY_DQS1_GATED)| \
                                        P_Fld((U32) ucbest_coarse_tune0p5T_backup[1][2], SELPH13_REG_DLY_DQS2_GATED)| \
                                        P_Fld((U32) ucbest_coarse_tune0p5T_backup[1][3], SELPH13_REG_DLY_DQS3_GATED));
    
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH6_1), (ucbest_coarse_tune0p5T_P1_backup[1][0] & 0x3), SELPH6_1_DLY_R1DQSGATE_P1); 
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH21), (ucbest_coarse_tune0p5T_P1_backup[1][0]>>2), SELPH21_DLY_R1DQSGATE_P1_2); 
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH15), \
                                        P_Fld((U32) ucbest_coarse_tune0p5T_P1_backup[1][1], SELPH13_REG_DLY_DQS1_GATED_P1)| \
                                        P_Fld((U32) ucbest_coarse_tune0p5T_P1_backup[1][2], SELPH13_REG_DLY_DQS2_GATED_P1)| \
                                        P_Fld((U32) ucbest_coarse_tune0p5T_P1_backup[1][3], SELPH13_REG_DLY_DQS3_GATED_P1));
    
        }
    #endif
    }
     
    u4ReadDQSINCTL = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DQSCTL1), DQSCTL1_DQSINCTL);
    u4ReadDQSINCTL -= s1ChangeDQSINCTL;

    if(u4ReadDQSINCTL>=2)
        u4RankINCTL_ROOT = u4ReadDQSINCTL-2;
    else
    {
        u4RankINCTL_ROOT=0;
        mcSHOW_ERR_MSG(("[DramcRxdqsGatingPostProcess] DQSINCTL <2,  Risk for supporting 1066/RL8\n"));
    }

    //XRTR2R=A-phy forbidden margin(4T) + reg_TX_dly_DQSgated (max) +Roundup(tDQSCKdiff/MCK+1UI)+1(05T sel_ph margin)-1(forbidden margin overlap part)    
    //Roundup(tDQSCKdiff/MCK+1UI) =2 for(1066~1866)
    #if SUPPORT_LP3_800
    if(p->frequency <= 400 )// LP3-800
        u4XRTR2R= 5 + u1TXDLY_Cal_max;  // 4+ u1TXDLY_Cal_max +1
    else
    #endif
    u4XRTR2R= 6 + u1TXDLY_Cal_max;  // 4+ u1TXDLY_Cal_max +2
  
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DQSCTL1), u4ReadDQSINCTL, DQSCTL1_DQSINCTL);  //Rank0 DQSINCTL
    //vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DQSCTL2), u4ReadDQSINCTL, DQSCTL2_R1DQSINCTL); //Rank1 DQSINCTL, no use in A-PHY. Common DQSINCTL of both rank.
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DUMMY), u4RankINCTL_ROOT, DUMMY_RANKINCTL_ROOT1);  
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DQSCAL1), u4RankINCTL_ROOT, DQSCAL1_RANKINCTL);  
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DQSCAL1), u4RankINCTL_ROOT, DQSCAL1_RANKINCTL_PHY);
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_RKCFG), u4XRTR2R, RKCFG_XRTR2R);  
    
    mcSHOW_DBG_MSG2(("TX_dly_DQSgated check: min %d  max %d,  s1ChangeDQSINCTL=%d\n", u1TXDLY_Cal_min, u1TXDLY_Cal_max, s1ChangeDQSINCTL));
    mcSHOW_DBG_MSG2(("DQSINCTL=%d, RANKINCTL=%d, u4XRTR2R=%d\n", u4ReadDQSINCTL, u4RankINCTL_ROOT, u4XRTR2R));       
}
#endif


#if GATING_ADJUST_TXDLY_FOR_TRACKING
DRAM_STATUS_T DramcRxdqsGatingPreProcess(DRAMC_CTX_T *p)
{
    u1TXDLY_Cal_min =0xff;
    u1TXDLY_Cal_max=0; 
}
#endif


#if 0
DRAM_STATUS_T DramcRxdqsGatingCal_Manual(DRAMC_CTX_T *p, S16 iDelay)
{
#if fcFOR_CHIP_ID == fcA60501

    U32 u4value;
    U32 u4Large, u4Small, u4PI;
    U32 u4Large_P1, u4Small_P1;
    U8  u1Datlat;
    #if GATING_RODT_LATANCY_EN
    U32 ucbest_coarse_large_RODT, ucbest_coarse_0p5T_RODT;
    #endif

    #if 0
    if((p->frequency != 800) &&(p->frequency != 1200)&&(p->frequency != 1600) && (p->frequency != 2133))
    {
        mcSHOW_DBG_MSG(("\n[DramcRxdqsGatingCal_Manual]   Not support Freq %d !!!!\n", p->frequency));
        mcFPRINTF((fp_A60501, "\n[DramcRxdqsGatingCal_Manual]   Begin %d \n !!!!!", p->frequency));
        return DRAM_FAIL;
    }
    #endif 
    mcSHOW_DBG_MSG(("\n======[DramcRxdqsGatingCal]   Freq %d=====\n", p->frequency));
    mcFPRINTF((fp_A60501, "\n======[DramcRxdqsGatingCal]   Begin %d =====\n", p->frequency));

    #if ENABLE_LP4_SW
    if (p->dram_type ==TYPE_LPDDR4)
    {
        if(p->frequency==800)
        {
            u4Large = 0;
            u4Small = 3;
            u4PI    = 8;
            u1Datlat = 0xa;
        }
        else if(p->frequency ==1200)
        {
            u4Large = 2;
            u4Small = 6;
            u4PI    = 0;
            u1Datlat = 0xb;
        }
        else if(p->frequency ==1866)
        {
            u4Large = 3;
            u4Small = 2;
            u4PI    = 24;
            u1Datlat = 0xc;
        }
        else if(p->frequency ==2133)
        {
            #if 0
            u4Large = 5;
            u4Small = 4;
            u4PI    = 24;
            u1Datlat = 0xd;
            #else
            u4Large = 3;
            u4Small = 4;
            u4PI    = 28;
            u1Datlat = 0xf;

            #endif
        }
        //For A60501 bootup
        else if(p->frequency ==400)
        {
            u4Large = 2;
            u4Small = 0;
            u4PI    = 8;
            u1Datlat = 0xA;
        }
        else// if(p->frequency ==1600)
        {
            #if 1
            u4Large = 3;
            u4Small = 2;
            u4PI    = 20;
            u1Datlat = 0xD;
            #else
            u4Large = 5;
            u4Small = 0;  //0;
            u4PI    = 24;
            u1Datlat = 0xd;
            #endif
        }
    }
    else//LPDDR3
    #endif
    {
    #if 0
        u4Large = 0;
        u4Small = 3;
        u4PI    = 24;
        u1Datlat = 0xc;
        #endif
        #if 0 //400 HMz
        u4Large = 0;
        u4Small = 0;
        u4PI    = 24;
        u1Datlat = 0xb;
        #endif

        u4PI = iDelay % 32;
        u4Small =  (iDelay /32) % 4;
        u4Large = (iDelay /32) / 4;
    }

    DramPhyCGReset(p, 1);// need to reset when UI update or PI change >=2

    // 4T or 2T coarse tune
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH2), u4Large, SELPH2_TXDLY_DQSGATE);  //DQS0_P0
    vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH12), \
                                P_Fld((U32) u4Large, SELPH12_TX_DLY_DQS1_GATED)| \
                                P_Fld((U32) u4Large, SELPH12_TX_DLY_DQS2_GATED)| \
                                P_Fld((U32) u4Large, SELPH12_TX_DLY_DQS3_GATED));
    // 0.5T coarse tune
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH5), (u4Small& 0x3), SELPH5_DLY_DQSGATE); 
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH21), (u4Small>>2), SELPH21_DLY_DQSGATE_2); 
    vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH13), \
                                P_Fld((U32) u4Small, SELPH13_REG_DLY_DQS1_GATED)| \
                                P_Fld((U32) u4Small, SELPH13_REG_DLY_DQS2_GATED)| \
                                P_Fld((U32) u4Small, SELPH13_REG_DLY_DQS3_GATED));

    #if GATING_RODT_LATANCY_EN  
    // RODT = Byte 0 Gating - 8UI,
    ucbest_coarse_large_RODT =u4Large -1;
    ucbest_coarse_0p5T_RODT = u4Small;

    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH6_1), ucbest_coarse_large_RODT, SELPH6_1_TXDLY_RODTEN);  
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH6_1), ucbest_coarse_0p5T_RODT, SELPH6_1_DLY_RODTEN); 
    mcSHOW_DBG_MSG2(("best RODT delay(2T, 0.5T) = (%d, %d)\n", ucbest_coarse_large_RODT, ucbest_coarse_0p5T_RODT));
    mcFPRINTF((fp_A60501,"best RODT delay(2T, 0.5T) = (%d, %d)\n", ucbest_coarse_large_RODT, ucbest_coarse_0p5T_RODT));
    #endif

    // for P1, 
    if (p->dram_type ==TYPE_LPDDR4)
    {
        u4Small_P1 = u4Small+ 4;
        u4Large_P1 = u4Large + u4Small_P1/8;
        u4Small_P1 = u4Small_P1%8;
    }
    else  //LPDDR3
    {
        u4Small_P1 = u4Small+ 2;
        u4Large_P1 = u4Large + u4Small_P1/4;
        u4Small_P1 = u4Small_P1%4;
    }
    
    // 4T or 2T coarse tune for P1
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH2), u4Large_P1, SELPH2_TXDLY_DQSGATE_P1);//DQS0_P1
    vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH12), \
                                P_Fld((U32) u4Large_P1, SELPH12_TX_DLY_DQS1_GATED_P1)| \
                                P_Fld((U32) u4Large_P1, SELPH12_TX_DLY_DQS2_GATED_P1)| \
                                P_Fld((U32) u4Large_P1, SELPH12_TX_DLY_DQS3_GATED_P1));
    // 0.5T coarse tune for P1
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH5), (u4Small_P1 & 0x3), SELPH5_DLY_DQSGATE_P1); 
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH21), (u4Small_P1>>2), SELPH21_DLY_DQSGATE_P1_2); 
    vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH13), \
                                P_Fld((U32) u4Small_P1, SELPH13_REG_DLY_DQS1_GATED_P1)| \
                                P_Fld((U32) u4Small_P1, SELPH13_REG_DLY_DQS2_GATED_P1)| \
                                P_Fld((U32) u4Small_P1, SELPH13_REG_DLY_DQS3_GATED_P1));

    // Set Fine Tune Value to registers
    u4value = u4PI | (u4PI<<8)| (u4PI<<16)| (u4PI<<24);
    vIO32Write4B(DRAMC_REG_ADDR(DRAMC_REG_R0DQSIEN), u4value);  
    
    DramPhyCGReset(p, 0);  
    mcDELAY_US(1);//delay 2T
    //DramPhyReset(p);   //reset phy, reset read data counter 

    dle_factor_handler(p, u1Datlat, 0);
    DramPhyReset(p);    

    mcSHOW_DBG_MSG(("Gating(%d, %d ,%d)  Gating_P1(%d, %d ,%d)  DATLAT 0x%x \n", u4Large, u4Small, u4PI, u4Large_P1, u4Small_P1, u4PI, u1Datlat));
    mcFPRINTF((fp_A60501, "Gating(%d, %d ,%d)  Gating_P1(%d, %d ,%d)  DATLAT 0x%x \n", u4Large, u4Small, u4PI, u4Large_P1, u4Small_P1, u4PI, u1Datlat));
#endif //#if fcFOR_CHIP_ID == fcA60501
    return DRAM_OK;
}
#endif

//-------------------------------------------------------------------------
/** DramcRxWindowPerbitCal (v2 version)
 *  start the rx dqs perbit sw calibration.
 *  @param p                Pointer of context created by DramcCtxCreate.
 *  @retval status          (DRAM_STATUS_T): DRAM_OK or DRAM_FAIL 
 */
//-------------------------------------------------------------------------
//#define RX_VREF_RANGE_SEL              0  //MR12,OP[6]
// default RX vref is 0xe=14
#define RX_VREF_RANGE_BEGIN       0
#define RX_VREF_RANGE_END          0x3f // binary 110010 , to be review
#define RX_VREF_RANGE_STEP         4

static U32 DramcRxWinRDDQC(DRAMC_CTX_T *p)
{
    U32 u4Result, u4Response;
    U32 u4TimeCnt;

    u4TimeCnt = TIME_OUT_CNT;

    //MPC_RDDQC_SET_MRS
    //Write MR32 DQ calibration pattern A //skip it if you want default 5Ah
    //Write MR40 DQ calibration pattern B //skip it if you want default 3Ch
    //Write MR15 low byte inverter for DQ calibration (byte 0) //skip it if you want default 55h
    //Write MR20 uper byte inverter for DQ calibration (byte 1) //skip it if you want default aah
    //DramcModeRegWrite(p, 32, 0x86);  
    //DramcModeRegWrite(p, 40, 0xC9);        
    //DramcModeRegWrite(p, 32, 0xaa);        
    //DramcModeRegWrite(p, 32, 0xbb);        

    //Issue RD DQ calibration
    //R_DMRDDQCEN, 0x1E4[7]=1 for RDDQC,  R_DMRDDQCDIS, 0x1EC[26]=1 to stop RDDQC burst
    //Wait rddqc_response=1, (dramc_conf_nao, 0x3b8[31])
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 1, SPCMD_RDDQCEN);
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_PERFCTL0), 1, PERFCTL0_RDDQCDIS); 

    do
    {
        u4Response = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMDRESP), SPCMDRESP_RDDQC_RESPONSE);
        u4TimeCnt --;
        mcDELAY_US(1);
    }while((u4Response ==0) &&(u4TimeCnt>0));

    if(u4TimeCnt==0)//time out
    {
        mcSHOW_DBG_MSG(("[DramcRxWinRDDQC] u4Response fail (time out)\n"));
        mcFPRINTF((fp_A60501, "[DramcRxWinRDDQC] u4Response fail (time out)\n"));
        //return DRAM_FAIL;
    }

    //Then read RDDQC compare result (dramc_conf_nao, 0x36c)
    u4Result = u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_RDQC_CMP));
    //R_DMRDDQCEN, 0x1E4[7]=0
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 0, SPCMD_RDDQCEN);
    //vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_PERFCTL0), 0, PERFCTL0_RDDQCDIS);

    return u4Result;
}

#if 0
DRAM_STATUS_T DramcRxWindowPerbitCal_Manual(DRAMC_CTX_T *p, U8 u1UseTestEngine, S16 iDelay)
{
#if fcFOR_CHIP_ID == fcA60501

    U8 ii;//, u1BitIdx, u1ByteIdx;
    U32 u4value, u4err_value;
    U32 u4AddrOfst = 0x60;

    if((iDelay <-127) || (iDelay >31))
    {
        mcSHOW_DBG_MSG(("\n\n Eror: out of boundary. \nPlease select TX DQ Delay (-127~31)\n\n"));
        return DRAM_FAIL;
    }

    mcSHOW_DBG_MSG(("\n=== DramcRxWindowPerbitCal_Manual Setting Begin ====\n"));

    if (iDelay <=0)
    {   
        mcSHOW_DBG_MSG(("\nRX manual DQS delay = %d\n",iDelay));
        
        // Adjust DQS output delay.
        #if ENABLE_LP4_SW
        if(p->dram_type == TYPE_LPDDR4)
        {    
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_RXDQ5), \
                                P_Fld(iDelay,RXDQ5_RK0_RX_ARDQS0_R_DLY) |P_Fld(-iDelay,RXDQ5_RK0_RX_ARDQS0_F_DLY));
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_RXB1_5), \
                                P_Fld(iDelay,RXB1_5_RK0_RX_ARDQS1_R_DLY) |P_Fld(-iDelay,RXB1_5_RK0_RX_ARDQS1_F_DLY));
        }
        else //LPDDR3
        #endif
        {
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ5), -iDelay, RXDQ5_RK0_RX_ARDQS0_R_DLY); 
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ5), -iDelay, RXDQ5_RK0_RX_ARDQS0_F_DLY);             
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXB1_5), -iDelay, RXB1_5_RK0_RX_ARDQS1_R_DLY); 
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXB1_5),-iDelay, RXB1_5_RK0_RX_ARDQS1_F_DLY); 
        }

        DramPhyReset(p);
    }

    if (iDelay >=0)
    {
        mcSHOW_DBG_MSG(("\nRX manual DQ delay = %d\n",iDelay));

        // Adjust DQM output delay.
        #if ENABLE_LP4_SW
        if(p->dram_type == TYPE_LPDDR4)
        {    
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_RXDQ5), \
                                P_Fld(iDelay,RXDQ5_RK0_RX_ARDQM0_R_DLY) |P_Fld(iDelay,RXDQ5_RK0_RX_ARDQM0_F_DLY));
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_RXB1_5), \
                                P_Fld(iDelay,RXB1_5_RK0_RX_ARDQM1_R_DLY) |P_Fld(iDelay,RXB1_5_RK0_RX_ARDQM1_F_DLY));
        }
        else
        #endif
        {
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ5), iDelay, RXDQ5_RK0_RX_ARDQM0_R_DLY); 
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ5), iDelay, RXDQ5_RK0_RX_ARDQM0_F_DLY);             
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXB1_5), iDelay, RXB1_5_RK0_RX_ARDQM1_R_DLY); 
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXB1_5), iDelay, RXB1_5_RK0_RX_ARDQM1_F_DLY); 
        }

        // Adjust DQ output delay.
        u4value = ((U32) iDelay) | (((U32)iDelay)<<8) | (((U32)iDelay)<<16) | (((U32)iDelay)<<24);  

        for (ii=0; ii<4; ii++)
        {
            #if ENABLE_LP4_SW
            if(p->dram_type == TYPE_LPDDR4)
            {
                vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_RXDQ1+ ii*4), u4value);//DQ0~DQ7
                vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_RXDQ1+ u4AddrOfst +ii*4), u4value);//DQ8~DQ15

                #if 0//CS_SWAP_WORKAROUND
                if(u1CSSwapEnable) 
                {
                    vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_RXDQ6+ii*4), u4value);  //DQ0~DQ7
                    vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_RXDQ6+ u4AddrOfst +ii*4), u4value);//DQ8~DQ15
                }
                #endif
            }
            else//LPDDR3
            #endif
            {
                vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ1+ii*4), u4value, PHY_FLD_FULL);  //Rank0, DQ0~DQ7
                vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ1+ u4AddrOfst +ii*4), u4value, PHY_FLD_FULL);//Rank0, DQ8~DQ15
            }
        }
    }

    if(u1UseTestEngine)
    {
        u4err_value = TestEngineCompare(p);
    }
    else
    {
        u4err_value = DramcRxWinRDDQC(p);
    }
    mcSHOW_DBG_MSG(("Engine compare result = 0x%x\n",u4err_value));
    mcSHOW_DBG_MSG(("=== Setting Done ====\n"));

#endif //#if fcFOR_CHIP_ID == fcA60501
    return DRAM_OK;
}
#endif

#define CS_SWAP_WORKAROUND  0 // for LP4 RDDQC, LP3 doesn't need this

static void SetRxDqDqsDelay(DRAMC_CTX_T *p, S16 iDelay, U8 u1CSSwapEnable)
{
    U8 ii;
    U32 u4value;
    U32 u4AddrOfst = 0x60;
    
    if (iDelay <=0)
    {   
        // Set DQS delay          
        #if ENABLE_LP4_SW
        if(p->dram_type == TYPE_LPDDR4)
        {
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_RXDQ5), \
                                    P_Fld((-iDelay),RXDQ5_RK0_RX_ARDQS0_R_DLY) |P_Fld((-iDelay),RXDQ5_RK0_RX_ARDQS0_F_DLY));
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_RXB1_5), \
                            P_Fld(-iDelay,RXB1_5_RK0_RX_ARDQS1_R_DLY) |P_Fld(-iDelay,RXB1_5_RK0_RX_ARDQS1_F_DLY));

            #if CS_SWAP_WORKAROUND
            if(u1CSSwapEnable)  // need to set both Rank0 and Rank1
            {
                vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_RXDQ10), \
                                        P_Fld((-iDelay),RXDQ5_RK0_RX_ARDQS0_R_DLY) |P_Fld((-iDelay),RXDQ5_RK0_RX_ARDQS0_F_DLY));
                vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_RXRK1_B1_4), \
                                        P_Fld((-iDelay),RXRK1_B1_4_RK1_RX_ARDQS1_R_DLY) |P_Fld((-iDelay),RXRK1_B1_4_RK1_RX_ARDQS1_F_DLY));
            }
            #endif
        }
        else
        #endif
        {
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ5), -iDelay, RXDQ5_RK0_RX_ARDQS0_R_DLY); 
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ5), -iDelay, RXDQ5_RK0_RX_ARDQS0_F_DLY);             
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXB1_5), -iDelay, RXB1_5_RK0_RX_ARDQS1_R_DLY); 
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXB1_5), -iDelay, RXB1_5_RK0_RX_ARDQS1_F_DLY);             
        }

        DramPhyReset(p);
    }
    else
    {
        // Adjust DQM output delay.
        #if ENABLE_LP4_SW
        if(p->dram_type == TYPE_LPDDR4)
        {    
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_RXDQ5), \
                                P_Fld(iDelay,RXDQ5_RK0_RX_ARDQM0_R_DLY) |P_Fld(iDelay,RXDQ5_RK0_RX_ARDQM0_F_DLY));
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_RXB1_5), \
                                P_Fld(iDelay,RXB1_5_RK0_RX_ARDQM1_R_DLY) |P_Fld(iDelay,RXB1_5_RK0_RX_ARDQM1_F_DLY));
        }
        else
        #endif
        {
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ5), iDelay, RXDQ5_RK0_RX_ARDQM0_R_DLY); 
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ5), iDelay, RXDQ5_RK0_RX_ARDQM0_F_DLY);             
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXB1_5), iDelay, RXB1_5_RK0_RX_ARDQM1_R_DLY); 
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXB1_5), iDelay, RXB1_5_RK0_RX_ARDQM1_F_DLY); 
        }

        // Adjust DQ output delay.
        u4value = ((U32) iDelay) | (((U32)iDelay)<<8) | (((U32)iDelay)<<16) | (((U32)iDelay)<<24);  

        for (ii=0; ii<4; ii++)
        {
            #if ENABLE_LP4_SW
            if(p->dram_type == TYPE_LPDDR4)
            {
                vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_RXDQ1+ ii*4), u4value);//DQ0~DQ7
                vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_RXDQ1+ u4AddrOfst +ii*4), u4value);//DQ8~DQ15

                #if CS_SWAP_WORKAROUND
                if(u1CSSwapEnable) 
                {
                    vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_RXDQ6+ii*4), u4value);  //DQ0~DQ7
                    vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_RXDQ6+ u4AddrOfst +ii*4), u4value);//DQ8~DQ15
                }
                #endif
            }
            else//LPDDR3
            #endif
            {            
                vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ1+ii*4), u4value, PHY_FLD_FULL);  //Rank0, DQ0~DQ7
                vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ1+ u4AddrOfst +ii*4), u4value, PHY_FLD_FULL);//Rank0, DQ8~DQ15
            }
        }
    }
}


DRAM_STATUS_T DramcRxWindowPerbitCal(DRAMC_CTX_T *p, U8 u1UseTestEngine)
{
    U8 ii, u1BitIdx, u1ByteIdx;
    U8 ucbit_first, ucbit_last;
    S16 iDelay, u4DelayBegin, u4DelayEnd, u4DelayStep=1;
    U32 uiFinishCount;
    U32 u4value, u4err_value, u4fail_bit;
    PASS_WIN_DATA_T WinPerBit[DQ_DATA_WIDTH], FinalWinPerBit[DQ_DATA_WIDTH];
    S32 iDQSDlyPerbyte[DQS_NUMBER];//, iFinalDQSDly[DQS_NUMBER];
    U8 u1VrefScanEnable;
    U16 u2VrefLevel, u2TempWinSum, u2FinalVref=0xe;
    U16 u2VrefBegin, u2VrefEnd, u2VrefStep;
    U32 u4fail_bit_R, u4fail_bit_F;
    U32 u4AddrOfst = 0x60;
    U8 u1CSSwapEnable;

    U16 u2WinSize;
    #if ENABLE_CLAIBTAION_WINDOW_LOG_FOR_FT
    U16 u2MinWinSize = 0xffff;
    U8 u1MinWinSizeBitidx;
    #endif
    
#if REG_ACCESS_PORTING_DGB  
    RegLogEnable =1;
    mcSHOW_DBG_MSG(("\n[REG_ACCESS_PORTING_FUNC]   DramcRxWindowPerbitCal\n"));
    mcFPRINTF((fp_A60501, "\n[REG_ACCESS_PORTING_FUNC]   DramcRxWindowPerbitCal\n"));
#endif
    
    // error handling
    if (!p)
    {
        mcSHOW_ERR_MSG(("context is NULL\n"));
        return DRAM_FAIL;
    }    

    // for debug
    #ifdef ETT_PRINT_FORMAT
    mcSHOW_DBG_MSG2(("\nRX DQS per bit para : test pattern=%d, test2_1=0x%X, test2_2=0x%X, LSW_HSR=%d\n", p->test_pattern, p->test2_1, p->test2_2, p->fglow_freq_write_en));    
    #else
    mcSHOW_DBG_MSG2(("\nRX DQS per bit para : test pattern=%d, test2_1=0x%8x, test2_2=0x%8x, LSW_HSR=%d\n", p->test_pattern, p->test2_1, p->test2_2, p->fglow_freq_write_en));    
    #endif   

    // 1.delay DQ ,find the pass widnow (left boundary).
    // 2.delay DQS find the pass window (right boundary). 
    // 3.Find the best DQ / DQS to satify the middle value of the overall pass window per bit
    // 4.Set DQS delay to the max per byte, delay DQ to de-skew

    mcSHOW_DBG_MSG2(("------------------------------------------------------\n"));     
    mcSHOW_DBG_MSG(("[DramcRxWindowPerbitCal] Frequency=%d, Channel=%d, Rank=%d\n", p->frequency, p->channel, p->rank));
    mcSHOW_DBG_MSG2(("Start DQ delay to find pass range, UseTestEngine =%d\n", u1UseTestEngine));
    mcSHOW_DBG_MSG2(("------------------------------------------------------\n")); 
    mcSHOW_DBG_MSG2(("x-axis is bit #; y-axis is DQ delay (%d~%d)\n", (-MAX_RX_DQSDLY_TAPS), MAX_RX_DQDLY_TAPS));

    mcFPRINTF((fp_A60501, "Start RX DQ/DQS calibration...UseTestEngine =%d\n", u1UseTestEngine));
    mcFPRINTF((fp_A60501, "x-axis is bit #; y-axis is DQ delay\n"));

    // for loop, different Vref,
    u2rx_window_sum = 0;

    #if CS_SWAP_WORKAROUND
    // mutli rank & swap enable & use RDDQC
    u1CSSwapEnable = (u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_RKCFG), RKCFG_RKMODE)) && \
                                 (u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_RKCFG), RKCFG_RKSWAP)) && \
                                 (u1UseTestEngine==0); 
    #else
    u1CSSwapEnable =0;
    #endif
    
    if((p->dram_type == TYPE_LPDDR4) && (u1UseTestEngine==1) && (p->enable_rx_scan_vref==ENABLE))
        u1VrefScanEnable =1;
    else
        u1VrefScanEnable =0;
    
    if(u1VrefScanEnable)
    {
        u2VrefBegin =RX_VREF_RANGE_BEGIN;
        u2VrefEnd =RX_VREF_RANGE_END;
        u2VrefStep=RX_VREF_RANGE_STEP;
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE2), 1, EYE2_RG_RX_ARDQ_VREF_EN_B0);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), 1, EYEB1_2_RG_RX_ARDQ_VREF_EN_B1);
    }
    else //LPDDR3 or diable RX Vref
    {
        u2VrefBegin = 0;
        u2VrefEnd = 1; // SEL[4:0] = 01110
        u2VrefStep =1; //don't care, just make for loop break;
    }

    //defult set result fail. When window found, update the result as oK
    if(u1UseTestEngine)
        vSetCalibrationResult(p, DRAM_CALIBRATION_RX_PERBIT, DRAM_FAIL);  
    else
        vSetCalibrationResult(p, DRAM_CALIBRATION_RX_RDDQC, DRAM_FAIL);  

    for(u2VrefLevel = u2VrefBegin; u2VrefLevel < u2VrefEnd; u2VrefLevel += u2VrefStep)
    {
        if(u1VrefScanEnable ==1)
        {
            mcSHOW_DBG_MSG(("\n======  RX VrefLevel=%d  ========\n", u2VrefLevel));
            mcFPRINTF((fp_A60501, "\n====== RX VrefLevel=%d ====== \n", u2VrefLevel));

            //Set RX Vref Here
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE2), u2VrefLevel, EYE2_RG_RX_ARDQ_VREF_SEL_B0);
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), u2VrefLevel, EYEB1_2_RG_RX_ARDQ_VREF_SEL_B1);
        }
        else
        {
            mcSHOW_DBG_MSG(("\n\n======  RX Vref Scan disable ========\n"));
            mcFPRINTF((fp_A60501, "\n\n====== RX Vref Scan disable ====== \n"));
        }
        
        // initialize parameters
        u2TempWinSum =0;
        uiFinishCount =0;
        
        for (u1BitIdx = 0; u1BitIdx < p->data_width; u1BitIdx++)
        {
            WinPerBit[u1BitIdx].first_pass = (S16)PASS_RANGE_NA;
            WinPerBit[u1BitIdx].last_pass = (S16)PASS_RANGE_NA;            
            FinalWinPerBit[u1BitIdx].first_pass = (S16)PASS_RANGE_NA;
            FinalWinPerBit[u1BitIdx].last_pass = (S16)PASS_RANGE_NA;
        }

        // Adjust DQM output delay to 0
        #if ENABLE_LP4_SW
        if(p->dram_type == TYPE_LPDDR4)
        {
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_RXDQ5), \
                                P_Fld(0,RXDQ5_RK0_RX_ARDQM0_R_DLY) |P_Fld(0,RXDQ5_RK0_RX_ARDQM0_F_DLY));
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_RXB1_5), \
                                P_Fld(0,RXB1_5_RK0_RX_ARDQM1_R_DLY) |P_Fld(0,RXB1_5_RK0_RX_ARDQM1_F_DLY));
        }
        else
        #endif
        {
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ5), 0, RXDQ5_RK0_RX_ARDQM0_R_DLY); 
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ5), 0, RXDQ5_RK0_RX_ARDQM0_F_DLY);             
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXB1_5), 0, RXB1_5_RK0_RX_ARDQM1_R_DLY); 
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXB1_5), 0, RXB1_5_RK0_RX_ARDQM1_F_DLY); 
        }

        // Adjust DQ output delay to 0
        //every 2bit dq have the same delay register address
        for (ii=0; ii<4; ii++)
        {
            #if ENABLE_LP4_SW
            if(p->dram_type == TYPE_LPDDR4)
            {
                vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_RXDQ1+ii*4), 0);  //DQ0~DQ7
                vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_RXDQ1+ u4AddrOfst +ii*4), 0);//DQ8~DQ15

                #if CS_SWAP_WORKAROUND
                if(u1CSSwapEnable)  // need to set both Rank0 and Rank1
                {
                    vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_RXDQ6+ii*4), 0);  //DQ0~DQ7
                    vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_RXDQ6+ u4AddrOfst +ii*4), 0);//DQ8~DQ15
                }
                #endif
            }
            else //LPDDR3
            #endif
            {
                vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ1+ii*4), 0, PHY_FLD_FULL);  //Rank0, DQ0~DQ7
                vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ1+ u4AddrOfst +ii*4), 0, PHY_FLD_FULL);//Rank0, DQ8~DQ15
            }
        }

        #if ENABLE_LP4_SW
        if(p->dram_type == TYPE_LPDDR4)
        {
            ///TODO: Fix me, change to define.
            if(p->frequency > HIGH_FREQ)
            {
                u4DelayBegin= -127; //4266 test speed up
            }
            else if(p->frequency >=1600)
            {
                u4DelayBegin= -5; //4266 test speed up
            }
            else if(p->frequency >=1200)
            {
                u4DelayBegin= -30; //4266 test speed up
            }
            else if(p->frequency >=800)
            {
                u4DelayBegin= -65; //4266 test speed up
            }
            else if(p->frequency >=400)
            {
                u4DelayBegin= -127;
            }
            else
            {
                u4DelayBegin= -RX_DQS_DELAY_BEGIN_LP4;
            }
            u4DelayEnd = RX_DQ_DELAY_END_LP4;
        }
        else
        #endif
        {
        
        #if SUPPORT_LP3_800
        if(p->frequency <=400) 
            u4DelayBegin= -(RX_DQS_DELAY_BEGIN_LP3*1.5);
        else
        #endif
            u4DelayBegin= -RX_DQS_DELAY_BEGIN_LP3;
        
            u4DelayEnd = RX_DQ_DELAY_END_LP3;

            if(p->frequency <600) 
                u4DelayStep =2;  //1066
            else 
                u4DelayStep =1;//1600, 1270
        }

        for (iDelay=u4DelayBegin; iDelay<=u4DelayEnd; iDelay+= u4DelayStep)
        //for (iDelay=u4DelayBegin; iDelay<=u4DelayEnd; iDelay+=3)
        {

            SetRxDqDqsDelay(p, iDelay, u1CSSwapEnable);

            if(u1UseTestEngine)
            {
                u4err_value = TestEngineCompare(p);
            }
            else
            {
                u4err_value = DramcRxWinRDDQC(p);
            }

            #ifdef ETT_PRINT_FORMAT
            if(u4err_value !=0)
            {
                mcSHOW_DBG_MSG2(("%d, [0]", iDelay));
            }
            #else
            mcSHOW_DBG_MSG2(("iDelay= %4d, [0]", iDelay));
            #endif
            mcFPRINTF((fp_A60501, "iDelay= %4d, [0]", iDelay));
            //mcSHOW_DBG_MSG2(("u4err_value %x, u1MRRValue %x\n", u4err_value, u1MRRValue));
            //mcFPRINTF((fp_A60501, "u4err_value!!  %x", u4err_value));

            // check fail bit ,0 ok ,others fail
            for (u1BitIdx = 0; u1BitIdx < p->data_width; u1BitIdx++)
            {
                u4fail_bit = u4err_value&((U32)1<<u1BitIdx);

                if(WinPerBit[u1BitIdx].first_pass== PASS_RANGE_NA)
                {
                    if(u4fail_bit==0) //compare correct: pass
                    {
                        WinPerBit[u1BitIdx].first_pass = iDelay;
                    }
                }
                else if(WinPerBit[u1BitIdx].last_pass == PASS_RANGE_NA)
                {
                    //mcSHOW_DBG_MSG(("fb%d \n", u4fail_bit));

                    if(u4fail_bit !=0) //compare error : fail
                    {
                        WinPerBit[u1BitIdx].last_pass  = (iDelay-1);
                    }
                    else if (iDelay==u4DelayEnd)
                    {
                        WinPerBit[u1BitIdx].last_pass  = iDelay;
                    }

                    if(WinPerBit[u1BitIdx].last_pass  !=PASS_RANGE_NA)
                    {
                        if((WinPerBit[u1BitIdx].last_pass -WinPerBit[u1BitIdx].first_pass) >= (FinalWinPerBit[u1BitIdx].last_pass -FinalWinPerBit[u1BitIdx].first_pass))
                        { 
                            #if 0 //for debug    
                            if(FinalWinPerBit[u1BitIdx].last_pass != PASS_RANGE_NA)
                            {
                                mcSHOW_DBG_MSG2(("Bit[%d] Bigger window update %d > %d\n", u1BitIdx, \
                                    (WinPerBit[u1BitIdx].last_pass -WinPerBit[u1BitIdx].first_pass), (FinalWinPerBit[u1BitIdx].last_pass -FinalWinPerBit[u1BitIdx].first_pass)));
                                mcFPRINTF((fp_A60501,"Bit[%d] Bigger window update %d > %d\n", u1BitIdx, \
                                    (WinPerBit[u1BitIdx].last_pass -WinPerBit[u1BitIdx].first_pass), (FinalWinPerBit[u1BitIdx].last_pass -FinalWinPerBit[u1BitIdx].first_pass)));
                            }
                            #endif
                            uiFinishCount |= (1<<u1BitIdx);
                            //update bigger window size 
                            FinalWinPerBit[u1BitIdx].first_pass = WinPerBit[u1BitIdx].first_pass;
                            FinalWinPerBit[u1BitIdx].last_pass = WinPerBit[u1BitIdx].last_pass;
                        }
                        //reset tmp window
                        WinPerBit[u1BitIdx].first_pass = PASS_RANGE_NA;
                        WinPerBit[u1BitIdx].last_pass = PASS_RANGE_NA;
                    }
                }

                #ifdef ETT_PRINT_FORMAT
                if(u4err_value !=0)
                #endif
                {
                    if(u1BitIdx%DQS_BIT_NUMBER ==0)
                    {
                        mcSHOW_DBG_MSG2((" "));
                        mcFPRINTF((fp_A60501, " "));
                    }

                    if (u4fail_bit == 0)
                    {
                        mcSHOW_DBG_MSG2(("o"));
                        mcFPRINTF((fp_A60501, "o"));
                    }
                    else
                    {
                    #if fcFOR_CHIP_ID == fcA60501
                        if (u1UseTestEngine && p->test_pattern == TEST_TESTPAT4_3)
                        {
                            u4fail_bit_R = (p->ta43_result[0] | p->ta43_result[2])&((U32)1<<ii);
                            u4fail_bit_F = (p->ta43_result[1] | p->ta43_result[3])&((U32)1<<ii);
                            if ((u4fail_bit_R == 0) && (u4fail_bit_F != 0))
                            {
                                // Rising pass, falling fail
                                mcSHOW_DBG_MSG2(("r"));
                                mcFPRINTF((fp_A60501, "r"));
                            }
                            else if ((u4fail_bit_R != 0) && (u4fail_bit_F == 0))
                            {
                                // Rising fail, falling pass
                                mcSHOW_DBG_MSG2(("f"));
                                mcFPRINTF((fp_A60501, "f"));
                            }
                            else                
                            {
                                mcSHOW_DBG_MSG2(("x"));
                                mcFPRINTF((fp_A60501, "x"));
                            }
                        }
                        else
                    #endif
                        {
                            mcSHOW_DBG_MSG2(("x"));
                            mcFPRINTF((fp_A60501, "x"));
                        }
                    }
                }
            }
            
            #ifdef ETT_PRINT_FORMAT
            if(u4err_value !=0)
            #endif
            {
                mcSHOW_DBG_MSG2((" [MSB]\n"));
                mcFPRINTF((fp_A60501, " [MSB]\n"));
            }

        //if all bits widnow found and all bits turns to fail again, early break;
        if(((p->dram_type == TYPE_LPDDR4) &&(uiFinishCount == 0xffff)) || \
            ((p->dram_type == TYPE_LPDDR3) &&(uiFinishCount == 0xffffffff)))
            {
                if(u1UseTestEngine)
                    vSetCalibrationResult(p, DRAM_CALIBRATION_RX_PERBIT, DRAM_OK);  
                else
                    vSetCalibrationResult(p, DRAM_CALIBRATION_RX_RDDQC, DRAM_OK);    
                
                if(u1VrefScanEnable ==0)
                {
                    if(((p->dram_type == TYPE_LPDDR4) &&(u4err_value == 0xffff)) || \
                        ((p->dram_type == TYPE_LPDDR3) &&(u4err_value == 0xffffffff)))
                                break;  //early break
                }
            }
        }

        mcSHOW_DBG_MSG(("RX CH%d R%d\n", p->channel, p->rank));
        
        for (u1BitIdx = 0; u1BitIdx < p->data_width; u1BitIdx++)
        {
            u2WinSize = FinalWinPerBit[u1BitIdx].last_pass-FinalWinPerBit[u1BitIdx].first_pass;
            u2TempWinSum += u2WinSize;  //Sum of CA Windows for vref selection
            
            // Everest : BU request RX & TX window size log.
            mcSHOW_DBG_MSG(("%d: %d\n", u1BitIdx, u2WinSize));
            
            #if ENABLE_CLAIBTAION_WINDOW_LOG_FOR_FT
            if(u2WinSize<u2MinWinSize)
            {
                u2MinWinSize = u2WinSize;
                u1MinWinSizeBitidx = u1BitIdx;
            }
            #endif 
        }
        
        #if ENABLE_CLAIBTAION_WINDOW_LOG_FOR_FT
        mcSHOW_DBG_MSG2(("FT log: RX min window : bit %d, size %d\n", u1MinWinSizeBitidx, u2MinWinSize));
        #endif    
        mcSHOW_DBG_MSG2(("\n\nRX Vref  %d, Window Sum %d > %d\n", u2VrefLevel, u2TempWinSum, u2rx_window_sum));
        mcFPRINTF((fp_A60501, "\n\n RX Vref  %d, Window Sum %d > %d\n", u2VrefLevel, u2TempWinSum, u2rx_window_sum));

        if(((p->dram_type == TYPE_LPDDR4) &&(u2TempWinSum <250)) || \
            ((p->dram_type == TYPE_LPDDR3) &&(u2TempWinSum <1100)))
        {
                mcSHOW_DBG_MSG2(("\n\n[NOTICE] Channel %d , TestEngine %d, RX_sum %d\n", p->channel, u1UseTestEngine, u2TempWinSum));
                mcFPRINTF((fp_A60501, "\n\n[NOTICE] Channel %d , TestEngine %d, RX_sum %d\n", p->channel, u1UseTestEngine, u2TempWinSum));
        }

        if(u2TempWinSum >u2rx_window_sum)
        {
            mcSHOW_DBG_MSG2(("\n\nBetter RX Vref found %d, Window Sum %d > %d\n", u2VrefLevel, u2TempWinSum, u2rx_window_sum));
            mcFPRINTF((fp_A60501, "\n\nBetter RX Vref found %d, Window Sum %d > %d\n", u2VrefLevel, u2TempWinSum, u2rx_window_sum));
            
            u2rx_window_sum =u2TempWinSum;
            u2FinalVref = u2VrefLevel;
            
             for (u1BitIdx=0; u1BitIdx<p->data_width; u1BitIdx++)
            {
                FinalWinPerBit[u1BitIdx].win_center = (FinalWinPerBit[u1BitIdx].last_pass + FinalWinPerBit[u1BitIdx].first_pass)>>1;     // window center of each DQ bit
                #ifdef ETT_PRINT_FORMAT
                mcSHOW_DBG_MSG(("Bit %d, %d (%d ~ %d)\n", u1BitIdx, FinalWinPerBit[u1BitIdx].win_center, FinalWinPerBit[u1BitIdx].first_pass, FinalWinPerBit[u1BitIdx].last_pass));
                #else
                mcSHOW_DBG_MSG(("Bit %2d, Center %3d (%4d ~ %4d)\n", u1BitIdx, FinalWinPerBit[u1BitIdx].win_center, FinalWinPerBit[u1BitIdx].first_pass, FinalWinPerBit[u1BitIdx].last_pass));
                #endif
                mcFPRINTF((fp_A60501, "Bit %2d, Center %3d (%4d ~ %4d)\n", u1BitIdx, FinalWinPerBit[u1BitIdx].win_center, FinalWinPerBit[u1BitIdx].first_pass, FinalWinPerBit[u1BitIdx].last_pass));
            }
        }
    }

    //Set RX Final Vref Here 
    if(u1VrefScanEnable==1)
    {
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE2), u2FinalVref, EYE2_RG_RX_ARDQ_VREF_SEL_B0);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), u2FinalVref, EYEB1_2_RG_RX_ARDQ_VREF_SEL_B1);
        mcSHOW_DBG_MSG(("\n\nFinal RX Vref %d\n", u2FinalVref));
        mcFPRINTF((fp_A60501, "\n\nFinal RX Vref %d\n", u2FinalVref));
    }

    // 3
    mcSHOW_DBG_MSG2(("------------------------------------------------------\n")); 
    mcSHOW_DBG_MSG2(("Start calculate dq time and dqs time / \n"));
    mcSHOW_DBG_MSG2(("Find max DQS delay per byte / Adjust DQ delay to align DQS...\n"));
    mcSHOW_DBG_MSG2(("------------------------------------------------------\n")); 

    mcFPRINTF((fp_A60501, "------------------------------------------------------\n")); 
    mcFPRINTF((fp_A60501, "Start calculate dq time and dqs time / \n"));
    mcFPRINTF((fp_A60501, "Find max DQS delay per byte / Adjust DQ delay to align DQS...\n"));
    mcFPRINTF((fp_A60501, "------------------------------------------------------\n")); 

    //As per byte, check max DQS delay in 8-bit. Except for the bit of max DQS delay, delay DQ to fulfill setup time = hold time
    for (u1ByteIdx = 0; u1ByteIdx < (p->data_width/DQS_BIT_NUMBER); u1ByteIdx++)
    {
        ucbit_first =DQS_BIT_NUMBER*u1ByteIdx;
        ucbit_last = DQS_BIT_NUMBER*u1ByteIdx+DQS_BIT_NUMBER-1;
        iDQSDlyPerbyte[u1ByteIdx] = MAX_RX_DQSDLY_TAPS;
        
        for (u1BitIdx = ucbit_first; u1BitIdx <= ucbit_last; u1BitIdx++)
        {
            // find out max Center value
            if(FinalWinPerBit[u1BitIdx].win_center < iDQSDlyPerbyte[u1ByteIdx])
                iDQSDlyPerbyte[u1ByteIdx] = FinalWinPerBit[u1BitIdx].win_center;

            //mcSHOW_DBG_MSG(("bit#%2d : center=(%2d)\n", u1BitIdx, FinalWinPerBit[u1BitIdx].win_center));
            //mcFPRINTF((fp_A60501, "bit#%2d : center=(%2d)\n", u1BitIdx, FinalWinPerBit[u1BitIdx].win_center));   
        }

        //mcSHOW_DBG_MSG(("----seperate line----\n"));
        //mcFPRINTF((fp_A60501, "----seperate line----\n"));

        if (iDQSDlyPerbyte[u1ByteIdx]  > 0)  // Delay DQS=0, Delay DQ only
        {
            iDQSDlyPerbyte[u1ByteIdx]  = 0;
        }
        else  //Need to delay DQS
        {
            iDQSDlyPerbyte[u1ByteIdx]  = -iDQSDlyPerbyte[u1ByteIdx] ;
        }

        // we delay DQ or DQS to let DQS sample the middle of rx pass window for all the 8 bits,
        for (u1BitIdx = ucbit_first; u1BitIdx <= ucbit_last; u1BitIdx++)
        {
            FinalWinPerBit[u1BitIdx].best_dqdly = iDQSDlyPerbyte[u1ByteIdx] + FinalWinPerBit[u1BitIdx].win_center;
        }
    }

    #if 1//A60501 RDDQC pin swap temp workaround
    if(u1UseTestEngine==0)
    {
        U8 u1TempIdx=0;
        iDQSDlyPerbyte[0] = iDQSDlyPerbyte[1];
        for (u1TempIdx = 0; u1TempIdx < p->data_width; u1TempIdx++)
        {
            FinalWinPerBit[u1TempIdx].best_dqdly = FinalWinPerBit[15].best_dqdly;
        }
    }
    #endif
       
    mcSHOW_DBG_MSG(("==================================================\n"));
    mcSHOW_DBG_MSG(("    dramc_rxdqs_perbit_swcal\n"));
    mcSHOW_DBG_MSG(("    channel=%d(1:cha) \n", p->channel));
    mcSHOW_DBG_MSG(("    bus width=%d\n", p->data_width));
    mcSHOW_DBG_MSG(("==================================================\n"));
    #ifdef ETT_PRINT_FORMAT
    mcSHOW_DBG_MSG(("DQS Delay :\nDQS0 = %d, DQS1 = %d, DQS2 = %d, DQS3 = %d\n", iDQSDlyPerbyte[0], iDQSDlyPerbyte[1], iDQSDlyPerbyte[2], iDQSDlyPerbyte[3]));
    #else
    mcSHOW_DBG_MSG(("DQS Delay :\nDQS0 = %2d, DQS1 = %2d, DQS2 = %2d, DQS3 = %2d\n", iDQSDlyPerbyte[0], iDQSDlyPerbyte[1], iDQSDlyPerbyte[2], iDQSDlyPerbyte[3]));
    #endif
    mcSHOW_DBG_MSG(("DQ Delay :\n"));

    mcFPRINTF((fp_A60501, "==================================================\n"));
    mcFPRINTF((fp_A60501, "    dramc_rxdqs_perbit_swcal\n"));
    mcFPRINTF((fp_A60501, "    channel=%d(1:cha) \n", p->channel));
    mcFPRINTF((fp_A60501, "    bus width=%d\n", p->data_width));
    mcFPRINTF((fp_A60501, "==================================================\n"));
    mcFPRINTF((fp_A60501, "DQS Delay :\n DQS0 = %2d DQS1 = %2d DQS2 = %2d DQS3 = %2d\n", iDQSDlyPerbyte[0], iDQSDlyPerbyte[1], iDQSDlyPerbyte[2], iDQSDlyPerbyte[3]));
    mcFPRINTF((fp_A60501, "DQ Delay :\n"));
    
    for (u1BitIdx = 0; u1BitIdx < p->data_width; u1BitIdx=u1BitIdx+4)
    {
        #ifdef ETT_PRINT_FORMAT
        mcSHOW_DBG_MSG(("DQ%d =%d, DQ%d =%d, DQ%d =%d, DQ%d =%d \n", u1BitIdx, FinalWinPerBit[u1BitIdx].best_dqdly, u1BitIdx+1, FinalWinPerBit[u1BitIdx+1].best_dqdly, u1BitIdx+2, FinalWinPerBit[u1BitIdx+2].best_dqdly, u1BitIdx+3, FinalWinPerBit[u1BitIdx+3].best_dqdly));
        #else
        mcSHOW_DBG_MSG(("DQ%2d =%2d, DQ%2d =%2d, DQ%2d =%2d, DQ%2d =%2d \n", u1BitIdx, FinalWinPerBit[u1BitIdx].best_dqdly, u1BitIdx+1, FinalWinPerBit[u1BitIdx+1].best_dqdly, u1BitIdx+2, FinalWinPerBit[u1BitIdx+2].best_dqdly, u1BitIdx+3, FinalWinPerBit[u1BitIdx+3].best_dqdly));
        #endif
        mcFPRINTF((fp_A60501, "DQ%2d =%2d, DQ%2d =%2d, DQ%2d =%2d, DQ%2d=%2d \n", u1BitIdx, FinalWinPerBit[u1BitIdx].best_dqdly, u1BitIdx+1, FinalWinPerBit[u1BitIdx+1].best_dqdly, u1BitIdx+2, FinalWinPerBit[u1BitIdx+2].best_dqdly, u1BitIdx+3, FinalWinPerBit[u1BitIdx+3].best_dqdly));
    }
    mcSHOW_DBG_MSG(("________________________________________________________________________\n"));
    mcFPRINTF((fp_A60501, "________________________________________________________________________\n"));

    #if REG_SHUFFLE_REG_CHECK
    ShuffleRegCheck =1;
    #endif
    
    // set dqs delay, (dqm delay)
    for (u1ByteIdx = 0; u1ByteIdx < (p->data_width/DQS_BIT_NUMBER); u1ByteIdx++)
    {
        #if ENABLE_LP4_SW
        if(p->dram_type == TYPE_LPDDR4)
        {
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_RXDQ5+ u4AddrOfst*u1ByteIdx), \
                                    P_Fld(((U32)iDQSDlyPerbyte[u1ByteIdx]),RXDQ5_RK0_RX_ARDQS0_R_DLY) |P_Fld(((U32)iDQSDlyPerbyte[u1ByteIdx]),RXDQ5_RK0_RX_ARDQS0_F_DLY));

            vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_RXDQ5+ u4AddrOfst*u1ByteIdx), \
                                    P_Fld(((U32)FinalWinPerBit[u1ByteIdx*8].best_dqdly),RXDQ5_RK0_RX_ARDQM0_R_DLY) |P_Fld(((U32)FinalWinPerBit[u1ByteIdx*8].best_dqdly),RXDQ5_RK0_RX_ARDQM0_F_DLY));

            #if CS_SWAP_WORKAROUND
            if(u1CSSwapEnable || (p->rank == RANK_1)) 
            {
                vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_RXDQ10+ u4AddrOfst*u1ByteIdx), \
                        P_Fld(((U32)iDQSDlyPerbyte[u1ByteIdx]),RXDQ10_RK1_RX_ARDQS0_R_DLY) |P_Fld(((U32)iDQSDlyPerbyte[u1ByteIdx]),RXDQ10_RK1_RX_ARDQS0_F_DLY));

                vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_RXDQ10+ u4AddrOfst*u1ByteIdx), \
                                    P_Fld(((U32)FinalWinPerBit[u1ByteIdx*8].best_dqdly),RXDQ10_RK1_RX_ARDQM0_R_DLY) |P_Fld(((U32)FinalWinPerBit[u1ByteIdx*8].best_dqdly),RXDQ10_RK1_RX_ARDQM0_F_DLY));
            }
            #endif
        }
        else //LPDDR3
        #endif
        {
            u4value = (((U32)iDQSDlyPerbyte[u1ByteIdx]) | ((U32)iDQSDlyPerbyte[u1ByteIdx] <<8) |\
                            ((U32)FinalWinPerBit[u1ByteIdx*8].best_dqdly <<16) |((U32)FinalWinPerBit[u1ByteIdx*8].best_dqdly <<24));

            vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_RXDQ5), u4value, PHY_FLD_FULL);

            if(p->rank == RANK_1) 
            {
                vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_RXDQ10), u4value, PHY_FLD_FULL);
            }
        }
    }

    #if REG_SHUFFLE_REG_CHECK
    ShuffleRegCheck =0;
    #endif

    DramPhyReset(p);

    #if REG_SHUFFLE_REG_CHECK
    ShuffleRegCheck =1;
    #endif
    
    // set dq delay  
    #if ENABLE_LP4_SW
    if(p->dram_type == TYPE_LPDDR4)
    {
        for (u1BitIdx=0; u1BitIdx<DQS_BIT_NUMBER; u1BitIdx+=2)
        {
             vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_RXDQ1+u1BitIdx*2), \
                                            P_Fld(((U32)FinalWinPerBit[u1BitIdx].best_dqdly),RXDQ1_RK0_RX_ARDQ0_R_DLY) |\
                                            P_Fld(((U32)FinalWinPerBit[u1BitIdx].best_dqdly),RXDQ1_RK0_RX_ARDQ0_F_DLY)| \
                                            P_Fld(((U32)FinalWinPerBit[u1BitIdx+1].best_dqdly),RXDQ1_RK0_RX_ARDQ1_R_DLY) |\
                                            P_Fld(((U32)FinalWinPerBit[u1BitIdx+1].best_dqdly),RXDQ1_RK0_RX_ARDQ1_F_DLY));

             vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_RXDQ1+u4AddrOfst +u1BitIdx*2), \
                                            P_Fld(((U32)FinalWinPerBit[u1BitIdx+8].best_dqdly),RXDQ1_RK0_RX_ARDQ0_R_DLY) | \
                                            P_Fld((U32)FinalWinPerBit[u1BitIdx+8].best_dqdly,RXDQ1_RK0_RX_ARDQ0_F_DLY)| \
                                            P_Fld(((U32)FinalWinPerBit[u1BitIdx+9].best_dqdly),RXDQ1_RK0_RX_ARDQ1_R_DLY) |\
                                            P_Fld((U32)FinalWinPerBit[u1BitIdx+9].best_dqdly,RXDQ1_RK0_RX_ARDQ1_F_DLY));

            #if CS_SWAP_WORKAROUND
            if(u1CSSwapEnable || (p->rank == RANK_1)) 
            {
                 vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_RXDQ6+u1BitIdx*2), \
                                                P_Fld(((U32)FinalWinPerBit[u1BitIdx].best_dqdly),RXDQ6_RK1_RX_ARDQ0_R_DLY) |\
                                                P_Fld(((U32)FinalWinPerBit[u1BitIdx].best_dqdly),RXDQ6_RK1_RX_ARDQ0_F_DLY)| \
                                                P_Fld(((U32)FinalWinPerBit[u1BitIdx+1].best_dqdly),RXDQ6_RK1_RX_ARDQ1_R_DLY) |\
                                                P_Fld(((U32)FinalWinPerBit[u1BitIdx+1].best_dqdly),RXDQ6_RK1_RX_ARDQ1_F_DLY));

                 vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_RXDQ6+u4AddrOfst +u1BitIdx*2), \
                                                P_Fld(((U32)FinalWinPerBit[u1BitIdx+8].best_dqdly),RXDQ6_RK1_RX_ARDQ0_R_DLY) | \
                                                P_Fld((U32)FinalWinPerBit[u1BitIdx+8].best_dqdly,RXDQ6_RK1_RX_ARDQ0_F_DLY)| \
                                                P_Fld(((U32)FinalWinPerBit[u1BitIdx+9].best_dqdly),RXDQ6_RK1_RX_ARDQ1_R_DLY) |\
                                                P_Fld((U32)FinalWinPerBit[u1BitIdx+9].best_dqdly,RXDQ6_RK1_RX_ARDQ1_F_DLY));
            }
            #endif

            //mcSHOW_DBG_MSG(("u1BitId %d  Addr 0x%2x = %2d %2d %2d %2d \n", u1BitIdx, DRAMC_REG_ADDR(DDRPHY_RXDQ1+u1BitIdx*2), \
//                FinalWinPerBit[u1BitIdx].best_dqdly, FinalWinPerBit[u1BitIdx+1].best_dqdly,  FinalWinPerBit[u1BitIdx+8].best_dqdly, FinalWinPerBit[u1BitIdx+9].best_dqdly));
        }
    }
    else //LPDDR3
    #endif
    {
        //every 2bit dq have the same delay register address
        for (u1ByteIdx=0; u1ByteIdx<DQS_NUMBER; u1ByteIdx++)
        {
             U8 u1BitIdx = u1ByteIdx *DQS_BIT_NUMBER;
             u4value = (FinalWinPerBit[u1BitIdx+1].best_dqdly) | ((FinalWinPerBit[u1BitIdx+1].best_dqdly)<<8) |\
                            (FinalWinPerBit[u1BitIdx].best_dqdly <<16) | ((FinalWinPerBit[u1BitIdx].best_dqdly)<<24);
             vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_RXDQ1), u4value, PHY_FLD_FULL);  //DQ0,1
             #if 0
            #ifdef ETT_PRINT_FORMAT
            mcSHOW_DBG_MSG(("Byte[%d, bit0&1] 0x%X\n",u1ByteIdx, u4value));
            #else
            mcSHOW_DBG_MSG(("Byte[%d, bit0&1] 0x%8x\n",u1ByteIdx, u4value));
            #endif
            #endif

             u4value = (FinalWinPerBit[u1BitIdx+3].best_dqdly) | ((FinalWinPerBit[u1BitIdx+3].best_dqdly)<<8) |\
                            (FinalWinPerBit[u1BitIdx+2].best_dqdly <<16) | ((FinalWinPerBit[u1BitIdx+2].best_dqdly)<<24);
             vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_RXDQ1+ 4), u4value, PHY_FLD_FULL);//DQ2,3
             #if 0
            #ifdef ETT_PRINT_FORMAT
            mcSHOW_DBG_MSG(("Byte[%d, bit2&3] 0x%X\n",u1ByteIdx, u4value));
            #else
            mcSHOW_DBG_MSG(("Byte[%d, bit2&3] 0x%8x\n",u1ByteIdx, u4value));
            #endif
            #endif

             u4value = ((U32)FinalWinPerBit[u1BitIdx+5].best_dqdly) | (((U32)FinalWinPerBit[u1BitIdx+5].best_dqdly)<<8) |\
                            ((U32)FinalWinPerBit[u1BitIdx+4].best_dqdly <<16) | (((U32)FinalWinPerBit[u1BitIdx+4].best_dqdly)<<24);
             vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_RXDQ1+ 8), u4value, PHY_FLD_FULL);//DQ4,5
             #if 0
            #ifdef ETT_PRINT_FORMAT
            mcSHOW_DBG_MSG(("Byte[%d, bit4&5] 0x%X\n",u1ByteIdx, u4value));
            #else
            mcSHOW_DBG_MSG(("Byte[%d, bit4&5] 0x%8x\n",u1ByteIdx, u4value));
            #endif
            #endif

             u4value = ((U32)FinalWinPerBit[u1BitIdx+7].best_dqdly) | (((U32)FinalWinPerBit[u1BitIdx+7].best_dqdly)<<8) |\
                            ((U32)FinalWinPerBit[u1BitIdx+6].best_dqdly <<16) | (((U32)FinalWinPerBit[u1BitIdx+6].best_dqdly)<<24);
             vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_RXDQ1+ 12), u4value, PHY_FLD_FULL);//DQ6,7
             #if 0
            #ifdef ETT_PRINT_FORMAT
            mcSHOW_DBG_MSG(("Byte[%d, bit6&7] 0x%X\n",u1ByteIdx, u4value));
            #else
            mcSHOW_DBG_MSG(("Byte[%d, bit6&7] 0x%8x\n",u1ByteIdx, u4value));
            #endif
            #endif

            if(p->rank == RANK_1) 
            {
                 u4value = (FinalWinPerBit[u1BitIdx+1].best_dqdly) | ((FinalWinPerBit[u1BitIdx+1].best_dqdly)<<8) |\
                                (FinalWinPerBit[u1BitIdx].best_dqdly <<16) | ((FinalWinPerBit[u1BitIdx].best_dqdly)<<24);
                 vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_RXDQ6), u4value, PHY_FLD_FULL);  //DQ0,1
                 #if 0
                #ifdef ETT_PRINT_FORMAT
                mcSHOW_DBG_MSG(("Byte[%d, bit0&1] 0x%X\n",u1ByteIdx, u4value));
                #else
                mcSHOW_DBG_MSG(("Byte[%d, bit0&1] 0x%8x\n",u1ByteIdx, u4value));
                #endif
                #endif

                 u4value = (FinalWinPerBit[u1BitIdx+3].best_dqdly) | ((FinalWinPerBit[u1BitIdx+3].best_dqdly)<<8) |\
                                (FinalWinPerBit[u1BitIdx+2].best_dqdly <<16) | ((FinalWinPerBit[u1BitIdx+2].best_dqdly)<<24);
                 vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_RXDQ6+ 4), u4value, PHY_FLD_FULL);//DQ2,3
                 #if 0
                #ifdef ETT_PRINT_FORMAT
                mcSHOW_DBG_MSG(("Byte[%d, bit2&3] 0x%X\n",u1ByteIdx, u4value));
                #else
                mcSHOW_DBG_MSG(("Byte[%d, bit2&3] 0x%8x\n",u1ByteIdx, u4value));
                #endif
                #endif

                 u4value = ((U32)FinalWinPerBit[u1BitIdx+5].best_dqdly) | (((U32)FinalWinPerBit[u1BitIdx+5].best_dqdly)<<8) |\
                                ((U32)FinalWinPerBit[u1BitIdx+4].best_dqdly <<16) | (((U32)FinalWinPerBit[u1BitIdx+4].best_dqdly)<<24);
                 vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_RXDQ6+ 8), u4value, PHY_FLD_FULL);//DQ4,5
                 #if 0
                #ifdef ETT_PRINT_FORMAT
                mcSHOW_DBG_MSG(("Byte[%d, bit4&5] 0x%X\n",u1ByteIdx, u4value));
                #else
                mcSHOW_DBG_MSG(("Byte[%d, bit4&5] 0x%8x\n",u1ByteIdx, u4value));
                #endif
                #endif

                 u4value = ((U32)FinalWinPerBit[u1BitIdx+7].best_dqdly) | (((U32)FinalWinPerBit[u1BitIdx+7].best_dqdly)<<8) |\
                                ((U32)FinalWinPerBit[u1BitIdx+6].best_dqdly <<16) | (((U32)FinalWinPerBit[u1BitIdx+6].best_dqdly)<<24);
                 vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_RXDQ6+ 12), u4value, PHY_FLD_FULL);//DQ6,7
                 #if 0
                #ifdef ETT_PRINT_FORMAT
                mcSHOW_DBG_MSG(("Byte[%d, bit6&7] 0x%X\n",u1ByteIdx, u4value));
                #else
                mcSHOW_DBG_MSG(("Byte[%d, bit6&7] 0x%8x\n",u1ByteIdx, u4value));
                #endif
                #endif
            }
        }
    }
    #if REG_SHUFFLE_REG_CHECK
    ShuffleRegCheck =0;
    #endif
    mcSHOW_DBG_MSG(("[DramcRxWindowPerbitCal] ====Done====\n"));
    mcFPRINTF((fp_A60501, "[DramcRxWindowPerbitCal] ====Done====\n"));    
    
#if REG_ACCESS_PORTING_DGB
    RegLogEnable =0;
#endif

return DRAM_OK;

    // Log example  ==> Neec to update
    /*
------------------------------------------------------
Start calculate dq time and dqs time /
Find max DQS delay per byte / Adjust DQ delay to align DQS...
------------------------------------------------------
bit# 0 : dq time=11 dqs time= 8
bit# 1 : dq time=11 dqs time= 8
bit# 2 : dq time=11 dqs time= 6
bit# 3 : dq time=10 dqs time= 8
bit# 4 : dq time=11 dqs time= 8
bit# 5 : dq time=10 dqs time= 8
bit# 6 : dq time=11 dqs time= 8
bit# 7 : dq time= 9 dqs time= 6
----seperate line----
bit# 8 : dq time=12 dqs time= 7
bit# 9 : dq time=10 dqs time= 8
bit#10 : dq time=11 dqs time= 8
bit#11 : dq time=10 dqs time= 8
bit#12 : dq time=11 dqs time= 8
bit#13 : dq time=11 dqs time= 8
bit#14 : dq time=11 dqs time= 8
bit#15 : dq time=12 dqs time= 8
----seperate line----
bit#16 : dq time=11 dqs time= 7
bit#17 : dq time=10 dqs time= 8
bit#18 : dq time=11 dqs time= 7
bit#19 : dq time=11 dqs time= 6
bit#20 : dq time=10 dqs time= 9
bit#21 : dq time=11 dqs time=10
bit#22 : dq time=11 dqs time=10
bit#23 : dq time= 9 dqs time= 9
----seperate line----
bit#24 : dq time=12 dqs time= 6
bit#25 : dq time=13 dqs time= 6
bit#26 : dq time=13 dqs time= 7
bit#27 : dq time=11 dqs time= 7
bit#28 : dq time=12 dqs time= 8
bit#29 : dq time=10 dqs time= 8
bit#30 : dq time=13 dqs time= 7
bit#31 : dq time=11 dqs time= 8
----seperate line----
==================================================
    dramc_rxdqs_perbit_swcal_v2
    channel=2(2:cha, 3:chb) apply = 1
==================================================
DQS Delay :
 DQS0 = 0 DQS1 = 0 DQS2 = 0 DQS3 = 0
DQ Delay :
DQ 0 =  1 DQ 1 =  1 DQ 2 =  2 DQ 3 =  1
DQ 4 =  1 DQ 5 =  1 DQ 6 =  1 DQ 7 =  1
DQ 8 =  2 DQ 9 =  1 DQ10 =  1 DQ11 =  1
DQ12 =  1 DQ13 =  1 DQ14 =  1 DQ15 =  2
DQ16 =  2 DQ17 =  1 DQ18 =  2 DQ19 =  2
DQ20 =  0 DQ21 =  0 DQ22 =  0 DQ23 =  0
DQ24 =  3 DQ25 =  3 DQ26 =  3 DQ27 =  2
DQ28 =  2 DQ29 =  1 DQ30 =  3 DQ31 =  1
_______________________________________________________________
   */
}


static void dle_factor_handler(DRAMC_CTX_T *p, U8 curr_val, U8 pip_num) 
{
#if REG_ACCESS_PORTING_DGB  
    RegLogEnable =1;
    mcSHOW_DBG_MSG(("\n[REG_ACCESS_PORTING_FUNC]   dle_factor_handler\n"));
    mcFPRINTF((fp_A60501, "\n[REG_ACCESS_PORTING_FUNC]   dle_factor_handler\n"));
#endif

    #if REG_SHUFFLE_REG_CHECK
    ShuffleRegCheck =1;
    #endif

    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_MISC), curr_val, MISC_DATLAT);
    #if ENABLE_LP4_SW
    if(p->dram_type ==TYPE_LPDDR4)
    {
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_MISC), curr_val, MISC_DATLAT_DSEL);
    }
    else
    #endif
    {
        if(curr_val<6)
            curr_val =6;
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_MISC), curr_val-DATLAT_DSEL_DIFFERENCE, MISC_DATLAT_DSEL);
    }
    
    #if REG_SHUFFLE_REG_CHECK
    ShuffleRegCheck =0;
    #endif

    DramPhyReset(p);
   
#if REG_ACCESS_PORTING_DGB
    RegLogEnable =0;
#endif
}

//-------------------------------------------------------------------------
/** Dramc_ta2_rx_scan
 */
//-------------------------------------------------------------------------
static U32 Dramc_ta2_rx_scan(DRAMC_CTX_T *p, U8 u1UseTestEngine)
{
    U32 ii, u4err_value;   
    S16 iDelay;
    U8 u1ByteIdx;
    U32 u4AddrOfst = 0x60;
    U32 u4RegBak_DDRPHY_RXDQ_RK0[DQS_NUMBER][5];  // rank 0
    U8 u1CSSwapEnable;
    #if CS_SWAP_WORKAROUND
    U32 u4RegBak_DDRPHY_RXDQ_RK1[DQS_NUMBER][5];  // rank 1
    #endif
    
#if CS_SWAP_WORKAROUND
        // mutli rank & swap enable & use RDDQC
        u1CSSwapEnable = (u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_RKCFG), RKCFG_RKMODE)) && \
                                     (u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_RKCFG), RKCFG_RKSWAP)) && \
                                     (u1UseTestEngine==0); 
#else
        u1CSSwapEnable =0;
#endif

        // reg backup
        for (ii=0; ii<5; ii++)
        {
            #if ENABLE_LP4_SW
            if(p->dram_type == TYPE_LPDDR4)
            {
                u4RegBak_DDRPHY_RXDQ_RK0[0][ii] =(u4IO32ReadFldAlign(DRAMC_REG_ADDR(DDRPHY_RXDQ1+ii*4), PHY_FLD_FULL)); // rank 0, byte 0   
                u4RegBak_DDRPHY_RXDQ_RK0[1][ii] =(u4IO32ReadFldAlign(DRAMC_REG_ADDR(DDRPHY_RXDQ1+ u4AddrOfst+ii*4), PHY_FLD_FULL)); // rank 0, byte 1  
                
                #if CS_SWAP_WORKAROUND
                if(u1CSSwapEnable)
                {
                    u4RegBak_DDRPHY_RXDQ_RK1[0][ii] = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DDRPHY_RXDQ6+ii*4), PHY_FLD_FULL);// rank 1, byte 0
                    u4RegBak_DDRPHY_RXDQ_RK1[1][ii] = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DDRPHY_RXDQ6+ u4AddrOfst +ii*4), PHY_FLD_FULL);// rank 0, byte 1
                }
                #endif
            }
            else //LPDDR3 , only record rank 0, will not modify rank1
            #endif
            {  
                for(u1ByteIdx=0; u1ByteIdx<4; u1ByteIdx++)
                {
                    u4RegBak_DDRPHY_RXDQ_RK0[u1ByteIdx][ii] =u4IO32ReadFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_RXDQ1+ii*4), PHY_FLD_FULL);
                }
            }
        }

        // Adjust DQM output delay to 0
        #if ENABLE_LP4_SW
        if(p->dram_type == TYPE_LPDDR4)
        {
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_RXDQ5), \
                                P_Fld(0,RXDQ5_RK0_RX_ARDQM0_R_DLY) |P_Fld(0,RXDQ5_RK0_RX_ARDQM0_F_DLY));
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_RXB1_5), \
                                P_Fld(0,RXB1_5_RK0_RX_ARDQM1_R_DLY) |P_Fld(0,RXB1_5_RK0_RX_ARDQM1_F_DLY));
        }
        else
        #endif
        {
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ5), 0, RXDQ5_RK0_RX_ARDQM0_R_DLY); 
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ5), 0, RXDQ5_RK0_RX_ARDQM0_F_DLY);             
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXB1_5), 0, RXB1_5_RK0_RX_ARDQM1_R_DLY); 
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXB1_5), 0, RXB1_5_RK0_RX_ARDQM1_F_DLY); 
        }

        // Adjust DQ output delay to 0
        //every 2bit dq have the same delay register address
        for (ii=0; ii<4; ii++)
        {
            #if ENABLE_LP4_SW
            if(p->dram_type == TYPE_LPDDR4)
            {
                vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_RXDQ1+ii*4), 0);  //DQ0~DQ7
                vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_RXDQ1+ u4AddrOfst +ii*4), 0);//DQ8~DQ15

                #if CS_SWAP_WORKAROUND
                if(u1CSSwapEnable)  // need to set both Rank0 and Rank1
                {
                    vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_RXDQ6+ii*4), 0);  //DQ0~DQ7
                    vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_RXDQ6+ u4AddrOfst +ii*4), 0);//DQ8~DQ15
                }
                #endif
            }
            else //LPDDR3
            #endif
            {
                vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ1+ii*4), 0, PHY_FLD_FULL);  //Rank0, DQ0~DQ7
                vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ1+ u4AddrOfst +ii*4), 0, PHY_FLD_FULL);//Rank0, DQ8~DQ15
            }
        }

       // quick rx dqs search
       //mcSHOW_DBG_MSG(("quick rx dqs search\n"));
       //mcFPRINTF((fp_A60817, "quick rx dqs search\n", iDelay));
       for (iDelay=-32; iDelay<=32; iDelay+=4)
       {
           //mcSHOW_DBG_MSG(("%2d, ", iDelay));
           //mcFPRINTF((fp_A60817,"%2d, ", iDelay));

           if(p->dram_type == TYPE_LPDDR4)
           {
               SetRxDqDqsDelay(p, iDelay, u1CSSwapEnable);
            }
           else
            {
                SetRxDqDqsDelay(p, iDelay, 0);
            }
       
           if(u1UseTestEngine)
           {
               u4err_value = TestEngineCompare(p);
           }
           else
           {
               u4err_value = DramcRxWinRDDQC(p);
           }

            if(u4err_value ==0)// rx dqs found.
                break; 
        }
    
        #ifdef ETT_PRINT_FORMAT
        mcSHOW_DBG_MSG(("RX DQS delay = %d, ", iDelay));
        #else
        mcSHOW_DBG_MSG(("RX DQS delay = %2d, ", iDelay));
        #endif
        mcFPRINTF((fp_A60501, "RX DQS delay = %2d, ", iDelay));
        
        // restore registers
        for (ii=0; ii<5; ii++)
        {
            #if ENABLE_LP4_SW
            if(p->dram_type == TYPE_LPDDR4)
            {
                vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_RXDQ1+ ii*4), u4RegBak_DDRPHY_RXDQ_RK0[0][ii] , PHY_FLD_FULL);
                vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_RXDQ1+ u4AddrOfst+ ii*4), u4RegBak_DDRPHY_RXDQ_RK0[1][ii] , PHY_FLD_FULL);
                
                #if CS_SWAP_WORKAROUND
                if(u1CSSwapEnable)  // need to set both Rank0 and Rank1
                {
                    vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_RXB1+ ii*4), u4RegBak_DDRPHY_RXDQ_RK1[0][ii] , PHY_FLD_FULL);
                    vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_RXB1+ u4AddrOfst+ ii*4), u4RegBak_DDRPHY_RXDQ_RK1[1][ii] , PHY_FLD_FULL);
                }
                #endif
            }
            else //LPDDR3
            #endif
            {  
            
                for(u1ByteIdx=0; u1ByteIdx<4; u1ByteIdx++)
                {
                     vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_RXDQ1+ ii*4), u4RegBak_DDRPHY_RXDQ_RK0[u1ByteIdx][ii] , PHY_FLD_FULL);                    
                }
            }
        }
        
        return u4err_value;    
}


static U8 aru1RxDatlatResult[CHANNEL_MAX][RANK_MAX];

U8 DramcRxdatlatScan(DRAMC_CTX_T *p, DRAM_DATLAT_CALIBRATION_TYTE_T use_rxtx_scan)
{
    U8 ii, ucStartCalVal=0;
    U32 u4prv_register_080;
    U32 u4err_value;//, u4Value;
    U8 ucfirst, ucbegin, ucsum, ucbest_step, ucpipe_num =0;    

    // error handling
    if (!p)
    {
        mcSHOW_ERR_MSG(("context is NULL\n"));
        return DRAM_FAIL;
    }
#if REG_ACCESS_PORTING_DGB  
    RegLogEnable =1;
    mcSHOW_DBG_MSG(("\n[REG_ACCESS_PORTING_FUNC]   DramcRxdatlatCal\n"));
    mcFPRINTF((fp_A60501, "\n[REG_ACCESS_PORTING_FUNC]   DramcRxdatlatCal\n"));
#endif

    mcSHOW_DBG_MSG2(("\n==============================================================\n"));
    mcSHOW_DBG_MSG(("[DATLAT] Frequency=%d, Channel=%d, Rank=%d, use_rxtx_scan=%d\n", p->frequency, p->channel, p->rank, use_rxtx_scan));
    mcSHOW_DBG_MSG2(("==============================================================\n"));

    mcFPRINTF((fp_A60501, "\n==============================================================\n"));
    mcFPRINTF((fp_A60501, "    DATLAT calibration \n"));
    mcFPRINTF((fp_A60501, "    channel=%d(1:cha), rank=%d, use_rxtx_scan=%d \n", p->channel, p->rank, use_rxtx_scan));
    mcFPRINTF((fp_A60501, "==============================================================\n"));

    // [11:10] DQIENQKEND 01 -> 00 for DATLAT calibration issue, DQS input enable will refer to DATLAT
    // if need to enable this (for power saving), do it after all calibration done
    //u4prv_register_0d8 = u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_MCKDLY));
    //vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_MCKDLY), P_Fld(0, MCKDLY_DQIENQKEND) | P_Fld(0, MCKDLY_DQIENLATEBEGIN));
    
    // pre-save
    // 0x07c[6:4]   DATLAT bit2-bit0
    u4prv_register_080 = u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_MISC));

    // init best_step to default
    ucbest_step = (U8) u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_MISC), MISC_DATLAT);
    mcSHOW_DBG_MSG(("DATLAT Default value = 0x%x\n", ucbest_step));
    mcFPRINTF((fp_A60501, "DATLAT Default value = 0x%x\n", ucbest_step));

    // 1.set DATLAT 0-15 (0-21 for MT6595)
    // 2.enable engine1 or engine2 
    // 3.check result  ,3~4 taps pass 
    // 4.set DATLAT 2nd value for optimal

    // Initialize
    ucfirst = 0xff;
    ucbegin = 0;
    ucsum = 0;        
             
    for (ii = 5; ii < DATLAT_TAP_NUMBER; ii++)
    {        
        // 1
        dle_factor_handler(p, ii, ucpipe_num);

        // 2
        /////Verified in A60817 LP3, LP4 is TODO!!!!!!!!!!!!!
        if(use_rxtx_scan == fcDATLAT_USE_DEFAULT)
            u4err_value = TestEngineCompare(p);
        else//fcDATLAT_USE_RX_SCAN
        {
            if(p->dram_type == TYPE_LPDDR4)
            {
                u4err_value = Dramc_ta2_rx_scan(p, 0);
            }
            else  //LPDDR3
            {
                u4err_value = Dramc_ta2_rx_scan(p, 1);
            }
        }

        // 3
        if (u4err_value == 0)
        {
            if (ucbegin == 0)
            {
                // first tap which is pass
                ucfirst = ii;
                ucbegin = 1;
            }
            if (ucbegin == 1)
            {
                ucsum++;

                if(ucsum >5)
                    break;  //early break.
            }
        }
        else
        {
            if (ucbegin == 1)
            {
                // pass range end
                ucbegin = 0xff;
            }
        }

        #ifdef ETT_PRINT_FORMAT
        mcSHOW_DBG_MSG(("%d, 0x%X, sum=%d\n", ii, u4err_value, ucsum));
        #else
        mcSHOW_DBG_MSG(("TAP=%2d, err_value=0x%8x,  sum=%d\n", ii, u4err_value, ucsum));
        #endif
        mcFPRINTF((fp_A60501, "TAP=%2d, err_value=0x%8x, begin=%d, first=%3d, sum=%d\n", ii, u4err_value, ucbegin, ucfirst, ucsum));
    }

    // 4
    if (ucsum == 0)
    {
        mcSHOW_ERR_MSG(("no DATLAT taps pass, DATLAT calibration fail!!\n"));
    } 
    else if (ucsum <= 3)
    {
        ucbest_step = ucfirst + (ucsum>>1);
    }
    else // window is larger htan 3
    {
        ucbest_step = ucfirst + 2;
    }    

    aru1RxDatlatResult[p->channel][p->rank] = ucbest_step;

    mcSHOW_DBG_MSG(("pattern=%d first_step=%d total pass=%d best_step=%d\n", p->test_pattern, ucfirst, ucsum, ucbest_step));
    mcFPRINTF((fp_A60501, "pattern=%d first_step=%d total pass=%d best_step=%d\n", p->test_pattern, ucfirst, ucsum, ucbest_step));

    if(ucsum <5)
    {
        mcSHOW_DBG_MSG2(("NOTICE] Channel %d , DatlatSum %d\n", p->channel, ucsum));
        mcFPRINTF((fp_A60501, "[NOTICE] Channel %d , DatlatSum  %d\n", p->channel, ucsum));
    }
        
    if (ucsum == 0)
    {
        mcSHOW_ERR_MSG(("DATLAT calibration fail, write back to default values!\n"));
        vIO32Write4B(DRAMC_REG_ADDR(DRAMC_REG_MISC), u4prv_register_080);
        vSetCalibrationResult(p, DRAM_CALIBRATION_DATLAT, DRAM_FAIL);  
    }
    else
    {
        dle_factor_handler(p, ucbest_step, ucpipe_num);
        vSetCalibrationResult(p, DRAM_CALIBRATION_DATLAT, DRAM_OK);  
    }

    // [11:10] DQIENQKEND 01 -> 00 for DATLAT calibration issue, DQS input enable will refer to DATLAT
    // if need to enable this (for power saving), do it after all calibration done    
    vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_MCKDLY), P_Fld(1, MCKDLY_DQIENQKEND) | P_Fld(1, MCKDLY_DQIENLATEBEGIN));

    mcSHOW_DBG_MSG(("[DramcRxdatlatCal] ====Done====\n"));
    mcFPRINTF((fp_A60501, "[DramcRxdatlatCal] ====Done====\n"));    

#if REG_ACCESS_PORTING_DGB
    RegLogEnable =0;
#endif

    return ucsum;
}

DRAM_STATUS_T DramcRxdatlatCal(DRAMC_CTX_T *p)
{
    U8 u1DatlatWindowSum;

    u1DatlatWindowSum = DramcRxdatlatScan(p, fcDATLAT_USE_DEFAULT);

    if(u1DatlatWindowSum <5)
    {
        mcSHOW_DBG_MSG(("\nu1DatlatWindowSum %d is too small(<5), Start RX + Datlat scan\n", u1DatlatWindowSum));
        DramcRxdatlatScan(p, fcDATLAT_USE_RX_SCAN);
    }
}

DRAM_STATUS_T DramcDualRankRxdatlatCal(DRAMC_CTX_T *p)
{
    U8 u1FinalDatlat, u1Datlat0, u1Datlat1;

    u1Datlat0 = aru1RxDatlatResult[p->channel][0];
    u1Datlat1 = aru1RxDatlatResult[p->channel][1];
    
    if(u1Datlat0> u1Datlat1)
    {
        u1FinalDatlat= u1Datlat0;
    }
    else
    {
        u1FinalDatlat= u1Datlat1;
    }

    dle_factor_handler(p, u1FinalDatlat, 3);
    mcSHOW_DBG_MSG(("[DramcDualRankRxdatlatCal] RANK0: %d, RANK1: %d, Final_Datlat %d\n", u1Datlat0, u1Datlat1, u1FinalDatlat));
    mcFPRINTF((fp_A60501, "[DramcDualRankRxdatlatCal] RANK0: %d, RANK1: %d, Final_Datlat %d\n", u1Datlat0, u1Datlat1, u1FinalDatlat));

    return DRAM_OK;

}
//-------------------------------------------------------------------------
/** DramcTxWindowPerbitCal (v2)
 *  TX DQS per bit SW calibration.
 *  @param p                Pointer of context created by DramcCtxCreate.
 *  @param  apply           (U8): 0 don't apply the register we set  1 apply the register we set ,default don't apply.
 *  @retval status          (DRAM_STATUS_T): DRAM_OK or DRAM_FAIL 
 */
//-------------------------------------------------------------------------
#define ENABLE_64_PI_TO_UI 1
#if ENABLE_64_PI_TO_UI
#define TX_DQ_UI_TO_PI_TAP         64 // 1 PI = tCK/64, total 128 PI, 1UI = 32 PI
#else
#define TX_DQ_UI_TO_PI_TAP         32 // 1 PI = tCK/64, total 128 PI, 1UI = 32 PI
#endif
#define TX_VREF_RANGE_BEGIN       0
#define TX_VREF_RANGE_END           50 // binary 110010
#define TX_VREF_RANGE_STEP         2

static void TxWinTransferDelayToUIPI(DRAMC_CTX_T *p, U16 uiDelay, U8 u1UISmallBak, U8 u1UILargeBak, U8* pu1UILarge, U8* pu1UISmall, U8* pu1PI)
{
    U8 u1Small_ui_to_large;
    U16 u2TmpValue;
    
    //in LP4, 8 small UI =  1 large UI
    //in LP3, 4 small UI =  1 large UI
    u1Small_ui_to_large = (p->dram_type ==TYPE_LPDDR4) ? 8 : 4; 

    *pu1PI = uiDelay% TX_DQ_UI_TO_PI_TAP;

    #if ENABLE_64_PI_TO_UI
    u2TmpValue = (uiDelay /TX_DQ_UI_TO_PI_TAP)*2 +u1UISmallBak;
    #else
    u2TmpValue = uiDelay /TX_DQ_UI_TO_PI_TAP +u1UISmallBak;
    #endif
    *pu1UISmall = u2TmpValue % u1Small_ui_to_large;
    *pu1UILarge = u2TmpValue / u1Small_ui_to_large +u1UILargeBak;
}

#if 0
DRAM_STATUS_T DramcTxWindowPerbitCal_Manual(DRAMC_CTX_T *p, S16 uiDelay)
{
#if fcFOR_CHIP_ID == fcA60501

#if 0
    U32 u4err_value;
#endif
    U8 ucdq_pi, ucdq_ui_small, ucdq_ui_large,ucdq_oen_ui_small, ucdq_oen_ui_large;
    U8 dq_ui_small_bak, dq_ui_large_bak,  dq_oen_ui_small_bak, dq_oen_ui_large_bak;

    if((uiDelay <0) || (uiDelay >255))
    {
        mcSHOW_DBG_MSG(("\n\n Eror: out of boundary. \nPlease select TX DQ Delay (0~255)\n\n"));
        return DRAM_FAIL;
    }
    //mcSHOW_DBG_MSG(("\n=== Setting Begin ====\n"));

    #if ENABLE_LP4_SW
    if(p->dram_type == TYPE_LPDDR4)
    {
        dq_ui_large_bak = dq_oen_ui_large_bak =3;  

        //small UI = OEN UI +3  for LP4  
        #if ENABLE_64_PI_TO_UI
        dq_oen_ui_small_bak = 1;
        #else
        dq_oen_ui_small_bak = 0;
        #endif
        dq_ui_small_bak = dq_oen_ui_small_bak+1;
    }
    else //LPDDR3
    #endif
    {
        dq_ui_large_bak = dq_oen_ui_large_bak =2;  

        // small UI = OEN UI +2  for LP3
        dq_oen_ui_small_bak = 0;
        dq_ui_small_bak = dq_oen_ui_small_bak+3;
    }

    TxWinTransferDelayToUIPI(p, uiDelay, dq_ui_small_bak, dq_ui_large_bak, &ucdq_ui_large, &ucdq_ui_small, &ucdq_pi);
    TxWinTransferDelayToUIPI(p, uiDelay, dq_oen_ui_small_bak, dq_oen_ui_large_bak, &ucdq_oen_ui_large, &ucdq_oen_ui_small, &ucdq_pi);

    //mcSHOW_DBG_MSG(("\nTX manual DQ delay = %d\n",uiDelay));
    //mcSHOW_DBG_MSG(("Reg Values = (%d, %d, %d)\n",ucdq_ui_large, ucdq_ui_small,ucdq_pi ));
    //mcSHOW_DBG_MSG(("Reg OEN Values = (%d, %d, %d)\n",ucdq_oen_ui_large, ucdq_oen_ui_small, ucdq_pi));

    //TXDLY_DQ , TXDLY_OEN_DQ
    vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH7), \
                                        P_Fld(ucdq_ui_large, SELPH7_TXDLY_DQ0) | \
                                        P_Fld(ucdq_ui_large, SELPH7_TXDLY_DQ1) | \
                                        P_Fld(ucdq_ui_large, SELPH7_TXDLY_DQ2) | \
                                        P_Fld(ucdq_ui_large, SELPH7_TXDLY_DQ3) | \
                                        P_Fld(ucdq_oen_ui_large, SELPH7_TXDLY_OEN_DQ0) | \
                                        P_Fld(ucdq_oen_ui_large, SELPH7_TXDLY_OEN_DQ1) | \
                                        P_Fld(ucdq_oen_ui_large, SELPH7_TXDLY_OEN_DQ2) | \
                                        P_Fld(ucdq_oen_ui_large, SELPH7_TXDLY_OEN_DQ3));

    // DLY_DQ[1:0]
   vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH10), \
                                    P_Fld(ucdq_ui_small, SELPH10_DLY_DQ0) | \
                                    P_Fld(ucdq_ui_small, SELPH10_DLY_DQ1) | \
                                    P_Fld(ucdq_ui_small, SELPH10_DLY_DQ2) | \
                                    P_Fld(ucdq_ui_small, SELPH10_DLY_DQ3) | \
                                    P_Fld(ucdq_oen_ui_small, SELPH10_DLY_OEN_DQ0) | \
                                    P_Fld(ucdq_oen_ui_small, SELPH10_DLY_OEN_DQ1) | \
                                    P_Fld(ucdq_oen_ui_small, SELPH10_DLY_OEN_DQ2) | \
                                    P_Fld(ucdq_oen_ui_small, SELPH10_DLY_OEN_DQ3));
    // DLY_DQ[2]
   vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH22), \
                                    P_Fld(ucdq_ui_small>>2, SELPH22_DLY_DQ0_2) | \
                                    P_Fld(ucdq_ui_small>>2, SELPH22_DLY_DQ1_2)| \
                                    P_Fld(ucdq_ui_small>>2, SELPH22_DLY_DQ2_2) | \
                                    P_Fld(ucdq_ui_small>>2, SELPH22_DLY_DQ3_2) | \
                                    P_Fld(ucdq_oen_ui_small>>2, SELPH22_DLY_OEN_DQ0_2) | \
                                    P_Fld(ucdq_oen_ui_small>>2, SELPH22_DLY_OEN_DQ1_2) | \
                                    P_Fld(ucdq_oen_ui_small>>2, SELPH22_DLY_OEN_DQ2_2) | \
                                    P_Fld(ucdq_oen_ui_small>>2, SELPH22_DLY_OEN_DQ3_2));

    //TXDLY_DQM , TXDLY_OEN_DQM
    vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH8), \
                                        P_Fld(ucdq_ui_large, SELPH8_TXDLY_DQM0) | \
                                        P_Fld(ucdq_ui_large, SELPH8_TXDLY_DQM1) | \
                                        P_Fld(ucdq_ui_large, SELPH8_TXDLY_DQM2) | \
                                        P_Fld(ucdq_ui_large, SELPH8_TXDLY_DQM3) | \
                                        P_Fld(ucdq_oen_ui_large, SELPH8_TXDLY_OEN_DQM0) | \
                                        P_Fld(ucdq_oen_ui_large, SELPH8_TXDLY_OEN_DQM1) | \
                                        P_Fld(ucdq_oen_ui_large, SELPH8_TXDLY_OEN_DQM2) | \
                                        P_Fld(ucdq_oen_ui_large, SELPH8_TXDLY_OEN_DQM3));

    // DLY_DQM[1:0]
   vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH10), \
                                    P_Fld(ucdq_ui_small, SELPH10_DLY_DQM0) | \
                                    P_Fld(ucdq_ui_small, SELPH10_DLY_DQM1) | \
                                    P_Fld(ucdq_ui_small, SELPH10_DLY_DQM2) | \
                                    P_Fld(ucdq_ui_small, SELPH10_DLY_DQM3) | \
                                    P_Fld(ucdq_oen_ui_small, SELPH10_DLY_OEN_DQM0) | \
                                    P_Fld(ucdq_oen_ui_small, SELPH10_DLY_OEN_DQM1) | \
                                    P_Fld(ucdq_oen_ui_small, SELPH10_DLY_OEN_DQM2) | \
                                    P_Fld(ucdq_oen_ui_small, SELPH10_DLY_OEN_DQM3));
    // DLY_DQM[2]
   vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH22), \
                                    P_Fld(ucdq_ui_small>>2, SELPH22_DLY_DQM0_2) | \
                                    P_Fld(ucdq_ui_small>>2, SELPH22_DLY_DQM1_2)| \
                                    P_Fld(ucdq_ui_small>>2, SELPH22_DLY_DQM2_2) | \
                                    P_Fld(ucdq_ui_small>>2, SELPH22_DLY_DQM3_2) | \
                                    P_Fld(ucdq_oen_ui_small>>2, SELPH22_DLY_OEN_DQM0_2) | \
                                    P_Fld(ucdq_oen_ui_small>>2, SELPH22_DLY_OEN_DQM1_2) | \
                                    P_Fld(ucdq_oen_ui_small>>2, SELPH22_DLY_OEN_DQM2_2) | \
                                    P_Fld(ucdq_oen_ui_small>>2, SELPH22_DLY_OEN_DQM3_2));
    
    //set to registers, PI DQ (per byte)
    #if ENABLE_LP4_SW
    if(p->dram_type ==TYPE_LPDDR4)
    {
        // update TX DQ PI delay, for rank 0 // need to take care rank 1 and 2
        vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_ARPI_DQ), \
                                        P_Fld(ucdq_pi, ARPI_DQ_RK0_ARPI_DQ_B0) | P_Fld(ucdq_pi, ARPI_DQ_RK0_ARPI_DQ_B1));
    }
    else //LPDDR3
    #endif
    {
        vIO32WriteFldMulti(DDRPHY_ARPI_DQ+(aru1PhyMap2Channel[0]<< POS_BANK_NUM),\
                                    P_Fld(ucdq_pi, ARPI_DQ_RK0_ARPI_DQ_B0) | P_Fld(ucdq_pi, ARPI_DQ_RK0_ARPI_DQ_B1));
    
        vIO32WriteFldMulti(DDRPHY_ARPI_DQ+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), \
                                        P_Fld(ucdq_pi, ARPI_DQ_RK0_ARPI_DQ_B0) | P_Fld(ucdq_pi, ARPI_DQ_RK0_ARPI_DQ_B1));
    }

#if 0
    u4err_value= TestEngineCompare(p);
#endif

    //mcSHOW_DBG_MSG(("Delay = %d |%4d %4d %4d | 0x%x\n",uiDelay, ucdq_ui_large,ucdq_ui_small, ucdq_pi, u4err_value));
   // mcFPRINTF((fp_A60501, "Delay = %d | %4d %4d %4d | 0x%x\n",uiDelay, ucdq_ui_large,ucdq_ui_small, ucdq_pi, u4err_value));    
   // mcSHOW_DBG_MSG(("=== Setting Done ====\n"));
    #ifdef ETT_PRINT_FORMAT
    mcSHOW_DBG_MSG(("Delay = %d |%d %d %d \n",uiDelay, ucdq_ui_large,ucdq_ui_small, ucdq_pi));
    #else
    mcSHOW_DBG_MSG(("Delay = %d |%4d %4d %4d \n",uiDelay, ucdq_ui_large,ucdq_ui_small, ucdq_pi));
    #endif
    mcFPRINTF((fp_A60501, "Delay = %d | %4d %4d %4d \n",uiDelay, ucdq_ui_large,ucdq_ui_small, ucdq_pi));    

#endif//#if fcFOR_CHIP_ID == fcA60501
	return DRAM_OK;
}
#endif

DRAM_STATUS_T DramcTxWindowPerbitCal(DRAMC_CTX_T *p)
{
    U8 u1BitIdx, u1ByteIdx;
    U8 ucindex;
    U32 uiFinishCount;
    PASS_WIN_DATA_T WinPerBit[DQ_DATA_WIDTH], FinalWinPerBit[DQ_DATA_WIDTH];
    
    U16 uiDelay, u2DQDelayBegin, u2DQDelayEnd;
    U8 ucdq_pi, ucdq_ui_small, ucdq_ui_large,ucdq_oen_ui_small, ucdq_oen_ui_large;
    static U8 dq_ui_small_bak, dq_ui_large_bak,  dq_oen_ui_small_bak, dq_oen_ui_large_bak;
    U8 ucdq_final_pi[DQS_NUMBER], ucdq_final_ui_large[DQS_NUMBER], ucdq_final_ui_small[DQS_NUMBER];
    U8 ucdq_final_oen_ui_large[DQS_NUMBER], ucdq_final_oen_ui_small[DQS_NUMBER];

    S16 s1temp1, s1temp2;
    S16 s2sum_dly[DQS_NUMBER];
    U32 u4err_value, u4fail_bit;
    U8 u1VrefScanEnable;
    U16 u2VrefLevel, u2FinalVref=0xd;
    U16 u2VrefBegin, u2VrefEnd, u2VrefStep;
    U16 u2TempWinSum, u2tx_window_sum;

    U16 u2WinSize;
#if ENABLE_CLAIBTAION_WINDOW_LOG_FOR_FT
    U16 u2MinWinSize = 0xffff;
    U8 u1MinWinSizeBitidx;
#endif

    //U32 u4prev_phyreg_0d0, u4prev_phyreg_2c8, u4prev_phyreg_2cc, u4prev_phyreg_004, u4prev_phyreg_008;
    //U32 u4prev_phyreg_00c, u4prev_phyreg_204, u4prev_phyreg_208, u4prev_phyreg_20c;
    //U32 u4prev_phyreg_2d4, u4prev_phyreg_2d8, u4prev_phyreg_4d0, u4prev_phyreg_258;
    //U32 u4prev_phyreg_25c, u4prev_phyreg_404, u4prev_phyreg_408, u4prev_phyreg_40c;

    // error handling
    if (!p)
    {
        mcSHOW_ERR_MSG(("context is NULL\n"));
        return DRAM_FAIL;
    }

    //A.set RX DQ/DQS in the middle of the pass region from read DQ/DQS calibration
    //B.Fix DQS (RG_PI_**_PBYTE*) at degree from write leveling. 
    //   Move DQ (per byte) gradually from 90 to -45 degree to find the left boundary
    //   Move DQ (per byte) gradually from 90 to 225 degree to find the right boundary
    //C.For each DQ delay in step B, start engine test
    //D.After engine test, read per bit results from registers.
    //E.Set RG_PI_**_DQ* to lie in the average of the middle of the pass region in the same byte

    #if ENABLE_LP4_SW
    if(p->dram_type == TYPE_LPDDR4)
    {
        #if SUPPORT_LP4_DBI
        if(u1WriteDBIEnable)
        {
            dq_ui_large_bak = 2; 
            dq_oen_ui_large_bak =3;   
        }
        else
        #endif
        {
            dq_ui_large_bak = dq_oen_ui_large_bak =3;  
        }
        // small UI = OEN UI +3  for LP4
        #if ENABLE_64_PI_TO_UI
        dq_oen_ui_small_bak = 1;
        #else
        dq_oen_ui_small_bak = 0;
        #endif
        dq_ui_small_bak = dq_oen_ui_small_bak+1;
    }
    else//LPDDR3
    #endif
    {
        if(p->fgTXPerbifInit[p->channel][p->rank]== FALSE)
        {
        #if EVEREST_CHANGE_OF_PHY_PBYTE
            // Scan from DQ delay = DQS delay
            // For everest, choose DQS 1 which is smaller and won't over 64.
            dq_ui_small_bak = (U8)(u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH11), SELPH11_DLY_DQS1) |\
                                              (u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH22), SELPH22_DLY_DQS1_2)<<2));
            
            dq_oen_ui_small_bak = (U8)(u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH11), SELPH11_DLY_OEN_DQS1) |\
                                                     (u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH22), SELPH22_DLY_OEN_DQS1_2)<<2));
            
            dq_ui_large_bak = (U8)u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH9), SELPH9_TXDLY_DQS1) ;
            dq_oen_ui_large_bak = (U8)u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH9), SELPH9_TXDLY_OEN_DQS1);
        
        #else //#if TX_PERBIT_INIT_FLOW_CONTROL
            dq_ui_small_bak = (U8)(u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH10), SELPH10_DLY_DQ0) |\
                                              (u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH22), SELPH22_DLY_DQ0_2)<<2));

            dq_oen_ui_small_bak = (U8)(u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH10), SELPH10_DLY_OEN_DQ0) |\
                                                     (u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH22), SELPH22_DLY_OEN_DQ0_2)<<2));

            dq_ui_large_bak = (U8)u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH7), SELPH7_TXDLY_DQ0) ;
            dq_oen_ui_large_bak = (U8)u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SELPH7), SELPH7_TXDLY_OEN_DQ0);    
        #endif
            p->fgTXPerbifInit[p->channel][p->rank]= TRUE;
        }
    }

    mcSHOW_DBG_MSG(("[DramcTxWindowPerbitCal] Frequency=%d, Channel=%d, Rank=%d\n", p->frequency, p->channel, p->rank));
    mcSHOW_DBG_MSG(("\n[DramcTxWindowPerbitCal] Begin, TX DQ(%d, %d),  DQ OEN(%d, %d)\n", dq_ui_large_bak, dq_ui_small_bak,dq_oen_ui_large_bak,  dq_oen_ui_small_bak));
    mcFPRINTF((fp_A60501, "\n[DramcTxWindowPerbitCal] Begin, TX DQ(%d, %d),  DQ OEN(%d, %d)\n", dq_ui_large_bak, dq_ui_small_bak,dq_oen_ui_large_bak,  dq_oen_ui_small_bak));

    #if ENABLE_LP4_SW
    if(p->dram_type == TYPE_LPDDR4)
    {
        if(p->frequency > HIGH_FREQ)
        {
            u2DQDelayBegin = 320;
            u2DQDelayEnd = 460;
        }
        else if(p->frequency >=1600)
        {
            #if SUPPORT_LP4_DBI
            if(u1WriteDBIEnable)
            {
                u2DQDelayBegin =70;
                u2DQDelayEnd = 200;
            }
            else
            #endif
            {
                u2DQDelayBegin = 170;
                u2DQDelayEnd = 270;
            }
        }
        else if(p->frequency >=1200)
        {
            u2DQDelayBegin = 160;
            u2DQDelayEnd = 250;
        }
        else if(p->frequency >800)
        {
            u2DQDelayBegin = 120;
            u2DQDelayEnd = 250;
        }
        else if(p->frequency >=400)
        {
             u2DQDelayBegin = 100;
             u2DQDelayEnd = 300;
        }
        else
        {
            u2DQDelayBegin = 0;//TX_DQ_DELAY_BEGIN_LP4;
            u2DQDelayEnd = TX_DQ_DELAY_END_LP4;
        }
    }
    else //LPDDR3
    #endif
    {
        #if EVEREST_CHANGE_OF_PHY_PBYTE
        if(fgwrlevel_done)
        {
            // Find smallest DQS delay after write leveling. DQ PI scan from smallest DQS PI.
            u2DQDelayBegin =0xff;
            for(u1ByteIdx=0; u1ByteIdx<DQS_NUMBER; u1ByteIdx++)
            {
                if(wrlevel_dqs_final_delay[p->channel][u1ByteIdx] < u2DQDelayBegin)
                    u2DQDelayBegin = wrlevel_dqs_final_delay[p->channel][u1ByteIdx];
            }
        }
        else
            u2DQDelayBegin =0;
            
        u2DQDelayEnd = u2DQDelayBegin +96; //Scan at least 1UI. Scan 64 to cover byte differece. if window found, early break.
        
        #else
        u2DQDelayBegin = TX_DQ_DELAY_BEGIN_LP3;
        u2DQDelayEnd = TX_DQ_DELAY_END_LP3;
        #endif       
    }

    if((p->dram_type == TYPE_LPDDR4) &&(p->enable_tx_scan_vref==ENABLE))
        u1VrefScanEnable =1;
    else
        u1VrefScanEnable =0;

    if(u1VrefScanEnable)
    {
        u2VrefBegin = TX_VREF_RANGE_BEGIN;
        u2VrefEnd = TX_VREF_RANGE_END;
        u2VrefStep = TX_VREF_RANGE_STEP;
    }
    else //LPDDR3, the for loop will only excute u2VrefLevel=TX_VREF_RANGE_END/2.
    {
        u2VrefBegin = 0;
        u2VrefEnd = 1;
        u2VrefStep =  1;
    }
    
    u2tx_window_sum =0;

    vSetCalibrationResult(p, DRAM_CALIBRATION_TX_PERBIT, DRAM_FAIL); 

    for(u2VrefLevel = u2VrefBegin; u2VrefLevel < u2VrefEnd; u2VrefLevel += u2VrefStep)
    {
         //   SET tx Vref (DQ) here, LP3 no need to set this.
        if((p->dram_type == TYPE_LPDDR4) && u1VrefScanEnable)
        {
            mcSHOW_DBG_MSG(("\n\n======  LP4 TX VrefLevel=%d  ========\n", u2VrefLevel));
            mcFPRINTF((fp_A60501, "\n\n====== LP4 TX VrefLevel=%d ====== \n", u2VrefLevel));

            vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_MRS), P_Fld(u2VrefLevel, MRS_MRSOP)|P_Fld(14, MRS_MRSMA));
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 1, SPCMD_MRWEN);
            mcDELAY_US(1);
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 0, SPCMD_MRWEN);
        }
        else
        {
            mcSHOW_DBG_MSG(("\n\n======  TX Vref Scan disable ========\n"));
            mcFPRINTF((fp_A60501, "\n\n====== TX Vref Scan disable ====== \n"));
        }

        // initialize parameters
        uiFinishCount = 0;
        u2TempWinSum =0;

        for (u1BitIdx = 0; u1BitIdx < p->data_width; u1BitIdx++)
        {
            WinPerBit[u1BitIdx].first_pass = (S16)PASS_RANGE_NA;
            WinPerBit[u1BitIdx].last_pass = (S16)PASS_RANGE_NA;
            FinalWinPerBit[u1BitIdx].first_pass = (S16)PASS_RANGE_NA;
            FinalWinPerBit[u1BitIdx].last_pass = (S16)PASS_RANGE_NA;
        }

        //Move DQ delay ,  1 PI = tCK/64, total 128 PI, 1UI = 32 PI
        //For data rate 3200, max tDQS2DQ is 2.56UI (82 PI)
        //For data rate 4266, max tDQS2DQ is 3.41UI (109 PI)
        for (uiDelay = u2DQDelayBegin; uiDelay <=u2DQDelayEnd; uiDelay++) 
        //    for (uiDelay = u2DQDelayBegin; uiDelay <=u2DQDelayEnd; uiDelay+=3)  //DBI test
        {
            TxWinTransferDelayToUIPI(p, uiDelay, dq_ui_small_bak, dq_ui_large_bak, &ucdq_ui_large, &ucdq_ui_small, &ucdq_pi);
            TxWinTransferDelayToUIPI(p, uiDelay, dq_oen_ui_small_bak, dq_oen_ui_large_bak, &ucdq_oen_ui_large, &ucdq_oen_ui_small, &ucdq_pi);

            //TXDLY_DQ , TXDLY_OEN_DQ
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH7), \
                                                P_Fld(ucdq_ui_large, SELPH7_TXDLY_DQ0) | \
                                                P_Fld(ucdq_ui_large, SELPH7_TXDLY_DQ1) | \
                                                P_Fld(ucdq_ui_large, SELPH7_TXDLY_DQ2) | \
                                                P_Fld(ucdq_ui_large, SELPH7_TXDLY_DQ3) | \
                                                P_Fld(ucdq_oen_ui_large, SELPH7_TXDLY_OEN_DQ0) | \
                                                P_Fld(ucdq_oen_ui_large, SELPH7_TXDLY_OEN_DQ1) | \
                                                P_Fld(ucdq_oen_ui_large, SELPH7_TXDLY_OEN_DQ2) | \
                                                P_Fld(ucdq_oen_ui_large, SELPH7_TXDLY_OEN_DQ3));

            // DLY_DQ[1:0]
           vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH10), \
                                            P_Fld(ucdq_ui_small, SELPH10_DLY_DQ0) | \
                                            P_Fld(ucdq_ui_small, SELPH10_DLY_DQ1) | \
                                            P_Fld(ucdq_ui_small, SELPH10_DLY_DQ2) | \
                                            P_Fld(ucdq_ui_small, SELPH10_DLY_DQ3) | \
                                            P_Fld(ucdq_oen_ui_small, SELPH10_DLY_OEN_DQ0) | \
                                            P_Fld(ucdq_oen_ui_small, SELPH10_DLY_OEN_DQ1) | \
                                            P_Fld(ucdq_oen_ui_small, SELPH10_DLY_OEN_DQ2) | \
                                            P_Fld(ucdq_oen_ui_small, SELPH10_DLY_OEN_DQ3));
            // DLY_DQ[2]
           vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH22), \
                                            P_Fld(ucdq_ui_small>>2, SELPH22_DLY_DQ0_2) | \
                                            P_Fld(ucdq_ui_small>>2, SELPH22_DLY_DQ1_2)| \
                                            P_Fld(ucdq_ui_small>>2, SELPH22_DLY_DQ2_2) | \
                                            P_Fld(ucdq_ui_small>>2, SELPH22_DLY_DQ3_2) | \
                                            P_Fld(ucdq_oen_ui_small>>2, SELPH22_DLY_OEN_DQ0_2) | \
                                            P_Fld(ucdq_oen_ui_small>>2, SELPH22_DLY_OEN_DQ1_2) | \
                                            P_Fld(ucdq_oen_ui_small>>2, SELPH22_DLY_OEN_DQ2_2) | \
                                            P_Fld(ucdq_oen_ui_small>>2, SELPH22_DLY_OEN_DQ3_2));



            //TXDLY_DQM , TXDLY_OEN_DQM
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH8), \
                                                P_Fld(ucdq_ui_large, SELPH8_TXDLY_DQM0) | \
                                                P_Fld(ucdq_ui_large, SELPH8_TXDLY_DQM1) | \
                                                P_Fld(ucdq_ui_large, SELPH8_TXDLY_DQM2) | \
                                                P_Fld(ucdq_ui_large, SELPH8_TXDLY_DQM3) | \
                                                P_Fld(ucdq_oen_ui_large, SELPH8_TXDLY_OEN_DQM0) | \
                                                P_Fld(ucdq_oen_ui_large, SELPH8_TXDLY_OEN_DQM1) | \
                                                P_Fld(ucdq_oen_ui_large, SELPH8_TXDLY_OEN_DQM2) | \
                                                P_Fld(ucdq_oen_ui_large, SELPH8_TXDLY_OEN_DQM3));

            // DLY_DQM[1:0]
           vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH10), \
                                            P_Fld(ucdq_ui_small, SELPH10_DLY_DQM0) | \
                                            P_Fld(ucdq_ui_small, SELPH10_DLY_DQM1) | \
                                            P_Fld(ucdq_ui_small, SELPH10_DLY_DQM2) | \
                                            P_Fld(ucdq_ui_small, SELPH10_DLY_DQM3) | \
                                            P_Fld(ucdq_oen_ui_small, SELPH10_DLY_OEN_DQM0) | \
                                            P_Fld(ucdq_oen_ui_small, SELPH10_DLY_OEN_DQM1) | \
                                            P_Fld(ucdq_oen_ui_small, SELPH10_DLY_OEN_DQM2) | \
                                            P_Fld(ucdq_oen_ui_small, SELPH10_DLY_OEN_DQM3));
            // DLY_DQM[2]
           vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH22), \
                                            P_Fld(ucdq_ui_small>>2, SELPH22_DLY_DQM0_2) | \
                                            P_Fld(ucdq_ui_small>>2, SELPH22_DLY_DQM1_2)| \
                                            P_Fld(ucdq_ui_small>>2, SELPH22_DLY_DQM2_2) | \
                                            P_Fld(ucdq_ui_small>>2, SELPH22_DLY_DQM3_2) | \
                                            P_Fld(ucdq_oen_ui_small>>2, SELPH22_DLY_OEN_DQM0_2) | \
                                            P_Fld(ucdq_oen_ui_small>>2, SELPH22_DLY_OEN_DQM1_2) | \
                                            P_Fld(ucdq_oen_ui_small>>2, SELPH22_DLY_OEN_DQM2_2) | \
                                            P_Fld(ucdq_oen_ui_small>>2, SELPH22_DLY_OEN_DQM3_2));

            //set to registers, PI DQ (per byte)
            #if ENABLE_LP4_SW
            if(p->dram_type ==TYPE_LPDDR4)
            {
                // update TX DQ PI delay, for rank 0 // need to take care rank 1 and 2
                vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_ARPI_DQ), \
                                                P_Fld(ucdq_pi, ARPI_DQ_RK0_ARPI_DQ_B0) | P_Fld(ucdq_pi, ARPI_DQ_RK0_ARPI_DQ_B1));
            }
            else //LPDDR3
            #endif
            {
                vIO32WriteFldMulti(DDRPHY_ARPI_DQ+(aru1PhyMap2Channel[0]<< POS_BANK_NUM),\
                                            P_Fld(ucdq_pi, ARPI_DQ_RK0_ARPI_DQ_B0) | P_Fld(ucdq_pi, ARPI_DQ_RK0_ARPI_DQ_B1));

                vIO32WriteFldMulti(DDRPHY_ARPI_DQ+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), \
                                                P_Fld(ucdq_pi, ARPI_DQ_RK0_ARPI_DQ_B0) | P_Fld(ucdq_pi, ARPI_DQ_RK0_ARPI_DQ_B1));
            }

            u4err_value= TestEngineCompare(p);

            //mcSHOW_DBG_MSG(("Delay=%3d |%2d %2d %3d| %2d %2d| 0x%8x [0]",uiDelay, ucdq_ui_large,ucdq_ui_small, ucdq_pi, ucdq_oen_ui_large,ucdq_oen_ui_small, u4err_value));
            #ifdef ETT_PRINT_FORMAT
            mcSHOW_DBG_MSG2(("%d |%d %d %d|[0]",uiDelay, ucdq_ui_large,ucdq_ui_small, ucdq_pi));
            #else
            mcSHOW_DBG_MSG2(("Delay=%3d |%2d %2d %3d| 0x%8x [0]",uiDelay, ucdq_ui_large,ucdq_ui_small, ucdq_pi, u4err_value));
            #endif
            mcFPRINTF((fp_A60501, "Delay=%3d | %2d %2d %3d| 0x%8x [0]",uiDelay, ucdq_ui_large,ucdq_ui_small, ucdq_pi, u4err_value));  


            // check fail bit ,0 ok ,others fail
            for (u1BitIdx = 0; u1BitIdx < p->data_width; u1BitIdx++)
            {
                u4fail_bit = u4err_value&((U32)1<<u1BitIdx);

                if(u1BitIdx%DQS_BIT_NUMBER ==0)
                {
                    mcSHOW_DBG_MSG2((" "));
                    mcFPRINTF((fp_A60501, " "));
                }
                
                if (u4fail_bit == 0)
                {
                    mcSHOW_DBG_MSG2(("o"));
                    mcFPRINTF((fp_A60501, "o"));
                }
                else
                {
                        mcSHOW_DBG_MSG2(("x"));
                        mcFPRINTF((fp_A60501, "x"));
                }

                if(WinPerBit[u1BitIdx].first_pass== PASS_RANGE_NA)
                {
                    if(u4fail_bit==0) //compare correct: pass
                    {
                        WinPerBit[u1BitIdx].first_pass = uiDelay;
                    }
                }
                else if(WinPerBit[u1BitIdx].last_pass == PASS_RANGE_NA)
                {
                    if(u4fail_bit !=0) //compare error : fail
                    {
                        WinPerBit[u1BitIdx].last_pass  = (uiDelay-1);
                    }
                    else if (uiDelay==u2DQDelayEnd)
                    //else if (uiDelay==MAX_TX_DQDLY_TAPS)
                    {
                        WinPerBit[u1BitIdx].last_pass  = uiDelay;
                    }

                    if(WinPerBit[u1BitIdx].last_pass  !=PASS_RANGE_NA)
                    {
                        if((WinPerBit[u1BitIdx].last_pass -WinPerBit[u1BitIdx].first_pass) >= (FinalWinPerBit[u1BitIdx].last_pass -FinalWinPerBit[u1BitIdx].first_pass))
                        { 
                            #if 0 //for debug    
                            if(FinalWinPerBit[u1BitIdx].last_pass != PASS_RANGE_NA)
                            {
                                mcSHOW_DBG_MSG2(("Bit[%d] Bigger window update %d > %d\n", u1BitIdx, \
                                    (WinPerBit[u1BitIdx].last_pass -WinPerBit[u1BitIdx].first_pass), (FinalWinPerBit[u1BitIdx].last_pass -FinalWinPerBit[u1BitIdx].first_pass)));
                                mcFPRINTF((fp_A60501,"Bit[%d] Bigger window update %d > %d\n", u1BitIdx, \
                                    (WinPerBit[u1BitIdx].last_pass -WinPerBit[u1BitIdx].first_pass), (FinalWinPerBit[u1BitIdx].last_pass -FinalWinPerBit[u1BitIdx].first_pass)));

                            }
                            #endif
                            uiFinishCount |= (1<<u1BitIdx);
                            //update bigger window size 
                            FinalWinPerBit[u1BitIdx].first_pass = WinPerBit[u1BitIdx].first_pass;
                            FinalWinPerBit[u1BitIdx].last_pass = WinPerBit[u1BitIdx].last_pass;
                        }
                        
                        //reset tmp window
                        WinPerBit[u1BitIdx].first_pass = PASS_RANGE_NA;
                        WinPerBit[u1BitIdx].last_pass = PASS_RANGE_NA;
                    }
                 }
               }

                mcSHOW_DBG_MSG2((" [MSB]\n"));
                mcFPRINTF((fp_A60501, " [MSB]\n"));

                //if all bits widnow found and all bits turns to fail again, early break;
                if(((p->dram_type == TYPE_LPDDR4) &&(uiFinishCount == 0xffff)) || \
                    ((p->dram_type == TYPE_LPDDR3) &&(uiFinishCount == 0xffffffff)))
                    {
                        vSetCalibrationResult(p, DRAM_CALIBRATION_TX_PERBIT, DRAM_OK);  
                        if(u1VrefScanEnable ==0)
                        {
                            if(((p->dram_type == TYPE_LPDDR4) &&(u4err_value == 0xffff)) || \
                                ((p->dram_type == TYPE_LPDDR3) &&(u4err_value == 0xffffffff)))
                                {
                                        #ifdef ETT_PRINT_FORMAT
                                        mcSHOW_DBG_MSG2(("TX calibration finding left boundary early break. PI DQ delay=0x%B\n", uiDelay));
                                        #else
                                        mcSHOW_DBG_MSG2(("TX calibration finding left boundary early break. PI DQ delay=0x%2x\n", uiDelay));
                                        #endif
                                        break;  //early break
                                }
                        }
                    }

        }

        mcSHOW_DBG_MSG(("TX CH%d R%d\n", p->channel, p->rank));
        
        for (u1BitIdx = 0; u1BitIdx < p->data_width; u1BitIdx++)
        {   
            u2WinSize = FinalWinPerBit[u1BitIdx].last_pass-FinalWinPerBit[u1BitIdx].first_pass;
            u2TempWinSum += u2WinSize;  //Sum of CA Windows for vref selection
            
            // Everest : BU request RX & TX window size log.
            mcSHOW_DBG_MSG(("%d: %d\n", u1BitIdx, u2WinSize));
            
            #if ENABLE_CLAIBTAION_WINDOW_LOG_FOR_FT
            if(u2WinSize<u2MinWinSize)
            {
                u2MinWinSize = u2WinSize;
                u1MinWinSizeBitidx = u1BitIdx;
            }
            #endif
        }
        
        #if ENABLE_CLAIBTAION_WINDOW_LOG_FOR_FT
        mcSHOW_DBG_MSG2(("FT log: TX min window : bit %d, size %d\n", u1MinWinSizeBitidx, u2MinWinSize));
        #endif    
        mcSHOW_DBG_MSG2(("\n\nTX Vref %d, Window Sum %d > %d\n", u2VrefLevel, u2TempWinSum, u2tx_window_sum));
        mcFPRINTF((fp_A60501, "\n\nTX Vref %d, Window Sum %d > %d\n", u2VrefLevel, u2TempWinSum, u2tx_window_sum));
        
        if(((p->dram_type == TYPE_LPDDR4) &&(u2TempWinSum <200)) || \
            ((p->dram_type == TYPE_LPDDR3) &&(u2TempWinSum <500)))
        {
            mcSHOW_DBG_MSG3(("\n\n[NOTICE]  Channel %d , u2tx_window_sum %d\n", p->channel, u2TempWinSum));
            mcFPRINTF((fp_A60501, "\n\n[NOTICE]  Channel %d , u2tx_window_sum  %d\n", p->channel, u2TempWinSum));
        }

        if(u2TempWinSum >u2tx_window_sum)
        {
            mcSHOW_DBG_MSG(("\n\nBetter TX Vref found %d, Window Sum %d > %d\n", u2VrefLevel, u2TempWinSum, u2tx_window_sum));
            mcFPRINTF((fp_A60501, "\n\nBetter TX Vref found %d, Window Sum %d > %d\n", u2VrefLevel, u2TempWinSum, u2tx_window_sum));

            u2tx_window_sum =u2TempWinSum;
            u2FinalVref = u2VrefLevel;   

            //Calculate the center of DQ pass window
            // Record center sum of each byte
            for (u1ByteIdx=0; u1ByteIdx<(p->data_width/DQS_BIT_NUMBER); u1ByteIdx++)
            {
                s2sum_dly[u1ByteIdx] = 0;
            
                for (u1BitIdx=0; u1BitIdx<DQS_BIT_NUMBER; u1BitIdx++)
                {
                    ucindex = u1ByteIdx * DQS_BIT_NUMBER + u1BitIdx;
                    FinalWinPerBit[ucindex].win_center = (FinalWinPerBit[ucindex].first_pass + FinalWinPerBit[ucindex].last_pass) >> 1;
                    s2sum_dly[u1ByteIdx] += FinalWinPerBit[ucindex].win_center;
                }
            }

            mcSHOW_DBG_MSG(("==================================================================\n"));
            mcSHOW_DBG_MSG(("    channel=%d(2:cha, 3:chb)  u2VrefLevel = %d\n", p->channel, u2VrefLevel)); 
            mcSHOW_DBG_MSG(("PI DQ (per byte) window\nx=pass dq delay value (min~max)center \ny=0-7bit DQ of every group\n"));
            mcSHOW_DBG_MSG(("==================================================================\n"));
            mcSHOW_DBG_MSG(("bit	Byte0	 bit      Byte1     bit     Byte2     bit     Byte3\n"));
            
            mcFPRINTF((fp_A60501,"==================================================================\n"));
            mcFPRINTF((fp_A60501,"    channel=%d(2:cha, 3:chb)  u2VrefLevel = %d\n", p->channel, u2VrefLevel)); 
            mcFPRINTF((fp_A60501,"PI DQ (per byte) window\nx=pass dq delay value (min~max)center \ny=0-7bit DQ of every group\n"));
            mcFPRINTF((fp_A60501,"==================================================================\n"));
            mcFPRINTF((fp_A60501,"bit	Byte0	 bit      Byte1     bit     Byte2     bit     Byte3\n"));

            for (u1BitIdx = 0; u1BitIdx < DQS_BIT_NUMBER; u1BitIdx++)
            {
                #ifdef ETT_PRINT_FORMAT
                mcSHOW_DBG_MSG(("%d (%d~%d) %d,   %d (%d~%d) %d,", \
                    u1BitIdx, FinalWinPerBit[u1BitIdx].first_pass, FinalWinPerBit[u1BitIdx].last_pass, FinalWinPerBit[u1BitIdx].win_center, \
                    u1BitIdx+8, FinalWinPerBit[u1BitIdx+8].first_pass, FinalWinPerBit[u1BitIdx+8].last_pass, FinalWinPerBit[u1BitIdx+8].win_center));			
                #else
                mcSHOW_DBG_MSG(("%2d (%2d~%2d) %2d,   %2d (%2d~%2d) %2d,", \
                    u1BitIdx, FinalWinPerBit[u1BitIdx].first_pass, FinalWinPerBit[u1BitIdx].last_pass, FinalWinPerBit[u1BitIdx].win_center, \
                    u1BitIdx+8, FinalWinPerBit[u1BitIdx+8].first_pass, FinalWinPerBit[u1BitIdx+8].last_pass, FinalWinPerBit[u1BitIdx+8].win_center));			
                #endif
                mcFPRINTF((fp_A60501,"%2d (%2d~%2d) %2d,   %2d (%2d~%2d) %2d,", \
                    u1BitIdx, FinalWinPerBit[u1BitIdx].first_pass, FinalWinPerBit[u1BitIdx].last_pass, FinalWinPerBit[u1BitIdx].win_center, \
                    u1BitIdx+8, FinalWinPerBit[u1BitIdx+8].first_pass, FinalWinPerBit[u1BitIdx+8].last_pass, FinalWinPerBit[u1BitIdx+8].win_center));			

                if(p->dram_type ==TYPE_LPDDR4)
                {
                    mcSHOW_DBG_MSG(("\n"));
                    mcFPRINTF((fp_A60501,"\n"));
                }
                else //LPDDR3
                {
                    #ifdef ETT_PRINT_FORMAT
                    mcSHOW_DBG_MSG(("  %d (%d~%d) %d,   %d (%d~%d) %d\n", \
                        u1BitIdx+16, FinalWinPerBit[u1BitIdx+16].first_pass, FinalWinPerBit[u1BitIdx+16].last_pass, FinalWinPerBit[u1BitIdx+16].win_center, \
                        u1BitIdx+24, FinalWinPerBit[u1BitIdx+24].first_pass, FinalWinPerBit[u1BitIdx+24].last_pass, FinalWinPerBit[u1BitIdx+24].win_center ));	
                    #else
                    mcSHOW_DBG_MSG(("  %2d (%2d~%2d) %2d,   %2d (%2d~%2d) %2d\n", \
                        u1BitIdx+16, FinalWinPerBit[u1BitIdx+16].first_pass, FinalWinPerBit[u1BitIdx+16].last_pass, FinalWinPerBit[u1BitIdx+16].win_center, \
                        u1BitIdx+24, FinalWinPerBit[u1BitIdx+24].first_pass, FinalWinPerBit[u1BitIdx+24].last_pass, FinalWinPerBit[u1BitIdx+24].win_center ));	
                    #endif

                    mcFPRINTF((fp_A60501,"  %2d (%2d~%2d) %2d,   %2d (%2d~%2d) %2d\n", \
                        u1BitIdx+16, FinalWinPerBit[u1BitIdx+16].first_pass, FinalWinPerBit[u1BitIdx+16].last_pass, FinalWinPerBit[u1BitIdx+16].win_center, \
                        u1BitIdx+24, FinalWinPerBit[u1BitIdx+24].first_pass, FinalWinPerBit[u1BitIdx+24].last_pass, FinalWinPerBit[u1BitIdx+24].win_center ));	
                }
            }
            mcSHOW_DBG_MSG(("\n==================================================================\n"));
            mcFPRINTF((fp_A60501,"\n==================================================================\n"));
        }
    }

    //Calculate the center of DQ pass window
    //average the center delay
    for (u1ByteIdx=0; u1ByteIdx<(p->data_width/DQS_BIT_NUMBER); u1ByteIdx++)
    {
        s1temp1 = s2sum_dly[u1ByteIdx] /DQS_BIT_NUMBER;
        s1temp2 = s1temp1+1;
        if ((s2sum_dly[u1ByteIdx] -s1temp1*DQS_BIT_NUMBER) > (s1temp2*DQS_BIT_NUMBER-s2sum_dly[u1ByteIdx] ))
        {
            uiDelay = (U16)s1temp2;
        }
        else
        {
            uiDelay = (U16)s1temp1;
        }

        TxWinTransferDelayToUIPI(p, uiDelay, dq_ui_small_bak, dq_ui_large_bak, &ucdq_final_ui_large[u1ByteIdx], &ucdq_final_ui_small[u1ByteIdx], &ucdq_final_pi[u1ByteIdx]);
        TxWinTransferDelayToUIPI(p, uiDelay, dq_oen_ui_small_bak, dq_oen_ui_large_bak, &ucdq_final_oen_ui_large[u1ByteIdx], &ucdq_final_oen_ui_small[u1ByteIdx], &ucdq_final_pi[u1ByteIdx]);

        #ifdef TX_EYE_SCAN
        aru2TXCaliDelay[p->channel][u1ByteIdx] = (ucdq_final_ui_large[u1ByteIdx]*4+ucdq_final_ui_small[u1ByteIdx])*32+ucdq_final_pi[u1ByteIdx];
        aru2TXCaliDelay_OEN[p->channel][u1ByteIdx] = (ucdq_final_oen_ui_large[u1ByteIdx]*4+ucdq_final_oen_ui_small[u1ByteIdx])*32+ucdq_final_pi[u1ByteIdx];
        #endif

        mcSHOW_DBG_MSG(("Byte%d, PI DQ Delay %d\n",  u1ByteIdx, uiDelay));
        mcSHOW_DBG_MSG(("Final DQ PI Delay(LargeUI, SmallUI, PI) =(%d ,%d, %d)\n", ucdq_final_ui_large[u1ByteIdx], ucdq_final_ui_small[u1ByteIdx], ucdq_final_pi[u1ByteIdx]));
        mcSHOW_DBG_MSG3(("OEN DQ PI Delay(LargeUI, SmallUI, PI) =(%d ,%d, %d)\n\n", ucdq_final_oen_ui_large[u1ByteIdx], ucdq_final_oen_ui_small[u1ByteIdx], ucdq_final_pi[u1ByteIdx]));

        mcFPRINTF((fp_A60501,"Byte%d, PI DQ Delay %d\n",  u1ByteIdx, uiDelay));
        mcFPRINTF((fp_A60501,"Final DQ PI Delay(LargeUI, SmallUI, PI) =(%d ,%d, %d)\n", ucdq_final_ui_large[u1ByteIdx], ucdq_final_ui_small[u1ByteIdx], ucdq_final_pi[u1ByteIdx]));
        mcFPRINTF((fp_A60501,"OEN DQ PI Delay(LargeUI, SmallUI, PI) =(%d ,%d, %d)\n\n", ucdq_final_oen_ui_large[u1ByteIdx], ucdq_final_oen_ui_small[u1ByteIdx], ucdq_final_pi[u1ByteIdx]));
    }

     // SET tx Vref (DQ) = u2FinalVref, LP3 no need to set this.
    if(u1VrefScanEnable)
    {
        mcSHOW_DBG_MSG(("\nFinal TX Vref %d\n\n", u2FinalVref));
        mcFPRINTF((fp_A60501, "\nFinal TX Vref %d\n\n", u2FinalVref));

       // u4MROP= DEFAULT_MR13_VALUE_LP4 | u2FinalVref;
        vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_MRS), P_Fld(u2FinalVref, MRS_MRSOP)|P_Fld(14, MRS_MRSMA));
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 1, SPCMD_MRWEN);
        mcDELAY_US(1);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_SPCMD), 0, SPCMD_MRWEN);
    }

#if REG_ACCESS_PORTING_DGB
    RegLogEnable =1;
#endif

    //set to registers, PI DQ (per byte)
    // Need DSQIEN clock gated and reset ?????  TO check.
    //TXDLY_DQ , TXDLY_OEN_DQ
    vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH7), \
                                        P_Fld(ucdq_final_ui_large[0], SELPH7_TXDLY_DQ0) | \
                                        P_Fld(ucdq_final_ui_large[1], SELPH7_TXDLY_DQ1) | \
                                        P_Fld(ucdq_final_ui_large[2], SELPH7_TXDLY_DQ2) | \
                                        P_Fld(ucdq_final_ui_large[3], SELPH7_TXDLY_DQ3) | \
                                        P_Fld(ucdq_final_oen_ui_large[0], SELPH7_TXDLY_OEN_DQ0) | \
                                        P_Fld(ucdq_final_oen_ui_large[1], SELPH7_TXDLY_OEN_DQ1) | \
                                        P_Fld(ucdq_final_oen_ui_large[2], SELPH7_TXDLY_OEN_DQ2) | \
                                        P_Fld(ucdq_final_oen_ui_large[3], SELPH7_TXDLY_OEN_DQ3));
    // DLY_DQ[1:0]
   vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH10), \
                                    P_Fld(ucdq_final_ui_small[0], SELPH10_DLY_DQ0) | \
                                    P_Fld(ucdq_final_ui_small[1], SELPH10_DLY_DQ1) | \
                                    P_Fld(ucdq_final_ui_small[2], SELPH10_DLY_DQ2) | \
                                    P_Fld(ucdq_final_ui_small[3], SELPH10_DLY_DQ3) | \
                                    P_Fld(ucdq_final_oen_ui_small[0], SELPH10_DLY_OEN_DQ0) | \
                                    P_Fld(ucdq_final_oen_ui_small[1], SELPH10_DLY_OEN_DQ1) | \
                                    P_Fld(ucdq_final_oen_ui_small[2], SELPH10_DLY_OEN_DQ2) | \
                                    P_Fld(ucdq_final_oen_ui_small[3], SELPH10_DLY_OEN_DQ3));
    // DLY_DQ[2]
   vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH22), \
                                    P_Fld(ucdq_final_ui_small[0]>>2, SELPH22_DLY_DQ0_2) | \
                                    P_Fld(ucdq_final_ui_small[1]>>2, SELPH22_DLY_DQ1_2)| \
                                    P_Fld(ucdq_final_ui_small[2]>>2, SELPH22_DLY_DQ2_2) | \
                                    P_Fld(ucdq_final_ui_small[3]>>2, SELPH22_DLY_DQ3_2) | \
                                    P_Fld(ucdq_final_oen_ui_small[0]>>2, SELPH22_DLY_OEN_DQ0_2) | \
                                    P_Fld(ucdq_final_oen_ui_small[1]>>2, SELPH22_DLY_OEN_DQ1_2) | \
                                    P_Fld(ucdq_final_oen_ui_small[2]>>2, SELPH22_DLY_OEN_DQ2_2) | \
                                    P_Fld(ucdq_final_oen_ui_small[3]>>2, SELPH22_DLY_OEN_DQ3_2));


    //TXDLY_DQM , TXDLY_OEN_DQM
    vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH8), \
                                        P_Fld(ucdq_final_ui_large[0], SELPH8_TXDLY_DQM0) | \
                                        P_Fld(ucdq_final_ui_large[1], SELPH8_TXDLY_DQM1) | \
                                        P_Fld(ucdq_final_ui_large[2], SELPH8_TXDLY_DQM2) | \
                                        P_Fld(ucdq_final_ui_large[3], SELPH8_TXDLY_DQM3) | \
                                        P_Fld(ucdq_final_oen_ui_large[0], SELPH8_TXDLY_OEN_DQM0) | \
                                        P_Fld(ucdq_final_oen_ui_large[1], SELPH8_TXDLY_OEN_DQM1) | \
                                        P_Fld(ucdq_final_oen_ui_large[2], SELPH8_TXDLY_OEN_DQM2) | \
                                        P_Fld(ucdq_final_oen_ui_large[3], SELPH8_TXDLY_OEN_DQM3));
    // DLY_DQM[1:0]
   vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH10), \
                                    P_Fld(ucdq_final_ui_small[0], SELPH10_DLY_DQM0) | \
                                    P_Fld(ucdq_final_ui_small[1], SELPH10_DLY_DQM1) | \
                                    P_Fld(ucdq_final_ui_small[2], SELPH10_DLY_DQM2) | \
                                    P_Fld(ucdq_final_ui_small[3], SELPH10_DLY_DQM3) | \
                                    P_Fld(ucdq_final_oen_ui_small[0], SELPH10_DLY_OEN_DQM0) | \
                                    P_Fld(ucdq_final_oen_ui_small[1], SELPH10_DLY_OEN_DQM1) | \
                                    P_Fld(ucdq_final_oen_ui_small[2], SELPH10_DLY_OEN_DQM2) | \
                                    P_Fld(ucdq_final_oen_ui_small[3], SELPH10_DLY_OEN_DQM3));
    // DLY_DQM[2]
   vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH22), \
                                    P_Fld(ucdq_final_ui_small[0]>>2, SELPH22_DLY_DQM0_2) | \
                                    P_Fld(ucdq_final_ui_small[1]>>2, SELPH22_DLY_DQM1_2)| \
                                    P_Fld(ucdq_final_ui_small[2]>>2, SELPH22_DLY_DQM2_2) | \
                                    P_Fld(ucdq_final_ui_small[3]>>2, SELPH22_DLY_DQM3_2) | \
                                    P_Fld(ucdq_final_oen_ui_small[0]>>2, SELPH22_DLY_OEN_DQM0_2) | \
                                    P_Fld(ucdq_final_oen_ui_small[1]>>2, SELPH22_DLY_OEN_DQM1_2) | \
                                    P_Fld(ucdq_final_oen_ui_small[2]>>2, SELPH22_DLY_OEN_DQM2_2) | \
                                    P_Fld(ucdq_final_oen_ui_small[3]>>2, SELPH22_DLY_OEN_DQM3_2));

    if(p->rank == RANK_1)
    {
        //TXDLY_DQ , TXDLY_OEN_DQ
        vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH16), \
                                            P_Fld(ucdq_final_ui_large[0], SELPH16_TX_DLY_R1DQ0) | \
                                            P_Fld(ucdq_final_ui_large[1], SELPH16_TX_DLY_R1DQ1) | \
                                            P_Fld(ucdq_final_ui_large[2], SELPH16_TX_DLY_R1DQ2) | \
                                            P_Fld(ucdq_final_ui_large[3], SELPH16_TX_DLY_R1DQ3) | \
                                            P_Fld(ucdq_final_oen_ui_large[0], SELPH16_TX_DLY_R1OEN_DQ0) | \
                                            P_Fld(ucdq_final_oen_ui_large[1], SELPH16_TX_DLY_R1OEN_DQ1) | \
                                            P_Fld(ucdq_final_oen_ui_large[2], SELPH16_TX_DLY_R1OEN_DQ2) | \
                                            P_Fld(ucdq_final_oen_ui_large[3], SELPH16_TX_DLY_R1OEN_DQ3));
        
        //TXDLY_DQM , TXDLY_OEN_DQM
        vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH17), \
                                            P_Fld(ucdq_final_ui_large[0], SELPH17_TX_DLY_R1DQM0) | \
                                            P_Fld(ucdq_final_ui_large[1], SELPH17_TX_DLY_R1DQM1) | \
                                            P_Fld(ucdq_final_ui_large[2], SELPH17_TX_DLY_R1DQM2) | \
                                            P_Fld(ucdq_final_ui_large[3], SELPH17_TX_DLY_R1DQM3) | \
                                            P_Fld(ucdq_final_oen_ui_large[0], SELPH17_TX_DLY_R1OEN_DQM0) | \
                                            P_Fld(ucdq_final_oen_ui_large[1], SELPH17_TX_DLY_R1OEN_DQM1) | \
                                            P_Fld(ucdq_final_oen_ui_large[2], SELPH17_TX_DLY_R1OEN_DQM2) | \
                                            P_Fld(ucdq_final_oen_ui_large[3], SELPH17_TX_DLY_R1OEN_DQM3));
        // DLY_DQ[1:0]
       vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH18), \
                                        P_Fld(ucdq_final_ui_small[0], SELPH18_DLY_R1DQM0) | \
                                        P_Fld(ucdq_final_ui_small[1], SELPH18_DLY_R1DQM1) | \
                                        P_Fld(ucdq_final_ui_small[2], SELPH18_DLY_R1DQM2) | \
                                        P_Fld(ucdq_final_ui_small[3], SELPH18_DLY_R1DQM3) | \
                                        P_Fld(ucdq_final_ui_small[0], SELPH18_DLY_R1DQ0) | \
                                        P_Fld(ucdq_final_ui_small[1], SELPH18_DLY_R1DQ1) | \
                                        P_Fld(ucdq_final_ui_small[2], SELPH18_DLY_R1DQ2) | \
                                        P_Fld(ucdq_final_ui_small[3], SELPH18_DLY_R1DQ3));

        // DLY_DQ[1:0]
       vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH19), \
                                        P_Fld(ucdq_final_oen_ui_small[0], SELPH19_DLY_R1OEN_DQM0) | \
                                        P_Fld(ucdq_final_oen_ui_small[1], SELPH19_DLY_R1OEN_DQM1) | \
                                        P_Fld(ucdq_final_oen_ui_small[2], SELPH19_DLY_R1OEN_DQM2) | \
                                        P_Fld(ucdq_final_oen_ui_small[3], SELPH19_DLY_R1OEN_DQM3) | \
                                        P_Fld(ucdq_final_oen_ui_small[0], SELPH19_DLY_R1OEN_DQ0) | \
                                        P_Fld(ucdq_final_oen_ui_small[1], SELPH19_DLY_R1OEN_DQ1) | \
                                        P_Fld(ucdq_final_oen_ui_small[2], SELPH19_DLY_R1OEN_DQ2) | \
                                        P_Fld(ucdq_final_oen_ui_small[3], SELPH19_DLY_R1OEN_DQ3));
    }

    #if ENABLE_LP4_SW
    if(p->dram_type ==TYPE_LPDDR4)
    {
        vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_ARPI_DQ), \
                                            P_Fld(ucdq_final_pi[0], ARPI_DQ_RK0_ARPI_DQ_B0) | \
                                            P_Fld(ucdq_final_pi[1], ARPI_DQ_RK0_ARPI_DQ_B1));

        if(p->rank == RANK_1)
        {
            vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_ARPI_DQ2), \
                                                P_Fld(ucdq_final_pi[0], ARPI_DQ2_RK1_ARPI_DQ_B0) | \
                                                P_Fld(ucdq_final_pi[1], ARPI_DQ2_RK1_ARPI_DQ_B1));
        }
    }
    else
    #endif
    {       
            vIO32WriteFldMulti(DDRPHY_ARPI_DQ+(aru1PhyMap2Channel[0]<< POS_BANK_NUM),\
                                        P_Fld(ucdq_final_pi[0], ARPI_DQ_RK0_ARPI_DQ_B0) | P_Fld(ucdq_final_pi[2], ARPI_DQ_RK0_ARPI_DQ_B1));
        
            vIO32WriteFldMulti(DDRPHY_ARPI_DQ+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), \
                                            P_Fld(ucdq_final_pi[1], ARPI_DQ_RK0_ARPI_DQ_B0) | P_Fld(ucdq_final_pi[3], ARPI_DQ_RK0_ARPI_DQ_B1));
            
            if(p->rank == RANK_1)
            {
                vIO32WriteFldMulti(DDRPHY_ARPI_DQ2+(aru1PhyMap2Channel[0]<< POS_BANK_NUM),\
                                            P_Fld(ucdq_final_pi[0], ARPI_DQ_RK0_ARPI_DQ_B0) | P_Fld(ucdq_final_pi[2], ARPI_DQ_RK0_ARPI_DQ_B1));
            
                vIO32WriteFldMulti(DDRPHY_ARPI_DQ2+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), \
                                                P_Fld(ucdq_final_pi[1], ARPI_DQ_RK0_ARPI_DQ_B0) | P_Fld(ucdq_final_pi[3], ARPI_DQ_RK0_ARPI_DQ_B1));
            }
    }
#if REG_ACCESS_PORTING_DGB
    RegLogEnable =0;
#endif

    mcSHOW_DBG_MSG(("[DramcTxWindowPerbitCal] ====Done====\n"));
    mcFPRINTF((fp_A60501, "[DramcTxWindowPerbitCal] ====Done====\n"));    

    #if 0
    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_PADCTL4), 1, PADCTL4_CKEFIXON);  // test only
    #endif

    return DRAM_OK;
                                        
    // log example
    /*
TX calibration finding left boundary early break. PI DQ delay=0x3e
TX calibration finding right boundary early break. PI DQ delay=0x1c
==================================================================
    TX DQS perbit delay software calibration v3
    channel=2(2:cha, 3:chb)  apply = 1
==================================================================
PI DQ (per byte) window
x=pass dq delay value (min~max)center
y=0-7bit DQ of every group
input delay:Byte0 = 13 Byte1 = 13 Byte2 = 12 Byte3 = 13
==================================================================
bit    Byte0    bit    Byte1    bit    Byte2    bit    Byte3
 0   ( 1~26)13,  8   ( 3~26)14, 16   ( 2~27)14, 24   ( 3~26)14
 1   ( 2~26)14,  9   ( 1~24)12, 17   ( 0~25)12, 25   ( 4~26)15
 2   ( 3~25)14, 10   ( 2~26)14, 18   ( 2~25)13, 26   ( 3~27)15
 3   ( 2~24)13, 11   ( 1~23)12, 19   ( 3~25)14, 27   ( 1~23)12
 4   ( 3~26)14, 12   ( 2~26)14, 20   ( 0~24)12, 28   ( 1~25)13
 5   ( 3~25)14, 13   ( 2~25)13, 21   (-1~25)12, 29   ( 2~24)13
 6   ( 2~26)14, 14   ( 2~24)13, 22   (-1~26)12, 30   ( 3~27)15
 7   ( 1~25)13, 15   ( 2~26)14, 23   (-1~22)10, 31   ( 2~26)14

==================================================================
   */
   
}


DRAM_STATUS_T DramcRunTimeModifyTXDQTest(DRAMC_CTX_T *p)
{
#if fcFOR_CHIP_ID == fcA60501
    U32 TA_ADDR, ii, u4value, u4addr, u4err_value;
    S16 i2Delay;

    // TA4_3 begin
    DramcTestPat4_3(p, TA43_OP_STOP);
    // run quiet
    DramcTestPat4_3(p, TA43_OP_RUNQUIET);
    mcSHOW_DBG_MSG2(("[DramcRunTimeModifyTX_DQ] TA4_3 is running.\n"));

    for (i2Delay = 103; i2Delay >= 79; i2Delay--)
    //for (i2Delay = 78; i2Delay <= 103; i2Delay++)
    {
    	//#ifdef A60501_BUILD_ERROR_TODO
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DLLCONF), 0, DLLCONF_MANUTXUPD);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DLLCONF), 1, DLLCONF_TXUPDMODE);
        DramcTestPat4_3(p, TA43_OP_CLEAR);
        
        DramcTxWindowPerbitCal_Manual(p, i2Delay);
        mcSHOW_DBG_MSG2(("i2Delay = %d, ", i2Delay));
        
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DLLCONF), 1, DLLCONF_MANUTXUPD);
        mcDELAY_US(1);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DLLCONF), 0, DLLCONF_TXUPDMODE);
		//#endif

        
        mcDELAY_MS(20);

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

        u4addr = (DRAMC_BASE_ADDRESS | (TA_ADDR << POS_BANK_NUM) | (0xe00));
        for (ii = 0; ii < 4; ii++)
        {
            u4value = u4IO32Read4B(u4addr);
            p->ta43_result[ii] = u4value;
            u4addr += 4;
        }

        u4err_value=0;
        for (ii = 0; ii < 4; ii++)
        {
            u4err_value |= (p->ta43_result[ii]);
        }
        mcSHOW_DBG_MSG2(("u4err_value = 0x%x\n ",u4err_value));
    }

    // TA4_3 End
    DramcTestPat4_3(p, TA43_OP_STOP);	
    mcSHOW_DBG_MSG2(("[DramcRunTimeModifyTXDQTest] Stop. \n"));
#endif //#if fcFOR_CHIP_ID == fcA60501
    return DRAM_OK;
}


#ifdef RX_EYE_SCAN
#define RX_EYE_VREF_LEVEL   32
#define RX_EYE_DELAY_LEVEL  63//40
#define RX_EYE_SAVE_TO_FILE 1
#define ENALBE_EYE_SCAN_LOG_DURING_PROCESS 0
#define RX_EYE_SCAN_VREF_EXTEND 1

typedef enum
{
    RX_EYE_SCAN_MOVE_DQS_ENTER=0,    
    RX_EYE_SCAN_MOVE_DQS_END,
} RX_EYE_SCAN_MOVE_DQS_STATUS_T;

typedef enum
{
    RX_EYE_SCAN_SCAN_AT_BOOT_TIME=0,    
    RX_EYE_SCAN_SCAN_AT_RUN_TIME,
} RX_EYE_SCAN_TIME_T;


void DramcRxEyeScanInit(DRAMC_CTX_T *p)
{   
    U8 u1ByteIdx;
    //Enable DQ eye scan
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_STBCAL_F), 1, STBCAL_F_RG_EX_EYE_SCAN_EN);
    
    //Disable MIOCK jitter meter mode (RG_??_RX_DQS_MIOCK_SEL=0, RG_RX_MIOCK_JIT_EN=0)
    //RG_RX_MIOCK_JIT_EN=0
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_STBCAL_F), 0, STBCAL_F_RG_RX_MIOCK_JIT_EN);

    #if ENABLE_LP4_SW
    if(p->dram_type == TYPE_LPDDR4)
    {
        //RG_??_RX_EYE_SCAN_EN,   RG_??_RX_VREF_EN,  RG_??_RX_SMT_EN
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE2), 1, EYE2_RG_RX_ARDQ_EYE_EN_B0);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), 1, EYEB1_2_RG_RX_ARDQ_EYE_EN_B1);
        
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE2), 1, EYE2_RG_RX_ARDQ_EYE_VREF_EN_B0);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), 1, EYEB1_2_RG_RX_ARDQ_EYE_VREF_EN_B1);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_TXDQ3), 1, TXDQ3_RG_RX_ARDQ_SMT_EN_B0); 
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_RXDQ13), 1, RXDQ13_RG_RX_ARDQ_SMT_EN_B1);    
    }
    else  //LPDDR3
    #endif
    {
        for(u1ByteIdx=0; u1ByteIdx<DQS_NUMBER; u1ByteIdx++)
        {
            //RG_??_RX_EYE_SCAN_EN,   RG_??_RX_VREF_EN,  RG_??_RX_SMT_EN
            vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_EYE2), 1, EYE2_RG_RX_ARDQ_EYE_EN_B0);
            vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_EYE2), 1, EYE2_RG_RX_ARDQ_EYE_VREF_EN_B0);        
            vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_TXDQ3), 1, TXDQ3_RG_RX_ARDQ_SMT_EN_B0);
        }   
    }

    DramEyeStbenReset(p);
    DramPhyReset(p);
}

#if 0
static void DramcRxEyeRuntimeDQS(DRAMC_CTX_T *p, U8 enter_or_end)
{
    U32 u4TimeCnt =1000;
    U8 u1Ready[DQS_NUMBER], u1Ready_all, u1Wait=1;

    #if ENABLE_LP4_SW
    if(p->dram_type == TYPE_LPDDR4)
    {
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE3), enter_or_end, EYE3_RG_RX_ARDQ_EYE_OE_GATE_EN_B0);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1_3), enter_or_end, EYEB1_3_RG_RX_ARDQ_EYE_OE_GATE_EN_B1);
        
        do
        {                        
            u1Ready[0]  =(U8)u4IO32ReadFldAlign(DRAMC_REG_ADDR(DDRPHY_PHY_RO_1), PHY_RO_1_RGS_RX_ARDQS0_DLY_RDY_EYE);
            u1Ready[1] = (U8)u4IO32ReadFldAlign(DRAMC_REG_ADDR(DDRPHY_PHY_RO_1), PHY_RO_1_RGS_RX_ARDQS1_DLY_RDY_EYE);

            if(enter_or_end ==RX_EYE_SCAN_MOVE_DQS_ENTER)
            {
               u1Ready_all = u1Ready[0] ||u1Ready[1];
            }
            else//RX_EYE_SCAN_MOVE_DQS_END
            {
               u1Ready_all = u1Ready[0] && u1Ready[1];
            }

            u1Wait = (u1Ready_all != enter_or_end) ? 1 : 0;
            u4TimeCnt--;
        }
        while((u1Wait ==1) && (u4TimeCnt >0));
        
        if(u4TimeCnt ==0)
            mcSHOW_DBG_MSG(("DQS setup timeout (enter_or_end = %d) %d %d\n", enter_or_end, u1Ready[0] , u1Ready[1]));
    }
    else //LPDDR3
    #endif
    {
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE3), enter_or_end, EYE3_RG_RX_ARDQ_EYE_OE_GATE_EN_B0);
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1_3), enter_or_end, EYEB1_3_RG_RX_ARDQ_EYE_OE_GATE_EN_B1);
        do
        {   
            u1Ready[0] = (U8)u4IO32ReadFldAlign(DDRPHY_PHY_RO_1+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), PHY_RO_1_RGS_RX_ARDQS0_DLY_RDY_EYE);
            u1Ready[1] = (U8)u4IO32ReadFldAlign(DDRPHY_PHY_RO_1+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), PHY_RO_1_RGS_RX_ARDQS0_DLY_RDY_EYE);
            u1Ready[2] = (U8)u4IO32ReadFldAlign(DDRPHY_PHY_RO_1+(aru1PhyMap2Channel[0]<< POS_BANK_NUM), PHY_RO_1_RGS_RX_ARDQS1_DLY_RDY_EYE);
            u1Ready[3] = (U8)u4IO32ReadFldAlign(DDRPHY_PHY_RO_1+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), PHY_RO_1_RGS_RX_ARDQS1_DLY_RDY_EYE);
           
            if(enter_or_end ==RX_EYE_SCAN_MOVE_DQS_ENTER)
            {
                u1Ready_all = u1Ready[0] ||u1Ready[1]||u1Ready[2]||u1Ready[3];
            }
            else//RX_EYE_SCAN_MOVE_DQS_END
            {
                u1Ready_all = u1Ready[0] && u1Ready[1] && u1Ready[2] && u1Ready[3];
            }
            
            u1Wait = (u1Ready_all != enter_or_end) ? 1 : 0;
            
            u4TimeCnt--;
            
         }
         while((u1Wait ==1) && (u4TimeCnt >0));
                
         if(u4TimeCnt ==0)
         {
             mcSHOW_DBG_MSG(("DQS setup timeout (enter_or_end = %d) (u1Ready_all %d) u4TimeCnt %d,%d %d %d %d\n", enter_or_end, u1Ready_all, u4TimeCnt, u1Ready[0], u1Ready[1], u1Ready[2], u1Ready[3]));
         }
    }
}
#endif
//-------------------------------------------------------------------------
/** DramcRxEyeScan
 *  start the rx dq eye scan.
 *  @param p                Pointer of context created by DramcCtxCreate.
 *  @param dq_no            (U8): 0~7.
 *  @retval status          (DRAM_STATUS_T): DRAM_OK or DRAM_FAIL 
 */
//-------------------------------------------------------------------------
DRAM_STATUS_T DramcRxEyeScanRun(DRAMC_CTX_T *p, U8 boot_or_run, U8 dq_no)
{
    S16 s2vref, s2dq_dly, s2DelayBegin, s2DelayEnd;
    U32  u4err_value = 0xffffffff, u4sample_cnt=0, u4error_cnt[DQS_NUMBER];;
    #if RX_EYE_SAVE_TO_FILE    
    static U32 arErrorCount[DQS_NUMBER][RX_EYE_VREF_LEVEL][RX_EYE_DELAY_LEVEL];
    static U16 arVrefLevelForLog[RX_EYE_VREF_LEVEL];
    U16 u2VrefIdx, u2DelayIdx, ii,jj, kk;
    #endif
    U16 u2VrefBegin, u2VrefEnd, u2VrefStep;

    U32 uiSrcAddr, uiDestAddr, uiLen;

    U8 u1DramCBit, u1PHYBit;
    #if fcFOR_CHIP_ID == fcA60501
    U8 au1PinMux[2][16] = {{/*CHA B0*/4, 7, 6, 3, 0xff, 0xff, 5, 0,   /*CHA B1*/15, 13, 14, 12, 0xff, 0xff, 11, 10},
                                          {/*CHB B0*/7, 4, 5, 6, 0xff, 0xff, 0, 1,   /*CHB B1*/13, 15, 12, 14, 0xff, 0xff, 11, 9}};  //0xff means N/A
    #endif

#if RX_EYE_SCAN_VREF_EXTEND
    U8 Vref_DDR4_SEL =0;
#endif
    // error handling
    if (!p)
    {
        mcSHOW_ERR_MSG(("context is NULL\n"));
        return DRAM_FAIL;
    }

    if (dq_no > 15)
    {
        mcSHOW_ERR_MSG(("DQ number should be 0~7 for 4 bytes\n"));
        return DRAM_FAIL;
    }

    #if fcFOR_CHIP_ID == fcA60501
    if(au1PinMux[p->channel][dq_no] == 0xff)
    {
        mcSHOW_ERR_MSG(("PHY and DramC no pin mapping, Skip this bit %d!\n", dq_no));
        mcFPRINTF((fp_A60501, "PHY and DramC no pin mapping, Skip this bit %d!\n", dq_no));
        return DRAM_FAIL;
    }
    #endif        

    // check if SoC platform has "%" operation?!    	
    #if RX_EYE_SAVE_TO_FILE    	
    for(ii=0; ii<DQS_NUMBER; ii++)
    {        
        for(jj=0; jj<RX_EYE_VREF_LEVEL; jj++)
        {
            for(kk=0; kk<RX_EYE_DELAY_LEVEL; kk++)
            {
                arErrorCount[ii][jj][kk] =0xffff; 
            }
        }
    }
    #endif

    if(boot_or_run==RX_EYE_SCAN_SCAN_AT_RUN_TIME)
    {
        //uiSrcAddr = DDR_BASE+0x10000000;
        uiSrcAddr = 0x50000000;
        uiLen = 0xff000;
    
#ifdef DUAL_RANKS
        if (uiDualRank) 
        {
        #if SINGLE_CHANNEL_ENABLE
            uiDestAddr = uiSrcAddr+0x30000000;
        #else
            //uiSrcAddr = 0xa0000000;
            //uiDestAddr = uiSrcAddr+0x10000000;
            uiDestAddr = uiSrcAddr+0x60000000;
        #endif                    
        }
        else
#endif
        {
            uiDestAddr = uiSrcAddr+0x08000000;          
        }
    }

    //DRAMC: RG_RX_DQ_EYE_SEL (0~7) for 4 bytes 
    #if fcFOR_CHIP_ID == fcA60501
    u1DramCBit = (au1PinMux[p->channel][dq_no])% DQS_BIT_NUMBER;
    #else
    u1DramCBit = dq_no% DQS_BIT_NUMBER;
    #endif
    u1PHYBit = dq_no % DQS_BIT_NUMBER;
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_STBCAL_F), u1DramCBit, STBCAL_F_RX_DQ_EYE_SEL);

    #if ENABLE_LP4_SW
    if(p->dram_type == TYPE_LPDDR4)
    {
        //select DQ to be scanned (0~7)
        //DDRPHY: RG_??_RX_DQ_EYE_SEL
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE2), u1PHYBit, EYE2_RG_RX_ARDQ_EYE_SEL_B0);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), u1PHYBit, EYEB1_2_RG_RX_ARDQ_EYE_SEL_B1);

        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE3), !boot_or_run, EYE3_RG_RX_ARDQ_EYE_DLY_DQS_BYPASS_B0);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1_3), !boot_or_run, EYEB1_3_RG_RX_ARDQ_EYE_DLY_DQS_BYPASS_B1);
    }
    else  //LPDDR3
    #endif
    {
        //select DQ to be scanned (0~7)
        //DDRPHY: RG_??_RX_DQ_EYE_SEL
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE2), u1PHYBit, EYE2_RG_RX_ARDQ_EYE_SEL_B0);
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), u1PHYBit, EYEB1_2_RG_RX_ARDQ_EYE_SEL_B1);
        
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE3), !boot_or_run, EYE3_RG_RX_ARDQ_EYE_DLY_DQS_BYPASS_B0);
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1_3), !boot_or_run, EYEB1_3_RG_RX_ARDQ_EYE_DLY_DQS_BYPASS_B1);
    }

    #if ENALBE_EYE_SCAN_LOG_DURING_PROCESS
    mcSHOW_DBG_MSG(("===============================================================================\n"));
    mcSHOW_DBG_MSG(("    DQ RX eye scan (channel=%d, dq_%d, DramCBit %d, PHYBit %d)\n", p->channel, dq_no, u1DramCBit, u1PHYBit));
    mcSHOW_DBG_MSG(("===============================================================================\n"));
    #endif
    u2VrefBegin =RX_VREF_RANGE_BEGIN;
    u2VrefEnd =RX_VREF_RANGE_END;
    u2VrefStep=RX_VREF_RANGE_STEP;

    #if RX_EYE_SAVE_TO_FILE    
    u2VrefIdx =0;
    #endif

    #if RX_EYE_SCAN_VREF_EXTEND
    s2vref =(u2VrefEnd-1);
    
    while(1)
    #else
    for(s2vref =(u2VrefEnd-1); s2vref > u2VrefBegin; s2vref-=u2VrefStep)
    //s2vref =0xa;
    #endif
    {
        #if RX_EYE_SCAN_VREF_EXTEND
        arVrefLevelForLog[u2VrefIdx] = s2vref;
        #endif
        
        if(boot_or_run ==RX_EYE_SCAN_SCAN_AT_RUN_TIME)
        {
            DramcDmaEngine((DRAMC_CTX_T *)p, DMA_OP_READ_WRITE, uiSrcAddr, uiDestAddr, uiLen, 8, DMA_CHECK_DATA_ACCESS_ONLY_AND_NO_WAIT, 2);
            DramcRxEyeRuntimeDQS(p, RX_EYE_SCAN_MOVE_DQS_ENTER);
        }
        //s2vref = s2vref | (1<<5);
        #if ENABLE_LP4_SW
         if(p->dram_type == TYPE_LPDDR4)
        {
            //Set Vref voltage
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE2), s2vref, EYE2_RG_RX_ARDQ_EYE_VREF_SEL_B0);
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), s2vref, EYEB1_2_RG_RX_ARDQ_EYE_VREF_SEL_B1);

            //Wait for Vref settles down, 1us is enough
            mcDELAY_US(1);

            //Set DQS delay (RG_??_RX_DQS_EYE_DLY) to 0
            vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_EYE1), 0);
            vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_EYEB1), 0);
        }
        else //LPDDR3
        #endif
        {
            //Set Vref voltage
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE2), s2vref, EYE2_RG_RX_ARDQ_EYE_VREF_SEL_B0);
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), s2vref, EYEB1_2_RG_RX_ARDQ_EYE_VREF_SEL_B1);
            
            //Wait for Vref settles down, 1us is enough
            mcDELAY_US(1);
            
            //Set DQS delay (RG_??_RX_DQS_EYE_DLY) to 0
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE1), 0, PHY_FLD_FULL);
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1), 0, PHY_FLD_FULL);
        }
        
        if(boot_or_run ==RX_EYE_SCAN_SCAN_AT_RUN_TIME)
        {
            DramcRxEyeRuntimeDQS(p, RX_EYE_SCAN_MOVE_DQS_END);
        }
        else //if(boot_or_run ==0)
        {
            DramEyeStbenReset(p);
            DramPhyReset(p);
        }
 	 #if RX_EYE_SAVE_TO_FILE    	                     
        u2DelayIdx=0;
        #endif

        #if ENABLE_LP4_SW
        if(p->dram_type == TYPE_LPDDR4)  //LP4 SBS
        {
            if(p->channel ==CHANNEL_A)
            {
                s2DelayBegin = -30;
                s2DelayEnd = 15;
            }
            else //CHB
            {
                s2DelayBegin = -40;
                s2DelayEnd = 5;
            }
        }
        else //LPDDR3
        #endif
        {
            s2DelayBegin = -60;
            s2DelayEnd = 30;
        }
        
        for (s2dq_dly=s2DelayBegin; s2dq_dly <s2DelayEnd; s2dq_dly+=2)
        {
        
            if(boot_or_run ==RX_EYE_SCAN_SCAN_AT_RUN_TIME)
            {               
                //Reset eye scan counters (reg_sw_rst): 1 to 0
                vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_STBCAL_F), 1, STBCAL_F_REG_SW_RST);
                mcDELAY_US(1);
                vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_STBCAL_F), 0, STBCAL_F_REG_SW_RST);
                
                DramcDmaEngine((DRAMC_CTX_T *)p, DMA_OP_READ_WRITE, uiSrcAddr, uiDestAddr, uiLen, 8, DMA_CHECK_DATA_ACCESS_ONLY_AND_NO_WAIT, 2);
            }
            
            if(s2dq_dly >0)  // move DQ delay
            {
                #if ENABLE_LP4_SW
                if(p->dram_type == TYPE_LPDDR4)
                {
                    //Set DQ delay (RG_??_RX_DQ_EYE_DLY)
                    vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE1), s2dq_dly, EYE1_RG_RX_ARDQ_EYE_R_DLY_B0);
                    vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE1), s2dq_dly, EYE1_RG_RX_ARDQ_EYE_F_DLY_B0);
                    vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1), s2dq_dly, EYEB1_RG_RX_ARDQ_EYE_R_DLY_B1);
                    vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1), s2dq_dly, EYEB1_RG_RX_ARDQ_EYE_F_DLY_B1);
                }
                else
                #endif
                {
                    //Set DQ delay (RG_??_RX_DQ_EYE_DLY)
                    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE1), s2dq_dly, EYE1_RG_RX_ARDQ_EYE_R_DLY_B0);
                    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE1), s2dq_dly, EYE1_RG_RX_ARDQ_EYE_F_DLY_B0);
                    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1), s2dq_dly, EYEB1_RG_RX_ARDQ_EYE_R_DLY_B1);
                    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1), s2dq_dly, EYEB1_RG_RX_ARDQ_EYE_F_DLY_B1);
                }
            }
            else// move DQS delay
            {
                if(boot_or_run ==RX_EYE_SCAN_SCAN_AT_RUN_TIME)
                {                
                    DramcRxEyeRuntimeDQS(p, RX_EYE_SCAN_MOVE_DQS_ENTER);
                }
                
                #if ENABLE_LP4_SW
                if(p->dram_type == TYPE_LPDDR4)
                {
                    //Set DQS delay (RG_??_RX_DQS_EYE_DLY)
                    vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE1), -s2dq_dly, EYE1_RG_RX_ARDQS_EYE_R_DLY_B0);
                    vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE1), -s2dq_dly, EYE1_RG_RX_ARDQS_EYE_F_DLY_B0);
                    vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1), -s2dq_dly, EYEB1_RG_RX_ARDQS_EYE_R_DLY_B1);
                    vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1), -s2dq_dly, EYEB1_RG_RX_ARDQS_EYE_F_DLY_B1);
                }
                else //LPDDR3
                #endif
                {
                    //Set DQS delay (RG_??_RX_DQS_EYE_DLY)
                    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE1), -s2dq_dly, EYE1_RG_RX_ARDQS_EYE_R_DLY_B0);
                    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE1), -s2dq_dly, EYE1_RG_RX_ARDQS_EYE_F_DLY_B0);
                    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1), -s2dq_dly, EYEB1_RG_RX_ARDQS_EYE_R_DLY_B1);
                    vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1), -s2dq_dly, EYEB1_RG_RX_ARDQS_EYE_F_DLY_B1);
                }

                if(boot_or_run ==RX_EYE_SCAN_SCAN_AT_RUN_TIME)
                {
                    DramcRxEyeRuntimeDQS(p, RX_EYE_SCAN_MOVE_DQS_END);
                }
                else //if(boot_or_run ==RX_EYE_SCAN_SCAN_AT_BOOT_TIME)
                {
                    DramEyeStbenReset(p);
                    DramPhyReset(p);
                }
            }

            if(boot_or_run ==RX_EYE_SCAN_SCAN_AT_BOOT_TIME)//boot time
            {
                //Reset eye scan counters (reg_sw_rst): 1 to 0
                vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_STBCAL_F), 1, STBCAL_F_REG_SW_RST);
                mcDELAY_US(1);
                vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_STBCAL_F), 0, STBCAL_F_REG_SW_RST);
                
                u4err_value= TestEngineCompare(p);
            }
            else 
            {
                //run time
                //mcDELAY_MS(1);
                u4err_value = DramcDmaEngine((DRAMC_CTX_T *)p, DMA_OP_READ_WRITE, uiSrcAddr, uiDestAddr, uiLen, 8, DMA_CHECK_COMAPRE_RESULT_ONLY, 2);  // comapre DMA result
            }
            
            //Read the counter values from registers (toggle_cnt*, dq_err_cnt*); 
            //At run time, the counter will change all the time. Therefore, the counter of different bytes and toggle count will be data of different time.
            u4sample_cnt = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_TOGGLE_CNT), TOGGLE_CNT_TOGGLE_CNT);
            u4error_cnt[0] = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DQ_ERR_CNT0), DQ_ERR_CNT0_DQ_ERR_CNT0);
            u4error_cnt[1] = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DQ_ERR_CNT1), DQ_ERR_CNT1_DQ_ERR_CNT1);                

            #if ENABLE_LP4_SW
            if(p->dram_type == TYPE_LPDDR4)
            {
                #ifdef ETT_PRINT_FORMAT
                mcSHOW_DBG_MSG(("RX Vref= %d, Delay= %d, ErrorValue=%d, Toggle %d, ErrorCnt %d\n",  s2vref, s2dq_dly, u4err_value, u4sample_cnt, u4error_cnt[dq_no/8]));    
                #else
                mcSHOW_DBG_MSG(("RX Vref= %2d, Delay= %4d, ErrorValue=%d, Toggle %4d, ErrorCnt %4d\n",  s2vref, s2dq_dly, u4err_value u4sample_cnt, u4error_cnt[dq_no/8]));    
                #endif
            }
            else
            #endif
            {
                u4error_cnt[2] = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DQ_ERR_CNT2), DQ_ERR_CNT2_DQ_ERR_CNT2);
                u4error_cnt[3] = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DQ_ERR_CNT3), DQ_ERR_CNT3_DQ_ERR_CNT3);
                #if ENALBE_EYE_SCAN_LOG_DURING_PROCESS
                #ifdef ETT_PRINT_FORMAT
                mcSHOW_DBG_MSG(("RX Vref= %d, Delay= %d,  %d|B0->B3| %d, %d, %d, %d\n", s2vref, s2dq_dly, u4sample_cnt,u4error_cnt[0], u4error_cnt[2],u4error_cnt[1],u4error_cnt[3]));    
                #else
                mcSHOW_DBG_MSG(("RX Vref= %2d, Delay= %4d,  %d|B0->B3| %4d, %4d, %4d, %4d\n", s2vref, s2dq_dly, u4sample_cnt,u4error_cnt[0], u4error_cnt[2],u4error_cnt[1],u4error_cnt[3]));    
                #endif
                #endif
            }
            
            #if RX_EYE_SAVE_TO_FILE      
            arErrorCount[0][u2VrefIdx][u2DelayIdx] = u4error_cnt[0];
            arErrorCount[1][u2VrefIdx][u2DelayIdx] = u4error_cnt[1];
            if(p->dram_type == TYPE_LPDDR3)
            {
                arErrorCount[2][u2VrefIdx][u2DelayIdx] = u4error_cnt[2];
                arErrorCount[3][u2VrefIdx][u2DelayIdx] = u4error_cnt[3];
            }

            if(u2DelayIdx<RX_EYE_DELAY_LEVEL)
            {
                u2DelayIdx ++;
            }
            else
            {
                mcSHOW_DBG_MSG(("\n\n[DramcRxEyeScan] WARNING: Out of error count array size, Delay %d\n\n", u2DelayIdx));    
            }
            #endif
        }
        
        #if RX_EYE_SAVE_TO_FILE    
        if(u2VrefIdx<RX_EYE_VREF_LEVEL)
        {
            u2VrefIdx ++;
        }
        else
        {
            mcSHOW_DBG_MSG(("\n\n[DramcRxEyeScan] WARNING: Out of error count array size, Vref %d\n\n", u2VrefIdx));    
        }
        #endif

        #if RX_EYE_SCAN_VREF_EXTEND
        s2vref-=u2VrefStep;

        if( s2vref < u2VrefBegin)  // switch to LP4 scan range
        {
            if(Vref_DDR4_SEL==0)
            {
                s2vref =(u2VrefEnd-1);
                Vref_DDR4_SEL =1; // LP4 range
                
                vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE3), 0, EYE3_RG_RX_ARDQ_DDR3_SEL_B0);
                vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1_3), 0, EYEB1_3_RG_RX_ARDQ_DDR3_SEL_B1);
                
                vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE3), 1, EYE3_RG_RX_ARDQ_DDR4_SEL_B0);
                vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1_3), 1, EYEB1_3_RG_RX_ARDQ_DDR4_SEL_B1);

                vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE2), 0x3f, EYE2_RG_RX_ARDQ_VREF_SEL_B0);
                vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), 0x3f, EYEB1_2_RG_RX_ARDQ_VREF_SEL_B1);
                mcSHOW_DBG_MSG(("\n\n[DramcRxEyeScan] Change to LP4 Vref range\n\n"));  
            }
            else  
            {     
                // Restore LP3 oringinal Vref setttings
                vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE3), 1, EYE3_RG_RX_ARDQ_DDR3_SEL_B0);
                vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1_3), 1, EYEB1_3_RG_RX_ARDQ_DDR3_SEL_B1);
                
                vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE3), 0, EYE3_RG_RX_ARDQ_DDR4_SEL_B0);
                vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1_3), 0, EYEB1_3_RG_RX_ARDQ_DDR4_SEL_B1);

                vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE2), 0xe, EYE2_RG_RX_ARDQ_VREF_SEL_B0);
                vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), 0xe, EYEB1_2_RG_RX_ARDQ_VREF_SEL_B1);
                mcSHOW_DBG_MSG(("\n\n[DramcRxEyeScan] Restore LP3 Vref range\n\n"));
                break;
            }
        }
        #endif
        
        #if ENALBE_EYE_SCAN_LOG_DURING_PROCESS
        mcSHOW_DBG_MSG(("\n"));   
        #else
        mcSHOW_DBG_MSG(("."));  
        #endif
    }

    #if !ENALBE_EYE_SCAN_LOG_DURING_PROCESS
    mcSHOW_DBG_MSG(("\n")); 
    #endif

#if 0  // When run time eye scan, don't disable RX eye scan.
    //Disable DQ eye scan (RG_RX_EYE_SCAN_EN=0)
    vIO32WriteFldAlign_All(DRAMC_REG_ADDR(DRAMC_REG_STBCAL_F), 0, STBCAL_F_RG_EX_EYE_SCAN_EN);

    #if ENABLE_LP4_SW
    if(p->dram_type == TYPE_LPDDR4)
    {
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE3), 1, EYE3_RG_RX_ARDQ_EYE_DLY_DQS_BYPASS_B0);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1_3), 1, EYEB1_3_RG_RX_ARDQ_EYE_DLY_DQS_BYPASS_B1);
        
        //Disable DQ eye scan (RG_RX_EYE_SCAN_EN=0, RG_RX_*RDQ_VREF_EN_B*=0, RG_RX_*RDQ_EYE_VREF_EN_B*=0, RG_RX_*RDQ_SMT_EN_B*=0)
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE2), 0, EYE2_RG_RX_ARDQ_EYE_VREF_EN_B0);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), 0, EYEB1_2_RG_RX_ARDQ_EYE_VREF_EN_B1);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_TXDQ3), 0, TXDQ3_RG_RX_ARDQ_SMT_EN_B0); 
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_RXDQ13),0, RXDQ13_RG_RX_ARDQ_SMT_EN_B1);   
    }
    else //LPDDR3
    #endif
    {    
        //Disable DQ eye scan (RG_RX_EYE_SCAN_EN=0, RG_RX_*RDQ_VREF_EN_B*=0, RG_RX_*RDQ_EYE_VREF_EN_B*=0, RG_RX_*RDQ_SMT_EN_B*=0)
        vIO32WriteFldAlign_All((DDRPHY_EYE2), 0, EYE2_RG_RX_ARDQ_EYE_VREF_EN_B0);
        vIO32WriteFldAlign_All((DDRPHY_EYEB1_2), 0, EYEB1_2_RG_RX_ARDQ_EYE_VREF_EN_B1);
        vIO32WriteFldAlign_All((DDRPHY_TXDQ3), 0, TXDQ3_RG_RX_ARDQ_SMT_EN_B0); 
        vIO32WriteFldAlign_All((DDRPHY_RXDQ13),0, RXDQ13_RG_RX_ARDQ_SMT_EN_B1);   
    }

#endif
    #if RX_EYE_SAVE_TO_FILE    
    for(ii=0; ii<DQS_NUMBER; ii++)
    //ii = dq_no/8;
    {
        //if((p->dram_type == TYPE_LPDDR4) && ii>1)
        //{
         //   break;
        //}
 
        mcSHOW_DBG_MSG(("===============================================================================\n"));
        mcSHOW_DBG_MSG(("    DQ RX eye scan (channel=%d, Byte %d, dq_%d, DramCBit %d, PHYBit %d)\n", p->channel, ii, dq_no, u1DramCBit, u1PHYBit));
        mcSHOW_DBG_MSG(("===============================================================================\n"));
        mcFPRINTF((fp_A60501, "===============================================================================\n"));
        mcFPRINTF((fp_A60501,"    DQ RX eye scan (channel=%d, Byte %d, dq_%d, DramCBit %d, PHYBit %d)\n", p->channel, ii, dq_no, u1DramCBit, u1PHYBit));
        mcFPRINTF((fp_A60501, "===============================================================================\n"));
        
        for(jj=0; jj<RX_EYE_VREF_LEVEL; jj++)
        {
            #if RX_EYE_SCAN_VREF_EXTEND
            mcSHOW_DBG_MSG(("%d. Vref = %X, ", jj, arVrefLevelForLog[jj]));  
            #endif
            
            for(kk=0; kk<RX_EYE_DELAY_LEVEL; kk++)
            {
                if(arErrorCount[ii][jj][kk] != 0xffff)
                {
                    mcSHOW_DBG_MSG(("%d, ", arErrorCount[ii][jj][kk])); 
                    mcFPRINTF((fp_A60501, "%8d, ", arErrorCount[ii][jj][kk])); 
                }
            }
            mcSHOW_DBG_MSG(("\n")); 
            mcFPRINTF((fp_A60501, "\n")); 
        }
        mcSHOW_DBG_MSG(("\n")); 
        mcFPRINTF((fp_A60501, "\n")); 
    }
    #endif
    return DRAM_OK;

    // log example
    /*
 ===============================================================================
     DQ RX eye scan (channel=1, byte=0, dq_0)
 ===============================================================================
 2450, 2311, 2304, 2299, 1721, 1628, 1466,  946,  814,  635,  507,  232,   48,   26,    7,    1,    3,    4,    1,    5,   24,  140,  386, 1435, 2304, 2560, 2560, 2575, 2816, 2816, 2816, 2853, 3072, 3072, 3072, 
 2313, 2304, 2304, 2202, 1655, 1620, 1223,  906,  713,  564,  278,   70,   28,    1,    0,    0,    0,    0,    0,    0,    1,    6,  130,  225, 1823, 2548, 2560, 2564, 2813, 2816, 2816, 3069, 3072, 3072, 3072, 
 2305, 2304, 2304, 1786, 1620, 1555,  961,  816,  609,  444,   60,   43,    4,    0,    0,    0,    0,    0,    0,    0,    0,    0,    8,  127,  789, 2265, 2560, 2560, 2571, 2816, 2994, 3072, 3072, 3072, 3072, 
 2305, 2304, 2246, 1686, 1589, 1248,  909,  660,  577,  231,   47,   31,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    5,  191, 1233, 2559, 2560, 2563, 2600, 3071, 3072, 3072, 3072, 3072, 
 2304, 2304, 2028, 1627, 1375, 1027,  848,  639,  516,   88,   30,    1,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,   54,  617, 2175, 2554, 2560, 2571, 3072, 3072, 3072, 3072, 3072, 
 2304, 2269, 1757, 1569, 1192,  947,  700,  559,  274,   44,    2,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,  208, 1005, 2300, 2560, 2808, 2874, 3072, 3072, 3072, 3072, 
 2302, 2141, 1672, 1350, 1000,  834,  602,  412,   63,   33,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,   67,  633, 1644, 2547, 2816, 2985, 3072, 3072, 3072, 3072, 
 2301, 1936, 1559, 1159,  852,  658,  504,  223,   45,    7,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,  278,  949, 2414, 2816, 3065, 3072, 3072, 3072, 3072, 
 2189, 1920, 1444,  994,  692,  561,  327,   66,   42,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,  110,  367, 1968, 2847, 3072, 3072, 3072, 3072, 3072, 
 2059, 1967, 1452,  826,  609,  441,  105,   44,   29,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    1,  161, 1640, 2965, 3072, 3072, 3072, 3072, 3072, 
 2208, 1948, 1561,  946,  548,  261,   46,   42,    5,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    1,  370, 1392, 2816, 3066, 3072, 3072, 3072, 3072, 
 2303, 2019, 1546, 1074,  404,   57,   44,   30,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,  250,  466, 1631, 2651, 2995, 3072, 3072, 3072, 3072, 
 2305, 2270, 1573, 1279,  390,   72,   41,    4,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,  418,  738, 1850, 2634, 3057, 3072, 3072, 3072, 3072, 
 2305, 2303, 1655, 1334,  723,  171,   25,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,  255,  712, 1468, 2149, 2709, 3072, 3072, 3072, 3072, 3072, 
 2305, 2247, 1953, 1323,  947,  426,   14,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    1,  492, 1271, 1854, 2308, 2573, 3072, 3072, 3072, 3072, 3072, 
 2303, 2221, 2211, 1449, 1150,  704,   91,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,  320,  767, 1775, 2193, 2552, 2562, 2923, 3072, 3072, 3072, 3072, 
 
 ===============================================================================
     DQ RX eye scan (channel=1, byte=1, dq_0)
 ===============================================================================
 2311, 2307, 2304, 2278, 2016, 1674, 1280, 1214,  929,  503,  377,  365,  359,  343,  349,  359,  373,  377,  396,  446, 1102, 1977, 2550, 2563, 2563, 2572, 2816, 2816, 2816, 2816, 3066, 3071, 3072, 3072, 3072, 
 2308, 2304, 2304, 2150, 1832, 1458, 1270, 1137,  615,  362,  232,  149,  139,  140,  144,  163,  211,  269,  336,  359,  673, 1300, 2223, 2555, 2565, 2564, 2782, 2816, 2816, 2816, 3070, 3072, 3072, 3072, 3072, 
 2304, 2304, 2304, 2050, 1708, 1307, 1198,  924,  388,  168,   59,   38,   24,   28,   27,   44,   82,  129,  199,  325,  354,  854, 1433, 2367, 2561, 2563, 2572, 2816, 2816, 2927, 3072, 3072, 3072, 3072, 3072, 
 2305, 2304, 2303, 2013, 1516, 1268, 1101,  545,  179,   54,    3,    2,    0,    1,    1,    2,    1,   28,   88,  141,  322,  590, 1097, 1621, 2535, 2564, 2562, 2816, 2816, 3069, 3072, 3072, 3072, 3072, 3072, 
 2304, 2304, 2168, 1838, 1332, 1213,  815,  304,   73,    2,    4,    0,    0,    0,    1,    1,    1,    1,    2,   24,  135,  295,  726, 1170, 2182, 2560, 2562, 2579, 2816, 3072, 3072, 3072, 3072, 3072, 3072, 
 2304, 2304, 1937, 1672, 1271, 1125,  474,  124,   22,    0,    0,    0,    0,    1,    0,    0,    2,    2,    0,    0,   28,  166,  513,  703, 1374, 2506, 2561, 2565, 2998, 3072, 3072, 3072, 3072, 3072, 3072, 
 2304, 2299, 1813, 1468, 1201,  873,  178,   47,    3,    1,    0,    0,    0,    0,    0,    0,    0,    0,    0,    1,    2,   58,  189,  588, 1062, 1768, 2560, 2563, 2878, 3072, 3072, 3072, 3072, 3072, 3072, 
 2304, 2086, 1778, 1308, 1153,  475,   56,   31,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    1,    0,  134,  206,  669, 1149, 2491, 2574, 2820, 3066, 3072, 3072, 3072, 3072, 3072, 
 2246, 1862, 1645, 1201,  844,  166,   41,    6,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    1,   11,  112,  503,  949, 1626, 2808, 2818, 3072, 3072, 3072, 3072, 3072, 3072, 
 2011, 1775, 1426, 1170,  430,   61,   10,    1,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    1,    1,   13,  185,  665, 1173, 2617, 2904, 3072, 3072, 3072, 3072, 3072, 3072, 
 1967, 1725, 1219,  842,  224,   43,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    1,   78,  431, 1161, 2096, 3062, 3072, 3072, 3072, 3072, 3072, 3072, 
 2044, 1749, 1098,  433,   83,   18,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    1,    5,  188,  940, 1751, 3060, 3072, 3072, 3072, 3072, 3072, 3072, 
 2048, 1878,  940,  316,   45,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    1,  316,  853, 2013, 3016, 3072, 3072, 3072, 3072, 3072, 3072, 
 2048, 1978,  934,  294,   56,    1,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    1,  260,  925, 2077, 2667, 3072, 3072, 3072, 3072, 3072, 3072, 
 2048, 1974, 1267,  402,   91,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,  211,  313, 1214, 2095, 2596, 3065, 3072, 3072, 3072, 3072, 3072, 
 2056, 1963, 1556,  535,  130,    2,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    1,  254,  728, 1563, 2004, 2892, 3072, 3072, 3072, 3072, 3072, 3072, 
   */
}
#endif


#ifdef TX_EYE_SCAN
#define TX_EYE_VREF_LEVEL   32
#define TX_EYE_DELAY_LEVEL  32//40
#define TX_EYE_SAVE_TO_FILE 1
#define ENALBE_EYE_SCAN_LOG_DURING_PROCESS 1
#define TX_EYE_SCAN_VREF_EXTEND 1


void DramcTxEyeScanInit(DRAMC_CTX_T *p)
{   
#if 0
    vIO32WriteFldMulti((DDRPHY_PLL15)+(1 <<POS_BANK_NUM), P_Fld(0, PLL15_RG_RPHYPLL_TST_SEL) | P_Fld(1, PLL15_RG_RPHYPLL_TST_EN) | \
                            P_Fld(0, PLL15_RG_RPHYPLL_TSTCK_EN) | P_Fld(0, PLL15_RG_RPHYPLL_TSTFM_EN) | \
                            P_Fld(0, PLL15_RG_RPHYPLL_TSTOD_EN) | P_Fld(1, PLL15_RG_RPHYPLL_TSTOP_EN) | \
                            P_Fld(1, PLL15_RG_RVREF_VREF_EN) );
    vIO32WriteFldMulti((DDRPHY_PLL15)+(1 <<POS_BANK_NUM), P_Fld(0x3f, PLL15_RG_RVREF_SEL_DQ) | \
                            P_Fld(1, PLL15_RG_RVREF_DDR3_SEL) | P_Fld(0, PLL15_RG_RVREF_DDR4_SEL));
#endif
}

DRAM_STATUS_T DramcTxEyeScanDelay(DRAMC_CTX_T *p, S16 uiDelay[], S16 uiOenDelay[])
{
    U8 u1ByteIdx;
    U8 ucdq_pi[DQS_NUMBER], ucdq_ui_small[DQS_NUMBER], ucdq_ui_large[DQS_NUMBER];
    U8 ucdq_oen_ui_small[DQS_NUMBER], ucdq_oen_ui_large[DQS_NUMBER];

    for(u1ByteIdx=0; u1ByteIdx<DQS_NUMBER; u1ByteIdx++)
    {
        TxWinTransferDelayToUIPI(p, uiDelay[u1ByteIdx], 0, 0, &(ucdq_ui_large[u1ByteIdx]), &ucdq_ui_small[u1ByteIdx], &ucdq_pi[u1ByteIdx]);
        TxWinTransferDelayToUIPI(p, uiOenDelay[u1ByteIdx], 0, 0, &ucdq_oen_ui_large[u1ByteIdx], &ucdq_oen_ui_small[u1ByteIdx], &ucdq_pi[u1ByteIdx]);

        #if ENALBE_EYE_SCAN_LOG_DURING_PROCESS
        #ifdef ETT_PRINT_FORMAT
        mcSHOW_DBG_MSG(("Delay = %d |%d %d %d \n",uiDelay[u1ByteIdx], ucdq_ui_large[u1ByteIdx],ucdq_ui_small[u1ByteIdx], ucdq_pi[u1ByteIdx]));
        #else
        mcSHOW_DBG_MSG(("Delay = %d |%4d %4d %4d \n",uiDelay[u1ByteIdx], ucdq_ui_large[u1ByteIdx],ucdq_ui_small[u1ByteIdx], ucdq_pi[u1ByteIdx]));
        #endif
        mcFPRINTF((fp_A60501, "Delay = %d | %4d %4d %4d \n",uiDelay[u1ByteIdx], ucdq_ui_large[u1ByteIdx],ucdq_ui_small[u1ByteIdx], ucdq_pi[u1ByteIdx]));  
        #endif //#if ENALBE_EYE_SCAN_LOG_DURING_PROCESS
    }

    //mcSHOW_DBG_MSG(("\nTX manual DQ delay = %d\n",uiDelay));
    //mcSHOW_DBG_MSG(("Reg Values = (%d, %d, %d)\n",ucdq_ui_large, ucdq_ui_small,ucdq_pi ));
    //mcSHOW_DBG_MSG(("Reg OEN Values = (%d, %d, %d)\n",ucdq_oen_ui_large, ucdq_oen_ui_small, ucdq_pi));

    //TXDLY_DQ , TXDLY_OEN_DQ
    vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH7), \
                                        P_Fld(ucdq_ui_large[0], SELPH7_TXDLY_DQ0) | \
                                        P_Fld(ucdq_ui_large[1], SELPH7_TXDLY_DQ1) | \
                                        P_Fld(ucdq_ui_large[2], SELPH7_TXDLY_DQ2) | \
                                        P_Fld(ucdq_ui_large[3], SELPH7_TXDLY_DQ3) | \
                                        P_Fld(ucdq_oen_ui_large[0], SELPH7_TXDLY_OEN_DQ0) | \
                                        P_Fld(ucdq_oen_ui_large[1], SELPH7_TXDLY_OEN_DQ1) | \
                                        P_Fld(ucdq_oen_ui_large[2], SELPH7_TXDLY_OEN_DQ2) | \
                                        P_Fld(ucdq_oen_ui_large[3], SELPH7_TXDLY_OEN_DQ3));

    // DLY_DQ[1:0]
   vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH10), \
                                    P_Fld(ucdq_ui_small[0], SELPH10_DLY_DQ0) | \
                                    P_Fld(ucdq_ui_small[1], SELPH10_DLY_DQ1) | \
                                    P_Fld(ucdq_ui_small[2], SELPH10_DLY_DQ2) | \
                                    P_Fld(ucdq_ui_small[3], SELPH10_DLY_DQ3) | \
                                    P_Fld(ucdq_oen_ui_small[0], SELPH10_DLY_OEN_DQ0) | \
                                    P_Fld(ucdq_oen_ui_small[1], SELPH10_DLY_OEN_DQ1) | \
                                    P_Fld(ucdq_oen_ui_small[2], SELPH10_DLY_OEN_DQ2) | \
                                    P_Fld(ucdq_oen_ui_small[3], SELPH10_DLY_OEN_DQ3));
    // DLY_DQ[2]
   vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH22), \
                                    P_Fld(ucdq_ui_small[0]>>2, SELPH22_DLY_DQ0_2) | \
                                    P_Fld(ucdq_ui_small[1]>>2, SELPH22_DLY_DQ1_2)| \
                                    P_Fld(ucdq_ui_small[2]>>2, SELPH22_DLY_DQ2_2) | \
                                    P_Fld(ucdq_ui_small[3]>>2, SELPH22_DLY_DQ3_2) | \
                                    P_Fld(ucdq_oen_ui_small[0]>>2, SELPH22_DLY_OEN_DQ0_2) | \
                                    P_Fld(ucdq_oen_ui_small[1]>>2, SELPH22_DLY_OEN_DQ1_2) | \
                                    P_Fld(ucdq_oen_ui_small[2]>>2, SELPH22_DLY_OEN_DQ2_2) | \
                                    P_Fld(ucdq_oen_ui_small[3]>>2, SELPH22_DLY_OEN_DQ3_2));

    //TXDLY_DQM , TXDLY_OEN_DQM
    vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH8), \
                                        P_Fld(ucdq_ui_large[0], SELPH8_TXDLY_DQM0) | \
                                        P_Fld(ucdq_ui_large[1], SELPH8_TXDLY_DQM1) | \
                                        P_Fld(ucdq_ui_large[2], SELPH8_TXDLY_DQM2) | \
                                        P_Fld(ucdq_ui_large[3], SELPH8_TXDLY_DQM3) | \
                                        P_Fld(ucdq_oen_ui_large[0], SELPH8_TXDLY_OEN_DQM0) | \
                                        P_Fld(ucdq_oen_ui_large[1], SELPH8_TXDLY_OEN_DQM1) | \
                                        P_Fld(ucdq_oen_ui_large[2], SELPH8_TXDLY_OEN_DQM2) | \
                                        P_Fld(ucdq_oen_ui_large[3], SELPH8_TXDLY_OEN_DQM3));

    // DLY_DQM[1:0]
   vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH10), \
                                    P_Fld(ucdq_ui_small[0], SELPH10_DLY_DQM0) | \
                                    P_Fld(ucdq_ui_small[1], SELPH10_DLY_DQM1) | \
                                    P_Fld(ucdq_ui_small[2], SELPH10_DLY_DQM2) | \
                                    P_Fld(ucdq_ui_small[3], SELPH10_DLY_DQM3) | \
                                    P_Fld(ucdq_oen_ui_small[0], SELPH10_DLY_OEN_DQM0) | \
                                    P_Fld(ucdq_oen_ui_small[1], SELPH10_DLY_OEN_DQM1) | \
                                    P_Fld(ucdq_oen_ui_small[2], SELPH10_DLY_OEN_DQM2) | \
                                    P_Fld(ucdq_oen_ui_small[3], SELPH10_DLY_OEN_DQM3));
    // DLY_DQM[2]
   vIO32WriteFldMulti(DRAMC_REG_ADDR(DRAMC_REG_SELPH22), \
                                    P_Fld(ucdq_ui_small[0]>>2, SELPH22_DLY_DQM0_2) | \
                                    P_Fld(ucdq_ui_small[1]>>2, SELPH22_DLY_DQM1_2)| \
                                    P_Fld(ucdq_ui_small[2]>>2, SELPH22_DLY_DQM2_2) | \
                                    P_Fld(ucdq_ui_small[3]>>2, SELPH22_DLY_DQM3_2) | \
                                    P_Fld(ucdq_oen_ui_small[0]>>2, SELPH22_DLY_OEN_DQM0_2) | \
                                    P_Fld(ucdq_oen_ui_small[1]>>2, SELPH22_DLY_OEN_DQM1_2) | \
                                    P_Fld(ucdq_oen_ui_small[2]>>2, SELPH22_DLY_OEN_DQM2_2) | \
                                    P_Fld(ucdq_oen_ui_small[3]>>2, SELPH22_DLY_OEN_DQM3_2));
    
    //set to registers, PI DQ (per byte)
    #if ENABLE_LP4_SW
    if(p->dram_type ==TYPE_LPDDR4)
    {
        // update TX DQ PI delay, for rank 0 // need to take care rank 1 and 2
        vIO32WriteFldMulti(DRAMC_REG_ADDR(DDRPHY_ARPI_DQ), \
                                        P_Fld(ucdq_pi, ARPI_DQ_RK0_ARPI_DQ_B0) | P_Fld(ucdq_pi, ARPI_DQ_RK0_ARPI_DQ_B1));
    }
    else //LPDDR3
    #endif
    {
        vIO32WriteFldMulti(DDRPHY_ARPI_DQ+(aru1PhyMap2Channel[0]<< POS_BANK_NUM),\
                                    P_Fld(ucdq_pi[0], ARPI_DQ_RK0_ARPI_DQ_B0) | P_Fld(ucdq_pi[2], ARPI_DQ_RK0_ARPI_DQ_B1));
    
        vIO32WriteFldMulti(DDRPHY_ARPI_DQ+(aru1PhyMap2Channel[1]<< POS_BANK_NUM), \
                                        P_Fld(ucdq_pi[1], ARPI_DQ_RK0_ARPI_DQ_B0) | P_Fld(ucdq_pi[3], ARPI_DQ_RK0_ARPI_DQ_B1));
    }

#if 0
    u4err_value= TestEngineCompare(p);
#endif
    //DramPhyReset(p);

	return DRAM_OK;
}


//-------------------------------------------------------------------------
/** DramcTxEyeScan
 *  start the rx dq eye scan.
 *  @param p                Pointer of context created by DramcCtxCreate.
 *  @param dq_no            (U8): 0~7.
 *  @retval status          (DRAM_STATUS_T): DRAM_OK or DRAM_FAIL 
 */
//-------------------------------------------------------------------------
DRAM_STATUS_T DramcTxEyeScanRun(DRAMC_CTX_T *p, U8 u1PatternDMA)
{
    S16 s2vref, s2dq_dly, s2Delay[DQS_NUMBER],  s2Delay_OEN[DQS_NUMBER];
    U32  u4err_value = 0xffffffff, u4sample_cnt=0, u4error_cnt[DQS_NUMBER];
    #if TX_EYE_SAVE_TO_FILE    
    static U8 arErrorCount[DQ_DATA_WIDTH][TX_EYE_VREF_LEVEL][TX_EYE_DELAY_LEVEL];
    static U16 arVrefLevelForLog[TX_EYE_VREF_LEVEL];
    U16 u2VrefIdx, u2DelayIdx, ii,jj, kk;
    #endif
    U16 u2VrefBegin, u2VrefEnd, u2VrefStep;
    U8 u1ByteIdx, u1BitIdx;
    U32 uiSrcAddr, uiDestAddr, uiLen;

#if TX_EYE_SCAN_VREF_EXTEND
    U8 Vref_DDR4_SEL =0;
#endif
    // error handling
    if (!p)
    {
        mcSHOW_ERR_MSG(("context is NULL\n"));
        return DRAM_FAIL;
    }

    // check if SoC platform has "%" operation?!    	
    #if TX_EYE_SAVE_TO_FILE    	
    for(ii=0; ii<DQ_DATA_WIDTH; ii++)
    {        
        for(jj=0; jj<TX_EYE_VREF_LEVEL; jj++)
        {
            for(kk=0; kk<TX_EYE_DELAY_LEVEL; kk++)
            {
                arErrorCount[ii][jj][kk] =0xff; 
            }
        }        
    }
    #endif

    //DMA init address
    if(u1PatternDMA)
    {
        //uiSrcAddr = DDR_BASE+0x10000000;
        uiSrcAddr = 0x50000000;
        uiLen = 0xff000;
    
#ifdef DUAL_RANKS
        if (uiDualRank) 
        {
        #if SINGLE_CHANNEL_ENABLE
            uiDestAddr = uiSrcAddr+0x30000000;
        #else
            //uiSrcAddr = 0xa0000000;
            //uiDestAddr = uiSrcAddr+0x10000000;
            uiDestAddr = uiSrcAddr+0x60000000;
        #endif                    
        }
        else
#endif
        {
            uiDestAddr = uiSrcAddr+0x08000000;          
        }
    }
    
    u2VrefBegin =0;
    u2VrefEnd =0x3f;
    u2VrefStep=4;

    #if TX_EYE_SAVE_TO_FILE    
    u2VrefIdx =0;
    #endif

    #if TX_EYE_SCAN_VREF_EXTEND
    s2vref =u2VrefEnd;
    
    while(1)
    #else
    for(s2vref =u2VrefEnd; s2vref > u2VrefBegin; s2vref-=u2VrefStep)
    #endif
    {
        #if TX_EYE_SCAN_VREF_EXTEND
        arVrefLevelForLog[u2VrefIdx] = s2vref;
        #endif
        
        //Set Vref voltage
        vIO32WriteFldAlign((DDRPHY_PLL15) + (1 <<POS_BANK_NUM), s2vref, PLL15_RG_RVREF_SEL_DQ);
        
        //Wait for Vref settles down, 1sec. is enough
        mcDELAY_MS(1000);
                
 	 #if TX_EYE_SAVE_TO_FILE    	                     
        u2DelayIdx=0;
        #endif
        
        for (s2dq_dly=-20; s2dq_dly <40; s2dq_dly+=2)
        {
            for(u1ByteIdx=0; u1ByteIdx<DQS_NUMBER; u1ByteIdx++)
            {
                s2Delay[u1ByteIdx] = (S16)aru2TXCaliDelay[p->channel][u1ByteIdx] +s2dq_dly;
                s2Delay_OEN[u1ByteIdx] = (S16)aru2TXCaliDelay_OEN[p->channel][u1ByteIdx] +s2dq_dly;
            }
        
            DramcTxEyeScanDelay(p, s2Delay, s2Delay_OEN);

            if(u1PatternDMA)
            {
                #if SINGLE_CHANNEL_ENABLE
                u4err_value = DramcDmaEngine((DRAMC_CTX_T *)p, DMA_OP_READ_WRITE, uiSrcAddr, uiDestAddr, uiLen, 8, DMA_CHECK_DATA_ACCESS_AND_COMPARE, 1);  // comapre DMA result
                #else
                u4err_value = DramcDmaEngine((DRAMC_CTX_T *)p, DMA_OP_READ_WRITE, uiSrcAddr, uiDestAddr, uiLen, 8, DMA_CHECK_DATA_ACCESS_AND_COMPARE, 2);  // comapre DMA result
                #endif
            }
            else
                u4err_value= TestEngineCompare(p);
            
            #if ENALBE_EYE_SCAN_LOG_DURING_PROCESS
            #ifdef ETT_PRINT_FORMAT
            mcSHOW_DBG_MSG(("TX Vref= %d, Delay= %d,  u4err_value=%X\n", s2vref, s2dq_dly, u4err_value));    
            #else
            mcSHOW_DBG_MSG(("TX Vref= %2d, Delay= %4d, u4err_value= %x\n", s2vref, s2dq_dly, u4err_value));    
            #endif
            #endif
            
            #if TX_EYE_SAVE_TO_FILE   

            for(u1BitIdx=0; u1BitIdx<DQ_DATA_WIDTH; u1BitIdx++)
            {
                arErrorCount[u1BitIdx][u2VrefIdx][u2DelayIdx] = ((u4err_value >>u1BitIdx)& 0x1);
            }

            if(u2DelayIdx<TX_EYE_DELAY_LEVEL)
            {
                u2DelayIdx ++;
            }
            else
            {
                mcSHOW_DBG_MSG(("\n\n[DramcTxEyeScan] WARNING: Out of error count array size, Delay %d\n\n", u2DelayIdx));    
            }
            #endif
        }
        
        #if TX_EYE_SAVE_TO_FILE    
        if(u2VrefIdx<TX_EYE_VREF_LEVEL)
        {
            u2VrefIdx ++;
        }
        else
        {
            mcSHOW_DBG_MSG(("\n\n[DramcTxEyeScan] WARNING: Out of error count array size, Vref %d\n\n", u2VrefIdx));    
        }
        #endif

        #if TX_EYE_SCAN_VREF_EXTEND
        s2vref-=u2VrefStep;

        if( s2vref < u2VrefBegin)  // switch to LP4 scan range
        {
            if(Vref_DDR4_SEL==0)
            {
                s2vref =u2VrefEnd;
                Vref_DDR4_SEL =1; // LP4 range
                
                vIO32WriteFldAlign((DDRPHY_PLL15)+(1 <<POS_BANK_NUM), 0, PLL15_RG_RVREF_DDR3_SEL);
                vIO32WriteFldAlign((DDRPHY_PLL15)+(1 <<POS_BANK_NUM), 1, PLL15_RG_RVREF_DDR4_SEL);    
                //vIO32WriteFldAlign((DDRPHY_PLL15)+(1 <<POS_BANK_NUM), 0x3f, PLL15_RG_RVREF_SEL_DQ);
                mcSHOW_DBG_MSG(("\n\n[DramcTxEyeScan] Change to LP4 Vref range\n\n"));  
            }
            else  
            {     
                // Restore LP3 oringinal Vref setttings
                vIO32WriteFldAlign((DDRPHY_PLL15)+(1 <<POS_BANK_NUM), 1, PLL15_RG_RVREF_DDR3_SEL);
                vIO32WriteFldAlign((DDRPHY_PLL15)+(1 <<POS_BANK_NUM), 0, PLL15_RG_RVREF_DDR4_SEL);    
                vIO32WriteFldAlign((DDRPHY_PLL15)+(1 <<POS_BANK_NUM), 0xf, PLL15_RG_RVREF_SEL_DQ);
                mcSHOW_DBG_MSG(("\n\n[DramcTxEyeScan] Restore LP3 Vref range\n\n"));
                break;
            }
        }
        #endif
        
        #if ENALBE_EYE_SCAN_LOG_DURING_PROCESS
        mcSHOW_DBG_MSG(("\n"));   
        #else
        mcSHOW_DBG_MSG(("."));  
        #endif
    }

    #if !ENALBE_EYE_SCAN_LOG_DURING_PROCESS
    mcSHOW_DBG_MSG(("\n")); 
    #endif

    #if TX_EYE_SAVE_TO_FILE    
    for(ii=0; ii<DQ_DATA_WIDTH; ii++)
    {
        mcSHOW_DBG_MSG(("===============================================================================\n"));
        mcSHOW_DBG_MSG(("    DQ TX eye scan (channel=%d, dq_%d)\n", p->channel, ii));
        mcSHOW_DBG_MSG(("===============================================================================\n"));
        mcFPRINTF((fp_A60501, "===============================================================================\n"));
        mcFPRINTF((fp_A60501,"    DQ TX eye scan (channel=%d, dq_%d)\n", p->channel, ii));
        mcFPRINTF((fp_A60501, "===============================================================================\n"));
        
        for(jj=0; jj<TX_EYE_VREF_LEVEL; jj++)
        {
            #if TX_EYE_SCAN_VREF_EXTEND
            mcSHOW_DBG_MSG(("%d. Vref = %X, ", jj, arVrefLevelForLog[jj]));  
            #endif
            
            for(kk=0; kk<TX_EYE_DELAY_LEVEL; kk++)
            {
                if(arErrorCount[ii][jj][kk] != 0xff)
                {
                    mcSHOW_DBG_MSG(("%d, ", arErrorCount[ii][jj][kk])); 
                    mcFPRINTF((fp_A60501, "%8d, ", arErrorCount[ii][jj][kk])); 
                     mcDELAY_MS(1);
                }
            }
            mcSHOW_DBG_MSG(("\n")); 
            mcFPRINTF((fp_A60501, "\n")); 
        }
        mcSHOW_DBG_MSG(("\n")); 
        mcFPRINTF((fp_A60501, "\n")); 
    }
    #endif
    return DRAM_OK;

    // log example
    /*
 ===============================================================================                                                                                                                         
     DQ TX eye scan (channel=0    dq_18)                                                                                                                     
 ===============================================================================                                                                                                                         
 0. Vref = 0000003F  1   1   1   1   1   1   0   0   0   0   0   0   0   0   0   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1    
 1. Vref = 0000003B  1   1   1   1   1   1   0   0   0   0   0   0   0   0   0   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1    
 2. Vref = 00000037  1   1   1   1   1   1   0   0   0   0   0   0   0   0   0   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1    
 3. Vref = 00000033  1   1   1   1   1   1   0   0   0   0   0   0   0   0   0   0   1   1   1   1   1   1   1   1   1   1   1   1   1   1    
 4. Vref = 0000002F  1   1   1   1   1   1   0   0   0   0   0   0   0   0   0   0   1   1   1   1   1   1   1   1   1   1   1   1   1   1    
 5. Vref = 0000002B  1   1   1   1   1   1   0   0   0   0   0   0   0   0   0   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1    
 6. Vref = 00000027  1   1   1   1   1   1   0   0   0   0   0   0   0   0   0   0   1   1   1   1   1   1   1   1   1   1   1   1   1   1    
 7. Vref = 00000023  1   1   1   1   1   1   0   0   0   0   0   0   0   0   0   0   1   1   1   1   1   1   1   1   1   1   1   1   1   1    
 8. Vref = 0000001F  1   1   1   1   1   0   0   0   0   0   0   0   0   0   0   0   0   1   1   1   1   1   1   1   1   1   1   1   1   1    
 9. Vref = 0000001B  1   1   1   1   1   0   0   0   0   0   0   0   0   0   0   0   0   1   1   1   1   1   1   1   1   1   1   1   1   1    
 10. Vref = 00000017 1   1   1   1   1   0   0   0   0   0   0   0   0   0   0   0   0   1   1   1   1   1   1   1   1   1   1   1   1   1    
 11. Vref = 00000013 1   1   1   1   1   0   0   0   0   0   0   0   0   0   0   0   0   1   1   1   1   1   1   1   1   1   1   1   1   1    
 12. Vref = 0000000F 1   1   1   1   1   0   0   0   0   0   0   0   0   0   0   0   0   1   1   1   1   1   1   1   1   1   1   1   1   1    
 13. Vref = 0000000B 1   1   1   1   1   0   0   0   0   0   0   0   0   0   0   0   0   1   1   1   1   1   1   1   1   1   1   1   1   1    
 14. Vref = 00000007 1   1   1   1   1   0   0   0   0   0   0   0   0   0   0   0   1   1   1   1   1   1   1   1   1   1   1   1   1   1    
 15. Vref = 00000003 1   1   1   1   1   0   0   0   0   0   0   0   0   0   0   0   1   1   1   1   1   1   1   1   1   1   1   1   1   1    
 16. Vref = 0000003F 1   1   1   1   1   0   0   0   0   0   0   0   0   0   0   0   1   1   1   1   1   1   1   1   1   1   1   1   1   1    
 17. Vref = 0000003B 1   1   1   1   1   0   0   0   0   0   0   0   0   0   0   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1    
 18. Vref = 00000037 1   1   1   1   1   1   0   0   0   0   0   0   0   0   0   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1    
 19. Vref = 00000033 1   1   1   1   1   0   0   0   0   0   0   0   0   0   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1    
 20. Vref = 0000002F 1   1   1   1   1   1   0   0   0   0   0   0   0   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1    
 21. Vref = 0000002B 1   1   1   1   1   1   0   0   0   0   0   0   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1    
 22. Vref = 00000027 1   1   1   1   1   1   0   0   0   0   0   0   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1    
 23. Vref = 00000023 1   1   1   1   1   1   0   0   0   0   0   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1    
 24. Vref = 0000001F 1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1    
 25. Vref = 0000001B 1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1    
 26. Vref = 00000017 1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1    
 27. Vref = 00000013 1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1    
 28. Vref = 0000000F 1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1    
 29. Vref = 0000000B 1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1    
 30. Vref = 00000007 1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1    
 31. Vref = 00000003 1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1   1    
   */
}
#endif

//-------------------------------------------------------------------------
/** DramcMiockJmeter
 *  start MIOCK jitter meter.
 *  @param p                Pointer of context created by DramcCtxCreate.
 *  @param block_no         (U8): block 0 or 1.
 *  @retval status          (DRAM_STATUS_T): DRAM_OK or DRAM_FAIL 
 */
//-------------------------------------------------------------------------

#ifdef ENABLE_MIOCK_JMETER
DRAM_STATUS_T DramcMiockJmeter(DRAMC_CTX_T *p)
{
    U8 u1ByteIdx;
    U32 u4RevBX[DQS_NUMBER];
    U8 ucsearch_state, ucdqs_dly, fgcurrent_value, fginitial_value, ucstart_period, ucend_period;
    U32 u4sample_cnt, u4ones_cnt[DQS_NUMBER], u4MPDIV_IN_SEL;
    U16 u2real_freq, u2real_period, u2delay_cell_ps;
    U32 u4prv_register_348, u4prv_register_014, u4prv_register_074, u4prv_register_018, u4prv_register_078;
    U32 u4prv_register_01c, u4prv_register_07c, u4prv_register_020, u4prv_register_080;

    U32 u4RegBak[DQS_NUMBER][4];
    U8 REF_CLK = 52;

#if CHECK_PLL_OK
        if(is_pll_good() == 0) //not all pll is good, use 26M reference clock
#endif
            REF_CLK = 26;    
    // error handling
    if (!p)
    {
        mcSHOW_ERR_MSG(("context is NULL\n"));
        return DRAM_FAIL;
    }

    //backup register value
    u4prv_register_348 = u4IO32Read4B(DRAMC_REG_ADDR(DRAMC_REG_STBCAL_F));

    #if ENABLE_LP4_SW
    if(p->dram_type== TYPE_LPDDR4)
    {
        u4prv_register_014 = u4IO32Read4B(DRAMC_REG_ADDR(DDRPHY_TXDQ3));
        u4prv_register_018 = u4IO32Read4B(DRAMC_REG_ADDR(DDRPHY_EYE1));
        u4prv_register_01c = u4IO32Read4B(DRAMC_REG_ADDR(DDRPHY_EYE2));
        u4prv_register_020 = u4IO32Read4B(DRAMC_REG_ADDR(DDRPHY_EYE3));
        u4prv_register_074 = u4IO32Read4B(DRAMC_REG_ADDR(DDRPHY_RXDQ13));
        u4prv_register_078 = u4IO32Read4B(DRAMC_REG_ADDR(DDRPHY_EYEB1));
        u4prv_register_07c = u4IO32Read4B(DRAMC_REG_ADDR(DDRPHY_EYEB1_2));
        u4prv_register_080 = u4IO32Read4B(DRAMC_REG_ADDR(DDRPHY_EYEB1_3));
    }
    else //LPDDR3
    #endif
    {  
        for(u1ByteIdx=0; u1ByteIdx<DQS_NUMBER; u1ByteIdx++)
        {
            u4RegBak[u1ByteIdx][0] =u4IO32ReadFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_TXDQ3), PHY_FLD_FULL);
            u4RegBak[u1ByteIdx][1] =u4IO32ReadFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_EYE1), PHY_FLD_FULL);
            u4RegBak[u1ByteIdx][2] =u4IO32ReadFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_EYE2), PHY_FLD_FULL);
            u4RegBak[u1ByteIdx][3] =u4IO32ReadFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_EYE3), PHY_FLD_FULL);
        }
    }
    // Bypass DQS glitch-free mode
    // RG_RX_*RDQ_EYE_DLY_DQS_BYPASS_B**
    #if ENABLE_LP4_SW
    if(p->dram_type== TYPE_LPDDR4)
    {
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE3), 1, EYE3_RG_RX_ARDQ_EYE_DLY_DQS_BYPASS_B0);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1_3), 1, EYEB1_3_RG_RX_ARDQ_EYE_DLY_DQS_BYPASS_B1);
    }
    else //LPDDR3
    #endif
    {
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE3), 1, EYE3_RG_RX_ARDQ_EYE_DLY_DQS_BYPASS_B0);
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1_3), 1, EYEB1_3_RG_RX_ARDQ_EYE_DLY_DQS_BYPASS_B1);
    }

    //Enable DQ eye scan
    //RG_??_RX_EYE_SCAN_EN
    //RG_??_RX_VREF_EN 
    //RG_??_RX_SMT_EN
    
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_STBCAL_F), 1, STBCAL_F_RG_EX_EYE_SCAN_EN);
    #if ENABLE_LP4_SW
    if(p->dram_type== TYPE_LPDDR4)
    {
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE2), 1, EYE2_RG_RX_ARDQ_EYE_EN_B0);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), 1, EYEB1_2_RG_RX_ARDQ_EYE_EN_B1);
        //vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE2), 1, EYE2_RG_RX_ARDQ_EYE_VREF_EN_B0);
        //vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), 1, EYEB1_2_RG_RX_ARDQ_EYE_VREF_EN_B1);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE2), 1, EYE2_RG_RX_ARDQ_VREF_EN_B0);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), 1, EYEB1_2_RG_RX_ARDQ_VREF_EN_B1);

        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_TXDQ3), 1, TXDQ3_RG_RX_ARDQ_SMT_EN_B0); 
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_RXDQ13), 1, RXDQ13_RG_RX_ARDQ_SMT_EN_B1);    
    }
    else
    #endif
    {
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE2), 1, EYE2_RG_RX_ARDQ_EYE_EN_B0);
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), 1, EYEB1_2_RG_RX_ARDQ_EYE_EN_B1);

        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE2), 1, EYE2_RG_RX_ARDQ_VREF_EN_B0);
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), 1, EYEB1_2_RG_RX_ARDQ_VREF_EN_B1);
        
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_TXDQ3), 1, TXDQ3_RG_RX_ARDQ_SMT_EN_B0); 
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_RXDQ13), 1, RXDQ13_RG_RX_ARDQ_SMT_EN_B1);    
    }
    
    //RG_RX_JM_SEL
    #if ENABLE_LP4_SW
    if(p->dram_type== TYPE_LPDDR4)
    {
        u4RevBX[0]  = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DDRPHY_TXDQ3), TXDQ3_RG_ARDQ_REV_B0);
        u4RevBX[1]  = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DDRPHY_RXDQ13), RXDQ13_RG_ARDQ_REV_B1);
        
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_TXDQ3), u4RevBX[0] |0x0800, TXDQ3_RG_ARDQ_REV_B0);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_RXDQ13), u4RevBX[1] |0x0800, TXDQ3_RG_ARDQ_REV_B0);
    }
    else  //LPDDR3
    #endif
    {
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_PHY_PDB_PU_0), 1, PHY_PDB_PU_0_RG_RX_ARDQ_JM_SEL_B0);
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_PHY_PDB_PU_1), 1, PHY_PDB_PU_1_RG_RX_ARDQ_JM_SEL_B1);
    }
    
    //Enable MIOCK jitter meter mode ( RG_RX_MIOCK_JIT_EN=1)
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_STBCAL_F), 1, STBCAL_F_RG_RX_MIOCK_JIT_EN);

    //Disable DQ eye scan (b'1), for counter clear
    vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_STBCAL_F), 0, STBCAL_F_RG_EX_EYE_SCAN_EN);

    ucsearch_state = 0;
    for (ucdqs_dly=0; ucdqs_dly<128; ucdqs_dly++)
    {
    
        //Set DQS delay (RG_??_RX_DQS_EYE_DLY)
        #if ENABLE_LP4_SW
        if(p->dram_type== TYPE_LPDDR4)
        {
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE1), ucdqs_dly, EYE1_RG_RX_ARDQS_EYE_R_DLY_B0);
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE1), ucdqs_dly, EYE1_RG_RX_ARDQS_EYE_F_DLY_B0);
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1), ucdqs_dly, EYEB1_RG_RX_ARDQS_EYE_R_DLY_B1);
            vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1), ucdqs_dly, EYEB1_RG_RX_ARDQS_EYE_F_DLY_B1);
        }
        else //LPDDR3
        #endif
        {        
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE1), ucdqs_dly, EYE1_RG_RX_ARDQS_EYE_R_DLY_B0);
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYE1), ucdqs_dly, EYE1_RG_RX_ARDQS_EYE_F_DLY_B0);
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1), ucdqs_dly, EYEB1_RG_RX_ARDQS_EYE_R_DLY_B1);
            vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_EYEB1), ucdqs_dly, EYEB1_RG_RX_ARDQS_EYE_F_DLY_B1);
        }

        DramPhyReset(p);
        
        //Reset eye scan counters (reg_sw_rst): 1 to 0
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_STBCAL_F), 1, STBCAL_F_REG_SW_RST);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_STBCAL_F), 0, STBCAL_F_REG_SW_RST);

        //Enable DQ eye scan (b'1)
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_STBCAL_F), 1, STBCAL_F_RG_EX_EYE_SCAN_EN);
        
        // 2ns/sample, here we delay 1ms about 500 samples
        mcDELAY_US(1000);

        //Disable DQ eye scan (b'1), for counter latch
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_STBCAL_F), 0, STBCAL_F_RG_EX_EYE_SCAN_EN);

        //Read the counter values from registers (toggle_cnt*, dqs_err_cnt*);
        u4sample_cnt = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_TOGGLE_CNT), TOGGLE_CNT_TOGGLE_CNT);
        u4ones_cnt[0] = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DQS0_ERR_CNT), DQS0_ERR_CNT_DQS0_ERR_CNT);
        u4ones_cnt[1] = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DQS1_ERR_CNT), DQS1_ERR_CNT_DQS1_ERR_CNT);
        u4ones_cnt[2] = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DQS2_ERR_CNT), DQS2_ERR_CNT_DQS2_ERR_CNT);
        u4ones_cnt[3] = u4IO32ReadFldAlign(DRAMC_REG_ADDR(DRAMC_REG_DQS3_ERR_CNT), DQS3_ERR_CNT_DQS3_ERR_CNT);
        #ifdef ETT_PRINT_FORMAT
        mcSHOW_DBG_MSG(("%d : %d, %d, %d, %d, %d\n", ucdqs_dly, u4sample_cnt, u4ones_cnt[0],u4ones_cnt[1],u4ones_cnt[2],u4ones_cnt[3]));         
        #else
        mcSHOW_DBG_MSG(("%3d : %8d, %8d, %8d, %8d, %8d\n", ucdqs_dly, u4sample_cnt, u4ones_cnt[0],u4ones_cnt[2],u4ones_cnt[3]));         
        #endif
        
        /*
        //Disable DQ eye scan (RG_RX_EYE_SCAN_EN=0, RG_RX_*RDQ_VREF_EN_B*=0, RG_RX_*RDQ_EYE_VREF_EN_B*=0, RG_RX_*RDQ_SMT_EN_B*=0)
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DRAMC_REG_STBCAL_F), 0, STBCAL_F_RG_EX_EYE_SCAN_EN);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE2), 0, EYE2_RG_RX_ARDQ_VREF_EN_B0);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), 0, EYEB1_2_RG_RX_ARDQ_VREF_EN_B1);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYE2), 0, EYE2_RG_RX_ARDQ_EYE_VREF_EN_B0);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), 0, EYEB1_2_RG_RX_ARDQ_EYE_VREF_EN_B1);
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_TXDQ3), 0, TXDQ3_RG_RX_ARDQ_SMT_EN_B0); 
        vIO32WriteFldAlign(DRAMC_REG_ADDR(DDRPHY_RXDQ13),0, RXDQ13_RG_RX_ARDQ_SMT_EN_B1);    
        */

        //change to boolean value
        if (u4ones_cnt[0] < (u4sample_cnt/2))
        {
            fgcurrent_value = 0;
        }
        else
        {
            fgcurrent_value = 1;
        }
        
        #if 0//more than 1T data
        {
        if (ucsearch_state==0)
        {
            //record initial value at the beginning
            fginitial_value = fgcurrent_value;            
            ucsearch_state = 1;
        }
        else if (ucsearch_state==1)
        {
            // check if change value
            if (fgcurrent_value != fginitial_value)
            {
                // start of the period
                fginitial_value = fgcurrent_value;
                ucstart_period = ucdqs_dly;
                ucsearch_state = 2;
            }
        }
        else if (ucsearch_state==2)
        {
            // check if change value
            if (fgcurrent_value != fginitial_value)
            {
                fginitial_value = fgcurrent_value;
                ucsearch_state = 3;
            }
        }
        else if (ucsearch_state==3)
        {
            // check if change value
            if (fgcurrent_value != fginitial_value)
            {
                // end of the period, break the loop
                ucend_period = ucdqs_dly;
                ucsearch_state = 4;
                break;
            }
        }
        else
        {
            //nothing
        }
    }
        #else //only 0.5T data
        {
            if (ucsearch_state==0)
            {
                //record initial value at the beginning
                fginitial_value = fgcurrent_value;            
                ucsearch_state = 1;
            }
            else if (ucsearch_state==1)
            {
                // check if change value
                if (fgcurrent_value != fginitial_value)
                {
                    // start of the period
                    fginitial_value = fgcurrent_value;
                    ucstart_period = ucdqs_dly;
                    ucsearch_state = 2;
                }
            }
            else if (ucsearch_state==2)
            {
                // check if change value
                if (fgcurrent_value != fginitial_value)
                {
                    // end of the period, break the loop
                    ucend_period = ucdqs_dly;
                    ucsearch_state = 4;
                   break;
                }
            }
        }
        #endif
    }

    //restore to orignal value
    vIO32Write4B(DRAMC_REG_ADDR(DRAMC_REG_STBCAL_F), u4prv_register_348);

    #if ENABLE_LP4_SW
    if(p->dram_type == TYPE_LPDDR4)
    {
        vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_EYE1), u4prv_register_018);
        vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_EYEB1), u4prv_register_078);
        vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_EYE2), u4prv_register_01c);
        vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_EYEB1_2), u4prv_register_07c);
        vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_EYE3), u4prv_register_020);
        vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_EYEB1_3), u4prv_register_080);
        vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_TXDQ3), u4prv_register_014);
        vIO32Write4B(DRAMC_REG_ADDR(DDRPHY_RXDQ13), u4prv_register_074);
    }
    else //LPDDR3
    #endif
    {  
        for(u1ByteIdx=0; u1ByteIdx<DQS_NUMBER; u1ByteIdx++)
        {
            vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_TXDQ3), u4RegBak[u1ByteIdx][0], PHY_FLD_FULL);
            vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_EYE1), u4RegBak[u1ByteIdx][1], PHY_FLD_FULL);
            vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_EYE2), u4RegBak[u1ByteIdx][2], PHY_FLD_FULL);
            vIO32WriteFldAlign_Phy_Byte(u1ByteIdx, DRAMC_REG_ADDR(DDRPHY_EYE3), u4RegBak[u1ByteIdx][3], PHY_FLD_FULL);
        }
        
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_PHY_PDB_PU_0), 0, PHY_PDB_PU_0_RG_RX_ARDQ_JM_SEL_B0);
        vIO32WriteFldAlign_Phy_All(DRAMC_REG_ADDR(DDRPHY_PHY_PDB_PU_1), 0, PHY_PDB_PU_1_RG_RX_ARDQ_JM_SEL_B1);
    }

    if(ucsearch_state!=4)
    {
        mcSHOW_DBG_MSG(("===============================================================================\n"));
        mcSHOW_DBG_MSG(("    MIOCK jitter meter - channel=%d\n", p->channel));
        mcSHOW_DBG_MSG(("    Less than 0.5T data. Cannot calculate delay cell time\n"));
        mcSHOW_DBG_MSG(("===============================================================================\n"));
        return DRAM_FAIL;
    }

    //Calculate 1 delay cell = ? ps
    // 1T = ? delay cell
    ucg_num_dlycell_perT = (ucend_period - ucstart_period)*2;
    // 1T = ? ps
    u4MPDIV_IN_SEL = u4IO32ReadFldAlign((DDRPHY_PLL12), PLL12_RG_ARPI_MPDIV_IN_SEL);
    if(u4MPDIV_IN_SEL==0)//PHYPLL
    {
        //FREQ_LCPLL = = FREQ_XTAL*(RG_*_RPHYPLL_FBKDIV+1)*2^(RG_*_RPHYPLL_FBKSEL)/2^(RG_*_RPHYPLL_PREDIV)
        U32 u4FBKDIV = u4IO32ReadFldAlign((DDRPHY_PLL1), PLL1_RG_RPHYPLL_FBKDIV);
        U8 u1FBKSEL = u4IO32ReadFldAlign((DDRPHY_PLL1), PLL1_RG_RPHYPLL_FBKSEL);
        U8 u1PREDIV = u4IO32ReadFldAlign((DDRPHY_PLL1), PLL1_RG_RPHYPLL_PREDIV);
        if(u1PREDIV==3)
            u1PREDIV = 2;
        u2real_freq = (U16) (((u4FBKDIV+1)*REF_CLK)<<u1FBKSEL)>>(u1PREDIV);
    }
    else
    {
        //Freq(VCO clock) = FREQ_XTAL*(RG_RCLRPLL_SDM_PCW)/2^(RG_*_RCLRPLL_PREDIV)/2^(RG_*_RCLRPLL_POSDIV)
        //Freq(DRAM clock)= Freq(VCO clock)/4
        U32 u4FBKDIV = u4IO32ReadFldAlign((DDRPHY_PLL3), PLL3_RG_RCLRPLL_FBKDIV);
        U8 u1PREDIV = u4IO32ReadFldAlign((DDRPHY_PLL3), PLL3_RG_RCLRPLL_PREDIV);
        U8 u1POSDIV = u4IO32ReadFldAlign((DDRPHY_PLL3), PLL3_RG_RCLRPLL_POSDIV);
        u2real_freq = (U16) ((((u4FBKDIV)*REF_CLK)>>u1PREDIV)>>(u1POSDIV))>>2;
    }
    
    
    u2real_period = (U16) (1000000/u2real_freq);
    //calculate delay cell time
    u2delay_cell_ps = u2real_period / ucg_num_dlycell_perT;

    
    mcSHOW_DBG_MSG(("===============================================================================\n"));
    mcSHOW_DBG_MSG(("    MIOCK jitter meter - channel=%d\n", p->channel));
    mcSHOW_DBG_MSG(("===============================================================================\n"));
    mcSHOW_DBG_MSG(("1T = (%d-%d)*2 = %d delay cells\n", ucend_period, ucstart_period, ucg_num_dlycell_perT));
    mcSHOW_DBG_MSG(("Clock frequency = %d MHz, Clock period = %d ps, 1 delay cell = %d ps\n", u2real_freq, u2real_period, u2delay_cell_ps));

    return DRAM_OK;

// log example
/* dly: sample_cnt   DQS0_cnt  DQS1_cnt
    0 : 10962054,        0,        0
    1 : 10958229,        0,        0
    2 : 10961109,        0,        0
    3 : 10946916,        0,        0
    4 : 10955421,        0,        0
    5 : 10967274,        0,        0
    6 : 10893582,        0,        0
    7 : 10974762,        0,        0
    8 : 10990278,        0,        0
    9 : 10972026,        0,        0
   10 :  7421004,        0,        0
   11 : 10943883,        0,        0
   12 : 10984275,        0,        0
   13 : 10955268,        0,        0
   14 : 10960326,        0,        0
   15 : 10952451,        0,        0
   16 : 10956906,        0,        0
   17 : 10960803,        0,        0
   18 : 10944108,        0,        0
   19 : 10959939,        0,        0
   20 : 10959246,        0,        0
   21 : 11002212,        0,        0
   22 : 10919700,        0,        0
   23 : 10977489,        0,        0
   24 : 11009853,        0,        0
   25 : 10991133,        0,        0
   26 : 10990431,        0,        0
   27 : 10970703,    11161,        0
   28 : 10970775,   257118,        0
   29 : 10934442,  9450467,        0
   30 : 10970622, 10968475,        0
   31 : 10968831, 10968831,        0
   32 : 10956123, 10956123,        0
   33 : 10950273, 10950273,        0
   34 : 10975770, 10975770,        0
   35 : 10983024, 10983024,        0
   36 : 10981701, 10981701,        0
   37 : 10936782, 10936782,        0
   38 : 10889523, 10889523,        0
   39 : 10985913, 10985913,    55562
   40 : 10970235, 10970235,   272294
   41 : 10996056, 10996056,  9322868
   42 : 10972350, 10972350, 10969738
   43 : 10963917, 10963917, 10963917
   44 : 10967895, 10967895, 10967895
   45 : 10961739, 10961739, 10961739
   46 : 10937097, 10937097, 10937097
   47 : 10937952, 10937952, 10937952
   48 : 10926018, 10926018, 10926018
   49 : 10943793, 10943793, 10943793
   50 : 10954638, 10954638, 10954638
   51 : 10968048, 10968048, 10968048
   52 : 10944036, 10944036, 10944036
   53 : 11012112, 11012112, 11012112
   54 : 10969137, 10969137, 10969137
   55 : 10968516, 10968516, 10968516
   56 : 10952532, 10952532, 10952532
   57 : 10985832, 10985832, 10985832
   58 : 11002527, 11002527, 11002527
   59 : 10950660, 10873571, 10950660
   60 : 10949022, 10781797, 10949022
   61 : 10974366, 10700617, 10974366
   62 : 10972422,  1331974, 10972422
   63 : 10926567,        0, 10926567
   64 : 10961658,        0, 10961658
   65 : 10978893,        0, 10978893
   66 : 10962828,        0, 10962828
   67 : 10957599,        0, 10957599
   68 : 10969227,        0, 10969227
   69 : 10960722,        0, 10960722
   70 : 10970937,        0, 10963180
   71 : 10962054,        0, 10711639
   72 : 10954719,        0, 10612707
   73 : 10958778,        0,   479589
   74 : 10973898,        0,        0
   75 : 11004156,        0,        0
   76 : 10944261,        0,        0
   77 : 10955340,        0,        0
   78 : 10998153,        0,        0
   79 : 10998774,        0,        0
   80 : 10953234,        0,        0
   81 : 10960020,        0,        0
   82 : 10923831,        0,        0
   83 : 10951362,        0,        0
   84 : 10965249,        0,        0
   85 : 10949103,        0,        0
   86 : 10948707,        0,        0
   87 : 10941147,        0,        0
   88 : 10966572,        0,        0
   89 : 10971333,        0,        0
   90 : 10943721,        0,        0
   91 : 10949337,        0,        0
   92 : 10965942,        0,        0
   93 : 10970397,        0,        0
   94 : 10956429,        0,        0
   95 : 10939896,        0,        0
   96 : 10967112,        0,        0
   97 : 10951911,        0,        0
   98 : 10953702,        0,        0
   99 : 10971090,        0,        0
  100 : 10939590,        0,        0
  101 : 10993392,        0,        0
  102 : 10975932,        0,        0
  103 : 10949499,    40748,        0
  104 : 10962522,   258638,        0
  105 : 10951524,   275292,        0
  106 : 10982475,   417642,        0
  107 : 10966887, 10564347,        0
  ===============================================================================
      MIOCK jitter meter - channel=0
  ===============================================================================
  1T = (107-29) = 78 delay cells
  Clock frequency = 936 MHz, Clock period = 1068 ps, 1 delay cell = 13 ps
*/
}
#endif //#ifdef ENABLE_MIOCK_JMETER
