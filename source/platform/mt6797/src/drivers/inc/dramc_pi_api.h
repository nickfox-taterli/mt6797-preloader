#ifndef _PI_API_H
#define _PI_API_H

/***********************************************************************/
/*              Includes                                               */
/***********************************************************************/
#include "dramc_common.h"
/***********************************************************************/
/*              Constant Define                                        */
/***********************************************************************/

#define __FLASH_TOOL_DA__   0

#if __FLASH_TOOL_DA__
  #include "sw_types.h"
#else
  #include "typedefs.h"
#endif

//Bring Up Selection : Do Not open it when normal operation
//#define FIRST_BRING_UP

//DRAMC Chip 
#define fcA60817 	0
#define fcA60501    1
#define fcEverest       2
#define fcFOR_CHIP_ID fcEverest

//Feature option
#ifndef MTK_SINGLE_RANK_TEST
#define DUAL_RANKS
#endif
#define SINGLE_CHANNEL_ENABLE 0
#define ENABLE_CA_TRAINING  1
#define ENABLE_WRITE_LEVELING 1

//SW option
#define EVEREST_PORTING_TODO 0
#define EVEREST_MEM_C_TEST_CODE 0
#define SW_CHANGE_FOR_SIMULATION 0  //calibration funciton for simulation. Code changed due to different compiler 
#define ENABLE_LP4_SW 0  // for LP3 only project, can set to 0 to save code size.
#define ENABLE_MRR_AFTER_FIRST_K 1  // first calibration for MR5 vender ID and MR8 dram density .
#ifdef DUAL_RANKS
#define ENABLE_RANK_NUMBER_AUTO_DETECTION 1
#else
#define ENABLE_RANK_NUMBER_AUTO_DETECTION 0
#endif

// Sw work around options.
#define WRITE_LEVELING_MOVE_DQS_INSTEAD_OF_CLK 1 // work around for clock multi phase problem(cannot move clk or the clk will be bad)
#define CA_TRAIN_RESULT_DO_NOT_MOVE_CLK 1   // work around for clock multi phase problem(cannot move clk or the clk will be bad)
#define TX_PERBIT_INIT_FLOW_CONTROL 1 // work around for clock multi phase problem(cannot move clk or the clk will be bad)
#define JADE_TRACKING_MODE 1

//init for test engine
#define DEFAULT_TEST2_1_CAL 0x55000000   // pattern0 and base address for test engine when we do calibration
#define DEFAULT_TEST2_2_CAL 0xaa000400   // pattern1 and offset address for test engine when we  do calibraion
#define CMP_CPT_POLLING_PERIOD 1       // timeout for TE2: (CMP_CPT_POLLING_PERIOD X MAX_CMP_CPT_WAIT_LOOP) 
#define MAX_CMP_CPT_WAIT_LOOP 10000   // max loop

// common
#define DQS_NUMBER   4
#define DQS_BIT_NUMBER  8
#define DQ_DATA_WIDTH   32   // define max support bus width in the system (to allocate array size)
#define TIME_OUT_CNT    100 //100us

// Gating window
#define DQS_GW_COARSE_START_LP4 12//28//12   
#define DQS_GW_COARSE_END_LP4   28//56//28     //Coarse 8 * Small oarse 8 = 64, and then reserve 2UI for Phase inverse, 64-2=62
#define DQS_GW_COARSE_START_LP3 4//0   
#define DQS_GW_COARSE_END_LP3   20//12
#define DQS_GW_COARSE_STEP	1//8 // for testing// 1
//#define DQS_GW_COARSE_MAX	((DQS_GW_COARSE_END-DQS_GW_COARSE_START)/DQS_GW_COARSE_STEP+1)
#define DQS_GW_FINE_START 0
#define DQS_GW_FINE_END 32
#define DQS_GW_FINE_STEP 2  // if step >1, need to do DramPhyCGReset
#define GATING_ADJUST_TXDLY_FOR_TRACKING  1

// DATLAT
#define DATLAT_TAP_NUMBER 32   // DATLAT[3:0] = {0x80[20:4]}
#define DATLAT_DSEL_DIFFERENCE  6  // Everest: Datlat_dsel = datlat -6

// RX DQ/DQS
#define MAX_RX_DQSDLY_TAPS 127   	// 0x018, May set back to 64 if no need.
#define MAX_RX_DQDLY_TAPS 63      // 0x210~0x22c, 0~15 delay tap
#define RX_DQS_DELAY_BEGIN_LP4 55   // first step to DQ delay
#define RX_DQS_DELAY_BEGIN_LP3 65  // first step to DQ delay
#define RX_DQ_DELAY_END_LP4 31   // first step to DQ delay
#define RX_DQ_DELAY_END_LP3 31   // first step to DQ delay

// TX DQ/DQS
#define TX_DQ_DELAY_BEGIN_LP4 40   // first step to DQ delay
#define TX_DQ_DELAY_BEGIN_LP3 0   // first step to DQ delay
#define TX_DQ_DELAY_END_LP4 208   // first step to DQ delay
#define TX_DQ_DELAY_END_LP3 31   // first step to DQ delay
#define MAX_TX_DQDLY_TAPS 31   // max DQ TAP number
#define MAX_TX_DQSDLY_TAPS 31   // max DQS TAP number

// DVFS
// need to review #if & #ifdef, Jeremy
#define DUAL_FREQ_K	   1		// If enable, need to define DDR_xxxx the same as DUAL_FREQ_HIGH
#define DFS_COMBINATION_TYPE3   0   //1700P
#define DFS_COMBINATION_TYPE1   0   //1866P
#define DFS_HW_SYNC_GATING_TRACKING 0
#define DFS_SUPPORT_THIRD_SHUFFLE 1
#if DFS_SUPPORT_THIRD_SHUFFLE
#define DFS_GTDMW_SYNC 0
#define DFS_THIRD_SHUFFLE_FREQ LC_DDR800
#else
#define DFS_GTDMW_SYNC 1
#endif

#define DUAL_FREQ_HIGH	800
#define DUAL_FREQ_LOW	400		

// Run time config
#define HW_GATING				// DQS strobe calibration enable
#define ZQCS_ENABLE
#define TEMP_SENSOR_ENABLE	// After enable rumtime MR4 read, Get DramC SPCMDRESP_REFRESH_RATE.
#define APPLY_LOWPOWER_GOLDEN_SETTINGS 1  // low pwer settings
#define SPM_CONTROL_AFTERK		 // Disable in bring-up and enable thereafter.   Don't enable in ETT

//Debug option
#define GATING_ONLY_FOR_DEBUG __ETT__
#define CPU_RW_TEST_AFTER_K 0 // need to enable GATING_ONLY_FOR_DEBUG at the same time for gating debug log
#define ENABLE_LCPLL_IC_SCAN 0
#define ETT_PRINT_FORMAT  // Apply for both preloader and ETT
#define LJPLL_FREQ_DEBUG_LOG 1
#define ENABLE_CLAIBTAION_WINDOW_LOG_FOR_FT 0  //FT log for finding out IC with small window size
#define ENABLE_DDRPHY_FREQ_METER 0
#define SUPPORT_LP3_800 1  // cross rank parameters are over spec.  cannot support

#define ENABLE_ALL_NEW_FUNC
//... add new feature compile option here.
#ifdef ENABLE_ALL_NEW_FUNC
#define LOW_SPEED_WRITE_LEVELING // not implement, just example
#define RX_DLY_TRACKING_VERIFY 0
#else
#define RX_DLY_TRACKING_VERIFY 0
#endif

//misc feature 
//#define RX_EYE_SCAN
//#define TX_EYE_SCAN
//#define ENABLE_MIOCK_JMETER  // debug purpose, don't enable normally
#define LP3_JV_WORKAROUND 1 //tISCK 1866 spec. out, 1600 spec. marginal.  Therefore, setting fine tune.

#define CHECK_PLL_OK 1 // check IC PHY PLL is good or not , if not , use FREERUN26M clk and Fix PLL band.
#if __ETT__
#define EVEREST_RX_POWER_ECO 0  // ETT cannot read 0x08000008, Illegal READ access memory 0x8000008 !
#else
#define EVEREST_RX_POWER_ECO 1
#endif

//======================== FIRST_BRING_UP Init Definition =====================
#ifdef FIRST_BRING_UP

#undef DUAL_FREQ_K
#define DUAL_FREQ_K	   0		// If enable, need to define DDR_xxxx the same as DUAL_FREQ_HIGH
#undef SINGLE_CHANNEL_ENABLE
#define SINGLE_CHANNEL_ENABLE	1// Boot-up debugging.
#undef DUAL_RANKS

#undef ENABLE_CA_TRAINING
#define ENABLE_CA_TRAINING  0
#undef ENABLE_WRITE_LEVELING
#define ENABLE_WRITE_LEVELING 0

#undef HW_GATING
#undef SPM_CONTROL_AFTERK		
#undef TEMP_SENSOR_ENABLE		

#undef ENABLE_LCPLL_IC_SCAN
#define ENABLE_LCPLL_IC_SCAN 1

#if 0 // no use now
#undef DUAL_FREQ_HIGH
#undef DUAL_FREQ_LOW
#define DUAL_FREQ_HIGH	533
#define DUAL_FREQ_LOW	533		
#define DDRTYPE_LPDDR3        
#undef DUAL_FREQ_DIFF_ACTIMING 
#undef DRAMC_DRAMPHY_DCM_ENABLE		
#undef ZQCS_ENABLE				
#undef EMI_OPTIMIZATION			

#undef DUAL_FREQ_TEST
#undef ONEPLL_TEST
#undef DUAL_FREQ_TEST_DIFFV
#undef DUAL_FREQ_DIFF_RLWL
#endif
#endif //FIRST_BRING_UP
//=============================================================================

//======================== ETT Definition =====================================
#if __ETT__
//#define DDR_INIT_TIME_PROFILING
#ifdef DUAL_RANKS
#define DUAL_RANK_RX_K
#define DUAL_RANK_TX_K
#endif
#define ENABLE_MPLL_SSC 1 // only for Everest ETT
#else // not ETT
#define ENABLE_MPLL_SSC 0 // not for android. never enable.
#endif

//=============================================================================

/***********************************************************************/
/*              Defines                                                */
/***********************************************************************/
#define ENABLE  1
#define DISABLE 0

typedef enum
{
    DRAM_OK = 0, // OK
    DRAM_FAIL    // FAIL
} DRAM_STATUS_T; // DRAM status type


typedef enum
{
    LJ_DDR1866 = 0,
    LJ_DDR1700,
    LJ_DDR1600,
    LJ_DDR933,  //mpdiv2
    LJ_DDR850,  //mpdiv2
    LJ_DDR800,  //mpdiv2
    LJ_MAX_SEL,
    LC_DDR1600,
    LC_DDR1270,
    LC_DDR1066,
    LC_DDR800,  //mpdiv2
    LC_DDR635,  //mpdiv2
    LC_DDR533,  //mpdiv2
    PLL_FREQ_SEL_MAX
} DRAM_PLL_FREQ_SEL_T; // DRAM DFS type

typedef enum
{
    DRAM_DFS_SHUFFLE_NORMAL = 0,
    DRAM_DFS_SHUFFLE_1,
    DRAM_DFS_SHUFFLE_2,
    DRAM_DFS_SHUFFLE_SW,
    DRAM_DFS_SHUFFLE_MAX
} DRAM_DFS_SHUFFLE_TYPE_T; // DRAM SHUFFLE RG type

typedef enum
{
    DRAM_CALIBRATION_ZQ = 0,
    DRAM_CALIBRATION_SW_IMPEDANCE ,
    DRAM_CALIBRATION_CA_TRAIN ,  
    DRAM_CALIBRATION_WRITE_LEVEL,
    DRAM_CALIBRATION_GATING,
    DRAM_CALIBRATION_DATLAT,
    DRAM_CALIBRATION_RX_RDDQC,      
    DRAM_CALIBRATION_RX_PERBIT,
    DRAM_CALIBRATION_TX_PERBIT,
    DRAM_CALIBRATION_MAX 
} DRAM_CALIBRATION_STATUS_T; 


typedef enum
{
    DDRPHY_CONF_A = 0,    
    DDRPHY_CONF_B,
    DDRPHY_CONF_C,
    DDRPHY_CONF_D,
    DDRPHY_CONF_MAX
} DDRPHY_CONF_T;


typedef enum
{
    CHANNEL_A = 0,    
    CHANNEL_B,
    CHANNEL_MAX
} DRAM_CHANNEL_T;

typedef enum
{
    RANK_0= 0,    
    RANK_1,
    RANK_MAX
} DRAM_RANK_T;

typedef enum
{
    RANK_SINGLE = 1,    
    RANK_DUAL
} DRAM_RANK_NUMBER_T;

typedef enum
{
    TYPE_DDR1 = 1,
    TYPE_LPDDR2,
    TYPE_LPDDR3,
    TYPE_PCDDR3,  
    TYPE_LPDDR4
} DRAM_DRAM_TYPE_T;

typedef enum
{
    DATA_WIDTH_16BIT = 16,
    DATA_WIDTH_32BIT = 32
} DRAM_DATA_WIDTH_T;

// for A60501 DDR3
typedef enum
{
    PCB_LOC_ASIDE = 0,
    PCB_LOC_BSIDE
} DRAM_PCB_LOC_T;

typedef enum
{
    MODE_1X = 0,
    MODE_2X
} DRAM_DRAM_MODE_T;

typedef enum
{
    PACKAGE_SBS = 0,
    PACKAGE_POP
} DRAM_PACKAGE_T;

typedef enum
{
    TE_OP_WRITE_READ_CHECK = 0,
    TE_OP_READ_CHECK
} DRAM_TE_OP_T;

typedef enum
{
    TEST_ISI_PATTERN = 0,
    TEST_AUDIO_PATTERN,
    TEST_TA1_SIMPLE,
    TEST_TESTPAT4,
    TEST_TESTPAT4_3, 
    TEST_XTALK_PATTERN,
    TEST_MIX_PATTERN,
    TEST_DMA,
} DRAM_TEST_PATTERN_T;

typedef enum
{
    BL_TYPE_4 = 0,
    BL_TYPE_8
} DRAM_BL_TYPE_T;

typedef enum
{
    DLINE_0 = 0,
    DLINE_1,
    DLINE_TOGGLE    
} PLL_PHASE_CAL_STATUS_T;

typedef enum
{
    TA43_OP_STOP,
    TA43_OP_CLEAR,
    TA43_OP_RUN,
    TA43_OP_RUNQUIET,
    TA43_OP_UNKNOWN,
} DRAM_TA43_OP_TYPE_T;

// used for record last test pattern in TA
typedef enum
{
    TA_PATTERN_IDLE,
    TA_PATTERN_TA43,
    TA_PATTERN_TA4,
    TA_PATTERN_UNKNOWM,
} DRAM_TA_PATTERN_T;

typedef enum
{
    DMA_OP_PURE_READ,
    DMA_OP_PURE_WRITE,
    DMA_OP_READ_WRITE,
} DRAM_DMA_OP_T;

typedef enum
{
    DMA_PREPARE_DATA_ONLY,
    DMA_CHECK_DATA_ACCESS_ONLY_AND_NO_WAIT,
    DMA_CHECK_COMAPRE_RESULT_ONLY,
    DMA_CHECK_DATA_ACCESS_AND_COMPARE,
} DRAM_DMA_CHECK_RESULT_T;


typedef enum
{
    fcDATLAT_USE_DEFAULT = 0,
    fcDATLAT_USE_RX_SCAN,
    //fcDATLAT_USE_TXRX_SCAN    
}DRAM_DATLAT_CALIBRATION_TYTE_T;

////////////////////////////
typedef struct _DRAMC_CTX_T
{
    DRAM_CHANNEL_T channel;
    DRAM_RANK_NUMBER_T support_rank_num;
    DRAM_RANK_T rank;
    DRAM_PLL_FREQ_SEL_T freq_sel;
    DRAM_DFS_SHUFFLE_TYPE_T shu_type;
    DRAM_DRAM_TYPE_T dram_type;
    DRAM_PACKAGE_T package;
    DRAM_DATA_WIDTH_T data_width;
    U32 test2_1;
    U32 test2_2;
    DRAM_TEST_PATTERN_T test_pattern;
    U16 frequency;
    U8 fglow_freq_write_en;
    U8 ssc_en;
    U8 en_4bitMux;
    U8 enable_rx_scan_vref;
    U8 enable_tx_scan_vref; 
    U32 aru4CalResultFlag[CHANNEL_MAX][RANK_MAX][DRAM_DFS_SHUFFLE_MAX];// record the calibration is fail or success,  0:success, 1: fail
    U32 aru4CalExecuteFlag[CHANNEL_MAX][RANK_MAX][DRAM_DFS_SHUFFLE_MAX]; // record the calibration is execute or not,  0:no operate, 1: done

#if fcFOR_CHIP_ID == fcA60501
    DRAM_PCB_LOC_T pcb_loc;
    U32 ta43_result[4];   // for eye scan to check falling or rising error
#endif    
    #if WRITE_LEVELING_MOVE_DQS_INSTEAD_OF_CLK
    BOOL arfgWriteLevelingInitShif[CHANNEL_MAX][RANK_MAX];
    #endif
    #if TX_PERBIT_INIT_FLOW_CONTROL
    BOOL fgTXPerbifInit[CHANNEL_MAX][RANK_MAX];
    #endif
} DRAMC_CTX_T;

typedef struct _PASS_WIN_DATA_T
{
    S16 first_pass;
    S16 last_pass;
    S16 win_center;
    U16 best_dqdly;
} PASS_WIN_DATA_T;

#if 0
typedef struct _RXDQS_PERBIT_DLY_T
{
    S8 first_dqdly_pass;
    S8 last_dqdly_pass;
    S8 first_dqsdly_pass;
    S8 last_dqsdly_pass;
    U8 best_dqdly;
    U8 best_dqsdly;
} RXDQS_PERBIT_DLY_T;
#endif
typedef struct _TXDQS_PERBIT_DLY_T
{
    S8 first_dqdly_pass;
    S8 last_dqdly_pass;
    S8 first_dqsdly_pass;
    S8 last_dqsdly_pass;
    U8 best_dqdly;
    U8 best_dqsdly;
} TXDQS_PERBIT_DLY_T;

#if 1//ENABLE_MRR_AFTER_FIRST_K
typedef struct _DRAM_INFO_BY_MRR_T
{
    U8 u1MR5VendorID;
    U32 u4MR8Density[CHANNEL_MAX][RANK_MAX];
} DRAM_INFO_BY_MRR_T;
#endif

//For new register access
#define DRAMC_REG_ADDR(offset)    ((p->channel << POS_BANK_NUM)+ (offset))
#define SYS_REG_ADDR(offset)    (offset)

// Different from Pi_calibration.c due to Base address
//#define mcSET_DRAMC_REG_ADDR(offset)    (DRAMC_BASE_ADDRESS | (p->channel << POS_BANK_NUM) | (offset))
#define mcSET_SYS_REG_ADDR(offset)    (DRAMC_BASE_ADDRESS | (offset))
#define mcSET_DRAMC_NAO_REG_ADDR(offset)   (DRAMC_NAO_BASE_ADDRESS | (offset))
#define mcSET_DRAMC_AO_REG_ADDR(offset)   (DRAMC_AO_BASE_ADDRESS | (offset))
//#define mcSET_DRAMC_AO_REG_ADDR_CHC(offset)   ((DRAMC_AO_BASE_ADDRESS + ((U32)CHANNEL_C << POS_BANK_NUM)) | (offset))
#define mcSET_DDRPHY_REG_ADDR(offset) (DDRPHY_BASE_ADDR | (offset))
#define mcSET_DDRPHY_REG_ADDR_CHA(offset) ((DDRPHY_BASE_ADDR + ((U32) CHANNEL_A << POS_BANK_NUM)) | (offset))
#define mcSET_DDRPHY_REG_ADDR_CHB(offset) ((DDRPHY_BASE_ADDR + ((U32) CHANNEL_B << POS_BANK_NUM)) | (offset))
//#define mcSET_DDRPHY_REG_ADDR_CHC(offset) ((DDRPHY_BASE_ADDR + ((U32) CHANNEL_C << POS_BANK_NUM)) | (offset))
//#define mcSET_DDRPHY_REG_ADDR_CHD(offset) ((DDRPHY_BASE_ADDR + ((U32) CHANNEL_D << POS_BANK_NUM)) | (offset))

/***********************************************************************/
/*              External declarations                                  */
/***********************************************************************/
extern U8 RXPERBIT_LOG_PRINT;

/***********************************************************************/
/*              Public Functions                                       */
/***********************************************************************/
// basic function
DRAMC_CTX_T *DramcCtxCreate(void);
void vDramcCtxDestroy(DRAMC_CTX_T *p);
void vDramcCtxInit(DRAMC_CTX_T *p);
DRAM_STATUS_T MemPllInit(DRAMC_CTX_T *p);
DRAM_STATUS_T SscEnable(DRAMC_CTX_T *p);
DRAM_STATUS_T DramcInit(DRAMC_CTX_T *p);
void DramcSetRankEngine2(DRAMC_CTX_T *p, U8 u1RankSel);
U32 DramcEngine1(DRAMC_CTX_T *p, U32 test2_1, U32 test2_2, S16 loopforever, U8 period);
U32 DramcEngine2(DRAMC_CTX_T *p, DRAM_TE_OP_T wr, U32 test2_1, U32 test2_2, U8 testaudpat, S16 loopforever, U8 period, U8 log2loopcount);
U32 TestEngineCompare(DRAMC_CTX_T *p);

void DramcEnterSelfRefresh(DRAMC_CTX_T *p, U8 op);
void DramcLowFreqWrite(DRAMC_CTX_T *p);
void DramcRunTimeConfig(DRAMC_CTX_T *p);
DRAM_STATUS_T DramcRegDump(DRAMC_CTX_T *p);
void DramcTestPat4_3(DRAMC_CTX_T *p, DRAM_TA43_OP_TYPE_T type);
U32 DramcPat4_3(DRAMC_CTX_T *p);
//U32 DramcDmaEngineNoWait(DRAMC_CTX_T *p, DRAM_DMA_OP_T op, U32 src_addr, U32 dst_addr, U32 trans_len, U8 burst_len);
//U32 DramcDmaWaitCompare(DRAMC_CTX_T *p, U32 src_addr, U32 dst_addr, U32 trans_len, U8 burst_len);

void DramcModeRegReadByRank(DRAMC_CTX_T *p, U8 u1Rank, U8 u1MRIdx, U8 *u1pValue);
void DramcModeRegRead(DRAMC_CTX_T *p, U8 u1MRIdx, U8 *u1pValue);
void DramcModeRegWrite(DRAMC_CTX_T *p, U8 u1MRIdx, U8 u1Value);

void DramPhyReset(DRAMC_CTX_T *p);
U8 u1GetMR4RefreshRate(DRAM_CHANNEL_T channel);
void vGetImpedanceResult(DRAMC_CTX_T *p, U8 *drvp, U8* drvn);

// mandatory calibration function
DRAM_STATUS_T DramcDQSOSCAuto(DRAMC_CTX_T *p);
DRAM_STATUS_T DramcWriteFifoTest(DRAMC_CTX_T *p);
DRAM_STATUS_T DramcReadFifoTest(DRAMC_CTX_T *p);
DRAM_STATUS_T DramcRunTimeModifyTXDQTest(DRAMC_CTX_T *p);
DRAM_STATUS_T DramcStartDQSOSC(DRAMC_CTX_T *p);
DRAM_STATUS_T DramcStopDQSOSC(DRAMC_CTX_T *p);
DRAM_STATUS_T DramcZQCalibration(DRAMC_CTX_T *p);
DRAM_STATUS_T DramcRXInputBufferOffsetCal(DRAMC_CTX_T *p);
DRAM_STATUS_T DramcSwImpedanceCal(DRAMC_CTX_T *p, U8 u1Para);
DRAM_STATUS_T DramcHwImpedanceCal(DRAMC_CTX_T *p);
DRAM_STATUS_T DramcWriteLeveling(DRAMC_CTX_T *p);
#if GATING_ONLY_FOR_DEBUG
void DramcGatingDebugInit(DRAMC_CTX_T *p);
void DramcGatingDebugRankSel(DRAMC_CTX_T *p, U8 u1Rank);
void DramcGatingDebug(DRAMC_CTX_T *p, U8 u1Channel);
void DramcGatingDebugExit(DRAMC_CTX_T *p);
#endif

DRAM_STATUS_T DramcRxdqsGatingCal_Manual(DRAMC_CTX_T *p, S16 iDelay);
DRAM_STATUS_T DramcRxdqsGatingPreProcess(DRAMC_CTX_T *p);
DRAM_STATUS_T DramcRxdqsGatingPostProcess(DRAMC_CTX_T *p);
DRAM_STATUS_T DramcRxWindowPerbitCal(DRAMC_CTX_T *p, U8 u1UseTestEngine);
DRAM_STATUS_T DramcRxWindowPerbitCal_Manual(DRAMC_CTX_T *p, U8 u1UseTestEngine, S16 iDelay);
DRAM_STATUS_T DramcRxdatlatCal(DRAMC_CTX_T *p);
DRAM_STATUS_T DramcTxWindowPerbitCal(DRAMC_CTX_T *p);
DRAM_STATUS_T DramcTxWindowPerbitCal_Manual(DRAMC_CTX_T *p, S16 uiDelay);
DRAM_STATUS_T DramcCATraining(DRAMC_CTX_T *p);
void Dram_Reset(DRAMC_CTX_T *p);

void DramcDFSSetDMAConfig(U32 src, U32 dst, U32 len, U32 config);
void PMIC_VCORE_ADJUST(U16 targetV);
#if DUAL_FREQ_K
void DramcDFS(DRAMC_CTX_T *p);
void DFSPLLSettingForLP800(DRAMC_CTX_T *p);
void DFSSwitchFreq(DRAMC_CTX_T *p);
void DramcSaveFreqSetting(DRAMC_CTX_T *p);
void DramcDumpFreqSetting(DRAMC_CTX_T *p);
void DramcRestorePLLSetting(DRAMC_CTX_T *p);
void DramcRestoreFreqSetting(DRAMC_CTX_T *p);
void DramcExchangeFreqSetting(DRAMC_CTX_T *p);
DRAM_STATUS_T DramcRestoreGatingTrackingToRG(DRAMC_CTX_T *p);
#endif

void DramcHWGatingInit(DRAMC_CTX_T *p);
void DramcHWGatingOnOff(DRAMC_CTX_T *p, U8 u1OnOff);
void DramcPrintHWGatingStatus(DRAMC_CTX_T *p, U8 u1Channel);

#if 0  //no use now, disable for saving code size.
void vGetCalibrationResult(DRAMC_CTX_T *p, U8 ucCalType, U8 *ucCalExecute, U8 *ucCalResult); // get result of specific calibration
#endif
// get result of all calibration of specific channel/rank/freq.
void vGetCalibrationResult_All(DRAMC_CTX_T *p, U8 u1Channel, U8 u1Rank, U8 u1FreqType, U32 *u4CalExecute, U32 *u4CalResult);
void vPrintCalibrationResult(DRAMC_CTX_T *p);


// reference function
void DramcRxEyeScanInit(DRAMC_CTX_T *p);
DRAM_STATUS_T DramcRxEyeScan(DRAMC_CTX_T *p, U8 boot_or_run, U8 dq_no);
DRAM_STATUS_T DramcTxEyeScan(DRAMC_CTX_T *p);
DRAM_STATUS_T DramcRankSwap(DRAMC_CTX_T *p, U8 u1Rank);

// DDR reserved mode function
void Dramc_DDR_Reserved_Mode_setting(void);

// dump all reg for debug
extern void RISCReadAll(void);
#if ENABLE_DDRPHY_FREQ_METER
extern void DDRPhyFreqMeter(void);
#else
#define DDRPhyFreqMeter(_x_)
#endif
extern void DDRPhyResetPLL(void);
extern void DDRAllLowPowerSettings(DRAMC_CTX_T *p);
extern void GetPhyPllFrequency(DRAMC_CTX_T *p);

// Global variables
#ifdef _WIN32
extern FILE *fp_A60501;
#endif

#ifdef DUAL_RANKS
extern unsigned int uiDualRank;
#endif

#define PHY_MAPPING_TO_CHANNEL 1  //only for Everst LP3, always enable. Don't disable this.
#if PHY_MAPPING_TO_CHANNEL
extern U8 aru1PhyMap2Channel[2];
extern void vSetPHY2ChannelMapping(DRAMC_CTX_T *p, U8 u1Channel);
#endif
extern U32 u4MR2Value;

#ifdef DDR_INIT_TIME_PROFILING
void TimeProfileBegin(void);
UINT32 TimeProfileEnd(void);
#endif

#if CHECK_PLL_OK
int is_pll_good(void);
#endif

// API prototypes.
//void DramcDmaEngine_Config(DRAMC_CTX_T *p);
//void DramcGeneralPat_mem(DRAMC_CTX_T *p, U32 src_addr);

extern U8 u1DMAtest;
#endif // _PI_API_H
