//=============================================================================
//  Include Files                                                                                                                                                                                
//=============================================================================
//#include <common.h>
//#include <ett_common.h>
//#include <test_case_controller.h>
//#include <api.h>
//#include "emi_hw.h"
//#include "gpio.h"
//#include "ett_cust.h"
//#include "emi_setting.h"
//#include "pll.h"
//#include "dramc_pi_api.h"
#include "dramc_common.h"
#if !__FLASH_TOOL_DA__ && !__ETT__ 
   #include "dram_buffer.h"
   #include "platform.h"
   #include "upmu_hw.h"
#endif

#if !__FLASH_TOOL_DA__
#include "custom_emi.h"   // fix build error: emi_settings
#endif

#if CFG_BOOT_ARGUMENT
#define bootarg g_dram_buf->bootarg
#endif

extern DRAMC_CTX_T DramCtx_LPDDR3;
unsigned int enable_4GB_flag=0;
//=============================================================================
//  Definition                                                                                                                                                                                   
//=============================================================================

EMI_SETTINGS emi_setting_default_lpddr3 =
{
        //default	//1300/1800
                0x0,            /* sub_version */
                0x0003,         /* TYPE */
                0,              /* EMMC ID/FW ID checking length */
                0,              /* FW length */
                {0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0},              /* NAND_EMMC_ID */
                {0x00,0x0,0x0,0x0,0x0,0x0,0x0,0x0},             /* FW_ID */
                
                #if SINGLE_CHANNEL_ENABLE
                    #ifdef DUAL_RANKS
                            0xa053a056,            /* EMI_CONA_VAL */  // single channel, dual rank                       
                            0x00330000,            /* EMI_CONH_VAL */  // single channel, dual rank
                    #else
                            0xa050a056,            /* EMI_CONA_VAL */  // single channel, single rank
                            0x00030000,            /* EMI_CONH_VAL */  // single channel, single rank
                    #endif
                #else  // 2 channel
                    #ifdef DUAL_RANKS
                            0xa053a057,            /* EMI_CONA_VAL */  // dual channel, dual rank
                            0x33330000,            /* EMI_CONH_VAL */  // dual channel, dual rank
                    #else
                            0xa050a057,            /* EMI_CONA_VAL */  // dual channel, single rank
                            0x03030000,            /* EMI_CONH_VAL */  // dual channel, single rank
                    #endif
                #endif
                {
                // 1866 AC timing.
                0x070C03A5,
                0x1F571801,
                0x0D070301,
                0x060F0407,
                0x28646570,
                0x01055054,
                0x00000000,
                },
                {0x10000000,0,0,0},                 /* DRAM RANK SIZE */
                #if SINGLE_CHANNEL_ENABLE
                    #ifdef DUAL_RANKS
                            0x00421000,            /* EMI_CONF_VAL */                        
                    #else
                            0x00421000,            /* EMI_CONF_VAL */
                    #endif
                #else  // 2 channel
                    #ifdef DUAL_RANKS
                            0x04210000,            /* EMI_CONF_VAL */
                    #else
                            0x04210000,            /* EMI_CONF_VAL */
                    #endif
                #endif                
                {0,0,0,0,0,0,0,0,0},              /* reserved 9*4 bytes */
                0x00000006,             /* LPDDR3_MODE_REG5 */
                0x00000000,             /* pin-mux type for tablet */
};

//=============================================================================
//  Global Variables                                                                                                                                                                         
//=============================================================================
int emi_setting_index = -1;
static int enable_combo_dis = 0;
extern DRAMC_CTX_T *psCurrDramCtx;
EMI_SETTINGS *default_emi_setting;

//=============================================================================
//  External references                                                                                                                                                                   
//=============================================================================
extern DRAMC_CTX_T *psCurrDramCtx;
extern char* opt_dle_value;

void print_DBG_info(DRAMC_CTX_T *p);
void Dump_EMIRegisters(DRAMC_CTX_T *p);

#if fcFOR_CHIP_ID == fcEverest 
#define EMI_APB_BASE    0x10203000
#define EMI_CONA	    (EMI_APB_BASE + 0x000)
#define EMI_CONB	    (EMI_APB_BASE + 0x008)
#define EMI_CONC	    (EMI_APB_BASE + 0x010)
#define EMI_COND	    (EMI_APB_BASE + 0x018)
#define EMI_CONE	    (EMI_APB_BASE + 0x020)
#define EMI_CONF	    (EMI_APB_BASE + 0x028)
#define EMI_CONG	    (EMI_APB_BASE + 0x030)
#define EMI_CONH	    (EMI_APB_BASE + 0x038)
#define EMI_CONI	    (EMI_APB_BASE + 0x040)
#define EMI_CONJ	    (EMI_APB_BASE + 0x048)
#define EMI_CONK	    (EMI_APB_BASE + 0x050)
#define EMI_MDCT	    (EMI_APB_BASE + 0x078)
#define EMI_TEST0	    (EMI_APB_BASE + 0x0d0)
#define EMI_TEST1	    (EMI_APB_BASE + 0x0d8)
#define EMI_TESTB	    (EMI_APB_BASE + 0x0e8)
#define EMI_TESTC	    (EMI_APB_BASE + 0x0f0)
#define EMI_TESTD	    (EMI_APB_BASE + 0x0f8)
#define EMI_ARBI    	    (EMI_APB_BASE + 0x140)
#define EMI_ARBI_2ND   (EMI_APB_BASE + 0x144)
#define EMI_ARBJ	           (EMI_APB_BASE + 0x148)
#define EMI_ARBJ_2ND   (EMI_APB_BASE + 0x14c)
#define EMI_ARBK	    (EMI_APB_BASE + 0x150)
#define EMI_ARBK_2ND  (EMI_APB_BASE + 0x154)
#define EMI_SLCT	           (EMI_APB_BASE + 0x158)
#define EMI_ARBA	    (EMI_APB_BASE + 0x100)
#define EMI_ARBB	    (EMI_APB_BASE + 0x108)
#define EMI_ARBC	    (EMI_APB_BASE + 0x110)
#define EMI_ARBD	    (EMI_APB_BASE + 0x118)
#define EMI_ARBE	    (EMI_APB_BASE + 0x120)
#define EMI_ARBF	    (EMI_APB_BASE + 0x128)
#define EMI_ARBG	    (EMI_APB_BASE + 0x130)
#define EMI_ARBH	    (EMI_APB_BASE + 0x138)
#define EMI_CONM	    (EMI_APB_BASE + 0x060)


void EMI_Init(DRAMC_CTX_T *p)
{
    EMI_SETTINGS *emi_set;
    //int emi_cona;
    
    if(emi_setting_index == -1)
        emi_set = default_emi_setting;
    else
        emi_set = &emi_settings[emi_setting_index];
 	
#if fcFOR_CHIP_ID == fcEverest
  
  if(emi_set->EMI_CONF_VAL)
  *(volatile unsigned *)EMI_CONF = emi_set->EMI_CONF_VAL;
  else
	*(volatile unsigned *)EMI_CONF = 0x04210000;
  		
  *(volatile unsigned *)EMI_CONA = emi_set->EMI_CONA_VAL;		
  *(volatile unsigned *)EMI_CONH = emi_set->EMI_CONH_VAL;
  *(volatile unsigned *)EMI_CONB	 = 0x17283544;
  *(volatile unsigned *)EMI_CONC	 = 0x0a1a0b1a;
  *(volatile unsigned *)EMI_COND	 = 0x3657587a;
  *(volatile unsigned *)EMI_CONE	 = 0xffff0848;
  *(volatile unsigned *)EMI_CONG	 = 0x2b2b2a38;
  *(volatile unsigned *)EMI_CONI	 = 0x00008813;
  *(volatile unsigned *)EMI_CONJ	 = 0xffff1fed;
  *(volatile unsigned *)EMI_CONK	 = 0x00000000;
  *(volatile unsigned *)EMI_MDCT	 = 0x889a0c3f;
  *(volatile unsigned *)EMI_TEST0	 = 0xcfcfcfcf;
  *(volatile unsigned *)EMI_TEST1	 = 0xcfcfcfcf;
  *(volatile unsigned *)EMI_TESTB	 = 0x00062027;
  *(volatile unsigned *)EMI_TESTC	 = 0x38460000;
  *(volatile unsigned *)EMI_TESTD	 = 0x00000000;
  *(volatile unsigned *)EMI_ARBI	 = 0x20407188;
  *(volatile unsigned *)EMI_ARBI_2ND	 = 0x20406188;
  *(volatile unsigned *)EMI_ARBJ	 = 0x9719595e;
  *(volatile unsigned *)EMI_ARBJ_2ND	 = 0x9719595e;
  *(volatile unsigned *)EMI_ARBK	 = 0x64f3fc79;
  *(volatile unsigned *)EMI_ARBK_2ND	 = 0x64f3fc79;
  *(volatile unsigned *)EMI_SLCT	 = 0xff01ff00;
  *(volatile unsigned *)EMI_ARBA	 = 0x7f807048;
  *(volatile unsigned *)EMI_ARBB	 = 0x7f807f40;
  *(volatile unsigned *)EMI_ARBC	 = 0xa0a070d7;
  *(volatile unsigned *)EMI_ARBD	 = 0x000070c5;
  *(volatile unsigned *)EMI_ARBE	 = 0x40306045;
  *(volatile unsigned *)EMI_ARBF	 = 0xa0a070d8;
  *(volatile unsigned *)EMI_ARBG	 = 0xa0a07042;
  *(volatile unsigned *)EMI_ARBH	 = 0x2010704d;
  *(volatile unsigned *)EMI_CONM	 = 0x0000063c;
  #endif
}
#endif

#if 0
void CHA_HWGW_Print(DRAMC_CTX_T *p)
{
    U8 u1RefreshRate;
    
    DramcPrintHWGatingStatus(p, CHANNEL_A);

#if 0//def TEMP_SENSOR_ENABLE
    u1RefreshRate = u1GetMR4RefreshRate(CHANNEL_A);
    mcSHOW_ERR_MSG(("[CHA] MRR(MR4) Reg.3B8h[10:8]=%x\n", u1RefreshRate));
    u1RefreshRate = u1GetMR4RefreshRate(CHANNEL_B);
    mcSHOW_ERR_MSG(("[CHB] MRR(MR4) Reg.3B8h[10:8]=%x\n", u1RefreshRate));
#endif
}
#endif

void Dump_EMIRegisters(DRAMC_CTX_T *p)
{
  U8 ucstatus = 0;
  U32 uiAddr;
  U32 u4value;

  for (uiAddr=0; uiAddr<0x160; uiAddr+=4)
  {
    mcSHOW_DBG_MSG(("EMI offset:%x, value:%x\n", uiAddr, *(volatile unsigned *)(EMI_APB_BASE+uiAddr)));
  }
}

void print_DBG_info(DRAMC_CTX_T *p)
{
    unsigned int addr = 0x0;
    U32 u4value;

#ifdef DDR_INIT_TIME_PROFILING    
    return;
#endif

    mcSHOW_DBG_MSG(("EMI_CONA=%x\n",*(volatile unsigned *)(EMI_APB_BASE+0x00000000)));
    mcSHOW_DBG_MSG(("EMI_CONH=%x\n",*(volatile unsigned *)(EMI_APB_BASE+0x00000038)));

    //RISCReadAll();
    mcSHOW_DBG_MSG(("=============================================\n"));
}

int mt_get_dram_type(void)
{
  return TYPE_LPDDR3;
}

int mt_get_freq_setting(DRAMC_CTX_T *p)
{
    return DUAL_FREQ_HIGH;
}

#ifdef DDR_RESERVE_MODE
extern U32 g_ddr_reserve_enable;
extern U32 g_ddr_reserve_success;
#define TIMEOUT 3
#endif

#ifdef DDR_RESERVE_MODE
extern u32 g_ddr_reserve_enable;
extern u32 g_ddr_reserve_success;
#define TIMEOUT 3
extern void before_Dramc_DDR_Reserved_Mode_setting(void);

#endif

void release_dram(void)
{   
#ifdef DDR_RESERVE_MODE  
    int i;
    int counter = TIMEOUT; 
        
    rgu_release_rg_dramc_conf_iso();
    Dramc_DDR_Reserved_Mode_setting();	
    rgu_release_rg_dramc_iso();    
    rgu_release_rg_dramc_sref();

    // setup for EMI
    DRV_WriteReg32(EMI_MPUP, 0x200);
    for (i=0;i<10;i++);
    
    while(counter)
    {
      if(rgu_is_dram_slf() == 0) /* expect to exit dram-self-refresh */
        break;
      counter--;
    }
  
    if(counter == 0)
    {
      if(g_ddr_reserve_enable==1 && g_ddr_reserve_success==1)
      {
        print("[DDR Reserve] release dram from self-refresh FAIL!\n");
        g_ddr_reserve_success = 0;
      }
    }
    else
    {
         print("[DDR Reserve] release dram from self-refresh PASS!\n");
    }

#if DUAL_FREQ_K
	if(g_ddr_reserve_enable==1 && g_ddr_reserve_success==0) //DDR reserved mode fail case
		Dramc_DDR_Reserved_Mode_ResetShuffle();	
	else //normal case	
		Dramc_DDR_Reserved_Mode_JumpToHigh();
#endif
    
#if 0    
	{
		DRAMC_CTX_T * p = psCurrDramCtx;
		DramcRegDump(p);
	}
#endif
#endif    
}

void check_ddr_reserve_status(void)
{
    //unsigned int wdt_mode;
    //unsigned int wdt_dbg_ctrl;
    //wdt_mode = READ_REG(MTK_WDT_MODE);
    //wdt_dbg_ctrl = READ_REG(MTK_WDT_DEBUG_CTL);

    //print("before test, wdt_mode = 0x%x, wdt_dbg_ctrl = 0x%x\n", wdt_mode, wdt_dbg_ctrl);     
    //if(((wdt_mode & MTK_WDT_MODE_DDR_RESERVE) !=0) && ((wdt_dbg_ctrl & MTK_DDR_RESERVE_RTA) != 0) )
    //{
    //    print("go sample reserved flow\n");
    //    print("[DDR Reserve_sp:DRAMC] 0x%x:0x%x\n",(0x10004080),*(volatile unsigned int *)(0x10004080));
    //    ddr_reserve_sample_flow();
    //    print("[DDR Reserve_sp:DRAMC] 0x%x:0x%x\n",(0x10004080),*(volatile unsigned int *)(0x10004080));
    //}

#ifdef DDR_RESERVE_MODE  
    int counter = TIMEOUT;
    if(rgu_is_reserve_ddr_enabled())
    {
      g_ddr_reserve_enable = 1;
      if(rgu_is_reserve_ddr_mode_success())
      {
        while(counter)
        {
          if(rgu_is_dram_slf())
          {
            g_ddr_reserve_success = 1;
            break;
          }
          counter--;
        }
        if(counter == 0)
        {
          print("[DDR Reserve] ddr reserve mode success but DRAM not in self-refresh!\n");
          g_ddr_reserve_success = 0;
        }
      }
      else
      {
        print("[DDR Reserve] ddr reserve mode FAIL!\n");
        g_ddr_reserve_success = 0;
      }
	  /* release dram, no matter success or failed */
      release_dram();      
    }
    else
    {
      print("[DDR Reserve] ddr reserve mode not be enabled yet\n");
      g_ddr_reserve_enable = 0;
    }   
#endif    
}

unsigned int DRAM_MRR(int MRR_num)
{
    unsigned char MRR_value = 0x0;
    DRAMC_CTX_T *p = psCurrDramCtx; 

    DramcModeRegRead(p, MRR_num, &MRR_value);
    return MRR_value;
}

static int mt_get_dram_type_for_dis(void)
{
    int i;
    int type = 2;
    type = (emi_settings[0].type & 0xF);
    for (i = 0 ; i < num_of_emi_records; i++)
    {
      //print("[EMI][%d] type%d\n",i,type);
      if (type != (emi_settings[i].type & 0xF))
      {
          print("It's not allow to combine two type dram when combo discrete dram enable\n");
          ASSERT(0);
          break;
      }
    }
    return type;
}

#if !__FLASH_TOOL_DA__ && !__ETT__ 
/*
 * reserve a memory from mblock
 * @mblock_info: address of mblock_info
 * @reserved_size: size of memory
 * @align: alignment
 * @limit: address limit. Must higher than return address + reserved_size
 * @rank: preferable rank, the returned address is in rank or lower ranks
 * It returns as high rank and high address as possible. (consider rank first)
 */
u64 mblock_reserve(mblock_info_t *mblock_info, u64 reserved_size, u64 align, u64 limit,
                   enum reserve_rank rank)
{
	int i, max_rank, target = -1;
	u64 start, end, sz, max_addr = 0;
	u64 reserved_addr = 0;
	mblock_t mblock;

	if (mblock_info->mblock_num == 128) {
		/* the mblock[] is full */
		printf("mblock_reserve error: mblock[] is full\n");
		return 0;
	}

	if (!align)
		align = 0x1000;
	/* must be at least 4k aligned */
	if (align & (0x1000 - 1))
		align &= ~(0x1000 - 1);

	if (rank == RANK0) {
		/* reserve memory from rank 0 */
		max_rank = 0;
	} else {
		/* reserve memory from any possible rank */
		/* mblock_num >= nr_ranks is true */
		max_rank = mblock_info->mblock_num - 1;
	}

	for (i = 0; i < mblock_info->mblock_num; i++) {
		start = mblock_info->mblock[i].start;
		sz = mblock_info->mblock[i].size;
		end = limit < (start + sz)? limit: (start + sz);
		reserved_addr = (end - reserved_size);
		reserved_addr &= ~(align - 1);
		printf("mblock[%d].start: 0x%llx, sz: 0x%llx, limit: 0x%llx, "
		        "max_addr: 0x%llx, max_rank: %d, target: %d, "
		        "mblock[].rank: %d, reserved_addr: 0x%llx,"
		        "reserved_size: 0x%llx\n",
		        i, start, sz, limit, max_addr, max_rank,
		        target, mblock_info->mblock[i].rank,
		        reserved_addr, reserved_size);
		printf("mblock_reserve dbg[%d]: %d, %d, %d, %d, %d\n",
		        i, (reserved_addr + reserved_size < start + sz),
		        (reserved_addr >= start),
		        (mblock_info->mblock[i].rank <= max_rank),
		        (start + sz > max_addr),
		        (reserved_addr + reserved_size <= limit));
		if ((reserved_addr + reserved_size <= start + sz) &&
		        (reserved_addr >= start) &&
		        (mblock_info->mblock[i].rank <= max_rank) &&
		        (start + sz > max_addr) &&
		        (reserved_addr + reserved_size <= limit)) {
			max_addr = start + sz;
			target = i;
		}
	}

	if (target < 0) {
		printf("mblock_reserve error\n");
		return 0;
	}

	start = mblock_info->mblock[target].start;
	sz = mblock_info->mblock[target].size;
	end = limit < (start + sz)? limit: (start + sz);
	reserved_addr = (end - reserved_size);
	reserved_addr &= ~(align - 1);

	/* split mblock if necessary */
	if (reserved_addr == start) {
		/*
		 * only needs to fixup target mblock
		 * [reserved_addr, reserved_size](reserved) +
		 * [reserved_addr + reserved_size, sz - reserved_size]
		 */
		mblock_info->mblock[target].start = reserved_addr + reserved_size;
		mblock_info->mblock[target].size -= reserved_size;
	} else if ((reserved_addr + reserved_size) == (start + sz)) {
		/*
		 * only needs to fixup target mblock
		 * [start, reserved_addr - start] +
		 * [reserved_addr, reserved_size](reserved)
		 */
		mblock_info->mblock[target].size = reserved_addr - start;
	} else {
		/*
		 * fixup target mblock and create a new mblock
		 * [start, reserved_addr - start] +
		 * [reserved_addr, reserved_size](reserved) +
		 * [reserved_addr + reserved_size, start + sz - reserved_addr - reserved_size]
		 */
		/* fixup original mblock */
		mblock_info->mblock[target].size = reserved_addr - start;

		/* new mblock */
		mblock.rank =  mblock_info->mblock[target].rank;
		mblock.start = reserved_addr + reserved_size;
		mblock.size = start + sz - (reserved_addr + reserved_size);

		/* insert the new node, keep the list sorted */
		memmove(&mblock_info->mblock[target + 2],
		        &mblock_info->mblock[target + 1],
		        sizeof(mblock_t) *
		        (mblock_info->mblock_num - target - 1));
		mblock_info->mblock[target + 1] = mblock;
		mblock_info->mblock_num += 1;
		printf( "mblock[%d]: %llx, %llx from mblock\n"
		        "mblock[%d]: %llx, %llx from mblock\n",
		        target,
		        mblock_info->mblock[target].start,
		        mblock_info->mblock[target].size,
		        target + 1,
		        mblock_info->mblock[target + 1].start,
		        mblock_info->mblock[target + 1].size);
	}

	printf("mblock_reserve: %llx - %llx from mblock %d\n",
	        reserved_addr, reserved_addr + reserved_size,
	        target);

	/* print debug info */
	for (i = 0; i < mblock_info->mblock_num; i++) {
		start = mblock_info->mblock[i].start;
		sz = mblock_info->mblock[i].size;
		printf("mblock-debug[%d].start: 0x%llx, sz: 0x%llx\n",
		        i, start, sz);
	}

	return reserved_addr;
}

/*
 * mblock_resize - resize mblock started at addr from oldsize to newsize,
 * current implementation only consider oldsize >= newsize.
 *
 * @mblock_info: mblock information
 * @orig_dram_info: original dram information
 * @addr: start address of a mblock
 * @oldsize: origianl size of the mblock
 * @newsize: new size of the given block
 * return 0 on success, otherwise 1
 */
#if 0
int mblock_resize(mblock_info_t *mblock_info, dram_info_t *orig_dram_info,
                  u64 addr, u64 oldsize, u64 newsize)
{
	int err = 1;
	unsigned int i;
	u64 start, sz;
	mblock_t mblock;

	/* check size, oldsize must larger than newsize */
	if (oldsize <= newsize) {
		printf("mblock_resize error: mblock %llx oldsize(%llx) <= newsize(%llx)",
		        addr, oldsize, newsize);
		goto error;
	}

	/* check alignment, at least 4k aligned */
	if ((oldsize & (0x1000 - 1)) || (newsize & (0x1000 - 1))) {
		printf("mblock_resize alignment error: oldsize(%llx) or newsize(%llx)\n",
		        oldsize, newsize);
		goto error;
	}

	/* check mblock */
	for (i = 0; i < mblock_info->mblock_num; i++) {
		start = mblock_info->mblock[i].start;
		sz = mblock_info->mblock[i].size;
		/* invalid mblock */
		if ((addr >= start) && ((addr + oldsize) <= (start + sz))) {
			printf("mblock_resize error: mblock %llx, size: %llx is free\n",
			        addr, oldsize);
			goto error;
		}
	}

	/*
	 * ok, the mblock is valid and oldsize > newsize, let's
	 * shrink this mblock
	 */
	/* setup a new mblock */
	mblock.start = addr + newsize;
	mblock.size = oldsize - newsize;
	printf("mblock_resize putback mblock %llx size: %llx\n",
	        mblock.start, mblock.size);
	/* setup rank */
	for (i = 0; i < orig_dram_info->rank_num; i++) {
		start = orig_dram_info->rank_info[i].start;
		sz = orig_dram_info->rank_info[i].size;
		if ((mblock.start >= start) && ((mblock.start + mblock.size) <= (start + sz))) {
			mblock.rank = i;
			break;
		}
	}
	if (i >= orig_dram_info->rank_num) {
		printf("mblock_resize error: mblock not in orig_dram_info: %llx, size(%llx)\n",
		        mblock.start, mblock.size);
		goto error;
	}

	/* put the mblock back to mblock_info */
	for (i = 0; i < mblock_info->mblock_num; i++) {
		start = mblock_info->mblock[i].start;
		sz = mblock_info->mblock[i].size;
		if (mblock.rank == mblock_info->mblock[i].rank) {
			if (mblock.start == start + sz) {
				/*
				 * the new mblock can be merged to this mblock
				 * [start, start + sz] +
				 * [mblock.start, mblock.start + mblock.size](new)
				 */
				mblock_info->mblock[i].size += mblock.size;
				/* destroy block */
				mblock.size = 0;
			} else if (start == mblock.start + mblock.size) {
				/*
				 * the new mblock can be merged to this mblock
				 * [mblock.start, mblock.start + * mblock.size](new) +
				 * [start, start + sz]
				 */
				mblock_info->mblock[i].start = mblock.start;
				mblock_info->mblock[i].size += mblock.size;
				/* destroy block */
				mblock.size = 0;
			}
		}
	}

	/*
	 * mblock cannot be merge info mblock_info, insert it into mblock_info
	 */
	if (mblock.size) {
		for (i = 0; i < mblock_info->mblock_num; i++) {
			if (mblock.start < mblock_info->mblock[i].start)
				break;
		}
		memmove(&mblock_info->mblock[i + 1],
		        &mblock_info->mblock[i],
		        sizeof(mblock_t) *
		        (mblock_info->mblock_num - i));
		mblock_info->mblock[i] = mblock;
		mblock_info->mblock_num += 1;
	}

	/* print debug info */
	for (i = 0; i < mblock_info->mblock_num; i++) {
		start = mblock_info->mblock[i].start;
		sz = mblock_info->mblock[i].size;
		printf("mblock-resize-debug[%d].start: 0x%llx, sz: 0x%llx\n",
		        i, start, sz);
	}

	return 0;
error:
	return err;
}
#endif

/*
 * mblock_create - create mblock started at addr or merge with existing mblock
 *
 * @mblock_info: mblock information
 * @orig_dram_info: original dram information
 * @addr: start address of a mblock, must be 4k align
 * @size: size of the given block, must be 4K align
 * return 0 on success, otherwise 1
 */
#if 0
int mblock_create(mblock_info_t *mblock_info, dram_info_t *orig_dram_info
                  , u64 addr, u64 size)
{
	int err = 1;
	unsigned i,valid;
	u64 start, sz;
	mblock_t mblock;
	mblock_t *mblock_candidate_left = NULL, *mblock_candidate_right = NULL;

	/* check size, addr valid and align with 4K*/
	if (!size || size&(0x1000 - 1) || addr&(0x1000 - 1) ) {
		printf("mblock_create size invalid size=%llx\n",size);
		goto error;
	}
	/* for lca check*/
	if (bootarg.lca_reserved_mem.start && bootarg.lca_reserved_mem.size) {
		if (addr >= bootarg.lca_reserved_mem.start || addr + size > bootarg.lca_reserved_mem.start) {
			printf("mblock_create ERROR , overlap with LCA addr and size invalid addr = %llx size=%llx\n", addr, size);
			goto error;
		}
	}

	/* for tee check*/
	if (bootarg.tee_reserved_mem.start && bootarg.tee_reserved_mem.size) {
		if (addr >= bootarg.tee_reserved_mem.start || addr + size > bootarg.tee_reserved_mem.start) {
			printf("mblock_create ERROR , overlap with TEE addr and size invalid addr = %llx size=%llx\n", addr, size);
			goto error;
		}
	}

	/*it's not allow to create mblock which is cross rank
	 * and mblock should not exceed rank size */
	for (i = 0, valid = 0; i < orig_dram_info->rank_num; i++) {
		start = orig_dram_info->rank_info[i].start;
		sz = orig_dram_info->rank_info[i].size;
		if (addr >= start && addr < start + sz && addr + size <= start + sz) {
			valid =1;
			break;
		}
	}
	if (!valid) {
		printf("mblock_create addr and size invalid addr=%llx size=%llx\n",
		        addr,size);
		goto error;
	}

	/* check every mblock the addr and size should not be within any existing mblock */
	for (i = 0; i < mblock_info->mblock_num; i++) {
		start = mblock_info->mblock[i].start;
		sz = mblock_info->mblock[i].size;
		/*addr should start from reserved memory space and addr + size should not overlap with mblock
		 * when addr is smaller than start*/
		if (((addr >= start) && (addr < start + sz)) || (addr < start && addr + size > start)) {
			printf("mblock_create error: addr %llx overlap with mblock %llx, size: %llx \n",
			        addr, start, sz);
			goto error;
		}
	}

	/*
	 * ok, the mblock is valid let's create the mblock
	 * and try to merge it with the same bank and choose the bigger size one
	 */
	/* setup a new mblock */
	mblock.start = addr;
	mblock.size = size;
	printf("mblock_create mblock start %llx size: %llx\n",
	        mblock.start, mblock.size);
	/* setup rank */
	for (i = 0; i < orig_dram_info->rank_num; i++) {
		start = orig_dram_info->rank_info[i].start;
		sz = orig_dram_info->rank_info[i].size;
		if ((mblock.start >= start) && ((mblock.start + mblock.size) <= (start + sz))) {
			mblock.rank = i;
			break;
		}
	}
	if (i >= orig_dram_info->rank_num) {
		printf("mblock_create error: mblock not in orig_dram_info: %llx, size(%llx)\n",
		        mblock.start, mblock.size);
		goto error;
	}

	/* put the mblock back to mblock_info */
	for (i = 0; i < mblock_info->mblock_num; i++) {
		start = mblock_info->mblock[i].start;
		sz = mblock_info->mblock[i].size;
		if (mblock.rank == mblock_info->mblock[i].rank) {
			if (mblock.start + mblock.size == start) {
				/*
				 * the new mblock could be merged to this mblock
				 */
				mblock_candidate_right = &mblock_info->mblock[i];
			} else if (start + sz == mblock.start) {
				/*
				 * the new mblock can be merged to this mblock
				 */
				mblock_candidate_left =  &mblock_info->mblock[i];
			}
		}
	}
	/*we can merge either left or right , choose the bigger one */
	if (mblock_candidate_right && mblock_candidate_left) {
		if (mblock_candidate_right->size >= mblock_candidate_left->size) {
			printf("mblock_candidate_right->size = %llx \
				mblock_candidate_left->size = %llx \n",mblock_candidate_right->size, mblock_candidate_left->size);
			mblock_candidate_right->start = mblock.start;
			mblock_candidate_right->size += mblock.size;
		} else { /*left bigger*/
			printf("mblock_candidate_right->size = %llx \
				mblock_candidate_left->size = %llx \n",mblock_candidate_right->size, mblock_candidate_left->size);
			mblock_candidate_left->size += mblock.size;
		}
		/* destroy block */
		mblock.size = 0;
	} else {
		if (mblock_candidate_right) {
			mblock_candidate_right->start = mblock.start;
			mblock_candidate_right->size += mblock.size;
			/* destroy block */
			mblock.size = 0;
		}

		if (mblock_candidate_left) {
			mblock_candidate_left->size += mblock.size;
			/* destroy block */
			mblock.size = 0;
		}
	}

	/*
	 * mblock cannot be merge into mblock_info, insert it into mblock_info
	 */
	if (mblock.size) {
		for (i = 0; i < mblock_info->mblock_num; i++) {
			if (mblock.start < mblock_info->mblock[i].start)
				break;
		}
		/* insert the new node, keep the list sorted */
		if (i != mblock_info->mblock_num) {
			memmove(&mblock_info->mblock[i + 1],
			        &mblock_info->mblock[i],
			        sizeof(mblock_t) *
			        (mblock_info->mblock_num - i));
		}
		mblock_info->mblock[i] = mblock;
		mblock_info->mblock_num += 1;
		printf( "create mblock[%d]: %llx, %llx \n",
		        i,
		        mblock_info->mblock[i].start,
		        mblock_info->mblock[i].size);
	}

	/* print debug info */
	for (i = 0; i < mblock_info->mblock_num; i++) {
		start = mblock_info->mblock[i].start;
		sz = mblock_info->mblock[i].size;
		printf("mblock-create-debug[%d].start: 0x%llx, sz: 0x%llx\n",
		        i, start, sz);
	}

	return 0;
error:
	return err;
}
#endif

unsigned int Is_enable_4GB(void)
{
	print("[Enable 4GB Support] 4GB_flag 0x%x\n",enable_4GB_flag);
	return enable_4GB_flag;
}
/*
 * setup block correctly, we should hander both 4GB mode and
 * non-4GB mode.
 */
void setup_mblock_info(mblock_info_t *mblock_info, dram_info_t *orig_dram_info,
		mem_desc_t *lca_reserved_mem)
{
	int i, j;
	u64 max_dram_size = -1; /* MAX value */
	u64 size = 0;
	u64 total_dram_size = 0;

	for (i = 0; i < orig_dram_info->rank_num; i++) {
		total_dram_size += 
			orig_dram_info->rank_info[i].size;
	}
#ifdef CUSTOM_CONFIG_MAX_DRAM_SIZE
	max_dram_size = CUSTOM_CONFIG_MAX_DRAM_SIZE;
	printf("CUSTOM_CONFIG_MAX_DRAM_SIZE: 0x%llx\n", max_dram_size);
#endif 
	lca_reserved_mem->start = lca_reserved_mem->size = 0;
	memset(mblock_info, 0, sizeof(mblock_info_t));

	if(Is_enable_4GB() == 0) {		
	/* 
	 * non-4GB mode case 
	 */
	/* we do some DRAM size fixup here base on orig_dram_info */
	for (i = 0; i < orig_dram_info->rank_num; i++) {
		size += orig_dram_info->rank_info[i].size;
		mblock_info->mblock[i].start = 
			orig_dram_info->rank_info[i].start;
		mblock_info->mblock[i].rank = i;	/* setup rank */
		if (size <= max_dram_size) {
			mblock_info->mblock[i].size = 
				orig_dram_info->rank_info[i].size;
		} else {
			/* max dram size reached */
			size -= orig_dram_info->rank_info[i].size;
			mblock_info->mblock[i].size = 
				max_dram_size - size;
			/* get lca_reserved_mem info */
			lca_reserved_mem->start = mblock_info->mblock[i].start
				+ mblock_info->mblock[i].size;
			if (mblock_info->mblock[i].size) {
				mblock_info->mblock_num++;
			}
			break;
		}
			if (mblock_info->mblock[i].size) {
				mblock_info->mblock_num++;
			}
		}
	}
	else {	
	 	mblock_info->mblock[0].start = 0x40000000;
	 	mblock_info->mblock[0].size = 0x40000000;
	 	mblock_info->mblock[0].rank = 0;
	 	mblock_info->mblock[1].start = 0x80000000;
	 	mblock_info->mblock[1].size = 0x80000000;
	 	mblock_info->mblock[1].rank = 1;
	 	mblock_info->mblock[2].start = 0x100000000;
	 	mblock_info->mblock[2].size = 0x40000000;
	 	mblock_info->mblock[2].rank = 0;
	 	
		for (i = 0; i < 3; i++) {
			size += mblock_info->mblock[i].size;
			if (size > max_dram_size) {
				/* max dram size reached */
				size -= mblock_info->mblock[i].size;
				mblock_info->mblock[i].size = 
					max_dram_size - size;
				/* get lca_reserved_mem info */
				lca_reserved_mem->start = mblock_info->mblock[i].start
					+ mblock_info->mblock[i].size;
				if (mblock_info->mblock[i].size) {
					mblock_info->mblock_num++;
				}
				for(j = i; j < 3; j++) {
	 				mblock_info->mblock[j].start = 0;
	 				mblock_info->mblock[j].size = 0;
	 				mblock_info->mblock[j].rank = 0;										
				}
				break;
			}
    	
			if (mblock_info->mblock[i].size) {
				mblock_info->mblock_num++;
			}
		}
	}
	
	printf("total_dram_size: 0x%llx, max_dram_size: 0x%llx\n",
			total_dram_size, max_dram_size);
	if (total_dram_size > max_dram_size) {
		/* add left unused memory to lca_reserved memory */
		lca_reserved_mem->size = total_dram_size - max_dram_size;
		printf("lca_reserved_mem start: 0x%llx, size: 0x%llx\n",
				lca_reserved_mem->start,
				lca_reserved_mem->size);
	}

	/*
	 * TBD
	 * for 4GB mode, we fixup the start address of every mblock
	 */
}

/* 
 * setup block correctly, we should hander both 4GB mode and 
 * non-4GB mode.
 */
void get_orig_dram_rank_info(dram_info_t *orig_dram_info)
{
	int i, j;
	u64 base = DRAM_BASE;
	unsigned int rank_size[4];

	orig_dram_info->rank_num = get_dram_rank_nr();
	get_dram_rank_size(rank_size);

	orig_dram_info->rank_info[0].start = base;
	for (i = 0; i < orig_dram_info->rank_num; i++) {

		orig_dram_info->rank_info[i].size = (u64)rank_size[i];

		if (i > 0) {
			orig_dram_info->rank_info[i].start =
				orig_dram_info->rank_info[i - 1].start +
				orig_dram_info->rank_info[i - 1].size;
		}
		printf("orig_dram_info[%d] start: 0x%llx, size: 0x%llx\n",
				i, orig_dram_info->rank_info[i].start,
				orig_dram_info->rank_info[i].size);
	}
	
	for(j=i; j<4; j++)
	{
	  		orig_dram_info->rank_info[j].start = 0;
	  		orig_dram_info->rank_info[j].size = 0;	
	}
}


static int mt_get_mdl_number (void)
{
    static int found = 0;
    static int mdl_number = -1;
    int i;
    int j;
    int has_emmc_nand = 0;
    int discrete_dram_num = 0;
    int mcp_dram_num = 0;

    unsigned int dram_type;
    DRAM_INFO_BY_MRR_T DramInfo;

    if (!(found))
    {
        int result=0;
        //platform_get_mcp_id (id, emmc_nand_id_len,&fw_id_len);
        for (i = 0 ; i < num_of_emi_records; i++)
        {
            if ((emi_settings[i].type & 0x0F00) == 0x0000) 
            {
                discrete_dram_num ++; 
            }
            else
            {
                mcp_dram_num ++; 
            }
        }

        /*If the number >=2  &&
         * one of them is discrete DRAM
         * enable combo discrete dram parse flow
         * */
        if ((discrete_dram_num > 0) && (num_of_emi_records >= 2))
        {
            /* if we enable combo discrete dram
             * check all dram are all same type and not DDR3
             * */
            enable_combo_dis = 1;
            dram_type = emi_settings[0].type & 0x000F;
            for (i = 0 ; i < num_of_emi_records; i++)
            {
                if (dram_type != (emi_settings[i].type & 0x000F))
                {
                    print("[EMI] Combo discrete dram only support when combo lists are all same dram type.");
                    ASSERT(0);
                }
                if ((emi_settings[i].type & 0x000F) == TYPE_PCDDR3) 
                {
                    // has PCDDR3, disable combo discrete drame, no need to check others setting 
                    enable_combo_dis = 0; 
                    break;
                }
                dram_type = emi_settings[i].type & 0x000F;
            }
            
        } 
        print("[EMI] mcp_dram_num:%d,discrete_dram_num:%d,enable_combo_dis:%d\r\n",mcp_dram_num,discrete_dram_num,enable_combo_dis);
        /*
         *
         * 0. if there is only one discrete dram, use index=0 emi setting and boot it.
         * */
        if ((0 == mcp_dram_num) && (1 == discrete_dram_num))
        {
            mdl_number = 0;
            found = 1;
            
        #if ENABLE_MRR_AFTER_FIRST_K 
            //K first frequency (1066MHz)            
            Init_DRAM(&DramInfo);         
        #endif			  
		 
            return mdl_number;
        }          
#if 0
        /* 1.
         * if there is MCP dram in the list, we try to find emi setting by emmc ID
         * */
        if (mcp_dram_num > 0)
        {
            result = platform_get_mcp_id (id, emmc_nand_id_len,&fw_id_len);
            
            for (i = 0; i < num_of_emi_records; i++)
            {
                if (emi_settings[i].type != 0)
                {
                    if ((emi_settings[i].type & 0xF00) != 0x000)
                    {
                        if (result == 0)
                        {   /* valid ID */
            
                            if ((emi_settings[i].type & 0xF00) == 0x100)
                            {
                                /* NAND */
                                if (memcmp(id, emi_settings[i].ID, emi_settings[i].id_length) == 0){
                                    memset(id + emi_settings[i].id_length, 0, sizeof(id) - emi_settings[i].id_length);                                
                                    mdl_number = i;
                                    found = 1;
                                    break; /* found */
                                }
                            }
                            else
                            {
                                
                                /* eMMC */
                                if (memcmp(id, emi_settings[i].ID, emi_settings[i].id_length) == 0)
                                {
                                    print("fw id len:%d\n",emi_settings[i].fw_id_length);
                                    if (emi_settings[i].fw_id_length > 0)
                                    {
                                        char fw_id[6];
                                        memset(fw_id, 0, sizeof(fw_id));
                                        memcpy(fw_id,id+emmc_nand_id_len,fw_id_len);
                                        for (j = 0; j < fw_id_len;j ++){
                                            print("0x%x, 0x%x ",fw_id[j],emi_settings[i].fw_id[j]); 
                                        }
                                        if(memcmp(fw_id,emi_settings[i].fw_id,fw_id_len) == 0)
                                        {
                                            mdl_number = i;
                                            found = 1;
                                            break; /* found */
                                        }
                                        else
                                        {
                                            print("[EMI] fw id match failed\n");
                                        }
                                    }
                                    else
                                    {
                                        mdl_number = i;
                                        found = 1;
                                        break; /* found */
                                    }    
                                }
                                else{
                                      print("[EMI] index(%d) emmc id match failed\n",i);
                                }
                                
                            }
                        }
                    }
                }
            }
        }
#endif        
#if 1
        /* 2. find emi setting by MODE register 5
         * */
        // if we have found the index from by eMMC ID checking, we can boot android by the setting
        // if not, we try by vendor ID            
        if ((0 == found) && (1 == enable_combo_dis))
        {
            EMI_SETTINGS *emi_set;              
            int rank0_size=0, rank1_size=0;
            Init_DRAM(&DramInfo);
  	 
            //try to find discrete dram by DDR2_MODE_REG5(vendor ID)
            for (i = 0; i < num_of_emi_records; i++)
            {
                print("emi_settings[%d].MODE_REG_5:%x,emi_settings[%d].type:%x, vender_id=%x\n", i, emi_settings[i].iLPDDR3_MODE_REG_5, i, emi_settings[i].type, DramInfo.u1MR5VendorID);
                //only check discrete dram type
                if ((emi_settings[i].type & 0x0F00) == 0x0000) 
                {
                    //support for compol discrete dram 
                    if (emi_settings[i].iLPDDR3_MODE_REG_5 == DramInfo.u1MR5VendorID)
                    {
                        rank0_size = DramInfo.u4MR8Density[0][0] + DramInfo.u4MR8Density[1][0];
                        rank1_size = DramInfo.u4MR8Density[0][1] + DramInfo.u4MR8Density[1][1];

                        print("emi_settings[%d].DRAM_RANK_SIZE[0]:0x%x, DRAM_RANK_SIZE[1]:0x%x, rank0_size:0x%x, rankl_size:0x%x\n",i,emi_settings[i].DRAM_RANK_SIZE[0], emi_settings[i].DRAM_RANK_SIZE[1], rank0_size, rank1_size);                            
                        if((emi_settings[i].DRAM_RANK_SIZE[0] == rank0_size) && (emi_settings[i].DRAM_RANK_SIZE[1] == rank1_size))
                        {  
                            mdl_number = i;
                            found = 1;
                            break;
                        } 
                    }
                }
            }
        }
#endif
        print("found:%d,i:%d\n",found,i);   
        //while(1);
    }
    return mdl_number;
}

int get_dram_rank_nr (void)
{

    int index;
    int emi_cona;
    
#ifdef COMBO_MCP
  #ifdef DDR_RESERVE_MODE    
    if(g_ddr_reserve_enable==1 && g_ddr_reserve_success==1)
    {
      emi_cona = *(volatile unsigned int*)(EMI_CONA);
    }
    else
  #endif   	
    {    
      index = mt_get_mdl_number ();
      if (index < 0 || index >=  num_of_emi_records)
      {
          return -1;
      }

      emi_cona = emi_settings[index].EMI_CONA_VAL;
    } 
#else
    emi_cona = default_emi_setting->EMI_CONA_VAL;
#if CFG_FPGA_PLATFORM
    return 1;
#endif
#endif

    if ((emi_cona & (1 << 17)) != 0 || //for channel 0  
        (emi_cona & (1 << 16)) != 0 )  //for channel 1
        return 2;
    else
        return 1;
}

void get_dram_rank_size_by_EMI_CONA (unsigned int dram_rank_size[])
{
    unsigned col_bit, row_bit, ch0_rank0_size, ch0_rank1_size, ch1_rank0_size, ch1_rank1_size;
#ifndef COMBO_MCP   
    unsigned emi_cona = default_emi_setting->EMI_CONA_VAL;
    unsigned emi_conh = default_emi_setting->EMI_CONH_VAL;
#else
    unsigned emi_cona = *(volatile unsigned int*)(EMI_CONA);
    unsigned emi_conh = *(volatile unsigned int*)(EMI_CONH);    
#endif
 
    dram_rank_size[0] = 0;
    dram_rank_size[1] = 0;
    
    ch0_rank0_size = (emi_conh >> 16) & 0xf;
    ch0_rank1_size = (emi_conh >> 20) & 0xf;
    ch1_rank0_size = (emi_conh >> 24) & 0xf;
    ch1_rank1_size = (emi_conh >> 28) & 0xf;
    
    //Channel 0
    {   
        if(ch0_rank0_size == 0)
        {
            //rank 0 setting
            col_bit = ((emi_cona >> 4) & 0x03) + 9;
            row_bit = ((emi_cona >> 12) & 0x03) + 13;
            dram_rank_size[0] = (1 << (row_bit + col_bit)) * 4 * 8; // 4 byte * 8 banks
        }
        else
        {
            dram_rank_size[0] = (ch0_rank0_size * 256 << 20);
        }
 
        if (0 != (emi_cona &  (1 << 17)))   //rank 1 exist
        {
            if(ch0_rank1_size == 0)
            {
                col_bit = ((emi_cona >> 6) & 0x03) + 9;
                row_bit = ((emi_cona >> 14) & 0x03) + 13;
                dram_rank_size[1] = ((1 << (row_bit + col_bit)) * 4 * 8); // 4 byte * 8 banks
            }
            else
            {
                dram_rank_size[1] = (ch0_rank1_size * 256 << 20);
            }                
        }                                        
    }
    
    if(0 != (emi_cona & 0x01))     //channel 1 exist
    {
        if(ch1_rank0_size == 0)
        {                 
            //rank0 setting
            col_bit = ((emi_cona >> 20) & 0x03) + 9;
            row_bit = ((emi_cona >> 28) & 0x03) + 13;             
            dram_rank_size[0] += ((1 << (row_bit + col_bit)) * 4 * 8); // 4 byte * 8 banks
        }
        else
        {
            dram_rank_size[0] += (ch1_rank0_size * 256 << 20);            
        }
        
        if (0 != (emi_cona &  (1 << 16)))   //rank 1 exist
        {
            if(ch1_rank1_size == 0)
            {            
                col_bit = ((emi_cona >> 22) & 0x03) + 9;
                row_bit = ((emi_cona >> 30) & 0x03) + 13;
                dram_rank_size[1] += ((1 << (row_bit + col_bit)) * 4 * 8); // 4 byte * 8 banks
            }
            else
            {
                dram_rank_size[1] += (ch1_rank1_size * 256 << 20);
            } 
        }            
    }       

    printf("DRAM rank0 size:0x%x,\nDRAM rank1 size=0x%x\n", dram_rank_size[0], dram_rank_size[1]);    
}

void get_dram_rank_size (unsigned int dram_rank_size[])
{
#ifdef COMBO_MCP
    int index, rank_nr, i;

  #ifdef DDR_RESERVE_MODE
    if(g_ddr_reserve_enable==1 && g_ddr_reserve_success==1)
    {
        get_dram_rank_size_by_EMI_CONA(dram_rank_size);
    } 
    else
  #endif  	
    { 
        index = mt_get_mdl_number();
        
        if (index < 0 || index >= num_of_emi_records)
        {
            return;
        }
        
        rank_nr = get_dram_rank_nr();
        
        for(i = 0; i < rank_nr; i++){
            dram_rank_size[i] = emi_settings[index].DRAM_RANK_SIZE[i];

            printf("%d:dram_rank_size:%x\n",i,dram_rank_size[i]);
        }
    }
    return;
#else
    get_dram_rank_size_by_EMI_CONA(dram_rank_size);
    return;        
#endif
}
#endif //#if !__FLASH_TOOL_DA__ && !__ETT__ 


#if !__ETT__
void mt_set_emi(void)
{
    int index;
    unsigned int pmic_val_0x452;
    unsigned int pmic_val_0x450;

    index = mt_get_mdl_number ();
    print("[EMI] MDL number = %d\r\n", index);
    if (index < 0 || index >=  num_of_emi_records)
    {
        print("[EMI] setting failed 0x%x\r\n", index);
        ASSERT(0);
    }
    else
    {
        emi_setting_index = index;
    }  
  
    printf("DdrPhySetting_Everest_LP3()+DramcSetting_Everest_LP3()\n");
    
    /* print("[Ahsin]1st 0x464=%x 0x46A=%x 0x466=%x  0x470=%x\n", upmu_get_reg_value(0x464),upmu_get_reg_value(0x46A),upmu_get_reg_value(0x466),upmu_get_reg_value(0x470)); */
    pmic_config_interface(0x464, 0x0, 0x7, 0);
    pmic_config_interface(0x464, 0xF, 0xF, 11);
    pmic_config_interface(0x466, 0x1, 0x1, 3);
    pmic_config_interface(0x466, 0x5, 0x7, 9);
    pmic_config_interface(0x46A, 0x1, 0x1, 7);
    /* print("[Ahsin]2nd 0x464=%x 0x46A=%x 0x466=%x  0x470=%x\n", upmu_get_reg_value(0x464),upmu_get_reg_value(0x46A),upmu_get_reg_value(0x466),upmu_get_reg_value(0x470)); */
    pmic_config_interface(0x44A, 0x3, 0x3, 8);
    pmic_config_interface(0x44A, 0x2, 0x3, 10);
    pmic_config_interface(0x44A, 0x3, 0x3, 12);
    pmic_config_interface(0x44A, 0x3, 0x3, 14);
    pmic_config_interface(0x44C, 0x3, 0x3, 0);
    
    /* limit the PMIC bandwidth and let the PMIC response time slower
       it could prevent oscillation of Vcore */
    pmic_read_interface(0x0452, &pmic_val_0x452, 0xffff, 0);
    pmic_read_interface(0x0450, &pmic_val_0x450, 0xffff, 0);
    pmic_config_interface(0x0452, 0x5602, 0xffff, 0);
    pmic_config_interface(0x0450, 0x7800, 0xffff, 0);

    pmic_config_interface(0x044E,0x1,0x1,1); /*Vcore force PWM mode : addr 0x044E ,bit 1 ,set value 1*/
    pmic_config_interface(0x0462,0x1,0x1,1); /*Vdram force PWM mode : addr 0x0462 ,bit 1 ,set value 1*/  

#if defined(TARGET_BUILD_VARIANT_ENG)    
    pmic_config_interface(0x002E, 0x1, 0x1, 0); /* reset long press reboot timer to avoid timeout */
#endif
  
    Init_DRAM();    

#if defined(TARGET_BUILD_VARIANT_ENG)    
    pmic_config_interface(0x002E, 0x1, 0x1, 0); /* reset long press reboot timer to avoid timeout */
#endif
        
    pmic_config_interface(0x044E,0x0,0x1,1); /*Vcore Auto mode : addr 0x044E ,bit 1 ,set value 0*/

    /* restore pmic value */
    pmic_config_interface(0x0452, pmic_val_0x452, 0xffff, 0);
    pmic_config_interface(0x0450, pmic_val_0x450, 0xffff, 0);

    pmic_config_interface(0x0462,0x1,0x1,1); /*Vdram force PWM mode : addr 0x0462 ,bit 1 ,set value 1*/
  
#if 0  
	{
		DRAMC_CTX_T * p = psCurrDramCtx;
		DramcRegDump(p);
	}
#endif
}

uint32 mt_set_emis(uint8* emi, uint32 len, bool use_default) //array of emi setting.
{
    EMI_SETTINGS *v_emi_settings = (EMI_SETTINGS*)emi;

    mcSHOW_DBG_MSG(("v_emi_settings->sub_version = 0x%x\n",v_emi_settings->sub_version));
    if(use_default)
    {
        if(v_emi_settings->sub_version == 0x1)
        {
            mcSHOW_DBG_MSG(("sub_version == 01, Using preloader tag\n"));
            memcpy(emi_settings,emi,len);
            num_of_emi_records = len/sizeof(EMI_SETTINGS);
            mcSHOW_DBG_MSG(("num_of_emi_records = %d\n",num_of_emi_records));
        }
        else
        {
            mcSHOW_DBG_MSG(("Use default EMI.\n"));
        }
        mt_set_emi();
    }
    else
    {
        mcSHOW_DBG_MSG(("EMI_TAG_Version == 17, Using preloader tag\n"));
        memcpy(emi_settings,emi,len);
        num_of_emi_records = len/sizeof(EMI_SETTINGS);
        mcSHOW_DBG_MSG(("num_of_emi_records = %d\n",num_of_emi_records));
        mt_set_emi();
    }
    mcSHOW_DBG_MSG(("EMI Setting OK.\n"));
    return 0;
}

                
#ifdef DRAM_HQA
int calculate_voltage(int x)
{
  return(600+((625*x)/100));
}

void dram_HQA_adjust_voltage(void)
{
	int Vcore_HV, Vcore_NV, Vcore_LV;
	
#if DUAL_FREQ_K	
    if(get_chip_id_by_efuse() == CHIP_MT6797M) { 
		//highest DRAM frequency is 1600
		Vcore_HV = 0x48; //1.05V
		Vcore_NV = 0x40; //1.00V
		Vcore_LV = 0x38; //0.95V    	
    }
    else {
		#if DFS_COMBINATION_TYPE1	//1866
			Vcore_HV = 0x52; //1.11V
			Vcore_NV = 0x48; //1.05V
			Vcore_LV = 0x3E; //0.99V
		#else //1700 or 1600
			Vcore_HV = 0x48; //1.05V
			Vcore_NV = 0x40; //1.00V
			Vcore_LV = 0x38; //0.95V
		#endif
	}

#else
	DRAM_PLL_FREQ_SEL_T freq_sel;
	freq_sel = DramCtx_LPDDR3.freq_sel;
	
	if(freq_sel == LJ_DDR1866) {
		Vcore_HV = 0x52; //1.11V
		Vcore_NV = 0x48; //1.05V
		Vcore_LV = 0x3E; //0.99V		
	} else if((freq_sel == LJ_DDR1700) || (freq_sel == LJ_DDR1600) || (freq_sel == LC_DDR1600)) {
		Vcore_HV = 0x48; //1.05V
		Vcore_NV = 0x40; //1.00V
		Vcore_LV = 0x38; //0.95V	
	} else {
		Vcore_HV = 0x38; //0.95V
		Vcore_NV = 0x30; //0.90V
		Vcore_LV = 0x28; //0.85V		
	}
#endif
		 
  pmic_config_interface(MT6351_BUCK_VCORE_CON0, 0x0, 0x0003, 0x0); // Enable SW mode
  
#ifdef HVcore
	pmic_config_interface(MT6351_BUCK_VCORE_CON4, Vcore_HV, 0x7F, 0);
	pmic_config_interface(MT6351_BUCK_VGPU_CON4, Vdram_HV, 0x7F, 0);
	print("[HQA]Set HVcore1 setting: Vcore = %d mV(0x%x, should be 0x%x), Vdram = %d mV(0x%x, should be 0x%x)\n",
		calculate_voltage(Vcore_HV), upmu_get_reg_value(MT6351_BUCK_VCORE_CON4), Vcore_HV, 
		calculate_voltage(Vdram_HV), upmu_get_reg_value(MT6351_BUCK_VGPU_CON4), Vdram_HV);
#endif

#ifdef NV	
	pmic_config_interface(MT6351_BUCK_VCORE_CON4, Vcore_NV, 0x7F, 0);
	pmic_config_interface(MT6351_BUCK_VGPU_CON4, Vdram_NV, 0x7F, 0);
	print("[HQA]Set NV setting: Vcore = %d mV(0x%x, should be 0x%x), Vdram = %d mV(0x%x, should be 0x%x)\n",
		calculate_voltage(Vcore_NV), upmu_get_reg_value(MT6351_BUCK_VCORE_CON4), Vcore_NV, 
		calculate_voltage(Vdram_NV), upmu_get_reg_value(MT6351_BUCK_VGPU_CON4), Vdram_NV);
#endif

#ifdef LVcore
	pmic_config_interface(MT6351_BUCK_VCORE_CON4, Vcore_LV, 0x7F, 0);
	pmic_config_interface(MT6351_BUCK_VGPU_CON4, Vdram_LV, 0x7F, 0);
	print("[HQA]Set NV setting: Vcore = %d mV(0x%x, should be 0x%x), Vdram = %d mV(0x%x, should be 0x%x)\n",
		calculate_voltage(Vcore_LV), upmu_get_reg_value(MT6351_BUCK_VCORE_CON4), Vcore_LV, 
		calculate_voltage(Vdram_LV), upmu_get_reg_value(MT6351_BUCK_VGPU_CON4), Vdram_LV);
#endif

#ifdef HVcore_LVdram
	pmic_config_interface(MT6351_BUCK_VCORE_CON4, Vcore_HV, 0x7F, 0);
	pmic_config_interface(MT6351_BUCK_VGPU_CON4, Vdram_LV, 0x7F, 0);
	print("[HQA]Set HVcore_LVdram setting: Vcore = %d mV(0x%x, should be 0x%x), Vdram = %d mV(0x%x, should be 0x%x)\n",
		calculate_voltage(Vcore_HV), upmu_get_reg_value(MT6351_BUCK_VCORE_CON4), Vcore_HV, 
		calculate_voltage(Vdram_LV), upmu_get_reg_value(MT6351_BUCK_VGPU_CON4), Vdram_LV);
#endif

#ifdef LVcore_HVdram	
	pmic_config_interface(MT6351_BUCK_VCORE_CON4, Vcore_LV, 0x7F, 0);
	pmic_config_interface(MT6351_BUCK_VGPU_CON4, Vdram_HV, 0x7F, 0);
	print("[HQA]Set LVcore_HVdram setting: Vcore = %d mV(0x%x, should be 0x%x), Vdram = %d mV(0x%x, should be 0x%x)\n",
		calculate_voltage(Vcore_LV), upmu_get_reg_value(MT6351_BUCK_VCORE_CON4), Vcore_LV, 
		calculate_voltage(Vdram_HV), upmu_get_reg_value(MT6351_BUCK_VGPU_CON4), Vdram_HV);
#endif
}
#endif //DRAM_HQA


void enable_4GB_mode(void)
{
        int i;
        u32 dram_rank_size[4] = {0,0,0,0};
        u64 total_dram_size = 0;
        get_dram_rank_size(dram_rank_size);
	enable_4GB_flag = 0;
        for(i=0; i<4; i++){
		total_dram_size += dram_rank_size[i];
		print("dram_rank_size[%d] = 0x%x\n", i, dram_rank_size[i]);
        }
#if 1
        if(total_dram_size > 0xC0000000ULL) {
		enable_4GB_flag = 1;
		print("[Enable 4GB Support] Total_dram_size = 0x%llx\n,flag:0x%x", total_dram_size,enable_4GB_flag);
		*(volatile unsigned int *)(0x10003208) |= 1 << 15;
		*(volatile unsigned int *)(0x10001f00) |= 1 << 13;
        }
#endif
}

#endif

int get_chip_id_by_efuse(void)
{
	unsigned int value = seclib_get_devinfo_with_index(17);
	
	if((value == 0x2) || (value == 0x7) || (value == 0xF)) {
		return CHIP_MT6797M;
	}
	else if((value == 0x0) || (value == 0x3)) {
		return CHIP_MT6797;
	}
	else if((value == 0x1) || (value == 0x6)) {
		return CHIP_MT6797T;
	}
	else
		return -1;
}

