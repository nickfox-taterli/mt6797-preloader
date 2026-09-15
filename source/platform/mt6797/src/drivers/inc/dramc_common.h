//
//  DRAMC_COMMON.H    
//

#ifndef _DRAMC_COMMON_H_
#define _DRAMC_COMMON_H_

#include "dramc_register.h"
#include "dramc_pi_api.h"
#include "emi.h"
#if __ETT__
    #include <common.h>
#endif

/***********************************************************************/
/*                  Public Types                                       */
/***********************************************************************/

/*------------------------------------------------------------*/
/*                  macros, defines, typedefs, enums          */
/*------------------------------------------------------------*/
#if 0 //remove useless code
/************************ Bit Process *************************/
#define mcBITL(b)               (1L << (b))
#define mcBIT(b)                mcBITL(b)
#define mcMASK(w)               (mcBIT(w) - 1)
#define mcMASKS(w, b)           (mcMASK(w) << (b))
#define mcCLR_BIT_C(a)          ((~Bit_##a) & 0xFFFF)

#define mcSET_MASK(a, b)        ((a) |= (b))
#define mcCLR_MASK(a, b)        ((a) &= (~(b)))
#define mcSET_BIT(a, b)         mcSET_MASK(a, mcBIT(b))
#define mcCLR_BIT(a, b)         mcCLR_MASK(a, mcBIT(b))
#define mcCHK_BIT1(a, b)        ((a) & mcBIT(b))
#define mcCHK_BITM(a, b, m)     (((a) >> (b)) & (m))
#define mcCHK_BITS(a, b, w)     mcCHK_BITM(a, b, mcMASK(w))
#define mcTEST_BIT(a, b)        mcCHK_BIT1(a, b)
#define mcCHG_BIT(a, b)         ((a) ^= mcBIT(b))

#define mcSET_FIELD0(var, value, mask, pos) mcSET_MASK(var, (((U32)value) << (pos)) & (mask))

#define mcSET_FIELD(var, value, mask, pos)  \
{                                           \
    mcCLR_MASK(var, mask);                  \
    mcSET_FIELD0(var, value, mask, pos);    \
}

#define mcGET_FIELD(var, mask, pos)     (((var) & (mask)) >> (pos))


#if fcFOR_CHIP_ID == fcK2 || fcFOR_CHIP_ID == fcJADE
#define mcSET_DRAMC_REG_ADDR(offset)    (offset)
#define mcSET_SYS_REG_ADDR(offset)      (offset)
#endif


#ifndef BIT0
#define BIT0                    mcBIT(0)
#define BIT1                    mcBIT(1)
#define BIT2                    mcBIT(2)
#define BIT3                    mcBIT(3)
#define BIT4                    mcBIT(4)
#define BIT5                    mcBIT(5)
#define BIT6                    mcBIT(6)
#define BIT7                    mcBIT(7)
#endif

#define mcLOW_MASK(u16)         ((UCHAR) ((u16) & mcMASK(8)))
#define mcHIGH_MASK(u16)        ((UCHAR) ((u16) & ~mcMASK(8)))
#define mcLOW2HIGH(u16)         (((UINT16) (u16)) << 8)
/* (1234, 5678) -> 7834 */
#define mc2BYTE(ch1L, ch2H)     (mcLOW_MASK(ch1L) | mcLOW2HIGH(ch2H))
/* (1234) -> 0034 */
//#define mcLOW_BYTE(u16)         ((UCHAR)(u16))
#define mcLOW_BYTE(u16)         mcLOW_MASK(u16)
/* (1234) -> 0012 */
#define mcHIGH_BYTE(u16)        ((UCHAR) (((UINT16) (u16)) >> 8))

/* xxxHHHHH + LLLLLxxx */
#define mcUNSIGN_2BYTE(u8h, u8l, hmsk, lmsk)                \
    (((mcLOW2HIGH((u8h) & mcMASK(hmsk)) | (u8l))            \
        << (8 - (hmsk))) >> ((8 - (hmsk)) + (8 - (lmsk))))

#define mcSIGN_2BYTE(u8h, u8l, hmsk, lmsk)                  \
    (((INT16) ((mcLOW2HIGH((u8h) & mcMASK(hmsk)) | (u8l))     \
        << (8 - (hmsk)))) >> ((8 - (hmsk)) + (8 - (lmsk))))

#define mcSET_MASKS(par)                mcMASKS(cw##par, cp##par)
#define mcFIELD_CMB(val, par)           mcFIELD(val, cm##par, cp##par)
#define mcGET_FIELD_CMB(var, par)       mcGET_FIELD(var, cm##par, cp##par)
#define mcSET_FIELD_CMB(var, val, par)  mcSET_FIELD(var, val, cm##par, cp##par)

#define CEIL_A_OVER_B(_A, _B) (((_A)-(_B)*((_A)/(_B))) > 0? (_A)/(_B)+1:(_A)/(_B))
#endif
/************************** Common Macro *********************/

#define dsb() __asm__ __volatile__ ("dsb" : : : "memory")

// K2?? : The following needs to be porting.
// choose a proper mcDELAY
#if __ETT__
#if EVEREST_MEM_C_TEST_CODE
#define mcDELAY_US(x)       //delay_a_while((U32) (x*10))
#define mcDELAY_MS(x)       //delay_a_while((U32) (x*10*1000))
#else
#define mcDELAY_US(x)       GPT_Delay_us((U32) (x))
#define mcDELAY_MS(x)       GPT_Delay_ms((U32) (x))
#endif
#else
//====
  #if __FLASH_TOOL_DA__
    #define mcDELAY_US(x)       gpt4_busy_wait_us(x)
    #define mcDELAY_MS(x)       gpt4_busy_wait_us(x*1000)
  #else
#define mcDELAY_US(x)       gpt_busy_wait_us(x)
#define mcDELAY_MS(x)       gpt_busy_wait_us(x*1000)
#endif
#endif

/**********************************************/
/* Priority of debug log                      */
/*--------------------------------------------*/
/* mcSHOW_DBG_MSG: High                       */
/* mcSHOW_DBG_MSG2: Medium High               */
/* mcSHOW_DBG_MSG3: Medium Low                */ 
/* mcSHOW_DBG_MSG4: Low                       */
/**********************************************/
#if __FLASH_TOOL_DA__
  #define printf DBG_MSG
  #define print DBG_MSG
#endif

#if EVEREST_MEM_C_TEST_CODE
#define mcSHOW_DBG_MSG(_x_)   printf _x_    
#define mcSHOW_DBG_MSG2(_x_)  printf _x_    
#define mcSHOW_DBG_MSG3(_x_)  printf _x_    
#define mcSHOW_DBG_MSG4(_x_)  if (RXPERBIT_LOG_PRINT) {printf _x_;}
#define mcSHOW_USER_MSG(_x_)    
#define mcSHOW_ERR_MSG(_x_)   printf _x_
#else

#if __ETT__
#ifdef DDR_INIT_TIME_PROFILING   
#define mcSHOW_DBG_MSG(_x_) 
#define mcSHOW_DBG_MSG2(_x_)   
#define mcSHOW_DBG_MSG3(_x_) 
#define mcSHOW_DBG_MSG4(_x_) 
#define mcSHOW_USER_MSG(_x_)   
#define mcSHOW_ERR_MSG(_x_)  opt_print _x_
#else
#ifdef RELEASE
#define mcSHOW_DBG_MSG(_x_)  opt_print _x_
#define mcSHOW_DBG_MSG2(_x_)   
#define mcSHOW_DBG_MSG3(_x_) 
#define mcSHOW_DBG_MSG4(_x_) 
#define mcSHOW_USER_MSG(_x_)  must_print _x_
#define mcSHOW_ERR_MSG(_x_)   must_print _x_
#else
#define mcSHOW_DBG_MSG(_x_)   mcDELAY_US(10);opt_print _x_     
#define mcSHOW_DBG_MSG2(_x_)  mcDELAY_US(10);opt_print _x_     
#define mcSHOW_DBG_MSG3(_x_)  
#define mcSHOW_DBG_MSG4(_x_)  
#define mcSHOW_USER_MSG(_x_)  
#define mcSHOW_ERR_MSG(_x_)   opt_print _x_
#endif
#endif
#else
#if defined(TARGET_BUILD_VARIANT_ENG)
#define mcSHOW_DBG_MSG(_x_)   print _x_    
#define mcSHOW_DBG_MSG2(_x_)  print _x_    
#define mcSHOW_DBG_MSG3(_x_)      
#define mcSHOW_DBG_MSG4(_x_)  
#define mcSHOW_USER_MSG(_x_)    
#define mcSHOW_ERR_MSG(_x_)   print _x_
#else 
#define mcSHOW_DBG_MSG(_x_)     
#define mcSHOW_DBG_MSG2(_x_)    
#define mcSHOW_DBG_MSG3(_x_)    
#define mcSHOW_DBG_MSG4(_x_)    
#define mcSHOW_USER_MSG(_x_)    
#define mcSHOW_ERR_MSG(_x_)     
#endif
#endif
#endif

#define mcFPRINTF(_x_)           
//#define mcFPRINTF(_x_)          fprintf _x_;

extern int dump_log;
#endif   // _DRAMC_COMMON_H_
