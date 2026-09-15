/** @file hal_io.cpp
 *  hal_io.cpp provides functions of register access
 */

#include "x_hal_io.h"
#include "dramc_common.h"

#if 1//REG_ACCESS_PORTING_DGB
extern U8 RegLogEnable;
#endif

#if fcFOR_CHIP_ID == fcEverest
const U32 arRegBaseAddrList[8] = 
{
    CHA_DRAMC_NAO_BASE, 
    CHB_DRAMC_NAO_BASE,
    CHA_DRAMC_AO_BASE, 
    CHB_DRAMC_AO_BASE, 
    PHY_A_BASE,
    PHY_B_BASE, 
    PHY_C_BASE, 
    PHY_D_BASE
};
#endif


// Translate SW virtual base address to real HW base.
U32 u4RegBaseAddrTraslate(U32 u4reg_addr)
{
	U32 u4Offset = u4reg_addr & 0xfff;
	U32 u4RegType = ((u4reg_addr-CHA_DRAMC_NAO_BASE_VIRTUAL) >> POS_BANK_NUM) & 0xf;
	
       //mcSHOW_DBG_MSG(("\n[u4RegBaseAddrTraslate]  0x%x => 0x%x", u4reg_addr , (arRegBaseAddrList[u4RegType] +u4Offset)));
	return (arRegBaseAddrList[u4RegType] +u4Offset); 
}

//-------------------------------------------------------------------------
/** ucDram_Register_Read
 *  DRAM register read (32-bit).
 *  @param  u4reg_addr    register address in 32-bit.
 *  @param  pu4reg_value  Pointer of register read value.
 *  @retval 0: OK, 1: FAIL 
 */
//-------------------------------------------------------------------------
// This function need to be porting by BU requirement
U32 u4Dram_Register_Read(U32 u4reg_addr)
{
	U32 u4reg_value;

#if fcFOR_CHIP_ID == fcA60501
    ucDramRegRead_1(u4reg_addr, &u4reg_value);

#elif fcFOR_CHIP_ID == fcEverest
   u4reg_addr = u4RegBaseAddrTraslate(u4reg_addr);
   u4reg_value = (*(volatile unsigned int *)(u4reg_addr));
#endif
   return u4reg_value;
}


//-------------------------------------------------------------------------
/** ucDram_Register_Write
 *  DRAM register write (32-bit).
 *  @param  u4reg_addr    register address in 32-bit.
 *  @param  u4reg_value   register write value.
 *  @retval 0: OK, 1: FAIL 
 */
//-------------------------------------------------------------------------
#if REG_SHUFFLE_REG_CHECK
extern U8 ShuffleRegCheck;
extern void ShuffleRegCheckProgram(U32 u4Addr);
#endif

// This function need to be porting by BU requirement
U8 ucDram_Register_Write(U32 u4reg_addr, U32 u4reg_value)
{
	U8 ucstatus = 0;
	
#if fcFOR_CHIP_ID == fcA60501
            //mcSHOW_DBG_MSG2(("[Reg Write] 0x%8x 0x%8x\n", u4reg_addr, u4reg_value));
        
            if (ucDramRegWrite_1(u4reg_addr, u4reg_value))
            {
                ucstatus = 1;
            }
#elif fcFOR_CHIP_ID == fcEverest
            u4reg_addr = u4RegBaseAddrTraslate(u4reg_addr);
    #if !EVEREST_MEM_C_TEST_CODE
            (*(volatile unsigned int *)u4reg_addr) = u4reg_value;
           dsb();
    #else
            printf("\n[REG_ACCESS_PORTING_DBG]   ucDramC_Register_Write Reg(0x%x) = 0x%x\n",u4reg_addr,  u4reg_value);  
    #endif
#endif
        
#if REG_ACCESS_PORTING_DGB
		if(RegLogEnable)
		{
			mcSHOW_DBG_MSG(("\n[REG_ACCESS_PORTING_DBG]   ucDramC_Register_Write Reg(0x%X) = 0x%X\n",u4reg_addr,  u4reg_value));
			mcFPRINTF((fp_A60501, "\n[REG_ACCESS_PORTING_DBG]	ucDramC_Register_Write Reg(0x%X) = 0x%X\n",u4reg_addr,	u4reg_value));
			//mcFPRINTF((fp_A60501, "\nRISCWrite ('h%05X , 32'h%X);\n",u4reg_addr&0xFFFFF,	u4reg_value));
		}
#endif

#if REG_SHUFFLE_REG_CHECK
	if(ShuffleRegCheck)
	{
		ShuffleRegCheckProgram(u4reg_addr);
	}
#endif

    return ucstatus;
}


void vIO32Write4BMsk(U32 reg32, U32 val32, U32 msk32)
{
    U32 u4Val;
    
    val32 &=msk32;
    
    u4Val = u4Dram_Register_Read(reg32);
    u4Val = ((u4Val &~msk32)|val32);   
    ucDram_Register_Write(reg32, u4Val);
}


void vIO32Write4B_All(U32 reg32, U32 val32)
{
#if fcFOR_CHIP_ID == fcA60501
    vIO32Write4B(reg32, val32);
    vIO32Write4B(reg32+((U32)2<<POS_BANK_NUM), val32);
    vIO32Write4B(reg32+((U32)1<<POS_BANK_NUM), val32);
	
#elif fcFOR_CHIP_ID == fcEverest	
	U8 ii, u1AllCount;
        U32 u4RegType = (reg32 & (0xf <<POS_BANK_NUM));
        
        reg32 &= 0xffff;     // remove channel information
        
        if(u4RegType >=PHY_A_BASE_VIRTUAL)//PHY 
        {
            u1AllCount =4;
            reg32 += PHY_A_BASE_VIRTUAL;
        }
        else if(u4RegType >=CHA_DRAMC_AO_BASE_VIRTUAL)// DramC AO
        {
            u1AllCount =2;
            reg32 += CHA_DRAMC_AO_BASE_VIRTUAL;
        }
        else // DramC NAO
        {
            u1AllCount =2;
            reg32 += CHA_DRAMC_NAO_BASE_VIRTUAL;
        }

	for(ii=0; ii< u1AllCount; ii++)
	{
		vIO32Write4B(reg32+((U32)ii<<POS_BANK_NUM), val32);
	}	
#endif
#if 0
    // debug
    //mcSHOW_DBG_MSG(("RISCWrite : address %05x data %08x\n", reg32&0x000fffff, val32));
    mcFPRINTF((fp_A60501, "RISCWrite : address %05x data %08x wait  0\n", reg32&0x000fffff, val32));
    //mcSHOW_DBG_MSG(("RISCWrite : address %05x data %08x\n", (reg32+((U32)1<<POS_BANK_NUM))&0x000fffff, val32));
    mcFPRINTF((fp_A60501, "RISCWrite : address %05x data %08x wait  0\n", (reg32+((U32)1<<POS_BANK_NUM))&0x000fffff, val32));
    //mcSHOW_DBG_MSG(("RISCWrite : address %05x data %08x\n", (reg32+((U32)2<<POS_BANK_NUM))&0x000fffff, val32));
    mcFPRINTF((fp_A60501, "RISCWrite : address %05x data %08x wait  0\n", (reg32+((U32)2<<POS_BANK_NUM))&0x000fffff, val32));
#endif
}

void vIO32Write4BMsk_All(U32 reg32, U32 val32, U32 msk32)
{
    U32 u4Val;
    
#if fcFOR_CHIP_ID == fcA60501
    val32 &=msk32;
    u4Val = u4Dram_Register_Read(reg32);
    u4Val = ((u4Val &~msk32)|val32);   
   ucDram_Register_Write(reg32, u4Val);
   
    u4Val = u4Dram_Register_Read(reg32+((U32)1<<POS_BANK_NUM));    
    u4Val = ((u4Val &~msk32)|val32);   
   ucDram_Register_Write(reg32+((U32)1<<POS_BANK_NUM), u4Val);

    u4Val = u4Dram_Register_Read(reg32+((U32)2<<POS_BANK_NUM));    
    u4Val = ((u4Val &~msk32)|val32);   
   ucDram_Register_Write(reg32+((U32)2<<POS_BANK_NUM), u4Val);

#elif fcFOR_CHIP_ID == fcEverest
    U8 ii, u1AllCount;
    U32 u4RegType = (reg32 & (0xf <<POS_BANK_NUM));

    reg32 &= 0xffff;     // remove channel information

    if(u4RegType >=PHY_A_BASE_VIRTUAL)//PHY 
    {
        u1AllCount =4;
        reg32 += PHY_A_BASE_VIRTUAL;
    }
    else if(u4RegType >=CHA_DRAMC_AO_BASE_VIRTUAL)// DramC AO
    {
        u1AllCount =2;
        reg32 += CHA_DRAMC_AO_BASE_VIRTUAL;
    }
    else // DramC NAO
    {
        u1AllCount =2;
        reg32 += CHA_DRAMC_NAO_BASE_VIRTUAL;
    }

    for(ii=0; ii< u1AllCount; ii++)
    {
        u4Val = u4Dram_Register_Read(reg32+((U32)ii<<POS_BANK_NUM));	
        u4Val = ((u4Val &~msk32)|val32);	
        ucDram_Register_Write(reg32+((U32)ii<<POS_BANK_NUM), u4Val);
    }	
#endif
}
