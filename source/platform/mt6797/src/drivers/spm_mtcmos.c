/*
//XXX: only in kernel
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/delay.h>    //udelay

#include <mach/mt_typedefs.h>
#include <mach/mt_spm.h>
#include <mach/mt_spm_mtcmos.h>
#include <mach/mt_spm_mtcmos_internal.h>
#include <mach/hotplug.h>
#include <mach/mt_clkmgr.h>
*/
#include <platform.h>
#include <spm.h>
#include <spm_mtcmos.h>
#include <spm_mtcmos_internal.h>
#include <timer.h>  //udelay
#include <pll.h>

/**************************************
 * for CPU MTCMOS
 **************************************/
 /*
//XXX: only in kernel
static DEFINE_SPINLOCK(spm_cpu_lock);

void spm_mtcmos_cpu_lock(unsigned long *flags)
{
    spin_lock_irqsave(&spm_cpu_lock, *flags);
}

void spm_mtcmos_cpu_unlock(unsigned long *flags)
{
    spin_unlock_irqrestore(&spm_cpu_lock, *flags);
}
*/
#define spm_mtcmos_cpu_lock(x)
#define spm_mtcmos_cpu_unlock(x)

typedef int (*spm_cpu_mtcmos_ctrl_func)(int state, int chkWfiBeforePdn);
static spm_cpu_mtcmos_ctrl_func spm_cpu_mtcmos_ctrl_funcs[] =
{
    spm_mtcmos_ctrl_cpu0,
    spm_mtcmos_ctrl_cpu1,
    spm_mtcmos_ctrl_cpu2,
    spm_mtcmos_ctrl_cpu3,
    spm_mtcmos_ctrl_cpu4,
    spm_mtcmos_ctrl_cpu5,
    spm_mtcmos_ctrl_cpu6,
    spm_mtcmos_ctrl_cpu7
};
int spm_mtcmos_ctrl_cpu(unsigned int cpu, int state, int chkWfiBeforePdn)
{
    return (*spm_cpu_mtcmos_ctrl_funcs[cpu])(state, chkWfiBeforePdn);
}

int spm_mtcmos_ctrl_cpu0(int state, int chkWfiBeforePdn)
{
    unsigned long flags;

    /* enable register control */
    spm_write(SPM_POWERON_CONFIG_SET, (SPM_PROJECT_CODE << 16) | (1U << 0));

    if (state == STA_POWER_DOWN)
    {
        if (chkWfiBeforePdn)
            while ((spm_read(SPM_SLEEP_TIMER_STA) & CA7_CPU0_STANDBYWFI) == 0);

        spm_mtcmos_cpu_lock(&flags);

        spm_write(SPM_CA7_CPU0_PWR_CON, spm_read(SPM_CA7_CPU0_PWR_CON) | PWR_ISO);

        spm_write(SPM_CA7_CPU0_PWR_CON, spm_read(SPM_CA7_CPU0_PWR_CON) | SRAM_CKISO);
        spm_write(SPM_CA7_CPU0_PWR_CON, spm_read(SPM_CA7_CPU0_PWR_CON) & ~SRAM_ISOINT_B);
        spm_write(SPM_CA7_CPU0_L1_PDN, spm_read(SPM_CA7_CPU0_L1_PDN) | L1_PDN);
    #ifndef CFG_FPGA_PLATFORM
        while ((spm_read(SPM_CA7_CPU0_L1_PDN) & L1_PDN_ACK) != L1_PDN_ACK);
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_write(SPM_CA7_CPU0_PWR_CON, spm_read(SPM_CA7_CPU0_PWR_CON) & ~PWR_RST_B);
        spm_write(SPM_CA7_CPU0_PWR_CON, spm_read(SPM_CA7_CPU0_PWR_CON) | PWR_CLK_DIS);

        spm_write(SPM_CA7_CPU0_PWR_CON, spm_read(SPM_CA7_CPU0_PWR_CON) & ~PWR_ON);
        spm_write(SPM_CA7_CPU0_PWR_CON, spm_read(SPM_CA7_CPU0_PWR_CON) & ~PWR_ON_2ND);
    #ifndef CFG_FPGA_PLATFORM
        while (((spm_read(SPM_PWR_STATUS) & CA7_CPU0) != 0) || ((spm_read(SPM_PWR_STATUS_2ND) & CA7_CPU0) != 0));
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_mtcmos_cpu_unlock(&flags);
    }
    else /* STA_POWER_ON */
    {
        spm_mtcmos_cpu_lock(&flags);

        spm_write(SPM_CA7_CPU0_PWR_CON, spm_read(SPM_CA7_CPU0_PWR_CON) | PWR_ON);
        udelay(1);
        spm_write(SPM_CA7_CPU0_PWR_CON, spm_read(SPM_CA7_CPU0_PWR_CON) | PWR_ON_2ND);
    #ifndef CFG_FPGA_PLATFORM
        while (((spm_read(SPM_PWR_STATUS) & CA7_CPU0) != CA7_CPU0) || ((spm_read(SPM_PWR_STATUS_2ND) & CA7_CPU0) != CA7_CPU0));
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_write(SPM_CA7_CPU0_PWR_CON, spm_read(SPM_CA7_CPU0_PWR_CON) & ~PWR_ISO);

        spm_write(SPM_CA7_CPU0_L1_PDN, spm_read(SPM_CA7_CPU0_L1_PDN) & ~L1_PDN);
    #ifndef CFG_FPGA_PLATFORM
        while ((spm_read(SPM_CA7_CPU0_L1_PDN) & L1_PDN_ACK) != 0);
    #endif //#ifndef CFG_FPGA_PLATFORM
        udelay(1);
        spm_write(SPM_CA7_CPU0_PWR_CON, spm_read(SPM_CA7_CPU0_PWR_CON) | SRAM_ISOINT_B);
        spm_write(SPM_CA7_CPU0_PWR_CON, spm_read(SPM_CA7_CPU0_PWR_CON) & ~SRAM_CKISO);

        spm_write(SPM_CA7_CPU0_PWR_CON, spm_read(SPM_CA7_CPU0_PWR_CON) & ~PWR_CLK_DIS);
        spm_write(SPM_CA7_CPU0_PWR_CON, spm_read(SPM_CA7_CPU0_PWR_CON) | PWR_RST_B);

        spm_mtcmos_cpu_unlock(&flags);
    }

    return 0;
}

int spm_mtcmos_ctrl_cpu1(int state, int chkWfiBeforePdn)
{
    unsigned long flags;

    /* enable register control */
    spm_write(SPM_POWERON_CONFIG_SET, (SPM_PROJECT_CODE << 16) | (1U << 0));

    if (state == STA_POWER_DOWN)
    {
        if (chkWfiBeforePdn)
            while ((spm_read(SPM_SLEEP_TIMER_STA) & CA7_CPU1_STANDBYWFI) == 0);

        spm_mtcmos_cpu_lock(&flags);

        spm_write(SPM_CA7_CPU1_PWR_CON, spm_read(SPM_CA7_CPU1_PWR_CON) | PWR_ISO);

        spm_write(SPM_CA7_CPU1_PWR_CON, spm_read(SPM_CA7_CPU1_PWR_CON) | SRAM_CKISO);
        spm_write(SPM_CA7_CPU1_PWR_CON, spm_read(SPM_CA7_CPU1_PWR_CON) & ~SRAM_ISOINT_B);
        spm_write(SPM_CA7_CPU1_L1_PDN, spm_read(SPM_CA7_CPU1_L1_PDN) | L1_PDN);
    #ifndef CFG_FPGA_PLATFORM
        while ((spm_read(SPM_CA7_CPU1_L1_PDN) & L1_PDN_ACK) != L1_PDN_ACK);
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_write(SPM_CA7_CPU1_PWR_CON, spm_read(SPM_CA7_CPU1_PWR_CON) & ~PWR_RST_B);
        spm_write(SPM_CA7_CPU1_PWR_CON, spm_read(SPM_CA7_CPU1_PWR_CON) | PWR_CLK_DIS);

        spm_write(SPM_CA7_CPU1_PWR_CON, spm_read(SPM_CA7_CPU1_PWR_CON) & ~PWR_ON);
        spm_write(SPM_CA7_CPU1_PWR_CON, spm_read(SPM_CA7_CPU1_PWR_CON) & ~PWR_ON_2ND);
    #ifndef CFG_FPGA_PLATFORM
        while (((spm_read(SPM_PWR_STATUS) & CA7_CPU1) != 0) || ((spm_read(SPM_PWR_STATUS_2ND) & CA7_CPU1) != 0));
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_mtcmos_cpu_unlock(&flags);
    }
    else /* STA_POWER_ON */
    {
        spm_mtcmos_cpu_lock(&flags);

        spm_write(SPM_CA7_CPU1_PWR_CON, spm_read(SPM_CA7_CPU1_PWR_CON) | PWR_ON);
        udelay(1);
        spm_write(SPM_CA7_CPU1_PWR_CON, spm_read(SPM_CA7_CPU1_PWR_CON) | PWR_ON_2ND);
    #ifndef CFG_FPGA_PLATFORM
        while (((spm_read(SPM_PWR_STATUS) & CA7_CPU1) != CA7_CPU1) || ((spm_read(SPM_PWR_STATUS_2ND) & CA7_CPU1) != CA7_CPU1));
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_write(SPM_CA7_CPU1_PWR_CON, spm_read(SPM_CA7_CPU1_PWR_CON) & ~PWR_ISO);

        spm_write(SPM_CA7_CPU1_L1_PDN, spm_read(SPM_CA7_CPU1_L1_PDN) & ~L1_PDN);
    #ifndef CFG_FPGA_PLATFORM
        while ((spm_read(SPM_CA7_CPU1_L1_PDN) & L1_PDN_ACK) != 0);
    #endif //#ifndef CFG_FPGA_PLATFORM
        udelay(1);
        spm_write(SPM_CA7_CPU1_PWR_CON, spm_read(SPM_CA7_CPU1_PWR_CON) | SRAM_ISOINT_B);
        spm_write(SPM_CA7_CPU1_PWR_CON, spm_read(SPM_CA7_CPU1_PWR_CON) & ~SRAM_CKISO);

        spm_write(SPM_CA7_CPU1_PWR_CON, spm_read(SPM_CA7_CPU1_PWR_CON) & ~PWR_CLK_DIS);
        spm_write(SPM_CA7_CPU1_PWR_CON, spm_read(SPM_CA7_CPU1_PWR_CON) | PWR_RST_B);

        spm_mtcmos_cpu_unlock(&flags);
    }

    return 0;
}

int spm_mtcmos_ctrl_cpu2(int state, int chkWfiBeforePdn)
{
    unsigned long flags;

    /* enable register control */
    spm_write(SPM_POWERON_CONFIG_SET, (SPM_PROJECT_CODE << 16) | (1U << 0));

    if (state == STA_POWER_DOWN)
    {
        if (chkWfiBeforePdn)
            while ((spm_read(SPM_SLEEP_TIMER_STA) & CA7_CPU2_STANDBYWFI) == 0);

        spm_mtcmos_cpu_lock(&flags);

        spm_write(SPM_CA7_CPU2_PWR_CON, spm_read(SPM_CA7_CPU2_PWR_CON) | PWR_ISO);

        spm_write(SPM_CA7_CPU2_PWR_CON, spm_read(SPM_CA7_CPU2_PWR_CON) | SRAM_CKISO);
        spm_write(SPM_CA7_CPU2_PWR_CON, spm_read(SPM_CA7_CPU2_PWR_CON) & ~SRAM_ISOINT_B);
        spm_write(SPM_CA7_CPU2_L1_PDN, spm_read(SPM_CA7_CPU2_L1_PDN) | L1_PDN);
    #ifndef CFG_FPGA_PLATFORM
        while ((spm_read(SPM_CA7_CPU2_L1_PDN) & L1_PDN_ACK) != L1_PDN_ACK);
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_write(SPM_CA7_CPU2_PWR_CON, spm_read(SPM_CA7_CPU2_PWR_CON) & ~PWR_RST_B);
        spm_write(SPM_CA7_CPU2_PWR_CON, spm_read(SPM_CA7_CPU2_PWR_CON) | PWR_CLK_DIS);

        spm_write(SPM_CA7_CPU2_PWR_CON, spm_read(SPM_CA7_CPU2_PWR_CON) & ~PWR_ON);
        spm_write(SPM_CA7_CPU2_PWR_CON, spm_read(SPM_CA7_CPU2_PWR_CON) & ~PWR_ON_2ND);
    #ifndef CFG_FPGA_PLATFORM
        while (((spm_read(SPM_PWR_STATUS) & CA7_CPU2) != 0) || ((spm_read(SPM_PWR_STATUS_2ND) & CA7_CPU2) != 0));
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_mtcmos_cpu_unlock(&flags);
    }
    else /* STA_POWER_ON */
    {
        spm_mtcmos_cpu_lock(&flags);

        spm_write(SPM_CA7_CPU2_PWR_CON, spm_read(SPM_CA7_CPU2_PWR_CON) | PWR_ON);
        udelay(1);
        spm_write(SPM_CA7_CPU2_PWR_CON, spm_read(SPM_CA7_CPU2_PWR_CON) | PWR_ON_2ND);
    #ifndef CFG_FPGA_PLATFORM
        while (((spm_read(SPM_PWR_STATUS) & CA7_CPU2) != CA7_CPU2) || ((spm_read(SPM_PWR_STATUS_2ND) & CA7_CPU2) != CA7_CPU2));
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_write(SPM_CA7_CPU2_PWR_CON, spm_read(SPM_CA7_CPU2_PWR_CON) & ~PWR_ISO);

        spm_write(SPM_CA7_CPU2_L1_PDN, spm_read(SPM_CA7_CPU2_L1_PDN) & ~L1_PDN);
    #ifndef CFG_FPGA_PLATFORM
        while ((spm_read(SPM_CA7_CPU2_L1_PDN) & L1_PDN_ACK) != 0);
    #endif //#ifndef CFG_FPGA_PLATFORM
        udelay(1);
        spm_write(SPM_CA7_CPU2_PWR_CON, spm_read(SPM_CA7_CPU2_PWR_CON) | SRAM_ISOINT_B);
        spm_write(SPM_CA7_CPU2_PWR_CON, spm_read(SPM_CA7_CPU2_PWR_CON) & ~SRAM_CKISO);

        spm_write(SPM_CA7_CPU2_PWR_CON, spm_read(SPM_CA7_CPU2_PWR_CON) & ~PWR_CLK_DIS);
        spm_write(SPM_CA7_CPU2_PWR_CON, spm_read(SPM_CA7_CPU2_PWR_CON) | PWR_RST_B);

        spm_mtcmos_cpu_unlock(&flags);
    }

    return 0;
}

int spm_mtcmos_ctrl_cpu3(int state, int chkWfiBeforePdn)
{
    unsigned long flags;

    /* enable register control */
    spm_write(SPM_POWERON_CONFIG_SET, (SPM_PROJECT_CODE << 16) | (1U << 0));

    if (state == STA_POWER_DOWN)
    {
        if (chkWfiBeforePdn)
            while ((spm_read(SPM_SLEEP_TIMER_STA) & CA7_CPU3_STANDBYWFI) == 0);

        spm_mtcmos_cpu_lock(&flags);

        spm_write(SPM_CA7_CPU3_PWR_CON, spm_read(SPM_CA7_CPU3_PWR_CON) | PWR_ISO);

        spm_write(SPM_CA7_CPU3_PWR_CON, spm_read(SPM_CA7_CPU3_PWR_CON) | SRAM_CKISO);
        spm_write(SPM_CA7_CPU3_PWR_CON, spm_read(SPM_CA7_CPU3_PWR_CON) & ~SRAM_ISOINT_B);
        spm_write(SPM_CA7_CPU3_L1_PDN, spm_read(SPM_CA7_CPU3_L1_PDN) | L1_PDN);
    #ifndef CFG_FPGA_PLATFORM
        while ((spm_read(SPM_CA7_CPU3_L1_PDN) & L1_PDN_ACK) != L1_PDN_ACK);
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_write(SPM_CA7_CPU3_PWR_CON, spm_read(SPM_CA7_CPU3_PWR_CON) & ~PWR_RST_B);
        spm_write(SPM_CA7_CPU3_PWR_CON, spm_read(SPM_CA7_CPU3_PWR_CON) | PWR_CLK_DIS);

        spm_write(SPM_CA7_CPU3_PWR_CON, spm_read(SPM_CA7_CPU3_PWR_CON) & ~PWR_ON);
        spm_write(SPM_CA7_CPU3_PWR_CON, spm_read(SPM_CA7_CPU3_PWR_CON) & ~PWR_ON_2ND);
    #ifndef CFG_FPGA_PLATFORM
        while (((spm_read(SPM_PWR_STATUS) & CA7_CPU3) != 0) || ((spm_read(SPM_PWR_STATUS_2ND) & CA7_CPU3) != 0));
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_mtcmos_cpu_unlock(&flags);
    }
    else /* STA_POWER_ON */
    {
        spm_mtcmos_cpu_lock(&flags);

        spm_write(SPM_CA7_CPU3_PWR_CON, spm_read(SPM_CA7_CPU3_PWR_CON) | PWR_ON);
        udelay(1);
        spm_write(SPM_CA7_CPU3_PWR_CON, spm_read(SPM_CA7_CPU3_PWR_CON) | PWR_ON_2ND);
    #ifndef CFG_FPGA_PLATFORM
        while (((spm_read(SPM_PWR_STATUS) & CA7_CPU3) != CA7_CPU3) || ((spm_read(SPM_PWR_STATUS_2ND) & CA7_CPU3) != CA7_CPU3));
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_write(SPM_CA7_CPU3_PWR_CON, spm_read(SPM_CA7_CPU3_PWR_CON) & ~PWR_ISO);

        spm_write(SPM_CA7_CPU3_L1_PDN, spm_read(SPM_CA7_CPU3_L1_PDN) & ~L1_PDN);
    #ifndef CFG_FPGA_PLATFORM
        while ((spm_read(SPM_CA7_CPU3_L1_PDN) & L1_PDN_ACK) != 0);
    #endif //#ifndef CFG_FPGA_PLATFORM
        udelay(1);
        spm_write(SPM_CA7_CPU3_PWR_CON, spm_read(SPM_CA7_CPU3_PWR_CON) | SRAM_ISOINT_B);
        spm_write(SPM_CA7_CPU3_PWR_CON, spm_read(SPM_CA7_CPU3_PWR_CON) & ~SRAM_CKISO);

        spm_write(SPM_CA7_CPU3_PWR_CON, spm_read(SPM_CA7_CPU3_PWR_CON) & ~PWR_CLK_DIS);
        spm_write(SPM_CA7_CPU3_PWR_CON, spm_read(SPM_CA7_CPU3_PWR_CON) | PWR_RST_B);

        spm_mtcmos_cpu_unlock(&flags);
    }

    return 0;
}

int spm_mtcmos_ctrl_cpu4(int state, int chkWfiBeforePdn)
{
    unsigned long flags;

    /* enable register control */
    spm_write(SPM_POWERON_CONFIG_SET, (SPM_PROJECT_CODE << 16) | (1U << 0));

    if (state == STA_POWER_DOWN)
    {
        if (chkWfiBeforePdn)
            //while ((spm_read(SPM_SLEEP_TIMER_STA) & CA15_CPU0_STANDBYWFI) == 0);
            mdelay(500);

        spm_mtcmos_cpu_lock(&flags);

        spm_write(SPM_CA15_CPU0_PWR_CON, spm_read(SPM_CA15_CPU0_PWR_CON) | PWR_ISO);

        spm_write(SPM_CA15_CPU0_PWR_CON, spm_read(SPM_CA15_CPU0_PWR_CON) | SRAM_CKISO);
        spm_write(SPM_CA15_CPU0_PWR_CON, spm_read(SPM_CA15_CPU0_PWR_CON) & ~SRAM_ISOINT_B);
        spm_write(SPM_CA15_L1_PWR_CON, spm_read(SPM_CA15_L1_PWR_CON) | CPU0_CA15_L1_PDN);
    #ifndef CFG_FPGA_PLATFORM
        while ((spm_read(SPM_CA15_L1_PWR_CON) & CPU0_CA15_L1_PDN_ACK) != CPU0_CA15_L1_PDN_ACK);
    #endif //#ifndef CFG_FPGA_PLATFORM
        spm_write(SPM_CA15_L1_PWR_CON, spm_read(SPM_CA15_L1_PWR_CON) | CPU0_CA15_L1_PDN_ISO);

        spm_write(SPM_CA15_CPU0_PWR_CON, spm_read(SPM_CA15_CPU0_PWR_CON) & ~PWR_RST_B);
        spm_write(SPM_CA15_CPU0_PWR_CON, spm_read(SPM_CA15_CPU0_PWR_CON) | PWR_CLK_DIS);

        spm_write(SPM_CA15_CPU0_PWR_CON, spm_read(SPM_CA15_CPU0_PWR_CON) & ~PWR_ON);
        spm_write(SPM_CA15_CPU0_PWR_CON, spm_read(SPM_CA15_CPU0_PWR_CON) & ~PWR_ON_2ND);
    #ifndef CFG_FPGA_PLATFORM
        while (((spm_read(SPM_PWR_STATUS) & CA15_CPU0) != 0) || ((spm_read(SPM_PWR_STATUS_2ND) & CA15_CPU0) != 0));
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_mtcmos_cpu_unlock(&flags);

        if (!(spm_read(SPM_PWR_STATUS) & (CA15_CPU1 | CA15_CPU2 | CA15_CPU3)) &&
            !(spm_read(SPM_PWR_STATUS_2ND) & (CA15_CPU1 | CA15_CPU2 | CA15_CPU3)))
            spm_mtcmos_ctrl_cpusys1(state, chkWfiBeforePdn);
    }
    else /* STA_POWER_ON */
    {
        if (!(spm_read(SPM_PWR_STATUS) & CA15_CPUTOP) &&
            !(spm_read(SPM_PWR_STATUS_2ND) & CA15_CPUTOP))
            spm_mtcmos_ctrl_cpusys1(state, chkWfiBeforePdn);

        spm_mtcmos_cpu_lock(&flags);

        spm_write(SPM_CA15_CPU0_PWR_CON, spm_read(SPM_CA15_CPU0_PWR_CON) | PWR_ON);
        udelay(1);
        spm_write(SPM_CA15_CPU0_PWR_CON, spm_read(SPM_CA15_CPU0_PWR_CON) | PWR_ON_2ND);
    #ifndef CFG_FPGA_PLATFORM
        while (((spm_read(SPM_PWR_STATUS) & CA15_CPU0) != CA15_CPU0) || ((spm_read(SPM_PWR_STATUS_2ND) & CA15_CPU0) != CA15_CPU0));
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_write(SPM_CA15_CPU0_PWR_CON, spm_read(SPM_CA15_CPU0_PWR_CON) & ~PWR_ISO);

        spm_write(SPM_CA15_L1_PWR_CON, spm_read(SPM_CA15_L1_PWR_CON) & ~CPU0_CA15_L1_PDN_ISO);
        spm_write(SPM_CA15_L1_PWR_CON, spm_read(SPM_CA15_L1_PWR_CON) & ~CPU0_CA15_L1_PDN);
    #ifndef CFG_FPGA_PLATFORM
        while ((spm_read(SPM_CA15_L1_PWR_CON) & CPU0_CA15_L1_PDN_ACK) != 0);
    #endif //#ifndef CFG_FPGA_PLATFORM
        udelay(1);
        spm_write(SPM_CA15_CPU0_PWR_CON, spm_read(SPM_CA15_CPU0_PWR_CON) | SRAM_ISOINT_B);
        spm_write(SPM_CA15_CPU0_PWR_CON, spm_read(SPM_CA15_CPU0_PWR_CON) & ~SRAM_CKISO);

        spm_write(SPM_CA15_CPU0_PWR_CON, spm_read(SPM_CA15_CPU0_PWR_CON) & ~PWR_CLK_DIS);
        spm_write(SPM_CA15_CPU0_PWR_CON, spm_read(SPM_CA15_CPU0_PWR_CON) | PWR_RST_B);

        spm_mtcmos_cpu_unlock(&flags);
    }

    return 0;
}

int spm_mtcmos_ctrl_cpu5(int state, int chkWfiBeforePdn)
{
    unsigned long flags;

    /* enable register control */
    spm_write(SPM_POWERON_CONFIG_SET, (SPM_PROJECT_CODE << 16) | (1U << 0));

    if (state == STA_POWER_DOWN)
    {
        if (chkWfiBeforePdn)
            while ((spm_read(SPM_SLEEP_TIMER_STA) & CA15_CPU1_STANDBYWFI) == 0);

        spm_mtcmos_cpu_lock(&flags);

        spm_write(SPM_CA15_CPU1_PWR_CON, spm_read(SPM_CA15_CPU1_PWR_CON) | PWR_ISO);

        spm_write(SPM_CA15_CPU1_PWR_CON, spm_read(SPM_CA15_CPU1_PWR_CON) | SRAM_CKISO);
        spm_write(SPM_CA15_CPU1_PWR_CON, spm_read(SPM_CA15_CPU1_PWR_CON) & ~SRAM_ISOINT_B);
        spm_write(SPM_CA15_L1_PWR_CON, spm_read(SPM_CA15_L1_PWR_CON) | CPU1_CA15_L1_PDN);
    #ifndef CFG_FPGA_PLATFORM
        while ((spm_read(SPM_CA15_L1_PWR_CON) & CPU1_CA15_L1_PDN_ACK) != CPU1_CA15_L1_PDN_ACK);
    #endif //#ifndef CFG_FPGA_PLATFORM
        spm_write(SPM_CA15_L1_PWR_CON, spm_read(SPM_CA15_L1_PWR_CON) | CPU1_CA15_L1_PDN_ISO);

        spm_write(SPM_CA15_CPU1_PWR_CON, spm_read(SPM_CA15_CPU1_PWR_CON) & ~PWR_RST_B);
        spm_write(SPM_CA15_CPU1_PWR_CON, spm_read(SPM_CA15_CPU1_PWR_CON) | PWR_CLK_DIS);

        spm_write(SPM_CA15_CPU1_PWR_CON, spm_read(SPM_CA15_CPU1_PWR_CON) & ~PWR_ON);
        spm_write(SPM_CA15_CPU1_PWR_CON, spm_read(SPM_CA15_CPU1_PWR_CON) & ~PWR_ON_2ND);
    #ifndef CFG_FPGA_PLATFORM
        while (((spm_read(SPM_PWR_STATUS) & CA15_CPU1) != 0) || ((spm_read(SPM_PWR_STATUS_2ND) & CA15_CPU1) != 0));
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_mtcmos_cpu_unlock(&flags);

        if (!(spm_read(SPM_PWR_STATUS) & (CA15_CPU1 | CA15_CPU2 | CA15_CPU3)) &&
            !(spm_read(SPM_PWR_STATUS_2ND) & (CA15_CPU1 | CA15_CPU2 | CA15_CPU3)))
            spm_mtcmos_ctrl_cpusys1(state, chkWfiBeforePdn);
    }
    else /* STA_POWER_ON */
    {
        if (!(spm_read(SPM_PWR_STATUS) & CA15_CPUTOP) &&
            !(spm_read(SPM_PWR_STATUS_2ND) & CA15_CPUTOP))
            spm_mtcmos_ctrl_cpusys1(state, chkWfiBeforePdn);

        spm_mtcmos_cpu_lock(&flags);

        spm_write(SPM_CA15_CPU1_PWR_CON, spm_read(SPM_CA15_CPU1_PWR_CON) | PWR_ON);
        udelay(1);
        spm_write(SPM_CA15_CPU1_PWR_CON, spm_read(SPM_CA15_CPU1_PWR_CON) | PWR_ON_2ND);
    #ifndef CFG_FPGA_PLATFORM
        while (((spm_read(SPM_PWR_STATUS) & CA15_CPU1) != CA15_CPU1) || ((spm_read(SPM_PWR_STATUS_2ND) & CA15_CPU1) != CA15_CPU1));
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_write(SPM_CA15_CPU1_PWR_CON, spm_read(SPM_CA15_CPU1_PWR_CON) & ~PWR_ISO);

        spm_write(SPM_CA15_L1_PWR_CON, spm_read(SPM_CA15_L1_PWR_CON) & ~CPU1_CA15_L1_PDN_ISO);
        spm_write(SPM_CA15_L1_PWR_CON, spm_read(SPM_CA15_L1_PWR_CON) & ~CPU1_CA15_L1_PDN);
    #ifndef CFG_FPGA_PLATFORM
        while ((spm_read(SPM_CA15_L1_PWR_CON) & CPU1_CA15_L1_PDN_ACK) != 0);
    #endif //#ifndef CFG_FPGA_PLATFORM
        udelay(1);
        spm_write(SPM_CA15_CPU1_PWR_CON, spm_read(SPM_CA15_CPU1_PWR_CON) | SRAM_ISOINT_B);
        spm_write(SPM_CA15_CPU1_PWR_CON, spm_read(SPM_CA15_CPU1_PWR_CON) & ~SRAM_CKISO);

        spm_write(SPM_CA15_CPU1_PWR_CON, spm_read(SPM_CA15_CPU1_PWR_CON) & ~PWR_CLK_DIS);
        spm_write(SPM_CA15_CPU1_PWR_CON, spm_read(SPM_CA15_CPU1_PWR_CON) | PWR_RST_B);

        spm_mtcmos_cpu_unlock(&flags);
    }

    return 0;
}

int spm_mtcmos_ctrl_cpu6(int state, int chkWfiBeforePdn)
{
    unsigned long flags;

    /* enable register control */
    spm_write(SPM_POWERON_CONFIG_SET, (SPM_PROJECT_CODE << 16) | (1U << 0));

    if (state == STA_POWER_DOWN)
    {
        if (chkWfiBeforePdn)
            while ((spm_read(SPM_SLEEP_TIMER_STA) & CA15_CPU2_STANDBYWFI) == 0);

        spm_mtcmos_cpu_lock(&flags);

        spm_write(SPM_CA15_CPU2_PWR_CON, spm_read(SPM_CA15_CPU2_PWR_CON) | PWR_ISO);

        spm_write(SPM_CA15_CPU2_PWR_CON, spm_read(SPM_CA15_CPU2_PWR_CON) | SRAM_CKISO);
        spm_write(SPM_CA15_CPU2_PWR_CON, spm_read(SPM_CA15_CPU2_PWR_CON) & ~SRAM_ISOINT_B);
        spm_write(SPM_CA15_L1_PWR_CON, spm_read(SPM_CA15_L1_PWR_CON) | CPU2_CA15_L1_PDN);
    #ifndef CFG_FPGA_PLATFORM
        while ((spm_read(SPM_CA15_L1_PWR_CON) & CPU2_CA15_L1_PDN_ACK) != CPU2_CA15_L1_PDN_ACK);
    #endif //#ifndef CFG_FPGA_PLATFORM
        spm_write(SPM_CA15_L1_PWR_CON, spm_read(SPM_CA15_L1_PWR_CON) | CPU2_CA15_L1_PDN_ISO);

        spm_write(SPM_CA15_CPU2_PWR_CON, spm_read(SPM_CA15_CPU2_PWR_CON) & ~PWR_RST_B);
        spm_write(SPM_CA15_CPU2_PWR_CON, spm_read(SPM_CA15_CPU2_PWR_CON) | PWR_CLK_DIS);

        spm_write(SPM_CA15_CPU2_PWR_CON, spm_read(SPM_CA15_CPU2_PWR_CON) & ~PWR_ON);
        spm_write(SPM_CA15_CPU2_PWR_CON, spm_read(SPM_CA15_CPU2_PWR_CON) & ~PWR_ON_2ND);
    #ifndef CFG_FPGA_PLATFORM
        while (((spm_read(SPM_PWR_STATUS) & CA15_CPU2) != 0) || ((spm_read(SPM_PWR_STATUS_2ND) & CA15_CPU2) != 0));
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_mtcmos_cpu_unlock(&flags);

        if (!(spm_read(SPM_PWR_STATUS) & (CA15_CPU1 | CA15_CPU2 | CA15_CPU3)) &&
            !(spm_read(SPM_PWR_STATUS_2ND) & (CA15_CPU1 | CA15_CPU2 | CA15_CPU3)))
            spm_mtcmos_ctrl_cpusys1(state, chkWfiBeforePdn);
    }
    else /* STA_POWER_ON */
    {
        if (!(spm_read(SPM_PWR_STATUS) & CA15_CPUTOP) &&
            !(spm_read(SPM_PWR_STATUS_2ND) & CA15_CPUTOP))
            spm_mtcmos_ctrl_cpusys1(state, chkWfiBeforePdn);

        spm_mtcmos_cpu_lock(&flags);

        spm_write(SPM_CA15_CPU2_PWR_CON, spm_read(SPM_CA15_CPU2_PWR_CON) | PWR_ON);
        udelay(1);
        spm_write(SPM_CA15_CPU2_PWR_CON, spm_read(SPM_CA15_CPU2_PWR_CON) | PWR_ON_2ND);
    #ifndef CFG_FPGA_PLATFORM
        while (((spm_read(SPM_PWR_STATUS) & CA15_CPU2) != CA15_CPU2) || ((spm_read(SPM_PWR_STATUS_2ND) & CA15_CPU2) != CA15_CPU2));
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_write(SPM_CA15_CPU2_PWR_CON, spm_read(SPM_CA15_CPU2_PWR_CON) & ~PWR_ISO);

        spm_write(SPM_CA15_L1_PWR_CON, spm_read(SPM_CA15_L1_PWR_CON) & ~CPU2_CA15_L1_PDN_ISO);
        spm_write(SPM_CA15_L1_PWR_CON, spm_read(SPM_CA15_L1_PWR_CON) & ~CPU2_CA15_L1_PDN);
    #ifndef CFG_FPGA_PLATFORM
        while ((spm_read(SPM_CA15_L1_PWR_CON) & CPU2_CA15_L1_PDN_ACK) != 0);
    #endif //#ifndef CFG_FPGA_PLATFORM
        udelay(1);
        spm_write(SPM_CA15_CPU2_PWR_CON, spm_read(SPM_CA15_CPU2_PWR_CON) | SRAM_ISOINT_B);
        spm_write(SPM_CA15_CPU2_PWR_CON, spm_read(SPM_CA15_CPU2_PWR_CON) & ~SRAM_CKISO);

        spm_write(SPM_CA15_CPU2_PWR_CON, spm_read(SPM_CA15_CPU2_PWR_CON) & ~PWR_CLK_DIS);
        spm_write(SPM_CA15_CPU2_PWR_CON, spm_read(SPM_CA15_CPU2_PWR_CON) | PWR_RST_B);

        spm_mtcmos_cpu_unlock(&flags);
    }

    return 0;
}

int spm_mtcmos_ctrl_cpu7(int state, int chkWfiBeforePdn)
{
    unsigned long flags;

    /* enable register control */
    spm_write(SPM_POWERON_CONFIG_SET, (SPM_PROJECT_CODE << 16) | (1U << 0));

    if (state == STA_POWER_DOWN)
    {
        if (chkWfiBeforePdn)
            while ((spm_read(SPM_SLEEP_TIMER_STA) & CA15_CPU3_STANDBYWFI) == 0);

        spm_mtcmos_cpu_lock(&flags);

        spm_write(SPM_CA15_CPU3_PWR_CON, spm_read(SPM_CA15_CPU3_PWR_CON) | PWR_ISO);

        spm_write(SPM_CA15_CPU3_PWR_CON, spm_read(SPM_CA15_CPU3_PWR_CON) | SRAM_CKISO);
        spm_write(SPM_CA15_CPU3_PWR_CON, spm_read(SPM_CA15_CPU3_PWR_CON) & ~SRAM_ISOINT_B);
        spm_write(SPM_CA15_L1_PWR_CON, spm_read(SPM_CA15_L1_PWR_CON) | CPU3_CA15_L1_PDN);
    #ifndef CFG_FPGA_PLATFORM
        while ((spm_read(SPM_CA15_L1_PWR_CON) & CPU3_CA15_L1_PDN_ACK) != CPU3_CA15_L1_PDN_ACK);
    #endif //#ifndef CFG_FPGA_PLATFORM
        spm_write(SPM_CA15_L1_PWR_CON, spm_read(SPM_CA15_L1_PWR_CON) | CPU3_CA15_L1_PDN_ISO);

        spm_write(SPM_CA15_CPU3_PWR_CON, spm_read(SPM_CA15_CPU3_PWR_CON) & ~PWR_RST_B);
        spm_write(SPM_CA15_CPU3_PWR_CON, spm_read(SPM_CA15_CPU3_PWR_CON) | PWR_CLK_DIS);

        spm_write(SPM_CA15_CPU3_PWR_CON, spm_read(SPM_CA15_CPU3_PWR_CON) & ~PWR_ON);
        spm_write(SPM_CA15_CPU3_PWR_CON, spm_read(SPM_CA15_CPU3_PWR_CON) & ~PWR_ON_2ND);
    #ifndef CFG_FPGA_PLATFORM
        while (((spm_read(SPM_PWR_STATUS) & CA15_CPU3) != 0) || ((spm_read(SPM_PWR_STATUS_2ND) & CA15_CPU3) != 0));
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_mtcmos_cpu_unlock(&flags);

        if (!(spm_read(SPM_PWR_STATUS) & (CA15_CPU1 | CA15_CPU2 | CA15_CPU3)) &&
            !(spm_read(SPM_PWR_STATUS_2ND) & (CA15_CPU1 | CA15_CPU2 | CA15_CPU3)))
            spm_mtcmos_ctrl_cpusys1(state, chkWfiBeforePdn);
    }
    else /* STA_POWER_ON */
    {
        if (!(spm_read(SPM_PWR_STATUS) & CA15_CPUTOP) &&
            !(spm_read(SPM_PWR_STATUS_2ND) & CA15_CPUTOP))
            spm_mtcmos_ctrl_cpusys1(state, chkWfiBeforePdn);

        spm_mtcmos_cpu_lock(&flags);

        spm_write(SPM_CA15_CPU3_PWR_CON, spm_read(SPM_CA15_CPU3_PWR_CON) | PWR_ON);
        udelay(1);
        spm_write(SPM_CA15_CPU3_PWR_CON, spm_read(SPM_CA15_CPU3_PWR_CON) | PWR_ON_2ND);
    #ifndef CFG_FPGA_PLATFORM
        while (((spm_read(SPM_PWR_STATUS) & CA15_CPU3) != CA15_CPU3) || ((spm_read(SPM_PWR_STATUS_2ND) & CA15_CPU3) != CA15_CPU3));
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_write(SPM_CA15_CPU3_PWR_CON, spm_read(SPM_CA15_CPU3_PWR_CON) & ~PWR_ISO);

        spm_write(SPM_CA15_L1_PWR_CON, spm_read(SPM_CA15_L1_PWR_CON) & ~CPU3_CA15_L1_PDN_ISO);
        spm_write(SPM_CA15_L1_PWR_CON, spm_read(SPM_CA15_L1_PWR_CON) & ~CPU3_CA15_L1_PDN);
    #ifndef CFG_FPGA_PLATFORM
        while ((spm_read(SPM_CA15_L1_PWR_CON) & CPU3_CA15_L1_PDN_ACK) != 0);
    #endif //#ifndef CFG_FPGA_PLATFORM
        udelay(1);
        spm_write(SPM_CA15_CPU3_PWR_CON, spm_read(SPM_CA15_CPU3_PWR_CON) | SRAM_ISOINT_B);
        spm_write(SPM_CA15_CPU3_PWR_CON, spm_read(SPM_CA15_CPU3_PWR_CON) & ~SRAM_CKISO);

        spm_write(SPM_CA15_CPU3_PWR_CON, spm_read(SPM_CA15_CPU3_PWR_CON) & ~PWR_CLK_DIS);
        spm_write(SPM_CA15_CPU3_PWR_CON, spm_read(SPM_CA15_CPU3_PWR_CON) | PWR_RST_B);

        spm_mtcmos_cpu_unlock(&flags);
    }

    return 0;
}

#if 0 //There is no dbgsys wrapper in ca53 cpusys
int spm_mtcmos_ctrl_dbg0(int state)
{
    unsigned long flags;

    /* enable register control */
    spm_write(SPM_POWERON_CONFIG_SET, (SPM_PROJECT_CODE << 16) | (1U << 0));

    if (state == STA_POWER_DOWN)
    {
        spm_mtcmos_cpu_lock(&flags);

        spm_write(SPM_CA7_DBG_PWR_CON, spm_read(SPM_CA7_DBG_PWR_CON) | SRAM_CKISO);
        spm_write(SPM_CA7_DBG_PWR_CON, spm_read(SPM_CA7_DBG_PWR_CON) & ~SRAM_ISOINT_B);
        spm_write(SPM_CA7_DBG_PWR_CON, spm_read(SPM_CA7_DBG_PWR_CON) | PWR_ISO);
        spm_write(SPM_CA7_DBG_PWR_CON, (spm_read(SPM_CA7_DBG_PWR_CON) | PWR_CLK_DIS) & ~PWR_RST_B);
        spm_write(SPM_CA7_DBG_PWR_CON, spm_read(SPM_CA7_DBG_PWR_CON) & ~PWR_ON);
        spm_write(SPM_CA7_DBG_PWR_CON, spm_read(SPM_CA7_DBG_PWR_CON) & ~PWR_ON_2ND);
    #ifndef CFG_FPGA_PLATFORM
        while (((spm_read(SPM_PWR_STATUS) & CA7_DBG) != 0) || ((spm_read(SPM_PWR_STATUS_2ND) & CA7_DBG) != 0));
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_mtcmos_cpu_unlock(&flags);
    }
    else /* STA_POWER_ON */
    {
        spm_mtcmos_cpu_lock(&flags);

        spm_write(SPM_CA7_DBG_PWR_CON, spm_read(SPM_CA7_DBG_PWR_CON) | PWR_ON);
        udelay(1);
        spm_write(SPM_CA7_DBG_PWR_CON, spm_read(SPM_CA7_DBG_PWR_CON) | PWR_ON_2ND);
    #ifndef CFG_FPGA_PLATFORM
        while (((spm_read(SPM_PWR_STATUS) & CA7_DBG) != CA7_DBG) || ((spm_read(SPM_PWR_STATUS_2ND) & CA7_DBG) != CA7_DBG));
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_write(SPM_CA7_DBG_PWR_CON, spm_read(SPM_CA7_DBG_PWR_CON) & ~PWR_CLK_DIS);
        spm_write(SPM_CA7_DBG_PWR_CON, spm_read(SPM_CA7_DBG_PWR_CON) | SRAM_ISOINT_B);
        spm_write(SPM_CA7_DBG_PWR_CON, spm_read(SPM_CA7_DBG_PWR_CON) & ~SRAM_CKISO);
        spm_write(SPM_CA7_DBG_PWR_CON, spm_read(SPM_CA7_DBG_PWR_CON) & ~PWR_ISO);
        spm_write(SPM_CA7_DBG_PWR_CON, spm_read(SPM_CA7_DBG_PWR_CON) | PWR_RST_B);

        spm_mtcmos_cpu_unlock(&flags);
    }

    return 0;
}

int spm_mtcmos_ctrl_dbg1(int state)
{
    unsigned long flags;

    /* enable register control */
    spm_write(SPM_POWERON_CONFIG_SET, (SPM_PROJECT_CODE << 16) | (1U << 0));

    if (state == STA_POWER_DOWN)
    {
        spm_mtcmos_cpu_lock(&flags);

        spm_write(SPM_MP1_DBG_PWR_CON, spm_read(SPM_MP1_DBG_PWR_CON) | SRAM_CKISO);
        spm_write(SPM_MP1_DBG_PWR_CON, spm_read(SPM_MP1_DBG_PWR_CON) & ~SRAM_ISOINT_B);
        spm_write(SPM_MP1_DBG_PWR_CON, spm_read(SPM_MP1_DBG_PWR_CON) | PWR_ISO);
        spm_write(SPM_MP1_DBG_PWR_CON, (spm_read(SPM_MP1_DBG_PWR_CON) | PWR_CLK_DIS) & ~PWR_RST_B);
        spm_write(SPM_MP1_DBG_PWR_CON, spm_read(SPM_MP1_DBG_PWR_CON) & ~PWR_ON);
        spm_write(SPM_MP1_DBG_PWR_CON, spm_read(SPM_MP1_DBG_PWR_CON) & ~PWR_ON_2ND);
    #ifndef CFG_FPGA_PLATFORM
        while (((spm_read(SPM_PWR_STATUS) & MP1_DBG) != 0) || ((spm_read(SPM_PWR_STATUS_2ND) & MP1_DBG) != 0));
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_mtcmos_cpu_unlock(&flags);
    }
    else /* STA_POWER_ON */
    {
        spm_mtcmos_cpu_lock(&flags);

        spm_write(SPM_MP1_DBG_PWR_CON, spm_read(SPM_MP1_DBG_PWR_CON) | PWR_ON);
        udelay(1);
        spm_write(SPM_MP1_DBG_PWR_CON, spm_read(SPM_MP1_DBG_PWR_CON) | PWR_ON_2ND);
    #ifndef CFG_FPGA_PLATFORM
        while (((spm_read(SPM_PWR_STATUS) & MP1_DBG) != MP1_DBG) || ((spm_read(SPM_PWR_STATUS_2ND) & MP1_DBG) != MP1_DBG));
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_write(SPM_MP1_DBG_PWR_CON, spm_read(SPM_MP1_DBG_PWR_CON) & ~PWR_CLK_DIS);
        spm_write(SPM_MP1_DBG_PWR_CON, spm_read(SPM_MP1_DBG_PWR_CON) | SRAM_ISOINT_B);
        spm_write(SPM_MP1_DBG_PWR_CON, spm_read(SPM_MP1_DBG_PWR_CON) & ~SRAM_CKISO);
        spm_write(SPM_MP1_DBG_PWR_CON, spm_read(SPM_MP1_DBG_PWR_CON) & ~PWR_ISO);
        spm_write(SPM_MP1_DBG_PWR_CON, spm_read(SPM_MP1_DBG_PWR_CON) | PWR_RST_B);

        spm_mtcmos_cpu_unlock(&flags);
    }

    return 0;
}
#endif //There is no dbgsys wrapper in ca53 cpusys

int spm_mtcmos_ctrl_cpusys0(int state, int chkWfiBeforePdn)
{
    unsigned long flags;

    /* enable register control */
    spm_write(SPM_POWERON_CONFIG_SET, (SPM_PROJECT_CODE << 16) | (1U << 0));

    if (state == STA_POWER_DOWN)
    {
        //TODO: add per cpu power status check?

        if (chkWfiBeforePdn)
            while ((spm_read(SPM_SLEEP_TIMER_STA) & CA7_CPUTOP_STANDBYWFI) == 0);

        //XXX: no dbg0 mtcmos on k2
        //spm_mtcmos_ctrl_dbg0(state);

        //XXX: no async adb on k2
        //spm_topaxi_prot(CA7_PDN_REQ, 1);

        spm_mtcmos_cpu_lock(&flags);

        spm_write(SPM_CA7_CPUTOP_PWR_CON, spm_read(SPM_CA7_CPUTOP_PWR_CON) | PWR_ISO);

        spm_write(SPM_CA7_CPUTOP_PWR_CON, spm_read(SPM_CA7_CPUTOP_PWR_CON) | SRAM_CKISO);
    #if 1
        spm_write(SPM_CA7_CPUTOP_PWR_CON, spm_read(SPM_CA7_CPUTOP_PWR_CON) & ~SRAM_ISOINT_B);
        spm_write(SPM_CA7_CPUTOP_L2_PDN, spm_read(SPM_CA7_CPUTOP_L2_PDN) | L2_SRAM_PDN);
      #ifndef CFG_FPGA_PLATFORM
        while ((spm_read(SPM_CA7_CPUTOP_L2_PDN) & L2_SRAM_PDN_ACK) != L2_SRAM_PDN_ACK);
      #endif //#ifndef CFG_FPGA_PLATFORM
        //XXX: only in kernel
        //ndelay(1500);
        udelay(2);
    #else
        //XXX: only in kernel
        //ndelay(100);
        udelay(1);
        spm_write(SPM_CA7_CPUTOP_PWR_CON, spm_read(SPM_CA7_CPUTOP_PWR_CON) & ~SRAM_ISOINT_B);
        spm_write(SPM_CA7_CPUTOP_L2_SLEEP, spm_read(SPM_CA7_CPUTOP_L2_SLEEP) & ~L2_SRAM_SLEEP_B);
      #ifndef CFG_FPGA_PLATFORM
        while ((spm_read(SPM_CA7_CPUTOP_L2_SLEEP) & L2_SRAM_SLEEP_B_ACK) != 0);
      #endif //#ifndef CFG_FPGA_PLATFORM
    #endif

        spm_write(SPM_CA7_CPUTOP_PWR_CON, spm_read(SPM_CA7_CPUTOP_PWR_CON) & ~PWR_RST_B);
        spm_write(SPM_CA7_CPUTOP_PWR_CON, spm_read(SPM_CA7_CPUTOP_PWR_CON) | PWR_CLK_DIS);

        spm_write(SPM_CA7_CPUTOP_PWR_CON, spm_read(SPM_CA7_CPUTOP_PWR_CON) & ~PWR_ON);
        spm_write(SPM_CA7_CPUTOP_PWR_CON, spm_read(SPM_CA7_CPUTOP_PWR_CON) & ~PWR_ON_2ND);
    #ifndef CFG_FPGA_PLATFORM
        while (((spm_read(SPM_PWR_STATUS) & CA7_CPUTOP) != 0) || ((spm_read(SPM_PWR_STATUS_2ND) & CA7_CPUTOP) != 0));
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_mtcmos_cpu_unlock(&flags);
    }
    else /* STA_POWER_ON */
    {
        spm_mtcmos_cpu_lock(&flags);

        spm_write(SPM_CA7_CPUTOP_PWR_CON, spm_read(SPM_CA7_CPUTOP_PWR_CON) | PWR_ON);
        udelay(1);
        spm_write(SPM_CA7_CPUTOP_PWR_CON, spm_read(SPM_CA7_CPUTOP_PWR_CON) | PWR_ON_2ND);
    #ifndef CFG_FPGA_PLATFORM
        while (((spm_read(SPM_PWR_STATUS) & CA7_CPUTOP) != CA7_CPUTOP) || ((spm_read(SPM_PWR_STATUS_2ND) & CA7_CPUTOP) != CA7_CPUTOP));
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_write(SPM_CA7_CPUTOP_PWR_CON, spm_read(SPM_CA7_CPUTOP_PWR_CON) & ~PWR_ISO);

    #if 1
        spm_write(SPM_CA7_CPUTOP_L2_PDN, spm_read(SPM_CA7_CPUTOP_L2_PDN) & ~L2_SRAM_PDN);
      #ifndef CFG_FPGA_PLATFORM
        while ((spm_read(SPM_CA7_CPUTOP_L2_PDN) & L2_SRAM_PDN_ACK) != 0);
      #endif //#ifndef CFG_FPGA_PLATFORM
    #else
        spm_write(SPM_CA7_CPUTOP_L2_SLEEP, spm_read(SPM_CA7_CPUTOP_L2_SLEEP) | L2_SRAM_SLEEP_B);
      #ifndef CFG_FPGA_PLATFORM
        while ((spm_read(SPM_CA7_CPUTOP_L2_SLEEP) & L2_SRAM_SLEEP_B_ACK) != L2_SRAM_SLEEP_B_ACK);
      #endif //#ifndef CFG_FPGA_PLATFORM
    #endif
        //XXX: only in kernel
        //ndelay(900);
        udelay(1);
        spm_write(SPM_CA7_CPUTOP_PWR_CON, spm_read(SPM_CA7_CPUTOP_PWR_CON) | SRAM_ISOINT_B);
        //XXX: only in kernel
        //ndelay(100);
        udelay(1);
        spm_write(SPM_CA7_CPUTOP_PWR_CON, spm_read(SPM_CA7_CPUTOP_PWR_CON) & ~SRAM_CKISO);

        spm_write(SPM_CA7_CPUTOP_PWR_CON, spm_read(SPM_CA7_CPUTOP_PWR_CON) & ~PWR_CLK_DIS);
        spm_write(SPM_CA7_CPUTOP_PWR_CON, spm_read(SPM_CA7_CPUTOP_PWR_CON) | PWR_RST_B);

        spm_mtcmos_cpu_unlock(&flags);

        //XXX: no async adb on k2
        //spm_topaxi_prot(CA7_PDN_REQ, 0);

        //XXX: no dbg0 mtcmos on k2
        //spm_mtcmos_ctrl_dbg0(state);
    }

    return 0;
}

int spm_mtcmos_ctrl_cpusys1(int state, int chkWfiBeforePdn)
{
    unsigned long flags;

    /* enable register control */
    spm_write(SPM_POWERON_CONFIG_SET, (SPM_PROJECT_CODE << 16) | (1U << 0));

    if (state == STA_POWER_DOWN)
    {
        //XXX: no async adb on k2
        //spm_topaxi_prot(CA15_PDN_REQ, 1);

        if (chkWfiBeforePdn)
            //while ((spm_read(SPM_SLEEP_TIMER_STA) & CA15_CPUTOP_STANDBYWFI) == 0);
            mdelay(500);

        spm_mtcmos_cpu_lock(&flags);

        spm_write(SPM_CA15_CPUTOP_PWR_CON, spm_read(SPM_CA15_CPUTOP_PWR_CON) | PWR_ISO);

        spm_write(SPM_CA15_CPUTOP_PWR_CON, spm_read(SPM_CA15_CPUTOP_PWR_CON) | SRAM_CKISO);
        spm_write(SPM_CA15_CPUTOP_PWR_CON, spm_read(SPM_CA15_CPUTOP_PWR_CON) & ~SRAM_ISOINT_B);
    #if 1
        spm_write(SPM_CA15_L2_PWR_CON, spm_read(SPM_CA15_L2_PWR_CON) | CA15_L2_PDN);
      #ifndef CFG_FPGA_PLATFORM
        while ((spm_read(SPM_CA15_L2_PWR_CON) & CA15_L2_PDN_ACK) != CA15_L2_PDN_ACK);
      #endif //#ifndef CFG_FPGA_PLATFORM
        spm_write(SPM_CA15_L2_PWR_CON, spm_read(SPM_CA15_L2_PWR_CON) | CA15_L2_PDN_ISO);
        //XXX: only in kernel
        //ndelay(1500);
        udelay(2);
    #else
        //XXX: only in kernel
        //ndelay(100);
        udelay(1);
        spm_write(SPM_CA15_L2_PWR_CON, spm_read(SPM_CA15_L2_PWR_CON) & ~L2_SRAM_SLEEP_B);
      #ifndef CFG_FPGA_PLATFORM
        while ((spm_read(SPM_CA15_L2_PWR_CON) & L2_SRAM_SLEEP_B_ACK) != 0);
        spm_write(SPM_CA15_L2_PWR_CON, spm_read(SPM_CA15_L2_PWR_CON) | CA15_L2_SLEEPB_ISO);
      #endif //#ifndef CFG_FPGA_PLATFORM
    #endif

        spm_write(SPM_CA15_CPUTOP_PWR_CON, spm_read(SPM_CA15_CPUTOP_PWR_CON) & ~PWR_RST_B);
        spm_write(SPM_CA15_CPUTOP_PWR_CON, spm_read(SPM_CA15_CPUTOP_PWR_CON) | PWR_CLK_DIS);

        spm_write(SPM_CA15_CPUTOP_PWR_CON, spm_read(SPM_CA15_CPUTOP_PWR_CON) & ~PWR_ON);
        spm_write(SPM_CA15_CPUTOP_PWR_CON, spm_read(SPM_CA15_CPUTOP_PWR_CON) & ~PWR_ON_2ND);
    #ifndef CFG_FPGA_PLATFORM
        while (((spm_read(SPM_PWR_STATUS) & CA15_CPUTOP) != 0) || ((spm_read(SPM_PWR_STATUS_2ND) & CA15_CPUTOP) != 0));
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_mtcmos_cpu_unlock(&flags);
    }
    else /* STA_POWER_ON */
    {
        spm_mtcmos_cpu_lock(&flags);

        spm_write(SPM_CA15_CPUTOP_PWR_CON, spm_read(SPM_CA15_CPUTOP_PWR_CON) | PWR_ON);
        udelay(1);
        spm_write(SPM_CA15_CPUTOP_PWR_CON, spm_read(SPM_CA15_CPUTOP_PWR_CON) | PWR_ON_2ND);
    #ifndef CFG_FPGA_PLATFORM
        while (((spm_read(SPM_PWR_STATUS) & CA15_CPUTOP) != CA15_CPUTOP) || ((spm_read(SPM_PWR_STATUS_2ND) & CA15_CPUTOP) != CA15_CPUTOP));
    #endif //#ifndef CFG_FPGA_PLATFORM

        spm_write(SPM_CA15_CPUTOP_PWR_CON, spm_read(SPM_CA15_CPUTOP_PWR_CON) & ~PWR_ISO);

    #if 1
        spm_write(SPM_CA15_L2_PWR_CON, spm_read(SPM_CA15_L2_PWR_CON) & ~CA15_L2_PDN_ISO);
        spm_write(SPM_CA15_L2_PWR_CON, spm_read(SPM_CA15_L2_PWR_CON) & ~CA15_L2_PDN);
      #ifndef CFG_FPGA_PLATFORM
        while ((spm_read(SPM_CA15_L2_PWR_CON) & CA15_L2_PDN_ACK) != 0);
      #endif //#ifndef CFG_FPGA_PLATFORM
    #else
        spm_write(SPM_CA15_L2_PWR_CON, spm_read(SPM_CA15_L2_PWR_CON) & ~CA15_L2_SLEEPB_ISO);
        spm_write(SPM_CA15_L2_PWR_CON, spm_read(SPM_CA15_L2_PWR_CON) | L2_SRAM_SLEEP_B);
      #ifndef CFG_FPGA_PLATFORM
        while ((spm_read(SPM_CA15_L2_PWR_CON) & L2_SRAM_SLEEP_B_ACK) != L2_SRAM_SLEEP_B_ACK);
      #endif //#ifndef CFG_FPGA_PLATFORM
    #endif
        //XXX: only in kernel
        //ndelay(900);
        udelay(1);
        spm_write(SPM_CA15_CPUTOP_PWR_CON, spm_read(SPM_CA15_CPUTOP_PWR_CON) | SRAM_ISOINT_B);
        //XXX: only in kernel
        //ndelay(100);
        udelay(1);
        spm_write(SPM_CA15_CPUTOP_PWR_CON, spm_read(SPM_CA15_CPUTOP_PWR_CON) & ~SRAM_CKISO);

        spm_write(SPM_CA15_CPUTOP_PWR_CON, spm_read(SPM_CA15_CPUTOP_PWR_CON) & ~PWR_CLK_DIS);
        spm_write(SPM_CA15_CPUTOP_PWR_CON, spm_read(SPM_CA15_CPUTOP_PWR_CON) | PWR_RST_B);

        spm_mtcmos_cpu_unlock(&flags);

        //XXX: no async adb on k2
        //spm_topaxi_prot(CA15_PDN_REQ, 0);
    }

    return 0;
}

void spm_mtcmos_ctrl_cpusys1_init_1st_bring_up(int state)
{

    if (state == STA_POWER_DOWN)
    {
        spm_mtcmos_ctrl_cpu7(STA_POWER_DOWN, 0);
        spm_mtcmos_ctrl_cpu6(STA_POWER_DOWN, 0);
        spm_mtcmos_ctrl_cpu5(STA_POWER_DOWN, 0);
        spm_mtcmos_ctrl_cpu4(STA_POWER_DOWN, 0);
    }
    else /* STA_POWER_ON */
    {
        spm_mtcmos_ctrl_cpu4(STA_POWER_ON, 1);
        spm_mtcmos_ctrl_cpu5(STA_POWER_ON, 1);
        spm_mtcmos_ctrl_cpu6(STA_POWER_ON, 1);
        spm_mtcmos_ctrl_cpu7(STA_POWER_ON, 1);
        //spm_mtcmos_ctrl_dbg1(STA_POWER_ON);
    }

    //unsigned long flags;
    //
    ///* enable register control */
    //spm_write(SPM_POWERON_CONFIG_SET, (SPM_PROJECT_CODE << 16) | (1U << 0));
    //
    //spm_mtcmos_cpu_lock(&flags);
    //
    //spm_write(SPM_CA15_CPU0_PWR_CON, spm_read(SPM_CA15_CPU0_PWR_CON) & ~PWR_CLK_DIS);
    //spm_write(SPM_CA15_CPU0_PWR_CON, spm_read(SPM_CA15_CPU0_PWR_CON) | PWR_RST_B);
    //spm_write(SPM_CA15_CPU1_PWR_CON, spm_read(SPM_CA15_CPU1_PWR_CON) & ~PWR_CLK_DIS);
    //spm_write(SPM_CA15_CPU1_PWR_CON, spm_read(SPM_CA15_CPU1_PWR_CON) | PWR_RST_B);
    //spm_write(SPM_CA15_CPU2_PWR_CON, spm_read(SPM_CA15_CPU2_PWR_CON) & ~PWR_CLK_DIS);
    //spm_write(SPM_CA15_CPU2_PWR_CON, spm_read(SPM_CA15_CPU2_PWR_CON) | PWR_RST_B);
    //spm_write(SPM_CA15_CPU3_PWR_CON, spm_read(SPM_CA15_CPU3_PWR_CON) & ~PWR_CLK_DIS);
    //spm_write(SPM_CA15_CPU3_PWR_CON, spm_read(SPM_CA15_CPU3_PWR_CON) | PWR_RST_B);
    //
    //spm_mtcmos_cpu_unlock(&flags);
    //
    //spm_mtcmos_ctrl_dbg1(STA_POWER_ON);
}

bool spm_cpusys0_can_power_down(void)
{
    return !(spm_read(SPM_PWR_STATUS) & (CA15_CPU0 | CA15_CPU1 | CA15_CPU2 | CA15_CPU3 | CA15_CPUTOP | CA7_CPU1 | CA7_CPU2 | CA7_CPU3)) &&
           !(spm_read(SPM_PWR_STATUS_2ND) & (CA15_CPU0 | CA15_CPU1 | CA15_CPU2 | CA15_CPU3 | CA15_CPUTOP | CA7_CPU1 | CA7_CPU2 | CA7_CPU3));
}

bool spm_cpusys1_can_power_down(void)
{
    return !(spm_read(SPM_PWR_STATUS) & (CA7_CPU0 | CA7_CPU1 | CA7_CPU2 | CA7_CPU3 | CA7_CPUTOP | CA15_CPU1 | CA15_CPU2 | CA15_CPU3)) &&
           !(spm_read(SPM_PWR_STATUS_2ND) & (CA7_CPU0 | CA7_CPU1 | CA7_CPU2 | CA7_CPU3 | CA7_CPUTOP | CA15_CPU1 | CA15_CPU2 | CA15_CPU3));
}

int spm_topaxi_prot(int bit, int en)
{
    unsigned long flags;
    //XXX: only in kernel
    //spm_mtcmos_noncpu_lock(flags);

    if (en == 1) {
        spm_write(TOPAXI_PROT_EN, spm_read(TOPAXI_PROT_EN) | (1<<bit));
        while ((spm_read(TOPAXI_PROT_STA1) & (1<<bit)) != (1<<bit)) {
        }
    } else {
   	    spm_write(TOPAXI_PROT_EN, spm_read(TOPAXI_PROT_EN) & ~(1<<bit));
        while (spm_read(TOPAXI_PROT_STA1) & (1<<bit)) {
        }
    }

    //XXX: only in kernel
    //spm_mtcmos_noncpu_unlock(flags);

    return 0;
}

/**************************************
 * for non-CPU MTCMOS
 **************************************/
#define SPM_REG(offset)			(0x10006000 + offset)
#define POWERON_CONFIG_EN			SPM_REG(0x0000)
#define PWR_STATUS			SPM_REG(0x0180) /* correct */
#define PWR_STATUS_2ND	SPM_REG(0x0184) /* correct */
#define MD1_PWR_CON			SPM_REG(0x0320) /* correct */
#define C2K_PWR_CON			SPM_REG(0x0328) /* correct */
#define CONN_PWR_CON		SPM_REG(0x032c) /* correct */

#define MFG_ASYNC_PWR_CON			SPM_REG(0x0334) /* correct */
#define MFG_PWR_CON			SPM_REG(0x0338)
#define MFG_SRAM_CON        SPM_REG(0x033C)
#define MFG_CORE0_PWR_CON			SPM_REG(0x0340)
#define MFG_CORE1_PWR_CON			SPM_REG(0x0344)
#define MFG_CORE2_PWR_CON			SPM_REG(0x0348)
#define MFG_CORE3_PWR_CON			SPM_REG(0x034c)

#define VDE_PWR_CON			SPM_REG(0x0300)
#define VEN_PWR_CON			SPM_REG(0x0304)
#define ISP_PWR_CON			SPM_REG(0x0308)
#define DIS_PWR_CON			SPM_REG(0x030c)
#define MJC_PWR_CON			SPM_REG(0x0310)
#define AUDIO_PWR_CON			SPM_REG(0x0314) /* correct */
#define IFR_PWR_CON			SPM_REG(0x0318) /* correct */
#define DPY_PWR_CON			SPM_REG(0x031C) /* correct */
#define DPY_CH1_PWR_CON			SPM_REG(0x03B8) /* correct */
#define MD_EXT_BUCK_ISO     SPM_REG(0x0390)


#define PWR_RST_B                        (0x1 << 0)
#define PWR_ISO                          (0x1 << 1)
#define PWR_ON                           (0x1 << 2)
#define PWR_ON_2ND                       (0x1 << 3)
#define PWR_CLK_DIS                      (0x1 << 4)
#define SRAM_CKISO                       (0x1 << 5)
#define SRAM_ISOINT_B                    (0x1 << 6)
	/* Define MTCMOS Bus Protect Mask */
#define MD1_PROT_MASK                    ((0x1 << 3)|(0x1 << 4)|(0x1 << 5)|(0x1 << 7)|(0x1 << 10)|(0x1 << 16))
#define CONN_PROT_MASK                   ((0x1 << 17)|(0x1 << 18))
#define DIS_PROT_MASK                    ((0x1 << 1)|(0x1 << 2))
#define MFG_PROT_MASK                    (0x1 << 21)
#define C2K_PROT_MASK                    ((0x1 << 13)|(0x1 << 14)|(0x1 << 15))
	/* Define MTCMOS Power Status Mask */
#define MD1_PWR_STA_MASK                 (0x1 << 0)
#define CONN_PWR_STA_MASK                (0x1 << 1)
#define DPY_PWR_STA_MASK                 (0x1 << 2)
#define DIS_PWR_STA_MASK                 (0x1 << 3)
#define DPY_CH1_PWR_STA_MASK             (0x1 << 4)
#define ISP_PWR_STA_MASK                 (0x1 << 5)
#define IFR_PWR_STA_MASK                 (0x1 << 6)
#define VDE_PWR_STA_MASK                 (0x1 << 7)
#define MFG_CORE3_PWR_STA_MASK           (0x1 << 8)
#define MFG_CORE2_PWR_STA_MASK           (0x1 << 9)
#define MFG_CORE1_PWR_STA_MASK           (0x1 << 10)
#define MFG_CORE0_PWR_STA_MASK           (0x1 << 11)
#define MFG_PWR_STA_MASK                 (0x1 << 12)
#define MFG_ASYNC_PWR_STA_MASK           (0x1 << 13)
#define MJC_PWR_STA_MASK                 (0x1 << 20)
#define VEN_PWR_STA_MASK                 (0x1 << 21)
#define AUDIO_PWR_STA_MASK               (0x1 << 24)
#define C2K_PWR_STA_MASK                 (0x1 << 28)
	
	/* Define CPU SRAM Mask */				  
	
	
	/* Define Non-CPU SRAM Mask */
#define MD1_SRAM_PDN                     (0x1 << 8)
#define MD1_SRAM_PDN_ACK                 (0x0 << 12)
#define DIS_SRAM_PDN                     (0x1 << 8)
#define DIS_SRAM_PDN_ACK                 (0x1 << 12)
#define ISP_SRAM_PDN                     (0x3 << 8)
#define ISP_SRAM_PDN_ACK                 (0x3 << 12)
#define IFR_SRAM_PDN                     (0xF << 8)
#define IFR_SRAM_PDN_ACK                 (0xF << 12)
#define VDE_SRAM_PDN                     (0x1 << 8)
#define VDE_SRAM_PDN_ACK                 (0x1 << 12)
#define MFG_CORE3_SRAM_PDN               (0x1 << 8)
#define MFG_CORE3_SRAM_PDN_ACK           (0x1 << 23)
#define MFG_CORE2_SRAM_PDN               (0x1 << 8)
#define MFG_CORE2_SRAM_PDN_ACK           (0x1 << 22)
#define MFG_CORE1_SRAM_PDN               (0x1 << 8)
#define MFG_CORE1_SRAM_PDN_ACK           (0x1 << 21)
#define MFG_CORE0_SRAM_PDN               (0x1 << 8)
#define MFG_CORE0_SRAM_PDN_ACK           (0x1 << 20)
#define MFG_SRAM_PDN                     (0x3 << 0)
#define MFG_SRAM_PDN_ACK                 (0x3 << 16)
#define MJC_SRAM_PDN                     (0x1 << 8)
#define MJC_SRAM_PDN_ACK                 (0x1 << 12)
#define VEN_SRAM_PDN                     (0xF << 8)
#define VEN_SRAM_PDN_ACK                 (0xF << 12)
#define AUDIO_SRAM_PDN                   (0xF << 8)
#define AUDIO_SRAM_PDN_ACK               (0xF << 12)


int spm_mtcmos_ctrl_md1(int state)
{
	int err = 0;

	/* TINFO="enable SPM register control" */
	spm_write(POWERON_CONFIG_EN, (SPM_PROJECT_CODE << 16) | (0x1 << 0));

	if (state == STA_POWER_DOWN) {
		/* TINFO="Start to turn off MD1" */
		/* TINFO="Set bus protect" */
		spm_write(INFRA_TOPAXI_PROTECTEN, spm_read(INFRA_TOPAXI_PROTECTEN) | MD1_PROT_MASK);
		while ((spm_read(INFRA_TOPAXI_PROTECTSTA1) & MD1_PROT_MASK) != MD1_PROT_MASK) {
		}
		/* TINFO="Set SRAM_PDN = 1" */
		spm_write(MD1_PWR_CON, spm_read(MD1_PWR_CON) | MD1_SRAM_PDN);
		/* TINFO="Set PWR_ISO = 1" */
		spm_write(MD1_PWR_CON, spm_read(MD1_PWR_CON) | PWR_ISO);
		/* TINFO="Set PWR_CLK_DIS = 1" */
		spm_write(MD1_PWR_CON, spm_read(MD1_PWR_CON) | PWR_CLK_DIS);
		/* TINFO="Set PWR_RST_B = 0" */
		spm_write(MD1_PWR_CON, spm_read(MD1_PWR_CON) & ~PWR_RST_B);
		/* TINFO="Set PWR_ON = 0" */
		spm_write(MD1_PWR_CON, spm_read(MD1_PWR_CON) & ~PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 0" */
		spm_write(MD1_PWR_CON, spm_read(MD1_PWR_CON) & ~PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 0 and PWR_STATUS_2ND = 0" */
		while ((spm_read(PWR_STATUS) & MD1_PWR_STA_MASK)
		       || (spm_read(PWR_STATUS_2ND) & MD1_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="MD_EXT_BUCK_ISO[0]=1"*/
		spm_write(MD_EXT_BUCK_ISO, spm_read(MD_EXT_BUCK_ISO) | (0x1 << 0));
		/* TINFO="Finish to turn off MD1" */
	} else {    /* STA_POWER_ON */
		/* TINFO="Start to turn on MD1" */
		/* TINFO="MD_EXT_BUCK_ISO[0]=0"*/
		spm_write(MD_EXT_BUCK_ISO, spm_read(MD_EXT_BUCK_ISO) & ~(0x1 << 0));
		/* TINFO="Set PWR_ON = 1" */
		spm_write(MD1_PWR_CON, spm_read(MD1_PWR_CON) | PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 1" */
		spm_write(MD1_PWR_CON, spm_read(MD1_PWR_CON) | PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 1 and PWR_STATUS_2ND = 1" */
		while (!(spm_read(PWR_STATUS) & MD1_PWR_STA_MASK)
		       || !(spm_read(PWR_STATUS_2ND) & MD1_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Set PWR_CLK_DIS = 0" */
		spm_write(MD1_PWR_CON, spm_read(MD1_PWR_CON) & ~PWR_CLK_DIS);
		/* TINFO="Set PWR_ISO = 0" */
		spm_write(MD1_PWR_CON, spm_read(MD1_PWR_CON) & ~PWR_ISO);
		/* TINFO="Set SRAM_PDN = 0" */
		spm_write(MD1_PWR_CON, spm_read(MD1_PWR_CON) & ~(0x1 << 8));
		/* TINFO="Release bus protect" */
		spm_write(INFRA_TOPAXI_PROTECTEN, spm_read(INFRA_TOPAXI_PROTECTEN) & ~MD1_PROT_MASK);
		while (spm_read(INFRA_TOPAXI_PROTECTSTA1) & MD1_PROT_MASK) {
		}
		/* TINFO="Set PWR_RST_B = 1" */
		spm_write(MD1_PWR_CON, spm_read(MD1_PWR_CON) | PWR_RST_B);
		/* TINFO="Finish to turn on MD1" */
	}
	return err;
}

int spm_mtcmos_ctrl_conn(int state)
{
	int err = 0;

	/* TINFO="enable SPM register control" */
	spm_write(POWERON_CONFIG_EN, (SPM_PROJECT_CODE << 16) | (0x1 << 0));

	if (state == STA_POWER_DOWN) {
		/* TINFO="Start to turn off CONN" */
		/* TINFO="Set bus protect" */
		spm_write(INFRA_TOPAXI_PROTECTEN, spm_read(INFRA_TOPAXI_PROTECTEN) | CONN_PROT_MASK);
		while ((spm_read(INFRA_TOPAXI_PROTECTSTA1) & CONN_PROT_MASK) != CONN_PROT_MASK) {
		}
		/* TINFO="Set PWR_ISO = 1" */
		spm_write(CONN_PWR_CON, spm_read(CONN_PWR_CON) | PWR_ISO);
		/* TINFO="Set PWR_CLK_DIS = 1" */
		spm_write(CONN_PWR_CON, spm_read(CONN_PWR_CON) | PWR_CLK_DIS);
		/* TINFO="Set PWR_RST_B = 0" */
		spm_write(CONN_PWR_CON, spm_read(CONN_PWR_CON) & ~PWR_RST_B);
		/* TINFO="Set PWR_ON = 0" */
		spm_write(CONN_PWR_CON, spm_read(CONN_PWR_CON) & ~PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 0" */
		spm_write(CONN_PWR_CON, spm_read(CONN_PWR_CON) & ~PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 0 and PWR_STATUS_2ND = 0" */
		while ((spm_read(PWR_STATUS) & CONN_PWR_STA_MASK)
		       || (spm_read(PWR_STATUS_2ND) & CONN_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Finish to turn off CONN" */
	} else {    /* STA_POWER_ON */
		/* TINFO="Start to turn on CONN" */
		/* TINFO="Set PWR_ON = 1" */
		spm_write(CONN_PWR_CON, spm_read(CONN_PWR_CON) | PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 1" */
		spm_write(CONN_PWR_CON, spm_read(CONN_PWR_CON) | PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 1 and PWR_STATUS_2ND = 1" */
		while (!(spm_read(PWR_STATUS) & CONN_PWR_STA_MASK)
		       || !(spm_read(PWR_STATUS_2ND) & CONN_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Set PWR_CLK_DIS = 0" */
		spm_write(CONN_PWR_CON, spm_read(CONN_PWR_CON) & ~PWR_CLK_DIS);
		/* TINFO="Set PWR_ISO = 0" */
		spm_write(CONN_PWR_CON, spm_read(CONN_PWR_CON) & ~PWR_ISO);
		/* TINFO="Set PWR_RST_B = 1" */
		spm_write(CONN_PWR_CON, spm_read(CONN_PWR_CON) | PWR_RST_B);
		/* TINFO="Release bus protect" */
		spm_write(INFRA_TOPAXI_PROTECTEN, spm_read(INFRA_TOPAXI_PROTECTEN) & ~CONN_PROT_MASK);
		while (spm_read(INFRA_TOPAXI_PROTECTSTA1) & CONN_PROT_MASK) {
		}
		/* TINFO="Finish to turn on CONN" */
	}
	return err;
}

int spm_mtcmos_ctrl_dis(int state)
{
	int err = 0;

	/* TINFO="enable SPM register control" */
	spm_write(POWERON_CONFIG_EN, (SPM_PROJECT_CODE << 16) | (0x1 << 0));

	if (state == STA_POWER_DOWN) {
		/* TINFO="Start to turn off DIS" */
		/* TINFO="Set bus protect" */
		spm_write(INFRA_TOPAXI_PROTECTEN, spm_read(INFRA_TOPAXI_PROTECTEN) | DIS_PROT_MASK);
		while ((spm_read(INFRA_TOPAXI_PROTECTSTA1) & DIS_PROT_MASK) != DIS_PROT_MASK) {
		}
		/* TINFO="Set SRAM_PDN = 1" */
		spm_write(DIS_PWR_CON, spm_read(DIS_PWR_CON) | DIS_SRAM_PDN);
		/* TINFO="Wait until DIS_SRAM_PDN_ACK = 1" */
		while (!(spm_read(DIS_PWR_CON) & DIS_SRAM_PDN_ACK)) {
				/* Need hf_fmm_ck for SRAM PDN delay IP. */
		}
		/* TINFO="Set PWR_ISO = 1" */
		spm_write(DIS_PWR_CON, spm_read(DIS_PWR_CON) | PWR_ISO);
		/* TINFO="Set PWR_CLK_DIS = 1" */
		spm_write(DIS_PWR_CON, spm_read(DIS_PWR_CON) | PWR_CLK_DIS);
		/* TINFO="Set PWR_RST_B = 0" */
		spm_write(DIS_PWR_CON, spm_read(DIS_PWR_CON) & ~PWR_RST_B);
		/* TINFO="Set PWR_ON = 0" */
		spm_write(DIS_PWR_CON, spm_read(DIS_PWR_CON) & ~PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 0" */
		spm_write(DIS_PWR_CON, spm_read(DIS_PWR_CON) & ~PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 0 and PWR_STATUS_2ND = 0" */
		while ((spm_read(PWR_STATUS) & DIS_PWR_STA_MASK)
		       || (spm_read(PWR_STATUS_2ND) & DIS_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Finish to turn off DIS" */
	} else {    /* STA_POWER_ON */
		/* TINFO="Start to turn on DIS" */
		/* TINFO="Set PWR_ON = 1" */
		spm_write(DIS_PWR_CON, spm_read(DIS_PWR_CON) | PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 1" */
		spm_write(DIS_PWR_CON, spm_read(DIS_PWR_CON) | PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 1 and PWR_STATUS_2ND = 1" */
		while (!(spm_read(PWR_STATUS) & DIS_PWR_STA_MASK)
		       || !(spm_read(PWR_STATUS_2ND) & DIS_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Set PWR_CLK_DIS = 0" */
		spm_write(DIS_PWR_CON, spm_read(DIS_PWR_CON) & ~PWR_CLK_DIS);
		/* TINFO="Set PWR_ISO = 0" */
		spm_write(DIS_PWR_CON, spm_read(DIS_PWR_CON) & ~PWR_ISO);
		/* TINFO="Set PWR_RST_B = 1" */
		spm_write(DIS_PWR_CON, spm_read(DIS_PWR_CON) | PWR_RST_B);
		/* TINFO="Set SRAM_PDN = 0" */
		spm_write(DIS_PWR_CON, spm_read(DIS_PWR_CON) & ~(0x1 << 8));
		/* TINFO="Wait until DIS_SRAM_PDN_ACK = 0" */
		while (spm_read(DIS_PWR_CON) & DIS_SRAM_PDN_ACK) {
				/* Need hf_fmm_ck for SRAM PDN delay IP. */
		}
		/* TINFO="Release bus protect" */
		spm_write(INFRA_TOPAXI_PROTECTEN, spm_read(INFRA_TOPAXI_PROTECTEN) & ~DIS_PROT_MASK);
		while (spm_read(INFRA_TOPAXI_PROTECTSTA1) & DIS_PROT_MASK) {
		}
		/* TINFO="Finish to turn on DIS" */
	}
	return err;
}


int spm_mtcmos_ctrl_isp(int state)
{
	int err = 0;

	/* TINFO="enable SPM register control" */
	spm_write(POWERON_CONFIG_EN, (SPM_PROJECT_CODE << 16) | (0x1 << 0));

	if (state == STA_POWER_DOWN) {
		/* TINFO="Start to turn off ISP" */
		/* TINFO="Set SRAM_PDN = 1" */
		spm_write(ISP_PWR_CON, spm_read(ISP_PWR_CON) | ISP_SRAM_PDN);
		/* TINFO="Wait until ISP_SRAM_PDN_ACK = 1" */
		while (!(spm_read(ISP_PWR_CON) & ISP_SRAM_PDN_ACK)) {
				/* Need hf_fmm_ck for SRAM PDN delay IP. */
		}
		/* TINFO="Set PWR_ISO = 1" */
		spm_write(ISP_PWR_CON, spm_read(ISP_PWR_CON) | PWR_ISO);
		/* TINFO="Set PWR_CLK_DIS = 1" */
		spm_write(ISP_PWR_CON, spm_read(ISP_PWR_CON) | PWR_CLK_DIS);
		/* TINFO="Set PWR_RST_B = 0" */
		spm_write(ISP_PWR_CON, spm_read(ISP_PWR_CON) & ~PWR_RST_B);
		/* TINFO="Set PWR_ON = 0" */
		spm_write(ISP_PWR_CON, spm_read(ISP_PWR_CON) & ~PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 0" */
		spm_write(ISP_PWR_CON, spm_read(ISP_PWR_CON) & ~PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 0 and PWR_STATUS_2ND = 0" */
		while ((spm_read(PWR_STATUS) & ISP_PWR_STA_MASK)
		       || (spm_read(PWR_STATUS_2ND) & ISP_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Finish to turn off ISP" */
	} else {    /* STA_POWER_ON */
		/* TINFO="Start to turn on ISP" */
		/* TINFO="Set PWR_ON = 1" */
		spm_write(ISP_PWR_CON, spm_read(ISP_PWR_CON) | PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 1" */
		spm_write(ISP_PWR_CON, spm_read(ISP_PWR_CON) | PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 1 and PWR_STATUS_2ND = 1" */
		while (!(spm_read(PWR_STATUS) & ISP_PWR_STA_MASK)
		       || !(spm_read(PWR_STATUS_2ND) & ISP_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Set PWR_CLK_DIS = 0" */
		spm_write(ISP_PWR_CON, spm_read(ISP_PWR_CON) & ~PWR_CLK_DIS);
		/* TINFO="Set PWR_ISO = 0" */
		spm_write(ISP_PWR_CON, spm_read(ISP_PWR_CON) & ~PWR_ISO);
		/* TINFO="Set PWR_RST_B = 1" */
		spm_write(ISP_PWR_CON, spm_read(ISP_PWR_CON) | PWR_RST_B);
		/* TINFO="Set SRAM_PDN = 0" */
		spm_write(ISP_PWR_CON, spm_read(ISP_PWR_CON) & ~(0x1 << 8));
		spm_write(ISP_PWR_CON, spm_read(ISP_PWR_CON) & ~(0x1 << 9));
		/* TINFO="Wait until ISP_SRAM_PDN_ACK = 0" */
		while (spm_read(ISP_PWR_CON) & ISP_SRAM_PDN_ACK) {
				/* Need hf_fmm_ck for SRAM PDN delay IP. */
		}
		/* TINFO="Finish to turn on ISP" */
	}
	return err;
}


int spm_mtcmos_ctrl_vde(int state)
{
	int err = 0;

	/* TINFO="enable SPM register control" */
	spm_write(POWERON_CONFIG_EN, (SPM_PROJECT_CODE << 16) | (0x1 << 0));

	if (state == STA_POWER_DOWN) {
		/* TINFO="Start to turn off VDE" */
		/* TINFO="Set SRAM_PDN = 1" */
		spm_write(VDE_PWR_CON, spm_read(VDE_PWR_CON) | VDE_SRAM_PDN);
		/* TINFO="Wait until VDE_SRAM_PDN_ACK = 1" */
		while (!(spm_read(VDE_PWR_CON) & VDE_SRAM_PDN_ACK)) {
				/* Need hf_fmm_ck for SRAM PDN delay IP. */
		}
		/* TINFO="Set PWR_ISO = 1" */
		spm_write(VDE_PWR_CON, spm_read(VDE_PWR_CON) | PWR_ISO);
		/* TINFO="Set PWR_CLK_DIS = 1" */
		spm_write(VDE_PWR_CON, spm_read(VDE_PWR_CON) | PWR_CLK_DIS);
		/* TINFO="Set PWR_RST_B = 0" */
		spm_write(VDE_PWR_CON, spm_read(VDE_PWR_CON) & ~PWR_RST_B);
		/* TINFO="Set PWR_ON = 0" */
		spm_write(VDE_PWR_CON, spm_read(VDE_PWR_CON) & ~PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 0" */
		spm_write(VDE_PWR_CON, spm_read(VDE_PWR_CON) & ~PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 0 and PWR_STATUS_2ND = 0" */
		while ((spm_read(PWR_STATUS) & VDE_PWR_STA_MASK)
		       || (spm_read(PWR_STATUS_2ND) & VDE_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Finish to turn off VDE" */
	} else {    /* STA_POWER_ON */
		/* TINFO="Start to turn on VDE" */
		/* TINFO="Set PWR_ON = 1" */
		spm_write(VDE_PWR_CON, spm_read(VDE_PWR_CON) | PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 1" */
		spm_write(VDE_PWR_CON, spm_read(VDE_PWR_CON) | PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 1 and PWR_STATUS_2ND = 1" */
		while (!(spm_read(PWR_STATUS) & VDE_PWR_STA_MASK)
		       || !(spm_read(PWR_STATUS_2ND) & VDE_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Set PWR_CLK_DIS = 0" */
		spm_write(VDE_PWR_CON, spm_read(VDE_PWR_CON) & ~PWR_CLK_DIS);
		/* TINFO="Set PWR_ISO = 0" */
		spm_write(VDE_PWR_CON, spm_read(VDE_PWR_CON) & ~PWR_ISO);
		/* TINFO="Set PWR_RST_B = 1" */
		spm_write(VDE_PWR_CON, spm_read(VDE_PWR_CON) | PWR_RST_B);
		/* TINFO="Set SRAM_PDN = 0" */
		spm_write(VDE_PWR_CON, spm_read(VDE_PWR_CON) & ~(0x1 << 8));
		/* TINFO="Wait until VDE_SRAM_PDN_ACK = 0" */
		while (spm_read(VDE_PWR_CON) & VDE_SRAM_PDN_ACK) {
				/* Need hf_fmm_ck for SRAM PDN delay IP. */
		}
		/* TINFO="Finish to turn on VDE" */
	}
	return err;
}

int spm_mtcmos_ctrl_mfg_core3(int state)
{
	int err = 0;

	/* TINFO="enable SPM register control" */
	spm_write(POWERON_CONFIG_EN, (SPM_PROJECT_CODE << 16) | (0x1 << 0));

	if (state == STA_POWER_DOWN) {
		/* TINFO="Start to turn off MFG_CORE3" */
		/* TINFO="Set SRAM_PDN = 1" */

		spm_write(MFG_CORE3_PWR_CON, spm_read(MFG_CORE3_PWR_CON) | MFG_CORE3_SRAM_PDN);
		/* TINFO="Wait until MFG_CORE3_SRAM_PDN_ACK = 1" */
		while (!(spm_read(MFG_SRAM_CON) & MFG_CORE3_SRAM_PDN_ACK)) {
				/* n/a */
		}
		
		
		/* TINFO="Set PWR_ISO = 1" */
		spm_write(MFG_CORE3_PWR_CON, spm_read(MFG_CORE3_PWR_CON) | PWR_ISO);
		/* TINFO="Set PWR_CLK_DIS = 1" */
		spm_write(MFG_CORE3_PWR_CON, spm_read(MFG_CORE3_PWR_CON) | PWR_CLK_DIS);
		/* TINFO="Set PWR_RST_B = 0" */
		spm_write(MFG_CORE3_PWR_CON, spm_read(MFG_CORE3_PWR_CON) & ~PWR_RST_B);
		/* TINFO="Set PWR_ON = 0" */
		spm_write(MFG_CORE3_PWR_CON, spm_read(MFG_CORE3_PWR_CON) & ~PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 0" */
		spm_write(MFG_CORE3_PWR_CON, spm_read(MFG_CORE3_PWR_CON) & ~PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 0 and PWR_STATUS_2ND = 0" */
		while ((spm_read(PWR_STATUS) & MFG_CORE3_PWR_STA_MASK)
		       || (spm_read(PWR_STATUS_2ND) & MFG_CORE3_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Finish to turn off MFG_CORE3" */
	} else {    /* STA_POWER_ON */
		/* TINFO="Start to turn on MFG_CORE3" */
		/* TINFO="Set PWR_ON = 1" */
		spm_write(MFG_CORE3_PWR_CON, spm_read(MFG_CORE3_PWR_CON) | PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 1" */
		spm_write(MFG_CORE3_PWR_CON, spm_read(MFG_CORE3_PWR_CON) | PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 1 and PWR_STATUS_2ND = 1" */
		while (!(spm_read(PWR_STATUS) & MFG_CORE3_PWR_STA_MASK)
		       || !(spm_read(PWR_STATUS_2ND) & MFG_CORE3_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Set PWR_CLK_DIS = 0" */
		spm_write(MFG_CORE3_PWR_CON, spm_read(MFG_CORE3_PWR_CON) & ~PWR_CLK_DIS);
		/* TINFO="Set PWR_ISO = 0" */
		spm_write(MFG_CORE3_PWR_CON, spm_read(MFG_CORE3_PWR_CON) & ~PWR_ISO);
		/* TINFO="Set PWR_RST_B = 1" */
		spm_write(MFG_CORE3_PWR_CON, spm_read(MFG_CORE3_PWR_CON) | PWR_RST_B);
		/* TINFO="Set SRAM_PDN = 0" */
		spm_write(MFG_CORE3_PWR_CON, spm_read(MFG_CORE3_PWR_CON) & ~(0x1 << 8));
		/* TINFO="Wait until MFG_CORE3_SRAM_PDN_ACK = 0" */
		while (spm_read(MFG_SRAM_CON) & MFG_CORE3_SRAM_PDN_ACK) {
				/* n/a */
		}
		/* TINFO="Finish to turn on MFG_CORE3" */
	}
	return err;
}

int spm_mtcmos_ctrl_mfg_core2(int state)
{
	int err = 0;

	/* TINFO="enable SPM register control" */
	spm_write(POWERON_CONFIG_EN, (SPM_PROJECT_CODE << 16) | (0x1 << 0));

	if (state == STA_POWER_DOWN) {
		/* TINFO="Start to turn off MFG_CORE2" */
		/* TINFO="Set SRAM_PDN = 1" */
		spm_write(MFG_CORE2_PWR_CON, spm_read(MFG_CORE2_PWR_CON) | MFG_CORE2_SRAM_PDN);
		/* TINFO="Wait until MFG_CORE2_SRAM_PDN_ACK = 1" */
		while (!(spm_read(MFG_SRAM_CON) & MFG_CORE2_SRAM_PDN_ACK)) {
				/* n/a */
		}
		/* TINFO="Set PWR_ISO = 1" */
		spm_write(MFG_CORE2_PWR_CON, spm_read(MFG_CORE2_PWR_CON) | PWR_ISO);
		/* TINFO="Set PWR_CLK_DIS = 1" */
		spm_write(MFG_CORE2_PWR_CON, spm_read(MFG_CORE2_PWR_CON) | PWR_CLK_DIS);
		/* TINFO="Set PWR_RST_B = 0" */
		spm_write(MFG_CORE2_PWR_CON, spm_read(MFG_CORE2_PWR_CON) & ~PWR_RST_B);
		/* TINFO="Set PWR_ON = 0" */
		spm_write(MFG_CORE2_PWR_CON, spm_read(MFG_CORE2_PWR_CON) & ~PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 0" */
		spm_write(MFG_CORE2_PWR_CON, spm_read(MFG_CORE2_PWR_CON) & ~PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 0 and PWR_STATUS_2ND = 0" */
		while ((spm_read(PWR_STATUS) & MFG_CORE2_PWR_STA_MASK)
		       || (spm_read(PWR_STATUS_2ND) & MFG_CORE2_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Finish to turn off MFG_CORE2" */
	} else {    /* STA_POWER_ON */
		/* TINFO="Start to turn on MFG_CORE2" */
		/* TINFO="Set PWR_ON = 1" */
		spm_write(MFG_CORE2_PWR_CON, spm_read(MFG_CORE2_PWR_CON) | PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 1" */
		spm_write(MFG_CORE2_PWR_CON, spm_read(MFG_CORE2_PWR_CON) | PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 1 and PWR_STATUS_2ND = 1" */
		while (!(spm_read(PWR_STATUS) & MFG_CORE2_PWR_STA_MASK)
		       || !(spm_read(PWR_STATUS_2ND) & MFG_CORE2_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Set PWR_CLK_DIS = 0" */
		spm_write(MFG_CORE2_PWR_CON, spm_read(MFG_CORE2_PWR_CON) & ~PWR_CLK_DIS);
		/* TINFO="Set PWR_ISO = 0" */
		spm_write(MFG_CORE2_PWR_CON, spm_read(MFG_CORE2_PWR_CON) & ~PWR_ISO);
		/* TINFO="Set PWR_RST_B = 1" */
		spm_write(MFG_CORE2_PWR_CON, spm_read(MFG_CORE2_PWR_CON) | PWR_RST_B);
		/* TINFO="Set SRAM_PDN = 0" */
		spm_write(MFG_CORE2_PWR_CON, spm_read(MFG_CORE2_PWR_CON) & ~(0x1 << 8));
		/* TINFO="Wait until MFG_CORE2_SRAM_PDN_ACK = 0" */
		while (spm_read(MFG_SRAM_CON) & MFG_CORE2_SRAM_PDN_ACK) {
				/* n/a */
		}
		/* TINFO="Finish to turn on MFG_CORE2" */
	}
	return err;
}

int spm_mtcmos_ctrl_mfg_core1(int state)
{
	int err = 0;

	/* TINFO="enable SPM register control" */
	spm_write(POWERON_CONFIG_EN, (SPM_PROJECT_CODE << 16) | (0x1 << 0));

	if (state == STA_POWER_DOWN) {
		/* TINFO="Start to turn off MFG_CORE1" */
		/* TINFO="Set SRAM_PDN = 1" */
		spm_write(MFG_CORE1_PWR_CON, spm_read(MFG_CORE1_PWR_CON) | MFG_CORE1_SRAM_PDN);
		/* TINFO="Wait until MFG_CORE1_SRAM_PDN_ACK = 1" */
		while (!(spm_read(MFG_SRAM_CON) & MFG_CORE1_SRAM_PDN_ACK)) {
				/* n/a */
		}
		/* TINFO="Set PWR_ISO = 1" */
		spm_write(MFG_CORE1_PWR_CON, spm_read(MFG_CORE1_PWR_CON) | PWR_ISO);
		/* TINFO="Set PWR_CLK_DIS = 1" */
		spm_write(MFG_CORE1_PWR_CON, spm_read(MFG_CORE1_PWR_CON) | PWR_CLK_DIS);
		/* TINFO="Set PWR_RST_B = 0" */
		spm_write(MFG_CORE1_PWR_CON, spm_read(MFG_CORE1_PWR_CON) & ~PWR_RST_B);
		/* TINFO="Set PWR_ON = 0" */
		spm_write(MFG_CORE1_PWR_CON, spm_read(MFG_CORE1_PWR_CON) & ~PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 0" */
		spm_write(MFG_CORE1_PWR_CON, spm_read(MFG_CORE1_PWR_CON) & ~PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 0 and PWR_STATUS_2ND = 0" */
		while ((spm_read(PWR_STATUS) & MFG_CORE1_PWR_STA_MASK)
		       || (spm_read(PWR_STATUS_2ND) & MFG_CORE1_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Finish to turn off MFG_CORE1" */
	} else {    /* STA_POWER_ON */
		/* TINFO="Start to turn on MFG_CORE1" */
		/* TINFO="Set PWR_ON = 1" */
		spm_write(MFG_CORE1_PWR_CON, spm_read(MFG_CORE1_PWR_CON) | PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 1" */
		spm_write(MFG_CORE1_PWR_CON, spm_read(MFG_CORE1_PWR_CON) | PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 1 and PWR_STATUS_2ND = 1" */
		while (!(spm_read(PWR_STATUS) & MFG_CORE1_PWR_STA_MASK)
		       || !(spm_read(PWR_STATUS_2ND) & MFG_CORE1_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Set PWR_CLK_DIS = 0" */
		spm_write(MFG_CORE1_PWR_CON, spm_read(MFG_CORE1_PWR_CON) & ~PWR_CLK_DIS);
		/* TINFO="Set PWR_ISO = 0" */
		spm_write(MFG_CORE1_PWR_CON, spm_read(MFG_CORE1_PWR_CON) & ~PWR_ISO);
		/* TINFO="Set PWR_RST_B = 1" */
		spm_write(MFG_CORE1_PWR_CON, spm_read(MFG_CORE1_PWR_CON) | PWR_RST_B);
		/* TINFO="Set SRAM_PDN = 0" */
		spm_write(MFG_CORE1_PWR_CON, spm_read(MFG_CORE1_PWR_CON) & ~(0x1 << 8));
		/* TINFO="Wait until MFG_CORE1_SRAM_PDN_ACK = 0" */
		while (spm_read(MFG_SRAM_CON) & MFG_CORE1_SRAM_PDN_ACK) {
				/* n/a */
		}
		/* TINFO="Finish to turn on MFG_CORE1" */
	}
	return err;
}

int spm_mtcmos_ctrl_mfg_core0(int state)
{
	int err = 0;

	/* TINFO="enable SPM register control" */
	spm_write(POWERON_CONFIG_EN, (SPM_PROJECT_CODE << 16) | (0x1 << 0));

	if (state == STA_POWER_DOWN) {
		/* TINFO="Start to turn off MFG_CORE0" */
		/* TINFO="Set SRAM_PDN = 1" */
		spm_write(MFG_CORE0_PWR_CON, spm_read(MFG_CORE0_PWR_CON) | MFG_CORE0_SRAM_PDN);
		/* TINFO="Wait until MFG_CORE0_SRAM_PDN_ACK = 1" */
		while (!(spm_read(MFG_SRAM_CON) & MFG_CORE0_SRAM_PDN_ACK)) {
				/* n/a */
		}
		/* TINFO="Set PWR_ISO = 1" */
		spm_write(MFG_CORE0_PWR_CON, spm_read(MFG_CORE0_PWR_CON) | PWR_ISO);
		/* TINFO="Set PWR_CLK_DIS = 1" */
		spm_write(MFG_CORE0_PWR_CON, spm_read(MFG_CORE0_PWR_CON) | PWR_CLK_DIS);
		/* TINFO="Set PWR_RST_B = 0" */
		spm_write(MFG_CORE0_PWR_CON, spm_read(MFG_CORE0_PWR_CON) & ~PWR_RST_B);
		/* TINFO="Set PWR_ON = 0" */
		spm_write(MFG_CORE0_PWR_CON, spm_read(MFG_CORE0_PWR_CON) & ~PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 0" */
		spm_write(MFG_CORE0_PWR_CON, spm_read(MFG_CORE0_PWR_CON) & ~PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 0 and PWR_STATUS_2ND = 0" */
		while ((spm_read(PWR_STATUS) & MFG_CORE0_PWR_STA_MASK)
		       || (spm_read(PWR_STATUS_2ND) & MFG_CORE0_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Finish to turn off MFG_CORE0" */
	} else {    /* STA_POWER_ON */
		/* TINFO="Start to turn on MFG_CORE0" */
		/* TINFO="Set PWR_ON = 1" */
		spm_write(MFG_CORE0_PWR_CON, spm_read(MFG_CORE0_PWR_CON) | PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 1" */
		spm_write(MFG_CORE0_PWR_CON, spm_read(MFG_CORE0_PWR_CON) | PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 1 and PWR_STATUS_2ND = 1" */
		while (!(spm_read(PWR_STATUS) & MFG_CORE0_PWR_STA_MASK)
		       || !(spm_read(PWR_STATUS_2ND) & MFG_CORE0_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Set PWR_CLK_DIS = 0" */
		spm_write(MFG_CORE0_PWR_CON, spm_read(MFG_CORE0_PWR_CON) & ~PWR_CLK_DIS);
		/* TINFO="Set PWR_ISO = 0" */
		spm_write(MFG_CORE0_PWR_CON, spm_read(MFG_CORE0_PWR_CON) & ~PWR_ISO);
		/* TINFO="Set PWR_RST_B = 1" */
		spm_write(MFG_CORE0_PWR_CON, spm_read(MFG_CORE0_PWR_CON) | PWR_RST_B);
		/* TINFO="Set SRAM_PDN = 0" */
		spm_write(MFG_CORE0_PWR_CON, spm_read(MFG_CORE0_PWR_CON) & ~(0x1 << 8));
		/* TINFO="Wait until MFG_CORE0_SRAM_PDN_ACK = 0" */
		while (spm_read(MFG_SRAM_CON) & MFG_CORE0_SRAM_PDN_ACK) {
				/* n/a */
		}
		/* TINFO="Finish to turn on MFG_CORE0" */
	}
	return err;
}

int spm_mtcmos_ctrl_mfg(int state)
{
	int err = 0;

	/* TINFO="enable SPM register control" */
	spm_write(POWERON_CONFIG_EN, (SPM_PROJECT_CODE << 16) | (0x1 << 0));

	if (state == STA_POWER_DOWN) {
		/* TINFO="Start to turn off MFG" */
		/* TINFO="Set bus protect" */
		spm_write(INFRA_TOPAXI_PROTECTEN, spm_read(INFRA_TOPAXI_PROTECTEN) | MFG_PROT_MASK);
		while ((spm_read(INFRA_TOPAXI_PROTECTSTA1) & MFG_PROT_MASK) != MFG_PROT_MASK) {
		}

		/* TINFO="Set SRAM_PDN = 1" */
		spm_write(MFG_SRAM_CON, spm_read(MFG_SRAM_CON) | MFG_SRAM_PDN);
		/* TINFO="Wait until MFG_SRAM_PDN_ACK = 1" */
		while (!(spm_read(MFG_SRAM_CON) & MFG_SRAM_PDN_ACK)) {
				/* Need f_fmfg_core_ck for SRAM PDN delay IP. */
		}
		
		
		
		
		/* TINFO="Set PWR_ISO = 1" */
		spm_write(MFG_PWR_CON, spm_read(MFG_PWR_CON) | PWR_ISO);
		/* TINFO="Set PWR_CLK_DIS = 1" */
		spm_write(MFG_PWR_CON, spm_read(MFG_PWR_CON) | PWR_CLK_DIS);
		/* TINFO="Set PWR_RST_B = 0" */
		spm_write(MFG_PWR_CON, spm_read(MFG_PWR_CON) & ~PWR_RST_B);
		/* TINFO="Set PWR_ON = 0" */
		spm_write(MFG_PWR_CON, spm_read(MFG_PWR_CON) & ~PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 0" */
		spm_write(MFG_PWR_CON, spm_read(MFG_PWR_CON) & ~PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 0 and PWR_STATUS_2ND = 0" */
		while ((spm_read(PWR_STATUS) & MFG_PWR_STA_MASK)
		       || (spm_read(PWR_STATUS_2ND) & MFG_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Finish to turn off MFG" */
	} else {    /* STA_POWER_ON */
		/* TINFO="Start to turn on MFG" */
		/* TINFO="Set PWR_ON = 1" */
		spm_write(MFG_PWR_CON, spm_read(MFG_PWR_CON) | PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 1" */
		spm_write(MFG_PWR_CON, spm_read(MFG_PWR_CON) | PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 1 and PWR_STATUS_2ND = 1" */
		while (!(spm_read(PWR_STATUS) & MFG_PWR_STA_MASK)
		       || !(spm_read(PWR_STATUS_2ND) & MFG_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Set PWR_CLK_DIS = 0" */
		spm_write(MFG_PWR_CON, spm_read(MFG_PWR_CON) & ~PWR_CLK_DIS);
		/* TINFO="Set PWR_ISO = 0" */
		spm_write(MFG_PWR_CON, spm_read(MFG_PWR_CON) & ~PWR_ISO);
		/* TINFO="Set PWR_RST_B = 1" */
		spm_write(MFG_PWR_CON, spm_read(MFG_PWR_CON) | PWR_RST_B);
		/* TINFO="Set SRAM_PDN = 0" */


		spm_write(MFG_SRAM_CON, spm_read(MFG_SRAM_CON) & ~(0x3 << 0));
		/* TINFO="Wait until MFG_SRAM_PDN_ACK = 0" */
		while (spm_read(MFG_SRAM_CON) & MFG_SRAM_PDN_ACK) {
				/* Need f_fmfg_core_ck for SRAM PDN delay IP. */
		}



		/* TINFO="Release bus protect" */
		spm_write(INFRA_TOPAXI_PROTECTEN, spm_read(INFRA_TOPAXI_PROTECTEN) & ~MFG_PROT_MASK);
		while (spm_read(INFRA_TOPAXI_PROTECTSTA1) & MFG_PROT_MASK) {
		}
		/* TINFO="Finish to turn on MFG" */
	}
	return err;
}

int spm_mtcmos_ctrl_mfg_async(int state)
{
	int err = 0;

	/* TINFO="enable SPM register control" */
	spm_write(POWERON_CONFIG_EN, (SPM_PROJECT_CODE << 16) | (0x1 << 0));

	if (state == STA_POWER_DOWN) {
		/* TINFO="Start to turn off MFG_ASYNC" */
		/* TINFO="Set PWR_ISO = 1" */
		spm_write(MFG_ASYNC_PWR_CON, spm_read(MFG_ASYNC_PWR_CON) | PWR_ISO);
		/* TINFO="Set PWR_CLK_DIS = 1" */
		spm_write(MFG_ASYNC_PWR_CON, spm_read(MFG_ASYNC_PWR_CON) | PWR_CLK_DIS);
		/* TINFO="Set PWR_RST_B = 0" */
		spm_write(MFG_ASYNC_PWR_CON, spm_read(MFG_ASYNC_PWR_CON) & ~PWR_RST_B);
		/* TINFO="Set PWR_ON = 0" */
		spm_write(MFG_ASYNC_PWR_CON, spm_read(MFG_ASYNC_PWR_CON) & ~PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 0" */
		spm_write(MFG_ASYNC_PWR_CON, spm_read(MFG_ASYNC_PWR_CON) & ~PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 0 and PWR_STATUS_2ND = 0" */
		while ((spm_read(PWR_STATUS) & MFG_ASYNC_PWR_STA_MASK)
		       || (spm_read(PWR_STATUS_2ND) & MFG_ASYNC_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Finish to turn off MFG_ASYNC" */
	} else {    /* STA_POWER_ON */
		/* TINFO="Start to turn on MFG_ASYNC" */
		/* TINFO="Set PWR_ON = 1" */
		spm_write(MFG_ASYNC_PWR_CON, spm_read(MFG_ASYNC_PWR_CON) | PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 1" */
		spm_write(MFG_ASYNC_PWR_CON, spm_read(MFG_ASYNC_PWR_CON) | PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 1 and PWR_STATUS_2ND = 1" */
		while (!(spm_read(PWR_STATUS) & MFG_ASYNC_PWR_STA_MASK)
		       || !(spm_read(PWR_STATUS_2ND) & MFG_ASYNC_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Set PWR_CLK_DIS = 0" */
		spm_write(MFG_ASYNC_PWR_CON, spm_read(MFG_ASYNC_PWR_CON) & ~PWR_CLK_DIS);
		/* TINFO="Set PWR_ISO = 0" */
		spm_write(MFG_ASYNC_PWR_CON, spm_read(MFG_ASYNC_PWR_CON) & ~PWR_ISO);
		/* TINFO="Set PWR_RST_B = 1" */
		spm_write(MFG_ASYNC_PWR_CON, spm_read(MFG_ASYNC_PWR_CON) | PWR_RST_B);
		/* TINFO="Finish to turn on MFG_ASYNC" */
	}
	return err;
}

int spm_mtcmos_ctrl_mjc(int state)
{
	int err = 0;

	/* TINFO="enable SPM register control" */
	spm_write(POWERON_CONFIG_EN, (SPM_PROJECT_CODE << 16) | (0x1 << 0));

	if (state == STA_POWER_DOWN) {
		/* TINFO="Start to turn off MJC" */
		/* TINFO="Set SRAM_PDN = 1" */
		spm_write(MJC_PWR_CON, spm_read(MJC_PWR_CON) | MJC_SRAM_PDN);
		/* TINFO="Wait until MJC_SRAM_PDN_ACK = 1" */
		while (!(spm_read(MJC_PWR_CON) & MJC_SRAM_PDN_ACK)) {
				/* Need hf_fmm_ck for SRAM PDN delay IP. */
		}
		/* TINFO="Set PWR_ISO = 1" */
		spm_write(MJC_PWR_CON, spm_read(MJC_PWR_CON) | PWR_ISO);
		/* TINFO="Set PWR_CLK_DIS = 1" */
		spm_write(MJC_PWR_CON, spm_read(MJC_PWR_CON) | PWR_CLK_DIS);
		/* TINFO="Set PWR_RST_B = 0" */
		spm_write(MJC_PWR_CON, spm_read(MJC_PWR_CON) & ~PWR_RST_B);
		/* TINFO="Set PWR_ON = 0" */
		spm_write(MJC_PWR_CON, spm_read(MJC_PWR_CON) & ~PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 0" */
		spm_write(MJC_PWR_CON, spm_read(MJC_PWR_CON) & ~PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 0 and PWR_STATUS_2ND = 0" */
		while ((spm_read(PWR_STATUS) & MJC_PWR_STA_MASK)
		       || (spm_read(PWR_STATUS_2ND) & MJC_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Finish to turn off MJC" */
	} else {    /* STA_POWER_ON */
		/* TINFO="Start to turn on MJC" */
		/* TINFO="Set PWR_ON = 1" */
		spm_write(MJC_PWR_CON, spm_read(MJC_PWR_CON) | PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 1" */
		spm_write(MJC_PWR_CON, spm_read(MJC_PWR_CON) | PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 1 and PWR_STATUS_2ND = 1" */
		while (!(spm_read(PWR_STATUS) & MJC_PWR_STA_MASK)
		       || !(spm_read(PWR_STATUS_2ND) & MJC_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Set PWR_CLK_DIS = 0" */
		spm_write(MJC_PWR_CON, spm_read(MJC_PWR_CON) & ~PWR_CLK_DIS);
		/* TINFO="Set PWR_ISO = 0" */
		spm_write(MJC_PWR_CON, spm_read(MJC_PWR_CON) & ~PWR_ISO);
		/* TINFO="Set PWR_RST_B = 1" */
		spm_write(MJC_PWR_CON, spm_read(MJC_PWR_CON) | PWR_RST_B);
		/* TINFO="Set SRAM_PDN = 0" */
		spm_write(MJC_PWR_CON, spm_read(MJC_PWR_CON) & ~(0x1 << 8));
		/* TINFO="Wait until MJC_SRAM_PDN_ACK = 0" */
		while (spm_read(MJC_PWR_CON) & MJC_SRAM_PDN_ACK) {
				/* Need hf_fmm_ck for SRAM PDN delay IP. */
		}
		/* TINFO="Finish to turn on MJC" */
	}
	return err;
}

int spm_mtcmos_ctrl_ven(int state)
{
	int err = 0;

	/* TINFO="enable SPM register control" */
	spm_write(POWERON_CONFIG_EN, (SPM_PROJECT_CODE << 16) | (0x1 << 0));

	if (state == STA_POWER_DOWN) {
		/* TINFO="Start to turn off VEN" */
		/* TINFO="Set SRAM_PDN = 1" */
		spm_write(VEN_PWR_CON, spm_read(VEN_PWR_CON) | VEN_SRAM_PDN);
		/* TINFO="Wait until VEN_SRAM_PDN_ACK = 1" */
		while (!(spm_read(VEN_PWR_CON) & VEN_SRAM_PDN_ACK)) {
				/* Need hf_fmm_ck for SRAM PDN delay IP. */
		}
		/* TINFO="Set PWR_ISO = 1" */
		spm_write(VEN_PWR_CON, spm_read(VEN_PWR_CON) | PWR_ISO);
		/* TINFO="Set PWR_CLK_DIS = 1" */
		spm_write(VEN_PWR_CON, spm_read(VEN_PWR_CON) | PWR_CLK_DIS);
		/* TINFO="Set PWR_RST_B = 0" */
		spm_write(VEN_PWR_CON, spm_read(VEN_PWR_CON) & ~PWR_RST_B);
		/* TINFO="Set PWR_ON = 0" */
		spm_write(VEN_PWR_CON, spm_read(VEN_PWR_CON) & ~PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 0" */
		spm_write(VEN_PWR_CON, spm_read(VEN_PWR_CON) & ~PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 0 and PWR_STATUS_2ND = 0" */
		while ((spm_read(PWR_STATUS) & VEN_PWR_STA_MASK)
		       || (spm_read(PWR_STATUS_2ND) & VEN_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Finish to turn off VEN" */
	} else {    /* STA_POWER_ON */
		/* TINFO="Start to turn on VEN" */
		/* TINFO="Set PWR_ON = 1" */
		spm_write(VEN_PWR_CON, spm_read(VEN_PWR_CON) | PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 1" */
		spm_write(VEN_PWR_CON, spm_read(VEN_PWR_CON) | PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 1 and PWR_STATUS_2ND = 1" */
		while (!(spm_read(PWR_STATUS) & VEN_PWR_STA_MASK)
		       || !(spm_read(PWR_STATUS_2ND) & VEN_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Set PWR_CLK_DIS = 0" */
		spm_write(VEN_PWR_CON, spm_read(VEN_PWR_CON) & ~PWR_CLK_DIS);
		/* TINFO="Set PWR_ISO = 0" */
		spm_write(VEN_PWR_CON, spm_read(VEN_PWR_CON) & ~PWR_ISO);
		/* TINFO="Set PWR_RST_B = 1" */
		spm_write(VEN_PWR_CON, spm_read(VEN_PWR_CON) | PWR_RST_B);
		/* TINFO="Set SRAM_PDN = 0" */
		spm_write(VEN_PWR_CON, spm_read(VEN_PWR_CON) & ~(0x1 << 8));
		spm_write(VEN_PWR_CON, spm_read(VEN_PWR_CON) & ~(0x1 << 9));
		spm_write(VEN_PWR_CON, spm_read(VEN_PWR_CON) & ~(0x1 << 10));
		spm_write(VEN_PWR_CON, spm_read(VEN_PWR_CON) & ~(0x1 << 11));
		/* TINFO="Wait until VEN_SRAM_PDN_ACK = 0" */
		while (spm_read(VEN_PWR_CON) & VEN_SRAM_PDN_ACK) {
				/* Need hf_fmm_ck for SRAM PDN delay IP. */
		}
		/* TINFO="Finish to turn on VEN" */
	}
	return err;
}

int spm_mtcmos_ctrl_audio(int state)
{
	int err = 0;

	/* TINFO="enable SPM register control" */
	spm_write(POWERON_CONFIG_EN, (SPM_PROJECT_CODE << 16) | (0x1 << 0));

	if (state == STA_POWER_DOWN) {
		/* TINFO="Start to turn off AUDIO" */
		/* TINFO="Set SRAM_PDN = 1" */
		spm_write(AUDIO_PWR_CON, spm_read(AUDIO_PWR_CON) | AUDIO_SRAM_PDN);
		/* TINFO="Wait until AUDIO_SRAM_PDN_ACK = 1" */
		while (!(spm_read(AUDIO_PWR_CON) & AUDIO_SRAM_PDN_ACK)) {
				/* Need f_f26M_aud_ck for SRAM PDN delay IP. */
		}
		/* TINFO="Set PWR_ISO = 1" */
		spm_write(AUDIO_PWR_CON, spm_read(AUDIO_PWR_CON) | PWR_ISO);
		/* TINFO="Set PWR_CLK_DIS = 1" */
		spm_write(AUDIO_PWR_CON, spm_read(AUDIO_PWR_CON) | PWR_CLK_DIS);
		/* TINFO="Set PWR_RST_B = 0" */
		spm_write(AUDIO_PWR_CON, spm_read(AUDIO_PWR_CON) & ~PWR_RST_B);
		/* TINFO="Set PWR_ON = 0" */
		spm_write(AUDIO_PWR_CON, spm_read(AUDIO_PWR_CON) & ~PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 0" */
		spm_write(AUDIO_PWR_CON, spm_read(AUDIO_PWR_CON) & ~PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 0 and PWR_STATUS_2ND = 0" */
		while ((spm_read(PWR_STATUS) & AUDIO_PWR_STA_MASK)
		       || (spm_read(PWR_STATUS_2ND) & AUDIO_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Finish to turn off AUDIO" */
	} else {    /* STA_POWER_ON */
		/* TINFO="Start to turn on AUDIO" */
		/* TINFO="Set PWR_ON = 1" */
		spm_write(AUDIO_PWR_CON, spm_read(AUDIO_PWR_CON) | PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 1" */
		spm_write(AUDIO_PWR_CON, spm_read(AUDIO_PWR_CON) | PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 1 and PWR_STATUS_2ND = 1" */
		while (!(spm_read(PWR_STATUS) & AUDIO_PWR_STA_MASK)
		       || !(spm_read(PWR_STATUS_2ND) & AUDIO_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Set PWR_CLK_DIS = 0" */
		spm_write(AUDIO_PWR_CON, spm_read(AUDIO_PWR_CON) & ~PWR_CLK_DIS);
		/* TINFO="Set PWR_ISO = 0" */
		spm_write(AUDIO_PWR_CON, spm_read(AUDIO_PWR_CON) & ~PWR_ISO);
		/* TINFO="Set PWR_RST_B = 1" */
		spm_write(AUDIO_PWR_CON, spm_read(AUDIO_PWR_CON) | PWR_RST_B);
		/* TINFO="Set SRAM_PDN = 0" */
		spm_write(AUDIO_PWR_CON, spm_read(AUDIO_PWR_CON) & ~(0x1 << 8));
		spm_write(AUDIO_PWR_CON, spm_read(AUDIO_PWR_CON) & ~(0x1 << 9));
		spm_write(AUDIO_PWR_CON, spm_read(AUDIO_PWR_CON) & ~(0x1 << 10));
		spm_write(AUDIO_PWR_CON, spm_read(AUDIO_PWR_CON) & ~(0x1 << 11));
		/* TINFO="Wait until AUDIO_SRAM_PDN_ACK = 0" */
		while (spm_read(AUDIO_PWR_CON) & AUDIO_SRAM_PDN_ACK) {
				/* Need f_f26M_aud_ck for SRAM PDN delay IP. */
		}
		/* TINFO="Finish to turn on AUDIO" */
	}
	return err;
}

int spm_mtcmos_ctrl_c2k(int state)
{
	int err = 0;

	/* TINFO="enable SPM register control" */
	spm_write(POWERON_CONFIG_EN, (SPM_PROJECT_CODE << 16) | (0x1 << 0));

	if (state == STA_POWER_DOWN) {
		/* TINFO="Start to turn off C2K" */
		/* TINFO="Set PWR_ISO = 1" */
		spm_write(INFRA_TOPAXI_PROTECTEN, spm_read(INFRA_TOPAXI_PROTECTEN) | C2K_PROT_MASK);
		while ((spm_read(INFRA_TOPAXI_PROTECTSTA1) & C2K_PROT_MASK) != C2K_PROT_MASK) {
		}

		spm_write(C2K_PWR_CON, spm_read(C2K_PWR_CON) | PWR_ISO);
		/* TINFO="Set PWR_CLK_DIS = 1" */
		spm_write(C2K_PWR_CON, spm_read(C2K_PWR_CON) | PWR_CLK_DIS);
		/* TINFO="Set PWR_RST_B = 0" */
		spm_write(C2K_PWR_CON, spm_read(C2K_PWR_CON) & ~PWR_RST_B);
		/* TINFO="Set PWR_ON = 0" */
		spm_write(C2K_PWR_CON, spm_read(C2K_PWR_CON) & ~PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 0" */
		spm_write(C2K_PWR_CON, spm_read(C2K_PWR_CON) & ~PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 0 and PWR_STATUS_2ND = 0" */
		while ((spm_read(PWR_STATUS) & C2K_PWR_STA_MASK)
		       || (spm_read(PWR_STATUS_2ND) & C2K_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Finish to turn off C2K" */
	} else {    /* STA_POWER_ON */
		/* TINFO="Start to turn on C2K" */
		/* TINFO="Set PWR_ON = 1" */
		spm_write(C2K_PWR_CON, spm_read(C2K_PWR_CON) | PWR_ON);
		/* TINFO="Set PWR_ON_2ND = 1" */
		spm_write(C2K_PWR_CON, spm_read(C2K_PWR_CON) | PWR_ON_2ND);

#ifndef IGNORE_PWR_ACK
		/* TINFO="Wait until PWR_STATUS = 1 and PWR_STATUS_2ND = 1" */
		while (!(spm_read(PWR_STATUS) & C2K_PWR_STA_MASK)
		       || !(spm_read(PWR_STATUS_2ND) & C2K_PWR_STA_MASK)) {
				/* No logic between pwr_on and pwr_ack. Print SRAM / MTCMOS control and PWR_ACK for debug. */
		}
#endif

		/* TINFO="Set PWR_CLK_DIS = 0" */
		spm_write(C2K_PWR_CON, spm_read(C2K_PWR_CON) & ~PWR_CLK_DIS);
		/* TINFO="Set PWR_ISO = 0" */
		spm_write(C2K_PWR_CON, spm_read(C2K_PWR_CON) & ~PWR_ISO);
		/* TINFO="Set PWR_RST_B = 1" */
		spm_write(C2K_PWR_CON, spm_read(C2K_PWR_CON) | PWR_RST_B);

		/* TINFO="Release bus protect" */
		spm_write(INFRA_TOPAXI_PROTECTEN, spm_read(INFRA_TOPAXI_PROTECTEN) & ~C2K_PROT_MASK);
		while (spm_read(INFRA_TOPAXI_PROTECTSTA1) & C2K_PROT_MASK) {
		}

		/* TINFO="Finish to turn on C2K" */
	}
	return err;
}



