/* This software/firmware and related documentation ("MediaTek Software") are
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
#include <platform.h>
#include "partition.h"
#include "log_store_pl.h"
#include "dram_buffer.h"



#define MOD "LOG_STORE"
typedef unsigned int u32;

#define AEE_IPANIC_PLABLE "expdb"

//#define	PRINT_EARLY_KERNEL_LOG 
#define EMMC_LOG_SIZE LOG_STORE_SIZE*2

#define DEBUG_LOG

#ifdef DEBUG_LOG
#define LOG_DEBUG(fmt, ...) \
	log_store_enable = false; \
	print(fmt, ##__VA_ARGS__); \
	log_store_enable = true
#else
#define LOG_DEBUG(fmt, ...)
#endif

#define LOG_MEM_ALIGNMENT (0x1000)  //4K Alignment


#define ASCII_CHAR_MIN	0X08
#define ASCII_CHAR_MAX	0X7E


// !!!!!!! Because log store be called by print, so these function don't use print log to debug.
// !!!!!!!

typedef enum {
    LOG_WRITE = 0x1,		// Log is write to buff
    LOG_READ_KERNEL = 0x2,	// Log have readed by kernel
    LOG_WRITE_EMMC = 0x4,	// log need save to emmc
    LOG_EMPTY = 0x8,		// log is empty
    LOG_FULL = 0x10,		// log is full
    LOG_PL_FINISH = 0X20,	// pl boot up finish
    LOG_LK_FINISH = 0X40,	// lk boot up finish
    LOG_DEFAULT = LOG_WRITE_EMMC|LOG_EMPTY,
} BLOG_FLAG;


//printk log store buff, 1 DRAM, 0 SRAM. only printk user DRAM, 
// we can user printk SRAM buff to store log.
extern int  g_log_drambuf;
#define C_LOG_SRAM_BUF_SIZE (20480)
extern char log_sram_buf[C_LOG_SRAM_BUF_SIZE];
#define bootarg g_dram_buf->bootarg


static int log_store_status = BUFF_NOT_READY;
static struct pl_lk_log *pl_buff_header = NULL;
static struct dram_buf_header *sram_dram_buff = NULL;
static char *pbuff = NULL;
static int log_store_sram = 1;
static int sram_store_count = 0;
static bool log_store_enable = true;


// set the flag whether store log to emmc in next boot phase in pl
void store_log_to_emmc_enable(bool value)
{

	if(sram_dram_buff == NULL)
	{
		LOG_DEBUG("%s: sram dram buff is NULL.\n", MOD);
		return;
	}

	if(value)
	{
		sram_dram_buff->flag |= NEED_SAVE_TO_EMMC;
	}
	else
	{
		sram_dram_buff->flag &= ~NEED_SAVE_TO_EMMC;
	}

	return;
}



void format_log_buff(void)
{
	log_store_enable = false;
	memset(pl_buff_header, 0, sizeof(struct pl_lk_log));
	pl_buff_header->sig = LOG_STORE_SIG;
	pl_buff_header->buff_size = LOG_STORE_SIZE;
	pl_buff_header->off_pl = sizeof(struct pl_lk_log);
	pl_buff_header->off_lk = sizeof(struct pl_lk_log);
	pl_buff_header->pl_flag = LOG_DEFAULT;
//	buff_header->off_lk =  ; off_lk will set in lk phase init.
	pl_buff_header->lk_flag = LOG_DEFAULT;
	log_store_enable = true;
	return;
}

int logbuf_valid(void)
{
	if((pl_buff_header != NULL) && (pl_buff_header->sig == LOG_STORE_SIG) && 
		(pl_buff_header->buff_size == LOG_STORE_SIZE) && (pl_buff_header->off_pl == sizeof(struct pl_lk_log))
		&& ((pl_buff_header->sz_lk + pl_buff_header->sz_pl + pl_buff_header->off_pl) <= LOG_STORE_SIZE))
		return 1;
	return 0;

}

// kernel log store to pl&lk dram buff
void kernel_log_dram()
{
	int i;
	char *value;

	pl_buff_header->sz_pl = 0;
	pl_buff_header->sz_lk = 0;
	for(i=0; i <sram_dram_buff->reserve2[1]; i++)
	{
		value = sram_dram_buff->reserve2[0]+i;
		if(*value < ASCII_CHAR_MIN || *value > ASCII_CHAR_MAX)
		{
			continue;			
		}
		*(pbuff + pl_buff_header->off_pl + pl_buff_header->sz_pl) = *value;
		pl_buff_header->sz_pl++;
	}
}

// store log buff to emmc
void log_to_emmc(struct sram_log_header *sram_buff)
{
	int i;
	char *value;
	part_t *part_ptr;
	blkdev_t *bootdev;
	u64 emmc_add;
	int store_size;
	int ret = 0;
	
	LOG_DEBUG("%s:log_to_emmc function flag 0x%x!\n",MOD, sram_dram_buff->flag);	
	
	if(sram_buff == NULL) {
		LOG_DEBUG("%s:sram_buff is null!\n",MOD);
		return;
	}
	if(sram_dram_buff == NULL)
	{
		LOG_DEBUG("%s:sram_dram_buff is NULL!\n",MOD);
		return;
	}
	
	if(sram_dram_buff->sig != DRAM_HEADER_SIG)
	{
		LOG_DEBUG("%s:sram_dram_buff->sig error 0x%x!\n",MOD,sram_dram_buff->sig);
		return;
	}

	if(sram_dram_buff->buf_addr == NULL)
	{
		LOG_DEBUG("%s:sram_dram_buff->buf_addr is NULL!\n",MOD);
		return;
	}
		
	if(sram_dram_buff->buf_size != LOG_STORE_SIZE)
	{
		LOG_DEBUG("%s:sram_dram_buff->buf_size is error 0x%x!\n",MOD,sram_dram_buff->buf_size);
		return;
	}

	if((sram_dram_buff->flag&NEED_SAVE_TO_EMMC) != NEED_SAVE_TO_EMMC)
	{
		LOG_DEBUG("%s:don't need to store to emmc, flag 0x%x!\n",MOD,sram_dram_buff->flag);
		return;
	}
	sram_buff->reboot_count++;
	pbuff =	sram_dram_buff->buf_addr;
	pl_buff_header = (struct pl_lk_log *)pbuff;	

	if (logbuf_valid() == 0) {
		LOG_DEBUG("%s:logbuf_valid!\n", MOD);
		return;
	}	

	if(sram_buff->reboot_count >= sram_buff->save_to_emmc)
	{
		log_store_enable = false;
		bootdev = blkdev_get(CFG_BOOT_DEV);
		log_store_enable = true;
		if(NULL == bootdev)
		{
			LOG_DEBUG("%s can't find boot device(%d)\n", MOD, CFG_BOOT_DEV);
			return ;
	    }
//		for(i = 0; i < MAX_DRAM_COUNT; i++)
		{
			log_store_enable = false;
			part_ptr = part_get(AEE_IPANIC_PLABLE);
			log_store_enable = true;
			if(part_ptr == NULL)
			{
				LOG_DEBUG("%s:log_to_emmc get partition error!\n",MOD);
			}
			else
			{
				LOG_DEBUG("%s: %s partition add 0x%x, size 0x%x, blk size 0x%x!\n",
					MOD,AEE_IPANIC_PLABLE,part_ptr->start_sect,part_ptr->nr_sects,bootdev->blksz);
				LOG_DEBUG("%s:lk size 0x%x, pl size 0x%x!\n",MOD, pl_buff_header->sz_lk, pl_buff_header->sz_pl);

//write unit is bootdev->blksz;

				if(part_ptr->nr_sects * bootdev->blksz > EMMC_LOG_SIZE)
				{
					//emmc_add = (part_ptr->start_sect + part_ptr->nr_sects) * bootdev->blksz - EMMC_LOG_SIZE;
					emmc_add = (part_ptr->start_sect + part_ptr->nr_sects) * bootdev->blksz - 0x200000;
					if((pl_buff_header->sz_lk + pl_buff_header->sz_pl)%bootdev->blksz)
					{
						store_size = ((pl_buff_header->sz_lk + pl_buff_header->sz_pl)/bootdev->blksz +1) * bootdev->blksz;
					}
					else
					{
						store_size = pl_buff_header->sz_lk + pl_buff_header->sz_pl;
					}
					
					
				}
				else
				{
					emmc_add = part_ptr->start_sect * bootdev->blksz;
					store_size = part_ptr->nr_sects * bootdev->blksz;
				}
				
				
				if (store_size != 0) {
					log_store_enable = false;
					//ret = blkdev_write(bootdev, emmc_add, store_size, pbuff + pl_buff_header->off_pl, part_ptr->part_id);
					ret = blkdev_write(bootdev, emmc_add, EMMC_LOG_SIZE, pbuff + pl_buff_header->off_pl, part_ptr->part_id);
					log_store_enable = true;
				} else {
					sram_buff->save_to_emmc = sram_buff->reboot_count = 0;
				}

				LOG_DEBUG("%s:log_to_emmc  add 0x%x!\n",MOD, emmc_add);	
				LOG_DEBUG("%s:log_to_emmc write size 0x%x, ret value 0x%x!\n", MOD, store_size, ret);	
				
				if((sram_dram_buff->flag & BUFF_EARLY_PRINTK) && sram_dram_buff->reserve2[0] != 0 
						&& sram_dram_buff->reserve2[1] != 0)
				{
					kernel_log_dram();
					emmc_add += store_size; 
					if(pl_buff_header->sz_pl%bootdev->blksz)
					{
						store_size = ( pl_buff_header->sz_pl/bootdev->blksz +1) * bootdev->blksz;
					}
					else
					{
						store_size = pl_buff_header->sz_pl;
					}
					
					if (store_size != 0) {
						log_store_enable = false;
						ret = blkdev_write(bootdev, emmc_add, store_size, pbuff + pl_buff_header->off_pl, part_ptr->part_id);
						log_store_enable = true;
					}					
					LOG_DEBUG("%s:kernel log log_to_emmc add %x, size %x, write ret value %d!\n", MOD, emmc_add, store_size, ret);
				}
			}
		}
		sram_buff->save_to_emmc = 10 * sram_buff->reboot_count;
		//reset the save log to emmc count, avoid save to emmc  frequently when reboot issue. 
	}else {
		LOG_DEBUG("%s:reboot_count %d,save_to_emmc %d.\n", MOD, sram_buff->reboot_count, sram_buff->save_to_emmc);
	}	
	return;
}

#ifdef PRINT_EARLY_KERNEL_LOG
void kernel_log_show(void)
{
	int i = 0;
	char *value;
	if(sram_dram_buff == NULL)
		return;
	
	// print early printk message
	if((sram_dram_buff->flag & BUFF_EARLY_PRINTK) && sram_dram_buff->reserve2[0] != 0 
		&& sram_dram_buff->reserve2[1] != 0)
	{
		log_store_enable = false;
		for(i=0; i <sram_dram_buff->reserve2[1]; i++)
		{
			value = *(char *)(sram_dram_buff->reserve2[0]+i);
			if(value< ASCII_CHAR_MIN || value>ASCII_CHAR_MAX)
			{
				continue;
			}
			print("%c",value);
		}
		log_store_enable = true;
	}

}
#endif

void log_store_init(void)
{
	struct sram_log_header *sram_header = NULL;	
	
	if(log_store_status != BUFF_NOT_READY)
	{
		LOG_DEBUG("%s:log_sotore_status is ready!\n",MOD);
		return;
	}
// SRAM buff header init
	sram_header = (struct sram_log_header*)SRAM_LOG_ADDR;
	LOG_DEBUG("%s:sram->sig value 0x%x!\n",MOD,sram_header->sig);
	if(sram_header->sig != SRAM_HEADER_SIG)
	{
		log_store_enable = false;
		memset(sram_header,0, sizeof(struct sram_log_header));
		LOG_DEBUG("%s:sram buff header is not match, format all!\n",MOD);
		sram_header->sig = SRAM_HEADER_SIG;
	}



	sram_dram_buff = &(sram_header->dram_buf[LOG_PL_LK]);

	
	// Save log to emmc
	log_to_emmc(sram_header);
	
#ifdef PRINT_EARLY_KERNEL_LOG
	kernel_log_show();
#endif
	log_store_enable = false;
	memset(sram_dram_buff, 0, sizeof(struct dram_buf_header));
	sram_dram_buff->sig = DRAM_HEADER_SIG;
	log_store_enable = false;
	
	pbuff = (u32)mblock_reserve(&bootarg.mblock_info,
		(u64)LOG_STORE_SIZE, (u64)LOG_MEM_ALIGNMENT, 0xc0000000, RANKMAX);
//        (u64)LOG_STORE_SIZE, (u64)LOG_MEM_ALIGNMENT, 0x100000000, RANKMAX);
       
	if(!pbuff)
	{
		LOG_DEBUG("%s:dram log allocation error!\n",MOD);
		sram_dram_buff->flag = BUFF_ALLOC_ERROR;
		log_store_status = BUFF_ALLOC_ERROR;
		return;
	}

	memset(pbuff, 0, LOG_STORE_SIZE);
	sram_dram_buff->buf_addr = pbuff;
	sram_dram_buff->buf_offsize = sizeof(struct pl_lk_log);
	sram_dram_buff->buf_size = LOG_STORE_SIZE;
	sram_dram_buff->flag = BUFF_VALIE | CAN_FREE | NEED_SAVE_TO_EMMC | ARRAY_BUFF;
	sram_dram_buff->buf_point = 0;
	
	pl_buff_header = (struct pl_lk_log *)pbuff;	

	// init DRAM buff
	format_log_buff();
	log_store_status = BUFF_READY;
	LOG_DEBUG("%s:sram_header 0x%x,sig 0x%x, sram_dram_buff 0x%x, buf_addr 0x%x, pl_buff_header 0x%x!\n", MOD, 
		sram_header,sram_header->sig,sram_dram_buff,sram_dram_buff->buf_addr,pl_buff_header);
	log_store_enable = true;
	return;

}


void store_switch_to_dram(void)
{
	int i=0;
	log_store_sram = 0;
	log_store_init();
	if(g_log_drambuf == 1){
		for(i=0; i < sram_store_count; i++)
		{
			pl_log_store(log_sram_buf[i]);
		}
	}
}

void pl_log_store(char c)
{
	if(log_store_enable == false)
	{
		return;
	}

	if(log_store_status == BUFF_ALLOC_ERROR)
		return;
	
	if((log_store_sram ==1) && (g_log_drambuf == 1))
	{
		if(sram_store_count < C_LOG_SRAM_BUF_SIZE)
		{
			log_sram_buf[sram_store_count++] = c;
		}
		
		return;
	}
	
	if(log_store_status == BUFF_NOT_READY)
	{
		log_store_init();
		return;
	}

	if(logbuf_valid() == 0)
	{
		return;
	}
/*
	LOG_DEBUG("%s:header point 0x%x, sig 0x%x, char %c, log store status %d!\n",
		MOD, pbuff, pl_buff_header->sig, c,log_store_status);

	LOG_DEBUG("%s:pl offsize 0x%x, pl size 0x%x!\n",MOD, pl_buff_header->off_pl,pl_buff_header->sz_pl);
*/	
	if(log_store_status != BUFF_READY)
	{
		return;
	}
	
	*(pbuff + pl_buff_header->off_pl + pl_buff_header->sz_pl) = c;
	/*
	LOG_DEBUG("%s:save adr 0x%x value %c!\n",
		MOD, pbuff + pl_buff_header->off_pl + pl_buff_header->sz_pl, *(pbuff + pl_buff_header->off_pl + pl_buff_header->sz_pl));
	*/
	pl_buff_header->sz_pl++;
	sram_dram_buff->buf_point = pl_buff_header->sz_pl;
	if((pl_buff_header->off_pl + pl_buff_header->sz_pl) >= LOG_STORE_SIZE)
	{
		log_store_status = BUFF_FULL;
	}

	return;
}
