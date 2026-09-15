###################################################################
# flag
###################################################################

MOD_SRC  =
MOD_OBJ  = $(PRELOADER_OUT)/obj

###################################################################
# source files
# - add your file here !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
###################################################################

MOD_SRC  := \
	circbuf.c \
	rtc.c \
	mmc_common_inter.c \
	mmc_core.c \
        mmc_rpmb.c \
	mmc_test.c \
	msdc_io.c \
	msdc.c \
	msdc_dma.c \
	msdc_irq.c \
	msdc_utils.c \
	i2c.c \
	keypad.c \
	usbd.c \
	usbphy.c \
	usbtty.c \
	emi.c \
	dramc_pi_main.c \
	dramc_pi_basic_api.c \
	dramc_pi_calibration_api.c \
	Hal_io.c \
	timer.c \
	wdt.c \
	efi.c \
	platform.c \
	pll.c \
	device_apc.c \
	gpio.c \
	uart.c \
	pmic_wrap_init.c \
	pmic.c \
	fan53555.c \
	fan53526.c \
	da9214.c \
	charging_bat.c \
	memory.c \
	memory_test.s \
	sync_write.c \
	spm_mtcmos.c \
	hotplug.s \
	log_store_pl.c \
	clkbuf_ctl.c \
	mt_ptp.c

#ifneq ("$(MTK_EMMC_SUPPORT)","yes")
ifndef MTK_EMMC_SUPPORT
	MOD_SRC  += nand.c
	MOD_SRC  +=	bmt.c
endif

ifeq ("$(PL_MODE)","DUMMY_AP")
	MOD_SRC  += gpio_init.c
endif

ifeq ("$(PL_MODE)","TINY")
	MOD_SRC  += gpio_init.c
endif

ifeq ("$(CFG_RAM_CONSOLE)", "1")
	MOD_SRC	+= ram_console.c
endif

###################################################################
# target
###################################################################

include $(COMMON_FILE_MK)

###################################################################
# trigger emigen
###################################################################
$(MOD_OBJ)/emi.o: $(EMIGEN_H_OUT)/custom_emi.h
