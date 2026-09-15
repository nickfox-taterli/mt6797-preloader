###################################################################
# Include Project Feautre  (cust_bldr.h)
###################################################################

#ifeq ("$(MTK_EMMC_SUPPORT)","yes")
ifdef MTK_EMMC_SUPPORT
CFG_BOOT_DEV :=BOOTDEV_SDMMC
else
CFG_BOOT_DEV :=BOOTDEV_NAND
endif

CFG_UART_LOG :=UART1
CFG_UART_META :=UART1

# X20 development boot contract: stage-2 Loader and ATF are read from a
# fixed raw user-area range.  The legacy GPT path remains a fallback until
# the raw chain has passed its physical migration gate.
CFG_X20_RAW_BOOT :=1
