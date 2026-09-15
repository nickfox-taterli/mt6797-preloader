/* Fixed raw stage format for the X20 development board. */
#ifndef X20_RAW_STAGE_H
#define X20_RAW_STAGE_H

#define X20_RAW_STAGE_MAGIC                 "X20STAGE"
#define X20_RAW_STAGE_MAGIC_SIZE            8
#define X20_RAW_STAGE_VERSION               1
#define X20_RAW_STAGE_HEADER_SIZE            0x200
#define X20_RAW_STAGE_ALIGNMENT              0x200
#define X20_RAW_STAGE_USER_LBA              0x1000
#define X20_RAW_STAGE_MAX_BYTES              0x200000

#define X20_RAW_STAGE_FLAGS_OFFSET           0x10
#define X20_RAW_STAGE_TOTAL_SIZE_OFFSET      0x18
#define X20_RAW_STAGE_LOADER_OFFSET          0x20
#define X20_RAW_STAGE_LOADER_SIZE            0x28
#define X20_RAW_STAGE_LOADER_LOAD            0x30
#define X20_RAW_STAGE_LOADER_ENTRY           0x38
#define X20_RAW_STAGE_LOADER_CRC             0x40
#define X20_RAW_STAGE_ATF_OFFSET             0x48
#define X20_RAW_STAGE_ATF_SIZE               0x50
#define X20_RAW_STAGE_ATF_LOAD               0x58
#define X20_RAW_STAGE_ATF_ENTRY              0x60
#define X20_RAW_STAGE_ATF_CRC                0x68
#define X20_RAW_STAGE_BUNDLE_LBA_OFFSET      0x70
#define X20_RAW_STAGE_BUNDLE_LBAS_OFFSET     0x78
#define X20_RAW_STAGE_HEADER_CRC             0x80

#define X20_RAW_STAGE_LOADER_ADDRESS         0x46000000
#define X20_RAW_STAGE_LOADER_MAX_BYTES       0x900000
#define X20_RAW_STAGE_ATF_MAX_BYTES          0x40000
#define X20_RAW_STAGE_ATF_LOAD_ADDRESS       0x001005c0
#define X20_RAW_STAGE_ATF_ENTRY_ADDRESS      0x00101000
#define X20_RAW_STAGE_BUNDLE_USER_LBA        0x4000
#define X20_RAW_STAGE_BUNDLE_MAX_LBAS        0x4000

#endif
