/* Load the X20 development Loader and ATF from a fixed raw eMMC range. */
#include "typedefs.h"
#include "platform.h"
#include "blkdev.h"
#include "mmc_core.h"
#include "string.h"
#include "x20_raw_stage.h"

#define MOD "[X20_STAGE]"

static u8 x20_stage_header[X20_RAW_STAGE_HEADER_SIZE]
__attribute__((aligned(4)));

static u32 x20_get_le32(const u8 *data)
{
    return (u32)data[0] | ((u32)data[1] << 8) |
           ((u32)data[2] << 16) | ((u32)data[3] << 24);
}

static u64 x20_get_le64(const u8 *data)
{
    return (u64)x20_get_le32(data) |
           ((u64)x20_get_le32(data + 4) << 32);
}

static void x20_put_le32(u8 *data, u32 value)
{
    data[0] = (u8)value;
    data[1] = (u8)(value >> 8);
    data[2] = (u8)(value >> 16);
    data[3] = (u8)(value >> 24);
}

static u32 x20_crc32(const u8 *data, u32 size)
{
    u32 crc = 0xffffffffU;
    u32 bit;

    while (size--) {
        crc ^= *data++;
        for (bit = 0; bit < 8; bit++) {
            if (crc & 1U)
                crc = (crc >> 1) ^ 0xedb88320U;
            else
                crc >>= 1;
        }
    }
    return crc ^ 0xffffffffU;
}

static int x20_range_valid(u64 offset, u64 size, u64 total, u32 maximum)
{
    if (offset < X20_RAW_STAGE_HEADER_SIZE ||
        (offset & (X20_RAW_STAGE_ALIGNMENT - 1)) ||
        size == 0 || size > maximum || offset + size <= offset ||
        offset + size > total)
        return 0;
    return 1;
}

static int x20_stage_header_valid(u64 *total, u64 *loader_offset,
                                  u32 *loader_size, u64 *loader_load,
                                  u64 *loader_entry, u32 *loader_crc,
                                  u64 *atf_offset, u32 *atf_size,
                                  u64 *atf_load, u64 *atf_entry,
                                  u32 *atf_crc)
{
    u32 stored_crc;
    u32 calculated_crc;
    u64 bundle_lba;
    u64 bundle_lbas;

    if (memcmp(x20_stage_header, X20_RAW_STAGE_MAGIC,
               X20_RAW_STAGE_MAGIC_SIZE) != 0 ||
        x20_get_le32(x20_stage_header + 0x08) != X20_RAW_STAGE_VERSION ||
        x20_get_le32(x20_stage_header + 0x0c) != X20_RAW_STAGE_HEADER_SIZE)
        return -1;

    stored_crc = x20_get_le32(x20_stage_header + X20_RAW_STAGE_HEADER_CRC);
    x20_put_le32(x20_stage_header + X20_RAW_STAGE_HEADER_CRC, 0);
    calculated_crc = x20_crc32(x20_stage_header,
                               X20_RAW_STAGE_HEADER_SIZE);
    x20_put_le32(x20_stage_header + X20_RAW_STAGE_HEADER_CRC, stored_crc);
    if (stored_crc != calculated_crc)
        return -1;

    *total = x20_get_le64(x20_stage_header + X20_RAW_STAGE_TOTAL_SIZE_OFFSET);
    *loader_offset = x20_get_le64(x20_stage_header +
                                  X20_RAW_STAGE_LOADER_OFFSET);
    *loader_size = x20_get_le32(x20_stage_header +
                                X20_RAW_STAGE_LOADER_SIZE);
    *loader_load = x20_get_le64(x20_stage_header +
                                X20_RAW_STAGE_LOADER_LOAD);
    *loader_entry = x20_get_le64(x20_stage_header +
                                 X20_RAW_STAGE_LOADER_ENTRY);
    *loader_crc = x20_get_le32(x20_stage_header + X20_RAW_STAGE_LOADER_CRC);
    *atf_offset = x20_get_le64(x20_stage_header + X20_RAW_STAGE_ATF_OFFSET);
    *atf_size = x20_get_le32(x20_stage_header + X20_RAW_STAGE_ATF_SIZE);
    *atf_load = x20_get_le64(x20_stage_header + X20_RAW_STAGE_ATF_LOAD);
    *atf_entry = x20_get_le64(x20_stage_header + X20_RAW_STAGE_ATF_ENTRY);
    *atf_crc = x20_get_le32(x20_stage_header + X20_RAW_STAGE_ATF_CRC);
    bundle_lba = x20_get_le64(x20_stage_header +
                              X20_RAW_STAGE_BUNDLE_LBA_OFFSET);
    bundle_lbas = x20_get_le64(x20_stage_header +
                               X20_RAW_STAGE_BUNDLE_LBAS_OFFSET);

    if (*total < X20_RAW_STAGE_HEADER_SIZE ||
        *total > X20_RAW_STAGE_MAX_BYTES ||
        (*total & (X20_RAW_STAGE_ALIGNMENT - 1)) ||
        !x20_range_valid(*loader_offset, *loader_size, *total,
                         X20_RAW_STAGE_LOADER_MAX_BYTES) ||
        !x20_range_valid(*atf_offset, *atf_size, *total,
                         X20_RAW_STAGE_ATF_MAX_BYTES) ||
        *loader_load != X20_RAW_STAGE_LOADER_ADDRESS ||
        *loader_entry != X20_RAW_STAGE_LOADER_ADDRESS ||
        *atf_load != CFG_ATF_ROM_MEMADDR || *atf_entry != CFG_ATF_ROM_MEMADDR ||
        bundle_lba != X20_RAW_STAGE_BUNDLE_USER_LBA ||
        bundle_lbas == 0 || bundle_lbas > X20_RAW_STAGE_BUNDLE_MAX_LBAS)
        return -1;

    return 0;
}

int x20_raw_stage_load(blkdev_t *bdev, u32 *jump_addr, u32 *loader_size)
{
    u64 stage_base;
    u64 total;
    u64 loader_offset;
    u64 loader_load;
    u64 loader_entry;
    u64 atf_offset;
    u64 atf_load;
    u64 atf_entry;
    u32 loaded_loader_size;
    u32 loaded_atf_size;
    u32 loader_crc;
    u32 atf_crc;

    if (!bdev || bdev->blksz != 512)
        return -1;

    stage_base = (u64)X20_RAW_STAGE_USER_LBA * 512ULL;
    print("%s read header at user LBA 0x%x\n", MOD,
          X20_RAW_STAGE_USER_LBA);
    if (blkdev_read(bdev, stage_base, X20_RAW_STAGE_HEADER_SIZE,
                    x20_stage_header, EMMC_PART_USER) != 0)
        return -1;

    if (x20_stage_header_valid(&total, &loader_offset,
                               &loaded_loader_size, &loader_load,
                               &loader_entry, &loader_crc,
                               &atf_offset, &loaded_atf_size, &atf_load,
                               &atf_entry, &atf_crc) != 0) {
        print("%s invalid header\n", MOD);
        return -1;
    }

    if (blkdev_read(bdev, stage_base + loader_offset, loaded_loader_size,
                    (u8 *)(unsigned long)loader_load, EMMC_PART_USER) != 0)
        return -1;
    if (x20_crc32((u8 *)(unsigned long)loader_load, loaded_loader_size) !=
        loader_crc) {
        print("%s Loader CRC mismatch\n", MOD);
        return -1;
    }

    if (blkdev_read(bdev, stage_base + atf_offset, loaded_atf_size,
                    (u8 *)(unsigned long)atf_load, EMMC_PART_USER) != 0)
        return -1;
    if (x20_crc32((u8 *)(unsigned long)atf_load, loaded_atf_size) !=
        atf_crc) {
        print("%s ATF CRC mismatch\n", MOD);
        return -1;
    }

    *jump_addr = (u32)loader_entry;
    if (loader_size)
        *loader_size = loaded_loader_size;
    print("%s Loader 0x%x (%u), ATF 0x%x (%u), total 0x%llx\n", MOD,
          (u32)loader_load, loaded_loader_size, (u32)atf_load,
          loaded_atf_size, total);
    return 0;
}
