# Preloader source status

Checked on 2026-09-15.

The original local package is:

`/root/code/x20/mediatek-x20-aosp-16.10`

Its `Source/sla.tar.gz` contains the vendor LK and vendor ATF trees, but the
only paths matching `preloader` are:

```text
vendor/amt/libs/mt6797_64/images/preloader.img
vendor/amt/libs/mt6797_64/images/preloader_amt6797_64_open.bin
```

No `vendor/mediatek/proprietary/bootable/bootloader/preloader` source tree is
present in that AOSP archive.  The public repositories cloned under the
workspace are likewise references: Google LK contains an MT6797 LK platform
directory, while the public `svoboda18/preloader` tree is a generic legacy-MTK
tree.

## Recovered source tree

A separate local Git checkout now provides the matching vendor source:

`/root/code/x20/vendor`

Remote:

`https://gitlab.com/mt6797/vendor`

The X20-specific Preloader source is:

`mediatek/proprietary/bootable/bootloader/preloader`

The maintained branch is `x20/nb6797-6c-m-preloader`, currently pinned at
`be545c074ea484d0d51433b6bbf1acc032affe5a`.  Its board-specific commit selects
the BOM DRAM part `H9CKNNNBKTMRPR-NUH`; the original `nb6797_6c_m` header had
the stale `H9CKNNNDATMUPR` entry and could not be built against its current
memory table.

The source build is intentionally kept in the Preloader component as a small
wrapper:

```sh
/root/code/x20/x20-sbc/components/preloader/build-nb6797-6c-m.sh
```

It uses `/usr/bin/arm-none-eabi-` by default, `-std=gnu89` plus narrowly scoped
GCC compatibility flags, and writes one rolling output directory under
`/root/code/x20/out/`.  It produces a GFH-bearing Preloader binary, a linked
raw binary, and a manifest with the source commit and SHA-256 values.  The
wrapper does not copy the 89 MiB source subtree into this component and does
not replace the verified prebuilt files automatically.

Set `SOURCE_DATE_EPOCH` (or `BUILD_TIME`) when a byte-stable artifact is
needed.  The vendor build-time patch is included in the pinned commit; without
either variable the wrapper records the current time for bring-up diagnostics.

The vendor Preloader still has to emit the MediaTek BootROM GFH/security
format.  The Android lock, AVB, fastboot policy and signature logic targeted
for removal belong to the replacement Loader; they must not be removed from
this immutable BootROM-facing stage without a separate BootROM test.

## Physical FES validation

The fixed-time source-built artifact was temporarily loaded by BootROM through
xflash on the real X20-96 board.  The paired vendor LK/ATF then reached:

- one-rank `dram_rank_size[0] = 0x80000000` (2 GiB DRAM);
- eMMC initialization at the known-good 25 MHz mode;
- `0123456789ABCDEF fastboot`.

The board's permanent Preloader, GPT, `lk`, and `tee1` were not changed.  The
remote test input and complete UART capture are retained here:

```text
/root/x20-fes-test/source-preloader-bkt-v1/preloader_amt6797_64_open.bin
SHA-256: 0cea579b36a3ff98bb4d68222570e6592b3a7a7b9dd8b8bd00742fe690b4ed5f

/root/x20-fes-test/normal-repair/source-preloader-bkt-v1-serial.log
SHA-256: 0414e93db62eef73a92d801ea700050fec73bb598b563a3da016c235e86e4e9a
```

This proves the source-built BootROM-facing stage can enter the existing
recovery chain; it is not yet a permanent Preloader replacement or the final
single-rootfs boot chain.

The old 96Boards archive URL was also checked and returned HTTP 404:

```text
https://builds.96boards.org/releases/helio-x20/mediatek/aosp/16.10/mediatek-x20-aosp-16.10.tar.xz
```

The local binaries are therefore intentionally retained rather than replaced
by an unverified source build.

## Checksums

| File | Bytes | SHA-256 | MD5 |
| --- | ---: | --- | --- |
| `prebuilt/preloader_amt6797_64_open.bin` | 186684 | `5e50d7c8820d24e08f6767911c7e53b39661cc5e90a2a29670236be8e7b3f2d7` | `0f3e541d1ca9b94c8b959d17b4d065b2` |
| `prebuilt/preloader.img` | 176640 | `eea25d04bb3879b37607abc42d3410d431541053f63a43a73313ec5425a32703` | `1a1d873bdc74e031f8c834c01ad0a419` |

These are reference artifacts for this board.  Their redistribution and
licensing remain subject to the original package terms.
