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

The source was copied into the maintained component repository at:

`/root/code/x20/x20-sbc/components/preloader/source`

The original checkout remains at `/root/code/x20/vendor` only as provenance
and a temporary reference.  The current Preloader component HEAD is
`9f2e2c9`; its last source-code snapshot was introduced at `c6f1c0d` (source
tree `29e9db324daae15280c1f88754f896b63099e6ab`).  The vendor checkout's provenance branch is
`x20/nb6797-6c-m-preloader` at `be545c074ea484d0d51433b6bbf1acc032affe5a`.

This is recovered MT6797 `nb6797_6c_m` source, not an established exact source
match for the X20-96 AOSP image.  The local AOSP image and the old LK tree use
the target name `amt6797_64_open`.  A direct comparison found differences in
DCT EINT/GPIO and peripheral data, RTC defaults, target make settings and the
DRAM candidate list.  UART1/MSDC0 pad definitions and the MT6351 PMIC are
common, which makes the source useful for controlled probes but does not make
it safe as a permanent BootROM-facing image.

The `H9CKNNNBKTMRPR-NUH` selection is a current working BOM hypothesis.  It
must remain explicitly marked until the physical DRAM marking or a complete
cold-boot qualification confirms it.

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

## Qualification status and physical validation

An earlier fixed-time source-built artifact was temporarily loaded by BootROM
through xflash on the real X20-96 board.  The paired vendor LK/ATF then
reached:

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

This proves that one source build was accepted far enough to enter the
existing recovery chain; it does not prove that `nb6797_6c_m` is the exact
X20-96 board configuration or that it is safe for cold boot.

In the later raw-stage migration probe, the current component-built artifact
`69aa9dba7be572e386b2fdac8d0f6d7a688ed2c290ef6456d5a6924b00df88eb` (202648
bytes) was written to Boot1 through the migration Loader.  The board showed
Preloader/Android USB enumeration but did not yield a complete UART-backed
cold-boot-to-Debian result.  Treat that Boot1 contents as experimental and
restore the exact known-good binary before the next persistent-chain test.

The follow-up recovery attempt was intentionally ordered for a fast BootROM
window: xflash was already at `wait for device`, then a manual cold power-on,
one D2 10000-ms pulse, and one D3 500-ms pulse were tried.  The controller
acknowledged both reset commands, but the physical host saw no new
`0e8d:2000` BootROM/Preloader device and xflash ended with `wait for com
error. code 0x9`.  This is additional negative qualification evidence for
the source image.  It does not identify whether the remaining failure is
DRAM, board DCT, eMMC, or another BootROM-facing mismatch, so the source
image must not be retried as a persistent Boot1 image.

The exact X20-96 binary remains the fallback:
`prebuilt/preloader_amt6797_64_open.bin`, SHA-256
`5e50d7c8820d24e08f6767911c7e53b39661cc5e90a2a29670236be8e7b3f2d7`.
Keeping this binary is intentional and is not considered a failure of the
source-migration work; source qualification can continue independently.

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
