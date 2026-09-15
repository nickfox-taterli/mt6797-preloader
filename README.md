# MT6797 Preloader component

This component contains the exact X20-96 Preloader binaries from the local
MediaTek X20 AOSP 16.10 package and a checked-in copy of a recovered MT6797
vendor Preloader source tree.  The binary and source inputs are deliberately
separate: the AOSP/LK target is `amt6797_64_open`, while the recovered source
build target is `nb6797_6c_m`.  Same SoC is not sufficient evidence that the
two board configurations are interchangeable.

| File | Role |
| --- | --- |
| `prebuilt/preloader.img` | exact X20-96 normal-image reference |
| `prebuilt/preloader_amt6797_64_open.bin` | exact X20-96 special/FES recovery reference |
| `source/` | experimental `nb6797_6c_m` source; not board-qualified |

The normal and special binaries are kept byte-for-byte.  Do not substitute
the generic preloader reference under `../../sources/`; that source is useful
for comparison only and is not an MT6797 X20-96 implementation.

To rebuild the recovered source tree:

```sh
./build-nb6797-6c-m.sh
```

The script builds the checked-in `source/` tree and records the component
commit, source-tree hash, target and SHA-256 values.  Its current DRAM choice
(`H9CKNNNBKTMRPR-NUH`) is a working BOM hypothesis, not a substitute for
reading the actual board marking or validating every board-specific timing.
Set `SOURCE_DATE_EPOCH` for byte-stable output.  Its rolling output and
SHA-256 manifest are under the workspace `out/` directory; it does not create
a new large versioned tree for every edit.

The source target differs from the exact X20-96 LK target in DCT EINT/GPIO
data, auxiliary peripheral definitions, RTC defaults and the DRAM candidate
list.  The common UART1/MSDC0 pad definitions and MT6351 PMIC are useful
clues, but do not prove a complete board match.  Keep the source build marked
experimental until a clean cold-boot test proves the required DRAM, eMMC and
UART behavior.

The replacement Loader work can continue independently.  Until the recovered
source target is qualified, the exact `amt6797_64_open` binary is the recovery
and production baseline.  A source Preloader probe may be loaded through FES,
but it must not replace that baseline merely because it enumerates on the
same MT6797.
