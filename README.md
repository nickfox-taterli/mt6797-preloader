# MT6797 Preloader component

This component contains the exact X20-96 preloader binaries from the local
MediaTek X20 AOSP 16.10 package and the reproducible build entry point for the
newly recovered vendor source tree.  The prebuilt files remain the known-good
BootROM input while the source-built result is being tested.

The two files have different roles:

| File | Role |
| --- | --- |
| `prebuilt/preloader.img` | normal-image preloader |
| `prebuilt/preloader_amt6797_64_open.bin` | special/FES preloader |

The normal and special binaries are kept byte-for-byte.  Do not substitute
the generic preloader reference under `../../sources/`; that source is useful
for comparison only and is not an MT6797 X20-96 implementation.

To rebuild the source tree (without copying it into this repository):

```sh
./build-nb6797-6c-m.sh
```

The script pins `/root/code/x20/vendor` to the board-specific Git commit and
uses the Hynix `H9CKNNNBKTMRPR-NUH` DRAM configuration from the X20 BOM.  Set
`SOURCE_DATE_EPOCH` for byte-stable output.  Its rolling output and SHA-256
manifest are under the workspace `out/` directory; it does not create a new
large versioned tree for every edit.

The replacement Loader work starts after this component.  Until DRAM,
eMMC-boot-area access, and the BootROM handoff are independently validated,
these binaries remain the recovery path and must not be overwritten.
