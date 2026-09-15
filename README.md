# MT6797 Preloader component

This component currently contains the exact X20-96 preloader binaries from
the local MediaTek X20 AOSP 16.10 package.  It is a preserved boot-ROM input,
not a newly built replacement.

The two files have different roles:

| File | Role |
| --- | --- |
| `prebuilt/preloader.img` | normal-image preloader |
| `prebuilt/preloader_amt6797_64_open.bin` | special/FES preloader |

The normal and special binaries are kept byte-for-byte.  Do not substitute
the generic preloader reference under `../../sources/`; that source is useful
for comparison only and is not an MT6797 X20-96 implementation.

The replacement Loader work starts after this component.  Until DRAM,
eMMC-boot-area access, and the BootROM handoff are independently validated,
these binaries remain the recovery path and must not be overwritten.
