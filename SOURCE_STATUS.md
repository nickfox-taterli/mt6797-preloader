# Preloader source status

Checked on 2026-09-15.

The local package is:

`/root/code/x20/mediatek-x20-aosp-16.10`

Its `Source/sla.tar.gz` contains the vendor LK and vendor ATF trees, but the
only paths matching `preloader` are:

```text
vendor/amt/libs/mt6797_64/images/preloader.img
vendor/amt/libs/mt6797_64/images/preloader_amt6797_64_open.bin
```

No `vendor/mediatek/proprietary/bootable/bootloader/preloader` source tree was
present in that archive.  The public repositories cloned under the workspace
are likewise references: Google LK contains an MT6797 LK platform directory,
while the public `svoboda18/preloader` tree is a generic legacy-MTK tree.

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
