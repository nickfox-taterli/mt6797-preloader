#!/usr/bin/env python3
"""Build and inspect the X20 raw stage consumed by the Preloader.

The stage is deliberately independent of GPT names and Android image headers.
It contains only a fixed header, the stage-2 Loader, and the ATF package
payload.  The raw boot bundle remains at its separately fixed user-area LBA.
"""

from __future__ import annotations

import argparse
import binascii
import struct
import sys
from pathlib import Path


MAGIC = b"X20STAGE"
VERSION = 1
HEADER_SIZE = 0x200
ALIGNMENT = 0x200
STAGE_USER_LBA = 0x1000
MAX_STAGE_BYTES = 0x200000

FLAGS_OFFSET = 0x10
TOTAL_SIZE_OFFSET = 0x18
LOADER_OFFSET = 0x20
LOADER_SIZE = 0x28
LOADER_LOAD = 0x30
LOADER_ENTRY = 0x38
LOADER_CRC = 0x40
ATF_OFFSET = 0x48
ATF_SIZE = 0x50
ATF_LOAD = 0x58
ATF_ENTRY = 0x60
ATF_CRC = 0x68
BUNDLE_LBA = 0x70
BUNDLE_LBAS = 0x78
HEADER_CRC = 0x80

LOADER_ADDRESS = 0x46000000
LOADER_MAX_BYTES = 0x900000
ATF_LOAD_ADDRESS = 0x001005C0
ATF_ENTRY_ADDRESS = 0x00101000
ATF_MAX_BYTES = 0x40000
BUNDLE_USER_LBA = 0x4000
BUNDLE_MAX_LBAS = 0x4000

PART_MAGIC = 0x58881688
PART_HEADER_SIZE = 0x200


def align_up(value: int, alignment: int = ALIGNMENT) -> int:
    return (value + alignment - 1) & ~(alignment - 1)


def get_u32(data: bytes, offset: int) -> int:
    return struct.unpack_from("<I", data, offset)[0]


def get_u64(data: bytes, offset: int) -> int:
    return struct.unpack_from("<Q", data, offset)[0]


def put_u32(data: bytearray, offset: int, value: int) -> None:
    struct.pack_into("<I", data, offset, value)


def put_u64(data: bytearray, offset: int, value: int) -> None:
    struct.pack_into("<Q", data, offset, value)


def crc32(data: bytes) -> int:
    return binascii.crc32(data) & 0xFFFFFFFF


def read_atf_payload(path: Path) -> bytes:
    package = path.read_bytes()
    if len(package) < PART_HEADER_SIZE:
        raise ValueError(f"ATF package is shorter than a partition header: {path}")
    if get_u32(package, 0) != PART_MAGIC:
        raise ValueError(f"ATF input is not a MediaTek partition package: {path}")
    payload_size = get_u32(package, 4)
    payload = package[PART_HEADER_SIZE:]
    if payload_size != len(payload):
        raise ValueError(
            f"ATF package dsize 0x{payload_size:x} != payload length 0x{len(payload):x}"
        )
    if len(payload) < 0x240:
        raise ValueError("ATF package payload has no complete inner header")
    return payload


def bundle_lbas(path: Path) -> int:
    size = path.stat().st_size
    return (size + ALIGNMENT - 1) // ALIGNMENT


def build_stage(loader_path: Path, atf_path: Path, bundle_path: Path) -> bytes:
    loader = loader_path.read_bytes()
    atf = read_atf_payload(atf_path)
    lbas = bundle_lbas(bundle_path)

    if not loader or len(loader) > LOADER_MAX_BYTES:
        raise ValueError(f"Loader size is out of range: 0x{len(loader):x}")
    if len(atf) > ATF_MAX_BYTES:
        raise ValueError(f"ATF payload size is out of range: 0x{len(atf):x}")
    if lbas == 0 or lbas > BUNDLE_MAX_LBAS:
        raise ValueError(f"bundle is too large for the raw-stage contract: {lbas} LBAs")

    loader_offset = HEADER_SIZE
    atf_offset = align_up(loader_offset + len(loader))
    total_size = align_up(atf_offset + len(atf))
    if total_size > MAX_STAGE_BYTES:
        raise ValueError(f"stage is too large: 0x{total_size:x}")

    stage = bytearray(total_size)
    header = bytearray(HEADER_SIZE)
    header[: len(MAGIC)] = MAGIC
    put_u32(header, 0x08, VERSION)
    put_u32(header, 0x0C, HEADER_SIZE)
    put_u32(header, FLAGS_OFFSET, 0)
    put_u64(header, TOTAL_SIZE_OFFSET, total_size)
    put_u64(header, LOADER_OFFSET, loader_offset)
    put_u64(header, LOADER_SIZE, len(loader))
    put_u64(header, LOADER_LOAD, LOADER_ADDRESS)
    put_u64(header, LOADER_ENTRY, LOADER_ADDRESS)
    put_u32(header, LOADER_CRC, crc32(loader))
    put_u64(header, ATF_OFFSET, atf_offset)
    put_u64(header, ATF_SIZE, len(atf))
    put_u64(header, ATF_LOAD, ATF_LOAD_ADDRESS)
    put_u64(header, ATF_ENTRY, ATF_ENTRY_ADDRESS)
    put_u32(header, ATF_CRC, crc32(atf))
    put_u64(header, BUNDLE_LBA, BUNDLE_USER_LBA)
    put_u64(header, BUNDLE_LBAS, lbas)

    stage[loader_offset : loader_offset + len(loader)] = loader
    stage[atf_offset : atf_offset + len(atf)] = atf
    put_u32(header, HEADER_CRC, 0)
    put_u32(header, HEADER_CRC, crc32(header))
    stage[:HEADER_SIZE] = header
    return bytes(stage)


def parse_stage(data: bytes) -> dict[str, int]:
    if len(data) < HEADER_SIZE:
        raise ValueError("stage is shorter than its header")
    if data[: len(MAGIC)] != MAGIC:
        raise ValueError("stage magic mismatch")
    if get_u32(data, 0x08) != VERSION or get_u32(data, 0x0C) != HEADER_SIZE:
        raise ValueError("unsupported raw-stage version or header size")
    if get_u32(data, FLAGS_OFFSET) != 0:
        raise ValueError("unknown raw-stage flags")

    expected_crc = get_u32(data, HEADER_CRC)
    header = bytearray(data[:HEADER_SIZE])
    put_u32(header, HEADER_CRC, 0)
    if crc32(header) != expected_crc:
        raise ValueError("raw-stage header CRC mismatch")

    fields = {
        "total_size": get_u64(data, TOTAL_SIZE_OFFSET),
        "loader_offset": get_u64(data, LOADER_OFFSET),
        "loader_size": get_u64(data, LOADER_SIZE),
        "loader_load": get_u64(data, LOADER_LOAD),
        "loader_entry": get_u64(data, LOADER_ENTRY),
        "loader_crc": get_u32(data, LOADER_CRC),
        "atf_offset": get_u64(data, ATF_OFFSET),
        "atf_size": get_u64(data, ATF_SIZE),
        "atf_load": get_u64(data, ATF_LOAD),
        "atf_entry": get_u64(data, ATF_ENTRY),
        "atf_crc": get_u32(data, ATF_CRC),
        "bundle_lba": get_u64(data, BUNDLE_LBA),
        "bundle_lbas": get_u64(data, BUNDLE_LBAS),
    }
    total = fields["total_size"]
    if total < HEADER_SIZE or total > MAX_STAGE_BYTES or total % ALIGNMENT:
        raise ValueError("raw-stage total size is invalid")
    if total != len(data):
        raise ValueError(
            f"raw-stage total size 0x{total:x} != file size 0x{len(data):x}"
        )

    for name, maximum in (("loader", LOADER_MAX_BYTES), ("atf", ATF_MAX_BYTES)):
        offset = fields[f"{name}_offset"]
        size = fields[f"{name}_size"]
        if (
            offset < HEADER_SIZE
            or offset % ALIGNMENT
            or size == 0
            or size > maximum
            or offset + size > total
        ):
            raise ValueError(f"{name} range is invalid")
    if (
        fields["loader_offset"] < fields["atf_offset"] + fields["atf_size"]
        and fields["atf_offset"] < fields["loader_offset"] + fields["loader_size"]
    ):
        raise ValueError("Loader and ATF ranges overlap")
    if fields["loader_load"] != LOADER_ADDRESS or fields["loader_entry"] != LOADER_ADDRESS:
        raise ValueError("Loader address contract mismatch")
    if fields["atf_load"] != ATF_LOAD_ADDRESS or fields["atf_entry"] != ATF_ENTRY_ADDRESS:
        raise ValueError("ATF address contract mismatch")
    if fields["bundle_lba"] != BUNDLE_USER_LBA or not 0 < fields["bundle_lbas"] <= BUNDLE_MAX_LBAS:
        raise ValueError("boot-bundle address contract mismatch")
    if crc32(data[fields["loader_offset"] : fields["loader_offset"] + fields["loader_size"]]) != fields["loader_crc"]:
        raise ValueError("Loader CRC mismatch")
    if crc32(data[fields["atf_offset"] : fields["atf_offset"] + fields["atf_size"]]) != fields["atf_crc"]:
        raise ValueError("ATF CRC mismatch")
    return fields


def command_build(args: argparse.Namespace) -> int:
    output = Path(args.output)
    if output.exists() and not args.force:
        raise ValueError(f"output exists; use --force explicitly: {output}")
    data = build_stage(Path(args.loader), Path(args.atf_package), Path(args.bundle))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(data)
    fields = parse_stage(data)
    print(f"raw stage: {output}")
    print(f"size: 0x{fields['total_size']:x} ({fields['total_size']} bytes)")
    print(f"Loader: offset=0x{fields['loader_offset']:x} size=0x{fields['loader_size']:x} crc=0x{fields['loader_crc']:08x}")
    print(f"ATF: offset=0x{fields['atf_offset']:x} size=0x{fields['atf_size']:x} crc=0x{fields['atf_crc']:08x}")
    print(f"bundle: user LBA=0x{fields['bundle_lba']:x} LBAs=0x{fields['bundle_lbas']:x}")
    return 0


def command_inspect(args: argparse.Namespace) -> int:
    data = Path(args.stage).read_bytes()
    fields = parse_stage(data)
    print(f"stage: {args.stage}")
    for key, value in fields.items():
        print(f"{key}={value:#x}")
    print("verify=ok")
    return 0


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    build = subparsers.add_parser("build", help="build a raw stage")
    build.add_argument("--loader", required=True, help="headerless Loader binary")
    build.add_argument("--atf-package", required=True, help="full MediaTek ATF partition package")
    build.add_argument("--bundle", required=True, help="raw boot bundle used by the Loader")
    build.add_argument("--output", required=True)
    build.add_argument("--force", action="store_true")
    build.set_defaults(function=command_build)

    inspect = subparsers.add_parser("inspect", aliases=["verify"], help="verify a raw stage")
    inspect.add_argument("stage")
    inspect.set_defaults(function=command_inspect)
    return parser


def main() -> int:
    parser = make_parser()
    args = parser.parse_args()
    try:
        return args.function(args)
    except (OSError, ValueError, struct.error) as error:
        print(f"raw-stage.py: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
