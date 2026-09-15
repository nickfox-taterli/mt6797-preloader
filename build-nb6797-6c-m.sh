#!/bin/sh

# Rebuild the X20-96 MT6797 Preloader from the separately managed vendor tree.
# The output is deliberately one rolling artifact directory; versioning is done
# by Git commits and the generated manifest, not by copying large build trees.

set -eu

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
workspace_dir=$(CDPATH= cd -- "$script_dir/../../.." && pwd)
vendor_root=${VENDOR_ROOT:-$workspace_dir/vendor}
source_dir=$vendor_root/mediatek/proprietary/bootable/bootloader/preloader
source_commit=be545c074ea484d0d51433b6bbf1acc032affe5a
build_dir=${PRELOADER_OUT:-$workspace_dir/out/preloader-source-nb6797-6c-m-current}
tool_dir=$build_dir/tools
cross_compile=${CROSS_COMPILE:-/usr/bin/arm-none-eabi-}
target_variant=${TARGET_BUILD_VARIANT:-eng}
jobs=${JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || printf '1')}
compat_flags=${MTK_CFLAGS:-'-std=gnu89 -Wno-error=implicit-function-declaration -Wno-error=incompatible-pointer-types -Wno-error=int-conversion -Wno-error=return-mismatch'}

if [ -n "${BUILD_TIME:-}" ]; then
	build_time=$BUILD_TIME
elif [ -n "${SOURCE_DATE_EPOCH:-}" ]; then
	build_time=$(date -u -d "@$SOURCE_DATE_EPOCH" +%Y%m%d-%H%M%S)
else
	build_time=$(date +%Y%m%d-%H%M%S)
fi

die()
{
	echo "build-nb6797-6c-m.sh: $*" >&2
	exit 1
}

[ -d "$vendor_root/.git" ] || die "VENDOR_ROOT is not a Git checkout: $vendor_root"
[ -f "$source_dir/Makefile" ] || die "Preloader source is missing: $source_dir"

actual_commit=$(git -C "$vendor_root" rev-parse HEAD)
[ "$actual_commit" = "$source_commit" ] || die "vendor commit is $actual_commit, expected $source_commit"

if ! git -C "$vendor_root" diff --quiet || ! git -C "$vendor_root" diff --cached --quiet; then
	[ "${ALLOW_DIRTY_SOURCE:-no}" = yes ] || die "vendor checkout is dirty; set ALLOW_DIRTY_SOURCE=yes only for an explicit probe"
	echo "warning: building a dirty vendor checkout" >&2
fi

command -v "${cross_compile}gcc" >/dev/null 2>&1 || die "missing compiler: ${cross_compile}gcc"

if [ -e "$build_dir/source-commit" ]; then
	stored_commit=$(sed -n '1p' "$build_dir/source-commit")
	[ "$stored_commit" = "$source_commit" ] || die "output belongs to $stored_commit; choose a new PRELOADER_OUT"
else
	mkdir -p "$build_dir"
	printf '%s\n' "$source_commit" > "$build_dir/source-commit"
fi

mkdir -p "$tool_dir/SignTool" "$tool_dir/CipherTool"

sign_tool=$vendor_root/mediatek/proprietary/scripts/sign-image/SignTool_PL
cipher_tool=$vendor_root/mediatek/proprietary/scripts/ciphertool/CipherTool
[ -x "$sign_tool" ] || die "missing SignTool_PL: $sign_tool"
[ -x "$cipher_tool" ] || die "missing CipherTool: $cipher_tool"

ensure_link()
{
	link=$1
	target=$2
	if [ -L "$link" ]; then
		[ "$(readlink "$link")" = "$target" ] || die "unexpected existing link: $link"
	elif [ -e "$link" ]; then
		die "unexpected existing tool path: $link"
	else
		ln -s "$target" "$link"
	fi
}

ensure_link "$tool_dir/SignTool/SignTool_PL" "$sign_tool"
ensure_link "$tool_dir/CipherTool/CipherTool" "$cipher_tool"

make -C "$source_dir" \
	MTK_PROJECT=nb6797_6c_m \
	PRELOADER_OUT="$build_dir" \
	ROOTDIR="$vendor_root" \
	TOOL_PATH="$tool_dir" \
	CROSS_COMPILE="$cross_compile" \
	TARGET_BUILD_VARIANT="$target_variant" \
	BUILD_TIME="$build_time" \
	MTK_CFLAGS="$compat_flags" \
	-j"$jobs" all

artifact=$build_dir/bin/preloader_nb6797_6c_m.bin
linked_artifact=$build_dir/bin/preloader_nb6797_6c_m_LINKED.bin
raw_artifact=$build_dir/bin/preloader_nb6797_6c_m_NO_GFH.bin
[ -f "$artifact" ] || die "build completed without $artifact"
[ -f "$linked_artifact" ] || die "build completed without $linked_artifact"
[ -f "$raw_artifact" ] || die "build completed without $raw_artifact"

{
	printf 'vendor_root=%s\n' "$vendor_root"
	printf 'vendor_commit=%s\n' "$actual_commit"
	printf 'project=nb6797_6c_m\n'
	printf 'target_build_variant=%s\n' "$target_variant"
	printf 'cross_compile=%s\n' "$cross_compile"
	printf 'build_time=%s\n' "$build_time"
	printf 'mtk_cflags=%s\n' "$compat_flags"
	printf 'board_dram=H9CKNNNBKTMRPR-NUH\n'
	printf 'artifact_sha256='; sha256sum "$artifact" | awk '{print $1}'
	printf 'linked_artifact_sha256='; sha256sum "$linked_artifact" | awk '{print $1}'
	printf 'raw_artifact_sha256='; sha256sum "$raw_artifact" | awk '{print $1}'
} > "$build_dir/build-manifest.txt"

printf '%s\n' "Preloader: $artifact"
cat "$build_dir/build-manifest.txt"
