#!/bin/sh

# Rebuild the X20-96 MT6797 Preloader from the source checked into this
# component.  The output is deliberately one rolling artifact directory;
# versioning is done by Git commits and the generated manifest.

set -eu

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
workspace_dir=$(CDPATH= cd -- "$script_dir/../../.." && pwd)
component_dir=$script_dir
source_dir=$component_dir/source
build_dir=${PRELOADER_OUT:-$workspace_dir/out/preloader-source-nb6797-6c-m-current}
cross_compile=${CROSS_COMPILE:-/usr/bin/arm-none-eabi-}
target_variant=${TARGET_BUILD_VARIANT:-eng}
jobs=${JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || printf '1')}
compat_flags=${MTK_CFLAGS:-'-std=gnu89 -Wno-error=implicit-function-declaration -Wno-error=incompatible-pointer-types -Wno-error=int-conversion -Wno-error=return-mismatch'}
uart_rescue=${X20_UART_RESCUE:-no}
case "$uart_rescue" in
	yes) compat_flags="$compat_flags -DCFG_X20_UART_RESCUE=1" ;;
	no) ;;
	*) echo 'X20_UART_RESCUE must be yes or no' >&2; exit 1 ;;
esac

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

[ -d "$component_dir/.git" ] || die "component is not a Git checkout: $component_dir"
[ -f "$source_dir/Makefile" ] || die "Preloader source is missing: $source_dir"

component_commit=$(git -C "$component_dir" rev-parse HEAD)
source_tree=$(git -C "$component_dir" rev-parse HEAD:source)

if ! git -C "$component_dir" diff --quiet -- source || \
	! git -C "$component_dir" diff --cached --quiet -- source; then
	[ "${ALLOW_DIRTY_SOURCE:-no}" = yes ] || die "component source is dirty; set ALLOW_DIRTY_SOURCE=yes only for an explicit probe"
	echo "warning: building a dirty component source tree" >&2
fi

command -v "${cross_compile}gcc" >/dev/null 2>&1 || die "missing compiler: ${cross_compile}gcc"

if [ -e "$build_dir/source-commit" ]; then
	stored_tree=$(sed -n '1p' "$build_dir/source-commit")
	if [ "$stored_tree" != "$source_tree" ]; then
		# Rebuild the rolling input instead of accumulating per-commit outputs.
		rm -rf "$build_dir/obj" "$build_dir/inc" "$build_dir/bin"
		printf '%s\n' "$source_tree" > "$build_dir/source-commit"
	fi
else
	mkdir -p "$build_dir"
	printf '%s\n' "$source_tree" > "$build_dir/source-commit"
fi

# This vendor Makefile does not track compiler flags as object dependencies.
# Reuse one output directory, but never mix normal and UART-rescue objects.
if [ ! -f "$build_dir/compiler-flags" ] || \
	[ "$(cat "$build_dir/compiler-flags")" != "$compat_flags" ]; then
	rm -rf "$build_dir/obj"
	printf '%s\n' "$compat_flags" > "$build_dir/compiler-flags"
fi

make -C "$source_dir" \
	MTK_PROJECT=nb6797_6c_m \
	PRELOADER_OUT="$build_dir" \
	ROOTDIR="$component_dir" \
	TOOL_PATH="$source_dir/tools" \
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
	printf 'component_dir=%s\n' "$component_dir"
	printf 'component_commit=%s\n' "$component_commit"
	printf 'source_tree=%s\n' "$source_tree"
	printf 'project=nb6797_6c_m\n'
	printf 'target_build_variant=%s\n' "$target_variant"
	printf 'cross_compile=%s\n' "$cross_compile"
	printf 'build_time=%s\n' "$build_time"
	printf 'mtk_cflags=%s\n' "$compat_flags"
	printf 'uart_rescue=%s\n' "$uart_rescue"
	printf 'board_target=nb6797_6c_m\n'
	printf 'board_qualification=experimental_not_proven_amt6797_64_open\n'
	printf 'board_dram=H9CKNNNBKTMRPR-NUH\n'
	printf 'fallback_preloader=prebuilt/preloader_amt6797_64_open.bin\n'
	printf 'fallback_preloader_sha256=5e50d7c8820d24e08f6767911c7e53b39661cc5e90a2a29670236be8e7b3f2d7\n'
	printf 'artifact_sha256='; sha256sum "$artifact" | awk '{print $1}'
	printf 'linked_artifact_sha256='; sha256sum "$linked_artifact" | awk '{print $1}'
	printf 'raw_artifact_sha256='; sha256sum "$raw_artifact" | awk '{print $1}'
} > "$build_dir/build-manifest.txt"

printf '%s\n' "Preloader: $artifact"
cat "$build_dir/build-manifest.txt"
