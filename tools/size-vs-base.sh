#!/usr/bin/env bash
# The size gate against main (specs/twelve-fet.md section 4.3): build the one-motor images at HEAD
# and at a base commit, and fail if any image's flashed span, `.hotcode` size or static RAM
# (.data + .bss + .uninit) is larger at HEAD. Per image it prints the three figures at both commits
# and either "byte-identical" (the two flashed binaries are equal) or the `nm -S` symbol diff, so a
# change that alters an image is visible in the log even when it passes.
#
#   tools/size-vs-base.sh <base-commit> [image-alias...]
#
# The aliases are motorcontroller/.cargo/config.toml's (default: image-f103 image-f130 image-mm32).
# The base is checked out as a detached worktree in `$SIZE_BASE_DIR` (default: a sibling of the
# repository, `<repo>-size-base`) and built there with the same alias, so both sides come out of the
# same invocation. It must be outside this checkout: cargo merges every `.cargo/config.toml` up the
# directory tree, and a base nested in this one would run each alias twice over. CI passes the merge
# base with origin/main, or the first parent on main itself.
set -euo pipefail

base=${1:?usage: tools/size-vs-base.sh <base-commit> [image-alias...]}
shift
aliases=("$@")
[ ${#aliases[@]} -gt 0 ] || aliases=(image-f103 image-f130 image-mm32)

root=$(git rev-parse --show-toplevel)
base=$(git -C "$root" rev-parse --verify "$base^{commit}")
work=${SIZE_BASE_DIR:-$(dirname "$root")/$(basename "$root")-size-base}
src=$work/src

# The alias's chip and target triple, as the alias itself names them.
chip_of() {
	case $1 in
	image-f103) echo "gd32f103c8 thumbv7m-none-eabi" ;;
	image-f130) echo "gd32f130c8 thumbv7m-none-eabi" ;;
	image-mm32) echo "mm32spin05 thumbv6m-none-eabi" ;;
	*)
		echo "size-vs-base: no chip for alias $1" >&2
		return 1
		;;
	esac
}

if [ -d "$src" ]; then
	git -C "$root" worktree remove --force "$src" 2>/dev/null || rm -rf "$src"
fi
git -C "$root" worktree prune
mkdir -p "$work"
git -C "$root" worktree add --detach "$src" "$base" >/dev/null
trap 'git -C "$root" worktree remove --force "$src" >/dev/null 2>&1 || true' EXIT

for a in "${aliases[@]}"; do
	(cd "$root/motorcontroller" && cargo "$a")
	(cd "$src/motorcontroller" && cargo "$a")
done

sec() { arm-none-eabi-size -A "$1" | awk -v n="$2" '$1==n {t+=$2} END{print t+0}'; }
static_ram() { arm-none-eabi-size -A "$1" | awk '$1==".data" || $1==".bss" || $1==".uninit" {t+=$2} END{print t+0}'; }

fail=0
for a in "${aliases[@]}"; do
	read -r chip triple < <(chip_of "$a")
	name=mainboard-$chip
	h=$root/motorcontroller/target/$chip/$triple/release/firmware
	b=$src/motorcontroller/target/$chip/$triple/release/firmware
	arm-none-eabi-objcopy -O binary "$h" "$work/$name-head.bin"
	arm-none-eabi-objcopy -O binary "$b" "$work/$name-base.bin"
	hs=$(wc -c <"$work/$name-head.bin" | tr -d ' ')
	bs=$(wc -c <"$work/$name-base.bin" | tr -d ' ')
	hh=$(sec "$h" .hotcode)
	bh=$(sec "$b" .hotcode)
	hr=$(static_ram "$h")
	br=$(static_ram "$b")
	echo "== $name: span ${bs} -> ${hs} B ($((hs - bs))), .hotcode ${bh} -> ${hh} B ($((hh - bh))), static RAM ${br} -> ${hr} B ($((hr - br)))"
	if cmp -s "$work/$name-head.bin" "$work/$name-base.bin"; then
		echo "byte-identical"
	else
		diff <(arm-none-eabi-nm -S --size-sort -C "$b" | awk 'NF==4 {print $4, $2}' | sort) \
			<(arm-none-eabi-nm -S --size-sort -C "$h" | awk 'NF==4 {print $4, $2}' | sort) || true
	fi
	for f in "span $hs $bs" ".hotcode $hh $bh" "static-RAM $hr $br"; do
		read -r what hv bv <<<"$f"
		if [ "$hv" -gt "$bv" ]; then
			echo "FAIL: $name $what grew from $bv B to $hv B against $(git -C "$root" rev-parse --short "$base")" >&2
			fail=1
		fi
	done
done
exit "$fail"
