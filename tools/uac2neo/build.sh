#!/bin/bash
# Сборка u_audio.ko и usb_f_uac2.ko — UAC2-гаджет из ядра Rockchip 6.1 с доработками DigiFox
# (ext_tree/patches/linux_rv1106.patch из DrModd/DigiFox: несколько частот, DSD, /sys/class/u_audio,
#  обратная связь) под ядро DigiNeo 5.15.93-PureOS (NanoPi NEO3). Штатные модули 5.15 умеют одну частоту.
# Запуск: build.sh <каталог с исходниками ядра linux-5.15.93> <каталог результата>
# Лицензия: GPL-2.0 (код — Linux kernel / Rockchip, правки — DigiFox).
set -euo pipefail
K=$(readlink -f "$1"); OUT=$(readlink -f "$2"); HERE=$(dirname "$(readlink -f "$0")")
M=$K/uac2neo-mod
rm -rf "$M"; mkdir -p "$M" "$OUT"
cp "$HERE"/src/* "$M"/
cd "$M"
cat > Makefile <<'MK'
obj-m := u_audio.o usb_f_uac2.o
usb_f_uac2-y := f_uac2.o
MK
make -C "$K" ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- M="$M" KCFLAGS="-Wno-error" modules
cp u_audio.ko usb_f_uac2.ko "$OUT"/
echo "== modinfo"; for f in u_audio usb_f_uac2; do ( modinfo "$OUT/$f.ko" 2>/dev/null || true ) | grep -E "vermagic|^name|license|depends" ; done
