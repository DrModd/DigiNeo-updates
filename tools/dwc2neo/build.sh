#!/bin/bash
# Сборка dwc2neo.ko — копии драйвера USB dwc2 из ядра 5.15.93 с изменёнными параметрами
# (LPM выключен, DMA без дескрипторов) под ядро DigiNeo 5.15.93-PureOS (NanoPi NEO3).
# Запуск: build.sh <каталог с исходниками ядра linux-5.15.93> <каталог результата>
# Лицензия: GPL-2.0 (код драйвера — Synopsys/Linux kernel, правки — DrModd/DigiNeo).
set -euo pipefail
K=$(readlink -f "$1"); OUT=$(readlink -f "$2"); HERE=$(dirname "$(readlink -f "$0")")
M=$K/dwc2neo-mod
rm -rf "$M"; mkdir -p "$M" "$OUT"
cp "$K"/drivers/usb/dwc2/{core.c,core.h,core_intr.c,platform.c,drd.c,params.c,debugfs.c,gadget.c,hcd.c,hcd.h,hcd_intr.c,hcd_queue.c,hcd_ddma.c,hw.h} "$M"/
cd "$M"
# 1) свои имена: встроенный драйвер остаётся в ядре, наш регистрируется как dwc2neo
sed -i -E '/^EXPORT_SYMBOL(_GPL)?\(.*\);/d' *.c
sed -i 's/dwc2_driver_name\[\] = "dwc2"/dwc2_driver_name[] = "dwc2neo"/' platform.c
sed -i 's/MODULE_ALIAS("platform:dwc2")/MODULE_ALIAS("platform:dwc2neo")/' platform.c
grep -q '"dwc2neo"' platform.c
# 2) параметры: без LPM и без DMA-дескрипторов (из-за них ep0 не взводится на RK3328)
python3 - <<'PY'
import re
s = open('params.c').read()
pat = '\tdwc2_check_params(hsotg);'
assert s.count(pat) == 1, 'dwc2_check_params not found'
s = s.replace(pat, '''\t/* dwc2neo: DigiNeo, NanoPi NEO3 */
\thsotg->params.lpm = false;
\thsotg->params.lpm_clock_gating = false;
\thsotg->params.hird_threshold_en = false;
\thsotg->params.besl = false;
\thsotg->params.g_dma_desc = false;
\thsotg->params.dma_desc_enable = false;
\thsotg->params.dma_desc_fs_enable = false;
''' + pat)
open('params.c', 'w').write(s)
PY
cat > Makefile <<'MK'
obj-m := dwc2neo.o
dwc2neo-y := core.o core_intr.o platform.o drd.o params.o debugfs.o gadget.o hcd.o hcd_intr.o hcd_queue.o hcd_ddma.o
MK
make -C "$K" ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- M="$M" KCFLAGS="-Wno-error" modules
cp dwc2neo.ko "$OUT"/
echo "== modinfo"; ( modinfo "$OUT/dwc2neo.ko" 2>/dev/null || true ) | grep -E "vermagic|name|license|alias" | head
