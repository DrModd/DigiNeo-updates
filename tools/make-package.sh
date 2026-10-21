#!/bin/bash
# make-package.sh <версия> <каталог-с-пакетом> "<что нового>"
# В каталоге: files/ (дерево относительно /), необязательный post.sh. VERSION создаётся сам.
# Результат: packages/digineo-<версия>.tar.gz и обновлённый manifest.txt (будет предложено в каталоге обновлений).
set -euo pipefail
ver="$1"; src="$2"; notes="${3:-}"
[[ "$ver" =~ ^[0-9]+(\.[0-9]+){1,3}$ ]] || { echo "неверная версия: $ver"; exit 1; }
cd "$(dirname "$0")/.."
echo "$ver" > "$src/VERSION"
out="packages/digineo-$ver.tar.gz"; mkdir -p packages
items=(VERSION); [ -d "$src/files" ] && items+=(files); [ -f "$src/post.sh" ] && items+=(post.sh)
tar --sort=name --mtime=@0 --owner=0 --group=0 --numeric-owner -cf - -C "$src" "${items[@]}" | gzip -9n > "$out"
sha=$(sha256sum "$out" | cut -d' ' -f1)
printf 'version=%s\nfile=%s\nsha256=%s\nsize=%s\nnotes=%s\n' "$ver" "$out" "$sha" "$(stat -c %s "$out")" "${notes//$'\n'/ }" > manifest.txt
echo "Готово: $out"; cat manifest.txt
echo "Дальше: git add -A && git commit && git push"
