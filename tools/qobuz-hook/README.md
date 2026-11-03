# libassert_hook.so для нативного Qobuz Connect на DigiNeo

Исходники — из DigiFox (`ext_tree/package/qobuz-connect-assert-hook`, MIT):
- `assert_hook.c` — перезапуск плеера при известном assert SDK (код выхода 42);
- `qbz_ctl.c` — кнопки плеера: FIFO `/tmp/qobuz_ctl` (toggle | next | prev | pause | resume).

Собирается GitHub Actions (`.github/workflows/qobuz-hook.yml`) кросс-компилятором armhf;
готовый файл кладётся в пакет обновления как `/opt/qobuz-connect/rt/libassert_hook.so`.
