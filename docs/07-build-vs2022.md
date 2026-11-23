# 07. Сборка в Visual Studio 2022 (MSVC + CMake)

## 1. Что установлено и проверено

| Компонент | Версия | Откуда |
|---|---|---|
| Visual Studio Community 2022 | 17.14.14 | `C:\Program Files\Microsoft Visual Studio\2022\Community` |
| MSVC (`cl.exe`) | 19.44 (toolset 14.44) | `VC\Tools\MSVC\14.44.35207` |
| Windows SDK | 10.0.26100 | `C:\Program Files (x86)\Windows Kits\10` |
| CMake | 3.31.6 | внутри VS (`…\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe`) |
| MSBuild | 17.14.23 | внутри VS |
| protoc | 33.6 | `C:\Program Files\Protobuf\bin\protoc.exe` (только для перегенерации) |

## 2. Структура проекта

```
VisualStudio\FrezaEgm\
├── CMakeLists.txt          цели: nanopb, freza_core, freza_cam, FrezaEgm, FrezaEgm_selftest
├── CMakePresets.json       пресеты VS (Ninja, x64-debug / x64-release / x64-cam-release)
├── cmake\Cam.cmake         поиск OpenCAMLib в out\deps и копирование libocl.dll
├── src\                    main.cpp, app_window.cpp, cam.cpp, trajectory.cpp, csv_io.cpp,
│                           egm_client.cpp, text_io.cpp, selftest.cpp
├── include\                app_window.h, cam.h, trajectory.h, csv_io.h, egm_client.h, text_io.h
├── generated\              egm3.pb.c, egm3.pb.h   ← результат nanopb-генерации
├── third_party\nanopb\     pb.h, pb_common.c/.h, pb_encode.c/.h, pb_decode.c/.h
├── data\                   trajectory.gcode, trajectory.xyz, points.csv, cam\Primer1.STL
├── tests\                  FrezaEgm_regression, FrezaEgm_cam_cli, protected.cmake
├── out\deps\               собранные OpenCAMLib (ocl-Release) и заголовки Boost
└── tools\                  generate_proto.ps1, build_opencamlib.ps1
```

Цели CMake:

| Цель | Что это |
|---|---|
| `nanopb` | статическая библиотека: `pb_*.c` + `egm3.pb.c` (компилируются как C) |
| `freza_core` | статическая библиотека логики: траектория, CSV, EGM-клиент, файловый ввод-вывод |
| `freza_cam` | STL/CAM: `cam.cpp`; при `FREZA_ENABLE_CAM=ON` линкуется с OpenCAMLib |
| `FrezaEgm` | GUI-приложение (`WIN32`, подсистема Windows, `WinMain`) |
| `FrezaEgm_selftest` | консольный самотест (первый из трёх тестов `ctest`) |
| `FrezaEgm_regression` | регрессия: парсеры, CSV, сериализация, CAM (73/73 ON, 46/46 OFF) |
| `FrezaEgm_cam_cli` | офлайн-расчёт STL → CSV без окна и без UDP |

## 3. Сборка из Visual Studio

1. `Файл → Открыть → Папка…` и выбрать `D:\developments\DSH_workspaces\freza_sirius\VisualStudio\FrezaEgm`
   (папка, где лежит `CMakeLists.txt`).
2. VS прочитает `CMakePresets.json` и выполнит конфигурацию. В выпадающем списке
   конфигураций выберите `x64-release` (или `x64-debug`).
3. `Сборка → Собрать все` (`Ctrl+Shift+B`).
4. Готовый файл: `out\build\x64-release\FrezaEgm.exe`
   (в VS: правый клик по цели `FrezaEgm` → «Открыть папку с файлом»).

**STL/CAM в VS.** Если зависимость уже собрана (`out\deps\ocl-Release`), `x64-release`
включает CAM автоматически — отдельный пресет `x64-cam-release` нужен только для
явного указания путей. `libocl.dll` копируется рядом с exe. Если зависимости нет,
CAM остаётся выключенным, а окно сообщает об этом при нажатии STL. Порядок сборки
зависимости — в `13-stl-cam.md`, §«Локальная проверка на Windows x64».

Если VS не находит CMake — установите компонент «Средства C++ CMake для Windows»
в Visual Studio Installer (обычно ставится вместе с «Разработка классических
приложений на C++»).

> Файлы `CMakeCache.txt` хранят **абсолютные** пути. Каталог сборки, скопированный
> из другой папки (например, после переноса проекта из `pullGitHub` в корень рабочей области), переконфигурировать
> нельзя: CMake пишет «current CMakeCache.txt directory … is different than the
> directory … where CMakeCache.txt was created». Такой каталог нужно удалить и
> собрать заново — исходники и `out\deps` при этом не страдают.

## 4. Сборка из командной строки (проверено)

```powershell
cd D:\developments\DSH_workspaces\freza_sirius\VisualStudio\FrezaEgm

$cmake = "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
$ctest = "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\ctest.exe"

# конфигурация под MSVC (генератор VS2022, x64); CAM включится сам, если собран out\deps
& $cmake -S . -B build-vs -G "Visual Studio 17 2022" -A x64

# сборка
& $cmake --build build-vs --config Release

# проверка
& $ctest --test-dir build-vs -C Release --output-on-failure
```

> На этой машине `cmake` и `ctest` **не в `PATH`** — они лежат внутри VS2022 по
> указанным выше путям. Тот же `cmake.exe` находит и `tools\build_opencamlib.ps1`
> (через `vswhere`), поэтому его можно запускать без настройки `PATH`.

Результат: `build-vs\Release\FrezaEgm.exe` и `build-vs\Release\FrezaEgm_selftest.exe`.
Каталог `build-vs` — обычное дерево сборки, его можно удалить и пересоздать.

> Почему два разных каталога (`out/build/...` и `build-vs/...`)? VS использует
> пресеты с генератором **Ninja**, а команда выше — генератор **Visual Studio**
> (тот же компилятор MSVC, но другой драйвер сборки). Оба варианта дают рабочий exe.

## 5. Генерация кода из `.proto`

Сгенерированные `generated/egm3.pb.c/.h` уже лежат в проекте — для сборки ничего
генерировать не нужно. Перегенерация нужна только если меняли `Egm/egm3.proto`:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\generate_proto.ps1
```

Скрипт требует `protoc` ≥ 3.15, Python 3 с пакетом `protobuf`; генератор nanopb уже
лежит в проекте (`third_party\nanopb\generator`). Пути задаются параметрами:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\generate_proto.ps1 `
     -Protoc "C:\Program Files\Protobuf\bin\protoc.exe"
```

> Флаг `-ExecutionPolicy Bypass` нужен потому, что на этой машине выполнение
> скриптов запрещено политикой Windows (`Restricted`). Он действует только на
> один запуск и ничего в системе не меняет. Если установлен PowerShell 7,
> можно короче: `pwsh -File .\tools\generate_proto.ps1`.

Почему файлы закоммичены в проект, а не генерируются на каждой сборке: так сборка
не зависит от наличия `protoc` и Python на машине, а результат воспроизводим.

## 6. Отладка

* Точки останова в `app_window.cpp` и `egm_client.cpp` работают как обычно.
* Рабочий каталог и аргументы задаются в `launch.vs.json`
  (правый клик по `CMakeLists.txt` → «Параметры отладки и запуска»).
  Приложению каталог с данными не нужен: путь к `data/` «вшит» в
  `FREZA_DATA_DIR` при сборке.
* Чтобы увидеть обмен пакетами: Wireshark, фильтр `udp.port == 6510`.
* Для проверки без окна есть `FrezaEgm_selftest.exe` (см. `08-testing.md`).

## 7. Частые проблемы сборки

| Симптом | Причина / решение |
|---|---|
| `LNK2005: main уже определён` | в проект попал старый `Egm/src/main_one_joint_move.cpp` — он не должен входить в цель (в `CMakeLists.txt` перечислены только `src/*.cpp` проекта) |
| `не удаётся открыть файл egm.pb.h` | не хватает `generated/egm3.pb.h` — выполните `tools\generate_proto.ps1` или восстановите файлы из репозитория |
| «кракозябры» в русских строках | потерян флаг `/utf-8` (он задан в `CMakeLists.txt` для MSVC) |
| `C4819`/`warning C4819` | файл сохранён не в UTF-8; пересохраните в UTF-8 |
| exe запускается и сразу закрывается | запущен Debug-вариант без консоли? смотрите журнал в окне; для диагностики — `FrezaEgm_selftest` |
| `MSB8020` / не найден toolset | не установлен компонент «C++» в VS Installer |
| nanopb ругается `PB_PROTO_HEADER_VERSION` | `pb.h` и сгенерированный код из разных версий nanopb — перегенерируйте код |
| `CMake Error: The current CMakeCache.txt directory … is different` | каталог сборки скопирован из другого пути (например, был `…\pullGitHub\VisualStudio\…`, проект перенесён в `…\freza_sirius\VisualStudio\…`) — удалите этот каталог сборки и сконфигурируйте заново |
| `'cmake' не распознано` / `'ctest' не распознано` | они не в `PATH`; вызывайте по полному пути из VS2022 (см. §4) или запускайте сборку из VS |
| после включения CAM ошибка `FREZA_ENABLE_CAM=ON requires OpenCAMLib` | зависимость не собрана: `tools\build_opencamlib.ps1` (см. `13-stl-cam.md`) |
| в окне «CAM не собран», хотя зависимость есть | сборка сделана старым кэшем с `FREZA_ENABLE_CAM=OFF`; переконфигурируйте с `-DFREZA_ENABLE_CAM=ON` или удалите каталог сборки |
| `Policy CMP0167 is not set` при configure | предупреждение устаревшего `FindBoost` внутри OpenCAMLib; в проекте политика уже задана, сообщение остаётся только при старом `CMakeLists.txt` |
