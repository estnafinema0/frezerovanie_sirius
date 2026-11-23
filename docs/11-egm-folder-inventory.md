# 11. Инвентаризация: что за файлы в рабочей области

Снимок на 23.11.2026. Проект перенесён из `pullGitHub` в корень рабочей области
(`D:\developments\DSH_workspaces\freza_sirius`); git-репозитория в этой копии нет.
Ниже — только то, что реально лежит на диске.

## 1. `Egm/` — схемы EGM

| Файл | Что это | Использование |
|---|---|---|
| `egm3.proto` | Схема EGM в синтаксисе **proto3**: те же имена, пакет и номера полей, что у ABB; `required` убраны, «обязательные» числа помечены `optional` (см. ADR-002) | **Используется** при перегенерации `generated/egm3.pb.*` |
| `egm3.options` | Опции nanopb: `M_STRIP_PACKAGE`, `max_count:6` для `repeated`-полей | **Используется** при генерации |

Историческая схема `egm.proto` (proto2, оригинал ABB), MinGW-прототип (`src/`,
`include/`, `CMakeLists.txt`, `Readme.txt`), `lib/protobuf_installed`, `mingw.7z`,
`cmake-*.msi` и пакеты `.rspag` **в этой копии отсутствуют** — при переносе проекта
их не копировали. Для сборки и работы они не нужны: сгенерированный код уже лежит
в `VisualStudio/FrezaEgm/generated`.

## 2. Корень рабочей области

| Объект | Что это | Использование |
|---|---|---|
| `README.md` | Точка входа: обзор, быстрый старт, структура | **Используется** |
| `PROJECT_STATE.md` | Снимок состояния: что сделано, что проверено, что дальше | **Используется** |
| `docs/` | Документация и контекст задачи (01–14 + справочные PDF ABB) | **Используется** |
| `VisualStudio/FrezaEgm/` | Основной проект: GUI, EGM-клиент, CAM, тесты | **Основной проект** |
| `RobotStudio/` | `ModuleEGM.mod` (RAPID под 6.08, `UCdevice`), `README.md`, `config/`, станция `FrezaEGM/` | **Используется** для симуляции |
| `Egm/` | Только схемы `egm3.proto` и `egm3.options` | Используется при перегенерации |
| `CAD/` | Модели KOMPAS-3D: `kub.m3d`, `kub_1.m3d`, `krug.m3d`, `FC_Spindel.STEP`, `kub.stl` (684 байта), `krug.stl` (0 байт) | Исходные модели пользователя. В CAM принимается только `kub.stl`-подобный замкнутый блок; `krug.stl` пустой, цилиндр `krug.m3d` текущей стратегией не поддерживается |
| `.github/workflows/windows-cam.yml` | CI: сборка MSVC 2022 x64 и CTest в режимах CAM ON/OFF | **Используется** в GitHub-копии репозитория |
| `.tmp/` | Временные файлы | Можно удалить |

**Отсутствуют** (были в старой копии): `SolutionEGM.rspag`, `SolutionEGM_6.08.rspag`,
`SolutionEGMCourse.rspag`. Станция берётся из `RobotStudio/FrezaEGM`, система 6.08
создаётся заново (см. `06-robotstudio-setup.md`).

## 3. Что именно использует проект

```
VisualStudio/FrezaEgm/
├── CMakeLists.txt                 сборка MSVC: nanopb, freza_core, freza_cam, GUI, selftest
├── CMakePresets.json              пресеты VS: x64-release, x64-cam-release, x64-no-cam-release
├── cmake/Cam.cmake                поиск OpenCAMLib в out\deps, копирование libocl.dll
├── include/…                      заголовки (включая cam.h)
├── src/…                          main, app_window, cam, trajectory, csv_io, egm_client, text_io, selftest
├── generated/egm3.pb.c/.h         ← сгенерировано из Egm/egm3.proto
├── third_party/nanopb/            pb.h + pb_encode.c + pb_decode.c + pb_common.c
├── data/trajectory.gcode          демо-траектория (5 точек, квадрат 100×100, Z=600)
├── data/trajectory.xyz            то же в формате «x y z»
├── data/points.csv                результат разбора
├── data/cam/Primer1.STL           контрольная пластина 20×20×0.5 (+ ASCII-копия, CSV, эталоны)
├── tests/                         FrezaEgm_regression, FrezaEgm_cam_cli, protected.cmake
├── out/deps/                      собранные OpenCAMLib (ocl-Release) и заголовки Boost 1.87
└── tools/                         generate_proto.ps1, build_opencamlib.ps1
```

## 4. Если понадобится `.rspag`

Это обычный zip. Скопируйте, переименуйте в `.zip` и распакуйте (`Expand-Archive`).
Внутри интересны:

```
Systems/<Имя>/SYSPAR/SIO.cfg          ← EGM-устройства (имя, адрес, порт)
Systems/<Имя>/system.xml              ← версия RobotWare и список опций
Backups/<Имя>/RAPID/TASK1/PROGMOD/ModuleEGM.mod   ← RAPID-программа
Stations/*.rsstn                      ← сама станция
```

## 5. Что можно почистить (по желанию)

| Объект | Почему |
|---|---|
| `VisualStudio/FrezaEgm/build-cam`, `build-vs`, `build-no-cam`, `out/build`, `.vs` | артефакты сборки, пересоздаются за минуты |
| `VisualStudio/FrezaEgm/out/deps/boost_1_87_0` | ~1.2 ГБ распакованных заголовков; можно удалить после сборки зависимости (но при пересборке OpenCAMLib понадобится снова) |
| `CAD/*.bak`, `CAD/krug.stl` (0 байт) | резервные копии KOMPAS и пустой экспорт |
| `.tmp/` | временные файлы |
