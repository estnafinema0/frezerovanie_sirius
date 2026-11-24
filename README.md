Ручная последовательность CAM: [дерево операций, выбор геометрии и запуск](docs/18-operation-tree.md).

Расчёт программы из STEP: [черновая обработка → чистовая, заготовка и остаток](docs/17-roughing-finishing-program.md).

# FrezaEgm — управление фрезеровкой через ABB EGM

Добавлен отдельный **STL → OpenCAMLib (C++) → абсолютные точки TCP** этап.
Первая стратегия: замкнутый верхний контур прямоугольного блока, реальный
`CylCutter`/`PathDropCutter`, ASCII/binary STL, явные единицы и размещение,
Windows-диалог/drag-and-drop и прежний экспорт CSV. CAM включается опционально.
[Сборка, контрольные файлы, ограничения и результаты](docs/13-stl-cam.md).
[Пошаговая инструкция оператору Windows / ABB](docs/14-operator-windows.md).

Проверено офлайн на macOS: CAM ON 66/66, OFF 46/46. Windows CI (MSVC 2022 x64)
прошёл в обоих режимах: приложение, исходный selftest/UDP-loopback, регрессия и
неизменность защищённых файлов. [Проверенный запуск](https://github.com/estnafinema0/frezerovanie_sirius/actions/runs/44800353312).
RobotStudio и физический ABB не перепроверены; сведения о работе станции ниже
относятся к исходному состоянию проекта.


Приложение на C++ (Win32 + MSVC 2022), которое берёт файл траектории, превращает его
в массив точек, сохраняет точки в CSV, а затем передаёт их на манипулятор ABB IRB 1600
в виде EGM-коррекций, сериализованных в protobuf 3 (nanopb) и отправленных по UDP в
RobotStudio.

```
STL (меш) ──[CAM]──► trajectory.gcode ──► точки (мм) ──► points.csv
                                              │
                                              ▼
                              EgmSensor (protobuf 3, nanopb)
                                              │ UDP
                                              ▼
                                виртуальный контроллер RobotStudio
```

## Состояние на текущий момент

| Компонент | Где | Состояние |
|---|---|---|
| Схема EGM в синтаксисе proto3 | `Egm/egm3.proto` + `Egm/egm3.options` | готово |
| Сгенерированный C-код | `VisualStudio/FrezaEgm/generated/egm3.pb.c/.h` | готово (nanopb 1.0.0-dev) |
| Приложение (GUI + EGM-клиент) | `VisualStudio/FrezaEgm/src`, `include` | готово, собирается MSVC |
| Самотест без робота | `FrezaEgm_selftest`, `ctest` | 35 проверок, все проходят |
| Симуляция в RobotStudio 6.08.01 | `RobotStudio/` | **работает**: робот проходит замкнутый квадрат 100×100 мм, скорость регулируется |
| Документация и контекст задачи | `docs/`, `PROJECT_STATE.md` | готово |

## Быстрый старт

```powershell
# 1. Сборка (MSVC, из каталога проекта)
cd VisualStudio\FrezaEgm
cmake -S . -B build-vs -G "Visual Studio 17 2022" -A x64
cmake --build build-vs --config Release

# 2. Проверка без робота
.\build-vs\Release\FrezaEgm_selftest.exe

# 3. Приложение
.\build-vs\Release\FrezaEgm.exe
```

Либо открыть папку `VisualStudio\FrezaEgm` в Visual Studio 2022 (см. `docs/07-build-vs2022.md`)
и нажать `Ctrl+Shift+B`.

Порядок запуска с роботом: `docs/12-runbook.md`.

## Как это работает (коротко)

1. **Траектория.** `data/trajectory.gcode` — замкнутый квадрат 100×100 мм на Z = 600 мм,
   5 точек (пятая совпадает с первой). Форматы: G-code (`G0/G1`, `G90/G91`, `G20/G21`)
   и простой `x y z` построчно.
2. **Интерполяция.** `InterpolatePath` режет отрезки на подточки с заданным шагом.
   Время на подточку = `шаг / скорость`; вершины исходной траектории помечаются, чтобы
   контроллер не срезал углы.
3. **EGM.** Приложение слушает UDP 6510, отвечает `EgmSensor` с **абсолютной позой TCP**
   в базовой системе (система коррекции = база, `corr_frame` единичный).
4. **Робот.** RAPID-модуль `ModuleEGM` ставит робота в `home` (750, 0, 600) инструментом
   `toolPencil` и запускает `EGMRunPose` (только X/Y/Z, `\MaxSpeedDeviation:=1000`,
   `\CondTime:=1`). Сессия одна на запуск программы.

## Структура рабочей области

```
freza_sirius\
├── README.md                     ← этот файл
├── PROJECT_STATE.md              ← снимок состояния: что сделано, что дальше
├── docs\                         ← вся документация и контекст задачи
│   ├── 01-requirements.md        ТЗ и критерии приёмки
│   ├── 02-architecture.md        архитектура, модули, состояния
│   ├── 03-trajectory-format.md   откуда берётся файл траектории и как разбирается
│   ├── 04-csv-format.md          формат CSV
│   ├── 05-egm-protocol.md        EGM, protobuf 3, nanopb, байты на проводе
│   ├── 06-robotstudio-setup.md   настройка RobotStudio 6.08.01
│   ├── 07-build-vs2022.md        сборка MSVC/CMake и запуск
│   ├── 08-testing.md             самотест, ctest, чек-лист симуляции
│   ├── 09-decisions.md           журнал технических решений (ADR)
│   ├── 10-glossary.md            глоссарий
│   ├── 11-egm-folder-inventory.md что за файлы лежат в Egm/ и зачем
│   └── 12-runbook.md             ПОЛНАЯ инструкция: RobotStudio → симуляция → управление
├── VisualStudio\FrezaEgm\        ← приложение
│   ├── CMakeLists.txt
│   ├── CMakePresets.json
│   ├── src\                      main, app_window, trajectory, csv_io, egm_client, selftest, text_io
│   ├── include\                  заголовки модулей
│   ├── generated\                egm3.pb.c/.h (генерируется)
│   ├── third_party\nanopb\       pb.h, pb_encode.c, pb_decode.c, pb_common.c
│   ├── data\                     trajectory.gcode, trajectory.xyz, points.csv
│   └── tools\generate_proto.ps1  перегенерация кода из .proto
├── RobotStudio\                  ← всё для симуляции
│   ├── ModuleEGM.mod             RAPID-модуль: toolPencil, только X/Y/Z, одна сессия
│   ├── README.md                 что нажать после открытия станции
│   └── FrezaEGM\                 станция: IRB1600-10/1.45, система с опцией 689-1 EGM
├── SolutionEGM.rspag             исходный пакет (система RobotWare 6.13) — справка
├── SolutionEGM_6.08.rspag        исправленный пакет под RobotStudio 6.08.01
└── Egm\                          ← исходный прототип (MinGW), оставлен как справочник
    ├── egm.proto                 оригинальная схема proto2 (не изменялась)
    ├── egm3.proto / .options     proto3-версия схемы
    ├── src\, include\, CMakeLists.txt, Readme.txt
    ├── lib\protobuf_installed\   protobuf 3.7.1 (MinGW)
    ├── mingw.7z, cmake-*.msi     инструменты прошлого разработчика
    └── *.rspag                   Pack&Go станции RobotStudio
```

## Ключевые решения (подробно — в `docs/09-decisions.md`)

* **nanopb вместо protobuf C++**: MinGW-библиотеки `libprotobuf.a` из `Egm/lib` нельзя
  линковать в MSVC, а nanopb — это C-исходники, которые компилируются тем же MSVC.
  Формат на проводе идентичен protobuf 3.
* **proto3 + `optional`**: у ABB в proto2 поля `EgmCartesian.x/y/z` объявлены `required`,
  а proto3 по умолчанию не передаёт нули. `optional` включает explicit presence и нули
  уходят явно (проверено самотестом).
* **Абсолютные координаты TCP, а не смещения**: `corr_frame` единичный и задан
  `EGM_FRAME_BASE`, поэтому система коррекции совпадает с базой. Со смещениями робот
  уезжал «к себе и вниз» (цель `0,0,0` = начало базы).
* **Только X/Y/Z без поворотов**: с `\Rx \Ry \Rz` контроллер постоянно доворачивал
  инструмент, система коррекции уезжала, робот уходил с траектории.
* **Инструмент `toolPencil`**: объявлен `tooldata` с TCP на кончике карандаша (280 мм),
  потому что модель в станции — только графика.
* **`\MaxSpeedDeviation:=1000.0`**: без него действует низкий лимит скорости
  контроллера, и поле «скорость» в приложении ни на что не влияет.
* **GUI на Win32 API**: без сторонних библиотек, только то, что есть в VS2022.

## Требования

| Что | Версия / примечание |
|---|---|
| ОС | Windows 11 (проверено), x64 |
| Visual Studio 2022 | 17.14, MSVC 14.44, Windows SDK 10.0.26100 |
| CMake | 3.31 (входит в VS) |
| protoc | 33.6 — **только** для перегенерации `egm3.pb.*` |
| nanopb | 0.4.9+/1.0 — **только** для перегенерации |
| RobotStudio | 6.08.01, RobotWare 6.08.1040 + опция `689-1 EGM` |

## Известные ограничения

* Управляется только положение (X/Y/Z); ориентация инструмента не меняется.
* Дуги `G2/G3` не поддерживаются — при разборе огрубляются до отрезков.
* Сессия EGM одна на запуск RAPID-программы: после завершения прохода нужно
  снова нажать `Start` в RobotStudio и «Запустить траекторию» в приложении.
* Скорость и шаг читаются при нажатии «Запустить траекторию» — на ходу не меняются.
* Станция `SolutionEGM.rspag` собрана под RobotWare **6.13.0164** и в RobotStudio
  **6.08.01** не работает — используйте `SolutionEGM_6.08.rspag` или станцию
  `RobotStudio/FrezaEGM`.

### Экспериментальный аналитический CAM

Отдельная ветка `feature/bezier-offline`: [расчёт огибающей плоской фрезы на поверхности Безье](docs/15-bezier-method.md), офлайн CLI и непрерывная проверка отрезков. Не требует OpenCAMLib, не заменяет STL-импорт и не включает ориентацию ABB.

Новый экспериментальный [Milling Geometry Analyzer](docs/16-milling-analyzer.md)
показывает STEP/Bezier в 3D, растровые проходы и XYZ + quaternion инструмента.
Это отдельная офлайн-программа; рабочая передача FrezaEgm/ABB не изменяется.
