# 11. Инвентаризация: что за файлы в рабочей области

Здесь описано всё, что лежит в `Egm/` и в корне, — включая наследство прошлого
разработчика. Столбец «судьба» отвечает на вопрос «нужно ли это новому проекту».

## 1. `Egm/` — прототип на MinGW

| Файл / каталог | Что это | Судьба |
|---|---|---|
| `egm.proto` | **Оригинальная схема ABB EGM V1.1, `syntax = "proto2"`**: `EgmHeader`, `EgmCartesian`, `EgmPose`, `EgmJoints`, `EgmPlanned`, `EgmSensor`, `EgmRobot`, `EgmFeedBack`, `EgmMotorState`, `EgmMCIState` и др. В ней `required`/`optional` и `[default = …]`. | **Оставить без изменений** — эталон формата и объяснение, откуда взялись поля |
| `egm3.proto` | **Новая схема в синтаксисе proto3** (наш файл): те же имена, пакет и номера полей; `required` убраны, «обязательные» числа помечены `optional`. | **Используется** для генерации кода приложения |
| `egm3.options` | Опции nanopb: `M_STRIP_PACKAGE` (имена без `abb_egm_`), `max_count:6` для `repeated`-полей. | **Используется** при генерации |
| `CMakeLists.txt` | Сборка старого прототипа **под MinGW**: `find_package(Protobuf)`, `protobuf_generate_cpp`, вывод в `bin/`, линковка `wsock32`/`ws2_32`. | Не используется (наш проект — `VisualStudio/FrezaEgm/CMakeLists.txt`) |
| `Readme.txt` | Инструкция: скачать MinGW с winlibs, добавить в PATH, поставить CMake, собрать `cmake -G "MinGW Makefiles" ..`. | Справочно: объясняет, почему в `lib/` лежат `.a` |
| `src/main_one_joint_move.cpp` | Демо-клиент: занимает **6510**, в цикле «принять → отправить», генерирует коррекцию по шарнирам (0,1° за итерацию до 80°), шлёт `MSGTYPE_CORRECTION`, логирует ответы. Файл-журнал спрашивает через `std::cin`. | **Не собирается** в новом проекте (иначе второй `main`); ценен как пример работы с `EgmSensor` |
| `src/udp_server.cpp`, `include/udp_server.h` | Обёртка над Winsock: `WSAStartup`, `bind`, `recvfrom` в буфер 1400 байт, `sendto` на адрес последнего клиента, строка статуса. | Не используется; наш `egm_client.cpp` делает то же, но с состояниями и потоками |
| `include/logging_data.h` | Шаблон `LoggingData<T>`: поток + очередь + мьютекс; `WriteRobotLogFile` разбирает `EgmRobot` и пишет `seqno`/`tm` в txt; `DisplayRobotMessageInTerminal` печатает в консоль. В конструкторе — блокирующий `getline` имени файла. | Не используется; идея логирования перенесена, блокирующий ввод — убран |
| `lib/protobuf_installed/` | **Google protobuf 3.7.1**: `include/` (заголовки и полные исходники `.cc`), `lib/libprotobuf*.a` (MinGW), `bin/protoc.exe` (не запускается — нужны MinGW-DLL). | Не используется (см. `09-decisions.md`, ADR-001). Может пригодиться, если понадобится полный protobuf под MSVC (тогда `.a` надо пересобрать) |
| `mingw.7z` | Архив MinGW-тулчейна. | Не нужен (MSVC). Можно удалить, чтобы освободить место |
| `cmake-3.23.2-windows-i386.msi` | Установщик CMake, который скачивал прошлый разработчик. | Не нужен: CMake входит в VS2022 |
| `SolutionEGMCourse.rspag` | Станция курса: joint-режим, устройство **`UCdevice` → 127.0.0.1:6511**, `EGMSetupUC … \Joint`, `EGMRunJoint`. | Справочник: пример joint-режима, если pose-режим недоступен |
| `src/`, `include/` | Исходники прототипа (см. выше). | Справочник |

### Как посмотреть внутрь `.rspag`

Это zip. Скопируйте, переименуйте в `.zip` и распакуйте (или `Expand-Archive`).
Внутри интересны:

```
Systems/<Имя>/SYSPAR/SIO.cfg          ← EGM-устройства (имя, адрес, порт)
Systems/<Имя>/system.xml              ← версия RobotWare и список опций
Backups/<Имя>/RAPID/TASK1/PROGMOD/ModuleEGM.mod   ← RAPID-программа
Stations/*.rsstn                      ← сама станция
```

## 2. Корень рабочей области

| Файл | Что это | Судьба |
|---|---|---|
| `SolutionEGM.rspag` | Исходный **Pack&Go станции IRB 1600**: `IRB_1600_10kg_1.45m`, система **RobotWare 6.13.0164**, опции `616-1 PC Interface`, `689-1 EGM`, `UDPUC Driver`; `SIO.cfg` с устройством `EGMSensor` → 127.0.0.1:6510; `ModuleEGM.mod` с `EGMSetupUC … \Pose`. | Только **справка**: с установленным RobotWare 6.08 не открывается (см. `06-robotstudio-setup.md`) |
| `SolutionEGM_6.08.rspag` | Тот же пакет **без системы 6.13**: станция (сохранена RobotStudio 6.08), превью, readme. | **Используется** для варианта B в `06-robotstudio-setup.md` |
| `RobotStudio/` | `ModuleEGM.mod` (RAPID под 6.08, `UCdevice`), `README.md`, `FrezaEGM/` — ваша станция `IRB1600-10/1.45` без контроллера. | **Используется** для варианта A (рекомендуемого) |
| `README.md` | Точка входа в проект: обзор, быстрый старт, структура. | **Используется** |
| `docs/` | Документация и весь контекст задачи. | **Используется** |
| `VisualStudio/FrezaEgm/` | Новый проект (приложение). | **Основной проект** |

## 3. Что именно использует новый проект

```
VisualStudio/FrezaEgm/
├── CMakeLists.txt                 сборка MSVC (nanopb, freza_core, GUI, selftest)
├── CMakePresets.json              пресеты VS (Ninja, x64-debug/release)
├── src/…                          код приложения
├── include/…                      заголовки
├── generated/egm3.pb.c/.h         ← сгенерировано из Egm/egm3.proto
├── third_party/nanopb/            pb.h + pb_encode.c + pb_decode.c + pb_common.c
├── data/trajectory.gcode          демо-траектория (5 точек, квадрат 100×100)
├── data/trajectory.xyz            то же в формате «x y z»
├── data/points.csv                результат разбора (создан самотестом)
└── tools/generate_proto.ps1       перегенерация кода из .proto
```

## 4. Что изменено в рабочей области

* **Добавлено:** `Egm/egm3.proto`, `Egm/egm3.options`, проект `VisualStudio/FrezaEgm/*`,
  документация `docs/*`, корневой `README.md`, `RobotStudio/` (`ModuleEGM.mod`,
  `FrezaEGM/` — станция под 6.08), `SolutionEGM_6.08.rspag`.
* **Удалено:** шаблонный подпроект `VisualStudio/FrezaEgm/FrezaEgm/`
  (`FrezaEgm.cpp` с `int main()`, `FrezaEgm.h`, его `CMakeLists.txt`) — он не
  использовался и мог дать второй `main` (см. ADR-014).
* **Не изменялось:** `Egm/egm.proto` (оригинал proto2), `Egm/SolutionEGMCourse.rspag`,
  корневой `SolutionEGM.rspag` (исходный пакет с системой 6.13), `lib/`, `mingw.7z`,
  `cmake-*.msi`, исходники прототипа. Изменения прав Windows касались только прав
  доступа к каталогу (подробности — в отчёте `D:\developments\DSH_workspaces\acl-recovery`).

## 5. Что можно почистить (по желанию)

| Объект | Почему |
|---|---|
| `Egm/mingw.7z`, `Egm/cmake-*.msi` | не нужны при сборке MSVC |
| `Egm/lib/protobuf_installed` | ~70 МБ; нужен только если возвращаться к полному protobuf |
| `SolutionEGM.rspag` в корне | исходный пакет с системой 6.13; заменяется на `SolutionEGM_6.08.rspag` |
| `VisualStudio/FrezaEgm/out`, `build-vs`, `.vs` | артефакты сборки, пересоздаются |
