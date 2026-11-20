# 02. Архитектура

## 1. Общая схема

```
   STL (меш)                 внешний CAM (FreeCAD/Fusion/MeshCAM)
       │                                  │
       └──────────► trajectory.gcode ◄─────┘
                          │
                          │  LoadTrajectory()          trajectory.cpp
                          ▼
                 std::vector<Point3>  (абсолютные координаты, мм)
                          │
              ┌───────────┴────────────┐
              │                        │
     WritePointsCsv()          InterpolatePath()
        csv_io.cpp              (шаг, мм/с, вершины)
              │                        │
          points.csv                   ▼
              │              std::vector<Point3> + dwell + corners
        ReadPointsCsv()                │
              └────────────────────────┤
                                       ▼
                            EgmClient::Start(settings, points, ...)
                                       │
        ┌──────────────────────────────┴───────────────────────────────┐
        │  рабочий поток EGM (egm_client.cpp)                          │
        │                                                              │
        │  recvfrom(6510) ──► pb_decode(EgmRobot) ──► Snapshot + журнал │
        │        │                                       │             │
        │        │ (адрес источника запоминается)        │             │
        │        ▼                                       │             │
        │  EncodeEgmSensor(абс. поза TCP) ──► sendto(источник)          │
        └────────────────────────────────────────────────┼─────────────┘
                                                         │
                                          GUI, WM_TIMER 100 мс
                                                         ▼
                                       окно: состояние, точка, счётчики, журнал
                                                         │
                                                         ▼
                                    виртуальный контроллер RobotStudio (EGM)
```

## 2. Модули

| Модуль | Файлы | Ответственность |
|---|---|---|
| Текстовый ввод-вывод | `include/text_io.h`, `src/text_io.cpp` | чтение/запись файлов, UTF-8 ↔ UTF-16, снятие BOM |
| Траектория | `include/trajectory.h`, `src/trajectory.cpp` | разбор G-code и XYZ, определение формата, `InterpolatePath` (шаг/скорость/вершины), форматирование точки |
| CSV | `include/csv_io.h`, `src/csv_io.cpp` | запись `index,x,y,z`, устойчивое чтение |
| EGM-клиент | `include/egm_client.h`, `src/egm_client.cpp` | UDP-сокет, поток, состояния, сериализация nanopb, журнал |
| GUI | `include/app_window.h`, `src/app_window.cpp` | окно, элементы управления, файловые диалоги, отрисовка состояния |
| Точка входа | `src/main.cpp` | `wWinMain`, DPI, цикл сообщений |
| Самотест | `src/selftest.cpp` | проверка всей цепочки без робота |

Сгенерированный код: `generated/egm3.pb.c/.h` (из `Egm/egm3.proto`).
Сторонний код: `third_party/nanopb` (pb.h, pb_encode.c, pb_decode.c, pb_common.c).

## 3. Ключевые типы

```cpp
struct Point3 { double x, y, z; };          // мм

struct EgmSettings {
    int port = 6510;      // что слушаем (SIO.cfg: EGMSensor)
    int sendPeriodMs = 4; // период отправки
    int dwellMs = 1000;   // пауза только для одиночной ручной команды
    int cornerSettleMs = 700;  // досылка на вершине траектории
    double reachToleranceMm = 1.5;
    int waitRobotTimeoutMs = 120000;
};

struct EgmSnapshot {      // снимок для UI, копируется под мьютексом
    EgmState state; unsigned seqno; int pointIndex, pointCount;
    Point3 target, feedback; bool hasFeedback;
    int mciState, motorState, rapidState; bool convergence;
    unsigned long long robotPackets, sentPackets;
    std::wstring message;
};
```

## 4. Состояния EGM-клиента

```
        Start()                первый EgmRobot            все точки отданы
 Idle ──────────► WaitingRobot ──────────────► Streaming ──────────────► Finished
  ▲                    │                          │                       │
  │                    │ таймаут 120 с            │ ошибка сокета         │ «Стоп»
  │                    ▼                          ▼                       ▼
  └──────────────── Error ◄───────────────────────┴───────────── Stop() ──► Idle
```

* **WaitingRobot** — порт занят, но контроллер ещё не прислал пакет. Отправлять
  некуда: адрес назначения берётся из входящего пакета.
* **Streaming** — идёт выдача подточек: на прямых участках индекс растёт по времени
  (`шаг / скорость`), на вершинах — после фактического достижения (до `cornerSettleMs`).
* **Finished** — точки закончились, но поток продолжается (удержание последней
  точки), чтобы сессия EGM не оборвалась по таймауту.
* **Error** — текст ошибки остаётся в `Snapshot::message` до следующего запуска.

## 5. Потоки и синхронизация

| Поток | Что делает |
|---|---|
| UI (главный) | сообщения Windows, таймер 100 мс: копирует `Snapshot`, забирает журнал, обновляет контролы |
| Рабочий EGM | `recvfrom` с таймаутом 200 мс, разбор, отправка, продвижение по точкам |

Общие данные: `Impl::snapshot` и `Impl::logs` под `std::mutex`; флаги
`stopRequested`, `running` — `std::atomic<bool>`. Сокет создаётся и биндится в
`Start()` (в UI-потоке), закрывается в `Stop()` после `join()`.

Почему не `PostMessage` с данными: снимок проще и не требует владения памятью;
100 мс достаточно для индикации, а сеть работает на своей частоте.

## 6. Обработка ошибок

| Ситуация | Поведение |
|---|---|
| Файл не найден / битый | `TrajectoryLoadResult::error`, сообщение в журнал и MessageBox |
| Меньше двух точек | отказ загрузки с текстом |
| Порт занят | `Start()` возвращает false, текст с кодом Winsock |
| Нет пакетов от контроллера | сообщение «нет пакетов более 2 с», поток продолжается |
| Ошибка декодирования пакета | строка в журнале, сессия продолжается |
| Ошибка `sendto` | запись в журнал, попытка на следующем такте |
| Закрытие окна | `Stop()` в `WM_DESTROY` — поток корректно завершается |

## 7. Точки расширения

* **Профиль скорости**: разгон/торможение на вершинах вместо фиксированной досылки —
  правки в `InterpolatePath` и `EgmClient::Worker`.
* **Ориентация**: включать `\Rx \Ry \Rz` и заполнять `planned.cartesian.orient`
  опорной ориентацией инструмента.
* **Скоростной режим**: использовать `speedRef` вместо `planned.cartesian`.
* **Дуги**: добавить разбиение G2/G3 на отрезки в `trajectory.cpp`.
* **Лог в файл**: `TakeLog()` уже отдаёт строки — достаточно писать их в файл.
