# 06. Настройка RobotStudio 6.08.01

> Пошаговый сценарий «открыть станцию → создать систему → запустить симуляцию →
> управлять из приложения» — в [12-runbook.md](12-runbook.md). Этот документ —
> технические детали и обоснования.
>
> **Актуальный RAPID-модуль:** `RobotStudio/ModuleEGM.mod` — одна EGM-сессия на
> проход, управление только положением (X/Y/Z без поворотов), инструмент
> `toolPencil` (TCP = кончик, 280 мм), `\CondTime:=1`.

## 1. Что установлено (проверено)

| Компонент | Версия | Где |
|---|---|---|
| ABB RobotStudio | **6.08.01** (6.08.8307.1040) | `C:\Program Files (x86)\ABB Industrial IT\Robotics IT\RobotStudio 6.08` |
| RobotWare | **6.08.1040** (только эта) | `%LOCALAPPDATA%\ABB Industrial IT\Robotics IT\RobotWare\RobotWare_6.08.1040` |
| Windows | 11, x64 | — |

## 2. Проблема исходного пакета и что сделано

`Egm/SolutionEGM.rspag` (и его копия в корне) содержит **систему RobotWare 6.13.0164**.
RobotWare 6.13 не установлен, и RobotStudio 6.08 в принципе не работает с системами 6.13
(поддерживается 6.08 и старше). Поэтому пакет не открывался, а RobotStudio падал
(в журнале — необработанное исключение .NET).

При этом **сама станция** в пакете сохранена RobotStudio 6.08 (`SaveVersion="6.8.8307.1040"`),
то есть графика совместима. Несовместима только вложенная система.

Сделано:

* `SolutionEGM_6.08.rspag` (корень) — тот же пакет, но **без системы 6.13**: станция,
  превью, readme. Открывается в 6.08.01.
* `RobotStudio/ModuleEGM.mod` — RAPID-модуль под 6.08 (устройство `UCdevice`).
* В `RobotStudio/FrezaEGM` лежит ваша станция `FrezaEGM.rsstn` (робот IRB1600-10/1.45
  уже установлен, контроллера нет) — это самая чистая база, см. §4.

> Ограничение прав: корневой `SolutionEGM.rspag` я перезаписать не смог (Windows не даёт
> менять этот файл из песочницы). Замените его вручную: удалите `SolutionEGM.rspag` и
> переименуйте `SolutionEGM_6.08.rspag` → `SolutionEGM.rspag`.

## 3. EGM в RobotWare 6.08: что важно знать

Проверено по файлам установленного RobotWare:

| Факт | Где смотреть |
|---|---|
| Опция EGM есть в медиа 6.08 | `RobotWare_6.08.1040\options\egm\` |
| Опция — это только конфигурация (никаких отдельных бинарников) | `options\egm\install.cmd`: `config -filename … rlegm.cfg / MOC_EGM.cfg / SIO_EGM.cfg` |
| Устройство EGM по умолчанию | `SIO_EGM.cfg`: **`UCdevice`**, `UDPUC`, `127.0.0.1`, порт **6510** |
| `EGMRunPose`, `EGMActPose`, `EGMSetupUC` есть в 6.08 | `options\egm\rlegm.cfg` |
| «`default`» в `EGMSetupUC` — это запись `EXT_MOTION_DATA` | `options\egm\MOC_EGM.cfg` |

Отсюда: **порт 6510 в приложении совпадает** с настройкой по умолчанию, а имя
EGM-устройства для RAPID — `UCdevice` (а не `EGMSensor`, как было в станции 6.13).

## 4. Как запустить: два варианта

### Вариант A (рекомендуется) — ваша станция `RobotStudio\FrezaEGM`

Станция уже содержит робота IRB1600-10/1.45 и не содержит контроллера, поэтому
никаких вопросов про «перемещённую систему» не будет.

1. Открыть `RobotStudio\FrezaEGM\FrezaEGM.rssln` (или `Stations\FrezaEGM.rsstn`).
2. Создать виртуального контроллера:
   `Контроллер → Виртуальный контроллер → Создать систему` (в 6.08 — кнопка
   «Virtual Controller» на вкладке Controller), имя, например `FrezaEGM`.
   В списке опций отметить:
   * **`616-1 PC Interface`**;
   * **`689-1 Externally Guided Motion (EGM)`**.
3. Запустить контроллер, дождаться состояния «Running».
4. `RAPID → Модули → Добавить модуль` → `RobotStudio\ModuleEGM.mod`.
5. `PP на main` → `Пуск` (Play). Робот поедет в `home` = (750, 0, 300).
6. В приложении FrezaEgm нажать «Запустить траекторию» (занимает UDP-порт 6510).

### Вариант B — исправленный пакет `SolutionEGM_6.08.rspag`

1. Открыть пакет в RobotStudio 6.08.01 (File → Open → Pack&Go, либо двойной щелчок).
2. RobotStudio спросит: *«The system … seems to be moved or deleted. Do you wish to
   locate the system directory manually?»* — ответить **No**.
3. Далее — пункты 2–6 варианта A.

Почему спрашивает: в станции сохранена ссылка на систему 6.13 (`SystemPath`,
`SystemIdentity` в `PIM.xml`). Ответ «No» оставляет станцию без контроллера, после чего
создаётся новая система 6.08.

## 5. RAPID-модуль (что внутри)

`RobotStudio/ModuleEGM.mod`:

```rapid
MODULE ModuleEGM
    ! Position-only EGM guidance, tool = pencil tip, one session per program start.
    LOCAL CONST robtarget home := [[750, 0, 600], [0, 1, 0, 0], [0, 0, 0, 0], [9E9, 0, 0, 9E9, 9E9, 9E9]];
    PERS tooldata toolPencil := [TRUE, [[0, 0, 280], [1, 0, 0, 0]], [0.05, [0, 0, 100], [1, 0, 0, 0], 0, 0, 0]];
    LOCAL VAR egmident egm_id;
    LOCAL VAR egm_minmax egm_condition := [-1, 1];
    LOCAL VAR pose corr_frame := [[0, 0, 0], [1, 0, 0, 0]];

    PROC main()
        MoveJ home, v100, fine, toolPencil;
        EGMGetId egm_id;
        EGMSetupUC ROB_1, egm_id, "default", "UCdevice" \Pose;
        EGMActPose egm_id \Tool:=toolPencil \WObj:=wobj0,
                   corr_frame, EGM_FRAME_BASE, corr_frame, EGM_FRAME_BASE
                   \X:=egm_condition \Y:=egm_condition \Z:=egm_condition
                   \MaxSpeedDeviation:=1000.0;
        EGMRunPose egm_id, EGM_STOP_HOLD, \X \Y \Z \CondTime:=1;
        EGMReset egm_id;
    ENDPROC
ENDMODULE
```

* `home` = первая точка `data\trajectory.gcode`; приложение передаёт **абсолютные**
  координаты TCP в базе, поэтому первая цель совпадает с `home`.
* `toolPencil` — `tooldata` с TCP на кончике карандаша (280 мм по Z). Модель
  `toolPencil` в станции — только графика; для расчёта движения нужен именно `tooldata`.
* `\MaxSpeedDeviation:=1000.0` снимает низкий лимит скорости по умолчанию. Без него
  поле «скорость» в приложении не влияет на движение.
* Только `\X \Y \Z`: повороты не управляются. С `\Rx \Ry \Rz` контроллер постоянно
  доворачивает инструмент, система коррекции уезжает и робот уходит с траектории.
* `\CondTime:=1` держит сессию живой, пока идут точки, и завершает её после остановки.
* `"default"` — имя записи `EXT_MOTION_DATA` (`MOC.cfg`), `"UCdevice"` — EGM-устройство
  из `SIO.cfg` (порт 6510).
* Если редактор 6.08 не знает какой-то необязательный параметр — удалите его.

## 6. Порядок запуска (важно)

1. Запустить контроллер в RobotStudio.
2. **Сначала** в приложении «Запустить траекторию» — оно занимает порт 6510 и ждёт.
3. Затем в RAPID `PP на main` → `Пуск`.
4. В приложении состояние станет «отработка траектории», пойдут счётчики пакетов.
5. «Стоп» останавливает поток; RAPID завершает сессию по таймауту и по `ERROR`-ветке
   переходит к следующей итерации.

## 7. Типичные ошибки

| Симптом | Причина / что делать |
|---|---|
| Пакет не открывается, «system created in newer version» | это исходный пакет с RobotWare 6.13 — используйте `SolutionEGM_6.08.rspag` или свою станцию |
| RobotStudio падает при открытии пакета | см. выше: несовместимая система 6.13 в пакете |
| `ERR_UDPUC_COMM` в RAPID | приложение не запущено / порт занят / опция EGM не включена в системе |
| Нет опции `689-1 EGM` в списке при создании системы | сообщите — включим EGM вручную через конфигурацию (опция в 6.08 — это только cfg-файлы) |
| `EGMSetupUC` не компилируется | в системе не включена опция EGM (RAPID-процедуры приходят из `rlegm.cfg`) |
| Робот стоит, пакеты идут | проверьте `MCI` = работает, `egm_condition`, `MaxSpeedDeviation` |
| Робот едет не туда | `corr_frame` должен быть единичным, координаты — абсолютные в базе |
| `50456 Close to singularity` | поза `home` неудачная: поднимите Z или смените ориентацию |

## 8. Если нужна именно станция 6.13

Единственный корректный путь — **другой RobotStudio** (2021+/2024), который умеет
RobotWare 6.13. На 6.08.01 система 6.13 не заработает никак: дело не в файле станции,
а в версии контроллера. Ставить вторую версию RobotStudio — только с вашего согласия.

## 9. Ограничения

* Всё проверялось на виртуальном контроллере; на реальном роботе EGM требует оценки
  рисков и ограничения зоны.
* RobotStudio 6.08 не сертифицирован под Windows 11: возможны сбои UI.
* Создание системы и загрузка RAPID-модуля выполняются в графическом интерфейсе —
  автоматизировать их без запуска RobotStudio нельзя.
