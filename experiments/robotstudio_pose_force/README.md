# RobotStudio: эксперимент XYZ + quaternion и отдельная силовая задача

Это отдельный **виртуальный** эксперимент для IRB1600-10/1.45. Исходные FrezaEgm,
ModuleEGM.mod, станция, tooldata, EGM/protobuf/nanopb не меняются. Новый адаптер
компилирует существующий nanopb и generated-код без изменений. Он принимает только
localhost и использует отдельные порты 6512/6513/6514. Старый UDP-порт не меняется.

## Что реализовано и что проверено

- MillingAnalyzer рассчитывает единую траекторию выбранной грани с подводами,
  отводами и переходами; экспортирует XYZ, quaternion wxyz, нормаль и общее время.
- MillingVirtualEgm — единственный владелец выходных EGM-команд. Интерполирует
  XYZ+SLERP, применяет ограниченное смещение вдоль нормали, кодирует quaternion
  в u0..u3 и сохраняет explicit presence всех нулей XYZ/quaternion. Euler не отправляет.
- VirtualEgm.mod — motion task T_ROB1, включает XYZ/Rx/Ry/Rz в отдельном VC.
- ForceSim.mod — NORMAL non-motion task T_FORCE: синтетический скаляр силы,
  фильтр, PI с anti-windup, предел ±0.5 мм, ограничение изменения 0.01 мм за цикл.
  По умолчанию коррекция выключена. Это эмулятор входа, **не симуляция физики резания
  и не активированный ABB Force Control**.
- Передача коррекции: UDP `sequence,displacement_mm`, ответ `RUN,sequence` или
  `DONE,sequence`. Это отдельный канал, не добавление полей в защищённый EGM protobuf.
- ПК ограничивает скорость применения коррекции 1 мм/с. Данные старше 250 мс,
  повторные sequence, NaN/Inf и выход за пределы не используются. Потеря свежих
  данных завершает адаптер; heartbeat второй задачи останавливает EGM через HOLD.
- После завершения пути ПК удерживает последнюю позу, посылает DONE; силовая task
  устанавливает общий simulationStop, motion task вызывает EGMStop и EGMWaitCond.
  CondTime=300 не даёт EGM закончиться преждевременно, когда робот хорошо следует
  промежуточной цели. NoWaitCond позволяет motion task контролировать heartbeat.

Проверки C++ и UDP-loopback выполнены без ABB. RAPID-файлы сверены с документацией,
но **не скомпилированы виртуальным контроллером и не испытаны в RobotStudio** в этой
среде. Windows CI не является компилятором RAPID. Реальный период 4 мс не подтверждён.

## Документация ABB и ограничения архитектуры

Официальные источники:

- [RAPID Instructions, Functions and Data Types, 3HAC050917, rev F](https://library.e.abb.com/public/b227fcd260204c4dbeb8a58f8002fe64/Rapid_instructions.pdf),
  SocketCreate/Bind/SendTo/ReceiveFrom, страницы 702, 709, 718, 727.
- [EGM RW6, 3HAC073319, rev L](https://library.e.abb.com/public/259850157a114d5fa9144fa0788c7700/3HAC073319%20AM%20Externally%20Guided%20Motion%20RW6-en.pdf),
  EGMActPose, EGMRunPose, EGMStop, EGMWaitCond, страницы 50–55, 68–70, 85, 89.
- [Force Control RW6, 3HAC050377, rev M](https://library.e.abb.com/public/21469a83cf8743bfb2125f83dc397db5/3HAC050377%20AM%20Force%20Control%20RW%206-en.pdf),
  ограничения страницы 13–14, FCGetForce страницы 208–209.

Фактическая сохранённая станция использует RobotWare **6.08.1040**. Последние EGM/FC
руководства относятся к более поздним ревизиям RW6; окончательная проверка инструкций
и лицензий должна выполняться компилятором именно установленного VC.

EGMRunPose выполняется только в motion task. FCAct/FCDeact также ограничены motion
задачей; нельзя просто включить FCAct в фоновой task параллельно EGM. ABB отдельно
запрещает сочетать EGM с FCPress/FCSpdChg. Поэтому в этом эксперименте вторая задача
считает только поправку; команды движения формирует один ПК-адаптер.

Реальный датчик потребует другого источника данных вместо simulatedN:
подтверждённые FC option, FCCalib, payload/гравитационная компенсация, преобразование
wrench и согласованный знак Fn=F·n. FCGetForce может возвращать измерение в системе
Tool/WObj; без калибровки и проверки его нельзя принимать за контактную силу.
Наличие measuredForce в схеме EGM само по себе не подтверждает доступность датчика.
Заменять синтетическое значение на неизвестное измерение молча нельзя.

## Сборка и офлайн-проверка на Windows

PowerShell из корня репозитория, VS2022 C++, CMake, Git, Python 3 **для тестов**:

```powershell
git status --short
git fetch origin
git switch feature/bezier-viewer
git pull --ff-only origin feature/bezier-viewer
cmake -P MillingAnalyzer/cmake/Dependencies.cmake
$deps = Join-Path $pwd 'build-bezier-viewer-deps'
cmake -S MillingAnalyzer -B build-bezier-viewer -G "Visual Studio 17 2022" -A x64 "-DCMAKE_PREFIX_PATH=$deps/occt" "-DEIGEN_INCLUDE_DIR=$deps/eigen"
cmake --build build-bezier-viewer --config Release --parallel 4
ctest --test-dir build-bezier-viewer -C Release --output-on-failure
cmake -S experiments/robotstudio_pose_force -B build-virtual -G "Visual Studio 17 2022" -A x64 "-DEIGEN_INCLUDE_DIR=$deps/eigen"
cmake --build build-virtual --config Release --parallel 4
ctest --test-dir build-virtual -C Release --output-on-failure

# Окно с загрузкой STEP, выбором грани и всей траекторией:
.\build-bezier-viewer\Release\MillingAnalyzer.exe
# Или воспроизводимый набор файлов без браузерного скачивания (новый каталог):
.\build-bezier-viewer\Release\MillingAnalyzer.exe --demo --output demo-connected --open
```

В GUI выберите STEP из любой папки → грань → параметры → Рассчитать → «Вся грань».
CSV сохраняется кнопкой «Позы CSV». В CLI он находится в `demo-connected/face-1.poses.csv`.
Нельзя подавать этот файл в старый FrezaEgm XYZ/CSV-парсер.
В архиве CI сохраните EXE анализатора и все его DLL рядом; виртуальный адаптер
зависит только от стандартного runtime MSVC и системного Winsock.

## Проверка в отдельной станции RobotStudio

1. Создайте **новый виртуальный** IRC5 с IRB1600-10/1.45 и доступным RW6.08.1040.
   Сохранённую рабочую станцию проекта не перезаписывайте. Нужны опции EGM,
   Multitasking и Socket Messaging для non-motion task; проверьте их в System Builder.
2. Добавьте NORMAL non-motion task `T_FORCE`; motion task — `T_ROB1`.
   Загрузите `VirtualEgm.mod` в T_ROB1, `ForceSim.mod` в T_FORCE. У обеих задач main.
   PERS `simulationStop`, `forceHeartbeat`, `virtualConfirmed` намеренно общие:
   одинаковые имя/тип/инициализатор, без LOCAL и TASK. Проверьте совместное значение.
3. В новом VC создайте UdpUc device `VIRTUAL_POSE`: remote address 127.0.0.1,
   remote port 6512. Имя external motion configuration `default` проверьте в VC.
   Эти настройки не переносить в рабочий контроллер. Порты 6513/6514 используются
   только двумя локальными процессами для коррекции/подтверждений.
4. Compile/Check Program обеих задач. Если RW не принимает инструкцию/опцию,
   остановитесь и сохраните точный текст ошибки, версию RW и список опций.
   Автоматически заменять ориентацию или отключать контроль ошибки нельзя.
5. Оставьте `correctionEnabled=FALSE`, `referenceN=simulatedN=10`.
   Установите `virtualConfirmed=TRUE` **только в виртуальном контроллере**.
   Положение simHome и демонстрационный simTool совпадают с прежними числовыми
   значениями, но не являются регистрацией присланного шпинделя.
6. В отдельном PowerShell запустите ПК-адаптер, затем в течение 30 секунд сначала
   T_FORCE, затем T_ROB1 (или обе задачи; motion task ждёт heartbeat):

```powershell
.\build-virtual\Release\MillingVirtualEgm.exe --virtual-only --anchor-first-to-feedback .\demo-connected\face-1.poses.csv
```

7. Ключ `--anchor-first-to-feedback` явно совмещает **первую отведённую позу**
   с первым feedback TCP; поворот детали выбирается так, чтобы совпала и ориентация.
   Это размещение для сухой демонстрации. ПК печатает translation и quaternion.
   Для проверки CAD-коллизий в станции разместите модель тем же преобразованием.
   Для демонстрационного simTool используется явная регистрация вращения:
   `q_base_tcp = q_base_part * q_part_cutter * [0,1,0,0]`.
   Планировщик считает +Z к хвостовику, а у pencil TCP +Z идёт от фланца к торцу;
   поворот Rx(180°) согласует эти оси. Это соглашение виртуального pencil, не
   измеренная регистрация реальной шпиндельной сборки.
   TCP смещение не прибавляется к XYZ повторно. IK/конфигурация, механическая
   ось шпинделя и реальная регистрация остаются отдельными проверками.
8. Убедитесь, что видны подвод, все проходы, отводы/переходы и изменение ориентации.
   Проверьте журнал VC, пределы суставов, конфигурацию, сингулярности и коллизии.
   Ограничения адаптера: ≤50 мм/с, ≤30°/с, путь ≤240 с, один face_id/pass_id=0.
   CSV с флагами пересечения отклоняется до открытия сокетов.
9. Для проверки регулятора установите `correctionEnabled=TRUE`, измените
   `simulatedN` с 10 на 12. Наблюдайте ограниченную поправку вдоль нормали.
   Поворот не корректируется силовым регулятором. Знак для реальной обработки
   из этого опыта не следует. Затем остановите T_FORCE: EGM должен остановиться
   через watchdog/HOLD; проверьте это именно в VC. Перезапускать весь опыт заново.
10. По окончании пути ожидаются DONE, EGMStop, EGMWaitCond, EGMReset.
    При любой ошибке прекратите обе задачи. На физический контроллер этот комплект
    не подключается: адрес интерфейса жёстко ограничен loopback.

Это позволяет проверить генерацию и обмен позами, а не качество фрезерования,
настройку реального датчика, аппаратную совместимость или реальное время отправки.
