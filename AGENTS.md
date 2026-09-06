# Контекст для ассистента (test_ros2)

## Проект
PET-проект: Java 17 + ROS2 (jros2 1.5.1) + CAN 2.0A. Лидар D500 (LDROBOT STL-19P)
вращается на сервоприводе MKS SERVO42D_CAN оси Z (узлы 01, 500 kbit/s, собственный
протокол привода поверх стандартных CAN-кадров; НЕ CANopen/CiA 402); из последовательных
положений собирается облако точек; далее постобработка (мерж/меш/сечения) и передача
инженеру в SolidWorks.

## Сборка и тесты
- `mvn -o -q compile test` (offline; local repo `C:\Users\borodin\.m2`).
- SocketCanBus (Linux): `libcanwrapper-<arch>.so` ВШИТЫ в jar как ресурсы
  (`/native/libcanwrapper-<arch>.so`), источники — `/canwrapper.c` в корне jar.
  Загрузка: 1) штатный поиск JNA (library path/CWD/LD_LIBRARY_PATH — можно положить
  локально собранный `libcanwrapper.so` рядом с jar), 2) встроенный ресурс по
  `os.arch` (x86_64/amd64 → x86-64, aarch64/arm64 → aarch64) — извлечён в tmp,
  загрузка по абсолютному пути. В jar есть ОБА арха (31.08: aarch64 добавлен —
  малина работает без компиляции на месте). Пересборка .so (docker из Windows,
  aarch64 через `--platform linux/arm64`):
  `docker run --rm --platform linux/arm64 --mount type=bind,source=<src\main\resources>,target=/src,ro --mount type=bind,source=<out>,target=/out gcc:latest sh -c "gcc -shared -fPIC -Wall -O2 -o /out/libcanwrapper-aarch64.so /src/canwrapper.c"`.
  На Windows используется PcanBus.
- Консольный мусор `?????` — кодировка PowerShell, файлы UTF-8; не ошибка.
- Тесты: JUnit 5 (surefire): Mks42dControllerTest, ScanMergerTest, SliceTransformTest,
  E57WriterTest, XyzWriterTest, LiDARReaderTest, DxfWriterTest, SingleScanTest
  (standalone-путь лидара; без подключённого устройства — skip через assumption, а не failure),
  AngleBufferTest, EncoderTrackerTest, SweepAccumulatorTest, TopicSweepTest (05.09: sweep-режим).

## Текущее состояние (состояние на 2026-09-05)
- `MainOrchestrator.java`: цикл `turnToAbsoluteAngle` (0xF5 + waitIdle по 0xF1) →
  `collectScans(10)` (wait/notify под scanLock, таймаут 30 c) →
  `ScanMerger.mergeBucket` (бакеты по углу от 0° лидара, фильтр выбросов по медиане,
  10% порог; в результате angleMin=0, angleMax=(N-1)*step — метаданные описывают
  реальный массив) →
  `publishAndProcess` (публикация /processedScan + синхронный срез в облако + /pointCloud,
   self-echo latch удалён — была гонка между углами); после цикла — экспорт облака в XYZ.
    24.08: редукция мотор:рама = 2:1; **26.08: перебрано на 4:1** (лёгкая рама,
     автоколебаний нет) — `GEAR_RATIO = 4.0`: мотору задаётся `baseAngle * 4`
     (90° рамы = 1 оборот мотора), облако поворачивается на угол РАМЫ; после
      прогона — возврат в нуль. Параметры прогона: `--sweep <° рамы>`
     (деф. 90), `--step <°>` (деф. 10, может быть 0.1), `--scans <n>` (деф. 10),
     `--current <mA>` (рабочий ток 0x83), `--hold <mA>` (удерживающий 0x9B),
      `--tilt <°>` (механический наклон луча 0° лидара от горизонтали, + = луч 0°
      смотрит вверх; деф. 0 — калибровка: подгонять так, чтобы горизонтальные
      линии в облаке были горизонтальны), первый позиционный аргумент —
      CAN-интерфейс (деф. `can0`).
     04.09: фаза 1 качества (против радиальной "ребристости" от откатов привода
     и эластичного ремня):
      - ход теперь "ход + стабилизация": `waitSettled` — 6 опросов "остановлен"
      подряд (~100 мс непрерывного стопа) до сканирования;
      - профиль по дистанции понижен: ≤1° — 1/5, ≤10° — 3/8, иначе 5/25
      (было 1/5, 10/20, 20/100) — против "перебега" и откатов ремня;
      - флаги `--rpm N` / `--acc N` (0 = профиль) — ограничители скорости/ускорения
      (`tuneMotion`), тюнинг привода без пересборки;
      - Ctrl-C / завершение JVM: shutdown hook → `can.close()` (E-stop 0xF7 +
      закрытие шины);
      - мотор не остановился/не стабилизировался → прерывание прогона с экспортом
      частичного облака (было: предупреждение + continue);
      - валидация параметров с fail-fast и `--help` (неизвестный флаг больше
      не трактуется как CAN-интерфейс).
    25.08: настройка привода — на стороне привода (стабильность — задача FOC-контура
    серво, а не оркестратора): ход по умолчанию — профиль по дистанции от текущей
    позиции в градусах МОТОРА (≤1° — 1 rpm/acc 5, ≤10° — 10/20, иначе 20/100 —
    консервативно против торможения «в упор», с 4:1 шаг рамы маппится в 4× больше
    градуса мотора, так что ступени сами масштабируются); оркестратор — просто
    ход + waitIdle. Флаги `--current`/`--hold` — «сила» контура (у 42D нет
    внешних Kp/Ki, FOC-контур внутренний).
   05.09: фаза 2 (sweep-режим) + фаза 3 (min-confidence, calibrate):
   - `--mode sweep` (деф. `step`): рама идёт НЕПРЕРЫВНО с `--rpm` (speedMove, без
     стопов/удержаний), сканы читаются с `/scan` на частоте сенсора; точки
     накапливаются в cloud по угловым бинам через `SweepAccumulator` (скользящее
     окно `--window` (деф. 0.2°)). Убирает "ребристость" от step-and-hold.
   - `--tension <°>` (деф. 2.0) — зона натяжения ремня у старта: точки с
     progress < tension отбрасываются.
   - `--cw`/`--ccw` — направление (деф. CW); progress всегда положительный:
     `dirSign = cw ? -1 : +1`, `progress = dirSign * (angle - start)`.
   - `--merge N` (1..32) — сканов /scan на срез; `--max-range M` (деф. 3.0) —
     ограничение дальности сенсора.
   - `--min-confidence N` (0..255, деф. 0 = выкл) — точки с интенсивностью ниже
     порога не попадают в облако (и step-, и sweep-путь).
   - `--calibrate` (только sweep) — диагностика движения без облака/XYZ
     (`runCalibrate`): число сэмплов, медианный/максимальный период, зависания
     (>150 мс), измеренная скорость vs заданная, разгон (до 90% скорости),
     рекомендуемый `--tension`, суммарный progress.
   - XYZ-экспорт стримингом (`XyzWriter.Stream`) + shutdown hook (`xyzOut`) —
     на Ctrl-C частичное облако пишется в `sweep_*.xyz`.
   - новые классы (05.09): `AngleBuffer` (ring-буфер (ts ns, угол °), интерполяция
     binary search, edge-клемпинг и гэпы ≤ maxGap), `EncoderTracker` (daemon-поток
     опроса угла, буфер 8192), `SweepPoint` (record), `FrameAngleSampler`
     (функциональный `angleAt(ns)`), `SweepAccumulator` (скользящее окно + merge по
     бинам через `ScanMerger.filterAverage`).
   - `TopicInterface`: `sweepSlicePoints(scan, angleAt, dirSign, tensionDeg,
     maxRangeM, minIntensity)` (static, юнит-тестируемый), `publishSweepSlice`,
     конструктор `TopicInterface(ns, tiltDeg, minIntensity)`.
   - `Mks42dController.transact/readSystemParameter` — `synchronized` (CAN с poll-
     потока и main-потока). `ScanMerger.filterAverage(points, tol)` — вынесен из
     `mergeBucket`.
   - тесты: AngleBufferTest (5), EncoderTrackerTest (2), SweepAccumulatorTest (3),
     TopicSweepTest (5).
    - JDK 17 грабли: `AtomicDouble` отсутствует (брать `AtomicReference<Double>`),
      JUnit 5.9.3 без `assertDoubleNaN` (брать `assertTrue(Double.isNaN(x))`),
      fastddsjava `IDLFloatSequence` без `set(int,float)` (только add/get/size).
    - **Особенность MKS 42D (проверено на железе):** `0x92` (setZero) привод
      подтверждает ТОЛЬКО ОДИН РАЗ за enable-цикл (после `0xF8 enable=true`);
      повторный `0x92` без нового enable ответа не шлёт → `writeOk` падал
      «нет ответа на команду 0x92» (step работал — у него нет второго setZero).
      Поэтому `init()` зануляет один раз, `runSweep`/`runCalibrate` setZero НЕ
       зовут — стартуют от текущего (нулевого) угла, логика относительная. Для
       повторного нуля (перестановка рамы) — сначала disable (`0xF8 =0`) + enable.
    - **Направление свипа (баг, проверено на железе):** раньше `dirSign` угадывался
      из флага как `cw ? -1 : +1` (предполагалось: при CW угол энкодера уменьшается).
      На этом стенде при `speedMove(CW)` (0x80) угол РАСТЁТ → прогресс уходил в минус,
      цикл `while (progress < sweepDeg)` никогда не заканчивался (мотор крутил до
      Ctrl-C, xyz 0 pts). Фикс: `dirSign` НЕ угадывается — `waitForFrameDirection()`
      читает фактический знак движения с энкодера после старта (+1 угол растёт / -1
      падает) и подбирает его так, чтобы прогресс был положительным. Сырой угол
      энкодера (со своим знаком) уходит в облако (`toWorldPoint`), поэтому геометрия
      не зависит от dirSign; флаг `--direction` теперь только задаёт, куда крутить
      мотор (0x80/0x00). Работает на любой сборке.
    - **init-ретраи:** стартовые команды init (e-stop/mode/enable/zero) повторяются
      до 3 раз с паузой 500 мс при «нет ответа» (привод после включения/аварии может
      пару сотен мс молчать; раньше первый запуск умирал на 0xF7).
- `TopicInterface.java`: ROS2-нод; аккумулятор `cloudPoints` (List<float[]>),
  снапшот `getCloudPoints()`; PointCloud2 с 3×FLOAT32, point_step=12. frame_id облака
  копируется из /scan (21.08; раньше хардкод "world" — PointCloud2 в RViz было не видно
  из-за отсутствующего TF). Прикол: в s-generated классах jros2 getFrameId() возвращает
  StringBuilder — читать через getFrameIdAsString(). 25.08: точка облака —
  {x, y, z, intensity}, PointCloud2 4×FLOAT32, point_step=16; intensity берётся
  из /scan (драйвер ldlidar_stl_ros2 заполняет intensities — видна гамма отражения
  в RViz2, Color → "Intensity"). 25.08: фильтр `MIN_CLOUD_RANGE_M = 0.15` — точки
  ближе 150 мм в облако не попадают (рама/основание лидара видна была на скане). 25.08: геометрия среза — круг
  луча лидара ВЕРТИКАЛЕН (⊥ основанию), луч 0° горизонтален вдоль +X при нуле
  основания; `toWorldPoint` маппит срез в вертикальную плоскость, содержащую ось Z
  (x = r·cosθ·cosA, y = r·cosθ·sinA, z = UP_SIGN·r·sinθ) — вращение вокруг Z даёт
   объём, а не плоскость XY (раньше z=0 — все срезы складывались в одну плоскость).
   Тест: `SliceTransformTest`. Если облако перевернуто по вертикали — `UP_SIGN = -1`.
   31.08: калибровка `--tilt <°>` (наклон луча 0° от горизонтали, + = вверх) —
   `toWorldPoint(r, θ, A, tiltRad)` считает высоту луча как θ + tilt; лидар установлен
   с механическим наклоном <5°, подгоняется экспериментально по горизонтальным линиям
   в облаке.
- `org.example.export.XyzWriter`: экспорт `scan_export/scan_<yyyyMMdd_HHmmss>.xyz`
  (строка "x y z" на точку). `scan_export/` в `.gitignore`. 05.09: `XyzWriter.Stream`
  (open → writePoint → close, `count()`) — стриминговый экспорт для sweep (частичное
  облако по Ctrl-C через shutdown hook).
- `org.example.export.E57Writer`: проверенный byte-level тестами E57 v1.0 writer,
  в пайплайне пока НЕ используется.
- `org.example.lidar` удалён 20.08 (TofbfParser/SpeedGovernor/PwmSink — не использовался
  пайплайном; при необходимости ToF-парсинг: кадр 47 байта 0x54 0x2C/0x9C, CRC8 по 46 байтам).
- `aold/PubSubDemo` удалён 20.08 (мёртвый код); jSerialComm вынесен из pom, 20.08 возвращён
  в test scope (standalone-тест лидара, не используется main-кодом).
- `org.example.standalone` (test scope, 20.08): отдельно стоящий путь лидара БЕЗ ROS2.
  `LiDARReader` — прямое чтение потока STL-19P через jSerialComm (кадр 47B, CRC8 0x4D;
  `readRotation(timeout)` складывает кадры в один оборот 360°, ~500 точек);
  `DxfWriter` — DXF R2000 LWPOLYLINE, мм ($INSUNITS=4), Y зеркален под CW-вращение,
  кольцо >=340° — замкнуто, разрыв >20° — новый полилайн; `SingleScanTest` — один оборот →
  `scan_export/scan_single_<yyyyMMdd_HHmmss>.dxf`. Запуск:
  `mvn -o -q test -Dtest=SingleScanTest` (порт COM8 по умолчанию, `-Dlidar.port=COMx`
  или `LIDAR_PORT`; без железа — skip).
- `org.example.MksDriveBench`: железный бенч привода (без ROS): запуск (F7→82 05→F3→92),
   опрос (F1/0x31/0x32/параметры 0x82/83/84/8A/8B), туда-обратно N циклов, Ctrl-C = аварийный
   стоп. Запуск: `java -cp target/ros2-demo-1.0-SNAPSHOT.jar org.example.MksDriveBench --help`.
   Проверен в docker (temurin 17, x86-64): встроенный .so извлекается и грузится.
   25.08: флаги `--params` (дамп всех параметров 0x00..0xBF, без движения, ~10 с) и
   `--hold MA` (удерживающий ток 0x9B, setHoldingCurrent) — диагностика автоколебаний
   тяжёлой рамы. ВАЖНО: в наборе параметров 42D НЕТ классических PID-коэффициентов —
   FOC-контуры (ток/позиция) внутренние; настройка из программы: 0x83 рабочий ток,
   0x9B удерживающий, 0x84 микрошаг, 0x82 режим (3/4/5), acc/скорость на ход, 0x80 калибровка.
- `org.example.ScanMerger`: мерж сырых LaserScan — `mergeBucket(scans, factor, tol)`
  (бакет = шаг луча/factor, опора фильтра на медиану, +1e-2 eps против float-дрейфа
  углов) и `mergeRaw(scans)` (без усреднения, сортировка по углу). 25.08: оба
  несут интенсивность — бакет усредняет (r, intensity) парой в том же фильтре
  выбросов, raw проносит за точкой; `pointIntensity` — fallback 0, если в скане
  поле не заполнено.
- `org.example.DebugScanMain`: отладка «лидар без мотора» для RViz2 — окна из N сканов
  /scan → /processedScan + накопленное /pointCloud, экспорт по Ctrl-C.
  Запуск: `mvn -o -q package`, затем
  `java -cp target/ros2-demo-1.0-SNAPSHOT.jar org.example.DebugScanMain --scans 10 --mode bucket6|bucket2|none`
  (параметры: --scans, --mode bucketN|none, --windows 0=∞, --hold, --tol,
  --export xyz,e57|none, --export-dir; --help).
- `org.example.can`: `Mks42dController` — протокол MKS SERVO42D_CAN
  (кадр `[op, args..., Check]`, Check = (CanID + сумма) & 0xFF, ось: 0x4000 = 360°;
  команды: F7 стоп, 82 режим (для CAN нужен 3-5, SR_vFOC=5), 84 микрошаг, 83 ток,
  F3 enable, 92 ноль, F5/F4 abs/rel ось, F6 скорость, F1 статус, 31/32 позиция/скорость);
  transports (PcanBus JNA, SocketCanBus AF_CAN, MockCanBus).

## Ключевые факты D500
- LDROBOT STL-19P, 40-pin: Tx / PWM (0–3.6 В) / GND / P5V.
- UART 230400 8N1, только «датчик → хост»; команд в потоке нет.
- Скорость вращения — дефолтная прошивкой; меняется только внешним PWM (не подключали).
- Кадр (little-endian), 47 байт: 0x54, 0x2C, speed u16 (°/с), start_angle u16 (град/100),
  12×{distance u16 (мм), confidence u8}, end_angle u16 (угол последней точки),
  timestamp u16 (мс), CRC8 по первым 46 байтам (полином 0x4D, init 0x00, MSB-first —
  таблица из ldlidar_stl_ros2). Угол точки: angle_i = start + i·(end−start)/11.
  Левая система координат, вращение CW. ~5000 изм/с, ~12 точек/кадр.
- Замерено 20.08 (COM8, 230400 8N1): ~419 кадров/с (~19 КБ/с), CRC 100% валиден,
  скорость ≈3570 °/с (~10 об/с), дальность до ~4.7 м; сектор ~309–335° с редкими/нулями
  точками (вероятно ToF-артефакты, в драйвере для них есть фильтр tofbf.cpp).
  Чекер: `python lidar_check.py --port COM8` (pyserial, автоопределение формата).
- Официальный ROS2-драйвер: ldlidar_stl_ros2 (github.com/ldrobotSensorTeam/ldlidar_stl_ros2,
  STL-серия DTOF: LD06/LD19/STL-27L; только читает поток).

## Незакрытые
- 04.09: радиальная "ребристость" облака (коррекции CAN-контроллера + эластичность
  ремня) — софтверные меры применены (профиль 5/25, settle ~0,1 c, `--rpm`/`--acc`);
  05.09: дополнительно — sweep-режим (непрерывное вращение) как основная мера.
  Проверить реальным прогоном: если ребристость осталась — снижать `--rpm`/`--acc`
  или перейти на `--mode sweep`.
- 05.09: sweep-режим (фаза 2) + `--min-confidence`/`--calibrate` (фаза 3) проверены
  только юнит-тестами — проверить на железе: качество sweep-облака (ребристость),
  отчёт calibrate (скорость, разгон, рекомендуемый tension) и что стриминговый XYZ
  (`sweep_*.xyz`) открывается в CloudCompare.
- XYZ/E57 не открывались в CloudCompare/SolidWorks вживую (только byte-level тесты).
- 25.08: объёмное облако не проверено на железе — сверить вертикаль (UP_SIGN) и
  направление обхода основания с реальным прогоном в RViz/CloudCompare.
- 26.08: автоколебания/сдвиг тяжёлой рамы — РЕШЕНО аппаратно: редукция 4:1
  (`GEAR_RATIO = 4.0`) + облегчённая рама (аккумулятор под штатив, малина на
  пауэрбанке — без силовых проводов в раме), автоколебаний больше нет. Осталось
  проверить на реальном прогоне: объёмное облако (вертикаль UP_SIGN, направление
  обхода) и что штатив/рама не видны (фильтр <150 мм в TopicInterface).
- Параллельная задача по лидару формулируется в новой сессии (НЕ тот код, что был в `org.example.lidar` — он удалён как не нужный).
- 20.08: USB-адаптер лидара не перечисливается системой (COM8 пропал, в системе только BT-COM) —
  SingleScanTest на этом пропущен. После переподключения: `mvn -o -q test -Dtest=SingleScanTest`,
  результат — `scan_export/scan_single_*.dxf`.

## Сторона постобработки (договорённости)
- CloudCompare: мерж облаков камер (Known positions + ICP), меш (Poisson/Ball Pivoting),
  сечения (plane → Cross section → DXF).
- FreeCAD: контроль/альтернатива сечений + DXF.
- Инженер SolidWorks: получает STL (меш) + DXF (замокнутые контуры сечений);
  PLY/PCD наружу не отдавать (SW не импортирует), сырое облако — ASCII XYZ/E57.
- Для резки — только замкнутые сглаженные контуры, не сырые точки.

## Стиль
- RU; комментарии в проекте на RU; минимальные diff'ы; коммиты только по явной просьбе.
- D500 скорость не управляем, свой драйвер не пишем.
