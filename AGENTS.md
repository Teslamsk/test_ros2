# Контекст для ассистента (test_ros2)

## Проект
PET-проект: Java 17 + ROS2 (jros2 1.5.1) + CAN 2.0A. Лидар D500 (LDROBOT STL-19P)
вращается на сервоприводе MKS SERVO42D_CAN оси Z (узлы 01, 500 kbit/s, собственный
протокол привода поверх стандартных CAN-кадров; НЕ CANopen/CiA 402); из последовательных
положений собирается облако точек; далее постобработка (мерж/меш/сечения) и передача
инженеру в SolidWorks.

## Сборка и тесты
- `mvn -o -q compile test` (offline; local repo `C:\Users\borodin\.m2`).
- SocketCanBus (Linux): `libcanwrapper.so` (Linux x86-64) ВШИТА в jar как ресурс В КОРНЕ
  (`libcanwrapper.so`); загрузка штатным механизмом JNA — если библиотека не найдена в
  library path, JNA сам извлекает ресурс в tmp. Пересборка .so (docker из Windows):
  `docker run -v <src\main\c>:/src:ro -v <out>:/out gcc:latest gcc -shared -fPIC -Wall -O2 -o /out/libcanwrapper.so /src/canwrapper.c`.
  На Windows используется PcanBus.
- Консольный мусор `?????` — кодировка PowerShell, файлы UTF-8; не ошибка.
- Тесты: JUnit 5 (surefire): Mks42dControllerTest, ScanMergerTest, E57WriterTest, XyzWriterTest,
  LiDARReaderTest, DxfWriterTest, SingleScanTest (standalone-путь лидара; без подключённого
  устройства — skip через assumption, а не failure).

## Текущее состояние (состояние на 2026-08-20)
- `MainOrchestrator.java`: цикл `turnToAbsoluteAngle` (0xF5 + waitIdle по 0xF1) →
  `collectScans(10)` (wait/notify под scanLock, таймаут 30 c) →
  `ScanMerger.mergeBucket` (бакеты по углу от 0° лидара, фильтр выбросов по медиане,
  10% порог; в результате angleMin=0, angleMax=(N-1)*step — метаданные описывают
  реальный массив) →
  `publishAndProcess` (публикация /processedScan + синхронный срез в облако + /pointCloud,
   self-echo latch удалён — была гонка между углами); после цикла — экспорт облака в XYZ.
   24.08: редукция мотор:рама = 2:1 (2 оборота мотора = 1 оборот рамы) —
   `GEAR_RATIO = 2.0`: мотору задаётся `baseAngle * 2`, облако поворачивается на угол
   РАМЫ; после прогона — возврат в нуль. Параметры прогона: `--sweep <° рамы>`
   (деф. 90), `--step <°>` (деф. 10), `--scans <n>` (деф. 10), первый позиционный
   аргумент — CAN-интерфейс (деф. `can0`).
- `TopicInterface.java`: ROS2-нод; аккумулятор `cloudPoints` (List<float[]>),
  снапшот `getCloudPoints()`; PointCloud2 с 3×FLOAT32, point_step=12. frame_id облака
  копируется из /scan (21.08; раньше хардкод "world" — PointCloud2 в RViz было не видно
  из-за отсутствующего TF). Прикол: в s-generated классах jros2 getFrameId() возвращает
  StringBuilder — читать через getFrameIdAsString(). 25.08: геометрия среза — круг
  луча лидара ВЕРТИКАЛЕН (⊥ основанию), луч 0° горизонтален вдоль +X при нуле
  основания; `toWorldPoint` маппит срез в вертикальную плоскость, содержащую ось Z
  (x = r·cosθ·cosA, y = r·cosθ·sinA, z = UP_SIGN·r·sinθ) — вращение вокруг Z даёт
  объём, а не плоскость XY (раньше z=0 — все срезы складывались в одну плоскость).
  Тест: `SliceTransformTest`. Если облако перевернуто по вертикали — `UP_SIGN = -1`.
- `org.example.export.XyzWriter`: экспорт `scan_export/scan_<yyyyMMdd_HHmmss>.xyz`
  (строка "x y z" на точку). `scan_export/` в `.gitignore`.
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
  углов) и `mergeRaw(scans)` (без усреднения, сортировка по углу).
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
- XYZ/E57 не открывались в CloudCompare/SolidWorks вживую (только byte-level тесты).
- 25.08: объёмное облако не проверено на железе — сверить вертикаль (UP_SIGN) и
  направление обхода основания с реальным прогоном в RViz/CloudCompare.
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
