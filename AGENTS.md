# Контекст для ассистента (test_ros2)

## Проект
PET-проект: Java 17 + ROS2 (jros2 1.5.1) + CAN 2.0A. Лидар D500 (LDROBOT STL-19P)
вращается на сервоприводе MKS SERVO42D_CAN оси Z (узлы 01, 500 kbit/s, собственный
протокол привода поверх стандартных CAN-кадров; НЕ CANopen/CiA 402); из последовательных
положений собирается облако точек; далее постобработка (мерж/меш/сечения) и передача
инженеру в SolidWorks.

## Сборка и тесты
- `mvn -o -q compile test` (offline; local repo `C:\Users\borodin\.m2`).
- Консольный мусор `?????` — кодировка PowerShell, файлы UTF-8; не ошибка.
- Тесты: JUnit 5 (surefire): Mks42dControllerTest, E57WriterTest, XyzWriterTest.

## Текущее состояние (состояние на 2026-08-20)
- `MainOrchestrator.java`: цикл `turnToAbsoluteAngle` (0xF5 + waitIdle по 0xF1) →
  `collectScans(10)` → `mergeScans` (бакеты по углу, 10% фильтр выбросов) →
  `publishAndAwaitProcessed`; после цикла — экспорт облака в XYZ.
  ⚠️ Сейчас `scanSteps = 18`, `stepDeg = 180/scanSteps` → проход 180°, НЕ 360°
  (раньше было 36 позиций по 10°). Если нужен 360° — поправить константы.
- `TopicInterface.java`: ROS2-нод; аккумулятор `cloudPoints` (List<float[]>),
  снапшот `getCloudPoints()`; PointCloud2 с 3×FLOAT32, point_step=12.
- `org.example.export.XyzWriter`: экспорт `scan_export/scan_<yyyyMMdd_HHmmss>.xyz`
  (строка "x y z" на точку). `scan_export/` в `.gitignore`.
- `org.example.export.E57Writer`: проверенный byte-level тестами E57 v1.0 writer,
  в пайплайне пока НЕ используется.
- `org.example.lidar` удалён 20.08 (TofbfParser/SpeedGovernor/PwmSink — не использовался
  пайплайном; при необходимости ToF-парсинг: кадр 47 байта 0x54 0x2C/0x9C, CRC8 по 46 байтам).
- `org.example.can`: `Mks42dController` — протокол MKS SERVO42D_CAN
  (кадр `[op, args..., Check]`, Check = (CanID + сумма) & 0xFF, ось: 0x4000 = 360°;
  команды: F7 стоп, 82 режим (для CAN нужен 3-5, SR_vFOC=5), 84 микрошаг, 83 ток,
  F3 enable, 92 ноль, F5/F4 abs/rel ось, F6 скорость, F1 статус, 31/32 позиция/скорость);
  transports (PcanBus JNA, SocketCanBus AF_CAN, MockCanBus).

## Ключевые факты D500
- LDROBOT STL-19P, 40-pin: Tx / PWM (0–3.6 В) / GND / P5V.
- UART 230400 8N1, только «датчик → хост»; команд в потоке нет.
- Скорость вращения — дефолтная прошивкой; меняется только внешним PWM (не подключали).
- Кадр (little-endian): 0x54, 0x2C, speed u16, start_angle u16 (град/100),
  12×{distance u16, confidence u8}, end_angle u16, timestamp u32, CRC8.
  Левая система координат, вращение CW. ~5000 изм/с, ~12 точек/кадр.
- Официальный ROS2-драйвер: Ldrobot-lidar-ros2 (только читает поток).

## Незакрытые
- XYZ/E57 не открывались в CloudCompare/SolidWorks вживую (только byte-level тесты).
- Параллельная задача по лидару формулируется в новой сессии (НЕ тот код, что был в `org.example.lidar` — он удалён как не нужный).

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
