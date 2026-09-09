# Программа испытаний VLP-16 (список строк для терминала)

Машина: Linux + ROS2 (humble/jazzy) + SocketCAN + Java 17. Все команды — из каталога,
где лежит `ros2-demo-1.0-SNAPSHOT.jar` (fat-JAR: `java -jar` = `org.example.vlp16.Vlp16Orchestrator`;
внутри уже jros2 + jna + нативный `libcanwrapper` для amd64/aarch64).
`ROS_DOMAIN_ID` должен совпадать во всех терминалах (деф. 0).

## 0. Тест без рамы (лидар на столе, CAN не нужен)

Всё, кроме движения рамы: драйвер, QoS, наш парсер, монтажка, `/pointCloud`.
Лидар поставить на стол ТАК, как он будет лежать на раме (ось вращения
горизонтальная), запитать. Терминал 1 — драйвер (раздел 3), затем:

```bash
# Терминал 2 — standalone-просмотр нашего нода (без CAN):
java -cp ros2-demo-1.0-SNAPSHOT.jar org.example.vlp16.Vlp16Cloud --window 1000
```

```bash
# Терминал 3 — RViz2 (раздел 5): Fixed frame = base, /pointCloud, Accumulate
rviz2
```

Критерии:
- строки `[VLP16] raw ... cloud ...` с `raw > 0` (raw = 0 — драйвер не отдаёт);
- в RViz видны кольца одного оборота;
- вертикальные поверхности (стены) вертикальны, пол горизонтален — если
  «лежит не так», править `Vlp16Mount` (матрица/знак/высота) ДО сборки рамы.

Дальше — полный испытательный цикл (разделы 1–9) уже с рамой.

## 1. Разовая подготовка машины (пропустить, если ROS2/драйвер/CAN уже стоят)

```bash
java -version                                             # нужна 17
sudo apt update
sudo apt install ros-humble-velodyne                     # для jazzy: ros-jazzy-velodyne
```

CAN-интерфейс (MKS 42D: узел 01, 500 kbit/s; пропустить, если can0 уже поднят):

```bash
sudo ip link add dev can0 type can
sudo ip link set up can0 type can bitrate 500000
ip link show can0
```

## 2. Железо

1. Подключить VLP-16 по Ethernet к той же сети, что и машина; запитать лидар.
2. Подключить USB-CAN → `can0`; запитать драйвер MKS 42D.
3. Убедиться, что машина видит IP лидара. **Наш лидар: 192.168.1.201, порт 2368**
   (стандартный VLP-16-порт пакетов); при необходимости настроить статический IP
   на машине в диапазоне 192.168.1.x:

```bash
ping -c 3 192.168.1.201
```

## 3. Терминал 1 — ROS2 + драйвер velodyne

IP/порт — только для драйвера (он сам ловит UDP с лидара). Java-программа
не знает про IP — читает только ROS2-топик `/velodyne_points`.

```bash
source /opt/ros/humble/setup.bash                        # для jazzy: /opt/ros/jazzy
ros2 launch velodyne velodyne_nodelet.launch.py model:=VLP16 device_ip:=192.168.1.201
```

Порт 2368 — дефолт драйвера для VLP-16, параметром не задаётся; если драйвер
не ловит пакеты — проверить, что UDP 2368 не занят/не заблокирован фаерволом:
`sudo ss -ulnp | grep 2368`.

## 4. Терминал 2 — проверка драйвера

```bash
source /opt/ros/humble/setup.bash
ros2 topic info /velodyne_points --verbose               # имя + QoS (best_effort)
ros2 topic hz /velodyne_points                           # ~10 Гц
ros2 topic echo /velodyne_points --once                  # layout: x/y/z/intensity
```

## 5. Терминал 3 — RViz2

```bash
source /opt/ros/humble/setup.bash
rviz2
```

В RViz (руками): Fixed frame = `base`; Add → By topic → PointCloud2 →
Topic = `/pointCloud`; Display → галка **Accumulate**.

## 6. Терминал 4 — дымовой тест (90°, ~9 шагов)

```bash
java -jar ros2-demo-1.0-SNAPSHOT.jar can0 --sweep 90 --step 10.3 --window 1000
```

Критерии:
- строка `step N/N: frame ... deg, raw ...` появляется каждый шаг;
- в RViz (Accumulate) видно сектор 90° из колец;
- после выхода рама возвращается в нуль; файл `scan_export/vlp16_step_*.xyz` создан.

## 7. Полный прогон (360°, шаг 10.3°, ~212 позиций)

```bash
java -jar ros2-demo-1.0-SNAPSHOT.jar can0 --sweep 360 --step 10.3 --window 1000
```

Дополнительно (по ситуации):
- `--min-intensity 30` — отбросить слабые отражения;
- `--max-range 30` — дальность под объект;
- `--height <м>` — высота центра лидара над основанием (подгонка);
- `--rpm 2 --acc 10` — ограничить скорость/ускорение мотора.

Остановка: Ctrl-C (shutdown hook дописывает и закрывает текущий XYZ — частичный
результат остаётся в `scan_export/`).

## 8. Результаты

```bash
ls -lt scan_export
wc -l scan_export/vlp16_step_*.xyz
head -5 scan_export/vlp16_step_*.xyz
```

Открыть XYZ в CloudCompare (File → Create → ASCII) и проверить:
1. вертикаль — стены/пол перпендикулярны (если «лежит не так» — `Vlp16Mount`:
   матрица/знак/высота);
2. направление обхода и полный круг без пропусков;
3. сам штатив/рама не видны (дальние точки отсечены `--max-range`).

## 9. Сброс

```bash
# Ctrl-C в терминалах 1..4
sudo ip link set down can0
```
