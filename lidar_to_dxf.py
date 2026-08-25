#!/usr/bin/env python3
# Разовый коллектор /scan (малина, ROS2; только rclpy, без pip-пакетов).
# Собирает N секунд сканов, мерджит по углу в бакеты 0.5° (среднее),
# пишет DXF (мм, R2000/LWPOLYLINE — тот же формат, что у Java DxfWriter)
# и XYZ (метры, для CloudCompare).
#
#   source /opt/ros/<humble|jazzy>/setup.bash
#   ros2 topic hz /scan                       # проверка, что есть данные
#   python3 lidar_to_dxf.py --seconds 5 --out scan_single
#   -> scan_single.dxf + scan_single.xyz
import argparse
import math
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

BUCKET_DEG = 0.5
BUCKETS = int(360.0 / BUCKET_DEG)
MAX_GAP_DEG = 20.0
FULL_RING_DEG = 340.0


class Collector(Node):
    def __init__(self, topic, seconds):
        super().__init__("scan_collector")
        self.t_end = time.monotonic() + seconds
        self.cells = [[] for _ in range(BUCKETS)]
        self.msg_count = 0
        self.create_subscription(LaserScan, topic, self.cb, 10)

    def cb(self, msg: LaserScan):
        self.msg_count += 1
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            deg = math.degrees(msg.angle_min + i * msg.angle_increment)
            deg = ((deg % 360.0) + 360.0) % 360.0
            self.cells[int(deg / BUCKET_DEG) % BUCKETS].append(float(r))


def split_rings(points):
    rings, cur = [], []
    prev = None
    for deg, r in points:
        if prev is not None and deg - prev > MAX_GAP_DEG:
            if len(cur) >= 2:
                rings.append(cur)
            cur = []
        cur.append((deg, r))
        prev = deg
    if len(cur) >= 2:
        rings.append(cur)
    return rings


def write_dxf(path, points):
    rings = split_rings(points)
    xs = [r * math.cos(math.radians(a)) * 1000.0 for a, r in points]
    ys = [r * math.sin(math.radians(a)) * 1000.0 for a, r in points]
    s = []
    s.append("0\nSECTION\n2\nHEADER\n")
    s.append("9\n$INSUNITS\n70\n4\n")
    if xs:
        s.append("9\n$EXTMIN\n10\n%.3f\n20\n%.3f\n" % (min(xs), min(ys)))
        s.append("9\n$EXTMAX\n10\n%.3f\n20\n%.3f\n" % (max(xs), max(ys)))
    else:
        s.append("9\n$EXTMIN\n10\n0.000\n20\n0.000\n")
        s.append("9\n$EXTMAX\n10\n0.000\n20\n0.000\n")
    s.append("0\nENDSEC\n0\nSECTION\n2\nENTITIES\n")
    for ring in rings:
        span = ring[-1][0] - ring[0][0]
        closed = len(ring) >= 3 and span >= FULL_RING_DEG
        s.append("0\nLWPOLYLINE\n8\n0\n90\n%d\n91\n0\n70\n%d\n" % (len(ring), 1 if closed else 0))
        for a, r in ring:
            x = r * math.cos(math.radians(a)) * 1000.0
            y = r * math.sin(math.radians(a)) * 1000.0
            s.append("10\n%.3f\n20\n%.3f\n" % (x + 0.0, y + 0.0))
    s.append("0\nENDSEC\n0\nEOF\n")
    open(path, "w", encoding="utf-8").write("".join(s))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/scan")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--out", default="scan_single")
    args = ap.parse_args()

    rclpy.init()
    col = Collector(args.topic, args.seconds)
    print("Сбор %s (%.1f с; Ctrl-C — стоп)..." % (args.topic, args.seconds))
    ex = SingleThreadedExecutor()
    ex.add_node(col)
    try:
        while rclpy.ok() and time.monotonic() < col.t_end:
            ex.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    ex.shutdown()

    if col.msg_count == 0:
        print("Данных по %s за %.1f с не было.\n"
              "Проверь: ros2 topic list; ros2 topic hz %s; ROS_DOMAIN_ID совпадает с драйвером."
              % (args.topic, args.seconds, args.topic))
        rclpy.shutdown()
        sys.exit(1)

    points = []
    for b, vals in enumerate(col.cells):
        if vals:
            points.append(((b + 0.5) * BUCKET_DEG, sum(vals) / len(vals)))
    points.sort(key=lambda p: p[0])

    if not points:
        print("Сообщения были, но ни одного валидного возврата (все inf/nan/0).")
        rclpy.shutdown()
        sys.exit(0)

    sectors = len({int(a // 10) % 36 for a, _ in points})
    rmin = min(r for _, r in points)
    rmax = max(r for _, r in points)
    print("Сообщений: %d, точек после мерджа: %d, дальность %.2f..%.2f м, секторов 10°: %d/36"
          % (col.msg_count, len(points), rmin, rmax, sectors))

    write_dxf(args.out + ".dxf", points)
    # Круг луча вертикален (⊥ основанию), луч 0° горизонтален: профиль —
    # плоскость XZ (x = r·cos, «y» DXF/XYZ = высота = r·sin).
    with open(args.out + ".xyz", "w", encoding="utf-8") as f:
        for a, r in points:
            f.write("%.3f 0.0 %.3f\n"
                    % (r * math.cos(math.radians(a)), r * math.sin(math.radians(a))))
    print("Готово: %s.dxf, %s.xyz" % (args.out, args.out))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
