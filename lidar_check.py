"""Проверка работоспособности лидара LDROBOT STL-19P (D500) по COM-порту.

Лидар шлёт только поток (230400 8N1, команд в потоке нет).
Кадр (little-endian): 0x54 0x2C | speed u16 | start_angle u16 (град/100) |
12 x {distance u16, confidence u8} | end_angle u16 | timestamp | CRC8.
Длина кадра и параметры CRC8 определяются автоматически.

Запуск: python lidar_check.py [--port COM8] [--baud 230400] [--duration 5]
"""
import argparse
import struct
import time

import serial

SYNC = b"\x54\x2C"


def crc8(data: bytes, poly: int, init: int) -> int:
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (((crc << 1) ^ poly) & 0xFF) if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def find_syncs(buf: bytes):
    out, i = [], 0
    while True:
        i = buf.find(SYNC, i)
        if i == -1:
            return out
        out.append(i)
        i += 2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM8")
    ap.add_argument("--baud", type=int, default=230400)
    ap.add_argument("--duration", type=float, default=5.0, help="секунд чтения")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=1)
    deadline = time.time() + args.duration
    buf = b""
    while time.time() < deadline:
        chunk = ser.read(4096)
        if chunk:
            buf += chunk
    ser.close()
    print(f"прочитано: {len(buf)} байт за {args.duration:.1f} c "
          f"({len(buf) / max(args.duration, 1e-9) / 1024:.1f} КБ/с)")
    if len(buf) < 50:
        print("слишком мало данных - проверьте порт и питание")
        return 1

    syncs = find_syncs(buf)
    print(f"заголовков 0x54 0x2C: {len(syncs)}")
    if len(syncs) < 2:
        print("кадров мало, первые 96 байтов как есть:")
        print(buf[:96].hex(" "))
        return 1

    gaps = [b - a for a, b in zip(syncs, syncs[1:]) if b - a < 200]
    stride = max(set(gaps), key=gaps.count)
    print(f"длина кадра: {stride}")

    body_len = stride - 1
    ok = 0
    for s in syncs:
        frame = buf[s:s + stride]
        if len(frame) == stride and crc8(frame[:body_len], 0x4D, 0x00) == frame[body_len]:
            ok += 1
    print(f"CRC8 (полином 0x4D, init 0x00, MSB-first): "
          f"валидно {ok}/{len(syncs)} кадров")

    if stride < 42:
        print("кадр короче, чем ожидается - поля не декодируются")
        return 1

    ts_len = 4 if stride == 49 else 2
    ts_fmt = "<I" if ts_len == 4 else "<H"
    print(f"кадров/с ~ {len(syncs) / args.duration:.1f}\n"
          "кадр | скорость | углы (старт..конец, град) | timestamp | 12 точек (дист, конф)")
    total_pts = 0
    max_dist = 0
    for s in syncs:
        f = buf[s:s + stride]
        if len(f) != stride:
            continue
        speed, start_a, end_a = struct.unpack_from("<HHH", f, 2)
        ts = struct.unpack_from(ts_fmt, f, 44)[0]
        pts = [struct.unpack_from("<HB", f, 6 + 3 * i) for i in range(12)]
        nz = [d for d, c in pts if c > 0]
        total_pts += len(nz)
        max_dist = max(max_dist, max((d for d, _ in pts), default=0))
        print(f"{s:6d} | {speed:6d} | {start_a / 100:7.2f}..{end_a / 100:7.2f} | "
              f"{ts} | " + " ".join(f"{d:5d}:{c}" for d, c in pts))

    n_frames = len([s for s in syncs if s + stride <= len(buf)])
    if n_frames:
        print(f"\nитого: ~{n_frames} кадров, точек с конфиденсией>0: {total_pts}, "
              f"макс. сырое расстояние: {max_dist}")


if __name__ == "__main__":
    raise SystemExit(main())
