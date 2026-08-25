package org.example.standalone;

import com.fazecast.jSerialComm.SerialPort;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Читает поток LDROBOT STL-19P напрямую по COM-порту (без ROS2).
 *
 * Кадр 47 байт (little-endian): 0x54 0x2C, speed u16 (°/с), start u16 (0.01°),
 * 12×{distance u16 (мм), confidence u8}, end u16 (0.01° — угол последней точки),
 * timestamp u16 (мс), CRC8 по первым 46 байтам (полином 0x4D, init 0x00, MSB-first,
 * таблица из ldlidar_stl_ros2). Угол i-й точки: start + i·(end−start)/11.
 */
public final class LiDARReader implements AutoCloseable {

    public static final int FRAME_LENGTH = 47;
    public static final int POINTS_PER_FRAME = 12;
    public static final int BAUD_RATE = 230400;
    private static final int CRC_POLYNOM = 0x4D;

    private final SerialPort port;

    public LiDARReader(String portName) throws IOException {
        port = SerialPort.getCommPort(portName);
        port.setComPortParameters(BAUD_RATE, 8, SerialPort.ONE_STOP_BIT, SerialPort.NO_PARITY);
        port.setComPortTimeouts(SerialPort.TIMEOUT_READ_SEMI_BLOCKING, 100, 0);
        if (!port.openPort()) {
            throw new IOException("Не удалось открыть " + portName);
        }
    }

    /** CRC8: полином 0x4D, init 0x00, MSB-first. */
    public static int crc8(byte[] data, int length) {
        int crc = 0;
        for (int i = 0; i < length; i++) {
            crc ^= data[i] & 0xFF;
            for (int bit = 0; bit < 8; bit++) {
                crc = (crc & 0x80) != 0 ? ((crc << 1) ^ CRC_POLYNOM) & 0xFF : (crc << 1) & 0xFF;
            }
        }
        return crc;
    }

    /** Декодирует кадр; бросает IOException при неверном заголовке или CRC. */
    public static Frame parse(byte[] frame) throws IOException {
        if (frame == null || frame.length != FRAME_LENGTH) {
            throw new IOException("Кадр не " + FRAME_LENGTH + " байт");
        }
        if ((frame[0] & 0xFF) != 0x54 || (frame[1] & 0xFF) != 0x2C) {
            throw new IOException("Неверный заголовок кадра");
        }
        if (crc8(frame, 46) != (frame[46] & 0xFF)) {
            throw new IOException("Неверный CRC8");
        }
        int[] distances = new int[POINTS_PER_FRAME];
        for (int i = 0; i < POINTS_PER_FRAME; i++) {
            distances[i] = le16(frame, 6 + 3 * i);
        }
        return new Frame(le16(frame, 2), le16(frame, 4) / 100.0, le16(frame, 42) / 100.0, distances);
    }

    /**
     * Читает один полный оборот: кадры складываются до накопленных 360°.
     * Углы точек нормированы в [0, 360) относительно старта первого кадра.
     */
    public List<LaserPoint> readRotation(long timeoutMs) throws IOException {
        long deadline = System.currentTimeMillis() + timeoutMs;
        List<LaserPoint> points = new ArrayList<>();
        byte[] frame = new byte[FRAME_LENGTH];
        int idx = 0;
        boolean first = true;
        double prevRawEnd = 0;
        double unwrappedEnd = 0;

        while (System.currentTimeMillis() < deadline) {
            int n = port.readBytes(frame, idx, 1);
            if (n <= 0) {
                continue;
            }
            int b = frame[idx] & 0xFF;
            if (idx == 0) {
                if (b == 0x54) {
                    idx = 1;
                }
                continue;
            }
            if (idx == 1) {
                if (b == 0x2C) {
                    idx = 2;
                } else if (b != 0x54) {
                    idx = 0;
                }
                continue;
            }
            idx++;
            if (idx < FRAME_LENGTH) {
                continue;
            }
            idx = 0;
            Frame f;
            try {
                f = parse(frame);
            } catch (IOException bad) {
                continue; // рассинхрон — ищем следующий заголовок
            }
            double span = f.endDeg - f.startDeg;
            span = (span + 540.0) % 360.0 - 180.0; // в диапазон (-180, 180]
            double relStart = first ? 0.0 : f.startDeg - prevRawEnd + 360.0;
            relStart = (relStart + 180.0) % 360.0 - 180.0; // соседний кадр — на пол-оборота не дальше
            for (int i = 0; i < POINTS_PER_FRAME; i++) {
                double angle = relStart + f.pointAngleDeg(i);
                double norm = ((angle % 360.0) + 360.0) % 360.0;
                points.add(new LaserPoint(norm, f.distancesMm[i]));
            }
            first = false;
            prevRawEnd = f.endDeg;
            unwrappedEnd = relStart + span;
            if (unwrappedEnd >= 360.0) {
                break;
            }
        }
        if (points.isEmpty() || unwrappedEnd < 360.0) {
            throw new IOException("Полный оборот не собран за " + timeoutMs + " мс (собрано " + unwrappedEnd + "°)");
        }
        return points;
    }

    private static int le16(byte[] d, int off) {
        return (d[off] & 0xFF) | ((d[off + 1] & 0xFF) << 8);
    }

    @Override
    public void close() {
        if (port.isOpen()) {
            port.closePort();
        }
    }

    /** Декодированный кадр: углы в градусах, дистанции в мм. */
    public static final class Frame {
        public final int speedDegPerSec;
        public final double startDeg;
        public final double endDeg;
        public final int[] distancesMm;

        Frame(int speedDegPerSec, double startDeg, double endDeg, int[] distancesMm) {
            this.speedDegPerSec = speedDegPerSec;
            this.startDeg = startDeg;
            this.endDeg = endDeg;
            this.distancesMm = distancesMm;
        }

        /** Угол i-й точки: start + i·(end−start)/11. */
        public double pointAngleDeg(int i) {
            return startDeg + i * (endDeg - startDeg) / (POINTS_PER_FRAME - 1);
        }
    }
}
