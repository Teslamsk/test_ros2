package org.example.standalone;

import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

/**
 * Синтетические кадры STL-19P: CRC8 (полином 0x4D, init 0x00, MSB-first),
 * декодирование полей, углы точек start + i·(end−start)/11.
 */
class LiDARReaderTest {

    /** Собирает 47-байтный кадр с валидным CRC. dists — до 12 дистанций (мм). */
    private static byte[] frame(int speed, int startC100, int endC100, int... dists) {
        byte[] f = new byte[LiDARReader.FRAME_LENGTH];
        f[0] = 0x54;
        f[1] = 0x2C;
        putLe16(f, 2, speed);
        putLe16(f, 4, startC100);
        for (int i = 0; i < 12 && i < dists.length; i++) {
            putLe16(f, 6 + 3 * i, dists[i]);
            f[6 + 3 * i + 2] = (byte) 200; // confidence
        }
        putLe16(f, 42, endC100);
        putLe16(f, 44, 1234); // timestamp (мс)
        f[46] = (byte) LiDARReader.crc8(f, 46);
        return f;
    }

    private static void putLe16(byte[] f, int off, int v) {
        f[off] = (byte) v;
        f[off + 1] = (byte) (v >>> 8);
    }

    @Test
    void parsesValidFrame() throws Exception {
        byte[] f = frame(3570, 1000, 1086,
                1500, 0, 2200, 800, 900, 1000, 1100, 1200, 1300, 1400, 1500, 1600);

        LiDARReader.Frame parsed = LiDARReader.parse(f);

        assertEquals(3570, parsed.speedDegPerSec);
        assertEquals(10.00, parsed.startDeg, 1e-9);
        assertEquals(10.86, parsed.endDeg, 1e-9);
        assertEquals(1500, parsed.distancesMm[0]);
        assertEquals(0, parsed.distancesMm[1]);
        assertEquals(2200, parsed.distancesMm[2]);
        assertEquals(1600, parsed.distancesMm[11]);
    }

    @Test
    void pointAngleIsLinearBetweenStartAndEnd() throws Exception {
        // start = 0.00°, end = 1.10° → шаг = 1.10/11 = 0.10°
        LiDARReader.Frame parsed = LiDARReader.parse(
                frame(0, 0, 110, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100));

        assertEquals(0.0, parsed.pointAngleDeg(0), 1e-9);
        assertEquals(0.5, parsed.pointAngleDeg(5), 1e-9);
        assertEquals(1.10, parsed.pointAngleDeg(11), 1e-9);
    }

    @Test
    void rejectsFrameWithBadCrc() {
        byte[] f = frame(0, 0, 110);
        f[10] ^= 0xFF;
        assertThrows(IOException.class, () -> LiDARReader.parse(f));
    }

    @Test
    void rejectsFrameWithBadHeader() {
        byte[] f = frame(0, 0, 110);
        f[1] = (byte) 0x9C;
        assertThrows(IOException.class, () -> LiDARReader.parse(f));
    }

    @Test
    void rejectsWrongLength() {
        byte[] f = frame(0, 0, 110);
        assertThrows(IOException.class, () -> LiDARReader.parse(new byte[46]));
    }
}
