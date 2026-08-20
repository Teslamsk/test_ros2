package org.example.export;

import org.junit.jupiter.api.Test;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class E57WriterTest {

    /**
     * Полный byte-level разбор маленького файла E57 v1.0:
     * подпись, заголовок 48B, секция CompressedVector 32B,
     * data-пакет 6B + 3 bytestream (float32 по каналу), XML с bounds.
     */
    @Test
    void fourPointsProduceValidE57Structure() throws Exception {
        List<float[]> points = List.of(
                new float[]{1.5f, -2.25f, 0.0f},
                new float[]{3.0f, 4.5f, -1.0f},
                new float[]{-0.5f, 0.5f, 2.0f},
                new float[]{7.75f, 8.0f, 9.25f}
        );
        Path file = Files.createTempFile("e57test", ".e57");

        E57Writer.write(file, points);

        byte[] b = Files.readAllBytes(file);
        ByteBuffer buf = ByteBuffer.wrap(b).order(ByteOrder.LITTLE_ENDIAN);

        // --- заголовок файла (48 байт)
        assertEquals("ASTM-E57", StandardCharsets.US_ASCII.decode(buf.slice(0, 8)).toString());
        assertEquals(1, buf.getInt(8));  // majorVersion
        assertEquals(0, buf.getInt(12)); // minorVersion
        assertEquals(b.length, buf.getLong(16)); // filePhysicalLength = фактический размер
        assertEquals(0, b.length % 1024);        // кратный pageSize
        long xmlOffset = buf.getLong(24);
        long xmlLength = buf.getLong(32);
        assertEquals(1024, buf.getLong(40)); // pageSize
        assertTrue(xmlOffset + xmlLength <= b.length);

        // --- секция CompressedVector (32 байта) на 48
        assertEquals(1, b[48] & 0xFF);                 // sectionId
        long sectionLength = buf.getLong(56);
        long dataPhysicalOffset = buf.getLong(64);
        long indexPhysicalOffset = buf.getLong(72);
        for (int i = 49; i < 56; i++) {
            assertEquals(0, b[i] & 0xFF, "reserved байт " + i);
        }
        assertEquals(80, dataPhysicalOffset);          // первый пакет сразу после 32B-заголовка секции
        assertEquals(0, indexPhysicalOffset);
        assertEquals(32 + 6 + 6 + 4 * 4 * 3, sectionLength); // 32 + пакет(4 точки)

        // --- data-пакет на 80
        assertEquals(1, b[80] & 0xFF);                            // E57_DATA_PACKET
        assertEquals(0, b[81] & 0xFF);                            // flags
        assertEquals(6 + 6 + 4 * 4 * 3 - 1, readU16(b, 82));      // logicalLengthMinus1
        assertEquals(3, readU16(b, 84));                          // bytestreamCount
        assertEquals(16, readU16(b, 86));                         // длина strим x (4×4)
        assertEquals(16, readU16(b, 88));
        assertEquals(16, readU16(b, 90));

        // --- данные: каналы отдельно (все x, все y, все z), порядок точек сохранён
        assertFloat(buf.getFloat(92), 1.5f);
        assertFloat(buf.getFloat(96), 3.0f);
        assertFloat(buf.getFloat(100), -0.5f);
        assertFloat(buf.getFloat(104), 7.75f);
        assertFloat(buf.getFloat(108), -2.25f);
        assertFloat(buf.getFloat(112), 4.5f);
        assertFloat(buf.getFloat(116), 0.5f);
        assertFloat(buf.getFloat(120), 8.0f);
        assertFloat(buf.getFloat(124), 0.0f);
        assertFloat(buf.getFloat(128), -1.0f);
        assertFloat(buf.getFloat(132), 2.0f);
        assertFloat(buf.getFloat(136), 9.25f);

        // --- XML
        String xml = StandardCharsets.UTF_8.decode(buf.slice((int) xmlOffset, (int) xmlLength)).toString();
        assertTrue(xml.contains("xmlns=\"http://www.astm.org/COMMIT/E57/2010-e57-v1.0\""));
        assertTrue(xml.contains("recordCount=\"4\""));
        assertTrue(xml.contains("fileOffset=\"48\""));
        assertTrue(xml.contains("<xMinimum type=\"Float\">-0.5</xMinimum>"));
        assertTrue(xml.contains("<xMaximum type=\"Float\">7.75</xMaximum>"));
        assertTrue(xml.contains("<yMinimum type=\"Float\">-2.25</yMinimum>"));
        assertTrue(xml.contains("<zMaximum type=\"Float\">9.25</zMaximum>"));
        assertTrue(xml.endsWith("</e57Root>\n"));
    }

    /**
     * Пакет ограничен 64K: 7000 точек -> два пакета с верной цепочкой,
     * вторые координаты на границе пакета читаются из следующего пакета.
     */
    @Test
    void largeCloudIsSplitIntoPacketChain() throws Exception {
        Random rnd = new Random(42);
        List<float[]> points = new ArrayList<>(7000);
        for (int i = 0; i < 7000; i++) {
            points.add(new float[]{rnd.nextFloat() * 10 - 5, rnd.nextFloat() * 10 - 5, rnd.nextFloat() * 10 - 5});
        }
        Path file = Files.createTempFile("e57big", ".e57");
        E57Writer.write(file, points);

        byte[] b = Files.readAllBytes(file);
        ByteBuffer buf = ByteBuffer.wrap(b).order(ByteOrder.LITTLE_ENDIAN);

        int pointsPerPacket = (64 * 1024 - 6 - 6) / 12; // 5460
        int firstPacketSize = 6 + 6 + pointsPerPacket * 4 * 3;
        int secondPacketStart = 80 + firstPacketSize;
        assertFalse(secondPacketStart >= b.length);
        assertEquals(1, b[secondPacketStart] & 0xFF);                 // второй пакет
        int secondCount = 7000 - pointsPerPacket;
        assertEquals(6 + 6 + secondCount * 4 * 3 - 1, readU16(b, secondPacketStart + 2));

        // первая точка второго пакета лежит в начале его x-strим
        int xStreamStart = secondPacketStart + 6 + 6;
        assertFloat(buf.getFloat(xStreamStart), points.get(pointsPerPacket)[0]);
    }

    /**
     * Пустое облако: файл валиден, recordCount=0, данные отсутствуют.
     */
    @Test
    void emptyCloudStillWritesValidFile() throws Exception {
        Path file = Files.createTempFile("e57empty", ".e57");
        E57Writer.write(file, List.of());

        byte[] b = Files.readAllBytes(file);
        ByteBuffer buf = ByteBuffer.wrap(b).order(ByteOrder.LITTLE_ENDIAN);
        assertEquals("ASTM-E57", StandardCharsets.US_ASCII.decode(buf.slice(0, 8)).toString());
        assertEquals(0, b.length % 1024);
        long xmlOffset = buf.getLong(24);
        assertEquals(48 + 32, xmlOffset); // только заголовок секции, без пакетов
        String xml = StandardCharsets.UTF_8
                .decode(buf.slice((int) xmlOffset, (int) buf.getLong(32))).toString();
        assertTrue(xml.contains("recordCount=\"0\""));
    }

    private static int readU16(byte[] b, int offset) {
        return (b[offset] & 0xFF) | ((b[offset + 1] & 0xFF) << 8);
    }

    private static void assertFloat(float actual, float expected) {
        assertTrue(Float.floatToIntBits(actual) == Float.floatToIntBits(expected),
                "expected " + expected + " got " + actual);
    }
}
