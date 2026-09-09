package org.example.vlp16;

import org.junit.jupiter.api.Test;
import sensor_msgs.PointCloud2;
import sensor_msgs.PointField;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Тесты парсинга PointCloud2 (синтетические облака, без ROS).
 *
 * <p>Ключевой сценарий: драйвер velodyne отдаёт x/y/z/intensity как FLOAT32
 * (velodyne layout: x0 y4 z8 intensity12, point_step 16), но парсер намеренно
 * независим от layout — поля ищутся по имени, offset/step/endianness — из
 * сообщения. Проверены: стандартный layout, extra-поля, UINT8 intensity,
 * big-endian, отсутствие intensity, невалидные облака.
 */
class Vlp16CloudParseTest {

    private static PointField field(String name, int offset, byte dt) {
        PointField f = new PointField();
        f.setName(name);
        f.setOffset(offset);
        f.setDatatype(dt);
        f.setCount(1);
        return f;
    }

    private static PointCloud2 cloud(List<PointField> fields, int width, int pointStep,
                                     byte[] data, boolean bigEndian) {
        PointCloud2 c = new PointCloud2();
        c.getFields().clear();
        for (PointField f : fields) {
            c.getFields().add(f);
        }
        c.setHeight(1);
        c.setWidth(width);
        c.setIsBigendian(bigEndian);
        c.setPointStep(pointStep);
        c.setRowStep(pointStep * width);
        c.getData().clear();
        c.getData().addAll(data);
        c.setIsDense(true);
        return c;
    }

    private static void assertPoint(float[] expected, float[] actual, float eps) {
        assertEquals(expected.length, actual.length);
        for (int i = 0; i < expected.length; i++) {
            assertEquals(expected[i], actual[i], eps);
        }
    }

    @Test
    void parsesStandardVelodyneLayout() {
        ByteBuffer b = ByteBuffer.allocate(3 * 16).order(ByteOrder.LITTLE_ENDIAN);
        b.putFloat(1f).putFloat(2f).putFloat(3f).putFloat(200f);
        b.putFloat(4f).putFloat(5f).putFloat(6f).putFloat(0f);
        b.putFloat(-1f).putFloat(0f).putFloat(0f).putFloat(7f);
        PointCloud2 c = cloud(List.of(
                field("x", 0, PointField.FLOAT32),
                field("y", 4, PointField.FLOAT32),
                field("z", 8, PointField.FLOAT32),
                field("intensity", 12, PointField.FLOAT32)),
                3, 16, b.array(), false);
        List<float[]> pts = Vlp16Cloud.parse(c);
        assertEquals(3, pts.size());
        assertPoint(new float[]{1f, 2f, 3f, 200f}, pts.get(0), 1e-6f);
        assertPoint(new float[]{4f, 5f, 6f, 0f}, pts.get(1), 1e-6f);
        assertPoint(new float[]{-1f, 0f, 0f, 7f}, pts.get(2), 1e-6f);
    }

    @Test
    void ignoresExtraFieldsAndUsesOffsets() {
        // extra-поле "ring" (UINT16) между z и intensity: x0 y4 z8 ring12 [pad14] intensity16, step 24
        ByteBuffer b = ByteBuffer.allocate(2 * 24).order(ByteOrder.LITTLE_ENDIAN);
        b.putFloat(1f).putFloat(2f).putFloat(3f).putShort((short) 5).putShort((short) 0).putFloat(99f)
                .putShort((short) 0).putShort((short) 0);
        b.putFloat(7f).putFloat(8f).putFloat(9f).putShort((short) 6).putShort((short) 0).putFloat(1f);
        PointCloud2 c = cloud(List.of(
                field("x", 0, PointField.FLOAT32),
                field("y", 4, PointField.FLOAT32),
                field("z", 8, PointField.FLOAT32),
                field("ring", 12, PointField.UINT16),
                field("intensity", 16, PointField.FLOAT32)),
                2, 24, b.array(), false);
        List<float[]> pts = Vlp16Cloud.parse(c);
        assertEquals(2, pts.size());
        assertPoint(new float[]{1f, 2f, 3f, 99f}, pts.get(0), 1e-6f);
        assertPoint(new float[]{7f, 8f, 9f, 1f}, pts.get(1), 1e-6f);
    }

    @Test
    void parsesUint8Intensity() {
        ByteBuffer b = ByteBuffer.allocate(2 * 13).order(ByteOrder.LITTLE_ENDIAN);
        b.putFloat(1f).putFloat(0f).putFloat(0f).put((byte) 200);
        b.putFloat(2f).putFloat(0f).putFloat(0f).put((byte) 5);
        PointCloud2 c = cloud(List.of(
                field("x", 0, PointField.FLOAT32),
                field("y", 4, PointField.FLOAT32),
                field("z", 8, PointField.FLOAT32),
                field("intensity", 12, PointField.UINT8)),
                2, 13, b.array(), false);
        List<float[]> pts = Vlp16Cloud.parse(c);
        assertEquals(200f, pts.get(0)[3], 1e-6f);
        assertEquals(5f, pts.get(1)[3], 1e-6f);
    }

    @Test
    void zeroIntensityWhenFieldAbsent() {
        ByteBuffer b = ByteBuffer.allocate(12).order(ByteOrder.LITTLE_ENDIAN);
        b.putFloat(1f).putFloat(2f).putFloat(3f);
        PointCloud2 c = cloud(List.of(
                field("x", 0, PointField.FLOAT32),
                field("y", 4, PointField.FLOAT32),
                field("z", 8, PointField.FLOAT32)),
                1, 12, b.array(), false);
        List<float[]> pts = Vlp16Cloud.parse(c);
        assertEquals(1, pts.size());
        assertPoint(new float[]{1f, 2f, 3f, 0f}, pts.get(0), 1e-6f);
    }

    @Test
    void parsesBigEndian() {
        ByteBuffer b = ByteBuffer.allocate(16).order(ByteOrder.BIG_ENDIAN);
        b.putFloat(1f).putFloat(2f).putFloat(3f).putFloat(100f);
        PointCloud2 c = cloud(List.of(
                field("x", 0, PointField.FLOAT32),
                field("y", 4, PointField.FLOAT32),
                field("z", 8, PointField.FLOAT32),
                field("intensity", 12, PointField.FLOAT32)),
                1, 16, b.array(), true);
        List<float[]> pts = Vlp16Cloud.parse(c);
        assertPoint(new float[]{1f, 2f, 3f, 100f}, pts.get(0), 1e-6f);
    }

    @Test
    void multiRowCloudParsesAllPoints() {
        // height=2, width=2: 4 точки, row_step = point_step * width
        ByteBuffer b = ByteBuffer.allocate(4 * 12).order(ByteOrder.LITTLE_ENDIAN);
        for (int i = 0; i < 4; i++) {
            b.putFloat(i).putFloat(i + 1).putFloat(i + 2);
        }
        PointCloud2 c = cloud(List.of(
                field("x", 0, PointField.FLOAT32),
                field("y", 4, PointField.FLOAT32),
                field("z", 8, PointField.FLOAT32)),
                2, 12, b.array(), false);
        c.setHeight(2);
        List<float[]> pts = Vlp16Cloud.parse(c);
        assertEquals(4, pts.size());
        assertPoint(new float[]{2f, 3f, 4f, 0f}, pts.get(2), 1e-6f);
        assertPoint(new float[]{3f, 4f, 5f, 0f}, pts.get(3), 1e-6f);
    }

    @Test
    void emptyOrInvalidCloudGivesEmpty() {
        assertTrue(Vlp16Cloud.parse(null).isEmpty());

        PointCloud2 empty = new PointCloud2();
        empty.setWidth(0);
        empty.setHeight(1);
        assertTrue(Vlp16Cloud.parse(empty).isEmpty());

        // нет обязательного поля x
        PointCloud2 noX = cloud(List.of(
                field("y", 0, PointField.FLOAT32),
                field("z", 4, PointField.FLOAT32)),
                1, 8, new byte[8], false);
        assertTrue(Vlp16Cloud.parse(noX).isEmpty());
    }
}
