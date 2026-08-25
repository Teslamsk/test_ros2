package org.example.export;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.UUID;

/**
 * Экспорт облака точек (картезианские x/y/z) в формате E57 v1.0
 * (подпись "ASTM-E57", стандарт 2010 г. — читают SolidWorks, FARO, PTC и др.).
 *
 * Бинарная раскладка (little-endian, сверена по эталонным файлам
 * библиотеки openE57/pye57):
 * <pre>
 *   0..47   заголовок: "ASTM-E57", major=1, minor=0,
 *           filePhysicalLength, xmlPhysicalOffset, xmlLogicalLength, pageSize=1024
 *   48..    секция CompressedVector: 32-байтный заголовок
 *           (sectionId=1, reserved[7], sectionLogicalLength,
 *            dataPhysicalOffset, indexPhysicalOffset=0)
 *           + цепочка data-пакетов: 6-байтный заголовок (type=1, flags=0,
 *           logicalLength-1, bytestreamCount) + u16 длина каждого стрима + строки
 *   после данных  метаданные XML (namespace 2010-e57-v1.0)
 *               один points-CompressedVector, fileOffset=48, recordCount=N
 *   хвост      нули до кратного PAGE_SIZE
 * </pre>
 *
 * Каждый канал (x, y, z) пишется отдельным bytestream: по float32 на точку
 * (32-битный BitPack для Float = сырой IEEE-754). Пустой вектор codecs в XML
 * означает кодек BitPack по умолчанию.
 */
public final class E57Writer {

    private static final byte[] SIGNATURE = "ASTM-E57".getBytes(StandardCharsets.US_ASCII);
    private static final int PAGE_SIZE = 1024;
    private static final int HEADER_SIZE = 48;
    private static final int SECTION_HEADER_SIZE = 32;
    private static final int PACKET_HEADER_SIZE = 6;
    private static final int MAX_PACKET_SIZE = 64 * 1024;
    private static final int CHANNELS = 3; // x, y, z
    private static final int SECTION_START = HEADER_SIZE;

    private E57Writer() {
    }

    public static void write(Path file, List<float[]> points) throws IOException {
        if (points == null) {
            throw new IllegalArgumentException("points == null");
        }
        int n = points.size();

        double minX = Double.POSITIVE_INFINITY, maxX = Double.NEGATIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;
        double minZ = Double.POSITIVE_INFINITY, maxZ = Double.NEGATIVE_INFINITY;
        for (float[] p : points) {
            minX = Math.min(minX, p[0]);
            maxX = Math.max(maxX, p[0]);
            minY = Math.min(minY, p[1]);
            maxY = Math.max(maxY, p[1]);
            minZ = Math.min(minZ, p[2]);
            maxZ = Math.max(maxZ, p[2]);
        }
        if (n == 0) {
            minX = maxX = minY = maxY = minZ = maxZ = 0.0;
        }

        int pointsPerPacket = (MAX_PACKET_SIZE - PACKET_HEADER_SIZE - 2 * CHANNELS) / (4 * CHANNELS);
        int packetCount = (n + pointsPerPacket - 1) / pointsPerPacket;
        int sectionLength = SECTION_HEADER_SIZE;
        for (int i = 0; i < packetCount; i++) {
            int count = Math.min(pointsPerPacket, n - i * pointsPerPacket);
            sectionLength += PACKET_HEADER_SIZE + 2 * CHANNELS + count * 4 * CHANNELS;
        }

        int xmlOffset = SECTION_START + sectionLength;
        byte[] xml = buildXml(n, minX, maxX, minY, maxY, minZ, maxZ).getBytes(StandardCharsets.UTF_8);
        int physicalLength = ((xmlOffset + xml.length + PAGE_SIZE - 1) / PAGE_SIZE) * PAGE_SIZE;

        ByteBuffer buf = ByteBuffer.allocate(physicalLength).order(ByteOrder.LITTLE_ENDIAN);

        // Заголовок файла (48 байт)
        buf.put(SIGNATURE);
        buf.putInt(1); // majorVersion
        buf.putInt(0); // minorVersion
        buf.putLong(physicalLength);
        buf.putLong(xmlOffset);
        buf.putLong(xml.length);
        buf.putLong(PAGE_SIZE);

        // Заголовок секции CompressedVector (32 байта)
        buf.put((byte) 1); // E57_COMPRESSED_VECTOR_SECTION
        buf.put(new byte[7]);
        buf.putLong(sectionLength);
        buf.putLong(packetCount > 0 ? SECTION_START + SECTION_HEADER_SIZE : xmlOffset);
        buf.putLong(0); // индекс-секция не используется

        // Пакеты данных: каналы идут в отдельном strиме (все x, потом все y, потом все z)
        for (int p = 0; p < packetCount; p++) {
            int start = p * pointsPerPacket;
            int count = Math.min(pointsPerPacket, n - start);
            int packetLogical = PACKET_HEADER_SIZE + 2 * CHANNELS + count * 4 * CHANNELS;
            buf.put((byte) 1); // E57_DATA_PACKET
            buf.put((byte) 0); // flags
            buf.putShort((short) (packetLogical - 1));
            buf.putShort((short) CHANNELS);
            for (int c = 0; c < CHANNELS; c++) {
                buf.putShort((short) (count * 4));
            }
            for (int c = 0; c < CHANNELS; c++) {
                for (int i = 0; i < count; i++) {
                    buf.putFloat(points.get(start + i)[c]);
                }
            }
        }

        // XML-метаданные (хвост файла нулями до PAGE_SIZE)
        buf.put(xml);
        Files.write(file, buf.array());
    }

    private static String buildXml(int n, double minX, double maxX, double minY, double maxY,
                                   double minZ, double maxZ) {
        StringBuilder x = new StringBuilder(4000);
        x.append("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        x.append("<e57Root type=\"Structure\" xmlns=\"http://www.astm.org/COMMIT/E57/2010-e57-v1.0\">\n");
        x.append("  <formatName type=\"String\"><![CDATA[ASTM E57 3D Imaging Data File]]></formatName>\n");
        x.append("  <guid type=\"String\"><![CDATA[{").append(UUID.randomUUID()).append("}]]></guid>\n");
        x.append("  <versionMajor type=\"Integer\">1</versionMajor>\n");
        x.append("  <versionMinor type=\"Integer\"/>\n");
        x.append("  <e57LibraryVersion type=\"String\"><![CDATA[ros2-demo-exporter-1.0]]></e57LibraryVersion>\n");
        x.append("  <coordinateMetadata type=\"String\"/>\n");
        x.append("  <creationDateTime type=\"Structure\">\n");
        x.append("    <dateTimeValue type=\"Float\"/>\n");
        x.append("    <isAtomicClockReferenced type=\"Integer\"/>\n");
        x.append("  </creationDateTime>\n");
        x.append("  <data3D type=\"Vector\" allowHeterogeneousChildren=\"1\">\n");
        x.append("    <vectorChild type=\"Structure\">\n");
        x.append("      <guid type=\"String\"><![CDATA[{").append(UUID.randomUUID()).append("}]]></guid>\n");
        x.append("      <name type=\"String\"><![CDATA[Scan 0]]></name>\n");
        x.append("      <description type=\"String\"><![CDATA[pet-lidar merged point cloud (ros2-demo)]]></description>\n");
        x.append("      <indexBounds type=\"Structure\">\n");
        x.append("        <rowMinimum type=\"Integer\"/>\n");
        // Пустой вектор (n=0): пишем пустой элемент, а не -1
        x.append("        <rowMaximum type=\"Integer\">").append(n > 0 ? n - 1 : "").append("</rowMaximum>\n");
        x.append("        <columnMinimum type=\"Integer\"/>\n");
        x.append("        <columnMaximum type=\"Integer\"/>\n");
        x.append("        <returnMinimum type=\"Integer\"/>\n");
        x.append("        <returnMaximum type=\"Integer\"/>\n");
        x.append("      </indexBounds>\n");
        x.append("      <cartesianBounds type=\"Structure\">\n");
        x.append("        <xMinimum type=\"Float\">").append(minX).append("</xMinimum>\n");
        x.append("        <xMaximum type=\"Float\">").append(maxX).append("</xMaximum>\n");
        x.append("        <yMinimum type=\"Float\">").append(minY).append("</yMinimum>\n");
        x.append("        <yMaximum type=\"Float\">").append(maxY).append("</yMaximum>\n");
        x.append("        <zMinimum type=\"Float\">").append(minZ).append("</zMinimum>\n");
        x.append("        <zMaximum type=\"Float\">").append(maxZ).append("</zMaximum>\n");
        x.append("      </cartesianBounds>\n");
        x.append("      <pose type=\"Structure\">\n");
        x.append("        <rotation type=\"Structure\">\n");
        x.append("          <w type=\"Float\">1.0</w>\n");
        x.append("          <x type=\"Float\">0.0</x>\n");
        x.append("          <y type=\"Float\">0.0</y>\n");
        x.append("          <z type=\"Float\">0.0</z>\n");
        x.append("        </rotation>\n");
        x.append("        <translation type=\"Structure\">\n");
        x.append("          <x type=\"Float\">0.0</x>\n");
        x.append("          <y type=\"Float\">0.0</y>\n");
        x.append("          <z type=\"Float\">0.0</z>\n");
        x.append("        </translation>\n");
        x.append("      </pose>\n");
        x.append("      <acquisitionStart type=\"Structure\">\n");
        x.append("        <dateTimeValue type=\"Float\"/>\n");
        x.append("        <isAtomicClockReferenced type=\"Integer\"/>\n");
        x.append("      </acquisitionStart>\n");
        x.append("      <acquisitionEnd type=\"Structure\">\n");
        x.append("        <dateTimeValue type=\"Float\"/>\n");
        x.append("        <isAtomicClockReferenced type=\"Integer\"/>\n");
        x.append("      </acquisitionEnd>\n");
        x.append("      <points type=\"CompressedVector\" fileOffset=\"").append(SECTION_START)
                .append("\" recordCount=\"").append(n).append("\">\n");
        x.append("        <prototype type=\"Structure\">\n");
        x.append("          <cartesianX type=\"Float\" precision=\"single\" minimum=\"").append(minX)
                .append("\" maximum=\"").append(maxX).append("\">")
                .append(mid(minX, maxX)).append("</cartesianX>\n");
        x.append("          <cartesianY type=\"Float\" precision=\"single\" minimum=\"").append(minY)
                .append("\" maximum=\"").append(maxY).append("\">")
                .append(mid(minY, maxY)).append("</cartesianY>\n");
        x.append("          <cartesianZ type=\"Float\" precision=\"single\" minimum=\"").append(minZ)
                .append("\" maximum=\"").append(maxZ).append("\">")
                .append(mid(minZ, maxZ)).append("</cartesianZ>\n");
        x.append("        </prototype>\n");
        x.append("        <codecs type=\"Vector\" allowHeterogeneousChildren=\"1\"/>\n");
        x.append("      </points>\n");
        x.append("    </vectorChild>\n");
        x.append("  </data3D>\n");
        x.append("  <images2D type=\"Vector\" allowHeterogeneousChildren=\"1\"/>\n");
        x.append("</e57Root>\n");
        return x.toString();
    }

    private static double mid(double min, double max) {
        double m = (min + max) / 2.0;
        return m == 0.0 ? 0.0 : m; // "-0.0" не пишем
    }
}
