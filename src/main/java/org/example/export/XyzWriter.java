package org.example.export;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

/**
 * Экспорт облака точек в ASCII-формат XYZ: одна строка "x y z" на точку.
 * Универсальный fallback: открывается везде (CloudCompare, SolidWorks, open3d...),
 * в отличие от PLY/PCD.
 */
public final class XyzWriter {

    private XyzWriter() {
    }

    public static void write(Path file, List<float[]> points) throws IOException {
        if (points == null) {
            throw new IllegalArgumentException("points == null");
        }
        StringBuilder sb = new StringBuilder(points.size() * 24);
        for (float[] p : points) {
            sb.append(p[0]).append(' ')
                    .append(p[1]).append(' ')
                    .append(p[2]).append('\n');
        }
        Files.write(file, sb.toString().getBytes(StandardCharsets.UTF_8));
    }
}
