package org.example.export;

import org.junit.jupiter.api.Test;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class XyzWriterTest {

    @Test
    void writesOnePointPerLine() throws Exception {
        List<float[]> points = List.of(
                new float[]{1.5f, -2.25f, 0.0f},
                new float[]{3.0f, 4.5f, -1.0f}
        );
        Path file = Files.createTempFile("xyztest", ".xyz");

        XyzWriter.write(file, points);

        String content = Files.readString(file, StandardCharsets.UTF_8);
        assertEquals("1.5 -2.25 0.0\n3.0 4.5 -1.0\n", content);
    }

    @Test
    void emptyCloudWritesEmptyFile() throws Exception {
        Path file = Files.createTempFile("xyzempty", ".xyz");
        XyzWriter.write(file, List.of());
        assertEquals(0, Files.size(file));
    }

    @Test
    void streamWritesPointsIncrementally() throws Exception {
        Path file = Files.createTempFile("xyzstream", ".xyz");
        try (XyzWriter.Stream s = XyzWriter.open(file)) {
            s.writePoint(new float[]{1.0f, 2.0f, 3.0f});
            assertEquals(1, s.count());
            s.writePoint(new float[]{4.0f, 5.0f, 6.0f});
            assertEquals(2, s.count());
        }
        String content = Files.readString(file, StandardCharsets.UTF_8);
        assertEquals("1.0 2.0 3.0\n4.0 5.0 6.0\n", content);
    }
}
