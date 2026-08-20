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
}
