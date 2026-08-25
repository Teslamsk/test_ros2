package org.example.standalone;

import org.junit.jupiter.api.Test;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * DXF R2000/LWPOLYLINE: единицы мм, замыкание кольца по покрытию угла,
 * разбивка на полилайны по разрывам.
 */
class DxfWriterTest {

    @Test
    void fullRingWritesClosedPolyline() throws Exception {
        Path file = Files.createTempFile("dxf", ".dxf");
        DxfWriter.write(file, ring(5, 1000));

        String dxf = Files.readString(file, StandardCharsets.UTF_8);
        assertTrue(dxf.contains("$INSUNITS"));
        assertTrue(dxf.contains("70\n4\n"), "единицы = миллиметры");
        assertEquals(1, count(dxf, "LWPOLYLINE"), "одно кольцо");
        assertTrue(dxf.contains("70\n1\n"), "кольцо замкнуто");
        assertTrue(dxf.contains("10\n1000.000\n20\n0.000\n"), "точка 0°: x=1000, y=0");
        assertTrue(dxf.contains("20\n-1000.000\n"), "90°: y=-1000 (CW-зеркало)");
        assertTrue(dxf.endsWith("EOF\n"));
    }

    @Test
    void gapSplitsIntoOpenPolylines() throws Exception {
        List<LaserPoint> pts = new ArrayList<>();
        for (int a = 0; a <= 170; a += 5) {
            pts.add(new LaserPoint(a, 1000));
        }
        for (int a = 200; a <= 355; a += 5) {
            pts.add(new LaserPoint(a, 1000));
        }
        Path file = Files.createTempFile("dxf", ".dxf");
        DxfWriter.write(file, pts);

        String dxf = Files.readString(file, StandardCharsets.UTF_8);
        assertEquals(2, count(dxf, "LWPOLYLINE"), "разрыв 30° делит на два");
        assertFalse(dxf.contains("70\n1\n"), "обе полилайны открытые");
    }

    @Test
    void zeroDistancePointsAreSkipped() throws Exception {
        List<LaserPoint> pts = new ArrayList<>();
        for (int a = 0; a < 360; a += 5) {
            // сектор 90..175° — без возврата
            pts.add(new LaserPoint(a, a >= 90 && a < 180 ? 0 : 1000));
        }
        Path file = Files.createTempFile("dxf", ".dxf");
        DxfWriter.write(file, pts);

        String dxf = Files.readString(file, StandardCharsets.UTF_8);
        assertEquals(2, count(dxf, "LWPOLYLINE"), "дыра 90..175° делит кольцо на два открытых");
    }

    @Test
    void emptyPointsWriteHeaderOnly() throws Exception {
        Path file = Files.createTempFile("dxf", ".dxf");
        DxfWriter.write(file, List.of());

        String dxf = Files.readString(file, StandardCharsets.UTF_8);
        assertFalse(dxf.contains("LWPOLYLINE"));
        assertTrue(dxf.contains("EOF"));
    }

    private static List<LaserPoint> ring(int stepDeg, int radiusMm) {
        List<LaserPoint> pts = new ArrayList<>();
        for (int a = 0; a < 360; a += stepDeg) {
            pts.add(new LaserPoint(a, radiusMm));
        }
        return pts;
    }

    private static int count(String s, String sub) {
        int n = 0;
        int i = 0;
        while ((i = s.indexOf(sub, i)) != -1) {
            n++;
            i += sub.length();
        }
        return n;
    }
}
