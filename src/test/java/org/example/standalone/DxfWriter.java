package org.example.standalone;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

/**
 * Экспорт одного скана в DXF R2000: LWPOLYLINE, единицы — миллиметры ($INSUNITS = 4).
 *
 * Точки сортируются по углу; разрыв (нет возврата / скачок угла > 20°) — новый полилайн.
 * Кольцо, накрывающее >= 340°, замыкается (70 = 1). Y зеркален по оси: лидар
 * вращается CW (левая система координат), RViz/ROS — CCW.
 */
public final class DxfWriter {

    private static final double MAX_GAP_DEG = 20.0;
    private static final double FULL_RING_DEG = 340.0;

    private DxfWriter() {
    }

    public static void write(Path file, List<LaserPoint> points) throws IOException {
        if (points == null) {
            throw new IllegalArgumentException("points == null");
        }
        List<LaserPoint> valid = points.stream()
                .filter(p -> p.distanceMm() > 0)
                .sorted(Comparator.comparingDouble(LaserPoint::angleDeg))
                .toList();
        List<List<LaserPoint>> rings = splitIntoRings(valid);

        double minX = 0, minY = 0, maxX = 0, maxY = 0;
        boolean any = false;
        for (List<LaserPoint> ring : rings) {
            for (LaserPoint p : ring) {
                double[] xy = xy(p);
                if (!any) {
                    minX = maxX = xy[0];
                    minY = maxY = xy[1];
                    any = true;
                } else {
                    minX = Math.min(minX, xy[0]);
                    maxX = Math.max(maxX, xy[0]);
                    minY = Math.min(minY, xy[1]);
                    maxY = Math.max(maxY, xy[1]);
                }
            }
        }

        StringBuilder sb = new StringBuilder(16 * 1024);
        sb.append("0\nSECTION\n2\nHEADER\n");
        sb.append("9\n$INSUNITS\n70\n4\n");
        sb.append("9\n$EXTMIN\n10\n").append(num(minX)).append("\n20\n").append(num(minY)).append("\n");
        sb.append("9\n$EXTMAX\n10\n").append(num(maxX)).append("\n20\n").append(num(maxY)).append("\n");
        sb.append("0\nENDSEC\n");
        sb.append("0\nSECTION\n2\nENTITIES\n");
        for (List<LaserPoint> ring : rings) {
            double span = ring.get(ring.size() - 1).angleDeg() - ring.get(0).angleDeg();
            boolean closed = ring.size() >= 3 && span >= FULL_RING_DEG;
            sb.append("0\nLWPOLYLINE\n8\n0\n");
            sb.append("90\n").append(ring.size()).append("\n91\n0\n");
            sb.append("70\n").append(closed ? 1 : 0).append("\n");
            for (LaserPoint p : ring) {
                double[] xy = xy(p);
                sb.append("10\n").append(num(xy[0])).append("\n20\n").append(num(xy[1])).append("\n");
            }
        }
        sb.append("0\nENDSEC\n0\nEOF\n");
        Files.write(file, sb.toString().getBytes(StandardCharsets.UTF_8));
    }

    /** x, y (мм) точки: зеркалим Y под CW-вращение лидара. */
    private static double[] xy(LaserPoint p) {
        double rad = Math.toRadians(p.angleDeg());
        double x = p.distanceMm() * Math.cos(rad);
        double y = -p.distanceMm() * Math.sin(rad);
        return new double[]{x, y};
    }

    private static List<List<LaserPoint>> splitIntoRings(List<LaserPoint> sortedByAngle) {
        List<List<LaserPoint>> rings = new ArrayList<>();
        List<LaserPoint> cur = new ArrayList<>();
        for (LaserPoint p : sortedByAngle) {
            if (!cur.isEmpty() && p.angleDeg() - cur.get(cur.size() - 1).angleDeg() > MAX_GAP_DEG) {
                rings.add(cur);
                cur = new ArrayList<>();
            }
            cur.add(p);
        }
        if (!cur.isEmpty()) {
            rings.add(cur);
        }
        rings.removeIf(r -> r.size() < 2);
        return rings;
    }

    /** "%.3f" (Locale.US) с гашением -0.000. */
    private static String num(double v) {
        return String.format(Locale.US, "%.3f", v + 0.0);
    }
}
