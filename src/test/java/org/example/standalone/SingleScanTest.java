package org.example.standalone;

import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.List;
import java.util.Locale;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Отдельно стоящий тест: ОДИН полный оборот лидара (без ROS2) → один скан в DXF.
 *
 * Запуск: mvn -o -q test -Dtest=SingleScanTest
 * Порт: по умолчанию COM8; переопределить -Dlidar.port=COMx или LIDAR_PORT=COMx.
 * Без подключённого лидара тест пропускается (assumption), а не падает.
 * Результат: scan_export/scan_single_<yyyyMMdd_HHmmss>.dxf.
 */
class SingleScanTest {

    private static final String PORT = port();
    private static final long READ_TIMEOUT_MS = 5000;

    private static String port() {
        String p = System.getProperty("lidar.port", System.getenv("LIDAR_PORT"));
        return p != null && !p.isBlank() ? p : "COM8";
    }

    @Test
    void capturesOneRotationAndWritesSingleScanDxf() throws Exception {
        LiDARReader reader;
        try {
            reader = new LiDARReader(PORT);
        } catch (Exception e) {
            Assumptions.assumeTrue(false, "Лидар недоступен на " + PORT + " — пропуск: " + e.getMessage());
            return;
        }
        try (reader) {
            long t0 = System.currentTimeMillis();
            List<LaserPoint> points = reader.readRotation(READ_TIMEOUT_MS);
            long elapsed = System.currentTimeMillis() - t0;

            Path dir = Paths.get("scan_export");
            Files.createDirectories(dir);
            String name = "scan_single_" + LocalDateTime.now()
                    .format(DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss")) + ".dxf";
            Path file = dir.resolve(name);
            DxfWriter.write(file, points);

            List<LaserPoint> returns = points.stream().filter(p -> p.distanceMm() > 0).toList();
            Assumptions.assumeTrue(!returns.isEmpty(),
                    "Нет ни одного возврата с " + PORT + " — лидар не видит препятствий или не подключён");
            double angleMin = points.stream().mapToDouble(LaserPoint::angleDeg).min().orElse(0);
            double angleMax = points.stream().mapToDouble(LaserPoint::angleDeg).max().orElse(0);
            int distMin = returns.stream().mapToInt(LaserPoint::distanceMm).min().orElse(0);
            int distMax = returns.stream().mapToInt(LaserPoint::distanceMm).max().orElse(0);
            // Угловое покрытие: считаем занятые 10°-ячейки
            boolean[] cells = new boolean[36];
            for (LaserPoint p : points) {
                if (p.distanceMm() > 0) {
                    cells[(int) (p.angleDeg() / 10.0) % 36] = true;
                }
            }
            int occupied = 0;
            for (boolean c : cells) {
                if (c) {
                    occupied++;
                }
            }

            assertTrue(returns.size() >= 100,
                    "Слишком мало точек возврата за оборот: " + returns.size());
            String dxf = Files.readString(file);
            assertTrue(dxf.contains("LWPOLYLINE"));
            assertTrue(dxf.endsWith("EOF\n"));

            System.out.println("[SingleScan] " + PORT + ": точек " + points.size()
                    + " (с возвратом " + returns.size() + "), " + elapsed + " мс");
            System.out.println("[SingleScan] дальность " + distMin + "..." + distMax + " мм, "
                    + String.format(Locale.US, "заполнено %d/36 секторов по 10°", occupied)
                    + ", углы " + String.format(Locale.US, "%.1f..%.1f°", angleMin, angleMax));
            System.out.println("[SingleScan] DXF: " + file.toAbsolutePath());
        }
    }
}
