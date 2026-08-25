package org.example;

import org.example.can.Mks42dController;
import org.example.can.transport.CanBus;
import org.example.can.transport.CanBusFactory;
import org.example.export.XyzWriter;
import sensor_msgs.LaserScan;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.List;

public class MainOrchestrator {

    private static final int BUCKET_FACTOR = 6;
    private static final float OUTLIER_TOLERANCE = 0.10f;

    // Передаточное число мотор:рама = 2:1 (2 оборота мотора = 1 оборот рамы)
    private static final double GEAR_RATIO = 2.0;

    public static void main(String[] args) {
        System.out.println("=== Main Orchestrator Started ===");
        // Первый позиционный аргумент — имя SocketCAN-интерфейса ("can0"), НЕ путь "/dev/can0";
        // на Windows PcanBus параметр не использует. --sweep/--step/--scans — параметры прогона.
        String canIface = "can0";
        double sweepDeg = 90.0; // ход ОСНОВАНИЯ в градусах (без рамы — умеренно, кабель не перекрутить)
        float stepDeg = 10.0f;  // шаг ОСНОВАНИЯ
        int scansPerStep = 10;
        for (int i = 0; i < args.length; i++) {
            switch (args[i]) {
                case "--sweep": sweepDeg = Double.parseDouble(args[++i]); break;
                case "--step":  stepDeg = Float.parseFloat(args[++i]); break;
                case "--scans": scansPerStep = Integer.parseInt(args[++i]); break;
                default:        canIface = args[i]; break;
            }
        }

        TopicInterface ros = new TopicInterface("orchestrator_node");
        CanBus bus = CanBusFactory.forCurrentOS(); // транспорт под текущую ОС
        Mks42dController can = new Mks42dController(bus);
        try {
            can.init(canIface); // шина CAN 2.0A + привод MKS 42D (node 01, 500 kbit/s)

            int scanSteps = (int) Math.round(sweepDeg / stepDeg);
            System.out.printf("[MAIN] Sweep: base %.1f deg, step %.1f deg (%d positions), motor = %.1fx base%n",
                    sweepDeg, stepDeg, scanSteps + 1, GEAR_RATIO);

            for (int i = 0; i <= scanSteps; i++) {
                float baseAngle = i * stepDeg; // физический поворот основания (лечит облако)
                double motorAngle = baseAngle * GEAR_RATIO; // угол, который задаём мотору
                boolean arrived = can.turnToAbsoluteAngle(motorAngle);
                if (!arrived) {
                    System.err.printf("[MAIN] Motor did not stop at motor %.1f deg (base %.1f deg) — checking status%n",
                            motorAngle, baseAngle);
                }

                List<LaserScan> rawScans = ros.collectScans(scansPerStep);
                System.out.printf("[MAIN] Merging %d scans for base %.1f deg (motor %.1f deg)%n",
                        rawScans.size(), baseAngle, motorAngle);
                LaserScan merged = ScanMerger.mergeBucket(rawScans, BUCKET_FACTOR, OUTLIER_TOLERANCE);

                // Облако поворачиваем на угол ОСНОВАНИЯ (реальный поворот планки лидара)
                ros.publishAndProcess(merged, baseAngle);
                System.out.println("[MAIN] Cloud updated at base " + baseAngle + " deg");
            }
            can.turnToAbsoluteAngle(0.0); // возврат основания в нуль
            System.out.println("=== Base sweep Complete ===");
        } catch (Exception e) {
            // не глыбаемся: выгружаем то, что успели собрать, и корректно закрываем ресурсы
            System.err.println("[MAIN] Scan aborted: " + e);
        }
        try {
            exportCloud(ros.getCloudPoints());
        } catch (IOException e) {
            System.err.println("[MAIN] Cloud export failed: " + e);
        } finally {
            can.close();
            ros.close();
        }
    }

    private static void exportCloud(List<float[]> points) throws IOException {
        Path dir = Paths.get("scan_export");
        Files.createDirectories(dir);
        String stamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss"));
        Path xyz = dir.resolve("scan_" + stamp + ".xyz");
        XyzWriter.write(xyz, points);
        System.out.println("[MAIN] Exported " + points.size() + " points -> " + xyz);
    }

}
