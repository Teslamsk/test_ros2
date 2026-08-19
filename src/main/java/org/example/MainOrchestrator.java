package org.example;

import org.example.can.CanController;
import org.example.can.dictionary.Node;
import org.example.can.transport.CanBus;
import org.example.can.transport.CanBusFactory;
import sensor_msgs.LaserScan;

import java.util.ArrayList;
import java.util.List;

public class MainOrchestrator {

    private static final float NO_RETURN = Float.NaN;
    private static final float OUTLIER_TOLERANCE = 0.10f;

    public static void main(String[] args) throws InterruptedException {
        System.out.println("=== Main Orchestrator Started ===");
        TopicInterface ros = new TopicInterface("orchestrator_node");
        CanBus bus = CanBusFactory.forCurrentOS(); // транспорт под текущую ОС
        CanController can = new CanController(bus);
        can.init("/dev/can0"); // Инициализация с CAN-интерфейсом

        int scanSteps = 18;
        int scansPerStep = 10;
        float stepDeg = 180.0f / scanSteps;

        for (int i = 0; i < scanSteps; i++) {
            float angle = i * stepDeg;
            can.turnToAbsoluteAngle(Node.ROTATE_LIDAR_Z, angle);
            can.awaitTargetReached(Node.ROTATE_LIDAR_Z, 5000);

            List<LaserScan> rawScans = ros.collectScans(scansPerStep);
            System.out.println("[MAIN] Merging " + rawScans.size() + " scans for angle " + angle);
            LaserScan merged = mergeScans(rawScans);

            ros.publishAndAwaitProcessed(merged, angle).await();
            System.out.println("[MAIN] Cloud updated at " + angle + " deg");
        }

        System.out.println("=== 360 Scan Complete ===");
        can.close();
        ros.close();
    }

    private static LaserScan mergeScans(List<LaserScan> scans) {
        if (scans.isEmpty()) {
            return new LaserScan();
        }

        LaserScan first = scans.get(0);
        int beamsPerScan = first.getRanges().size();
        // Лидар стреляет с фиксированной частотой: сканы фазированы относительно
        // друга, луч i в разных сканах — разные направления. Поэтому не усредняем
        // "по индексу", а суммируем точки по углу: бакет = 1/6 шага луча —
        // склеиваются только почти совпавшие направления, квантование угла
        // ~ 1/12 шага (для 240 лучей ~ 0.1°). Для "просто сканирования" этого
        // достаточно, точные углы потом можно поправить по облаку; для SLAM
        // потребовалось бы хранить пары (угол, дальность) без сетки.
        double bucketDeg = 360.0 / beamsPerScan / 6.0;
        int buckets = (int) Math.round(360.0 / bucketDeg);

        List<List<Float>> cells = new ArrayList<>(buckets);
        for (int b = 0; b < buckets; b++) {
            cells.add(new ArrayList<>());
        }

        for (LaserScan scan : scans) {
            for (int i = 0; i < scan.getRanges().size(); i++) {
                float r = scan.getRanges().get(i);
                if (!Float.isFinite(r) || r <= 0f) {
                    continue;
                }
                double deg = Math.toDegrees(scan.getAngleMin() + i * scan.getAngleIncrement());
                deg = ((deg % 360.0) + 360.0) % 360.0;
                cells.get((int) (deg / bucketDeg) % buckets).add(r);
            }
        }

        LaserScan result = new LaserScan(scans.get(scans.size() - 1));
        result.setAngleMin(first.getAngleMin());
        result.setAngleMax(first.getAngleMax());
        result.setAngleIncrement((float) Math.toRadians(bucketDeg));
        result.getRanges().clear();
        for (int b = 0; b < buckets; b++) {
            List<Float> points = cells.get(b);
            float value = NO_RETURN;
            if (!points.isEmpty()) {
                double mean = points.stream().mapToDouble(Float::doubleValue).average().orElse(0.0);
                List<Float> kept = points.stream()
                        .filter(v -> Math.abs(v - mean) <= OUTLIER_TOLERANCE * Math.abs(mean))
                        .toList();
                if (!kept.isEmpty()) {
                    value = (float) kept.stream().mapToDouble(Float::doubleValue).average().orElse(NO_RETURN);
                }
            }
            result.getRanges().add(value);
        }

        return result;
    }
}
