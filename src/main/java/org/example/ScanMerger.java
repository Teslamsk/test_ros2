package org.example;

import sensor_msgs.LaserScan;
import us.ihmc.fastddsjava.cdr.idl.IDLFloatSequence;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Стратегии мержа сырых LaserScans в один скан.
 *
 * Все результаты несут метаданные, описывающие РЕАЛЬНЫЙ массив:
 * угол_i = angleMin + i * angleIncrement (лидарная система, 0 = +X, CCW).
 *
 * BUCKET: точки группируются в угловые бакеты (шаг луча / factor),
 *          в бакете — среднее выбросов (отбрасываются значения, отклоняющиеся
 *          от среднего больше чем на outlierTol); пустой бакет = NaN.
 * RAW:    все валидные точки всех сканов — отдельными точками, без усреднения
 *          (сортируются по углу); «чистку» потом делать в постпроцессоре.
 */
public final class ScanMerger {

    public static final float NO_RETURN = Float.NaN;
    public static final float DEFAULT_OUTLIER_TOLERANCE = 0.10f;

    private ScanMerger() {
    }

    /**
     * Угловые бакеты: bucket = (360 / beamsPerScan) / factor, фактор 1..64.
     */
    public static LaserScan mergeBucket(List<LaserScan> scans, int factor, float outlierTolerance) {
        if (scans.isEmpty()) {
            return new LaserScan();
        }
        if (factor < 1 || factor > 64) {
            throw new IllegalArgumentException("factor должен быть 1..64, получен " + factor);
        }

        LaserScan first = scans.get(0);
        int beamsPerScan = first.getRanges().size();
        double bucketDeg = 360.0 / beamsPerScan / factor;
        int buckets = (int) Math.round(360.0 / bucketDeg);

        // Лидар стреляет с фиксированной частотой: сканы фазированы относительно
        // друга, луч i в разных сканах — разные направления. Поэтому не усредняем
        // "по индексу", а суммируем точки по углу: бакет = 1/factor шага луча —
        // склеиваются только почти совпавшие направления.
        // Пара (дальность, интенсивность) на точку: интенсивность усредняется
        // в том же бакете, что и дальность (после мержа индексы с сырыми не совпадают).
        List<List<float[]>> cells = new ArrayList<>(buckets);
        for (int b = 0; b < buckets; b++) {
            cells.add(new ArrayList<>());
        }

        for (LaserScan scan : scans) {
            IDLFloatSequence intensities = scan.getIntensities();
            for (int i = 0; i < scan.getRanges().size(); i++) {
                float r = scan.getRanges().get(i);
                if (!Float.isFinite(r) || r <= 0f) {
                    continue;
                }
                double ang = scan.getAngleMin() + i * scan.getAngleIncrement(); // рад, float-сложение
                double deg = Math.toDegrees(ang);
                deg = ((deg % 360.0) + 360.0) % 360.0;
                // +eps: float-ошибка угла может чуть сдвинуть точку за границу бакета,
                // у точки, лежащей РОВНО на границе (10i°), floor без eps упал бы влево.
                int idx = (int) Math.floor(deg / bucketDeg + 1e-2) % buckets;
                cells.get(idx).add(new float[]{r, pointIntensity(intensities, i)});
            }
        }

        LaserScan result = new LaserScan(scans.get(scans.size() - 1));
        // Метаданные о реальном массиве: бакет b смотрит в [b*bucket, (b+1)*bucket)
        // от +X (точки нормированы в [0, 360)); скопированный из драйвера angleMin
        // повёрнул бы облако на свою величину у каждого потребителя.
        result.setAngleMin(0f);
        result.setAngleMax((float) Math.toRadians((buckets - 1) * bucketDeg));
        result.setAngleIncrement((float) Math.toRadians(bucketDeg));
        result.getRanges().clear();
        result.getIntensities().clear();
        for (int b = 0; b < buckets; b++) {
            float[] av = filterAverage(cells.get(b), outlierTolerance);
            result.getRanges().add(av[0]);
            result.getIntensities().add(av[1]);
        }

        return result;
    }

    /**
     * Интенсивность i-й точки скана; если в скане интенсивности не заполнены
     * (не все драйверы пишут это поле) — 0.
     */
    public static float pointIntensity(IDLFloatSequence intensities, int i) {
        if (intensities == null || i >= intensities.size()) {
            return 0f;
        }
        float v = intensities.get(i);
        return Float.isFinite(v) ? v : 0f;
    }

    /**
     * Усреднение пары (дальность, интенсивность) с отсечкой выбросов по медиане:
     * опорой служит медиана дальностей (один крупный выброс не тащит среднее и
     * не вырезает весь бакет целиком), затем усредняются оставшиеся точки.
     *
     * @return {@code {value, intensity}}, либо {@code {NaN, NaN}} если точек нет
     *         или все вырезаны фильтром.
     */
    public static float[] filterAverage(List<float[]> points, float outlierTolerance) {
        float value = NO_RETURN;
        float intensity = NO_RETURN;
        if (points != null && !points.isEmpty()) {
            float median = median(points);
            List<float[]> kept = points.stream()
                    .filter(p -> Math.abs(p[0] - median) <= outlierTolerance * Math.abs(median))
                    .toList();
            if (!kept.isEmpty()) {
                double sumR = 0.0;
                double sumI = 0.0;
                for (float[] p : kept) {
                    sumR += p[0];
                    sumI += p[1];
                }
                value = (float) (sumR / kept.size());
                intensity = (float) (sumI / kept.size());
            }
        }
        return new float[]{value, intensity};
    }

    private static float median(List<float[]> points) {
        List<Float> sorted = new ArrayList<>(points.size());
        for (float[] p : points) {
            sorted.add(p[0]);
        }
        sorted.sort(Float::compare);
        int n = sorted.size();
        if (n % 2 == 1) {
            return sorted.get(n / 2);
        }
        return (float) ((sorted.get(n / 2 - 1) + sorted.get(n / 2)) / 2.0);
    }

    /**
     * Без усреднения: все валидные точки всех сканов, отсортированные по углу.
     * Метаданные приближённые: равномерный increment = 360/точек (для RViz-просмотра
     * достаточно; точные углы — в облаке /pointCloud).
     */
    public static LaserScan mergeRaw(List<LaserScan> scans) {
        if (scans.isEmpty()) {
            return new LaserScan();
        }

        record Beam(double deg, float range, float intensity) {
        }

        List<Beam> points = new ArrayList<>();
        for (LaserScan scan : scans) {
            IDLFloatSequence intensities = scan.getIntensities();
            for (int i = 0; i < scan.getRanges().size(); i++) {
                float r = scan.getRanges().get(i);
                if (!Float.isFinite(r) || r <= 0f) {
                    continue;
                }
                double deg = Math.toDegrees(scan.getAngleMin() + i * scan.getAngleIncrement());
                deg = ((deg % 360.0) + 360.0) % 360.0;
                points.add(new Beam(deg, r, pointIntensity(intensities, i)));
            }
        }
        points.sort(Comparator.comparingDouble(Beam::deg));

        LaserScan result = new LaserScan(scans.get(scans.size() - 1));
        int n = points.size();
        result.getRanges().clear();
        result.getIntensities().clear();
        if (n == 0) {
            result.setAngleMin(0f);
            result.setAngleMax(0f);
            result.setAngleIncrement(0f);
        } else {
            float step = (float) (360.0 / n);
            for (Beam p : points) {
                result.getRanges().add(p.range);
                result.getIntensities().add(p.intensity);
            }
            result.setAngleMin((float) Math.toRadians(points.get(0).deg));
            result.setAngleMax(result.getAngleMin() + step * (n - 1));
            result.setAngleIncrement(step);
        }

        return result;
    }
}
