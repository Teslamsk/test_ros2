package org.example;

import org.junit.jupiter.api.Test;
import sensor_msgs.LaserScan;

import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class ScanMergerTest {

    private static LaserScan scan(float angleMinDeg, float incDeg, float... ranges) {
        LaserScan s = new LaserScan();
        s.setAngleMin((float) Math.toRadians(angleMinDeg));
        s.setAngleIncrement((float) Math.toRadians(incDeg));
        s.setAngleMax((float) Math.toRadians(angleMinDeg + incDeg * (ranges.length - 1)));
        s.setRangeMin(0f);
        s.setRangeMax(100f);
        for (float r : ranges) {
            s.getRanges().add(r);
        }
        return s;
    }

    private static LaserScan scan(float angleMinDeg, float incDeg, float[] ranges, float[] intensities) {
        LaserScan s = new LaserScan();
        s.setAngleMin((float) Math.toRadians(angleMinDeg));
        s.setAngleIncrement((float) Math.toRadians(incDeg));
        s.setAngleMax((float) Math.toRadians(angleMinDeg + incDeg * (ranges.length - 1)));
        s.setRangeMin(0f);
        s.setRangeMax(100f);
        for (int i = 0; i < ranges.length; i++) {
            s.getRanges().add(ranges[i]);
            s.getIntensities().add(intensities[i]);
        }
        return s;
    }

    @Test
    void mergeBucketAveragesPhasedScans() {
        int beams = 36; // шаг 10°
        float[] a = new float[beams];
        float[] b = new float[beams];
        Arrays.fill(a, 1.0f);
        Arrays.fill(b, 1.02f); // 2% разброс — в пределах фильтра, усредняется
        LaserScan s1 = scan(0f, 10f, a);
        LaserScan s2 = scan(5f, 10f, b); // фазовый сдвиг — точки должны склеиться по углу

        LaserScan merged = ScanMerger.mergeBucket(List.of(s1, s2), 1, 0.10f);

        assertEquals(beams, merged.getRanges().size());
        for (float r : merged.getRanges()) {
            assertEquals(1.01f, r, 1e-4f);
        }
        // метаданные о реальном массиве: от +X, бакет 10°
        assertEquals(0f, merged.getAngleMin(), 1e-6f);
        assertEquals((float) Math.toRadians(10f), merged.getAngleIncrement(), 1e-6f);
        assertEquals((float) Math.toRadians(350f), merged.getAngleMax(), 1e-6f);
    }

    @Test
    void mergeBucketAveragesIntensityWithRange() {
        int beams = 36; // шаг 10°
        float[] a = new float[beams];
        float[] b = new float[beams];
        float[] ia = new float[beams];
        float[] ib = new float[beams];
        Arrays.fill(a, 1.0f);
        Arrays.fill(b, 1.02f);
        Arrays.fill(ia, 0.4f);
        Arrays.fill(ib, 0.6f);
        LaserScan s1 = scan(0f, 10f, a, ia);
        LaserScan s2 = scan(5f, 10f, b, ib);

        LaserScan merged = ScanMerger.mergeBucket(List.of(s1, s2), 1, 0.10f);

        assertEquals(beams, merged.getIntensities().size());
        for (float v : merged.getIntensities()) {
            assertEquals(0.5f, v, 1e-4f); // усреднена в том же бакете, что и дальность
        }
    }

    @Test
    void mergeRawCarriesIntensitySortedByAngle() {
        // сканы фазированы на 45°: общий порядок точек 0,45,90,... — интенсивности следуют за точками
        LaserScan s1 = scan(0f, 90f,
                new float[]{1f, 2f, 3f, 4f}, new float[]{0.1f, 0.2f, 0.3f, 0.4f});
        LaserScan s2 = scan(45f, 90f,
                new float[]{1f, 2f, 3f, 4f}, new float[]{0.5f, 0.6f, 0.7f, 0.8f});

        LaserScan merged = ScanMerger.mergeRaw(List.of(s1, s2));

        assertEquals(8, merged.getIntensities().size());
        float[] expected = {0.1f, 0.5f, 0.2f, 0.6f, 0.3f, 0.7f, 0.4f, 0.8f};
        for (int i = 0; i < expected.length; i++) {
            assertEquals(expected[i], merged.getIntensities().get(i), 1e-6f);
        }
    }

    @Test
    void mergeBucketFactorChangesResolution() {
        int beams = 12; // шаг 30°
        float[] ones = new float[beams];
        Arrays.fill(ones, 1.0f);
        LaserScan s = scan(0f, 30f, ones);

        LaserScan f1 = ScanMerger.mergeBucket(List.of(s), 1, 0.10f);
        LaserScan f3 = ScanMerger.mergeBucket(List.of(s), 3, 0.10f);

        assertEquals(beams, f1.getRanges().size());
        assertEquals(beams * 3, f3.getRanges().size()); // бакет 10°
        assertEquals((float) Math.toRadians(10f), f3.getAngleIncrement(), 1e-6f);
    }

    @Test
    void mergeBucketFiltersOutlier() {
        int beams = 36;
        List<LaserScan> scans = new java.util.ArrayList<>();
        for (int k = 0; k < 10; k++) {
            float[] v = new float[beams];
            Arrays.fill(v, 1.0f);
            if (k == 3) {
                v[5] = 1.5f; // выброс в одном скане
            }
            scans.add(scan(0f, 10f, v));
        }

        LaserScan merged = ScanMerger.mergeBucket(scans, 1, 0.10f);

        assertEquals(1.0f, merged.getRanges().get(0), 1e-4f);
        assertEquals(1.0f, merged.getRanges().get(5), 1e-4f); // выброс отброшен, среднее чистых точек
    }

    @Test
    void mergeBucketIgnoresNoReturn() {
        int beams = 8;
        float[] ones = new float[beams];
        Arrays.fill(ones, 1.0f);
        float[] bad = new float[beams];
        Arrays.fill(bad, Float.NaN);

        LaserScan merged = ScanMerger.mergeBucket(List.of(scan(0f, 45f, ones), scan(45f, 45f, bad)), 2, 0.10f);

        // 8 валидных точек из первого скана по 16 бакетам (45°/2), половина бакетов пуста
        long valid = 0;
        long empty = 0;
        for (int i = 0; i < merged.getRanges().size(); i++) {
            float r = merged.getRanges().get(i);
            if (Float.isFinite(r)) {
                valid++;
            } else {
                empty++;
            }
        }
        assertEquals(8, valid);
        assertEquals(8, empty);
    }

    @Test
    void mergeRawKeepsAllPointsSortedByAngle() {
        LaserScan s1 = scan(0f, 90f, 1f, 2f, 3f, 4f);
        LaserScan s2 = scan(45f, 90f, 1f, 2f, 3f, 4f);

        LaserScan merged = ScanMerger.mergeRaw(List.of(s1, s2));

        assertEquals(8, merged.getRanges().size());
        // оба скана отсортированы и фазированы на 45° -> общий порядок 0,45,90,...,315
        float[] expected = {1f, 1f, 2f, 2f, 3f, 3f, 4f, 4f};
        for (int i = 0; i < expected.length; i++) {
            assertEquals(expected[i], merged.getRanges().get(i), 1e-6f);
        }
        assertEquals(0f, merged.getAngleMin(), 1e-6f);
        assertEquals((float) (360f / 8f), merged.getAngleIncrement(), 1e-6f);
    }

    @Test
    void mergeRawDropsInvalid() {
        LaserScan s1 = scan(0f, 90f, 1f, Float.NaN, 3f, -5f);

        LaserScan merged = ScanMerger.mergeRaw(List.of(s1));

        assertEquals(2, merged.getRanges().size());
        assertEquals(1f, merged.getRanges().get(0), 1e-6f);
        assertEquals(3f, merged.getRanges().get(1), 1e-6f);
    }

    @Test
    void emptyInputGivesEmptyResult() {
        assertTrue(ScanMerger.mergeBucket(List.of(), 6, 0.1f).getRanges().isEmpty());
        assertTrue(ScanMerger.mergeRaw(List.of()).getRanges().isEmpty());
    }

    @Test
    void badFactorRejected() {
        assertThrows(IllegalArgumentException.class,
                () -> ScanMerger.mergeBucket(List.of(scan(0f, 10f, 1f, 2f)), 0, 0.1f));
        assertThrows(IllegalArgumentException.class,
                () -> ScanMerger.mergeBucket(List.of(scan(0f, 10f, 1f, 2f)), 65, 0.1f));
    }
}
