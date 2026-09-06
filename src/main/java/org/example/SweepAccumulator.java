package org.example;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

/**
 * Накопитель точек свипа по индексам лучей (бин = колонка лидара) с
 * скользящим окном по углу рамы.
 *
 * <p>Рама вращается медленно; каждый луч (бин) видит одну физическую область
 * пока рама не повернётся на ширину окна. Пока точка свежая (её прогресс
 * внутри {@code [progressNow - windowDeg, progressNow]}) — она остаётся в
 * бине; свежая точка того же бина добавляется, и по бину выдаётся одна
 * смерженная точка (средний угол рамы, усреднение дальности/интенсивности
 * с отсечением выбросов, средний азимут). Старые точки (рама ушла дальше
 * окна) выбрасываются.
 *
 * <p>Окно — в «прогресс»-пространстве: {@code progress = dirSign * frameAngle},
 * поэтому одинаково работает и в CW (угол уменьшается), и в CCW (растёт):
 * прогресс всегда растёт по ходу движения.
 *
 * <p>Однопоточный: вызывается из главного потока оркестратора.
 */
public final class SweepAccumulator {

    private final double windowDeg;
    private final float outlierTolerance;
    /** бин (индекс луча) -> свежие точки. */
    private final Map<Integer, List<SweepPoint>> bins = new HashMap<>();

    /**
     * @param windowDeg        ширина окна, град поворота рамы (прогресс)
     * @param outlierTolerance относительная отсечка выбросов при усреднении
     */
    public SweepAccumulator(double windowDeg, float outlierTolerance) {
        if (!(windowDeg > 0)) {
            throw new IllegalArgumentException("windowDeg должен быть > 0, получен " + windowDeg);
        }
        this.windowDeg = windowDeg;
        this.outlierTolerance = outlierTolerance;
    }

    public SweepAccumulator(double windowDeg) {
        this(windowDeg, ScanMerger.DEFAULT_OUTLIER_TOLERANCE);
    }

    /**
     * Добавляет точки свежего среза и возвращает смерженные по бинам точки,
     * чьи сэмплы ещё внутри окна на момент frameAngleNow.
     *
     * @param slicePoints   точки среза (каждая несёт свой frameAngleDeg)
     * @param frameAngleNow текущий угол рамы, град
     * @param dirSign       знак фактического движения энкодера: +1 (угол растёт) / -1 (уменьшается)
     */
    public List<SweepPoint> addAndWindow(List<SweepPoint> slicePoints, double frameAngleNow, int dirSign) {
        if (Double.isNaN(frameAngleNow)) {
            // Нет данных энкодера — окно не сдвигается, новых точек не принимаем.
            return List.of();
        }
        for (SweepPoint p : slicePoints) {
            if (Double.isNaN(p.frameAngleDeg())) {
                continue;
            }
            bins.computeIfAbsent(p.bin(), k -> new ArrayList<>()).add(p);
        }

        double progressNow = dirSign * frameAngleNow;
        double windowStart = progressNow - windowDeg;

        List<SweepPoint> out = new ArrayList<>();
        Iterator<Map.Entry<Integer, List<SweepPoint>>> it = bins.entrySet().iterator();
        while (it.hasNext()) {
            List<SweepPoint> pts = it.next().getValue();
            pts.removeIf(p -> dirSign * p.frameAngleDeg() < windowStart);
            if (pts.isEmpty()) {
                it.remove();
            } else {
                out.add(mergeBin(pts));
            }
        }
        return out;
    }

    /** Счётчик точек, накопленных по всем бинам (для диагностики). */
    public int totalPoints() {
        int n = 0;
        for (List<SweepPoint> pts : bins.values()) {
            n += pts.size();
        }
        return n;
    }

    private SweepPoint mergeBin(List<SweepPoint> pts) {
        double sumFrame = 0.0;
        double sumTheta = 0.0;
        List<float[]> pairs = new ArrayList<>(pts.size());
        for (SweepPoint p : pts) {
            sumFrame += p.frameAngleDeg();
            sumTheta += p.thetaRad();
            pairs.add(new float[]{p.range(), p.intensity()});
        }
        float[] av = ScanMerger.filterAverage(pairs, outlierTolerance);
        int bin = pts.get(0).bin();
        return new SweepPoint(
                sumFrame / pts.size(),
                av[0],
                av[1],
                (float) (sumTheta / pts.size()),
                bin);
    }
}
