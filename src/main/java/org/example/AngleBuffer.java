package org.example;

/**
 * Кольцевой буфер сэмплов энкодера (wall-clock, ns + угол рамы, град).
 *
 * <p>Заполняется фоновым трекером с фиксированным шагом опроса; читается
 * основным потоком, которому нужен угол рамы на произвольный момент времени
 * (метка времени скана). Линейная интерполяция между соседними сэмплами.
 *
 * <p>Правила:
 * <ul>
 *  <li>запрос в пределах maxGapNs до первого или после последнего сэмпла
 *      — возврат угла крайнего сэмпла (клатминг: свежий запрос в пределах
 *      пары шагов опроса от последнего сэмпла не должен давать NaN);</li>
 *  <li>дальше maxGapNs от краёв — NaN (буфер пуст или запрос слишком старый);</li>
 *  <li>разрыв между скобящими сэмплами больше maxGapNs — NaN (трекер
 *      зависал, интерполировать через пропасть нельзя).</li>
 * </ul>
 */
public final class AngleBuffer {

    private final long[] tsNs;
    private final double[] angleDeg;
    private final int capacity;
    private final long maxGapNs;

    /** Индекс самого старого сэмпла в кольце. */
    private int head;
    /** Сколько сэмплов в кольце (<= capacity). */
    private int size;

    public AngleBuffer(int capacity, long maxGapNs) {
        if (capacity <= 0) {
            throw new IllegalArgumentException("capacity должен быть > 0, получен " + capacity);
        }
        if (maxGapNs <= 0) {
            throw new IllegalArgumentException("maxGapNs должен быть > 0, получен " + maxGapNs);
        }
        this.capacity = capacity;
        this.maxGapNs = maxGapNs;
        this.tsNs = new long[capacity];
        this.angleDeg = new double[capacity];
    }

    /**
     * Добавляет сэмпл (время должно не убывать). При переполнении старейший
     * сэмпл вытесняется.
     */
    public synchronized void add(long tsNs, double angleDeg) {
        int i = (head + size) % capacity;
        this.tsNs[i] = tsNs;
        this.angleDeg[i] = angleDeg;
        if (size < capacity) {
            size++;
        } else {
            head = (head + 1) % capacity;
        }
    }

    /**
     * Угол рамы на момент wallNs (ns) линейной интерполяцией.
     *
     * @return угол в градусах, или NaN — буфер пуст / запрос вне зоны доверия.
     */
    public synchronized double angleAt(long wallNs) {
        if (size == 0) {
            return Double.NaN;
        }
        // Бинарный поиск по кольцу: первое j, где ts[j] > wallNs.
        int lo = 0;
        int hi = size;
        while (lo < hi) {
            int mid = (lo + hi) >>> 1;
            if (tsAt(mid) > wallNs) {
                hi = mid;
            } else {
                lo = mid + 1;
            }
        }
        if (lo == 0) {
            // wallNs раньше первого сэмпла: клатминг в пределах maxGapNs.
            if (tsAt(0) - wallNs <= maxGapNs) {
                return angleAt(0);
            }
            return Double.NaN;
        }
        if (lo == size) {
            // wallNs позже последнего сэмпла: клатминг в пределах maxGapNs.
            if (wallNs - tsAt(size - 1) <= maxGapNs) {
                return angleAt(size - 1);
            }
            return Double.NaN;
        }
        long t0 = tsAt(lo - 1);
        long t1 = tsAt(lo);
        if (t1 - t0 > maxGapNs) {
            return Double.NaN; // разрыв — интерполяция недостоверна
        }
        double frac = (double) (wallNs - t0) / (t1 - t0);
        double a0 = angleAt(lo - 1);
        double a1 = angleAt(lo);
        return a0 + frac * (a1 - a0);
    }

    /** Время последнего сэмпла, или -1 если буфер пуст. */
    public synchronized long lastTs() {
        if (size == 0) {
            return -1;
        }
        return tsAt(size - 1);
    }

    public synchronized int size() {
        return size;
    }

    /** Копия времён сэмплов (старые → новые, ns). */
    public synchronized long[] snapshotTimestamps() {
        long[] out = new long[size];
        for (int j = 0; j < size; j++) {
            out[j] = tsAt(j);
        }
        return out;
    }

    /** Копия углов сэмплов (старые → новые, град), в паре с {@link #snapshotTimestamps()}. */
    public synchronized double[] snapshotAngles() {
        double[] out = new double[size];
        for (int j = 0; j < size; j++) {
            out[j] = angleAt(j);
        }
        return out;
    }

    /** Сэмпл по кольцевому индексу j (0 = старейший). */
    private long tsAt(int j) {
        return tsNs[(head + j) % capacity];
    }

    /** Угол по кольцевому индексу j (0 = старейший). */
    private double angleAt(int j) {
        return angleDeg[(head + j) % capacity];
    }
}
