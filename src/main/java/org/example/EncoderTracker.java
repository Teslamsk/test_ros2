package org.example;

import java.util.function.DoubleSupplier;

/**
 * Фоновый трекер угла рамы: отдельный daemon-поток опрашивает источник угла
 * (обычно {@code Mks42dController.getAngle() / GEAR_RATIO}) каждые pollMs и
 * складывает сэмплы в {@link AngleBuffer}.
 *
 * <p>Время в буфере — единая эпоха трекера: {@code wallNowNs()} монотонна и
 * совпадает в трекере и у читателя, поэтому метки сканов (преобразованные
 * в ns той же эпохи через разницу wall-time) корректно интерполируются.
 *
 * <p>Источник передаётся как {@link DoubleSupplier} — контроллер не
 * нужен на прямую, тесты подставляют лямбду.
 */
public final class EncoderTracker implements AutoCloseable {

    private static final int BUFFER_CAPACITY = 8192;

    private final DoubleSupplier angleSource;
    private final AngleBuffer buffer;
    private final int pollMs;
    private final long baseNs;
    private final Thread thread;
    private volatile boolean running = true;

    /**
     * @param angleSource источник угла рамы в градусах (опрашивается фоном)
     * @param pollMs      шаг опроса, мс
     */
    public EncoderTracker(DoubleSupplier angleSource, int pollMs) {
        if (angleSource == null) {
            throw new IllegalArgumentException("angleSource == null");
        }
        if (pollMs <= 0) {
            throw new IllegalArgumentException("pollMs должен быть > 0, получен " + pollMs);
        }
        this.angleSource = angleSource;
        this.pollMs = pollMs;
        // Эпоха: wall-time (мс → ns) минус nanoTime — wallNowNs() = baseNs + nanoTime
        // монотонна и совпадает с System.currentTimeMillis() * 1e6 (±дрейф nanoTime).
        this.baseNs = System.currentTimeMillis() * 1_000_000L - System.nanoTime();
        this.buffer = new AngleBuffer(BUFFER_CAPACITY, pollMs * 4L * 1_000_000L);
        this.thread = new Thread(this::loop, "encoder-tracker");
        this.thread.setDaemon(true);
        this.thread.start();
    }

    private void loop() {
        try {
            while (running) {
                double angle = angleSource.getAsDouble();
                if (Double.isFinite(angle)) {
                    buffer.add(wallNowNs(), angle);
                }
                Thread.sleep(pollMs);
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /** Текущий wall-clock в эпохе трекера (ns). */
    public long wallNowNs() {
        return baseNs + System.nanoTime();
    }

    /** Угол рамы (град) на момент wallNs, интерполяцией из буфера; NaN вне зоны доверия. */
    public double angleAt(long wallNs) {
        return buffer.angleAt(wallNs);
    }

    /** Время последнего сэмпла (ns), или -1 если ещё нет сэмплов. */
    public long lastSampleNs() {
        return buffer.lastTs();
    }

    /** Копия времён сэмплов (старые → новые, ns) — для диагностики/калибровки. */
    public long[] snapshotTimestamps() {
        return buffer.snapshotTimestamps();
    }

    /** Копия углов (старые → новые, град) — для диагностики/калибровки. */
    public double[] snapshotAngles() {
        return buffer.snapshotAngles();
    }

    @Override
    public void close() {
        running = false;
        try {
            thread.join(500);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
