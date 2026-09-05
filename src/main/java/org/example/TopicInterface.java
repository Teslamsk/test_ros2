package org.example;

import builtin_interfaces.Time;
import sensor_msgs.LaserScan;
import sensor_msgs.PointCloud2;
import sensor_msgs.PointField;
import us.ihmc.fastddsjava.cdr.idl.IDLFloatSequence;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Topic;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

/**
 * ROS2-нод PET-пайплайна:
 * - подписан на /scan (лидар), накапливает слайсы;
 * - публикует мержнутый скан в /processedScan (для внешних ROS2-потребителей);
 * - синхронно строит облако (в frame того же /scan) и публикует в /pointCloud.
 *
 * Модель потоков:
 * - callback /scan (поток executor'а ROS) пишет в scanBuffer под scanLock;
 * - облако строится в потоке вызывающего (publishAndProcess); cloudPoints
 *   защищён cloudLock, снапшоты — через getCloudPoints().
 *
 * Раньше облако строилось в callback на echo собственного /processedScan:
 * это было гонкой (callback прежнего угла мог сосчитать latch нового, потерянный
 * echo = вечный ханг). Теперь обработка синхронная — гонок нет.
 */
public class TopicInterface {
    private final ROS2Node node;
    private final ROS2Publisher<LaserScan> pubProcessedScan;
    private final ROS2Publisher<PointCloud2> pubPointCloud;

    /** Буфер /scan. Все доступы под scanLock. */
    private final Object scanLock = new Object();
    private final List<LaserScan> scanBuffer = new ArrayList<>();

    /** Точки облака в системе координат /scan (frame_id копируется из скана): {x, y, z, intensity}. Все доступы под cloudLock. */
    private final Object cloudLock = new Object();
    private final List<float[]> cloudPoints = new ArrayList<>();

    private static final int POINT_STEP = 16; // x + y + z + intensity, каждый float32 (4 байта)
    private static final long SCAN_TIMEOUT_MS = 30_000;

    /** Точки ближе этого (м) в облако не попадают: рама/основание лидара попадает в луч
     *  на короткой дальности и видна на скане. */
    static final float MIN_CLOUD_RANGE_M = 0.15f;

    /**
     * Frame облака в свип-режиме: единая мировая система (не frame /scan).
     * Угол рамы каждой точки уже "выпекается" в координаты, поэтому RViz
     * дисплей PointCloud2 в Fixed frame = base показывает готовый объём без TF.
     */
    public static final String BASE_FRAME_ID = "base";

    /** Механический наклон луча 0° лидара относительно горизонтали (рад): + — луч 0° смотрит вверх. */
    private final float tiltRad;

    /** Порог интенсивности ("confidence") 0..255: точки с intensity ниже порога в облако не попадают. 0 — фильтр выключен. */
    private final float minIntensity;

    public TopicInterface(String ns) {
        this(ns, 0f, 0f);
    }

    public TopicInterface(String ns, float tiltDeg) {
        this(ns, tiltDeg, 0f);
    }

    public TopicInterface(String ns, float tiltDeg, float minIntensity) {
        this.tiltRad = (float) Math.toRadians(tiltDeg);
        this.minIntensity = minIntensity;
        this.node = new ROS2Node(ns);
        this.pubProcessedScan = node.createPublisher(new ROS2Topic<LaserScan>("/processedScan", LaserScan.class));
        this.pubPointCloud = node.createPublisher(new ROS2Topic<PointCloud2>("/pointCloud", PointCloud2.class));

        startListeningToScan();
    }

    private void startListeningToScan() {
        node.createSubscription(
                new ROS2Topic<LaserScan>("/scan", LaserScan.class),
                reader -> {
                    LaserScan scan = reader.read();
                    synchronized (scanLock) {
                        scanBuffer.add(scan);
                        scanLock.notifyAll();
                    }
                }
        );
        System.out.println("[ROS] Subscribed to /scan");
    }

    /**
     * Ждёт `count` слайсов с /scan. Buffer очищается под тем же локом,
     * поэтому первый слайс нового окна не может "уехать" в приём предыдущего.
     *
     * @throws InterruptedException  если поток прерван.
     * @throws IllegalStateException если за SCAN_TIMEOUT_MS пришло меньше `count` слайсов.
     */
    public List<LaserScan> collectScans(int count) throws InterruptedException {
        if (count > 1) {
            System.out.println("[ROS] Waiting for " + count + " scans from /scan...");
        }
        synchronized (scanLock) {
            scanBuffer.clear();
            long deadline = System.currentTimeMillis() + SCAN_TIMEOUT_MS;
            while (scanBuffer.size() < count) {
                long remaining = deadline - System.currentTimeMillis();
                if (remaining <= 0) {
                    throw new IllegalStateException(
                            "[ROS] timeout: получено " + scanBuffer.size() + "/" + count
                                    + " сканов за " + SCAN_TIMEOUT_MS + " ms (лидар молчит?)");
                }
                scanLock.wait(Math.min(remaining, 100));
            }
            return new ArrayList<>(scanBuffer);
        }
    }

    /**
     * Публикует мержнутый скан в /processedScan, синхронно добавляет срез в облако
     * и публикует обновлённый /pointCloud.
     */
    public void publishAndProcess(LaserScan avgScan, float angle) {
        pubProcessedScan.publish(avgScan);
        PointCloud2 cloud = addSliceToCloud(avgScan, angle);
        pubPointCloud.publish(cloud);
        System.out.println("[ROS] Published to /processedScan and /pointCloud (angle " + angle + ")");
    }

    // ==================== Свип-режим ====================

    /**
     * Метка времени скана в ns (sec * 1e9 + nanosec) — эпоха wall-clock,
     * совместимая с {@link EncoderTracker#wallNowNs()}.
     */
    public static long stampNs(LaserScan scan) {
        Time t = scan.getHeader().getStamp();
        return t.getSec() * 1_000_000_000L + t.getNanosec();
    }

    /**
     * Точки одного среза для свипа: валидные по дальности (с ограничением
     * maxRangeM), угол рамы каждой точки восстановлен по энкодеру на метку
     * времени скана. Точки среза с прогрессом меньше tensionDeg отбрасываются
     * целиком (эластичный ремень: первые градусы хода недостоверны).
     *
     * <p>Static, tiltRad не нужен: наклон луча применяется позже в
     * {@link #worldPoint(SweepPoint)}.
     *
     * @param scan         срез лидара
     * @param angleAt      источник угла рамы по wall-clock (ns)
     * @param dirSign      +1 (CCW) / -1 (CW)
     * @param tensionDeg   зона натяжения у старта, град (прогресс)
     * @param maxRangeM    ограничение дальности, м (<= rangeMax сенсора)
     * @param minIntensity порог интенсивности 0..255 (0 — фильтр выключен)
     */
    public static List<SweepPoint> sweepSlicePoints(LaserScan scan, FrameAngleSampler angleAt,
                                                    int dirSign, double tensionDeg, double maxRangeM,
                                                    float minIntensity) {
        double frameAngle = angleAt.angleAt(stampNs(scan));
        double progress = dirSign * frameAngle;
        if (Double.isNaN(frameAngle) || progress < tensionDeg) {
            return List.of();
        }
        float rangeMin = scan.getRangeMin();
        float rangeMax = (float) Math.min(scan.getRangeMax(), maxRangeM);
        IDLFloatSequence intensities = scan.getIntensities();
        List<SweepPoint> out = new ArrayList<>(scan.getRanges().size());
        for (int i = 0; i < scan.getRanges().size(); i++) {
            float r = scan.getRanges().get(i);
            if (!inCloudRange(r, rangeMin, rangeMax)) {
                continue;
            }
            float intensity = ScanMerger.pointIntensity(intensities, i);
            if (minIntensity > 0f && intensity < minIntensity) {
                continue; // низкая confidence — отбрасываем
            }
            float theta = scan.getAngleMin() + i * scan.getAngleIncrement();
            out.add(new SweepPoint(frameAngle, r, intensity, theta, i));
        }
        return out;
    }

    /**
     * Мировая координата точки среза (система base): рама повёрнута на
     * frameAngleDeg, наклон луча 0° = tiltRad. Интенсивность — у вызывающего.
     */
    public float[] worldPoint(SweepPoint p) {
        return toWorldPoint(p.range(), p.thetaRad(), (float) p.frameAngleDeg(), tiltRad);
    }

    /**
     * Публикует срез свипа (дельту) в /pointCloud и накапливает в облако.
     * В RViz: Fixed frame = base, Display = Accumulate — объём растёт "на лету".
     */
    public void publishSweepSlice(List<SweepPoint> slice) {
        if (slice == null || slice.isEmpty()) {
            return;
        }
        List<float[]> newPoints = new ArrayList<>(slice.size());
        for (SweepPoint p : slice) {
            float[] w = worldPoint(p);
            newPoints.add(new float[]{w[0], w[1], w[2], p.intensity()});
        }
        synchronized (cloudLock) {
            cloudPoints.addAll(newPoints);
        }
        pubPointCloud.publish(buildPointCloud(newPoints, BASE_FRAME_ID, rosTimeNow()));
    }

    /**
     * Публикует накопленное облако целиком в /pointCloud (финальный кадр свипа).
     */
    public void publishFullCloud() {
        List<float[]> snapshot;
        synchronized (cloudLock) {
            snapshot = new ArrayList<>(cloudPoints);
        }
        pubPointCloud.publish(buildPointCloud(snapshot, BASE_FRAME_ID, rosTimeNow()));
        System.out.println("[ROS] Published full cloud: " + snapshot.size() + " points");
    }

    /** Текущее wall-time как ROS Time (sec/nanosec, int). */
    public static Time rosTimeNow() {
        long now = System.currentTimeMillis();
        Time t = new Time();
        t.setSec((int) (now / 1000));
        t.setNanosec((int) (now % 1000 * 1_000_000));
        return t;
    }

    /**
     * Снапшот накопленных точек облака (копия, можно свободно копировать/экспортировать).
     */
    public List<float[]> getCloudPoints() {
        synchronized (cloudLock) {
            return new ArrayList<>(cloudPoints);
        }
    }

    public void close() {
        node.close();
    }

    /**
     * Точка для облака: валидный возврат, в доверенном диапазоне сенсора и не
     * ближе MIN_CLOUD_RANGE_M (рама лидара в луче).
     */
    static boolean inCloudRange(float r, float rangeMin, float rangeMax) {
        if (!Float.isFinite(r) || r <= 0f) {
            return false; // нет возврата
        }
        if (r < MIN_CLOUD_RANGE_M) {
            return false; // рама/основание — не сканируем
        }
        return (rangeMin <= 0f || r >= rangeMin) && (rangeMax <= 0f || r <= rangeMax); // доверенный диапазон
    }

    /**
     * Знак вертикали: +1 — луч 90° смотрит вверх, -1 — вниз.
     * Если облако в RViz/CloudCompare оказалось перевернутым (верх/низ), поменяйте на -1.
     */
    static final float UP_SIGN = 1f;

    /**
     * Точка среза в мировых координатах. Круг луча лидара вертикален
     * (перпендикулярен основанию), луч 0° горизонтален вдоль +X при нуле
     * основания; основание повёрнуто на baseAngleDeg вокруг оси Z:
     *   x = r·cos(θ)·cos(A), y = r·cos(θ)·sin(A), z = UP_SIGN·r·sin(θ).
     * Срез лежит в вертикальной плоскости, содержащей ось Z — вращение
     * основания даёт объём, а не плоскость XY.
     */
    static float[] toWorldPoint(float r, float theta, float baseAngleDeg) {
        return toWorldPoint(r, theta, baseAngleDeg, 0f);
    }

    /**
     * tiltRad — механический наклон луча 0° лидара относительно горизонтали
     * (+ — луч 0° смотрит вверх): истинная высота луча с углом θ равна θ + tiltRad.
     */
    static float[] toWorldPoint(float r, float theta, float baseAngleDeg, float tiltRad) {
        double a = Math.toRadians(baseAngleDeg);
        float horiz = (float) (r * Math.cos(theta + tiltRad)); // радиальная (горизонтальная) составляющая
        float vert = (float) (r * Math.sin(theta + tiltRad));  // вертикальная составляющая
        return new float[]{
                (float) (horiz * Math.cos(a)),
                (float) (horiz * Math.sin(a)),
                (float) (UP_SIGN * vert)
        };
    }

    /**
     * Поворачивает срез на угол основания (окружность вокруг оси Z):
     * "локальная" система лидара -> мировая, и добавляет точки в облако.
     * Вызывается синхронно из потока оркестратора.
     */
    private PointCloud2 addSliceToCloud(LaserScan slice, float angle) {
        float rangeMin = slice.getRangeMin();
        float rangeMax = slice.getRangeMax();
        IDLFloatSequence intensities = slice.getIntensities();
        int slicePoints = 0;
        List<float[]> newPoints = new ArrayList<>();
        for (int i = 0; i < slice.getRanges().size(); i++) {
            float r = slice.getRanges().get(i);
            if (!inCloudRange(r, rangeMin, rangeMax)) {
                continue;
            }
            // Интенсивность — прямо из /scan (в бакетах мержа уже усреднена).
            float intensity = ScanMerger.pointIntensity(intensities, i);
            if (minIntensity > 0f && intensity < minIntensity) {
                continue; // низкая confidence — отбрасываем
            }

            // Луч i: θ = angle_min + i * angle_increment (рад, 0 = горизонт, вдоль +X)
            float theta = slice.getAngleMin() + i * slice.getAngleIncrement();
            float[] p = toWorldPoint(r, theta, angle, tiltRad);
            float[] point = new float[]{p[0], p[1], p[2], intensity};
            newPoints.add(point);
            slicePoints++;
        }

        synchronized (cloudLock) {
            cloudPoints.addAll(newPoints);
            PointCloud2 cloud = buildPointCloud(cloudPoints, slice);
            System.out.printf("[ROS] Slice at %.1f deg: %d points added, cloud total %d%n",
                    angle, slicePoints, cloudPoints.size());
            return cloud;
        }
    }

    /**
     * Собирает PointCloud2 (x, y, z, intensity — 4× FLOAT32, little-endian) из точек.
     * Вызывается под cloudLock.
     */
    private static PointCloud2 buildPointCloud(List<float[]> points, LaserScan reference) {
        return buildPointCloud(points, reference.getHeader().getFrameIdAsString(), reference.getHeader().getStamp());
    }

    /**
     * Собирает PointCloud2 (x, y, z, intensity — 4× FLOAT32, little-endian) из точек
     * в заданном frame с заданной меткой времени.
     */
    private static PointCloud2 buildPointCloud(List<float[]> points, String frameId, Time stamp) {
        PointCloud2 cloud = new PointCloud2();

        ByteBuffer data = ByteBuffer.allocate(points.size() * POINT_STEP).order(ByteOrder.LITTLE_ENDIAN);
        for (float[] p : points) {
            data.putFloat(p[0]).putFloat(p[1]).putFloat(p[2]).putFloat(p[3]);
        }

        cloud.getFields().clear();
        cloud.getFields().add(pointField("x", 0));
        cloud.getFields().add(pointField("y", 4));
        cloud.getFields().add(pointField("z", 8));
        cloud.getFields().add(pointField("intensity", 12));
        cloud.setHeight(1);
        cloud.setWidth(points.size());
        cloud.setIsBigendian(false);
        cloud.setPointStep(POINT_STEP);
        cloud.setRowStep(POINT_STEP * points.size());
        cloud.getData().clear();
        cloud.getData().addAll(data.array());
        cloud.setIsDense(true);

        cloud.getHeader().setFrameId(frameId);
        cloud.getHeader().getStamp().setSec(stamp.getSec());
        cloud.getHeader().getStamp().setNanosec(stamp.getNanosec());

        return cloud;
    }

    private static PointField pointField(String name, int offset) {
        PointField f = new PointField();
        f.setName(name);
        f.setOffset(offset);
        f.setDatatype(PointField.FLOAT32);
        f.setCount(1);
        return f;
    }
}
