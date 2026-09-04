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

    /** Механический наклон луча 0° лидара относительно горизонтали (рад): + — луч 0° смотрит вверх. */
    private final float tiltRad;

    public TopicInterface(String ns) {
        this(ns, 0f);
    }

    public TopicInterface(String ns, float tiltDeg) {
        this.tiltRad = (float) Math.toRadians(tiltDeg);
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
        System.out.println("[ROS] Waiting for " + count + " scans from /scan...");
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

            // Луч i: θ = angle_min + i * angle_increment (рад, 0 = горизонт, вдоль +X)
            float theta = slice.getAngleMin() + i * slice.getAngleIncrement();
            float[] p = toWorldPoint(r, theta, angle, tiltRad);
            // Интенсивность — прямо из /scan (в бакетах мержа уже усреднена).
            float[] point = new float[]{p[0], p[1], p[2], ScanMerger.pointIntensity(intensities, i)};
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

        // Frame облака — как у приходящих /scan (у драйвера base_laser): в RViz не
        // нужен TF, дисплей PointCloud2 в Fixed frame = это же frame видит данные.
        cloud.getHeader().setFrameId(reference.getHeader().getFrameIdAsString());
        Time stamp = reference.getHeader().getStamp();
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
