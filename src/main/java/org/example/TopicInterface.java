package org.example;

import builtin_interfaces.Time;
import sensor_msgs.LaserScan;
import sensor_msgs.PointCloud2;
import sensor_msgs.PointField;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Topic;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.atomic.AtomicReference;

public class TopicInterface {
    private final ROS2Node node;
    private final ROS2Publisher<LaserScan> pubProcessedScan;
    private final ROS2Publisher<PointCloud2> pubPointCloud;

    private final List<LaserScan> scanBuffer = new ArrayList<>();
    private CountDownLatch scanBufferLatch = new CountDownLatch(10);

    private AtomicReference<CountDownLatch> processingLatch = new AtomicReference<>(new CountDownLatch(1));
    private volatile PointCloud2 lastCloud;
    private volatile float currentAngle = 0;

    private static final int POINT_STEP = 12; // x + y + z, каждый float32 (4 байта)

    /**
     * Накопленные точки облака в мировой системе координат: каждая — {x, y, z}.
     * Заполняется по мере прихода слайсов с /processedScan.
     */
    private final List<float[]> cloudPoints = new ArrayList<>();

    public TopicInterface(String ns) {
        this.node = new ROS2Node(ns);
        this.pubProcessedScan = node.createPublisher(new ROS2Topic<LaserScan>("/processedScan", LaserScan.class));
        this.pubPointCloud = node.createPublisher(new ROS2Topic<PointCloud2>("/pointCloud", PointCloud2.class));

        startListeningToScan();
        startListeningToProcessedScan();
    }

    private void startListeningToScan() {
        node.createSubscription(
                new ROS2Topic<LaserScan>("/scan", LaserScan.class),
                reader -> {
                    LaserScan scan = reader.read();
                    synchronized (scanBuffer) {
                        scanBuffer.add(scan);
                    }
                    scanBufferLatch.countDown();
                }
        );
        System.out.println("[ROS] Subscribed to /scan");
    }

    private void startListeningToProcessedScan() {
        node.createSubscription(
                new ROS2Topic<LaserScan>("/processedScan", LaserScan.class),
                reader -> {
                    LaserScan processedScan = reader.read();
                    lastCloud = addSliceToCloud(lastCloud, processedScan, currentAngle);
                    pubPointCloud.publish(lastCloud);
                    processingLatch.get().countDown();
                }
        );
        System.out.println("[ROS] Subscribed to /processedScan (async handler started)");
    }

    public List<LaserScan> collectScans(int count) throws InterruptedException {
        synchronized (scanBuffer) {
            scanBuffer.clear();
        }
        scanBufferLatch = new CountDownLatch(count);
        System.out.println("[ROS] Waiting for " + count + " scans from /scan...");
        scanBufferLatch.await();

        List<LaserScan> readyScans;
        synchronized (scanBuffer) {
            readyScans = new ArrayList<>(scanBuffer);
        }
        return readyScans;
    }

    /**
     * Снапшот накопленных точек облака (копия, можно свободно копировать/экспортировать).
     */
    public List<float[]> getCloudPoints() {
        synchronized (cloudPoints) {
            return new ArrayList<>(cloudPoints);
        }
    }

    public LatchWrapper publishAndAwaitProcessed(LaserScan avgScan, float angle) {
        this.currentAngle = angle;
        CountDownLatch latch = new CountDownLatch(1);
        processingLatch.set(latch);
        pubProcessedScan.publish(avgScan);
        System.out.println("[ROS] Published to /processedScan (angle " + angle + "), waiting for async handler...");
        return new LatchWrapper(latch);
    }

    public void close() {
        node.close();
    }

    private PointCloud2 addSliceToCloud(PointCloud2 cloud, LaserScan slice, float angle) {
        if (cloud == null) {
            cloud = new PointCloud2();
        }

        // Поворот слайса на угол головки (окружность вокруг оси Z):
        // "локальная" система лида́ра -> мировая.
        float cosA = (float) Math.cos(Math.toRadians(angle));
        float sinA = (float) Math.sin(Math.toRadians(angle));

        float rangeMin = slice.getRangeMin();
        float rangeMax = slice.getRangeMax();
        int slicePoints = 0;
        for (int i = 0; i < slice.getRanges().size(); i++) {
            float r = slice.getRanges().get(i);
            if (!Float.isFinite(r) || r <= 0f) {
                continue; // нет возврата
            }
            if ((rangeMin > 0f && r < rangeMin) || (rangeMax > 0f && r > rangeMax)) {
                continue; // вне доверенного диапазона сенсора
            }

            // Луч i: theta = angle_min + i * angle_increment (рад, против часовой, 0 = +x)
            float theta = slice.getAngleMin() + i * slice.getAngleIncrement();
            float x = r * (float) Math.cos(theta);
            float y = r * (float) Math.sin(theta);

            cloudPoints.add(new float[]{x * cosA - y * sinA, x * sinA + y * cosA, 0f});
            slicePoints++;
        }

        ByteBuffer data = ByteBuffer.allocate(cloudPoints.size() * POINT_STEP).order(ByteOrder.LITTLE_ENDIAN);
        for (float[] p : cloudPoints) {
            data.putFloat(p[0]).putFloat(p[1]).putFloat(p[2]);
        }

        cloud.getFields().clear();
        cloud.getFields().add(pointField("x", 0));
        cloud.getFields().add(pointField("y", 4));
        cloud.getFields().add(pointField("z", 8));
        cloud.setHeight(1);
        cloud.setWidth(cloudPoints.size());
        cloud.setIsBigendian(false);
        cloud.setPointStep(POINT_STEP);
        cloud.setRowStep(POINT_STEP * cloudPoints.size());
        cloud.getData().clear();
        cloud.getData().addAll(data.array());
        cloud.setIsDense(true);

        cloud.getHeader().setFrameId("world");
        Time stamp = slice.getHeader().getStamp();
        cloud.getHeader().getStamp().setSec(stamp.getSec());
        cloud.getHeader().getStamp().setNanosec(stamp.getNanosec());

        System.out.printf("[ROS] Slice at %.1f deg: %d points added, cloud total %d%n",
                angle, slicePoints, cloudPoints.size());
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

    public static class LatchWrapper {
        private final CountDownLatch latch;

        public LatchWrapper(CountDownLatch latch) {
            this.latch = latch;
        }

        public void await() throws InterruptedException {
            latch.await();
        }
    }
}