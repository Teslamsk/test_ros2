package org.example;

import sensor_msgs.LaserScan;
import sensor_msgs.PointCloud2;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Topic;

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
            // Создаем заглушку для дебага
            cloud = new PointCloud2();
        }
        // TODO: Реальная математика: конвертация LaserScan в PointCloud2, поворот на angle, добавление к cloud
        System.out.println("[ROS] Added rotated slice at " + angle + " to /pointCloud");
        return cloud;
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