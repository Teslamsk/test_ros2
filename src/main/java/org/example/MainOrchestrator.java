package org.example;

import org.example.can.CanController;
import org.example.can.dictionary.Node;
import sensor_msgs.LaserScan;

import java.util.List;

public class MainOrchestrator {
    public static void main(String[] args) throws InterruptedException {
        System.out.println("=== Main Orchestrator Started ===");
        TopicInterface ros = new TopicInterface("orchestrator_node");
        CanController can = new CanController();
        can.init("/dev/can0");

        float angle = 0;
        int scanSteps = 36;
        int scansPerStep = 10;



        for (int i = 0; i < scanSteps; i++) {
            List<LaserScan> rawScans = ros.collectScans(scansPerStep);

            System.out.println("[MAIN] Averaging " + rawScans.size() + " scans for angle " + angle);
            LaserScan averaged = averageScans(rawScans);

            TopicInterface.LatchWrapper sync = ros.publishAndAwaitProcessed(averaged, angle);
            sync.await();
            System.out.println("[MAIN] Cloud updated, moving motor...");

            angle += 10.0f;
        }

        System.out.println("=== 360 Scan Complete ===");
        ros.close();
    }

    private static LaserScan averageScans(List<LaserScan> scans) {
        System.out.println("[MAIN] TODO: implement averaging for LaserScan");
        return scans.get(scans.size() / 2);
    }
}
