package aold;

import std_msgs.String_;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.jros2.ROS2Topic;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

public class PubSubDemo {

    private static final String TOPIC_NAME = "/demo_topic";

    public static void main(String[] args) throws InterruptedException {
        System.out.println("=== ROS 2 Pub/Sub Demo (jros2 1.0.1) ===");

        CountDownLatch latch = new CountDownLatch(5);

        // --- ПОДПИСЧИК ---
        ROS2Node subscriptionNode = new ROS2Node("subscription_node");
        ROS2Topic<String_> topic = new ROS2Topic<>(TOPIC_NAME, String_.class);

        ROS2Subscription<String_> subscription = subscriptionNode.createSubscription(
                topic, reader -> {
                    String_ msg = reader.read();
                    System.out.println("  [SUB] Received: " + msg.getData());
                    latch.countDown();
                }
        );
        System.out.println("Subscribed to: " + TOPIC_NAME);

        // --- ИЗДАТЕЛЬ ---
        ROS2Node publisherNode = new ROS2Node("publisher_node");
        ROS2Publisher<String_> publisher = publisherNode.createPublisher(topic);
        System.out.println("Publisher created for: " + TOPIC_NAME);

        Thread.sleep(500);

        // --- ПУБЛИКАЦИЯ ---
        System.out.println("\n--- Publishing 5 messages ---");
        for (int i = 0; i < 5; i++) {
            String_ msg = new String_();
            msg.getData().append("Hello from Java! #").append(i);
            publisher.publish(msg);
            System.out.println("  [PUB] Sent: " + msg.getData());
            Thread.sleep(200);
        }

        // --- ОЖИДАНИЕ ---
        System.out.println("\n--- Waiting for subscriber (max 5 sec) ---");
        boolean allReceived = latch.await(5, TimeUnit.SECONDS);

        if (allReceived) {
            System.out.println("All 5 messages received!");
        } else {
            System.out.println("Received " + (5 - latch.getCount()) + "/5 messages.");
        }

        // --- ЗАВЕРШЕНИЕ ---
        subscriptionNode.destroySubscription(subscription);
        publisherNode.destroyPublisher(publisher);
        subscriptionNode.close();
        publisherNode.close();

        System.out.println("=== Demo finished ===");
    }
}