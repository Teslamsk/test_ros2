package org.example.can.transport;

import java.util.ArrayDeque;
import java.util.Deque;

/**
 * In-memory mock of CanBus for unit tests.
 *
 * Usage:
 * <pre>
 * MockCanBus mockBus = new MockCanBus();
 * mockBus.queue(0x581, new byte[]{0x43, 0, 0, 0, 0x1A, 0, 0, 0});
 * CanController ctrl = new CanController(mockBus);
 * // calls to sdoRead will drain the queue.
 * </pre>
 */
public class MockCanBus implements CanBus {

    /** Queue of canned responses: (cobId, data). */
    private final Deque<CanFrame> rxQueue = new ArrayDeque<>();

    /** All frames sent via send() — for assertions. */
    private final Deque<CanFrame> txLog = new ArrayDeque<>();

    public static class CanFrame {
        public final int cobId;
        public final byte[] data;

        public CanFrame(int cobId, byte[] data) {
            this.cobId = cobId;
            this.data = data;
        }

        @Override
        public String toString() {
            StringBuilder sb = new StringBuilder(String.format("0x%03X# ", cobId));
            for (byte b : data) sb.append(String.format("%02X", b));
            return sb.toString();
        }
    }

    /**
     * Pre-queues a response. Drained by receive() in FIFO order, but only
     * if the requested Cob-ID matches.
     */
    public void queue(int cobId, byte[] data) {
        rxQueue.offer(new CanFrame(cobId, data));
    }

    /** Returns the number of frames waiting in the mock RX queue. */
    public int rxQueueSize() {
        return rxQueue.size();
    }

    /** Returns all frames that were sent via send(). */
    public java.util.List<CanFrame> getTxLog() {
        return new java.util.ArrayList<>(txLog);
    }

    /** Clears both RX queue and TX log. */
    public void reset() {
        rxQueue.clear();
        txLog.clear();
    }

    @Override
    public void open(String iface) {
        System.out.println("[MOCK-CanBus] open(" + iface + ")");
    }

    @Override
    public void send(int cobId, byte[] data) {
        txLog.offer(new CanFrame(cobId, data.clone()));
    }

    @Override
    public byte[] receive(int expectedCobId, int timeoutMs) {
        for (CanFrame frame : rxQueue) {
            if (frame.cobId == expectedCobId) {
                rxQueue.remove();
                return frame.data.clone();
            }
        }
        return null;  // timeout — no matching frame in queue
    }

    @Override
    public void close() {
        System.out.println("[MOCK-CanBus] closed");
    }
}
