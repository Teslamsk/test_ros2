package org.example;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

import java.util.concurrent.atomic.AtomicReference;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

@Timeout(10)
class EncoderTrackerTest {

    @Test
    void tracksConstantAngle() throws Exception {
        try (EncoderTracker enc = new EncoderTracker(() -> 12.0, 10)) {
            waitForSample(enc);
            double a = enc.angleAt(enc.wallNowNs());
            assertEquals(12.0, a, 1e-9);
        }
    }

    @Test
    void reflectsAngleChanges() throws Exception {
        AtomicReference<Double> v = new AtomicReference<>(0.0);
        try (EncoderTracker enc = new EncoderTracker(v::get, 10)) {
            waitForSample(enc);
            v.set(30.0);
            long deadline = System.currentTimeMillis() + 2000;
            double a = Double.NaN;
            while (System.currentTimeMillis() < deadline) {
                a = enc.angleAt(enc.wallNowNs());
                if (a >= 29.9) break;
                Thread.sleep(5);
            }
            assertTrue(a >= 29.9, "angle did not reach 30, got " + a);
        }
    }

    private static void waitForSample(EncoderTracker enc) throws InterruptedException {
        long deadline = System.currentTimeMillis() + 2000;
        while (enc.lastSampleNs() < 0 && System.currentTimeMillis() < deadline) {
            Thread.sleep(5);
        }
    }
}
