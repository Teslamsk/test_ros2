package org.example;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class AngleBufferTest {

    @Test
    void interpolatesLinearlyBetweenSamples() {
        AngleBuffer buf = new AngleBuffer(16, 100_000_000L); // maxGap 100ms
        buf.add(0L, 0.0);
        buf.add(10_000_000L, 10.0); // 10ms later, 10 deg
        assertEquals(5.0, buf.angleAt(5_000_000L), 1e-9); // midpoint
        assertEquals(0.0, buf.angleAt(0L), 1e-9);
        assertEquals(10.0, buf.angleAt(10_000_000L), 1e-9);
    }

    @Test
    void returnsNaNOutsideBuffer() {
        AngleBuffer buf = new AngleBuffer(16, 10_000_000L); // maxGap 10ms
        assertTrue(Double.isNaN(buf.angleAt(5_000_000L))); // empty
        buf.add(100_000_000L, 50.0); // one sample at t=100ms
        assertTrue(Double.isNaN(buf.angleAt(0L))); // 100ms before first sample, > 10ms maxGap
        assertTrue(Double.isNaN(buf.angleAt(200_000_000L))); // 100ms after last sample, > 10ms maxGap
    }

    @Test
    void clampsNearBufferEdges() {
        AngleBuffer buf = new AngleBuffer(16, 100_000_000L); // maxGap 100ms
        buf.add(100_000_000L, 50.0);
        // 10ms before first sample (within 100ms maxGap) -> clamp to first
        assertEquals(50.0, buf.angleAt(90_000_000L), 1e-9);
        // 10ms after last sample (within 100ms maxGap) -> clamp to last
        assertEquals(50.0, buf.angleAt(110_000_000L), 1e-9);
    }

    @Test
    void returnsNaNAcrossGap() {
        AngleBuffer buf = new AngleBuffer(16, 100_000_000L); // maxGap 100ms
        buf.add(0L, 0.0);
        buf.add(200_000_000L, 20.0); // 200ms gap > 100ms maxGap
        assertTrue(Double.isNaN(buf.angleAt(100_000_000L))); // middle of the gap
    }

    @Test
    void ringWrapKeepsNewestSamples() {
        AngleBuffer buf = new AngleBuffer(4, 100_000_000L); // capacity 4, maxGap 100ms
        for (int k = 0; k < 6; k++) {
            buf.add(k * 10_000_000L, k * 10.0); // t=0..50ms, angles 0..50
        }
        // Buffer holds newest 4: t=20,30,40,50ms (angles 20,30,40,50); head wrapped
        assertEquals(50.0, buf.angleAt(50_000_000L), 1e-9); // newest
        assertEquals(25.0, buf.angleAt(25_000_000L), 1e-9); // between 20 and 30
        assertEquals(45.0, buf.angleAt(45_000_000L), 1e-9); // between 40 and 50
        // t=10ms evicted; 10ms before first kept (t=20) within maxGap -> clamp to 20
        assertEquals(20.0, buf.angleAt(10_000_000L), 1e-9);
        // Far before (> maxGap) -> NaN
        assertTrue(Double.isNaN(buf.angleAt(-90_000_000L)));
    }
}
