package org.example;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class SweepAccumulatorTest {

    private static SweepPoint pt(double frameAngle, int bin, float range, float intensity, float theta) {
        return new SweepPoint(frameAngle, range, intensity, theta, bin);
    }

    @Test
    void mergesFreshPointsPerBin() {
        SweepAccumulator acc = new SweepAccumulator(0.2);
        List<SweepPoint> out = acc.addAndWindow(
                List.of(pt(0.0, 0, 1.0f, 0.5f, 0.0f),
                        pt(0.05f, 0, 1.02f, 0.6f, 0.0f),
                        pt(0.0f, 1, 2.0f, 0.7f, 0.1f)),
                0.05, +1);
        // Bin 0 merged two points; bin 1 alone
        assertEquals(2, out.size());
        SweepPoint m0 = out.stream().filter(p -> p.bin() == 0).findFirst().orElseThrow();
        assertEquals((0.0 + 0.05) / 2, m0.frameAngleDeg(), 1e-9);
        assertEquals(1.01f, m0.range(), 1e-4f);
        assertEquals(0.55f, m0.intensity(), 1e-4f);
        SweepPoint m1 = out.stream().filter(p -> p.bin() == 1).findFirst().orElseThrow();
        assertEquals(2.0f, m1.range(), 1e-4f);
    }

    @Test
    void dropsPointsOutsideWindow() {
        SweepAccumulator acc = new SweepAccumulator(0.2);
        acc.addAndWindow(List.of(pt(0.0, 0, 1.0f, 0.5f, 0.0f)), 0.0, +1);
        // Advance frame to 0.3 (> window 0.2) -> old point pruned
        List<SweepPoint> out = acc.addAndWindow(List.of(), 0.3, +1);
        assertTrue(out.isEmpty(), "old point should be dropped");
        assertEquals(0, acc.totalPoints());
    }

    @Test
    void windowFollowsNegativeProgressDirection() {
        // dirSign = -1: angle decreases (CW), progress = -angle increases
        SweepAccumulator acc = new SweepAccumulator(0.2);
        acc.addAndWindow(List.of(pt(0.0, 0, 1.0f, 0.5f, 0.0f)), 0.0, -1);
        // Advance in negative direction: frame = -0.05, progress = 0.05
        List<SweepPoint> out = acc.addAndWindow(
                List.of(pt(-0.05, 0, 1.0f, 0.5f, 0.0f)), -0.05, -1);
        // Both points within window [0.05 - 0.2, 0.05] = [-0.15, 0.05]
        assertEquals(1, out.size());
        SweepPoint m = out.get(0);
        assertEquals(0, m.bin());
        assertEquals(-0.025, m.frameAngleDeg(), 1e-9);
        assertEquals(1.0f, m.range(), 1e-4f);
    }
}
