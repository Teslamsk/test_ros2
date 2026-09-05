package org.example;

import org.junit.jupiter.api.Test;
import sensor_msgs.LaserScan;

import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class TopicSweepTest {

    private static LaserScan scan(long stampNs, float... ranges) {
        LaserScan s = new LaserScan();
        s.getHeader().getStamp().setSec((int) (stampNs / 1_000_000_000L));
        s.getHeader().getStamp().setNanosec((int) (stampNs % 1_000_000_000L));
        s.setAngleMin(0f);
        s.setAngleIncrement(0f);
        s.setAngleMax(0f);
        s.setRangeMin(0f);
        s.setRangeMax(12f);
        for (float r : ranges) {
            s.getRanges().add(r);
        }
        return s;
    }

    private static float[] beams(int total, int valid, float value) {
        float[] r = new float[total];
        Arrays.fill(r, 0, valid, value);
        Arrays.fill(r, valid, total, Float.NaN);
        return r;
    }

    @Test
    void keepsValidBeamsAndRestoresFramePerSlice() {
        LaserScan s = scan(1_000_000_000L, beams(500, 433, 1.0f));
        FrameAngleSampler angleAt = ns -> 0.0201;
        List<SweepPoint> pts = TopicInterface.sweepSlicePoints(s, angleAt, +1, 0.02, 12.0, 0f);
        assertEquals(433, pts.size());
        for (SweepPoint p : pts) {
            assertEquals(0.0201, p.frameAngleDeg(), 1e-9);
        }
    }

    @Test
    void dropsSliceWhenProgressBelowTension() {
        LaserScan s = scan(1_000_000_000L, beams(500, 500, 1.0f));
        FrameAngleSampler angleAt = ns -> 0.01; // прогресс 0.01 < tension 0.02
        List<SweepPoint> pts = TopicInterface.sweepSlicePoints(s, angleAt, +1, 0.02, 12.0, 0f);
        assertTrue(pts.isEmpty());
    }

    @Test
    void maxRangeCapsSensorRange() {
        LaserScan s = scan(1_000_000_000L, 2.0f, 5.0f); // maxRangeM=3: 2.0 в пределах, 5.0 нет
        FrameAngleSampler angleAt = ns -> 10.0;
        List<SweepPoint> pts = TopicInterface.sweepSlicePoints(s, angleAt, +1, 0.0, 3.0, 0f);
        assertEquals(1, pts.size());
        assertEquals(2.0f, pts.get(0).range(), 1e-6f);
    }

    @Test
    void dropsPointsWithoutEncoderData() {
        LaserScan s = scan(1_000_000_000L, beams(500, 500, 1.0f));
        FrameAngleSampler angleAt = ns -> Double.NaN;
        List<SweepPoint> pts = TopicInterface.sweepSlicePoints(s, angleAt, +1, 0.0, 12.0, 0f);
        assertTrue(pts.isEmpty());
    }

    @Test
    void minConfidenceFiltersLowIntensityPoints() {
        LaserScan s = new LaserScan();
        s.getHeader().getStamp().setSec(1);
        s.setAngleMin(0f);
        s.setAngleIncrement(0f);
        s.setAngleMax(0f);
        s.setRangeMin(0f);
        s.setRangeMax(12f);
        s.getRanges().add(1.0f);
        s.getIntensities().add(200f); // strong — keep
        s.getRanges().add(1.0f);
        s.getIntensities().add(50f); // weak — drop
        s.getRanges().add(1.0f);
        s.getIntensities().add(0f); // none — drop
        FrameAngleSampler angleAt = ns -> 5.0;
        List<SweepPoint> pts = TopicInterface.sweepSlicePoints(s, angleAt, +1, 0.0, 12.0, 100f);
        assertEquals(1, pts.size());
        assertEquals(200f, pts.get(0).intensity(), 1e-6f);
    }
}
