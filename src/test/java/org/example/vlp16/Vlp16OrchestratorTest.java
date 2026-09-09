package org.example.vlp16;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Тесты фильтрации/трансформации шага (без ROS и CAN):
 * no-return, NaN, дальность, интенсивность + трансформ в мировые координаты.
 */
class Vlp16OrchestratorTest {

    private static void assertPoint(float[] expected, float[] actual, float eps) {
        assertEquals(expected.length, actual.length);
        for (int i = 0; i < expected.length; i++) {
            assertEquals(expected[i], actual[i], eps);
        }
    }

    @Test
    void dropsNoReturnNaNAndOutOfRange() {
        List<float[]> raw = List.of(
                new float[]{0f, 0f, 0f, 200f},           // нет возврата
                new float[]{1f, 0f, 0f, Float.NaN},      // NaN
                new float[]{10f, 0f, 0f, 200f},          // 10 м — в пределах 20
                new float[]{25f, 0f, 0f, 200f});         // 25 м > 20
        List<float[]> out = Vlp16Orchestrator.transformAndFilter(raw, 0.0, 0.4f, 20.0, 0);
        assertEquals(1, out.size());
        // forward лидара (x_l) = вверх: (0, 0, 10 + 0.4)
        assertPoint(new float[]{0f, 0f, 10.4f, 200f}, out.get(0), 1e-4f);
    }

    @Test
    void rotatesByFrameAngle() {
        // 1 м вдоль оси вращения (z_l) → x_f = -1; рама на 90° → (0, -1, h)
        List<float[]> out = Vlp16Orchestrator.transformAndFilter(
                List.of(new float[]{0f, 0f, 1f, 100f}), 90.0, 0.5f, 30.0, 0);
        assertEquals(1, out.size());
        assertPoint(new float[]{0f, -1f, 0.5f, 100f}, out.get(0), 1e-4f);
    }

    @Test
    void softwareFrameAngleInvertsStitchDirection() {
        assertEquals(-90.0, Vlp16Orchestrator.softwareFrameAngle(90.0), 1e-9);
        assertEquals(45.0, Vlp16Orchestrator.softwareFrameAngle(-45.0), 1e-9);
    }

    @Test
    void filtersByIntensity() {
        List<float[]> raw = List.of(
                new float[]{1f, 0f, 0f, 50f},
                new float[]{1f, 0f, 0f, 200f});
        List<float[]> out = Vlp16Orchestrator.transformAndFilter(raw, 0.0, 0.0f, 30.0, 150);
        assertEquals(1, out.size());
        assertEquals(200f, out.get(0)[3], 1e-6f);
    }

    @Test
    void intensityFilterOffKeepsAll() {
        List<float[]> raw = List.of(
                new float[]{1f, 0f, 0f, 0f},
                new float[]{2f, 0f, 0f, 1f});
        List<float[]> out = Vlp16Orchestrator.transformAndFilter(raw, 0.0, 0.0f, 30.0, 0);
        assertEquals(2, out.size());
    }

    @Test
    void colorizeAppliesDistanceCorrectionAndHeatMap() {
        // r: ближний 1 м, дальний 2 м; одинаковая intensity=100.
        // colorExp=2: V = 100*1² = 100 и 100*2² = 400 → t = 0.25 и 1.0.
        // heat: 0.25 → циан (0,255,255), 1.0 → красный (255,0,0).
        List<float[]> world = List.of(
                new float[]{1f, 0f, 0.4f, 100f},
                new float[]{2f, 0f, 0.4f, 100f});
        List<float[]> out = Vlp16Orchestrator.colorize(world, 0.4f, 2, "heat");
        assertEquals(2, out.size());
        assertPoint(new float[]{1f, 0f, 0.4f, 100f, 0f, 255f, 255f}, out.get(0), 1e-4f);
        assertPoint(new float[]{2f, 0f, 0.4f, 100f, 255f, 0f, 0f}, out.get(1), 1e-4f);
    }

    @Test
    void colorizeNoCorrectionUsesRawIntensity() {
        // colorExp=0: V = intensity * r^0 = intensity (решает только отражающая способность).
        // V = 50 и 200 → t = 0.25 и 1.0 → циан / красный.
        List<float[]> world = List.of(
                new float[]{1f, 0f, 0.4f, 50f},
                new float[]{2f, 0f, 0.4f, 200f});
        List<float[]> out = Vlp16Orchestrator.colorize(world, 0.4f, 0, "heat");
        assertPoint(new float[]{1f, 0f, 0.4f, 50f, 0f, 255f, 255f}, out.get(0), 1e-4f);
        assertPoint(new float[]{2f, 0f, 0.4f, 200f, 255f, 0f, 0f}, out.get(1), 1e-4f);
    }

    @Test
    void colorizeGrayModeIsMonochrome() {
        // Одна точка → t=1.0 → белый (255,255,255).
        List<float[]> out = Vlp16Orchestrator.colorize(
                List.of(new float[]{1f, 0f, 0.4f, 100f}), 0.4f, 0, "gray");
        assertPoint(new float[]{1f, 0f, 0.4f, 100f, 255f, 255f, 255f}, out.get(0), 1e-4f);
    }

    @Test
    void colorizeOffModeIsFlatGray() {
        List<float[]> out = Vlp16Orchestrator.colorize(
                List.of(new float[]{1f, 0f, 0.4f, 100f}), 0.4f, 0, "off");
        assertPoint(new float[]{1f, 0f, 0.4f, 100f, 128f, 128f, 128f}, out.get(0), 1e-4f);
    }

    @Test
    void colorizeEmptyReturnsEmpty() {
        assertEquals(0, Vlp16Orchestrator.colorize(List.of(), 0.4f, 2, "heat").size());
    }
}
