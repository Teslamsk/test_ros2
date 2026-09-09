package org.example.vlp16;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Тесты трансформации монтажа VLP-16: матрица дефолтного монтажа,
 * поворот рамы вокруг вертикальной Z, добавление высоты.
 */
class Vlp16MountTest {

    private static void assertPoint(float[] expected, float[] actual, float eps) {
        assertEquals(expected.length, actual.length);
        for (int i = 0; i < expected.length; i++) {
            assertEquals(expected[i], actual[i], eps);
        }
    }

    @Test
    void defaultMountMapsLidarAxesToFrameAxes() {
        // forward (x_l) → вверх, left (y_l) → -Y рамы, ось вращения (z_l) → -X рамы
        assertPoint(new float[]{0f, 0f, 1f}, Vlp16Mount.toFrame(new float[]{1f, 0f, 0f}), 1e-6f);
        assertPoint(new float[]{0f, -1f, 0f}, Vlp16Mount.toFrame(new float[]{0f, 1f, 0f}), 1e-6f);
        assertPoint(new float[]{-1f, 0f, 0f}, Vlp16Mount.toFrame(new float[]{0f, 0f, 1f}), 1e-6f);
    }

    @Test
    void frameZeroKeepsPointInFrameCoordinates() {
        // 1 м вдоль оси вращения (z_l) → x_f = -1 → мировые (-1, 0, height)
        assertPoint(new float[]{-1f, 0f, Vlp16Mount.DEFAULT_HEIGHT},
                Vlp16Mount.toWorld(new float[]{0f, 0f, 1f}, 0f), 1e-5f);
    }

    @Test
    void rotatesAroundFrameZ() {
        // точка 1 м вдоль оси вращения: 90° → (0, -1, h), 180° → (1, 0, h)
        assertPoint(new float[]{0f, -1f, Vlp16Mount.DEFAULT_HEIGHT},
                Vlp16Mount.toWorld(new float[]{0f, 0f, 1f}, 90f), 1e-5f);
        assertPoint(new float[]{1f, 0f, Vlp16Mount.DEFAULT_HEIGHT},
                Vlp16Mount.toWorld(new float[]{0f, 0f, 1f}, 180f), 1e-5f);
    }

    @Test
    void leftAxisRotatesWithFrame() {
        // left (y_l) → (0, −1, 0) в раме; при повороте рамы на 90° уходит в +X
        assertPoint(new float[]{0f, -1f, Vlp16Mount.DEFAULT_HEIGHT},
                Vlp16Mount.toWorld(new float[]{0f, 1f, 0f}, 0f), 1e-5f);
        assertPoint(new float[]{1f, 0f, Vlp16Mount.DEFAULT_HEIGHT},
                Vlp16Mount.toWorld(new float[]{0f, 1f, 0f}, 90f), 1e-5f);
    }

    @Test
    void customMatrixAndHeight() {
        float[][] identity = {{1f, 0f, 0f}, {0f, 1f, 0f}, {0f, 0f, 1f}};
        assertPoint(new float[]{1f, 2f, 3.5f},
                Vlp16Mount.toWorld(new float[]{1f, 2f, 3f}, 0f, identity, 0.5f), 1e-6f);
    }
}
