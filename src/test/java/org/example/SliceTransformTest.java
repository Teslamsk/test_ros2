package org.example;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Геометрия среза: круг луча лидара вертикален, луч 0° горизонтален
 * (вдоль +X при нуле основания), основание вращается вокруг Z.
 */
class SliceTransformTest {

    private static final float EPS = 1e-6f;

    private static float[] pt(float r, float thetaDeg, float baseDeg) {
        return TopicInterface.toWorldPoint(r, (float) Math.toRadians(thetaDeg), baseDeg);
    }

    @Test
    void beamZeroIsHorizontalAlongX() {
        assertArrayEquals(new float[]{1f, 0f, 0f}, pt(1f, 0f, 0f), EPS);
    }

    @Test
    void beamNinetyIsVerticalUp() {
        assertArrayEquals(new float[]{0f, 0f, 1f}, pt(1f, 90f, 0f), EPS);
    }

    @Test
    void baseRotationTurnsHorizontalOnly() {
        // A=90°: горизонтальная составляющая уходит в +Y, вертикаль не меняется
        assertArrayEquals(new float[]{0f, 1f, 0f}, pt(1f, 0f, 90f), EPS);
        assertArrayEquals(new float[]{0f, 0f, 1f}, pt(1f, 90f, 90f), EPS);
    }

    @Test
    void oppositeSidesAndBelowHorizon() {
        assertArrayEquals(new float[]{-1f, 0f, 0f}, pt(1f, 180f, 0f), EPS);
        // луч ниже горизонта — точка под плоскостью основания (объём, а не пол-шара)
        assertArrayEquals(new float[]{0f, 0f, -1f}, pt(1f, 270f, 0f), EPS);
    }

    @Test
    void cloudRangeFiltersNearFieldAndNoReturn() {
        // 150 мм — граница: всё, что ближе (рама/основание), и нет возврата — отбрасываем
        assertFalse(TopicInterface.inCloudRange(0.149f, 0f, 12f), "ближе 150 мм — рама");
        assertTrue(TopicInterface.inCloudRange(0.150f, 0f, 12f), "ровно 150 мм — на границе");
        assertTrue(TopicInterface.inCloudRange(0.5f, 0f, 12f));
        assertFalse(TopicInterface.inCloudRange(0f, 0f, 12f), "нет возврата");
        assertFalse(TopicInterface.inCloudRange(Float.NaN, 0f, 12f), "NaN");
        assertFalse(TopicInterface.inCloudRange(13f, 0f, 12f), "вне доверенного диапазона сенсора");
    }

    @Test
    void radiusPreserved() {
        float[] p = pt(2.5f, 40f, 35f);
        double dist = Math.sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
        org.junit.jupiter.api.Assertions.assertEquals(2.5f, (float) dist, EPS);
    }

    @Test
    void tiltShiftsBeamElevation() {
        // tilt +10°: луч 0° смотрит вверх на 10°, радиус и азимут не меняются
        float t = (float) Math.toRadians(10);
        assertArrayEquals(new float[]{(float) Math.cos(t), 0f, (float) Math.sin(t)},
                TopicInterface.toWorldPoint(1f, 0f, 0f, t), EPS);
        // луч 80° + tilt 10° = ровно вертикально
        assertArrayEquals(new float[]{0f, 0f, 1f},
                TopicInterface.toWorldPoint(1f, (float) Math.toRadians(80), 0f, t), EPS);
        // tilt не влияет на поворот основания
        assertArrayEquals(new float[]{0f, (float) Math.cos(t), (float) Math.sin(t)},
                TopicInterface.toWorldPoint(1f, 0f, 90f, t), EPS);
    }
}
