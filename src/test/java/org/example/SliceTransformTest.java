package org.example;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

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
    void radiusPreserved() {
        float[] p = pt(2.5f, 40f, 35f);
        double dist = Math.sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
        org.junit.jupiter.api.Assertions.assertEquals(2.5f, (float) dist, EPS);
    }
}
