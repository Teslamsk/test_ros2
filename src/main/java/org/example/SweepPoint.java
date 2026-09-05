package org.example;

/**
 * Одна точка среза свипа в системе координат лидара + угол рамы в момент съёмки.
 *
 * <ul>
 *  <li>{@code frameAngleDeg} — угол рамы (основания), град, восстановлен по
 *      энкодеру на метку времени скана;</li>
 *  <li>{@code range} — дальность, м;</li>
 *  <li>{@code intensity} — интенсивность;</li>
 *  <li>{@code thetaRad} — азимут луча в плоскости лидара (0 = +X, CCW), рад;</li>
 *  <li>{@code bin} — индекс луча (колонка) — ключ накопления в {@link SweepAccumulator}.</li>
 * </ul>
 */
public record SweepPoint(double frameAngleDeg, float range, float intensity, float thetaRad, int bin) {
}
