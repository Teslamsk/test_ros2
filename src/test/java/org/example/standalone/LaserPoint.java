package org.example.standalone;

/**
 * Точка скана: угол в градусах (лидарная система, 0 = +X, CW)
 * и дистанция в мм (0 = нет возврата).
 */
public record LaserPoint(double angleDeg, int distanceMm) {
}
