package org.example;

/**
 * Источник угла рамы по wall-clock (ns): интерполяция из буфера энкодера.
 * {@link EncoderTracker#angleAt(long)} подходит напрямую.
 */
@FunctionalInterface
public interface FrameAngleSampler {
    double angleAt(long wallNs);
}
