package org.example;

import org.example.can.Mks42dController;
import org.example.can.transport.CanBus;
import org.example.can.transport.CanBusFactory;
import org.example.export.XyzWriter;
import sensor_msgs.LaserScan;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.List;

/**
 * Оркестратор: поворот основания (MKS 42D по CAN) + сборка облака (ROS2).
 *
 * <p>Два режима:
 * <ul>
 *  <li><b>step</b> (по умолч.) — пошаговый: остановка на позициях, мерж N сканов,
 *      публикация среза. Быстро, но между позициями рама стоит — облако "лесенкой".</li>
 *  <li><b>sweep</b> — непрерывный: медленный поворот основания с постоянной
 *      скоростью, угол рамы каждой точки восстанавливается по энкодеру, точки
 *      накапливаются в облако и стримятся в XYZ "на лету". Плавный, без лесенки.</li>
 * </ul>
 */
public class MainOrchestrator {

    public enum Mode { STEP, SWEEP }

    private static final int BUCKET_FACTOR = 6;
    private static final float OUTLIER_TOLERANCE = 0.10f;

    /** Передаточное число мотор:рама = 4:1 (4 оборота мотора = 1 оборот рамы). */
    private static final double GEAR_RATIO = 4.0;

    // ---- sweep-режим ----
    /** Шаг опроса энкодера трекером, мс. */
    private static final int ENCODER_POLL_MS = 30;
    /** Ускорение/замедление движения в свипе. */
    private static final int SWEEP_ACCEL = 25;
    /** Ширина окна накопления, град поворота рамы. */
    private static final double SWEEP_WINDOW_DEG = 0.2;
    /** Энкодер считается "живым", если сэмпл свежее этого, ns. */
    private static final long ENCODER_STALL_NS = 5L * ENCODER_POLL_MS * 1_000_000L;
    /** Дать раме тронуться после команды движения, мс. */
    private static final long WAIT_MOVING_TIMEOUT_MS = 5000;
    /** На сколько градусов рама должна сдвинуться, чтобы считать её "идущей". */
    private static final double WAIT_MOVING_EPS_DEG = 0.5;

    /** Инкрементальный XYZ-поток свипа (static: шатдаун-хук может его дописать). */
    private static volatile XyzWriter.Stream xyzOut;

    public static void main(String[] args) {
        System.out.println("=== Main Orchestrator Started ===");
        // Первый позиционный аргумент — имя SocketCAN-интерфейса ("can0"), НЕ путь "/dev/can0";
        // на Windows PcanBus параметр не использует.
        String canIface = "can0";
        Mode mode = Mode.STEP;
        double sweepDeg = 90.0;  // ход ОСНОВАНИЯ в градусах (общий для обоих режимов)
        float stepDeg = 10.0f;   // шаг основания (step-режим)
        int scansPerStep = 10;   // сканов /scan на позицию (step-режим)
        int currentMa = 0;       // рабочий ток, мА (0 = не трогать)
        int holdMa = 0;          // удерживающий ток, мА (0 = не трогать)
        double tiltDeg = 0.0;    // наклон луча 0° лидара от горизонтали, град
        int maxRpm = 0;          // ограничитель скорости мотора, RPM (0 = профиль)
        int maxAcc = 0;          // ограничитель ускорения (0 = профиль)
        // sweep-режим
        double tensionDeg = 2.0;     // зона натяжения у старта, град
        boolean clockwise = true;    // cw (по умолч.) / ccw
        int sweepRpm = 1;            // скорость мотора, RPM (рама = rpm / GEAR_RATIO)
        int merge = 1;               // сканов /scan на срез, 1..32
        double maxRangeM = 3.0;      // ограничение дальности, м
        int minConfidence = 0;       // порог интенсивности 0..255 (0 = фильтр выключен)
        boolean calibrate = false;   // только диагностика движения (без облака/XYZ)

        try {
            for (int i = 0; i < args.length; i++) {
                switch (args[i]) {
                    case "--mode":       mode = parseMode(args[++i]); break;
                    case "--sweep":      sweepDeg = Double.parseDouble(args[++i]); break;
                    case "--step":       stepDeg = Float.parseFloat(args[++i]); break;
                    case "--scans":      scansPerStep = Integer.parseInt(args[++i]); break;
                    case "--current":    currentMa = Integer.parseInt(args[++i]); break;
                    case "--hold":       holdMa = Integer.parseInt(args[++i]); break;
                    case "--tilt":       tiltDeg = Double.parseDouble(args[++i]); break;
                    case "--rpm":        maxRpm = Integer.parseInt(args[++i]); break;
                    case "--acc":        maxAcc = Integer.parseInt(args[++i]); break;
                    case "--tension":    tensionDeg = Double.parseDouble(args[++i]); break;
                    case "--direction":  clockwise = parseDirection(args[++i]); break;
                    case "--sweep-rpm":  sweepRpm = Integer.parseInt(args[++i]); break;
                    case "--merge":      merge = Integer.parseInt(args[++i]); break;
                    case "--max-range":  maxRangeM = Double.parseDouble(args[++i]); break;
                    case "--min-confidence": minConfidence = Integer.parseInt(args[++i]); break;
                    case "--calibrate":  calibrate = true; break;
                    case "-h", "--help":
                        printUsage();
                        System.exit(0);
                    default:
                        if (args[i].startsWith("-")) {
                            fail("Неизвестный параметр: " + args[i]);
                        }
                        canIface = args[i];
                }
            }
        } catch (NumberFormatException e) {
            fail("Некорректное числовое значение параметра: " + e.getMessage());
        }
        validate(mode, sweepDeg, stepDeg, scansPerStep, currentMa, holdMa, tiltDeg, maxRpm, maxAcc,
                tensionDeg, sweepRpm, merge, maxRangeM, minConfidence, calibrate);

        TopicInterface ros = new TopicInterface("orchestrator_node", (float) tiltDeg, (float) minConfidence);
        CanBus bus = CanBusFactory.forCurrentOS(); // транспорт под текущую ОС
        Mks42dController can = new Mks42dController(bus);
        try {
            can.init(canIface); // шина CAN 2.0A + привод MKS 42D (node 01, 500 kbit/s)
            // Ctrl-C / завершение JVM: дописать XYZ + аварийный стоп привода + закрыть шину
            Runtime.getRuntime().addShutdownHook(new Thread(() -> {
                System.err.println("[MAIN] Ctrl-C: аварийный стоп...");
                flushXyz();
                can.close();
            }, "main-shutdown"));
            if (maxRpm > 0 || maxAcc > 0) {
                can.tuneMotion(maxRpm, maxAcc);
            }
            if (currentMa > 0) {
                can.setWorkingCurrent(currentMa); // сила FOC-контра: момент против нагрузки
            }
            if (holdMa > 0) {
                can.setHoldingCurrent(holdMa); // жёсткое удержание тяжёлой рамы
            }

            if (mode == Mode.STEP) {
                runStep(ros, can, sweepDeg, stepDeg, scansPerStep);
            } else if (calibrate) {
                runCalibrate(can, sweepDeg, clockwise, sweepRpm);
            } else {
                runSweep(ros, can, sweepDeg, tensionDeg, clockwise, sweepRpm, merge, maxRangeM, minConfidence);
            }
        } catch (Exception e) {
            // не глыбаемся: выгружаем то, что успели собрать, и корректно закрываем ресурсы
            System.err.println("[MAIN] Scan aborted: " + e);
        }
        try {
            if (mode == Mode.STEP) {
                exportCloud(ros.getCloudPoints());
            }
            // sweep: точки уже стримились в XYZ и поток закрыт
        } catch (IOException e) {
            System.err.println("[MAIN] Cloud export failed: " + e);
        } finally {
            can.close();
            ros.close();
        }
    }

    // ==================== step-режим (пошаговый) ====================

    private static void runStep(TopicInterface ros, Mks42dController can,
                                double sweepDeg, float stepDeg, int scansPerStep) throws Exception {
        int scanSteps = (int) Math.round(sweepDeg / stepDeg);
        System.out.printf("[MAIN] Step mode: base %.1f deg, step %.1f deg (%d positions), motor = %.1fx base%n",
                sweepDeg, stepDeg, scanSteps + 1, GEAR_RATIO);
        for (int i = 0; i <= scanSteps; i++) {
            float baseAngle = i * stepDeg; // физический поворот основания (лечит облако)
            double motorAngle = baseAngle * GEAR_RATIO; // угол, который задаём мотору
            boolean arrived = can.turnToAbsoluteAngle(motorAngle); // ход + остановка + стабилизация
            if (!arrived) {
                throw new IllegalStateException("мотор не остановился и не стабилизировался на motor "
                        + motorAngle + " deg (base " + baseAngle + " deg)");
            }
            List<LaserScan> rawScans = ros.collectScans(scansPerStep);
            System.out.printf("[MAIN] Merging %d scans for base %.1f deg (motor %.1f deg)%n",
                    rawScans.size(), baseAngle, motorAngle);
            LaserScan merged = ScanMerger.mergeBucket(rawScans, BUCKET_FACTOR, OUTLIER_TOLERANCE);
            // Облако поворачиваем на угол ОСНОВАНИЯ (реальный поворот планки лидара)
            ros.publishAndProcess(merged, baseAngle);
            System.out.println("[MAIN] Cloud updated at base " + baseAngle + " deg");
        }
        can.turnToAbsoluteAngle(0.0); // возврат основания в нуль
        System.out.println("=== Step sweep Complete ===");
    }

    // ==================== sweep-режим (непрерывный) ====================

    private static void runSweep(TopicInterface ros, Mks42dController can, double sweepDeg,
                                 double tensionDeg, boolean clockwise, int sweepRpm,
                                 int merge, double maxRangeM, int minConfidence) throws Exception {
        // CW: энкодер идёт в минус (угол уменьшается) -> прогресс = -угол; CCW: +угол.
        int dirSign = clockwise ? -1 : +1;
        System.out.printf("[MAIN] Sweep mode: base %.1f deg, dir %s, motor %d rpm (base %.2f deg/s),"
                        + " tension %.2f deg, merge %d, maxRange %.1f m, window %.2f deg%n",
                sweepDeg, clockwise ? "CW" : "CCW", sweepRpm, sweepRpm / GEAR_RATIO,
                tensionDeg, merge, maxRangeM, SWEEP_WINDOW_DEG);

        can.setZero(); // текущая позиция = 0 (ручная установка лидара в старт)
        double initialFrameAngle = can.getAngle() / GEAR_RATIO;

        // Фоновый трекер угла рамы (мотор/4) в единой wall-clock эпохе
        EncoderTracker enc = new EncoderTracker(() -> can.getAngle() / GEAR_RATIO, ENCODER_POLL_MS);
        try {
            can.speedMove(clockwise, sweepRpm, SWEEP_ACCEL);
            waitForFrameMoving(can, initialFrameAngle, WAIT_MOVING_TIMEOUT_MS);

            SweepAccumulator acc = new SweepAccumulator(SWEEP_WINDOW_DEG, OUTLIER_TOLERANCE);
            xyzOut = XyzWriter.open(sweepXyzPath());
            long lastPrint = 0;
            double progress = 0;
            while (progress < sweepDeg) {
                ensureEncoderAlive(enc);
                long now = enc.wallNowNs();
                double frameAngle = enc.angleAt(now);
                if (Double.isNaN(frameAngle)) {
                    Thread.sleep(50); // энкодер ещё не дал данных — подождать
                    continue;
                }
                progress = dirSign * frameAngle;
                List<LaserScan> raw = ros.collectScans(merge);
                LaserScan slice = merge == 1
                        ? raw.get(0)
                        : ScanMerger.mergeBucket(raw, 1, OUTLIER_TOLERANCE);
                List<SweepPoint> pts = TopicInterface.sweepSlicePoints(slice, enc::angleAt, dirSign,
                        tensionDeg, maxRangeM, (float) minConfidence);
                List<SweepPoint> merged = acc.addAndWindow(pts, frameAngle, dirSign);
                ros.publishSweepSlice(merged);
                for (SweepPoint mp : merged) {
                    xyzOut.writePoint(ros.worldPoint(mp));
                }
                long t = System.currentTimeMillis();
                if (t - lastPrint >= 1000) {
                    lastPrint = t;
                    System.out.printf("[MAIN] Sweep progress %.1f / %.1f deg, xyz %d pts%n",
                            progress, sweepDeg, xyzOut.count());
                }
            }
            can.stop(SWEEP_ACCEL);
            can.waitSettled(5000);
            ros.publishFullCloud();
        } finally {
            enc.close();
            flushXyz();
        }
        can.turnToAbsoluteAngle(0.0); // возврат основания в нуль
        System.out.println("=== Sweep Complete ===");
    }

    // ==================== калибровка (диагностика движения) ====================

    /**
     * Калибровка: вращает раму, как в свипе (без облака и XYZ), и диагностирует
     * энкодер и привод: периоды опроса, зависания, фактическую скорость против
     * заданной, разгон до 90% скорости и рекомендуемый --tension.
     */
    private static void runCalibrate(Mks42dController can, double sweepDeg,
                                     boolean clockwise, int sweepRpm) throws Exception {
        int dirSign = clockwise ? -1 : +1;
        double targetDegPerSec = sweepRpm / GEAR_RATIO * 6.0;
        System.out.printf("[CAL] Calibrate: base %.1f deg, dir %s, motor %d rpm (target %.2f deg/s)%n",
                sweepDeg, clockwise ? "CW" : "CCW", sweepRpm, targetDegPerSec);

        can.setZero();
        double initialFrameAngle = can.getAngle() / GEAR_RATIO;

        EncoderTracker enc = new EncoderTracker(() -> can.getAngle() / GEAR_RATIO, ENCODER_POLL_MS);
        try {
            can.speedMove(clockwise, sweepRpm, SWEEP_ACCEL);
            waitForFrameMoving(can, initialFrameAngle, WAIT_MOVING_TIMEOUT_MS);
            double nominalSec = sweepDeg / targetDegPerSec;
            long deadline = System.currentTimeMillis() + (long) (nominalSec * 3000); // 3x номинал
            double progress = 0;
            while (progress < sweepDeg) {
                ensureEncoderAlive(enc);
                double frameAngle = enc.angleAt(enc.wallNowNs());
                if (Double.isNaN(frameAngle)) {
                    Thread.sleep(50);
                    continue;
                }
                progress = dirSign * (frameAngle - initialFrameAngle);
                if (System.currentTimeMillis() > deadline) {
                    break; // рама не дошла (зависла) — диагностируем на том, что есть
                }
                Thread.sleep(50);
            }
            can.stop(SWEEP_ACCEL);
            can.waitSettled(5000);
        } finally {
            enc.close();
        }

        long[] ts = enc.snapshotTimestamps();
        double[] ang = enc.snapshotAngles();
        printCalibrateReport(ts, ang, initialFrameAngle, dirSign, targetDegPerSec, sweepDeg);
        can.turnToAbsoluteAngle(0.0); // возврат основания в нуль
        System.out.println("=== Calibrate Complete ===");
    }

    private static void printCalibrateReport(long[] ts, double[] ang, double initialFrameAngle,
                                             int dirSign, double targetDegPerSec, double sweepDeg) {
        int n = ts.length;
        if (n < 2) {
            System.out.println("[CAL] Нет достаточных сэмплов энкодера (" + n + ")");
            return;
        }
        // Периоды опроса и зависания
        double[] gapsMs = new double[n - 1];
        int stalls = 0;
        double stallMs = ENCODER_STALL_NS / 1_000_000.0;
        for (int i = 1; i < n; i++) {
            gapsMs[i - 1] = (ts[i] - ts[i - 1]) / 1_000_000.0;
            if (gapsMs[i - 1] > stallMs) {
                stalls++;
            }
        }
        System.out.printf("[CAL] Samples: %d, median gap %.1f ms, max gap %.1f ms, stalls (>%d ms): %d%n",
                n, median(gapsMs), max(gapsMs), (int) stallMs, stalls);

        // Прогресс и скорость по парам сэмплов
        double[] progress = new double[n];
        for (int i = 0; i < n; i++) {
            progress[i] = dirSign * (ang[i] - initialFrameAngle);
        }
        double[] speeds = new double[n - 1];
        for (int i = 1; i < n; i++) {
            double dt = (ts[i] - ts[i - 1]) / 1_000_000_000.0;
            speeds[i - 1] = dt > 0 ? (progress[i] - progress[i - 1]) / dt : 0.0;
        }
        double[] posSpeeds = filterPositive(speeds);
        double measured = posSpeeds.length > 0 ? median(posSpeeds) : Double.NaN;
        if (Double.isNaN(measured)) {
            System.out.printf("[CAL] Target speed: %.2f deg/s, measured: — (нет положительных скоростей)%n",
                    targetDegPerSec);
        } else {
            System.out.printf("[CAL] Target speed: %.2f deg/s, measured (median): %.2f deg/s (dev %.1f%%)%n",
                    targetDegPerSec, measured, (measured - targetDegPerSec) / targetDegPerSec * 100);
        }

        // Разгон: первый сэмпл, где скорость достигла 90% отцёленной
        int rampIdx = -1;
        for (int i = 0; i < speeds.length; i++) {
            if (speeds[i] >= 0.9 * targetDegPerSec) {
                rampIdx = i;
                break;
            }
        }
        double rampDist = 0;
        if (rampIdx >= 0) {
            double rampTimeMs = (ts[rampIdx + 1] - ts[0]) / 1_000_000.0;
            rampDist = progress[rampIdx + 1];
            double tension = Math.ceil((rampDist + 0.5) * 10) / 10; // разгон + запас 0.5 град
            System.out.printf("[CAL] Ramp: %.1f s, %.1f deg (до 90%% скорости)%n",
                    rampTimeMs / 1000.0, rampDist);
            System.out.printf("[CAL] Recommended --tension: %.1f deg (ramp %.1f + margin 0.5)%n",
                    tension, rampDist);
        } else {
            System.out.println("[CAL] Ramp: скорость не достигла 90% отцёленной (привод слабый?)");
        }
        System.out.printf("[CAL] Total progress: %.1f deg (requested %.1f)%n", progress[n - 1], sweepDeg);
    }

    private static double[] filterPositive(double[] v) {
        int c = 0;
        for (double x : v) {
            if (x > 0) c++;
        }
        double[] out = new double[c];
        int k = 0;
        for (double x : v) {
            if (x > 0) out[k++] = x;
        }
        return out;
    }

    private static double median(double[] arr) {
        double[] a = arr.clone();
        java.util.Arrays.sort(a);
        int m = a.length / 2;
        return a.length % 2 == 0 ? (a[m - 1] + a[m]) / 2 : a[m];
    }

    private static double max(double[] arr) {
        double m = Double.NEGATIVE_INFINITY;
        for (double x : arr) {
            if (x > m) m = x;
        }
        return m;
    }

    /** Ждём, пока рама реально тронется (угол сдвинется на eps от стартового). */
    private static void waitForFrameMoving(Mks42dController can, double initialFrameAngle, long timeoutMs)
            throws InterruptedException {
        long deadline = System.currentTimeMillis() + timeoutMs;
        while (System.currentTimeMillis() < deadline) {
            double a = can.getAngle() / GEAR_RATIO;
            if (Math.abs(a - initialFrameAngle) > WAIT_MOVING_EPS_DEG) {
                return;
            }
            Thread.sleep(50);
        }
        throw new IllegalStateException("рама не тронулась за " + timeoutMs + " ms (мотор не пошёл?)");
    }

    /** Бросает, если трекер энкодера давно не писал сэмплы (CAN-зависание). */
    private static void ensureEncoderAlive(EncoderTracker enc) {
        long last = enc.lastSampleNs();
        if (last < 0) {
            throw new IllegalStateException("энкодер не дал ни одного сэмпла");
        }
        long stall = enc.wallNowNs() - last;
        if (stall > ENCODER_STALL_NS) {
            throw new IllegalStateException("энкодер молчит " + (stall / 1_000_000) + " ms (CAN-зависание?)");
        }
    }

    // ==================== общий ====================

    private static Path sweepXyzPath() throws IOException {
        Path dir = Paths.get("scan_export");
        Files.createDirectories(dir);
        String stamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss"));
        return dir.resolve("sweep_" + stamp + ".xyz");
    }

    private static void exportCloud(List<float[]> points) throws IOException {
        Path dir = Paths.get("scan_export");
        Files.createDirectories(dir);
        String stamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss"));
        Path xyz = dir.resolve("scan_" + stamp + ".xyz");
        XyzWriter.write(xyz, points);
        System.out.println("[MAIN] Exported " + points.size() + " points -> " + xyz);
    }

    /** Дописать и закрыть инкрементальный XYZ-поток (идемпотентно). */
    private static void flushXyz() {
        XyzWriter.Stream s = xyzOut;
        if (s != null) {
            xyzOut = null;
            try {
                s.close();
            } catch (IOException e) {
                System.err.println("[MAIN] XYZ flush failed: " + e);
            }
        }
    }

    private static void validate(Mode mode, double sweepDeg, float stepDeg, int scansPerStep,
                                  int currentMa, int holdMa, double tiltDeg, int maxRpm, int maxAcc,
                                  double tensionDeg, int sweepRpm, int merge, double maxRangeM,
                                  int minConfidence, boolean calibrate) {
        if (sweepDeg <= 0 || sweepDeg > 360) {
            fail("--sweep должен быть в (0..360] град, получен " + sweepDeg);
        }
        if (mode == Mode.STEP) {
            if (stepDeg <= 0) {
                fail("--step должен быть > 0, получен " + stepDeg);
            }
            if (scansPerStep < 1) {
                fail("--scans должен быть >= 1, получен " + scansPerStep);
            }
        }
        if (currentMa < 0) {
            fail("--current должен быть >= 0 мА, получен " + currentMa);
        }
        if (holdMa < 0) {
            fail("--hold должен быть >= 0 мА, получен " + holdMa);
        }
        if (!Double.isFinite(tiltDeg)) {
            fail("--tilt должен быть конечным числом, получен " + tiltDeg);
        }
        if (maxRpm < 0 || maxRpm > Mks42dController.SPEED_LIMIT_RPM) {
            fail("--rpm должен быть 0.." + Mks42dController.SPEED_LIMIT_RPM + ", получен " + maxRpm);
        }
        if (maxAcc < 0 || maxAcc > Mks42dController.ACCEL_LIMIT) {
            fail("--acc должен быть 0.." + Mks42dController.ACCEL_LIMIT + ", получен " + maxAcc);
        }
        if (mode == Mode.SWEEP) {
            if (tensionDeg < 0) {
                fail("--tension должен быть >= 0, получен " + tensionDeg);
            }
            if (sweepRpm < 1 || sweepRpm > Mks42dController.SPEED_LIMIT_RPM) {
                fail("--sweep-rpm должен быть 1.." + Mks42dController.SPEED_LIMIT_RPM + ", получен " + sweepRpm);
            }
            if (merge < 1 || merge > 32) {
                fail("--merge должен быть 1..32, получен " + merge);
            }
            if (maxRangeM <= 0) {
                fail("--max-range должен быть > 0, получен " + maxRangeM);
            }
        }
        if (minConfidence < 0 || minConfidence > 255) {
            fail("--min-confidence должен быть 0..255, получен " + minConfidence);
        }
        if (calibrate && mode != Mode.SWEEP) {
            fail("--calibrate доступен только в режиме sweep");
        }
    }

    private static Mode parseMode(String s) {
        if (s.equalsIgnoreCase("step")) {
            return Mode.STEP;
        }
        if (s.equalsIgnoreCase("sweep")) {
            return Mode.SWEEP;
        }
        fail("Неизвестный режим: " + s + " (step|sweep)");
        return Mode.STEP; // fail() завершает процесс
    }

    private static boolean parseDirection(String s) {
        if (s.equalsIgnoreCase("cw")) {
            return true;
        }
        if (s.equalsIgnoreCase("ccw")) {
            return false;
        }
        fail("Неизвестное направление: " + s + " (cw|ccw)");
        return true; // fail() завершает процесс
    }

    private static void fail(String message) {
        System.err.println("[MAIN] " + message);
        printUsage();
        System.exit(1);
    }

    private static void printUsage() {
        System.out.println("""
            Сканирование объёма: поворот основания на сервоприводе + сборка облака.
            Использование: java -cp <jar> org.example.MainOrchestrator [canIface] [параметры]

              canIface       SocketCAN-интерфейс, Linux (по умолч. can0; на Windows — PcanBus, игнорируется)
              --mode M       режим: step (пошаговый, по умолч.) | sweep (непрерывный)
              --sweep DEG    ход основания, град (по умолч. 90)
              --step DEG     шаг, град (step; по умолч. 10)
              --scans N      сканов /scan на позицию (step; по умолч. 10)
              --rpm N        макс. скорость мотора, RPM (0 = профиль: 1/3/5)
              --acc N        макс. ускорение (0 = профиль: 5/8/25)
              --current MA   рабочий ток, мА (0 = не менять)
              --hold MA      удерживающий ток, мА (0 = не менять)
              --tilt DEG     наклон луча 0° от горизонтали (по умолч. 0)
              --tension DEG  (sweep) зона натяжения у старта, град (по умолч. 2)
              --direction D  (sweep) cw (по умолч.) | ccw
              --sweep-rpm N  (sweep) скорость мотора, RPM (по умолч. 1; рама = N/4 град/с*6)
               --merge N      (sweep) сканов /scan на срез, 1..32 (по умолч. 1)
               --max-range M  (sweep) ограничение дальности, м (по умолч. 3)
               --min-confidence N порог интенсивности 0..255: точки с меньшей confidence
                                 отбрасываются из облака (0 = фильтр выключен, по умолч.)
               --calibrate    (sweep) диагностика движения: скорость, разгон, зависания
                              энкодера, рекомендуемый --tension (без облака и XYZ)
               -h, --help     эта справка""");
    }
}
