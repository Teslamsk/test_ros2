package org.example.vlp16;

import org.example.can.Mks42dController;
import org.example.can.transport.CanBus;
import org.example.can.transport.CanBusFactory;
import org.example.export.XyzWriter;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;

/**
 * VLP-16 на раме MKS 42D: пошаговый сбор объёма.
 *
 * <p>Лидар VLP-16 смонтирован на раме с ГORIZОНТАЛЬНОЙ осью вращения и сам
 * вращается (~5 об/с); рама поворачивается шагами вокруг вертикальной оси Z.
 * Не-кратный шаг (деф. 10.3°) обеспечивает покрытие объёма объединением колец
 * (симуляция: 99% в ~212 шагах, см. AGENTS.md).
 *
 * <p>Каждый шаг: turnToAbsoluteAngle (ход + остановка + стабилизация) → окно
 * удержания: все точки за окно (рама неподвижна — каждая помечается текущим
 * углом рамы A_n, per-point time не нужен) → трансформ в мировые координаты →
 * стриминг XYZ + срез в /pointCloud (RViz: Fixed frame = base, Display = Accumulate).
 *
 * <p>Точки приходят по ROS2-топику /velodyne_points от официального ROS2-драйвера
 * velodyne (см. {@link Vlp16Cloud}).
 *
 * <p>Запуск (из каталога с jar):
 * <pre>
 *   java -cp ros2-demo-1.0-SNAPSHOT.jar org.example.vlp16.Vlp16Orchestrator \
 *        [canIface] --sweep 360 --step 10.3 --window 1000 --max-range 20 --height 0.4
 * </pre>
 */
public final class Vlp16Orchestrator {

    /**
     * Передаточное число мотор:рама = 17:1. НЕ тот же, что в MainOrchestrator
     * (D500: 4:1) — при смене редукции рамы VLP-16 менять только эту константу.
     */
    private static final double GEAR_RATIO = 17.0;

    private static final double FRAME_ANGLE_SIGN = -1.0;

    /** Инкрементальный XYZ-поток (static: шатдаун-хук может его дописать). */
    private static volatile XyzWriter.Stream xyzOut;

    public static void main(String[] args) {
        String canIface = "can0";
        double sweepDeg = 360.0;      // ход рамы, град
        float stepDeg = 10.3f;        // шаг, град (не-кратный: 99% покрытия в ~212 шагах)
        int windowMs = 1000;          // окно удержания на позицию, ms (5 полных перерисовок колец)
        double maxRangeM = 20.0;      // ограничение дальности, м
        int minIntensity = 0;         // порог интенсивности 0..255 (0 = фильтр выключен)
        int colorExp = 2;             // поправка на расстояние: V = intensity * r^N (0 = без поправки)
        String colorMode = "heat";    // карта цвета: heat | gray | off
        float heightM = Vlp16Mount.DEFAULT_HEIGHT; // высота центра лидара над основанием, м
        String topic = Vlp16Cloud.DEFAULT_TOPIC;
        int currentMa = 0;            // рабочий ток, мА (0 = не трогать)
        int holdMa = 0;               // удерживающий ток, мА (0 = не трогать)
        int maxRpm = 0;               // ограничитель скорости мотора, RPM (0 = профиль)
        int maxAcc = 0;               // ограничитель ускорения (0 = профиль)

        try {
            for (int i = 0; i < args.length; i++) {
                switch (args[i]) {
                    case "--sweep":         sweepDeg = Double.parseDouble(args[++i]); break;
                    case "--step":          stepDeg = Float.parseFloat(args[++i]); break;
                    case "--window":        windowMs = Integer.parseInt(args[++i]); break;
                    case "--max-range":     maxRangeM = Double.parseDouble(args[++i]); break;
                    case "--min-intensity": minIntensity = Integer.parseInt(args[++i]); break;
                    case "--color-exp":     colorExp = Integer.parseInt(args[++i]); break;
                    case "--color":         colorMode = args[++i]; break;
                    case "--height":        heightM = (float) Double.parseDouble(args[++i]); break;
                    case "--topic":         topic = args[++i]; break;
                    case "--current":       currentMa = Integer.parseInt(args[++i]); break;
                    case "--hold":          holdMa = Integer.parseInt(args[++i]); break;
                    case "--rpm":           maxRpm = Integer.parseInt(args[++i]); break;
                    case "--acc":           maxAcc = Integer.parseInt(args[++i]); break;
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
        validate(sweepDeg, stepDeg, windowMs, maxRangeM, minIntensity, colorExp, colorMode,
                heightM, currentMa, holdMa, maxRpm, maxAcc);

        System.out.println("=== VLP-16 Orchestrator (step) ===");
        Vlp16Cloud ros = new Vlp16Cloud("vlp16_node", topic);
        CanBus bus = CanBusFactory.forCurrentOS();
        Mks42dController can = new Mks42dController(bus);
        try {
            can.init(canIface);
            Runtime.getRuntime().addShutdownHook(new Thread(() -> {
                System.err.println("[VLP16] Ctrl-C: аварийный стоп...");
                flushXyz();
                can.close();
            }, "vlp16-shutdown"));
            if (maxRpm > 0 || maxAcc > 0) {
                can.tuneMotion(maxRpm, maxAcc);
            }
            if (currentMa > 0) {
                can.setWorkingCurrent(currentMa);
            }
            if (holdMa > 0) {
                can.setHoldingCurrent(holdMa);
            }

            int steps = (int) Math.round(sweepDeg / stepDeg);
            System.out.printf("[VLP16] Step mode: frame %.1f deg, step %.1f deg (%d positions), "
                    + "window %d ms, motor = %.1fx frame%n", sweepDeg, stepDeg, steps + 1, windowMs, GEAR_RATIO);
            System.out.printf("[VLP16] maxRange %.1f m, minIntensity %d (0=off), height %.2f m, topic %s%n",
                    maxRangeM, minIntensity, heightM, topic);

            xyzOut = XyzWriter.open(xyzPath());
            long t0 = System.currentTimeMillis();
            try {
                for (int i = 0; i <= steps; i++) {
                    float baseAngle = Math.min(i * stepDeg, (float) sweepDeg);
                    boolean arrived = can.turnToAbsoluteAngle(baseAngle * GEAR_RATIO);
                    if (!arrived) {
                        throw new IllegalStateException("мотор не остановился и не стабилизировался на motor "
                                + baseAngle * GEAR_RATIO + " deg (frame " + baseAngle + " deg)");
                    }
                    // Фактический угол рамы с энкодера (не заданный) — геометрия
                    // независима от ошибки привода.
                    double encoderFrameAngle = can.getAngle() / GEAR_RATIO;
                    double frameAngle = softwareFrameAngle(encoderFrameAngle);
                    List<float[]> raw = ros.collectPoints(windowMs);
                    List<float[]> slice = transformAndFilter(raw, frameAngle, heightM, maxRangeM, minIntensity);
                    List<float[]> colored = colorize(slice, heightM, colorExp, colorMode);
                    for (float[] p : colored) {
                        xyzOut.writeColoredPoint(p);
                    }
                    ros.publishSlice(colored);
                    System.out.printf("[VLP16] step %d/%d: frame %.1f deg (enc %.1f), raw %d, cloud %d (xyz %d)%n",
                            i, steps, frameAngle, encoderFrameAngle, raw.size(), colored.size(), xyzOut.count());
                }
            } finally {
                flushXyz();
            }
            can.turnToAbsoluteAngle(0.0); // возврат рамы в нуль
            System.out.printf("=== VLP-16 step complete in %d ms ===%n", System.currentTimeMillis() - t0);
        } catch (Exception e) {
            // не глыбаемся: выгружаем то, что успели собрать, и корректно закрываем ресурсы
            System.err.println("[VLP16] Scan aborted: " + e);
        } finally {
            can.close();
            ros.close();
        }
    }

    /**
     * Точки системы координат лидара → мировые (base) + фильтр валидности:
     * нет возврата (0,0,0), NaN/Inf, дальность > maxRangeM, интенсивность < minIntensity.
     */
    static double softwareFrameAngle(double encoderFrameAngleDeg) {
        return FRAME_ANGLE_SIGN * encoderFrameAngleDeg;
    }

    static List<float[]> transformAndFilter(List<float[]> raw, double frameAngleDeg,
                                            float heightM, double maxRangeM, int minIntensity) {
        List<float[]> out = new ArrayList<>(raw.size());
        for (float[] p : raw) {
            float x = p[0], y = p[1], z = p[2], intensity = p[3];
            if (!Float.isFinite(x) || !Float.isFinite(y) || !Float.isFinite(z)
                    || !Float.isFinite(intensity)) {
                continue;
            }
            if (x == 0f && y == 0f && z == 0f) {
                continue; // нет возврата
            }
            double r = Math.sqrt(x * x + y * y + z * z);
            if (r > maxRangeM) {
                continue;
            }
            if (minIntensity > 0 && intensity < minIntensity) {
                continue;
            }
            float[] w = Vlp16Mount.toWorld(new float[]{x, y, z}, (float) frameAngleDeg,
                    Vlp16Mount.DEFAULT_MATRIX, heightM);
            out.add(new float[]{w[0], w[1], w[2], intensity});
        }
        return out;
    }

    /**
     * Цвет точки: интенсивность с поправкой на расстояние, нормированная по срезу, в RGB.
     *
     * <p>Поправка: {@code V = intensity * r^colorExp}, где r — дальность от центра лидара
     * (мир: {@code sqrt(x²+y²+(z-heightM)²)}). Компенсирует 1/r²-затухание сигнала:
     * отражающая цель одинаково ярка на любой дальности. Нормировка по срезу (V/V_max) —
     * стриминг без буферизации всего скана.
     *
     * <p>Вход: {@code {x, y, z, intensity}} (мир). Выход: {@code {x, y, z, intensity, r, g, b}}.
     */
    static List<float[]> colorize(List<float[]> worldPoints, float heightM, int colorExp, String colorMode) {
        if (worldPoints == null || worldPoints.isEmpty()) {
            return List.of();
        }
        int n = worldPoints.size();
        double[] v = new double[n];
        double vMax = 0.0;
        for (int i = 0; i < n; i++) {
            float[] p = worldPoints.get(i);
            double dx = p[0], dy = p[1], dz = p[2] - heightM;
            double r = Math.sqrt(dx * dx + dy * dy + dz * dz);
            v[i] = p[3] * Math.pow(r, colorExp);
            if (v[i] > vMax) {
                vMax = v[i];
            }
        }
        List<float[]> out = new ArrayList<>(n);
        for (int i = 0; i < n; i++) {
            float[] p = worldPoints.get(i);
            float t = vMax > 0 ? (float) (v[i] / vMax) : 0f;
            int[] rgb = colorMap(t, colorMode);
            out.add(new float[]{p[0], p[1], p[2], p[3], rgb[0], rgb[1], rgb[2]});
        }
        return out;
    }

    /**
     * Карта цвета для t в [0,1]: {@code off} — серый, {@code gray} — серый градиент,
     * {@code heat} (по умолч.) — 5 опор: сине → циан → зелё → жёлт → крас.
     */
    static int[] colorMap(float t, String mode) {
        if ("off".equals(mode)) {
            return new int[]{128, 128, 128};
        }
        if ("gray".equals(mode)) {
            int g = (int) Math.round(clamp01(t) * 255);
            return new int[]{g, g, g};
        }
        // heat: 5 опор (t, r, g, b)
        float[] st = {0f, 0.25f, 0.5f, 0.75f, 1f};
        int[] rs = {0, 0, 0, 255, 255};
        int[] gs = {0, 255, 255, 255, 0};
        int[] bs = {255, 255, 0, 0, 0};
        t = clamp01(t);
        for (int i = 0; i < 4; i++) {
            if (t <= st[i + 1]) {
                float f = (t - st[i]) / (st[i + 1] - st[i]);
                return new int[]{
                        (int) Math.round(rs[i] + f * (rs[i + 1] - rs[i])),
                        (int) Math.round(gs[i] + f * (gs[i + 1] - gs[i])),
                        (int) Math.round(bs[i] + f * (bs[i + 1] - bs[i]))
                };
            }
        }
        return new int[]{255, 0, 0};
    }

    private static float clamp01(float t) {
        return Math.max(0f, Math.min(1f, t));
    }

    private static Path xyzPath() throws IOException {
        Path dir = Paths.get("scan_export");
        Files.createDirectories(dir);
        String stamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss"));
        return dir.resolve("vlp16_step_" + stamp + ".xyz");
    }

    /** Дописать и закрыть инкрементальный XYZ-поток (идемпотентно). */
    private static void flushXyz() {
        XyzWriter.Stream s = xyzOut;
        if (s != null) {
            xyzOut = null;
            try {
                s.close();
            } catch (IOException e) {
                System.err.println("[VLP16] XYZ flush failed: " + e);
            }
        }
    }

    private static void validate(double sweepDeg, float stepDeg, int windowMs, double maxRangeM,
                                  int minIntensity, int colorExp, String colorMode,
                                  float heightM, int currentMa, int holdMa,
                                  int maxRpm, int maxAcc) {
        if (sweepDeg <= 0 || sweepDeg > 360) {
            fail("--sweep должен быть в (0..360] град, получен " + sweepDeg);
        }
        if (stepDeg <= 0 || stepDeg > sweepDeg) {
            fail("--step должен быть в (0, sweep) град, получен " + stepDeg);
        }
        if (windowMs < 100 || windowMs > 30_000) {
            fail("--window должен быть 100..30000 ms, получен " + windowMs);
        }
        if (maxRangeM <= 0) {
            fail("--max-range должен быть > 0 м, получен " + maxRangeM);
        }
        if (minIntensity < 0 || minIntensity > 255) {
            fail("--min-intensity должен быть 0..255, получен " + minIntensity);
        }
        if (colorExp < 0) {
            fail("--color-exp должен быть >= 0, получен " + colorExp);
        }
        if (!"heat".equals(colorMode) && !"gray".equals(colorMode) && !"off".equals(colorMode)) {
            fail("--color должен быть heat|gray|off, получен " + colorMode);
        }
        if (heightM < 0) {
            fail("--height должен быть >= 0 м, получен " + heightM);
        }
        if (currentMa < 0) {
            fail("--current должен быть >= 0 мА, получен " + currentMa);
        }
        if (holdMa < 0) {
            fail("--hold должен быть >= 0 мА, получен " + holdMa);
        }
        if (maxRpm < 0 || maxRpm > Mks42dController.SPEED_LIMIT_RPM) {
            fail("--rpm должен быть 0.." + Mks42dController.SPEED_LIMIT_RPM + ", получен " + maxRpm);
        }
        if (maxAcc < 0 || maxAcc > Mks42dController.ACCEL_LIMIT) {
            fail("--acc должен быть 0.." + Mks42dController.ACCEL_LIMIT + ", получен " + maxAcc);
        }
    }

    private static void fail(String message) {
        System.err.println("[VLP16] " + message);
        printUsage();
        System.exit(1);
    }

    private static void printUsage() {
        System.out.println("""
            VLP-16 3D cloud: пошаговый поворот рамы MKS 42D + точки с /velodyne_points.
            Использование: java -cp <jar> org.example.vlp16.Vlp16Orchestrator [canIface] [параметры]

              canIface       SocketCAN-интерфейс, Linux (по умолч. can0; на Windows — PcanBus, игнорируется)
              --sweep DEG    ход рамы, град (по умолч. 360)
              --step DEG     шаг, град (по умолч. 10.3 — не-кратный: 99% покрытия в ~212 шагах)
              --window MS    окно удержания на позицию, ms (по умолч. 1000 — 5 полных перерисовок колец)
              --max-range M  ограничение дальности, м (по умолч. 20)
              --min-intensity N порог интенсивности 0..255: точки с меньшей intensity
                                отбрасываются (0 = фильтр выключен, по умолч.)
              --color-exp N   поправка на расстояние: V = intensity * r^N (N=0 — без поправки,
                                по умолч. 2 = inverse-square; компенсирует 1/r²-затухание)
              --color MODE    карта цвета точек: heat | gray | off (по умолч. heat);
                                RGB пишется в /pointCloud и в XYZ (x y z r g b)
              --height M     высота центра лидара над основанием, м (по умолч. 0.4)
              --topic TOPIC  топик драйвера velodyne (по умолч. /velodyne_points)
              --current MA   рабочий ток, мА (0 = не менять)
              --hold MA      удерживающий ток, мА (0 = не менять)
              --rpm N        макс. скорость мотора, RPM (0 = профиль: 1/3/5)
              --acc N        макс. ускорение (0 = профиль: 5/8/25)
              -h, --help     эта справка""");
    }
}
