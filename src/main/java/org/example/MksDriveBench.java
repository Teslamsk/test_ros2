package org.example;

import org.example.can.Mks42dController;
import org.example.can.transport.CanBus;
import org.example.can.transport.PcanBus;
import org.example.can.transport.SocketCanBus;

/**
 * Железный бенч для MKS SERVO42D_CAN: завести привод, опросить состояние,
 * подвигать туда-обратно, показать реальную позицию после каждого хода.
 *
 * Запуск (сначала собрать: mvn -o -q package):
 *   java -cp target/ros2-demo-1.0-SNAPSHOT.jar org.example.MksDriveBench --help
 *
 * Windows: нужен CAN-адаптер Peak (PCAN-USB*) с драйвером и pcanbasic.dll
 * (транспорт PcanBus). Linux: SocketCAN (libcanwrapper.so, /dev/can0).
 * Привод: +24V, CAN-H/L/GND к шине, 500 kbit/s, адрес по умолчанию 0x01.
 *
 * ВНИМАНИЕ: программа реально вращает вал привода. Свобода вала — допускать!
 */
public final class MksDriveBench {

    private static final long MOVE_TIMEOUT_MS = 15_000;

    private static volatile boolean stopRequested;

    public static void main(String[] args) {
        Options o = parseArgs(args);

        System.out.println("=== MKS SERVO42D bench (привод будет ВРАЩАТЬСЯ) ===");
        System.out.println("  узел        : 0x" + Integer.toHexString(o.node));
        System.out.println("  скорость    : " + o.speed + " RPM, ускорение " + o.acc);
        System.out.println("  ход         : +" + o.angle + "° -> 0°, циклов: " + o.moves);
        System.out.println("  ток         : " + (o.currentMa > 0 ? o.currentMa + " mA" : "не менять"));
        System.out.println("  удержив. ток: " + (o.holdMa > 0 ? o.holdMa + " mA" : "не менять"));
        System.out.println("  микрошаг    : " + (o.subdiv > 0 ? o.subdiv : "не менять"));
        System.out.println("  калибровка  : " + (o.calibrate ? "БУДЕТ (0x80)" : "нет"));
        System.out.println("  дамп парам. : " + (o.params ? "БЕЗ ДВИЖЕНИЯ (0x00..0xBF)" : "нет"));
        System.out.println("  нуль        : автоматически при init (текущая позиция = 0)");
        System.out.println("Ctrl-C — аварийный стоп и выход.");

        boolean windows = System.getProperty("os.name", "").toLowerCase().contains("win");
        CanBus bus = windows ? new PcanBus(o.channel) : new SocketCanBus();
        Mks42dController drive = new Mks42dController(bus, o.node);

        Thread mainThread = Thread.currentThread();
        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            System.out.println("[BENCH] Ctrl-C: аварийный стоп...");
            stopRequested = true;
            try {
                drive.emergencyStop();
            } catch (Exception ignored) {
                // привод мог молчать
            }
        }, "bench-shutdown"));

        try {
            run(drive, o, windows);
        } catch (InterruptedException e) {
            System.out.println("[BENCH] Остановлено по Ctrl-C.");
        } catch (Exception e) {
            System.err.println("[BENCH] Ошибка: " + e);
            e.printStackTrace();
        } finally {
            try {
                drive.emergencyStop();
            } catch (Exception ignored) {
                // уже стоит / молчит
            }
            drive.close();
        }
    }

    private static void run(Mks42dController drive, Options o, boolean windows) throws InterruptedException {
        // 1. Запуск: F7 -> режим SR_vFOC -> enable -> нуль (один раз, в init).
        // Повторный 0x92 привод отвечает молчанием — зануляем строго один раз за запуск.
        // PcanBus игнорирует iface (канал задан в конструкторе), SocketCanBus использует его.
        String iface = windows ? "PCAN (channel 0x" + Integer.toHexString(o.channel) + ")" : o.iface;
        drive.init(iface);
        if (o.params) {
            dumpAllParams(drive);
            return;
        }
        if (o.calibrate) {
            System.out.println("[BENCH] Калибровка энкодера (0x80) — вал может повернуться...");
            drive.calibrate();
        }
        if (o.currentMa > 0) {
            drive.setWorkingCurrent(o.currentMa);
        }
        if (o.holdMa > 0) {
            drive.setHoldingCurrent(o.holdMa);
        }
        if (o.subdiv > 0) {
            drive.setSubdivisions(o.subdiv);
        }
        sleep(200);
        snapshot(drive, "после запуска");

        // 2. Туда-обратно
        for (int m = 1; m <= o.moves && !stopRequested; m++) {
            System.out.println("[BENCH] Ход " + m + "/" + o.moves);
            move(drive, o.angle, o, "туда");
            if (stopRequested) {
                break;
            }
            move(drive, 0.0, o, "обратно");
        }

        // 3. Финальный опрос
        snapshot(drive, "финал");
        double angle = drive.getAngle();
        if (Math.abs(angle) > 1.0) {
            System.err.printf("[BENCH] WARNING: финальная позиция %.2f deg — не нуль!%n", angle);
        } else {
            System.out.printf("[BENCH] OK: финальная позиция %.2f deg%n", angle);
        }
    }

    /**
     * Ход к абсолютной позиции с проверкой остановки и реальной позиции.
     */
    private static void move(Mks42dController drive, double to, Options o, String label) throws InterruptedException {
        long t0 = System.currentTimeMillis();
        drive.turnToAbsoluteAngle(to, o.speed, o.acc);
        boolean ok = waitForStop(drive);
        double actual = 0;
        try {
            actual = drive.getAngle();
        } catch (Exception e) {
            System.err.println("[BENCH] не удалось прочитать позицию: " + e.getMessage());
        }
        System.out.printf("[BENCH]   %s: цель %.1f deg — %s (за %d ms), реально %.2f deg (ошибка %+.2f deg)%n",
                label, to, ok ? "остановился" : "ТАЙМАУТ остановки", System.currentTimeMillis() - t0, actual, actual - to);
        if (!ok) {
            System.err.println("[BENCH] аварийный стоп (мотор не остановился)");
            drive.emergencyStop();
            throw new IllegalStateException("мотор не остановился после хода в " + to + " deg");
        }
    }

    private static boolean waitForStop(Mks42dController drive) throws InterruptedException {
        long deadline = System.currentTimeMillis() + MOVE_TIMEOUT_MS;
        while (System.currentTimeMillis() < deadline) {
            if (stopRequested) {
                return false;
            }
            if (drive.isStopped()) {
                return true;
            }
            Thread.sleep(50);
        }
        return drive.isStopped();
    }

    /**
     * Опрос и таблица состояния привода.
     */
    private static void snapshot(Mks42dController drive, String when) {
        System.out.println("[BENCH] ── Опрос: " + when + " ──");
        Integer st = drive.statusOrNull();
        System.out.println("[BENCH]   статус (0xF1)   : " + (st == null ? "НЕТ ОТВЕТА" : st + " (" + statusName(st) + ")"));
        try {
            long pos = drive.readEncoder();
            System.out.printf("[BENCH]   позиция (0x31)  : %d units = %.2f deg%n", pos, Mks42dController.axisToDegrees(pos));
        } catch (Exception e) {
            System.out.println("[BENCH]   позиция (0x31)  : НЕТ ОТВЕТА");
        }
        try {
            System.out.println("[BENCH]   скорость (0x32) : " + drive.readSpeed() + " RPM");
        } catch (Exception e) {
            System.out.println("[BENCH]   скорость (0x32) : НЕТ ОТВЕТА");
        }
        for (String[] p : systemParams()) {
            byte[] v = drive.readSystemParameter(Integer.parseInt(p[0], 16));
            if (v == null) {
                System.out.println("[BENCH]   0x" + p[0] + " " + p[1] + ": (не читается)");
            } else {
                System.out.println("[BENCH]   0x" + p[0] + " " + p[1] + ": " + bytesToDecimal(v)
                        + (p.length > 2 ? p[2] : ""));
            }
        }
    }

    private static String[][] systemParams() {
        return new String[][]{
                {"82", "режим работы", " (0-2=stepper без CAN, 3-5=serial)"},
                {"83", "рабочий ток, мА", ""},
                {"84", "микрошаг", ""},
                {"8A", "скорость CAN", " (0=125k,1=250k,2=500k,3=1M)"},
                {"8B", "CAN ID", ""},
        };
    }

    private static String bytesToDecimal(byte[] v) {
        long x = 0;
        for (byte b : v) {
            x = (x << 8) | (b & 0xFF);
        }
        return Long.toUnsignedString(x);
    }

    /**
     * Справочный дамп ВСЕХ системных параметров 0x00..0xBF (только чтение, вал не крутится).
     * Нечитаемые параметры (FF FF / тишина) пропускаются. Служит для сверки с таблицей
     * параметров мануала — в т.ч. чтобы найти регуляторные параметры, если они есть.
     */
    private static void dumpAllParams(Mks42dController drive) {
        System.out.println("[BENCH] ── Дамп параметров 0x00..0xBF (без движения) ──");
        for (int code = 0x00; code <= 0xBF; code++) {
            byte[] v;
            try {
                v = drive.readSystemParameter(code, 40);
            } catch (Exception e) {
                continue;
            }
            if (v == null) {
                continue; // FF FF (не поддерживается) или тишина
            }
            System.out.printf("[BENCH]   0x%02X: %s%n", code, toHexBytes(v));
        }
        System.out.println("[BENCH] Дамп завершён.");
    }

    private static String toHexBytes(byte[] v) {
        StringBuilder sb = new StringBuilder();
        for (byte b : v) {
            sb.append(String.format("%02X", b & 0xFF)).append(' ');
        }
        return sb.toString().trim();
    }

    private static String statusName(int st) {
        return switch (st) {
            case Mks42dController.ST_STOP -> "остановлен";
            case Mks42dController.ST_SPEED_UP -> "разгон";
            case Mks42dController.ST_SPEED_DOWN -> "замедление";
            case Mks42dController.ST_FULL_SPEED -> "полная скорость";
            case Mks42dController.ST_HOMING -> "homing";
            case Mks42dController.ST_CALIBRATING -> "калибровка";
            default -> "неизвестный";
        };
    }

    // ---------- параметры ----------

    private static final class Options {
        int node = 1;
        int channel = PcanBus.PCAN_USB1;
        // имя SocketCAN-интерфейса ("can0"), НЕ путь "/dev/can0": ядро ищет по ifname
        String iface = "can0";
        int speed = 20;
        int acc = 100;
        double angle = 30.0;
        int moves = 3;
        int currentMa = 0;
        int holdMa = 0;
        int subdiv = 0;
        boolean calibrate = false;
        boolean params = false;
    }

    private static Options parseArgs(String[] args) {
        Options o = new Options();
        for (int i = 0; i < args.length; i++) {
            String a = args[i];
            switch (a) {
                case "-h", "--help" -> {
                    printUsage();
                    System.exit(0);
                }
                case "--node" -> o.node = toInt(value(args, ++i, a), a);
                case "--channel" -> o.channel = toChannel(value(args, ++i, a));
                case "--iface" -> o.iface = value(args, ++i, a);
                case "--speed" -> o.speed = toInt(value(args, ++i, a), a);
                case "--acc" -> o.acc = toInt(value(args, ++i, a), a);
                case "--angle" -> o.angle = toDouble(value(args, ++i, a), a);
                case "--moves" -> o.moves = toInt(value(args, ++i, a), a);
                case "--current" -> o.currentMa = toInt(value(args, ++i, a), a);
                case "--hold" -> o.holdMa = toInt(value(args, ++i, a), a);
                case "--subdiv" -> o.subdiv = toInt(value(args, ++i, a), a);
                case "--calibrate" -> o.calibrate = true;
                case "--params" -> o.params = true;
                default -> {
                    System.err.println("Неизвестный параметр: " + a);
                    printUsage();
                    System.exit(1);
                }
            }
        }
        if (o.node < 1 || o.node > 0x7FF) {
            fail("--node: 01..0x7FF");
        }
        if (o.speed < 1 || o.speed > Mks42dController.SPEED_LIMIT_RPM) {
            fail("--speed: 1..3000 RPM");
        }
        if (o.acc < 0 || o.acc > Mks42dController.ACCEL_LIMIT) {
            fail("--acc: 0..255");
        }
        if (o.angle < 0 || o.angle > 360) {
            fail("--angle: 0..360 deg");
        }
        if (o.moves < 1) {
            fail("--moves: >= 1");
        }
        return o;
    }

    private static int toChannel(String s) {
        if (s.startsWith("0x") || s.startsWith("0X")) {
            return Integer.parseInt(s.substring(2), 16);
        }
        return Integer.parseInt(s);
    }

    private static String value(String[] args, int i, String flag) {
        if (i >= args.length) {
            fail("Для " + flag + " нужно значение");
        }
        return args[i];
    }

    private static int toInt(String s, String flag) {
        try {
            return Integer.parseInt(s);
        } catch (NumberFormatException e) {
            fail(flag + ": не целое: " + s);
            return 0;
        }
    }

    private static double toDouble(String s, String flag) {
        try {
            return Double.parseDouble(s);
        } catch (NumberFormatException e) {
            fail(flag + ": не число: " + s);
            return 0;
        }
    }

    private static void fail(String msg) {
        System.err.println(msg);
        System.exit(1);
    }

    private static void sleep(int ms) throws InterruptedException {
        Thread.sleep(ms);
    }

    private static void printUsage() {
        System.out.println("""
                Бенч MKS SERVO42D_CAN: запуск + опрос + движение туда-обратно.

                Использование: java -cp <jar> org.example.MksDriveBench [параметры]

                  --node N       адрес привода (по умолч. 1; можно 0x01)
                  --channel C    PCAN-канал, Windows (по умолч. 0x43 = PCAN-USB1;
                                 0x44 = PCAN-USB2, ...)
                   --iface IF     SocketCAN-интерфейс, Linux (по умолч. can0)
                  --speed RPM    скорость (по умолч. 20, макс 3000)
                  --acc A        ускорение 0..255 (по умолч. 100)
                  --angle D      угол хода туда в градусах (по умолч. 30)
                  --moves N      число туда-обратно циклов (по умолч. 3)
                   --current MA   рабочий ток, мА (0 = не менять, макс 5200)
                   --hold MA      удерживающий ток (0x9B), мА (0 = не менять) —
                                  против проскальзывания рамы в останове
                   --subdiv N     микрошаг (0 = не менять)
                    --calibrate    запустить калибровку энкодера 0x80 (ВАЛ ПОВОРАТИТСЯ)
                    --params       дамп всех параметров 0x00..0xBF и ВЫХОД
                                   (без движения; для сверки с мануалом)
                    -h, --help     эта справка

                 Примеры:
                   org.example.MksDriveBench                              defaults: 20 RPM, 30°, 3 цикла
                   org.example.MksDriveBench --angle 90 --moves 5         большие ходы
                   org.example.MksDriveBench --node 0x02                  второй привод на шине
                   org.example.MksDriveBench --calibrate                  если нужна калибровка
                   org.example.MksDriveBench --params                     только дамп параметров
                   org.example.MksDriveBench --current 2400 --hold 2000   больший ток против автоколебаний
                """);
    }
}
