package org.example;

import org.example.export.E57Writer;
import org.example.export.XyzWriter;
import sensor_msgs.LaserScan;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;

/**
 * Отладочный режим «лидар без мотора» для RViz2.
 *
 * Собирает окна из N сырых сканов /scan, мерджит по выбранной стратегии
 * (среднее по бакетам bucketN / вообще без деления — none) и публикует:
 *   /processedScan — LaserScan (окно; в RViz2 видно текущее окно по скану);
 *   /pointCloud    — PointCloud2 (накопленное облако, растёт от окна к окну).
 *
 * Запуск (сначала собрать: mvn -o -q package):
 *   java -cp target/ros2-demo-1.0-SNAPSHOT.jar org.example.DebugScanMain --help
 */
public final class DebugScanMain {

    private static volatile boolean stopRequested;

    public static void main(String[] args) {
        Options opts = parseArgs(args);

        System.out.println("=== Debug scan (lidar only, no motor) ===");
        System.out.println("  scans/window : " + opts.scans);
        System.out.println("  mode         : " + (opts.raw ? "none (без усреднения)" : "bucket" + opts.factor));
        System.out.println("  windows      : " + (opts.windows == 0 ? "∞ (до Ctrl-C)" : opts.windows));
        System.out.println("  hold         : " + opts.holdMs + " ms");
        System.out.println("  tol          : " + opts.tol);
        System.out.println("  export       : " + opts.export + " -> " + opts.exportDir);
        System.out.println("RViz2: добавьте дисплеи /processedScan (LaserScan) и /pointCloud (PointCloud2, frame world).");
        System.out.println("Ctrl-C — корректная остановка с экспортом облака.");

        TopicInterface ros = new TopicInterface("debug_scan_node");
        Thread mainThread = Thread.currentThread();
        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            System.out.println("[DEBUG] Остановка: финализирую текущее окно...");
            stopRequested = true;
            mainThread.interrupt();
        }, "debug-shutdown"));

        try {
            int window = 0;
            while (!stopRequested && (opts.windows == 0 || window < opts.windows)) {
                List<LaserScan> raw = ros.collectScans(opts.scans);
                LaserScan merged = opts.raw
                        ? ScanMerger.mergeRaw(raw)
                        : ScanMerger.mergeBucket(raw, opts.factor, opts.tol);
                // мотора нет — срез всегда при угле 0
                ros.publishAndProcess(merged, 0f);
                System.out.printf("[DEBUG] окно %d: %d сырых сканов -> %d точек, облако всего %d точек%n",
                        window, raw.size(), merged.getRanges().size(), ros.getCloudPoints().size());
                window++;
                if (opts.holdMs > 0) {
                    Thread.sleep(opts.holdMs);
                }
            }
            if (stopRequested) {
                System.out.println("[DEBUG] Остановлено по Ctrl-C на окне " + window + ".");
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            System.out.println("[DEBUG] Остановлено по Ctrl-C.");
        } catch (Exception e) {
            System.err.println("[DEBUG] Остановлено с ошибкой: " + e);
        }

        try {
            exportCloud(ros.getCloudPoints(), opts);
        } catch (IOException e) {
            System.err.println("[DEBUG] Экспорт облака не удался: " + e);
        } finally {
            ros.close();
        }
    }

    private static void exportCloud(List<float[]> points, Options opts) throws IOException {
        if (opts.export.isEmpty()) {
            System.out.println("[DEBUG] --export none — экспорт пропущен.");
            return;
        }
        Path dir = Paths.get(opts.exportDir);
        Files.createDirectories(dir);
        String stamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss"));
        for (String format : opts.export) {
            switch (format) {
                case "xyz" -> {
                    Path f = dir.resolve("debug_" + stamp + ".xyz");
                    XyzWriter.write(f, points);
                    System.out.println("[DEBUG] Экспорт " + points.size() + " точек -> " + f);
                }
                case "e57" -> {
                    Path f = dir.resolve("debug_" + stamp + ".e57");
                    E57Writer.write(f, points);
                    System.out.println("[DEBUG] Экспорт " + points.size() + " точек -> " + f);
                }
                default -> throw new IllegalStateException("Неизвестный формат экспорта: " + format);
            }
        }
    }

    // ---------- параметры ----------

    private static final class Options {
        int scans = 10;
        int factor = 6;
        boolean raw = false;
        int windows = 0;
        long holdMs = 500;
        float tol = 0.10f;
        String exportDir = "scan_export";
        List<String> export = new ArrayList<>(List.of("xyz"));
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
                case "--scans" -> o.scans = toInt(value(args, ++i, a), a);
                case "--mode" -> parseMode(value(args, ++i, a), o);
                case "--windows" -> o.windows = toInt(value(args, ++i, a), a);
                case "--hold" -> o.holdMs = toLong(value(args, ++i, a), a);
                case "--tol" -> o.tol = toFloat(value(args, ++i, a), a);
                case "--export" -> o.export = parseExport(value(args, ++i, a));
                case "--export-dir" -> o.exportDir = value(args, ++i, a);
                default -> {
                    System.err.println("Неизвестный параметр: " + a);
                    printUsage();
                    System.exit(1);
                }
            }
        }
        if (o.scans < 1) {
            fail("--scans должен быть >= 1");
        }
        if (o.windows < 0) {
            fail("--windows должен быть >= 0");
        }
        if (o.holdMs < 0) {
            fail("--hold должен быть >= 0");
        }
        if (!(o.tol > 0) || o.tol > 1) {
            fail("--tol должен быть в (0, 1] (доля от среднего)");
        }
        if (o.exportDir.isBlank()) {
            fail("--export-dir не может быть пустым");
        }
        return o;
    }

    private static void parseMode(String s, Options o) {
        if (s.equals("none")) {
            o.raw = true;
            return;
        }
        if (s.startsWith("bucket")) {
            int f = toInt(s.substring("bucket".length()), "--mode " + s);
            if (f < 1 || f > 64) {
                fail("--mode bucketN: N должен быть 1..64 (получено " + f + ")");
            }
            o.raw = false;
            o.factor = f;
            return;
        }
        fail("--mode: ожидается bucketN (N=1..64) или none (получено: " + s + ")");
    }

    private static List<String> parseExport(String s) {
        List<String> out = new ArrayList<>();
        for (String part : s.split(",")) {
            String p = part.trim().toLowerCase();
            if (p.isEmpty()) {
                continue;
            }
            if (!p.equals("xyz") && !p.equals("e57") && !p.equals("none")) {
                fail("--export: допустимы xyz, e57, none (получено: " + p + ")");
            }
            out.add(p);
        }
        out.remove("none");
        return out;
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
            fail(flag + ": не целое число: " + s);
            return 0;
        }
    }

    private static long toLong(String s, String flag) {
        try {
            return Long.parseLong(s);
        } catch (NumberFormatException e) {
            fail(flag + ": не целое число: " + s);
            return 0;
        }
    }

    private static float toFloat(String s, String flag) {
        try {
            return Float.parseFloat(s);
        } catch (NumberFormatException e) {
            fail(flag + ": не число: " + s);
            return 0;
        }
    }

    private static void fail(String msg) {
        System.err.println(msg);
        System.exit(1);
    }

    private static void printUsage() {
        System.out.println("""
                Отладочный запуск: лидар без мотора, окна публикуются в RViz2.

                Использование: java -cp <jar> org.example.DebugScanMain [параметры]

                  --scans N        сырых сканов /scan в одном окне (по умолч. 10)
                  --mode STRAT     bucketN — среднее по бакетам N=1..64 (бакет = шаг луча / N,
                                   по умолч. bucket6)
                                   none  — вообще без усреднения, чистить в постпроцессоре
                  --windows N      окон запускать, 0 = до Ctrl-C (по умолч. 0)
                  --hold MS        пауза между окнами, мс (по умолч. 500)
                  --tol F          порог выбросов для bucket-режима, доля от среднего (по умолч. 0.10)
                  --export LIST    xyz, e57, none — через запятую (по умолч. xyz)
                  --export-dir DIR каталог экспорта (по умолч. scan_export)
                  -h, --help       эта справка

                Примеры:
                  --scans 10 --mode bucket6               среднее из 10 сканов, бакет 1/6 шага
                  --scans 100 --mode bucket2              100 сканов, бакет 1/2 шага
                  --scans 10 --mode none                  без деления, сырые точки
                  --scans 10 --mode none --export xyz,e57 --windows 5
                """);
    }
}
