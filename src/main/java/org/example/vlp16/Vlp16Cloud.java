package org.example.vlp16;

import builtin_interfaces.Time;
import org.example.TopicInterface;
import sensor_msgs.PointCloud2;
import sensor_msgs.PointField;
import us.ihmc.fastddsjava.cdr.idl.IDLByteSequence;
import us.ihmc.fastddsjava.cdr.idl.IDLObjectSequence;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2QoSProfile;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Topic;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

/**
 * ROS2-нод VLP-16: подписывается на /velodyne_points (PointCloud2) официального
 * ROS2-драйвера velodyne и накапливает точки в окно удержания.
 *
 * <p>Драйвер делает UDP-приём + парсинг + калибровку каналов — нод получает готовые
 * декартовы точки {x, y, z, intensity, ...}. PointCloud2 парсится по ИМЕНИ поля
 * (не по layout): x/y/z обязательны, intensity опционален (любое числовое),
 * offset/point_step/endianness — из сообщения; ring/time игнорируются (в step-режиме
 * угол рамы берётся из энкодера, per-point time не нужен).
 *
 * <p>~9000 msg/s (1 пакет → 1 сообщение): подписка best_effort KEEP_LAST(100) —
 * если Java не успевает, DDS дропает; кольца перерисовываются 5 раз/с, потеря
 * допустима. Если захлёбывается — мостик (Python-нод: буфер оборота → републикация
 * ~5 Гц).
 */
public class Vlp16Cloud {

    /** Топик официального velodyne ROS2-драйвера. */
    public static final String DEFAULT_TOPIC = "/velodyne_points";

    private static final int POINT_STEP = 28; // x + y + z + intensity + r + g + b, каждый float32 (7 × 4 байта)

    private final ROS2Node node;
    private final ROS2Publisher<PointCloud2> pubPointCloud;

    /** Точки {x, y, z, intensity} в системе координат лидара. Все доступы под bufLock. */
    private final Object bufLock = new Object();
    private final List<float[]> buffer = new ArrayList<>();

    public Vlp16Cloud(String ns, String topic) {
        this.node = new ROS2Node(ns);
        this.pubPointCloud = node.createPublisher(new ROS2Topic<PointCloud2>("/pointCloud", PointCloud2.class));

        ROS2QoSProfile qos = new ROS2QoSProfile();
        qos.history(ROS2QoSProfile.History.KEEP_LAST);
        qos.depth(100);
        qos.reliability(ROS2QoSProfile.Reliability.BEST_EFFORT);
        node.createSubscription(new ROS2Topic<PointCloud2>(topic, PointCloud2.class), reader -> {
            List<float[]> pts = parse(reader.read());
            if (pts.isEmpty()) {
                return;
            }
            synchronized (bufLock) {
                buffer.addAll(pts);
                bufLock.notifyAll();
            }
        }, qos);
        System.out.println("[ROS] Subscribed to " + topic);
    }

    /**
     * Все точки, полученные за окно удержания (рама неподвижна). Буфер очищается
     * под тем же локом, поэтому первая точка следующего окна не может "уехать"
     * в приём предыдущего.
     *
     * @param holdMs длительность окна удержания, ms
     */
    public List<float[]> collectPoints(long holdMs) throws InterruptedException {
        synchronized (bufLock) {
            buffer.clear();
            long deadline = System.currentTimeMillis() + holdMs;
            while (true) {
                long remaining = deadline - System.currentTimeMillis();
                if (remaining <= 0) {
                    break;
                }
                bufLock.wait(Math.min(remaining, 100));
            }
            return new ArrayList<>(buffer);
        }
    }

    /**
     * Публикует срез шага (мировые координаты, frame base) в /pointCloud.
     * В RViz: Fixed frame = base, Display = Accumulate.
     */
    public void publishSlice(List<float[]> worldPoints) {
        if (worldPoints == null || worldPoints.isEmpty()) {
            return;
        }
        pubPointCloud.publish(buildPointCloud(worldPoints, TopicInterface.BASE_FRAME_ID, TopicInterface.rosTimeNow()));
        System.out.println("[ROS] Published slice: " + worldPoints.size() + " points");
    }

    public void close() {
        node.close();
    }

    // ==================== Standalone-режим (без CAN/рамы) ====================

    /**
     * Просмотр без рамы: подписка на топик драйвера, окно за окно — срез в
     * /pointCloud (угол рамы = 0). Для проверки драйвера/QoS/парсера/монтажки
     * по статичной сцене до сборки рамы.
     * <pre>
     *   java -cp ros2-demo-1.0-SNAPSHOT.jar org.example.vlp16.Vlp16Cloud
     *        [--topic /velodyne_points] [--window 1000] [--max-range 20]
     *        [--min-intensity 0] [--height 0.4]
     * </pre>
     */
    public static void main(String[] args) {
        String topic = DEFAULT_TOPIC;
        int windowMs = 1000;
        double maxRangeM = 20.0;
        int minIntensity = 0;
        int colorExp = 2;
        String colorMode = "heat";
        float heightM = Vlp16Mount.DEFAULT_HEIGHT;
        try {
            for (int i = 0; i < args.length; i++) {
                switch (args[i]) {
                    case "--topic":         topic = args[++i]; break;
                    case "--window":        windowMs = Integer.parseInt(args[++i]); break;
                    case "--max-range":     maxRangeM = Double.parseDouble(args[++i]); break;
                    case "--min-intensity": minIntensity = Integer.parseInt(args[++i]); break;
                    case "--color-exp":     colorExp = Integer.parseInt(args[++i]); break;
                    case "--color":         colorMode = args[++i]; break;
                    case "--height":        heightM = (float) Double.parseDouble(args[++i]); break;
                    case "-h", "--help":
                        printStandaloneUsage();
                        return;
                    default:
                        System.err.println("[VLP16] Неизвестный параметр: " + args[i]);
                        printStandaloneUsage();
                        System.exit(1);
                }
            }
        } catch (NumberFormatException e) {
            System.err.println("[VLP16] Некорректное числовое значение: " + e.getMessage());
            System.exit(1);
        }
        if (colorExp < 0) {
            System.err.println("[VLP16] --color-exp должен быть >= 0, получен " + colorExp);
            System.exit(1);
        }
        if (!"heat".equals(colorMode) && !"gray".equals(colorMode) && !"off".equals(colorMode)) {
            System.err.println("[VLP16] --color должен быть heat|gray|off, получен " + colorMode);
            System.exit(1);
        }

        Vlp16Cloud cloud = new Vlp16Cloud("vlp16_view", topic);
        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            System.err.println("[VLP16] Ctrl-C: закрываю...");
            cloud.close();
        }, "vlp16-view-shutdown"));
        System.out.printf("[VLP16] Standalone view: topic %s, window %d ms, maxRange %.1f m, "
                + "minIntensity %d, colorExp %d, color %s, height %.2f m (frame angle = 0)%n",
                topic, windowMs, maxRangeM, minIntensity, colorExp, colorMode, heightM);
        try {
            while (true) {
                List<float[]> raw = cloud.collectPoints(windowMs);
                List<float[]> slice = Vlp16Orchestrator.transformAndFilter(raw, 0.0, heightM, maxRangeM, minIntensity);
                List<float[]> colored = Vlp16Orchestrator.colorize(slice, heightM, colorExp, colorMode);
                cloud.publishSlice(colored);
                System.out.printf("[VLP16] raw %d, cloud %d%n", raw.size(), colored.size());
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } finally {
            cloud.close();
        }
    }

    private static void printStandaloneUsage() {
        System.out.println("""
            VLP-16 standalone view (без CAN/рамы): topic драйвера -> /pointCloud (RViz).
            Использование: java -cp <jar> org.example.vlp16.Vlp16Cloud [параметры]

              --topic TOPIC     топик драйвера (по умолч. /velodyne_points)
              --window MS       окно накопления, ms (по умолч. 1000)
              --max-range M     ограничение дальности, м (по умолч. 20)
              --min-intensity N порог интенсивности 0..255 (0 = выключен)
              --color-exp N     поправка на расстояние: V = intensity * r^N (N=0 — без поправки, по умолч. 2)
              --color MODE      карта цвета: heat | gray | off (по умолч. heat)
              --height M        высота, м (по умолч. 0.4)
              -h, --help        эта справка""");
    }

    // ==================== Парсинг PointCloud2 ====================

    /**
     * Парсит PointCloud2 в {x, y, z, intensity} (система координат лидара).
     * Поля ищутся по имени: x/y/z обязательны (любое числовое), intensity опционален
     * (отсутствует — 0), всё остальное (ring, time, ...) игнорируется.
     *
     * @return список точек {x, y, z, intensity}; пусто, если облако невалидно
     *         или обязательных полей нет
     */
    public static List<float[]> parse(PointCloud2 cloud) {
        if (cloud == null) {
            return List.of();
        }
        int pointStep = cloud.getPointStep();
        int n = cloud.getWidth() * cloud.getHeight();
        if (n <= 0 || pointStep <= 0) {
            return List.of();
        }
        IDLByteSequence data = cloud.getData();
        ByteBuffer src = data.getBuffer();
        if (src == null || src.position() < pointStep * n) {
            return List.of();
        }
        // Вид, ограниченный реальными данными: capacity буфера может быть больше
        // (over-alloc в ensureMinCapacity) — чтение за пределы данных запрещено.
        ByteBuffer buf = src.duplicate().clear();
        buf.limit(pointStep * n);
        buf.order(cloud.getIsBigendian() ? ByteOrder.BIG_ENDIAN : ByteOrder.LITTLE_ENDIAN);

        int offX = -1, offY = -1, offZ = -1, offInt = -1;
        byte dtX = 0, dtY = 0, dtZ = 0, dtInt = 0;
        IDLObjectSequence<PointField> fields = cloud.getFields();
        for (int i = 0; i < fields.size(); i++) {
            PointField f = fields.get(i);
            if (f.getCount() != 1) {
                continue;
            }
            switch (f.getNameAsString()) {
                case "x": offX = f.getOffset(); dtX = f.getDatatype(); break;
                case "y": offY = f.getOffset(); dtY = f.getDatatype(); break;
                case "z": offZ = f.getOffset(); dtZ = f.getDatatype(); break;
                case "intensity": offInt = f.getOffset(); dtInt = f.getDatatype(); break;
                default: break; // ring, time, ... — в step-режиме не нужны
            }
        }
        if (offX < 0 || offY < 0 || offZ < 0) {
            return List.of();
        }

        List<float[]> out = new ArrayList<>(n);
        for (int i = 0; i < n; i++) {
            int base = i * pointStep;
            float intensity = offInt < 0 ? 0f : readNum(buf, base + offInt, dtInt);
            out.add(new float[]{readNum(buf, base + offX, dtX), readNum(buf, base + offY, dtY),
                    readNum(buf, base + offZ, dtZ), intensity});
        }
        return out;
    }

    private static float readNum(ByteBuffer b, int off, byte dt) {
        switch (dt) {
            case PointField.FLOAT32: return b.getFloat(off);
            case PointField.FLOAT64: return (float) b.getDouble(off);
            case PointField.INT8: return b.get(off);
            case PointField.UINT8: return b.get(off) & 0xFF;
            case PointField.INT16: return b.getShort(off);
            case PointField.UINT16: return b.getShort(off) & 0xFFFF;
            case PointField.INT32: return b.getInt(off);
            case PointField.UINT32: return b.getInt(off);
            default: return 0f;
        }
    }

    // ==================== Сборка PointCloud2 (для /pointCloud) ====================

    /**
     * Собирает PointCloud2 (x, y, z, intensity, r, g, b — 7× FLOAT32, little-endian) из точек.
     * Точки {@code {x,y,z,intensity,r,g,b}} (цвет из {@link Vlp16Orchestrator#colorize});
     * если цвета нет (4 float), r/g/b пишутся 0.
     */
    private static PointCloud2 buildPointCloud(List<float[]> points, String frameId, Time stamp) {
        PointCloud2 cloud = new PointCloud2();

        ByteBuffer data = ByteBuffer.allocate(points.size() * POINT_STEP).order(ByteOrder.LITTLE_ENDIAN);
        for (float[] p : points) {
            data.putFloat(p[0]).putFloat(p[1]).putFloat(p[2]).putFloat(p[3])
                    .putFloat(p.length > 4 ? p[4] : 0f)
                    .putFloat(p.length > 5 ? p[5] : 0f)
                    .putFloat(p.length > 6 ? p[6] : 0f);
        }

        cloud.getFields().clear();
        cloud.getFields().add(pointField("x", 0));
        cloud.getFields().add(pointField("y", 4));
        cloud.getFields().add(pointField("z", 8));
        cloud.getFields().add(pointField("intensity", 12));
        cloud.getFields().add(pointField("r", 16));
        cloud.getFields().add(pointField("g", 20));
        cloud.getFields().add(pointField("b", 24));
        cloud.setHeight(1);
        cloud.setWidth(points.size());
        cloud.setIsBigendian(false);
        cloud.setPointStep(POINT_STEP);
        cloud.setRowStep(POINT_STEP * points.size());
        cloud.getData().clear();
        cloud.getData().addAll(data.array());
        cloud.setIsDense(true);

        cloud.getHeader().setFrameId(frameId);
        cloud.getHeader().getStamp().setSec(stamp.getSec());
        cloud.getHeader().getStamp().setNanosec(stamp.getNanosec());

        return cloud;
    }

    private static PointField pointField(String name, int offset) {
        PointField f = new PointField();
        f.setName(name);
        f.setOffset(offset);
        f.setDatatype(PointField.FLOAT32);
        f.setCount(1);
        return f;
    }
}
