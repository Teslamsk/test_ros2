package org.example.can;

import org.example.can.dictionary.*;
import org.example.can.transport.CanBus;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.stream.Collectors;

/**
 * CanController — управление шаговым двигателем лида́ра через CANopen.
 * <p>
 * Что делает этот класс:
 * 1. общается с двигателем по шине CAN (отправляет команды, читает ответы).
 * 2. переводит понятные команды ("поверни на 30°", "отпусти руку")
 * в низкоуровневые CANopen-фреймы (SDO / PDO).
 * 3. читает состояние двигателя из самого двигателя.
 * <p>
 * Состояние хранится у привода, а не в переменных: мы запрашиваем
 * Status Word, Actual Position и т.д. напрямую из Object Dictionary.
 * <p>
 * Типичный рабочий цикл:
 * <pre>
 * controller.init("/dev/can0");
 * controller.setHandMode(Node.ROTATE_LIDAR_Z);
 * controller.setZero(Node.ROTATE_LIDAR_Z);
 * controller.setMode(Node.ROTATE_LIDAR_Z, MotorMode.LOCKED);
 * controller.turnToAbsoluteAngle(Node.ROTATE_LIDAR_Z, 45f);
 * controller.emergencyStop(Node.ROTATE_LIDAR_Z);
 * </pre>
 */
public class CanController implements AutoCloseable {

    public enum MotorMode {
        HAND("Двигатель отпущен — можно крутить рукой"),
        RELAXED("Мягкое удержание — можно повернуть усилием"),
        LOCKED("Жесткая фиксация — для точного сканирования");

        final String description;

        MotorMode(String desc) {
            this.description = desc;
        }
    }

    // ==================== Параметры режимов ====================

    private static final float RELAXED_P_GAIN = 30.0f;
    private static final float RELAXED_MAX_TORQUE = 100.0f;
    private static final float LOCKED_P_GAIN = 300.0f;
    private static final float LOCKED_MAX_TORQUE = 1000.0f;

    // ==================== CAN transport ====================

    private final CanBus bus;

    // ==================== Состояние (ключ — Node) ====================

    /**
     * Текущий "режим жесткости" для каждого мотора. ConcurrentHashMap — изменчива и потокобезопасна.
     */
    private final ConcurrentHashMap<Node, MotorMode> currentModeMap = new ConcurrentHashMap<>();

    /**
     * Целевая позиция каждого двигателя (в тиках). ConcurrentHashMap для потокобезопасности.
     */
    private final ConcurrentHashMap<Node, AtomicInteger> targetPosMap = new ConcurrentHashMap<>();

    // ==================== Конструктор ====================

    /**
     * Конструктор — принимает готовый транспортный слой.
     *
     * @param bus реализация CanBus (SocketCanBus, PcanBus, MockCanBus).
     */
    public CanController(CanBus bus) {
        this.bus = bus;
        Arrays
                .stream(Node.values())
                .forEach(node -> initMotor(node, MotorMode.LOCKED));
    }

    /**
     * Инициализирует новый мотор в системе.
     *
     * @param node Node мотора.
     * @param mode начальный режим жесткости.
     */
    private void initMotor(Node node, MotorMode mode) {
        currentModeMap.put(node, mode);
        targetPosMap.put(node, new AtomicInteger(0));
    }

    // ==================== Инициализация ====================

    /**
     * Инициализация с CAN-интерфейсом.
     *
     * @param canInterface имя CAN-интерфейса (например, "/dev/can0").
     */
    public void init(String canInterface) {
        assert canInterface != null && !canInterface.isBlank() : "canInterface не может быть пустым";
        System.out.println("[CAN] Initializing on " + canInterface + "...");

        bus.open(canInterface);
        Arrays.stream(Node.values()).forEach(node -> {
            System.out.printf("[CAN] Node %d starting initialization.%n", node.getNodeId());
            setMode(node, MotorMode.LOCKED);
            enableMotor(node);
            waitStatus(node, StatusWord.OPERATION_ENABLED, 1000);
            System.out.printf("[CAN] Node %d initialized.%n", node.getNodeId());
        });
        System.out.println("[CAN] Succeed nitialized on " + canInterface);

    }

    // ==================== CANopen SDO / PDO ====================

    /**
     * SDO-запись (Service Data Object — медленный канал настроек).
     * Один запрос = один ответ. Подходит для параметров.
     */
    private void sdoWrite(Node node, CanOD entry, int value) {
        sdoWrite(node, entry.getIndex(), entry.getSubIndex(), value);
    }

    /**
     * SDO-запись float-значения (32-bit IEEE 754 → 4 байта, Little-Endian).
     */
    private void sdoWriteFloat(Node node, CanOD entry, float value) {
        sdoWrite(node, entry, Float.floatToIntBits(value));
    }

    /**
     * SDO-чтение (SDO upload — запрашиваем значение у привода).
     * <p>
     * Формат SDO-запроса (expedited upload):
     * COB-ID: 0x600 + NodeID
     * Byte 0: 0x40 — команда "прочти мне 4 байта"
     * Byte 1-2: индекс (LE)
     * Byte 3: суб-индекс
     * Byte 4-7: 0 (не используется)
     * <p>
     * Ответ (приходит по COB-ID 0x580 + NodeID):
     * Byte 0: 0x43 — ответ "вот тебе 4 байта" (expedited upload, CiA 301)
     * Byte 4-7: данные (LE)
     */
    private int sdoRead(Node node, int index, int subIndex) {
        byte[] request = new byte[8];
        request[0] = 0x40;  // SDO upload expedited
        request[1] = (byte) (index & 0xFF);
        request[2] = (byte) ((index >> 8) & 0xFF);
        request[3] = (byte) subIndex;

        bus.send(0x600 + node.getNodeId(), request);
        System.out.printf("[CAN] SDO read 0x%04X.%02X from node %d%n", index, subIndex, node.getNodeId());

        byte[] response = bus.receive(0x580 + node.getNodeId(), 1000);
        if (response != null && response[0] == 0x43) {
            ByteBuffer bb = ByteBuffer.wrap(response).order(ByteOrder.LITTLE_ENDIAN);
            bb.position(4);
            int data = bb.getInt();
            System.out.printf("[CAN] SDO response = 0x%08X%n", data);
            return data;
        }
        System.out.printf("[CAN] WARNING: SDO response timeout for node %d%n", node.getNodeId());
        return 0;
    }

    /**
     * Считывает значение из Object Dictionary.
     */
    private int sdoRead(Node node, CanOD entry) {
        return sdoRead(node, entry.getIndex(), entry.getSubIndex());
    }

    /**
     * Низкоуровневая SDO-запись.
     * <p>
     * Формат SDO-кадра (expedited download):
     * Byte 0:  0x2B — "запиши 4 байта"
     * Byte 1-2: индекс (Low, High) — адрес регистра
     * Byte 3:  суб-индекс
     * Byte 4-7: данные (Little-Endian)
     */
    private void sdoWrite(Node node, int index, int subIndex, int value) {
        byte[] frame = new byte[8];
        frame[0] = 0x2B;
        frame[1] = (byte) (index & 0xFF);
        frame[2] = (byte) ((index >> 8) & 0xFF);
        frame[3] = (byte) subIndex;
        frame[4] = (byte) (value & 0xFF);
        frame[5] = (byte) ((value >> 8) & 0xFF);
        frame[6] = (byte) ((value >> 16) & 0xFF);
        frame[7] = (byte) ((value >> 24) & 0xFF);

        bus.send(0x600 + node.getNodeId(), frame);
        System.out.printf("[CAN] SDO write %-29s = 0x%08X%n", CanOD.toHex(index, subIndex), value);
    }

    /**
     * PDO-запись целевой позиции (быстрый канал, real-time).
     * <p>
     * Формат (8 байт, Little-Endian):
     * Byte 0-1: Control Word (команда: включение/стоп)
     * Byte 2-3: (запас)
     * Byte 4-7: Target Position (в тиках)
     */
    private void pdoWritePosition(Node node, int position) {
        byte[] frame = new byte[8];
        ByteBuffer bb = ByteBuffer.wrap(frame).order(ByteOrder.LITTLE_ENDIAN);
        bb.putShort(0, (short) ControlWord.ENABLE_FULL.getValue());
        bb.putShort(2, (short) 0);
        bb.putInt(4, position);

        bus.send(CanPDO.RPDO1.master2slave(node.getNodeId()), frame);
    }

    /**
     * PDO-запись целевого крутящего момента (режим TORQUE).
     * Формат тот же, но Byte 4-7 — Target Torque.
     */
    private void pdoWriteTorque(Node node, int torque) {
        byte[] frame = new byte[8];
        ByteBuffer bb = ByteBuffer.wrap(frame).order(ByteOrder.LITTLE_ENDIAN);
        bb.putShort(0, (short) ControlWord.ENABLE_FULL.getValue());
        bb.putShort(2, (short) 0);
        bb.putInt(4, torque);

        bus.send(CanPDO.RPDO1.master2slave(node.getNodeId()), frame);
    }

    /**
     * Проверяет, что двигатель включен (Operation Enabled).
     * Если нет — бросает RuntimeException. Используется в начале публичных методов.
     */
    private void assertEnabled(Node node) {
        if (!hasStatus(node, StatusWord.OPERATION_ENABLED)) {
            throw new IllegalStateException("Node " + node + " is not Operation Enabled. Status: " + getStatusByCode(node));
        }
    }

    // ==================== Публичный API ====================

    /**
     * Переключает режим жесткости двигателя.
     * <p>
     * HAND → Torque mode, moment = 0 — не сопротивляется.
     * RELAXED → Position mode, P-Gain = 30, Max Torque = 100 мН·м.
     * LOCKED → Position mode, P-Gain = 300, Max Torque = 1000 мН·м.
     */
    public void setMode(Node node, MotorMode mode) {
        if (mode == currentModeMap.get(node)) return;
        System.out.printf("[CAN] Mode %s -> %s: %s%n", currentModeMap.get(node), mode, mode.description);

        switch (mode) {
            case HAND:
                sdoWrite(node, CanOD.MODE_OF_OPERATION, OperationMode.TORQUE.getValue());
                sdoWrite(node, CanOD.MAX_TORQUE, 0);
                sdoWrite(node, CanOD.TARGET_TORQUE, 0);
                pdoWriteTorque(node, 0);
                break;

            case RELAXED:
                sdoWrite(node, CanOD.MODE_OF_OPERATION, OperationMode.PROFILED_POSITION.getValue());
                sdoWriteFloat(node, CanOD.P_GAIN, RELAXED_P_GAIN);
                sdoWrite(node, CanOD.MAX_TORQUE, Math.round(RELAXED_MAX_TORQUE));
                pdoWritePosition(node, targetPosMap.get(node).get());
                break;

            case LOCKED:
                sdoWrite(node, CanOD.MODE_OF_OPERATION, OperationMode.PROFILED_POSITION.getValue());
                sdoWriteFloat(node, CanOD.P_GAIN, LOCKED_P_GAIN);
                sdoWrite(node, CanOD.MAX_TORQUE, Math.round(LOCKED_MAX_TORQUE));
                pdoWritePosition(node, targetPosMap.get(node).get());
                break;
        }

        currentModeMap.put(node, mode);
    }

    /**
     * Быстрый перевод в ручной режим (обёртка для setMode(HAND)).
     */
    public void setHandMode(Node node) {
        setMode(node, MotorMode.HAND);
        System.out.println("[CAN] Hand mode: crank by hand freely.");
    }

    /**
     * Включает двигатель (подает напряжение, активирует рабочий режим).
     * <p>
     * Последовательность (стандарт CiA 402):
     * 1. SHUTDOWN (0x0006) → "Ready to Switch On".
     * 2. Ждём подтверждения (читаем Status Word).
     * 3. ENABLE (0x0007) → "Operation Enabled".
     * 4. Ждём подтверждения (читаем Status Word).
     */
    public void enableMotor(Node node) {
        System.out.printf("[CAN] Enabling motor node %d...%n", node.getNodeId());
        sdoWrite(node, CanOD.CONTROL_WORD, ControlWord.SHUTDOWN.getValue());
        waitStatus(node, StatusWord.READY_TO_SWITCH_ON, 500);
        sdoWrite(node, CanOD.CONTROL_WORD, ControlWord.ENABLE.getValue());
        waitStatus(node, StatusWord.OPERATION_ENABLED, 1000);
        System.out.printf("[CAN] Motor node %d enabled.%n", node.getNodeId());
    }

    /**
     * Отключает двигатель (снимает напряжение с обмоток).
     * <p>
     * Последовательность:
     * 1. QUICK_STOP (0x0080) → экстренно останавливаем.
     * 2. Ждём подтверждения.
     * 3. DISABLE (0x0000) → Switch On Disabled.
     */
    public void disconnectMotor(Node node) {
        System.out.printf("[CAN] Disconnecting motor node %d...%n", node.getNodeId());
        sdoWrite(node, CanOD.CONTROL_WORD, ControlWord.QUICK_STOP.getValue());
        waitStatus(node, StatusWord.FAST_STOPPED, 500);
        sdoWrite(node, CanOD.CONTROL_WORD, ControlWord.DISABLE.getValue());
        System.out.printf("[CAN] Motor node %d disconnected — voltage off.%n", node.getNodeId());
    }

    /**
     * Устанавливает текущее положение валика как "ноль".
     * <p>
     * Не двигает двигатель — просто "перезагружает" координаты.
     * Используется после ручной установки лида́ра в начальное положение.
     */
    public void setZero(Node node) {
        System.out.printf("[CAN] Setting zero for node %d.%n", node.getNodeId());
        targetPosMap.get(node).set(0);
        sdoWrite(node, CanOD.CONTROL_WORD, ControlWord.HOME.getValue());
        sdoWrite(node, CanOD.TARGET_POSITION, 0);
        pdoWritePosition(node, 0);
    }

    /**
     * Поворачивает двигатель к указанному углу (относительно нуля).
     */
    public void turnToAbsoluteAngle(Node node, float degrees) {
        int ticks = degreesToTicks(degrees);
        targetPosMap.get(node).set(ticks);
        pdoWritePosition(node, ticks);
        System.out.printf("[CAN] turnToAbs %.1f deg (node=%d, pos=%d ticks)%n", degrees, node.getNodeId(), ticks);
    }

    /**
     * Поворачивает двигатель на указанное смещение (относительно текущей позиции).
     */
    public void turnRelative(Node node, float degrees) {
        int delta = degreesToTicks(degrees);
        targetPosMap.get(node).addAndGet(delta);
        pdoWritePosition(node, targetPosMap.get(node).get());
        System.out.printf("[CAN] turnRel %.1f deg (node=%d, pos=%d ticks)%n", degrees, node.getNodeId(), targetPosMap.get(node).get());
    }

    /**
     * Поворачивает двигатель к указанному углу (синоним turnToAbsoluteAngle).
     */
    public void setAbsolutePosition(Node node, float degrees) {
        int ticks = degreesToTicks(degrees);
        targetPosMap.get(node).set(ticks);
        pdoWritePosition(node, ticks);
        System.out.printf("[CAN] setAbs %.1f deg (node=%d, pos=%d ticks)%n", degrees, node.getNodeId(), ticks);
    }

    /**
     * Аварийная остановка. Двигатель останавливается по максимальному замедлению.
     */
    public void emergencyStop(Node node) {
        System.out.printf("[CAN] EMERGENCY STOP node %d!%n", node.getNodeId());
        sdoWrite(node, CanOD.CONTROL_WORD, ControlWord.QUICK_STOP.getValue());
    }

    // ==================== Обратная связь ====================

    /**
     * Считывает Status Word (0x6041) для конкретного мотора.
     */
    public int getStatusWord(Node node) {
        return sdoRead(node, CanOD.STATUS_WORD.getIndex(), CanOD.STATUS_WORD.getSubIndex());
    }

    /**
     * Возвращает все установленные в Status Word флаги для мотора.
     */
    public Set<StatusWord> getStatusByCode(Node node) {
        int sw = getStatusWord(node);
        return Arrays.stream(StatusWord.values())
                .filter(status -> status.isSet(sw))
                .collect(Collectors.toSet());
    }

    /**
     * Проверяет, установлен ли конкретный статус.
     */
    public boolean hasStatus(Node node, StatusWord status) {
        return status.isSet(getStatusWord(node));
    }

    /**
     * Проверяет, что хотя бы один из заданных статусов активен.
     */
    public boolean hasAnyStatus(Node node, Set<StatusWord> statuses) {
        return statuses.stream().anyMatch(s -> hasStatus(node, s));
    }

    /**
     * Ждём, пока привод доберётся до целевой позиции (Target Reached, бит 13).
     *
     * @return true, если позиция достигнута в пределах таймаута.
     */
    public boolean awaitTargetReached(Node node, int timeoutMs) {
        return waitStatus(node, StatusWord.TARGET_REACHED, timeoutMs);
    }

    /**
     * Ждём подтверждение состояния.
     *
     * @param node      мотор.
     * @param status    ожидаемый флаг (Ready, Enabled, FastStopped...).
     * @param timeoutMs таймаут в миллисекундах.
     * @return true, если статус подтвердился.
     */
    private boolean waitStatus(Node node, StatusWord status, int timeoutMs) {
        long deadline = System.currentTimeMillis() + timeoutMs;
        while (System.currentTimeMillis() < deadline) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException ignored) {
            }
            if (hasStatus(node, status)) return true;
        }
        System.out.printf("[CAN] WARNING: node %d %s not confirmed (timeout %dms)%n", node.getNodeId(), status.name(), timeoutMs);
        return false;
    }

    /**
     * Ждём подтверждение любого из статусов.
     */
    private boolean waitAnyStatus(Node node, Set<StatusWord> statuses, int timeoutMs) {
        long deadline = System.currentTimeMillis() + timeoutMs;
        while (System.currentTimeMillis() < deadline) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException ignored) {
            }
            if (hasAnyStatus(node, statuses)) return true;
        }
        System.out.printf("[CAN] WARNING: node %d none of %s confirmed (timeout %dms)%n", node.getNodeId(), statuses, timeoutMs);
        return false;
    }

    // ==================== Внутренние методы ====================

    /**
     * Перевод "градусы → тики". 400 taps/rev → 0.9° на тик. Зависит от драйвера шаговика.
     */
    private int degreesToTicks(float degrees) {
        int ticksPerRev = 400;
        return Math.round(degrees * ticksPerRev / 360.0f);
    }

    /**
     * Обратный перевод "тики → градусы".
     */
    private float ticksToDegrees(int ticks) {
        int ticksPerRev = 400;
        return ticks * 360.0f / ticksPerRev;
    }

    // ==================== Getters ====================

    /**
     * Возвращает режим жесткости.
     */
    public MotorMode getMode(Node node) {
        return currentModeMap.get(node);
    }

    /**
     * Возвращает текущую целевую позицию в тиках.
     */
    public int getTargetPosition(Node node) {
        return targetPosMap.get(node).get();
    }

    @Override
    public void close() {
        System.out.println("[CAN] Shutting down...");
        for (var node : Node.values()) {
            emergencyStop(node);
        }
        bus.close();
    }
}
