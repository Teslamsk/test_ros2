package org.example.can;

import org.example.can.transport.CanBus;

/**
 * MKS SERVO42D_CAN — привід лидара по Z.
 *
 * <p>Привід НЕ является CANopen-слеймом: он говорит собственным протоколом
 * на обычных кадрах CAN 2.0A (11-битные стандартные ID, 500 kbit/s).
 *
 * <p>Кадр: {@code [op_code, params..., Check]}, где
 * {@code Check = (CanID + сумма всех байтов кадра) & 0xFF}.
 * CAN ID = адрес слэйва (01..10, по умолчанию 01). Все многобайтовые поля
 * big-endian. Ответ приходит от того же ID и начинается с того же op_code.
 *
 * <p>Спецификация: "MKS SERVO42&57D_CAN User Manual V1.0.5/1.0.6",
 * референс-реализация: github.com/DzymFardreamer/mks-servo-can.
 *
 * <p>Шкала оси: 1 оборот = 0x4000 = 16384 единицы энкодера (14-битный
 * абсолютный энкодер), т.е. 360/16384 = 0.0219727° на единицу.
 * Позиция — накопительная (multi-turn), читается командой 0x31.
 *
 * <p>Типичная инициализация (как в примерах производителя):
 * <pre>
 * F7  — аварийный стоп (чистим состояние)
 * 82 05 — рабочий режим SR_vFOC (без серийного режима движение по CAN не работает)
 * 84 10 — микрошаг 16
 * 83 xx xx — рабочий ток
 * 92  — текущее положение = ноль
 * F5  — движение к абсолютной оси
 * </pre>
 */
public class Mks42dController implements AutoCloseable {

    // ==================== Опкоды команд ====================

    public static final byte OP_READ_SYSTEM_PARAMS = 0x00;
    public static final byte OP_READ_ENCODER_CARRY = 0x30;   // ответ: [30, carry(4B), value(2B), crc]
    public static final byte OP_READ_ENCODER = 0x31;         // ответ: [31, pos(6B signed BE), crc]
    public static final byte OP_READ_SPEED = 0x32;           // ответ: [32, speed(2B signed), crc], RPM
    public static final byte OP_READ_PULSES = 0x33;          // ответ: [33, pulses(4B), crc]
    public static final byte OP_READ_ANGLE_ERROR = 0x39;     // ответ: [39, err(4B signed), crc]
    public static final byte OP_READ_EN_PINS = 0x3A;
    public static final byte OP_READ_ZERO_STATUS = 0x3B;
    public static final byte OP_RELEASE_PROTECTION = 0x3D;
    public static final byte OP_READ_PROTECTION = 0x3E;
    public static final byte OP_RESTORE_DEFAULTS = 0x3F;
    public static final byte OP_RESTART = 0x41;
    public static final byte OP_CALIBRATE = (byte) 0x80;
    public static final byte OP_WORK_MODE = (byte) 0x82;
    public static final byte OP_WORKING_CURRENT = (byte) 0x83;
    public static final byte OP_SUBDIVISIONS = (byte) 0x84;
    public static final byte OP_EN_PIN = (byte) 0x85;
    public static final byte OP_ROTATION_DIR = (byte) 0x86;
    public static final byte OP_ROTOR_PROTECTION = (byte) 0x88;
    public static final byte OP_CAN_BITRATE = (byte) 0x8A;   // 0=125k, 1=250k, 2=500k, 3=1M
    public static final byte OP_CAN_ID = (byte) 0x8B;
    public static final byte OP_RESPOND_ACTIVE = (byte) 0x8C;
    public static final byte OP_GROUP_ID = (byte) 0x8D;
    public static final byte OP_KEY_LOCK = (byte) 0x8F;
    public static final byte OP_SET_HOME = (byte) 0x90;
    public static final byte OP_GO_HOME = (byte) 0x91;
    public static final byte OP_ZERO_AXIS = (byte) 0x92;
    public static final byte OP_NO_LIMIT_HOME = (byte) 0x94;
    public static final byte OP_MODE0 = (byte) 0x9A;
    public static final byte OP_HOLDING_CURRENT = (byte) 0x9B;
    public static final byte OP_POSITION_ERROR = (byte) 0x9D;
    public static final byte OP_LIMIT_PORT_REMAP = (byte) 0x9E;
    public static final byte OP_READ_STATUS = (byte) 0xF1;   // ответ: [F1, status, crc]
    public static final byte OP_ENABLE = (byte) 0xF3;        // [F3, 00|01, crc]
    public static final byte OP_SPEED_MODE = (byte) 0xF6;
    public static final byte OP_EMERGENCY_STOP = (byte) 0xF7;
    public static final byte OP_REL_AXIS = (byte) 0xF4;      // [F4, spdHi(4b), spdLo, acc, rel(3B), crc]
    public static final byte OP_ABS_AXIS = (byte) 0xF5;      // [F5, spdHi(4b), spdLo, acc, abs(3B signed BE), crc]
    public static final byte OP_REL_PULSES = (byte) 0xFD;
    public static final byte OP_ABS_PULSES = (byte) 0xFE;
    public static final byte OP_SAVE_CLEAN = (byte) 0xFF;    // [FF, C8=save | CA=clean, crc]

    // ==================== Константы ====================

    public static final int DEFAULT_NODE_ID = 0x01;

    /** Единиц оси в одном обороте (14-битный энкодер). 360° = 16384. */
    public static final int AXIS_UNITS_PER_REV = 0x4000;

    public static final int SPEED_LIMIT_RPM = 3000;
    public static final int ACCEL_LIMIT = 255;

    /** Рабочие режимы (команда 0x82). */
    public static final int MODE_CR_OPEN = 0;   // шестерёнка, открытый контур
    public static final int MODE_CR_CLOSE = 1;  // шестерёнка, замкнутый — после установки CAN теряется!
    public static final int MODE_CR_VFOC = 2;   // шестерёнка, vFOC — после установки CAN теряется!
    public static final int MODE_SR_OPEN = 3;   // серийный, открытый
    public static final int MODE_SR_CLOSE = 4;  // серийный, замкнутый
    public static final int MODE_SR_VFOC = 5;   // серийный, vFOC — рекомендуемый для CAN positioning

    /** Статусы движения (команда 0xF1). */
    public static final int ST_STOP = 1;
    public static final int ST_SPEED_UP = 2;
    public static final int ST_SPEED_DOWN = 3;
    public static final int ST_FULL_SPEED = 4;
    public static final int ST_HOMING = 5;
    public static final int ST_CALIBRATING = 6;

    /** Статусы команд-движений (ответ на 0xF4/0xF5/0xFE/0xFD/0xF6). */
    public static final int RUN_FAIL = 0;
    public static final int RUN_STARTING = 1;
    public static final int RUN_COMPLETE = 2;
    public static final int RUN_END_LIMIT = 3;

    private static final int DEFAULT_RESPONSE_TIMEOUT_MS = 100;
    private static final int POLL_INTERVAL_MS = 10;
    /** Стартовые команды init: сколько раз повторять при «нет ответа». */
    private static final int INIT_RETRY_ATTEMPTS = 3;
    /** Пауза между повторами стартовой команды init, мс. */
    private static final int INIT_RETRY_DELAY_MS = 500;
    private static final int DEFAULT_MOVE_TIMEOUT_MS = 120000;
    private static final int HOME_TIMEOUT_MS = 20000;
    private static final int CALIBRATION_TIMEOUT_MS = 30000;
    /**
     * Дефолтные скорость/ускорение для больших ходов (мелкие шаги ≤10° идут
     * по профилю 1–3 rpm против "перебега" и откатов эластичного ремня).
     */
    private static final int DEFAULT_SPEED_RPM = 25;
    private static final int DEFAULT_ACCEL = 25;

    /**
     * Стабилизация рамы после остановки: столько опросов "остановлен" подряд
     * (каждый через SETTLE_POLL_MS — не менее ~100 мс непрерывного стопа)
     * требуется до начала сканирования.
     */
    private static final int SETTLE_STOP_SAMPLES = 6;
    private static final int SETTLE_POLL_MS = 20;

    // ==================== Транспорт ====================

    private final CanBus bus;
    private final long responseTimeoutMs;
    private int nodeId;
    private int maxSpeedRpm = DEFAULT_SPEED_RPM;
    private int maxAccel = DEFAULT_ACCEL;

    public Mks42dController(CanBus bus) {
        this(bus, DEFAULT_NODE_ID, DEFAULT_RESPONSE_TIMEOUT_MS);
    }

    public Mks42dController(CanBus bus, int nodeId) {
        this(bus, nodeId, DEFAULT_RESPONSE_TIMEOUT_MS);
    }

    public Mks42dController(CanBus bus, int nodeId, long responseTimeoutMs) {
        if (nodeId < 0x01 || nodeId > 0x7FF) {
            throw new IllegalArgumentException("MKS 42D: CAN ID должен быть 01..0x7FF, получен 0x" + Integer.toHexString(nodeId));
        }
        this.bus = bus;
        this.nodeId = nodeId;
        this.responseTimeoutMs = responseTimeoutMs;
    }

    // ==================== Низкоуровневый обмен ====================

    /**
     * Строит кадр: [op, params..., Check].
     */
    byte[] buildFrame(byte op, int... args) {
        byte[] payload = new byte[args.length + 2];
        payload[0] = op;
        int sum = nodeId + (op & 0xFF);
        for (int i = 0; i < args.length; i++) {
            payload[i + 1] = (byte) (args[i] & 0xFF);
            sum += args[i] & 0xFF;
        }
        payload[payload.length - 1] = (byte) (sum & 0xFF);
        return payload;
    }

    /**
     * Проверка Check байта входящего кадра.
     */
    private boolean checksumOk(byte[] data) {
        int sum = nodeId;
        for (int i = 0; i < data.length - 1; i++) {
            sum += data[i] & 0xFF;
        }
        return (sum & 0xFF) == (data[data.length - 1] & 0xFF);
    }

    /**
     * Отправляет команду и ждёт ответ с тем же op_code от нашего узла.
     * Кадры с битым Check или чужим op_code отбрасываются.
     *
     * @return payload ответа (включая op_code), или null по таймауту.
     *
     * <p>Synchronized: на шину ходят и главный поток, и фоновый трекер энкодера —
     * без лока чужой запрос/ответ мог бы "просочиться" в чужой цикл ожидания.
     */
    private synchronized byte[] transact(byte op, int... args) {
        bus.send(nodeId, buildFrame(op, args));
        long deadline = System.currentTimeMillis() + responseTimeoutMs;
        while (System.currentTimeMillis() < deadline) {
            byte[] data = bus.receive(nodeId, POLL_INTERVAL_MS);
            if (data != null && data.length >= 2
                    && (data[0] & 0xFF) == (op & 0xFF)
                    && checksumOk(data)) {
                return data;
            }
        }
        return null;
    }

    /**
     * Команда, ожидающая ответ [op, status, crc]. Бросает исключение при таймауте.
     *
     * @return статус-байт ответа.
     */
    private int writeRaw(byte op, int... args) {
        byte[] data = transact(op, args);
        if (data == null) {
            throw new IllegalStateException("MKS 42D: нет ответа на команду 0x" + toHex(op));
        }
        return data[1] & 0xFF;
    }

    /**
     * Команда, для которой успешным считается только status = 1.
     */
    private void writeOk(byte op, int... args) {
        int st = writeRaw(op, args);
        if (st != 1) {
            throw new IllegalStateException("MKS 42D: команда 0x" + toHex(op) + " отклонена (status=" + st + ")");
        }
    }

    // ==================== Инициализация ====================

    /**
     * Открывает шину и подготавливает привод к работе по CAN.
     *
     * Последовательность: AV-stop → режим SR_vFOC → enable → ноль в текущей позиции.
     */
    public void init(String iface) {
        bus.open(iface);
        System.out.println("[CAN] MKS 42D node 0x" + toHex(nodeId) + " init on " + iface);
        // Стартовые команды с ретраями: после включения/аварийного стопа привод
        // может пару сотен миллисек не отвечать на первые кадры.
        initCommand("emergencyStop", this::emergencyStop);
        initCommand("work mode", () -> setWorkMode(MODE_SR_VFOC));
        initCommand("enable", () -> enable(true));
        initCommand("zero", this::setZero);
        System.out.println("[CAN] MKS 42D node 0x" + toHex(nodeId) + " готов");
    }

    /**
     * Стартовая команда init с ретраями: если привод не ответил (таймаут),
     * повторяем INIT_RETRY_ATTEMPTS раз с паузой INIT_RETRY_DELAY_MS.
     */
    private void initCommand(String name, Runnable cmd) {
        for (int i = 0; ; i++) {
            try {
                cmd.run();
                return;
            } catch (IllegalStateException e) {
                if (i >= INIT_RETRY_ATTEMPTS - 1) {
                    throw e;
                }
                System.err.println("[CAN] " + name + ": нет ответа, повтор " + (i + 1) + "/" + INIT_RETRY_ATTEMPTS);
                sleep(INIT_RETRY_DELAY_MS);
            }
        }
    }

    /**
     * Опциональная настройка после init: микрошаг и рабочий ток.
     * Пропускает параметры <= 0 (оставляет значения привода).
     */
    public void configure(int subdivisions, int currentMa) {
        if (subdivisions > 0) {
            setSubdivisions(subdivisions);
        }
        if (currentMa > 0) {
            setWorkingCurrent(currentMa);
        }
    }

    // ==================== Управление движением ====================

    /**
     * Движение к абсолютному углу, затем ожидание остановки и стабилизации рамы.
     * Профиль скорости/ускорения — по дистанции от текущей позиции
     * (мелкие шаги медленнее: тяжёлая голова не должна тормозить "в упор"),
     * с ограничением maxSpeedRpm/maxAccel (tuneMotion).
     *
     * @return true, если мотор остановился и провисел без движения
     *         (SETTLE_STOP_SAMPLES опросов подряд) в пределах таймаута.
     */
    public boolean turnToAbsoluteAngle(double degrees) {
        double dist = Math.abs(degrees - getAngle());
        turnToAbsoluteAngle(degrees, Math.min(speedForDistance(dist), maxSpeedRpm),
                Math.min(accelForDistance(dist), maxAccel));
        return waitSettled(DEFAULT_MOVE_TIMEOUT_MS);
    }

    /**
     * Движение к углу без ожидания остановки.
     * Привод поддерживает real-time update: во время движения можно отправить
     * новую F5 (смена скорости/координаты).
     */
    public void turnToAbsoluteAngle(double degrees, int speedRpm, int accel) {
        long axis = degreesToAxis(degrees);
        runAxis(OP_ABS_AXIS, speedRpm, accel, axis);
        System.out.printf("[CAN] moveAbs %.2f deg -> axis %d (%d rpm, acc %d)%n", degrees, axis, speedRpm, accel);
    }

    /**
     * Профиль скорости по дистанции хода: мелкие шаги — минимальная скорость
     * и acc. Тяжёлая голова на подшипниках из 5 rpm в упор почти не
     * перепрыгивает; на 1 rpm позиционный контур удерживает её легко.
     */
    static int speedForDistance(double distDeg) {
        if (distDeg <= 1.0) {
            return 1;
        }
        if (distDeg <= 10.0) {
            return 3;
        }
        return DEFAULT_SPEED_RPM;
    }

    /**
     * Профиль ускорения по дистанции хода (парно со speedForDistance).
     */
    static int accelForDistance(double distDeg) {
        if (distDeg <= 1.0) {
            return 5;
        }
        if (distDeg <= 10.0) {
            return 8;
        }
        return DEFAULT_ACCEL;
    }

    /**
     * Относительное движение на угол (от текущей позиции), без ожидания.
     */
    public void turnRelativeAngle(double degrees, int speedRpm, int accel) {
        long axis = degreesToAxis(degrees);
        runAxis(OP_REL_AXIS, speedRpm, accel, axis);
        System.out.printf("[CAN] moveRel %.2f deg -> axis %d (%d rpm, acc %d)%n", degrees, axis, speedRpm, accel);
    }

    private void runAxis(byte op, int speedRpm, int accel, long axis) {
        if (speedRpm < 0 || speedRpm > SPEED_LIMIT_RPM) {
            throw new IllegalArgumentException("Speed должен быть 0.." + SPEED_LIMIT_RPM + ", получен " + speedRpm);
        }
        if (accel < 0 || accel > ACCEL_LIMIT) {
            throw new IllegalArgumentException("Accel должен быть 0.." + ACCEL_LIMIT + ", получен " + accel);
        }
        if (axis < -0x7FFFFF || axis > 0x7FFFFF) {
            throw new IllegalArgumentException("Axis вне 24-bit signed: " + axis);
        }
        int st = writeRaw(op,
                (speedRpm >> 8) & 0x0F,
                speedRpm & 0xFF,
                accel,
                (int) ((axis >> 16) & 0xFF),
                (int) ((axis >> 8) & 0xFF),
                (int) (axis & 0xFF));
        if (st == RUN_FAIL) {
            throw new IllegalStateException("MKS 42D: движение отклонено (status=0), op=0x" + toHex(op));
        }
    }

    /**
     * Сохранение/очистка параметров speed-режима (команда 0xFF): save = C8, clean = CA.
     */
    public void saveCleanSpeedMode(boolean save) {
        writeOk(OP_SAVE_CLEAN, save ? 0xC8 : 0xCA);
    }

    /**
     * Вращения с постоянной скоростью (режим 0xF6): dir — 0xCCW, 0x80|spdHi — CW.
     */
    public void speedMove(boolean clockwise, int speedRpm, int accel) {
        if (speedRpm < 0 || speedRpm > SPEED_LIMIT_RPM) {
            throw new IllegalArgumentException("Speed должен быть 0.." + SPEED_LIMIT_RPM + ", получен " + speedRpm);
        }
        if (accel < 0 || accel > ACCEL_LIMIT) {
            throw new IllegalArgumentException("Accel должен быть 0.." + ACCEL_LIMIT + ", получен " + accel);
        }
        int dirByte = (clockwise ? 0x80 : 0x00) | ((speedRpm >> 8) & 0x0F);
        writeRaw(OP_SPEED_MODE, dirByte, speedRpm & 0xFF, accel);
        System.out.printf("[CAN] speedMove %s %d rpm (acc %d)%n", clockwise ? "CW" : "CCW", speedRpm, accel);
    }

    /**
     * Остановка: скорость = 0 с заданным замедлением (режим 0xF6).
     */
    public void stop(int accel) {
        writeRaw(OP_SPEED_MODE, 0, 0, accel);
        System.out.println("[CAN] stop (acc " + accel + ")");
    }

    /**
     * Ждём, пока мотор остановится (статус 0xF1 = 1).
     *
     * @return true, если остановился в пределах таймаута.
     */
    public boolean waitIdle(long timeoutMs) {
        long deadline = System.currentTimeMillis() + timeoutMs;
        while (System.currentTimeMillis() < deadline) {
            if (isStopped()) {
                return true;
            }
            sleep(50);
        }
        boolean stopped = isStopped();
        if (!stopped) {
            System.err.println("[CAN] WARNING: мотор не остановился за " + timeoutMs + " ms");
        }
        return stopped;
    }

    /**
     * Ограничители профиля хода: скорость/ускорение не выше указанных.
     * Позволяют подогнать "мягкость" привода без пересборки
     * (флаги --rpm/--acc в MainOrchestrator). Значения <= 0 — не менять.
     */
    public void tuneMotion(int maxSpeedRpm, int maxAccel) {
        if (maxSpeedRpm > 0) {
            if (maxSpeedRpm > SPEED_LIMIT_RPM) {
                throw new IllegalArgumentException("Max speed должен быть 1.." + SPEED_LIMIT_RPM + ", получен " + maxSpeedRpm);
            }
            this.maxSpeedRpm = maxSpeedRpm;
        }
        if (maxAccel > 0) {
            if (maxAccel > ACCEL_LIMIT) {
                throw new IllegalArgumentException("Max accel должен быть 1.." + ACCEL_LIMIT + ", получен " + maxAccel);
            }
            this.maxAccel = maxAccel;
        }
        System.out.println("[CAN] motion limits: speed <= " + maxSpeedRpm + " rpm, accel <= " + maxAccel);
    }

    /**
     * Ждём остановки мотора и стабилизации рамы: SETTLE_STOP_SAMPLES опросов
     * "остановлен" подряд (каждый через SETTLE_POLL_MS — не менее ~100 мс
     * непрерывного стопа). Покачивание после торможения (эластичный ремень)
     * гасится до начала сканирования.
     *
     * @return true, если мотор остановился и стабилизировался в пределах таймаута.
     */
    public boolean waitSettled(long timeoutMs) {
        long deadline = System.currentTimeMillis() + timeoutMs;
        int consecutiveStops = 0;
        while (System.currentTimeMillis() < deadline) {
            if (isStopped()) {
                consecutiveStops++;
                if (consecutiveStops >= SETTLE_STOP_SAMPLES) {
                    return true;
                }
            } else {
                consecutiveStops = 0;
            }
            sleep(SETTLE_POLL_MS);
        }
        System.err.println("[CAN] WARNING: рама не стабилизировалась за " + timeoutMs + " ms");
        return false;
    }

    // ==================== Зануление / homing / калибровка ====================

    /**
     * Текущее положение = ноль (команда 0x92). Мотор не двигается.
     * Используется после ручной установки лидара в начальное положение.
     */
    public void setZero() {
        writeOk(OP_ZERO_AXIS);
        System.out.println("[CAN] zero: текущая позиция = 0");
    }

    /**
     * Возврат в "домашнюю" позицию (команда 0x91) с ожиданием завершения.
     * Параметры задаются командой 0x90 (setHomeParams).
     */
    public void home() {
        int st = writeRaw(OP_GO_HOME);
        if (st == RUN_FAIL) {
            throw new IllegalStateException("MKS 42D: go home отклонён (status=0)");
        }
        if (st == RUN_STARTING) {
            long deadline = System.currentTimeMillis() + HOME_TIMEOUT_MS;
            while (System.currentTimeMillis() < deadline && !isStopped()) {
                sleep(50);
            }
            if (!isStopped()) {
                throw new IllegalStateException("MKS 42D: таймаут homing (" + HOME_TIMEOUT_MS + " ms)");
            }
        }
        System.out.println("[CAN] home completed");
    }

    /**
     * Параметры home (команда 0x90, 6 байт данных):
     * уровень срабатывания концевика (0=Low, 1=High), направление (0=CW, 1=CCW),
     * скорость (0..3000 RPM), включение ограничения концом, режим:
     * hmMode = 0 — go home с концевиком, 1 — без концевика.
     */
    public void setHomeParams(int trigLevel, int direction, int speed, boolean endLimit, int hmMode) {
        if (speed < 0 || speed > SPEED_LIMIT_RPM) {
            throw new IllegalArgumentException("Home speed должен быть 0.." + SPEED_LIMIT_RPM + ", получен " + speed);
        }
        writeOk(OP_SET_HOME, trigLevel, direction, (speed >> 8) & 0x0F, speed & 0xFF, endLimit ? 1 : 0, hmMode);
    }

    /**
     * "noLimit" go home (команда 0x94): угол возврата (единицы оси: 0x4000 = 360°,
     * 0x2000 = 180° — дефолт) и ток при возврате, мА.
     */
    public void setNoLimitHome(int angleUnits, int currentMa) {
        writeOk(OP_NO_LIMIT_HOME,
                (angleUnits >> 24) & 0xFF, (angleUnits >> 16) & 0xFF,
                (angleUnits >> 8) & 0xFF, angleUnits & 0xFF,
                (currentMa >> 8) & 0xFF, currentMa & 0xFF);
    }

    /**
     * Защита по ошибке positions + En-триггер возврата в нуль (команда 0x9D):
     * Tim — в единицах ~15 мс, Errors — 28000 ≈ 360°.
     */
    public void setPositionErrorProtection(boolean enTriggerZero, boolean posErrorProtect, int timeUnits, int errorUnits) {
        int flags = (enTriggerZero ? 0x01 : 0x00) | (posErrorProtect ? 0x02 : 0x00);
        writeOk(OP_POSITION_ERROR, flags,
                (timeUnits >> 8) & 0xFF, timeUnits & 0xFF,
                (errorUnits >> 8) & 0xFF, errorUnits & 0xFF);
    }

    /**
     * Калибровка энкодера (команда 0x80) с ожиданием завершения.
     */
    public void calibrate() {
        int st = writeRaw(OP_CALIBRATE);
        if (st == 2) {
            throw new IllegalStateException("MKS 42D: калибровка завершена с ошибкой на старте");
        }
        long deadline = System.currentTimeMillis() + CALIBRATION_TIMEOUT_MS;
        while (System.currentTimeMillis() < deadline) {
            Integer status = statusOrNull();
            if (status == null || status != ST_CALIBRATING) {
                break;
            }
            sleep(100);
        }
        Integer status = statusOrNull();
        if (status != null && status == ST_CALIBRATING) {
            throw new IllegalStateException("MKS 42D: таймаут калибровки (" + CALIBRATION_TIMEOUT_MS + " ms)");
        }
        System.out.println("[CAN] calibration completed");
    }

    // ==================== Чтение состояния ====================

    /**
     * Статус движения (0xF1): 1=stopped, 2=ramp up, 3=ramp down, 4=full speed, 5=homing, 6=calibrating.
     *
     * @return статус, или null если привод не ответил.
     */
    public Integer statusOrNull() {
        byte[] data = transact(OP_READ_STATUS);
        if (data == null) {
            return null;
        }
        return data[1] & 0xFF;
    }

    /**
     * Мотор остановился (статус = 1). Если нет ответа — считаем, что ещё крутится.
     */
    public boolean isStopped() {
        Integer st = statusOrNull();
        return st != null && st == ST_STOP;
    }

    /**
     * Позиция энкодера в единицах оси (команда 0x31), 1 оборот = 0x4000 = 16384.
     * Значение накопительное: может выходить за пределы одного оборота.
     */
    public long readEncoder() {
        byte[] data = transact(OP_READ_ENCODER);
        if (data == null || data.length < 8) {
            throw new IllegalStateException("MKS 42D: нет ответа на чтение позиции (0x31)");
        }
        long v = 0;
        for (int i = 1; i <= 6; i++) {
            v = (v << 8) | (data[i] & 0xFF);
        }
        if ((v & (1L << 47)) != 0) {
            v -= (1L << 48);
        }
        return v;
    }

    /**
     * Текущий угол в градусах (от текущего нуля). Может превышать 360° при multi-turn.
     */
    public double getAngle() {
        return readEncoder() * 360.0 / AXIS_UNITS_PER_REV;
    }

    /**
     * Текущая скорость, RPM (команда 0x32): CCW — положительная, CW — отрицательная.
     */
    public int readSpeed() {
        byte[] data = transact(OP_READ_SPEED);
        if (data == null || data.length < 4) {
            throw new IllegalStateException("MKS 42D: нет ответа на чтение скорости (0x32)");
        }
        return (short) (((data[1] & 0xFF) << 8) | (data[2] & 0xFF));
    }

    /**
     * Чтение системного параметра (команда 0x00): [00, code, crc], напр. code = 0x82 (work mode),
     * 0x83 (current), 0x84 (mstep), 0x8B (can id).
     * Ответ: [code, params..., crc], формат params — как при установке;
     * если параметр не поддерживается: [code, FF, FF, crc].
     *
     * @return байты params, или null если нет ответа / параметр не читается.
     */
    public byte[] readSystemParameter(int code) {
        return readSystemParameter(code, responseTimeoutMs);
    }

    public synchronized byte[] readSystemParameter(int code, long timeoutMs) {
        // Ответ на 0x00 начинается с КОДА ПАРАМЕТРА, а не с op 0x00 — поэтому
        // transact() (ищет data[0] == op) тут годится только через собственный цикл.
        // Synchronized — как у transact(): один обмен на шину за раз.
        bus.send(nodeId, buildFrame(OP_READ_SYSTEM_PARAMS, code & 0xFF));
        long deadline = System.currentTimeMillis() + timeoutMs;
        while (System.currentTimeMillis() < deadline) {
            byte[] data = bus.receive(nodeId, POLL_INTERVAL_MS);
            if (data != null && data.length >= 3
                    && (data[0] & 0xFF) == (code & 0xFF)
                    && checksumOk(data)) {
                if (data.length == 4 && (data[1] & 0xFF) == 0xFF && (data[2] & 0xFF) == 0xFF) {
                    return null; // [code, FF, FF, crc] — параметр не поддерживается
                }
                byte[] params = new byte[data.length - 2];
                System.arraycopy(data, 1, params, 0, params.length);
                return params;
            }
        }
        return null;
    }

    // ==================== Конфигурация привода ====================

    /**
     * Рабочий режим (0x82). Реже 3 (CR_*) — шестерёнчатый,
     * после установки привод может перестать отвечать по CAN!
     * Для управления по CAN использовать 3, 4 или 5.
     */
    public void setWorkMode(int mode) {
        if (mode < MODE_CR_OPEN || mode > MODE_SR_VFOC) {
            throw new IllegalArgumentException("Work mode должен быть 0..5, получен " + mode);
        }
        if (mode < MODE_SR_OPEN) {
            System.err.println("[CAN] WARNING: режим " + mode + " (stepper CR_*) — после установки CAN может стать недоступен");
        }
        writeOk(OP_WORK_MODE, mode);
        System.out.println("[CAN] work mode = " + mode);
    }

    /**
     * Микрошаг (0x84), например 16.
     */
    public void setSubdivisions(int subdivisions) {
        writeOk(OP_SUBDIVISIONS, subdivisions);
        System.out.println("[CAN] subdivisions = " + subdivisions);
    }

    /**
     * Рабочий ток, мА (0x83). Для 42D максимум 3000.
     */
    public void setWorkingCurrent(int currentMa) {
        if (currentMa < 0 || currentMa > 5200) {
            throw new IllegalArgumentException("Current должен быть 0..5200 мА, получен " + currentMa);
        }
        writeOk(OP_WORKING_CURRENT, (currentMa >> 8) & 0xFF, currentMa & 0xFF);
        System.out.println("[CAN] working current = " + currentMa + " mA");
    }

    /**
     * Удерживающий ток, мА (0x9B) — ток, которым ротор удерживается на месте в останове.
     * Ключевой параметр против проскальзывания/автоколебаний тяжёлой рамы при включении.
     */
    public void setHoldingCurrent(int currentMa) {
        if (currentMa < 0 || currentMa > 5200) {
            throw new IllegalArgumentException("Holding current должен быть 0..5200 мА, получен " + currentMa);
        }
        writeOk(OP_HOLDING_CURRENT, (currentMa >> 8) & 0xFF, currentMa & 0xFF);
        System.out.println("[CAN] holding current = " + currentMa + " mA");
    }

    /**
     * Защита от заклинивания ротора (0x88).
     */
    public void setRotorProtection(boolean enable) {
        writeOk(OP_ROTOR_PROTECTION, enable ? 1 : 0);
    }

    /**
     * Снятие защиты обёртки ротора (0x3D).
     *
     * @return true если сняли успешно.
     */
    public boolean releaseProtection() {
        return writeRaw(OP_RELEASE_PROTECTION) == 1;
    }

    /**
     * Настройки pin EN (0x85): 0=ActiveLow, 1=ActiveHigh, 2=ActiveAlways.
     */
    public void setEnPinConfig(int config) {
        writeOk(OP_EN_PIN, config);
    }

    /**
     * Направление вращения (0x86): 0=CW, 1=CCW. Только для pulse-интерфейса;
     * для serial направление задаётся знаком в командах.
     */
    public void setRotationDirection(int direction) {
        writeOk(OP_ROTATION_DIR, direction);
    }

    /**
     * Response/Active (0x8C): отвечает ли слэйв на команды и генерирует ли активно.
     */
    public void setRespondActive(boolean respond, boolean active) {
        writeOk(OP_RESPOND_ACTIVE, respond ? 1 : 0, active ? 1 : 0);
    }

    /**
     * Скорость CAN (0x8A): 0=125k, 1=250k, 2=500k, 3=1M.
     */
    public void setCanBitrate(int code) {
        writeOk(OP_CAN_BITRATE, code);
    }

    /**
     * Смена CAN ID привода (0x8B: hi&0x0F, lo). После успешного ответа контроллер
     * продолжает пользоваться новым адресом.
     */
    public void setCanId(int newId) {
        if (newId < 0x01 || newId > 0x7FF) {
            throw new IllegalArgumentException("MKS 42D: CAN ID должен быть 01..0x7FF (0 — broadcast), получен 0x" + Integer.toHexString(newId));
        }
        writeOk(OP_CAN_ID, (newId >> 8) & 0x0F, newId & 0xFF);
        this.nodeId = newId;
        System.out.println("[CAN] CAN ID изменён на 0x" + toHex(newId));
    }

    /**
     * Mode 0 — автоматический возврат в 0-точку при включении питания (0x9A).
     */
    public void setMode0(int mode, boolean enable, int speed, int direction) {
        writeOk(OP_MODE0, mode, enable ? 1 : 0, speed, direction);
    }

    // ==================== Питание и аварийные команды ====================

    /**
     * Enable/disable обмоток (0xF3).
     */
    public void enable(boolean on) {
        writeOk(OP_ENABLE, on ? 1 : 0);
        System.out.println("[CAN] motor " + (on ? "enabled" : "disabled"));
    }

    /**
     * Аварийная остановка (0xF7).
     */
    public void emergencyStop() {
        writeRaw(OP_EMERGENCY_STOP);
        System.out.println("[CAN] EMERGENCY STOP");
    }

    /**
     * Перезапуск привода (0x41).
     */
    public void restartMotor() {
        writeRaw(OP_RESTART);
        System.out.println("[CAN] motor restart");
    }

    /**
     * Сброс параметров к заводским (0x3F). После сброса привод перезагружается
     * и требует калибровки.
     */
    public void restoreDefaults() {
        writeOk(OP_RESTORE_DEFAULTS);
    }

    // ==================== Вспомогательное ====================

    /**
     * Градусы → единицы оси (1 оборот = 16384).
     */
    public static long degreesToAxis(double degrees) {
        return Math.round(degrees * AXIS_UNITS_PER_REV / 360.0);
    }

    /**
     * Единицы оси → градусы.
     */
    public static double axisToDegrees(long axis) {
        return axis * 360.0 / AXIS_UNITS_PER_REV;
    }

    private static String toHex(int v) {
        return String.format("%02X", v & 0xFF);
    }

    private static void sleep(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    @Override
    public void close() {
        try {
            emergencyStop();
        } catch (Exception ignored) {
            // привод мог уже не отвечать — закрываем шину так или иначе
        }
        bus.close();
    }
}
