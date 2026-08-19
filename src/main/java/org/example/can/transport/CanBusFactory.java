package org.example.can.transport;

/**
 * Фабрика транспорта: подбирает CAN-транспорт под текущую ОС.
 *
 * Windows  → PcanBus (PCAN-Basic DLL, ПКМ-адаптер Peak-System).
 * Linux    → SocketCanBus (AF_CAN raw socket + libcanwrapper.so).
 * Остальное (включая macOS, где нет AF_CAN) — UnsupportedOperationException.
 *
 * Используется в композит-рооте (MainOrchestrator): решение "какой транспорт"
 * принимается в одном месте, а не в каждом вызывающем коде.
 */
public final class CanBusFactory {

    private CanBusFactory() {
    }

    /**
     * Создаёт транспорт для текущей ОС.
     *
     * @throws UnsupportedOperationException если ОС не поддерживается.
     */
    public static CanBus forCurrentOS() {
        String os = System.getProperty("os.name", "unknown").toLowerCase();
        if (os.contains("win")) {
            return new PcanBus(PcanBus.PCAN_USB1);
        }
        if (os.contains("linux")) {
            return new SocketCanBus();
        }
        throw new UnsupportedOperationException("CAN transport is not available on OS: " + os);
    }
}
