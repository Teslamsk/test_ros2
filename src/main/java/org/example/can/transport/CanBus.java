package org.example.can.transport;

/**
 * Транспортный слой — абстракция над физической CAN-шиной.
 *
 * Реализации:
 *   SocketCanBus — Linux, AF_CAN raw socket + тонкий C-wrapper (~50 строк).
 *   PcanBus      — Windows/Linux, PCAN-Basic DLL от Peak-System через JNA.
 *   MockCanBus   — заглушка для JUnit-тестов.
 *
 * Mks42dController не зависит от вендора: принимает любой CanBus.
 */
public interface CanBus {

    /**
     * Открывает CAN-интерфейс.
     *
     * @param iface имя интерфейса ("/dev/can0"), канал ("PCAN_USB1"), и т.д.
     */
    void open(String iface);

    /**
     * Отправляет CAN-фрейм.
     *
     * @param cobId COB-ID (0x000 — 0x7FF стандартный, 0x18000000 — 0x1FFFFFFF расширенный).
     * @param data  полезная нагрузка (0–8 байт).
     */
    void send(int cobId, byte[] data);

    /**
     * Принимает CAN-фрейм с ожидаемым COB-ID (блокирующее чтение с таймаутом).
     *
     * @param cobId     ожидаемый COB-ID (для MKS 42D — адрес узла, напр. 0x01).
     * @param timeoutMs таймаут в миллисекундах.
     * @return byte[] (0–8 байт), или null если таймаут.
     */
    byte[] receive(int cobId, int timeoutMs);

    /**
     * Закрывает транспорт и освобождает ресурсы.
     */
    void close();
}
