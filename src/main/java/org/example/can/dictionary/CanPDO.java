package org.example.can.dictionary;

/**
 * PDO (Process Data Object) addressing in CANopen.
 *
 * PDO — это быстрые кадры для обмена реальным временем (позиция, момент,
 * статус слов и т.д.). В отличие от SDO, PDO не требуют запроса-ответа:
 * мастер просто "кидает" фрейм по CAN ID, и слейв его ловит.
 *
 * Адресация в CANopen:
 *   CAN ID = база_смещения + nodeID
 *
 * Где `baза_смещения` определяется стандартом CiA 402:
 *   RPDO1 = 0x160 — приём #1  (master → slave)
 *   RPDO2 = 0x170 — приём #2  (master → slave)
 *   TPDO1 = 0x180 — передача #1 (slave → master)
 *   TPDO2 = 0x190 — передача #2 (slave → master)
 *
 * Пример использования:
 *   CanPDO.RPDO1.master2slave(1);  // 0x161 — отправляем двигателю #1
 *   CanPDO.TPDO1.slave2master(1);  // 0x181 — слушаем ответ от двигателя #1
 */

public enum CanPDO {

    RPDO1(0x160, "Receive PDO #1  (master → slave)"),
    RPDO2(0x170, "Receive PDO #2  (master → slave)"),
    TPDO1(0x180, "Transmit PDO #1 (slave → master)"),
    TPDO2(0x190, "Transmit PDO #2 (slave → master)");

    private final int baseId;
    private final String description;

    CanPDO(int baseId, String description) {
        this.baseId = baseId;
        this.description = description;
    }

    /**
     * CAN ID для передачи данных от мастера к слейву.
     * Используется для RPDO (master отправляет, slave принимает).
     */
    public int master2slave(int nodeId) {
        return baseId + nodeId;
    }

    /**
     * CAN ID для приёма данных от слейва мастером.
     * Используется для TPDO (slave отправляет, master принимает).
     */
    public int slave2master(int nodeId) {
        return baseId + nodeId;
    }

    public String getDescription() { return description; }

    @Override
    public String toString() {
        return name() + " (base 0x" + Integer.toHexString(baseId).toUpperCase() + ")";
    }
}
