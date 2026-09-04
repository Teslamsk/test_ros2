package org.example.can;

import org.example.can.transport.MockCanBus;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Тесты MKS 42D протокола по MockCanBus: сборка кадров (Check = (ID + сумма) & 0xFF),
 * разбор ответов, таймауты.
 */
class Mks42dControllerTest {

    /**
     * Собирает "ответный" кадр от узла nodeId: [op, rest..., crc].
     */
    private static byte[] resp(int nodeId, int op, int... rest) {
        byte[] payload = new byte[rest.length + 2];
        payload[0] = (byte) op;
        int sum = nodeId + (op & 0xFF);
        for (int i = 0; i < rest.length; i++) {
            payload[i + 1] = (byte) rest[i];
            sum += rest[i] & 0xFF;
        }
        payload[payload.length - 1] = (byte) (sum & 0xFF);
        return payload;
    }

    @Test
    void readSystemParameterParsesResponseByParamCode() {
        // Живой пример с шины: запрос [00, 83, 84], ответ [83, 06 40, CA] = 1600 мА.
        // Ответ начинается с кода параметра, а не с 0x00.
        MockCanBus mock = new MockCanBus();
        mock.queue(1, resp(1, 0x83, 0x06, 0x40));

        try (Mks42dController c = new Mks42dController(mock)) {
            byte[] params = c.readSystemParameter(0x83);

            assertNotNull(params);
            assertEquals(2, params.length);
            assertEquals(0x06, params[0] & 0xFF);
            assertEquals(0x40, params[1] & 0xFF);
            // запрос ушёл как [00, 83, crc]
            assertArrayEquals(new byte[]{0x00, (byte) 0x83, (byte) 0x84}, mock.getTxLog().get(0).data);
        }
    }

    @Test
    void readSystemParameterUnsupportedReturnsNull() {
        MockCanBus mock = new MockCanBus();
        mock.queue(1, resp(1, 0x9E, 0xFF, 0xFF));

        try (Mks42dController c = new Mks42dController(mock)) {
            assertNull(c.readSystemParameter(0x9E));
        }
    }

    @Test
    void readSystemParameterTimeoutReturnsNull() {
        MockCanBus mock = new MockCanBus();

        try (Mks42dController c = new Mks42dController(mock)) {
            assertNull(c.readSystemParameter(0x82));
        }
    }

    @Test
    void enableSendsCorrectFrame() {
        MockCanBus mock = new MockCanBus();
        mock.queue(1, resp(1, 0xF3, 1));

        try (Mks42dController c = new Mks42dController(mock)) {
            c.enable(true);

            List<MockCanBus.CanFrame> tx = mock.getTxLog();
            assertEquals(1, tx.size());
            assertEquals(1, tx.get(0).cobId);
            assertArrayEquals(new byte[]{(byte) 0xF3, 0x01, (byte) 0xF5}, tx.get(0).data);
        }
    }

    @Test
    void statusRequestSendsSingleOpcodeAndParsesResponse() {
        MockCanBus mock = new MockCanBus();
        // Ответ на 0xF1: [F1, status=1 (stopped), crc] — по одному на каждый запрос
        mock.queue(1, resp(1, 0xF1, 1));
        mock.queue(1, resp(1, 0xF1, 1));

        try (Mks42dController c = new Mks42dController(mock)) {
            assertEquals(Integer.valueOf(Mks42dController.ST_STOP), c.statusOrNull());
            assertTrue(c.isStopped());

            List<MockCanBus.CanFrame> tx = mock.getTxLog();
            // Запрос чтения: [F1, crc], crc = 1+F1 = 0xF2 (по мануалу 7.3: "01 30 31" — один байт команды)
            assertArrayEquals(new byte[]{(byte) 0xF1, (byte) 0xF2}, tx.get(0).data);
        }
    }

    @Test
    void turnToAbsoluteAngle180BuildsF5Frame() {
        MockCanBus mock = new MockCanBus();
        // 180° = 8192 axis units; 500 rpm = 0x01F4; acc 100 = 0x64
        mock.queue(1, resp(1, 0xF5, 1));   // движение принято

        try (Mks42dController c = new Mks42dController(mock)) {
            c.turnToAbsoluteAngle(180.0, 500, 100);

            List<MockCanBus.CanFrame> tx = mock.getTxLog();
            assertEquals(1, tx.size());
            // axis 8192 = 0x00002000 → [00 20 00]; crc = 1+F5+01+F4+64+20 = 0x6F
            assertArrayEquals(
                    new byte[]{(byte) 0xF5, 0x01, (byte) 0xF4, 0x64, 0x00, 0x20, 0x00, (byte) 0x6F},
                    tx.get(0).data);
        }
    }

    @Test
    void blockingTurnUsesDistanceProfileAndWaitsIdle() {
        MockCanBus mock = new MockCanBus();
        // дефолт (профиль по дистанции): из 0° на 180° -> 20 rpm = 0x14, acc 100 = 0x64, axis 8192
        mock.queue(1, resp(1, 0x31, 0, 0, 0, 0, 0, 0)); // текущая позиция: 0°
        mock.queue(1, resp(1, 0xF5, 1));               // движение принято
        mock.queue(1, resp(1, 0xF1, 1));               // мотор остановился

        try (Mks42dController c = new Mks42dController(mock)) {
            assertTrue(c.turnToAbsoluteAngle(180.0));

            List<MockCanBus.CanFrame> tx = mock.getTxLog();
            assertEquals(3, tx.size());
            // axis 8192 = 0x00002000 → [00 20 00]; crc = 1+F5+00+14+64+20 = 0x8E
            assertArrayEquals(
                    new byte[]{(byte) 0xF5, 0x00, 0x14, 0x64, 0x00, 0x20, 0x00, (byte) 0x8E},
                    tx.get(1).data);
            assertArrayEquals(new byte[]{(byte) 0xF1, (byte) 0xF2}, tx.get(2).data);
        }
    }

    @Test
    void microMoveProfileLowersSpeedAndAccel() {
        // мелкие шаги — самая медленная скорость и минимальное ускорение
        assertEquals(1, Mks42dController.speedForDistance(0.5));
        assertEquals(5, Mks42dController.accelForDistance(0.5));
        assertEquals(10, Mks42dController.speedForDistance(5.0));
        assertEquals(20, Mks42dController.accelForDistance(5.0));
        assertEquals(20, Mks42dController.speedForDistance(90.0));
        assertEquals(100, Mks42dController.accelForDistance(90.0));
    }

    @Test
    void defaultTurnUsesDistanceProfileFromCurrentPosition() {
        MockCanBus mock = new MockCanBus();
        mock.queue(1, resp(1, 0x31, 0, 0, 0, 0, 0x20, 0)); // текущая позиция: 180° (8192)
        mock.queue(1, resp(1, 0xF5, 1));                   // ход на 181°: dist 1° -> 1 rpm, acc 5
        mock.queue(1, resp(1, 0xF1, 1));                   // остановился

        try (Mks42dController c = new Mks42dController(mock, 1, 10)) {
            assertTrue(c.turnToAbsoluteAngle(181.0));

            List<MockCanBus.CanFrame> tx = mock.getTxLog();
            // F5: 1 rpm (0x0001), acc 5, axis 8238 = 0x202E
            // crc = 1+F5+00+01+05+00+20+2E = 330 = 0x4A
            assertArrayEquals(
                    new byte[]{(byte) 0xF5, 0x00, 0x01, 0x05, 0x00, 0x20, 0x2E, 0x4A},
                    tx.get(1).data);
        }
    }

    @Test
    void setZeroSends92Frame() {
        MockCanBus mock = new MockCanBus();
        mock.queue(1, resp(1, 0x92, 1));

        try (Mks42dController c = new Mks42dController(mock)) {
            c.setZero();

            List<MockCanBus.CanFrame> tx = mock.getTxLog();
            assertArrayEquals(new byte[]{(byte) 0x92, (byte) 0x93}, tx.get(0).data);
        }
    }

    @Test
    void encoderResponseParsesTo180Degrees() {
        MockCanBus mock = new MockCanBus();
        // Позиция 8192 = 0x2000, 6 байт big-endian — по одному ответу на каждый запрос
        mock.queue(1, resp(1, 0x31, 0, 0, 0, 0, 0x20, 0));
        mock.queue(1, resp(1, 0x31, 0, 0, 0, 0, 0x20, 0));

        try (Mks42dController c = new Mks42dController(mock)) {
            assertEquals(8192, c.readEncoder());
            assertEquals(180.0, c.getAngle(), 1e-9);

            List<MockCanBus.CanFrame> tx = mock.getTxLog();
            // Запрос чтения: [31, crc], crc = 1+31 = 0x32
            assertArrayEquals(new byte[]{0x31, 0x32}, tx.get(0).data);
        }
    }

    @Test
    void waitIdleReturnsFalseOnSilence() {
        MockCanBus mock = new MockCanBus();

        try (Mks42dController c = new Mks42dController(mock, 1, 10)) {
            long t0 = System.currentTimeMillis();
            assertFalse(c.waitIdle(60));
            assertTrue(System.currentTimeMillis() - t0 >= 50);
        }
    }

    @Test
    void badChecksumResponseIsIgnored() {
        MockCanBus mock = new MockCanBus();
        mock.queue(1, new byte[]{(byte) 0xF1, 0x01, 0x00}); // crc битый (должен быть 0xF3)

        try (Mks42dController c = new Mks42dController(mock, 1, 20)) {
            assertNull(c.statusOrNull());
        }
    }

    @Test
    void setHomeParamsBuilds90Frame() {
        MockCanBus mock = new MockCanBus();
        mock.queue(1, resp(1, 0x90, 1));

        try (Mks42dController c = new Mks42dController(mock)) {
            c.setHomeParams(1, 0, 3000, true, 1); // trig=1, dir=0, speed=3000=0x0BB8, endLimit=1, hmMode=1

            List<MockCanBus.CanFrame> tx = mock.getTxLog();
            // crc = 1+90+01+00+0B+B8+01+01 = 0x57
            assertArrayEquals(
                    new byte[]{(byte) 0x90, 0x01, 0x00, 0x0B, (byte) 0xB8, 0x01, 0x01, (byte) 0x57},
                    tx.get(0).data);
        }
    }

    @Test
    void setHoldingCurrentBuilds9BFrame() {
        MockCanBus mock = new MockCanBus();
        mock.queue(1, resp(1, 0x9B, 1));

        try (Mks42dController c = new Mks42dController(mock)) {
            c.setHoldingCurrent(2000); // 2000 мА = 0x07D0

            List<MockCanBus.CanFrame> tx = mock.getTxLog();
            // crc = 1+9B+07+D0 = 0x73
            assertArrayEquals(
                    new byte[]{(byte) 0x9B, 0x07, (byte) 0xD0, (byte) 0x73},
                    tx.get(0).data);
        }
    }

    @Test
    void rejectedCommandThrows() {
        MockCanBus mock = new MockCanBus();
        mock.queue(1, resp(1, 0xF3, 0)); // status = 0 (Fail)

        try (Mks42dController c = new Mks42dController(mock)) {
            assertThrows(IllegalStateException.class, () -> c.enable(false));
        }
    }
}
