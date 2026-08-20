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
    void statusRequestDuplicatesOpcodeAndParsesResponse() {
        MockCanBus mock = new MockCanBus();
        // Ответ на 0xF1: [F1, status=1 (stopped), crc] — по одному на каждый запрос
        mock.queue(1, resp(1, 0xF1, 1));
        mock.queue(1, resp(1, 0xF1, 1));

        try (Mks42dController c = new Mks42dController(mock)) {
            assertEquals(Integer.valueOf(Mks42dController.ST_STOP), c.statusOrNull());
            assertTrue(c.isStopped());

            List<MockCanBus.CanFrame> tx = mock.getTxLog();
            // Запрос: [F1, F1, crc] — для чтения opcode дублируется; crc = 1+F1+F1 = 0xE3
            assertArrayEquals(new byte[]{(byte) 0xF1, (byte) 0xF1, (byte) 0xE3}, tx.get(0).data);
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
    void blockingTurnUsesDefaultsAndWaitsIdle() {
        MockCanBus mock = new MockCanBus();
        // дефолты: 20 rpm = 0x14, acc 100 = 0x64, axis 8192
        mock.queue(1, resp(1, 0xF5, 1));   // движение принято
        mock.queue(1, resp(1, 0xF1, 1));   // мотор остановился

        try (Mks42dController c = new Mks42dController(mock)) {
            assertTrue(c.turnToAbsoluteAngle(180.0));

            List<MockCanBus.CanFrame> tx = mock.getTxLog();
            assertEquals(2, tx.size());
            // axis 8192 = 0x00002000 → [00 20 00]; crc = 1+F5+00+14+64+20 = 0x8E
            assertArrayEquals(
                    new byte[]{(byte) 0xF5, 0x00, 0x14, 0x64, 0x00, 0x20, 0x00, (byte) 0x8E},
                    tx.get(0).data);
            assertArrayEquals(new byte[]{(byte) 0xF1, (byte) 0xF1, (byte) 0xE3}, tx.get(1).data);
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
            assertArrayEquals(new byte[]{0x31, 0x31, 0x63}, tx.get(0).data);
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
    void rejectedCommandThrows() {
        MockCanBus mock = new MockCanBus();
        mock.queue(1, resp(1, 0xF3, 0)); // status = 0 (Fail)

        try (Mks42dController c = new Mks42dController(mock)) {
            assertThrows(IllegalStateException.class, () -> c.enable(false));
        }
    }
}
