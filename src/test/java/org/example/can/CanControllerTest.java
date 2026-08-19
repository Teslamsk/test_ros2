package org.example.can;

import org.example.can.transport.MockCanBus;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class CanControllerTest {

    /**
     * 1. CanController accepts CanBus
     * 2. MockCanBus queues a SDO response: 0x581 → 0x43 0 0 0 0x1A 0 0 0 (response = 0x0000001A = 26)
     * 3. CanController.sdoRead(0x6041) → bus.send(0x601, ...) → bus.receive(0x581) → value = 26
     */
    @Test
    void sdoReadResolvesFromMockBus() {
        MockCanBus mock = new MockCanBus();
        // Pre-queue the SDO response for 0x581 (SDO response from node 1)
        byte[] response = new byte[8];
        response[0] = (byte) 0x43; // header byte (SDO expedited upload response, CiA 301)
        response[1] = 0; // high index
        response[2] = 0; // sub-index
        response[3] = 0; // reserved
        response[4] = 0x1A;
        response[5] = 0;
        response[6] = 0;
        response[7] = 0;

        mock.queue(0x581, response);

        try (CanController ctrl = new CanController(mock)) {
            // Read Status Word (0x6041) from node 1
            int status = ctrl.getStatusWord(org.example.can.dictionary.Node.ROTATE_LIDAR_Z);
            assertEquals(26, status);
        }
    }

    /**
     * 2. MockCanBus.log records all frames sent via bus.
     */
    @Test
    void sdoWritesLogFrames() {
        MockCanBus mock = new MockCanBus();

        try (CanController ctrl = new CanController(mock)) {
            ctrl.setHandMode(org.example.can.dictionary.Node.ROTATE_LIDAR_Z);

            // After setHandMode, we expect at least 4 writes:
            // 1) SDO write Mode of Operation (0x601)
            // 2) SDO write Max Torque (0x601)
            // 3) SDO write Target Torque (0x601)
            // 4) PDO torque (RXPDO1, likely 0x2A1)
            List<MockCanBus.CanFrame> txLog = mock.getTxLog();
            assertTrue(txLog.size() >= 4, "Expected at least 4 frames, got " + txLog.size());
        }
    }
}
