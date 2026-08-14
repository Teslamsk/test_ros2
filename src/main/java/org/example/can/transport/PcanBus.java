package org.example.can.transport;

import com.sun.jna.Library;
import com.sun.jna.Native;
import com.sun.jna.ptr.IntByReference;

import java.util.Arrays;

/**
 * PCAN-Basic transport (Windows primary, also Linux via libpcanbasic.so).
 *
 * Relies on PCAN-Basic DLL from Peak-System:
 *   https://www.peak-system.com/PCAN-Basic-API-DLLs:98.htm
 *
 * API:
 *   TPCANError PCAN_Initialize(TPCAN channel, TPCAN_BAUDRATE baudrate);
 *   TPCANError PCAN_Write(TPCAN channel, TPCAN_ID id, TPCAN_MESSAGE_TYPE type, const char *txdata, TPCAN_DLC dlc);
 *   TPCANError PCAN_Read(TPCAN channel, TPCAN_ID *id, char *rxdata, TPCAN_DLC *dlc);
 *   TPCANError PCAN_Reset(TPCAN channel);
 *   TPCANError PCAN_Uninitialize(TPCAN channel);
 */
public class PcanBus implements CanBus {

    public interface PcanLib extends Library {
        PcanLib INSTANCE = Native.load("pcanbasic", PcanLib.class);

        int PCAN_Initialize(int channel, int baudrate);
        int PCAN_Write(int channel, int id, int type, byte[] txdata, short dlc);
        int PCAN_Read(int channel, IntByReference id, byte[] rxdata, IntByReference dlc);
        int PCAN_Reset(int channel);
        int PCAN_Uninitialize(int channel);
    }

    // PCAN constants
    public static final int PCAN_USB1   = 0x43;
    public static final int PCAN_ERROR_OK = 0x00;
    public static final int PCAN_MESSAGE_STANDARD = 0;
    public static final int PCAN_BAUD_500K = 0x0006;

    private int channel;
    private boolean initialized = false;

    public PcanBus(int channel) {
        this.channel = channel;
    }

    @Override
    public void open(String iface) {
        int ret = PcanLib.INSTANCE.PCAN_Initialize(channel, PCAN_BAUD_500K);
        if (ret != PCAN_ERROR_OK) {
            throw new RuntimeException("PCAN_Initialize failed (code=0x" + Integer.toHexString(ret) + ")");
        }
        initialized = true;
        System.out.println("[CAN] PCAN initialized on channel 0x" + Integer.toHexString(channel));
    }

    @Override
    public void send(int cobId, byte[] data) {
        int ret = PcanLib.INSTANCE.PCAN_Write(channel, cobId, PCAN_MESSAGE_STANDARD, data, (short) data.length);
        if (ret != PCAN_ERROR_OK) {
            System.err.printf("[CAN] PCAN_Write error 0x%02X cob=0x%03X%n", ret, cobId);
        }
    }

    @Override
    public byte[] receive(int cobId, int timeoutMs) {
        // PCAN_Read is non-blocking; we poll with a deadline.
        long deadline = System.currentTimeMillis() + timeoutMs;
        while (System.currentTimeMillis() < deadline) {
            byte[] buf = new byte[8];
            IntByReference outId = new IntByReference();
            IntByReference outDlc = new IntByReference();

            int ret = PcanLib.INSTANCE.PCAN_Read(channel, outId, buf, outDlc);
            if (ret != PCAN_ERROR_OK) {
                try { Thread.sleep(1); } catch (InterruptedException ignored) {}
                continue;
            }

            if (outId.getValue() == cobId) {
                return Arrays.copyOf(buf, outDlc.getValue());
            }
        }
        return null;  // timeout
    }

    @Override
    public void close() {
        if (initialized) {
            PcanLib.INSTANCE.PCAN_Uninitialize(channel);
            initialized = false;
            System.out.println("[CAN] PCAN uninitialized channel 0x" + Integer.toHexString(channel));
        }
    }
}
