package org.example.can.transport;

import com.sun.jna.Library;
import com.sun.jna.Native;
import com.sun.jna.ptr.IntByReference;

import java.util.Arrays;

/**
 * SocketCAN transport via AF_CAN raw sockets (Linux).
 *
 * Native library: libcanwrapper.so — compiled from src/main/c/canwrapper.c
 *
 * On Windows use PcanBus instead.
 */
public class SocketCanBus implements CanBus {

    public interface CanWrapper extends Library {
        CanWrapper INSTANCE = Native.load("canwrapper", CanWrapper.class);

        int can_open(String ifname);

        int can_send(int fd, int cobId, byte[] data, int dlc);

        int can_recv(int fd, int expectedCobId, byte[] data, IntByReference dlc, int timeoutMs);

        int can_close(int fd);
    }

    private int fd = -1;
    private String ifname;

    @Override
    public void open(String iface) {
        this.ifname = iface;
        this.fd = CanWrapper.INSTANCE.can_open(iface);
        if (fd < 0) {
            throw new RuntimeException("Cannot open CAN interface " + iface);
        }
        System.out.println("[CAN] Opened " + iface + " (fd=" + fd + ")");
    }

    @Override
    public void send(int cobId, byte[] data) {
        int dlc = data.length;
        int ret = CanWrapper.INSTANCE.can_send(fd, cobId, data, dlc);
        if (ret < 0) {
            System.err.printf("[CAN] send error fd=%d cob=0x%03X%n", fd, cobId);
        }
    }

    @Override
    public byte[] receive(int cobId, int timeoutMs) {
        byte[] buf = new byte[8];
        IntByReference outDlc = new IntByReference();

        int ret = CanWrapper.INSTANCE.can_recv(fd, cobId, buf, outDlc, timeoutMs);
        if (ret < 0) {
            return null;
        }

        int dlc = outDlc.getValue();
        return Arrays.copyOf(buf, dlc > 8 ? 8 : dlc);
    }

    @Override
    public void close() {
        if (fd >= 0) {
            CanWrapper.INSTANCE.can_close(fd);
            fd = -1;
            System.out.println("[CAN] Closed " + ifname);
        }
    }
}
