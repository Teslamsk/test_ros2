package org.example.can.transport;

import com.sun.jna.Library;
import com.sun.jna.Native;
import com.sun.jna.ptr.IntByReference;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;
import java.util.Arrays;

/**
 * SocketCAN transport via AF_CAN raw sockets (Linux).
 *
 * Native library: libcanwrapper-<arch>.so, compiled from canwrapper.c
 * (source is also bundled in the JAR as resource /canwrapper.c).
 * Load order:
 *   1. Standard JNA search (library path, CWD, LD_LIBRARY_PATH) — allows
 *      dropping a locally compiled libcanwrapper.so next to the jar.
 *   2. The embedded resource matched by os.arch (x86-64 / aarch64) — extracted to tmp
 *      and loaded by absolute path.
 *
 * On Windows use PcanBus instead.
 */
public class SocketCanBus implements CanBus {

    public interface CanWrapper extends Library {
        int can_open(String ifname);

        int can_send(int fd, int cobId, byte[] data, int dlc);

        int can_recv(int fd, int expectedCobId, byte[] data, IntByReference dlc, int timeoutMs);

        int can_close(int fd);
    }

    private static final CanWrapper LIB;

    static {
        CanWrapper lib;
        try {
            lib = Native.load("canwrapper", CanWrapper.class);
        } catch (UnsatisfiedLinkError e) {
            lib = Native.load(extractEmbedded().getAbsolutePath(), CanWrapper.class);
        }
        LIB = lib;
    }

    /**
     * Extracts libcanwrapper-<arch>.so from the JAR (resources /native/libcanwrapper-<arch>.so)
     * into the tmp directory and returns the path. The file is cleaned up on JVM exit.
     */
    private static File extractEmbedded() {
        String resource = "/native/libcanwrapper-" + archName() + ".so";
        try (InputStream in = SocketCanBus.class.getResourceAsStream(resource)) {
            if (in == null) {
                throw new UnsatisfiedLinkError("Native library cannot be loaded: not found on the library path, "
                        + "and " + resource + " is not in the JAR either (os.arch=" + osArch() + ")."
                        + " Compile yourself: unzip -o <jar> canwrapper.c && gcc -shared -fPIC -o libcanwrapper.so canwrapper.c");
            }
            File f = extractToTmp(in);
            // Sanity check: the file must exist and be non-empty before dlopen
            if (!f.exists() || f.length() == 0) {
                throw new UnsatisfiedLinkError("Native library extraction failed: " + f);
            }
            return f;
        } catch (IOException e) {
            throw new UnsatisfiedLinkError("libcanwrapper.so: extraction from JAR failed: " + e);
        }
    }

    private static File extractToTmp(InputStream in) throws IOException {
        File dir = new File(System.getProperty("java.io.tmpdir"), "canwrapper-" + System.nanoTime());
        if (!dir.mkdir() && !dir.isDirectory()) {
            throw new IOException("Cannot create directory " + dir);
        }
        File f = new File(dir, "libcanwrapper.so");
        Files.copy(in, f.toPath(), StandardCopyOption.REPLACE_EXISTING);
        if (!f.setExecutable(true) && !f.canExecute()) {
            throw new IOException("libcanwrapper.so: no execute permission " + f);
        }
        f.deleteOnExit();
        dir.deleteOnExit();
        return f;
    }

    private static String archName() {
        return switch (osArch()) {
            case "amd64", "x86_64", "i386" -> "x86-64";
            case "aarch64", "arm64" -> "aarch64";
            default -> throw new UnsatisfiedLinkError(
                    "Unsupported os.arch: " + osArch() + " (only x86-64 and aarch64 are available)");
        };
    }

    private static String osArch() {
        return System.getProperty("os.arch", "").toLowerCase();
    }

    private int fd = -1;
    private String ifname;

    @Override
    public void open(String iface) {
        this.ifname = iface;
        this.fd = LIB.can_open(iface);
        if (fd < 0) {
            throw new RuntimeException("Cannot open CAN interface " + iface);
        }
        System.out.println("[CAN] Opened " + iface + " (fd=" + fd + ")");
    }

    @Override
    public void send(int cobId, byte[] data) {
        if (fd < 0) {
            return; // bus is not open — silently ignore (e.g., emergency stop after a failed launch)
        }
        int dlc = data.length;
        int ret = LIB.can_send(fd, cobId, data, dlc);
        if (ret < 0) {
            System.err.printf("[CAN] send error fd=%d cob=0x%03X%n", fd, cobId);
        }
    }

    @Override
    public byte[] receive(int cobId, int timeoutMs) {
        if (fd < 0) {
            return null;
        }
        byte[] buf = new byte[8];
        IntByReference outDlc = new IntByReference();

        int ret = LIB.can_recv(fd, cobId, buf, outDlc, timeoutMs);
        if (ret < 0) {
            return null;
        }

        int dlc = outDlc.getValue();
        return Arrays.copyOf(buf, dlc > 8 ? 8 : dlc);
    }

    @Override
    public void close() {
        if (fd >= 0) {
            LIB.can_close(fd);
            fd = -1;
            System.out.println("[CAN] Closed " + ifname);
        }
    }
}
