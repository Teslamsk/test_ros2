/**
 * canwrapper.c — thin bridge from JNA to Linux AF_CAN raw sockets.
 *
 * Compile:
 *   Linux:  gcc -shared -fPIC -o libcanwrapper.so canwrapper.c
 *   macOS:  gcc -shared -fPIC -o libcanwrapper.dylib canwrapper.c
 *   Windows (MSYS2/MinGW):  gcc -shared -o canwrapper.dll canwrapper.c
 *
 * Place resulting .so/.dll in a directory on java.library.path,
 * e.g.  the working directory or /usr/lib/jni/.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#ifdef _WIN32
    #include <winsock2.h>
    #include <windows.h>
    /* Windows AF_CAN is not natively available.
     * On WSL2 you'd run this as a Linux binary.
     * On bare Windows, stub out with a PCAN shim DLL instead. */
    int  can_open_stub  (const char *ifname) { return -1; }
    int  can_send_stub  (int fd, int cobId, const uint8_t *data, int dlc) { return -1; }
    int  can_recv_stub  (int fd, int *cobId, uint8_t *data, int *dlc, int timeoutMs) { return -1; }
    int  can_close_stub (int fd) { return -1; }
#else
    #include <sys/socket.h>
    #include <sys/ioctl.h>
    #include <sys/time.h>
    #include <net/if.h>
    #include <unistd.h>
    #include <linux/can.h>
    #include <linux/can/raw.h>
    #include <arpa/inet.h>

    /*
     * can_open
     *   Creates AF_CAN raw socket, binds to interface, enables can_rxerror_frames/can_txerror_frames off.
     *   Returns fd (>=0) or -1 on error.
     */
    int can_open(const char *ifname) {
        int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (fd < 0) {
            perror("socket");
            return -1;
        }

        struct ifreq ifr;
        struct sockaddr_can addr;
        memset(&ifr, 0, sizeof(ifr));
        strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);

        if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
            perror("ioctl SIOCGIFINDEX");
            close(fd);
            return -1;
        }

        addr.can_family  = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        addr.can_addr.tp = 0;

        /* Bind to RX filter: accept all frames (mask 0 means "accept anything"). */
        struct can_filter rfilter[1];
        rfilter[0].can_id = 0x00000000;
        rfilter[0].can_mask = 0x00000000;

        if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, rfilter, sizeof(rfilter)) < 0) {
            perror("setsockopt CAN_RAW_FILTER");
            close(fd);
            return -1;
        }

        if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            perror("bind");
            close(fd);
            return -1;
        }

        fprintf(stdout, "[canwrapper] bound to %s (index=%d, fd=%d)\n", ifname, addr.can_ifindex, fd);
        return fd;
    }

    /*
     * can_send
     *   Transmits a single CAN frame.
     *   Returns 0 on success, -1 on error.
     */
    int can_send(int fd, int cobId, const uint8_t *data, int dlc) {
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));
        frame.can_id = cobId & 0x1FFFFFFF;  /* standard or extended */

        if (cobId > 0x7FF) {
            frame.can_id |= CAN_EFF_FLAG;
        }
        frame.can_dlc = dlc > 8 ? 8 : dlc;
        memcpy(frame.data, data, frame.can_dlc);

        ssize_t sent = write(fd, &frame, sizeof(frame));
        if (sent < 0) {
            perror("write");
            return -1;
        }
        return 0;
    }

    /*
     * can_recv
     *   Polls frames until one matches the expected COB-ID or deadline is reached.
     *   Doing the loop in C avoids JNA round-trips for every non-matching frame.
     *
     *   Returns 0 on success (fills *cobId, data[], *dlc), -1 on timeout/error.
     */
    #include <time.h>

    int can_recv(int fd, int expectedCobId, uint8_t *data, int *dlc, int timeoutMs) {
        struct timespec now, deadline, ts;

        clock_gettime(CLOCK_MONOTONIC, &now);
        deadline.tv_sec  = now.tv_sec  + timeoutMs / 1000;
        deadline.tv_nsec = now.tv_nsec + ((long)(timeoutMs % 1000)) * 1000000L;
        if (deadline.tv_nsec >= 1000000000L) {
            deadline.tv_sec  += 1;
            deadline.tv_nsec -= 1000000000L;
        }

        struct can_frame frame;
        while (1) {
            clock_gettime(CLOCK_MONOTONIC, &now);
            if (now.tv_sec >= deadline.tv_sec && deadline.tv_nsec != 999999999L) {
                /* deadline passed or equal — check nanoseconds too */
                if (now.tv_sec > deadline.tv_sec) return -1;
                /* same second */
                if (now.tv_nsec >= deadline.tv_nsec) return -1;
            }

            ts.tv_sec  = deadline.tv_sec  - now.tv_sec;
            ts.tv_nsec = deadline.tv_nsec - now.tv_nsec;
            if (ts.tv_nsec < 0) {
                ts.tv_sec  -= 1;
                ts.tv_nsec += 1000000000L;
            }

            struct timespec ts_zero = {0, 0};
            if (ts.tv_sec < 0 || (ts.tv_sec == 0 && ts.tv_nsec <= 0))
                return -1;

            ssize_t n = recv(fd, &frame, sizeof(frame), MSG_DONTWAIT);
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    /* No data right now — recalculate and try again. */
                    nanosleep(&(struct timespec){0, 1000000L}, NULL); /* 1 ms sleep */
                    continue;
                }
                perror("recv");
                return -1;
            }

            int raw_id = frame.can_id;
            int plain_id;
            if (raw_id & CAN_EFF_FLAG) {
                plain_id = raw_id & CAN_EFF_MASK;
            } else {
                plain_id = raw_id & CAN_SFF_MASK;
            }

            if (expectedCobId != -1 && plain_id != expectedCobId)
                continue;  /* non-matching frame — discard and loop */

            *expectedCobId = plain_id;
            *dlc = frame.can_dlc > 8 ? 8 : frame.can_dlc;
            memcpy(data, frame.data, *dlc);
            return 0;
        }
    }

    /*
     * can_close
     *   Closes the socket fd.
     *   Returns 0 on success, -1 on invalid fd.
     */
    int can_close(int fd) {
        if (fd < 0) return -1;
        fprintf(stdout, "[canwrapper] closing fd=%d\n", fd);
        return close(fd);
    }
#endif
