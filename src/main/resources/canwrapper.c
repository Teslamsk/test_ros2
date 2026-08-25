/**
 * canwrapper.c — thin JNA bridge to Linux AF_CAN raw sockets.
 *
 * Compile:
 *   Linux:        gcc -shared -fPIC -Wall -o libcanwrapper.so canwrapper.c
 *   Windows(MinGW): gcc -shared -Wall -o canwrapper.dll canwrapper.c   (stubs only, not functional)
 *
 * Requires the Linux kernel (AF_CAN, linux/can.h) — not portable to macOS.
 * Place the resulting .so/.dll in a directory on java.library.path
 * (working directory or /usr/lib/jni/).
 *
 * API (mapped in SocketCanBus, package org.example.can.transport):
 *   int can_open(const char *ifname);
 *   int can_send(int fd, int cobId, const uint8_t *data, int dlc);
 *   int can_recv(int fd, int expectedCobId, uint8_t *data, int *dlc, int timeoutMs);
 *   int can_close(int fd);
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>

#ifdef _WIN32
    /*
     * AF_CAN is not natively available on Windows.
     * On WSL2 run this as a Linux binary; on bare Windows use PcanBus (PCAN-Basic DLL).
     * Stubs below (with the SAME names as the real API) let canwrapper.dll link
     * so the JNA interface can load; every call returns -1 (unsupported).
     */
    int can_open(const char *ifname) {
        (void) ifname;
        return -1;
    }

    int can_send(int fd, int cobId, const uint8_t *data, int dlc) {
        (void) fd; (void) cobId; (void) data; (void) dlc;
        return -1;
    }

    int can_recv(int fd, int expectedCobId, uint8_t *data, int *dlc, int timeoutMs) {
        (void) fd; (void) expectedCobId; (void) data; (void) dlc; (void) timeoutMs;
        return -1;
    }

    int can_close(int fd) {
        (void) fd;
        return -1;
    }
#else
    #include <sys/socket.h>
    #include <sys/ioctl.h>
    #include <sys/time.h>
    #include <time.h>
    #include <net/if.h>
    #include <unistd.h>
    #include <linux/can.h>
    #include <linux/can/raw.h>

    /*
     * can_open
     *   Creates an AF_CAN raw socket, binds it to the interface,
     *   and sets a pass-all RX filter.
     *   Returns fd (>=0), or -1 on error.
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
        memset(&addr, 0, sizeof(addr));
        strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);

        if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
            perror("ioctl SIOCGIFINDEX");
            close(fd);
            return -1;
        }

        addr.can_family  = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        /* RX filter: accept all frames (mask 0 means "accept anything"). */
        struct can_filter rfilter[1];
        rfilter[0].can_id   = 0x00000000;
        rfilter[0].can_mask = 0x00000000;

        if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, rfilter, sizeof(rfilter)) < 0) {
            perror("setsockopt CAN_RAW_FILTER");
            close(fd);
            return -1;
        }

        if (bind(fd, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
            perror("bind");
            close(fd);
            return -1;
        }

        fprintf(stdout, "[canwrapper] bound to %s (index=%d, fd=%d)\n", ifname, addr.can_ifindex, fd);
        return fd;
    }

    /*
     * can_send
     *   Transmits a single CAN frame (standard, or extended if cobId > 0x7FF).
     *   Returns 0 on success, -1 on error.
     */
    int can_send(int fd, int cobId, const uint8_t *data, int dlc) {
        struct can_frame frame;
        memset(&frame, 0, sizeof(frame));
        frame.can_id = cobId & 0x1FFFFFFF; /* standard or extended */
        if (cobId > 0x7FF) {
            frame.can_id |= CAN_EFF_FLAG;
        }
        frame.can_dlc = dlc > 8 ? 8 : (dlc < 0 ? 0 : dlc);
        memcpy(frame.data, data, frame.can_dlc);

        ssize_t sent = write(fd, &frame, sizeof(frame));
        if (sent < 0) {
            perror("write");
            return -1;
        }
        return 0;
    }

    /*
     * is_deadline_passed
     *   Monotonic-clock comparison against a precomputed deadline.
     */
    static int is_deadline_passed(const struct timespec *deadline) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        return (now.tv_sec > deadline->tv_sec) ||
               (now.tv_sec == deadline->tv_sec && now.tv_nsec >= deadline->tv_nsec);
    }

    /*
     * can_recv
     *   Waits for a frame whose COB-ID matches expectedCobId,
     *   or for the deadline (timeoutMs from now) to elapse.
     *   expectedCobId == -1: accept any frame.
     *   Non-matching frames are discarded in C (avoids JNA round-trips).
     *
     *   Implementation: blocking recv with SO_RCVTIMEO = 10 ms per iteration
     *   + an overall monotonic deadline. (A non-blocking recv + 1 ms spin
     *   burns CPU and had a broken deadline check.)
     *
     *   Returns 0 on success (fills data[], *dlc), -1 on timeout/error.
     */
    int can_recv(int fd, int expectedCobId, uint8_t *data, int *dlc, int timeoutMs) {
        struct timespec now, deadline;
        struct timeval rcvto;

        if (timeoutMs < 1) {
            timeoutMs = 1;
        }
        clock_gettime(CLOCK_MONOTONIC, &now);
        deadline.tv_sec  = now.tv_sec  + timeoutMs / 1000;
        deadline.tv_nsec = now.tv_nsec + (long) (timeoutMs % 1000) * 1000000L;
        if (deadline.tv_nsec >= 1000000000L) {
            deadline.tv_sec  += 1;
            deadline.tv_nsec -= 1000000000L;
        }

        /* Kernel-side wait: up to 10 ms per recv() call. */
        rcvto.tv_sec  = 0;
        rcvto.tv_usec = 10 * 1000;
        if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &rcvto, sizeof(rcvto)) < 0) {
            perror("setsockopt SO_RCVTIMEO");
            return -1;
        }

        for (;;) {
            struct can_frame frame;
            ssize_t n = recv(fd, &frame, sizeof(frame), 0);
            if (n < 0) {
                if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                    if (is_deadline_passed(&deadline)) {
                        return -1; /* timeout */
                    }
                    continue;
                }
                perror("recv");
                return -1; /* persistent error */
            }
            if (n != (ssize_t) sizeof(frame)) {
                continue; /* malformed — skip */
            }
            if (is_deadline_passed(&deadline)) {
                return -1; /* hard timeout even under a flood of frames */
            }

            int plain_id;
            if (frame.can_id & CAN_EFF_FLAG) {
                plain_id = frame.can_id & CAN_EFF_MASK;
            } else {
                plain_id = frame.can_id & CAN_SFF_MASK;
            }

            if (expectedCobId != -1 && plain_id != expectedCobId) {
                continue; /* non-matching frame — discard and keep waiting */
            }

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
        if (fd < 0) {
            return -1;
        }
        fprintf(stdout, "[canwrapper] closing fd=%d\n", fd);
        return close(fd);
    }
#endif
