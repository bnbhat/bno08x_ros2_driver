#ifndef UART_INTERFACE_HPP
#define UART_INTERFACE_HPP

#include <cstring>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/time.h>
#include "comm_interface.hpp"

/**
 * @brief UART-SHTP communication interface for the BNO08x sensor.
 *
 * This implements full bidirectional UART-SHTP mode (BNO08x datasheet §1.2.3).
 * It is NOT the H-format output mode (SH-2 ref §3.1, Euler-only, output-only)
 * and NOT the UART-RVC mode (§1.2.5, 19-byte 0xAAAA packets at 115200 baud).
 * UART-SHTP is selected by setting PS1=high, PS0=low on the BNO08x.
 *
 * Hardware requirements (datasheet §1.2.3):
 *   - External crystal or clock required; internal oscillator is not stable
 *     enough for reliable UART at 3 Mb/s.
 *   - 8 data bits, 1 stop bit, no parity, LSB first.
 *   - H_INTN is asserted (low) before the first byte of each sensor message
 *     and deasserted between messages. We do not read this pin — instead
 *     select() detects data arrival via the UART receive buffer.
 *
 * SHTP framing (datasheet §1.3.1, shtp.c rxAssemble):
 *   Every packet begins with a 4-byte SHTP header immediately followed by
 *   the payload. There are no additional start/end framing bytes; the length
 *   field in the header delimits packets.
 *     byte 0:   length LSB  }  bits 14:0 = total bytes (header + payload)
 *     byte 1:   length MSB  }  bit  15   = continuation flag
 *     byte 2:   channel
 *     byte 3:   sequence number
 *
 * For UART (unlike I2C), the sensor sends the entire packet in one continuous
 * transmission — no 32-byte chunking. read() reads exactly packet_size bytes
 * (header + payload) per call and returns 0 if no packet is ready.
 */
class UARTInterface : public CommInterface {
public:
    UARTInterface(const std::string& uart_device, int baudrate = 3000000)
        : uart_fd_(-1), uart_device_(uart_device), baudrate_(baudrate) {
        std::cout << "UART device:   " << uart_device_ << std::endl;
        std::cout << "UART baudrate: " << baudrate_    << std::endl;
    }

    int open() override {
        uart_fd_ = ::open(uart_device_.c_str(), O_RDWR | O_NOCTTY);
        if (uart_fd_ < 0) {
            std::cerr << "BNO08x - Failed to open UART: " << uart_device_ << std::endl;
            return -1;
        }

        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        if (tcgetattr(uart_fd_, &tty) != 0) {
            std::cerr << "BNO08x - tcgetattr failed" << std::endl;
            ::close(uart_fd_);
            uart_fd_ = -1;
            return -1;
        }

        speed_t speed = baud_to_speed(baudrate_);
        if (speed == B0) {
            std::cerr << "BNO08x - Unsupported baud rate: " << baudrate_ << std::endl;
            ::close(uart_fd_);
            uart_fd_ = -1;
            return -1;
        }
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);

        // Raw 8N1, no flow control.
        cfmakeraw(&tty);
        tty.c_cflag |=  (CLOCAL | CREAD);
        tty.c_cflag &= ~CRTSCTS;
        // Block until at least 1 byte arrives; 1-second inter-byte timeout.
        tty.c_cc[VMIN]  = 1;
        tty.c_cc[VTIME] = 10;

        if (tcsetattr(uart_fd_, TCSANOW, &tty) != 0) {
            std::cerr << "BNO08x - tcsetattr failed" << std::endl;
            ::close(uart_fd_);
            uart_fd_ = -1;
            return -1;
        }

        // Flush any stale pre-open data before the reset.
        tcflush(uart_fd_, TCIOFLUSH);

        // Send a SHTP soft-reset command on the executable channel (channel 1).
        // Packet: length=5 (4 header + 1 payload), channel=1, seq=0, cmd=1 (reset).
        // This matches what the I2C interface sends and is required because sh2_open()
        // waits up to 200 ms for EXECUTABLE_DEVICE_RESP_RESET_COMPLETE — which only
        // arrives after the sensor goes through a reset cycle.
        uint8_t reset_pkt[] = {5, 0, 1, 0, 1};
        if (::write(uart_fd_, reset_pkt, sizeof(reset_pkt)) != sizeof(reset_pkt)) {
            std::cerr << "BNO08x - Warning: soft reset packet may not have been sent" << std::endl;
        }

        // Wait for the sensor to reboot and emit its advertisement + reset-complete
        // messages into the UART RX buffer.  The OS UART driver buffers these bytes
        // until sh2_open() starts calling read().  Sensor boot takes ~90 ms; we use
        // 300 ms to match the I2C implementation and provide a safe margin.
        usleep(300000);

        return 0;
    }

    void close() override {
        if (uart_fd_ >= 0) {
            ::close(uart_fd_);
            uart_fd_ = -1;
        }
    }

    /**
     * Called by sh2_service() on every poll. Returns one complete SHTP packet
     * (header + payload) in pBuffer, or 0 if no packet is ready yet.
     *
     * Non-blocking poll via select() with zero timeout: if no bytes are waiting
     * we return 0 immediately so the SH-2 library can continue.
     */
    int read(uint8_t* pBuffer, unsigned len, uint32_t* t_us) override {
        // Non-blocking availability check.
        struct timeval tv = {0, 0};
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(uart_fd_, &readfds);
        if (select(uart_fd_ + 1, &readfds, nullptr, nullptr, &tv) <= 0) {
            return 0;
        }

        // Read the 4-byte SHTP header.
        uint8_t header[4];
        if (!read_exactly(header, 4)) {
            return 0;
        }

        // Total packet length is encoded in header[0:1] (LE), bit-15 = continuation.
        uint16_t packet_size = ((uint16_t)header[0] | ((uint16_t)header[1] << 8)) & ~0x8000;
        if (packet_size < 4 || packet_size > len) {
            return 0;
        }

        // Place the header at the start of pBuffer (SH-2 rxAssemble() expects it there).
        memcpy(pBuffer, header, 4);

        // Read the payload that follows.
        uint16_t payload_len = packet_size - 4;
        if (payload_len > 0 && !read_exactly(pBuffer + 4, payload_len)) {
            return 0;
        }

        *t_us = getTimeUs();
        return packet_size;
    }

    // The SH-2 library supplies a fully-formed SHTP packet; send all bytes.
    // The SH-2 library does not check the return value of write(), so a
    // partial write would silently drop bytes — loop until all are sent.
    int write(uint8_t* pBuffer, unsigned len) override {
        size_t sent = 0;
        while (sent < len) {
            ssize_t w = ::write(uart_fd_, pBuffer + sent, len - sent);
            if (w <= 0) break;
            sent += static_cast<size_t>(w);
        }
        return static_cast<int>(sent);
    }

private:
    int uart_fd_;
    std::string uart_device_;
    int baudrate_;

    // Blocks until exactly n bytes are received or an error occurs.
    bool read_exactly(uint8_t* buf, size_t n) {
        size_t received = 0;
        while (received < n) {
            ssize_t r = ::read(uart_fd_, buf + received, n - received);
            if (r <= 0) return false;
            received += static_cast<size_t>(r);
        }
        return true;
    }

    speed_t baud_to_speed(int baud) {
        switch (baud) {
            case 9600:    return B9600;
            case 19200:   return B19200;
            case 38400:   return B38400;
            case 57600:   return B57600;
            case 115200:  return B115200;
            case 230400:  return B230400;
            case 460800:  return B460800;
            case 921600:  return B921600;
#ifdef B1000000
            case 1000000: return B1000000;
#endif
#ifdef B1500000
            case 1500000: return B1500000;
#endif
#ifdef B2000000
            case 2000000: return B2000000;
#endif
#ifdef B3000000
            case 3000000: return B3000000;
#endif
            default:      return B0;
        }
    }
};

#endif // UART_INTERFACE_HPP
