#ifndef UART_INTERFACE_HPP
#define UART_INTERFACE_HPP

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <sys/ioctl.h>
#include "comm_interface.hpp"

class UARTInterface : public CommInterface {
public:
    UARTInterface(const std::string& uart_device, int baudrate = B3000000)
        : uart_fd_(-1), uart_device_(uart_device), baudrate_(baudrate) {}

    ~UARTInterface() override {
        close();
    }

    // Open the UART interface
    int open() override {
        uart_fd_ = ::open(uart_device_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (uart_fd_ == -1) {
            std::cerr << "Failed to open UART device: " << uart_device_ << std::endl;
            return -1;
        }

        // Configure UART settings using termios
        struct termios options;
        tcgetattr(uart_fd_, &options);

        // Set the baud rate
        cfsetispeed(&options, baudrate_);
        cfsetospeed(&options, baudrate_);

        // 8N1 mode (8 data bits, no parity, 1 stop bit)
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;

        // Enable the receiver and set local mode
        options.c_cflag |= (CLOCAL | CREAD);

        // Disable hardware flow control
        options.c_cflag &= ~CRTSCTS;

        // Set non-canonical mode (raw input mode)
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

        // Disable software flow control
        options.c_iflag &= ~(IXON | IXOFF | IXANY);

        // Apply the settings
        tcsetattr(uart_fd_, TCSANOW, &options);

        // Flush input buffer
        tcflush(uart_fd_, TCIFLUSH);

        // Send a software reset packet
        uint8_t softreset_pkt[] = {0x7E, 1, 5, 0, 1, 0, 1, 0x7E};
        ::write(uart_fd_, softreset_pkt, sizeof(softreset_pkt));
        usleep(1000);  // Delay to ensure packet is sent

        return 0;
    }

    // Close the UART interface
    void close() override {
        if (uart_fd_ != -1) {
            ::close(uart_fd_);
            uart_fd_ = -1;
        }
    }

    // Read from the UART interface
    int read(uint8_t* pBuffer, unsigned len, uint32_t* t_us) override {
        uint8_t c;
        uint16_t packet_size = 0;

        // Read the start byte (0x7E)
        while (true) {
            if (::read(uart_fd_, &c, 1) == 1 && c == 0x7E) {
                break;
            }
        }

        // Read protocol ID
        if (::read(uart_fd_, &c, 1) != 1 || c != 0x01) {
            return 0;
        }

        // Read the data until the stop byte (0x7E)
        while (packet_size < len) {
            if (::read(uart_fd_, &c, 1) == 1) {
                if (c == 0x7E) break;

                if (c == 0x7D) {
                    // Escape sequence
                    ::read(uart_fd_, &c, 1);
                    c ^= 0x20;
                }
                pBuffer[packet_size++] = c;
            }
        }

        *t_us = getTimeUs();
        return packet_size;
    }

    // Write to the UART interface
    int write(uint8_t* pBuffer, unsigned len) override {
        uint8_t c;

        // Write the start byte
        ::write(uart_fd_, "\x7E", 1);

        // Write protocol ID
        ::write(uart_fd_, "\x01", 1);

        // Write data
        for (unsigned i = 0; i < len; ++i) {
            c = pBuffer[i];
            if (c == 0x7E || c == 0x7D) {
                ::write(uart_fd_, "\x7D", 1);  // Escape sequence
                c ^= 0x20;
            }
            ::write(uart_fd_, &c, 1);
        }

        // Write the stop byte
        ::write(uart_fd_, "\x7E", 1);

        return len;
    }

private:
    int uart_fd_;
    std::string uart_device_;
    int baudrate_;
};

#endif // UART_INTERFACE_HPP
