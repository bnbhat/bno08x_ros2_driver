#ifndef I2C_INTERFACE_HPP
#define I2C_INTERFACE_HPP

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "comm_interface.hpp"

/**
 * @brief I2C communication interface
 * This class implements the I2C communication interface for the BNO08x sensor.
 */
class I2CInterface : public CommInterface {
public:
    I2CInterface(const char* i2c_bus_file, uint8_t i2c_addr)
        : i2c_fd(-1), i2c_bus_file(i2c_bus_file), i2c_addr(i2c_addr) {
            DEBUG_LOG("[BNO08x] I2C Interface Created");
        }

    int open() override final {
        // Open the I2C bus
        i2c_fd = ::open(i2c_bus_file, O_RDWR);
        if (i2c_fd < 0) {
            std::cerr << "[BNO08x] Failed to open the I2C bus" << std::endl;
            return -1;
        }

        // Acquire bus access and set the I2C address
        if (ioctl(i2c_fd, I2C_SLAVE, i2c_addr) < 0) {
            std::cerr << "[BNO08x] Failed to acquire bus access and/or talk to slave" << std::endl;
            ::close(i2c_fd);
            return -1;
        }

        // Send a soft reset packet
        uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};  // Soft reset packet
        bool success = false;
        for (uint8_t attempts = 0; attempts < 5; ++attempts) {
            DEBUG_LOG("[BNO08x] Sending soft reset packet to the sensor");
            if (::write(i2c_fd, softreset_pkt, sizeof(softreset_pkt)) == sizeof(softreset_pkt)) {
                success = true;
                break;
            }
            usleep(30000);  // Delay of 30ms between attempts
        }

        if (!success) {
            std::cerr << "[BNO08x] Failed to send soft reset packet to the sensor" << std::endl;
            ::close(i2c_fd);
            return -1;
        }

        usleep(300000);  // Delay for 300ms after the reset

        DEBUG_LOG("[BNO08x] I2C Comm Opened and Soft Reset Sent");
        return 0;
    }

    void close() override final {
        if (i2c_fd >= 0) {
            ::close(i2c_fd);
            i2c_fd = -1;
            DEBUG_LOG("[BNO08x] I2C Comm Closed");
        }
    }

    int read(uint8_t *pBuffer, unsigned len, uint32_t *t_us) override final {
        DEBUG_LOG("[BNO08x] I2C Comm Read");
        uint8_t header[4];
        if (::read(i2c_fd, header, 4) != 4) {
            return 0;
        }

        uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
        packet_size &= ~0x8000;

        DEBUG_LOG("[BNO08x] Packet size: " << packet_size);
        DEBUG_LOG_BUFFER(header, 4);

        if (packet_size > len) {
            return 0;
        }

        uint16_t cargo_remaining = packet_size;
        uint8_t i2c_buffer[32];
        uint16_t read_size;
        uint16_t cargo_read_amount = 0;
        bool first_read = true;

        while (cargo_remaining > 0) {
            read_size = std::min((size_t)32, (size_t)cargo_remaining + (first_read ? 0 : 4));
            if (::read(i2c_fd, i2c_buffer, read_size) != read_size) {
                return 0;
            }

            if (first_read) {
                cargo_read_amount = read_size;
                memcpy(pBuffer, i2c_buffer, cargo_read_amount);
                first_read = false;
            } else {
                cargo_read_amount = read_size - 4;
                memcpy(pBuffer, i2c_buffer + 4, cargo_read_amount);
            }

            pBuffer += cargo_read_amount;
            cargo_remaining -= cargo_read_amount;
        }

        *t_us = getTimeUs();
        return packet_size;
    }

    int write(uint8_t *pBuffer, unsigned len) override final{
        size_t write_size = std::min((size_t)32, (size_t)len);
        if (::write(i2c_fd, pBuffer, write_size) != write_size) {
            return 0;
        }
        return write_size;
    }

private:
    int i2c_fd;
    const char* i2c_bus_file;
    uint8_t i2c_addr;
};

#endif // I2C_INTERFACE_HPP