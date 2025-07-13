#ifndef UART_INTERFACE_HPP
#define UART_INTERFACE_HPP

#include <stdexcept>
#include "comm_interface.hpp"

/**
 * @brief UART communication interface (not implemented)
 */
class UARTInterface : public CommInterface {
public:
    UARTInterface(const std::string& uart_device)
        : uart_device_(uart_device) {
        throw std::runtime_error("UART Interface not implemented");
    }

    int open() override {
        throw std::runtime_error("UART Interface not implemented");
    }

    void close() override {
        throw std::runtime_error("UART Interface not implemented");
    }

    int read(uint8_t *pBuffer, unsigned len, uint32_t *t_us) override {
        throw std::runtime_error("UART Interface not implemented");
    }

    int write(uint8_t *pBuffer, unsigned len) override {
        throw std::runtime_error("UART Interface not implemented");
    }

private:
    std::string uart_device_;
    int baudrate_ = 3000000;
};

#endif // UART_INTERFACE_HPP
