#ifndef SPI_INTERFACE_HPP
#define SPI_INTERFACE_HPP

#include <stdexcept>
#include "comm_interface.hpp"

/**
 * @brief SPI communication interface (not implemented)
 */
class SPIInterface : public CommInterface {
public:
    SPIInterface(const std::string& spi_device)
        : spi_device_(spi_device) {
        throw std::runtime_error("SPI Interface not implemented");
    }

    int open() override {
        throw std::runtime_error("SPI Interface not implemented");
    }

    void close() override {
        throw std::runtime_error("SPI Interface not implemented");
    }

    int read(uint8_t *pBuffer, unsigned len, uint32_t *t_us) override {
        throw std::runtime_error("SPI Interface not implemented");
    }

    int write(uint8_t *pBuffer, unsigned len) override {
        throw std::runtime_error("SPI Interface not implemented");
    }

private:
    std::string spi_device_;
};

#endif // SPI_INTERFACE_HPP