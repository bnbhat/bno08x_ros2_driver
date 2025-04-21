#ifndef COMM_INTERFACE_HPP
#define COMM_INTERFACE_HPP

#include <cstdint>
#include <sys/time.h> 
#include "logger.h"
/**
 * @brief Interface for the communication with the BNO08x sensor
 * This class defines the interface for the communication with the BNO08x sensor.
 * It is used by the BNO08x class to abstract
 * the communication with the sensor, so that the BNO08x class can be used with
 * different communication interfaces (I2C, UART, SPI).
 */
class CommInterface {
public:
    virtual ~CommInterface() {}
    virtual int open() = 0;
    virtual void close() = 0;
    virtual int read(uint8_t *pBuffer, unsigned len, uint32_t *t_us) = 0;
    virtual int write(uint8_t *pBuffer, unsigned len) = 0;
    
    uint32_t getTimeUs() {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        uint32_t t = tv.tv_sec * 1000000 + tv.tv_usec;
        return t;
    }
};

#endif //COMM_INTERFACE_HPP
