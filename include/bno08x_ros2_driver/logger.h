#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sstream>

// Define DEBUG_LOG_ENABLED to enable debug logs at compile time
//#define DEBUG_LOG_ENABLED

#ifdef DEBUG_LOG_ENABLED
    #define DEBUG_ONLY(code) \
        do { \
            std::cout << "DEBUG: "<< std::endl; \
            code \
            } while (0)  // End DEBUG_ONLY

    #define DEBUG_LOG(message) \
        do { \
            std::ostringstream oss; \
            oss << message; \
            std::cout << "DEBUG: " << oss.str() << std::endl; \
        } while (0) // End DEBUG_LOG

    #define DEBUG_LOG_BUFFER(buffer, length) \
        do { \
            std::cout << "DEBUG BUFFER: "; \
            for (size_t i = 0; i < length; ++i) { \
                std::cout << std::hex << static_cast<int>(buffer[i]) << " "; \
            } \
            std::cout << std::dec << std::endl; /* Reset to default formatting */ \
        } while (0) // End DEBUG_LOG_BUFFER
#else
    // When DEBUG_LOG_ENABLED is not defined, compile out debug logs
    #define DEBUG_ONLY(code) do { } while (0) // End DEBUG_ONLY (No-op)
    #define DEBUG_LOG(message) do { } while (0) // End DEBUG_LOG (No-op)
    #define DEBUG_LOG_BUFFER(buffer, length) do { } while (0) // End DEBUG_LOG_BUFFER (No-op)
#endif

#endif // LOGGER_HPP
