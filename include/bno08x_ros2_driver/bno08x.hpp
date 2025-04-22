#ifndef BNO08X_HPP
#define BNO08X_HPP

#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <functional> 
#include "sh2/sh2.h"
#include "sh2/sh2_SensorValue.h"
#include "sh2/sh2_err.h"
#include "bno08x_ros2_driver/comm_interface.hpp"
#include "bno08x_ros2_driver/logger.h"

/* Additional Activities not listed in SH-2 lib */
#define PAC_ON_STAIRS 8 ///< Activity code for being on stairs
#define PAC_OPTION_COUNT 9 ///< The number of current options for the activity classifier

/**
 * @brief  Class that stores state and functions for interacting with
 *         the BNO08x 9-DOF Orientation IMU Fusion Breakout
 */
class BNO08x {
public:
  BNO08x(CommInterface *comm, 
          std::function<void(void*, sh2_SensorValue_t*)> sensor_callback, void *cookie);
  ~BNO08x();

  bool begin(int32_t sensor_id = 0);
  void hardware_reset(void);
  bool was_reset(void);
  bool enable_report(sh2_SensorId_t sensor, uint32_t interval_us = 10000);
  void poll(void);
  sh2_ProductIds_t prodIds; ///< The product IDs returned by the sensor

protected:
  bool init(int32_t sensor_id);
  sh2_Hal_t HAL_; // The struct representing the SH2 Hardware Abstraction Layer

private:
  static inline void sensor_event_callback(void *cookie, sh2_SensorEvent_t *event);
  static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);
  static inline int open_wrapper(sh2_Hal_t* HAL);
  static inline void close_wrapper(sh2_Hal_t* HAL);
  static inline int read_wrapper(sh2_Hal_t* HAL, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
  static inline int write_wrapper(sh2_Hal_t* HAL, uint8_t *pBuffer, unsigned len);
  CommInterface* comm_;
  void* cookie_;
  std::function<void(void*, sh2_SensorValue_t*)> host_callback_;
  bool reset_occurred_;
};

#endif // BNO08X_HPP