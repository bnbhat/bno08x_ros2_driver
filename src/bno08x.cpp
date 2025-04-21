#include "bno08x_driver/bno08x.hpp"
#include <unistd.h>
#include <string.h>
#include <sstream>
#include <iostream>
#include <sys/time.h>

static int8_t _init_pin, _reset_pin; // only for spi

static uint32_t get_time_us(sh2_Hal_t* /*self*/) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint32_t t = tv.tv_sec * 1000000 + tv.tv_usec;
    return t;
}

/**
 * @brief Construct a new BNO08x::BNO08x object
 * 
 * @param comm The communication interface to use
 * 
 */
BNO08x::BNO08x(CommInterface* comm, std::function<void(void*, sh2_SensorValue_t*)> sensor_callback, 
                  void *cookie) : comm_(comm), cookie_(cookie), host_callback_(sensor_callback) {};

/**
 * @brief Destroy the BNO08x::BNO08x object
 *
 */
BNO08x::~BNO08x(void) {
  sh2_close();
  comm_->close();
}

/*!   @brief Initialize the sensor
 *    @param  sensor_id
 *            The unique ID to differentiate the sensors from others
 *    @return True if initialization was successful, otherwise false.
 */
bool BNO08x::begin(int32_t sensor_id) {
    if (comm_ == nullptr) {
        std::cerr << "Communication interface not initialized!" << std::endl;
        return false;
    }

    HAL_.cookie = this;
    HAL_.open = open_wrapper;
    HAL_.close = close_wrapper;
    HAL_.read = read_wrapper;
    HAL_.write = write_wrapper;
    HAL_.getTimeUs = get_time_us;
    return init(sensor_id);
}

/*!  @brief Initializer for post i2c/spi init
 *   @param sensor_id Optional unique ID for the sensor set
 *   @returns True if chip identified and initialized
 */
bool BNO08x::init(int32_t sensor_id) {
  int status;

  hardware_reset();

  // Open SH2 interface (also registers non-sensor event handler.)
  status = sh2_open(&HAL_, hal_callback, this);
  if (status != SH2_OK) {
    return false;
    std::cerr << "BNO08x - Failed to open SH2 interface" << std::endl;
  }

  // Check connection partially by getting the product id's
  memset(&prodIds, 0, sizeof(prodIds));
  status = sh2_getProdIds(&prodIds);
  if (status != SH2_OK) {
    std::cerr << "BNO08x - Failed to get product IDs" << std::endl;
    return false;
  }

  DEBUG_ONLY({
    std::cout << "Product Info:" << std::endl;
    for(int i = 0; i < prodIds.numEntries; i++){
      std::cout << "Part: " << prodIds.entry[i].swPartNumber<< std::endl;
      std::cout << "Build: " << prodIds.entry[i].swBuildNumber<< std::endl;
      std::cout << "Version: " << prodIds.entry[i].swVersionMajor<< "." << 
                                  prodIds.entry[i].swVersionMinor << "." << 
                                  prodIds.entry[i].swVersionPatch << std::endl;
    }
  });

  // Register sensor listener
  status = sh2_setSensorCallback(sensor_event_callback, this);
  if (status != SH2_OK) {
    return false;
    std::cerr << "BNO08x - Failed to set sensor callback" << std::endl;
  }

  return true;
}

/**
 * @brief Callback for sensor events
 * This function is called when a sensor event is received
 * sh2_service() must be called periodically to get the buffered sensor events
 * 
 * @param cookie The cookie to pass to the callback
 * @param event The sensor event
 */
inline void BNO08x::sensor_event_callback(void *cookie, sh2_SensorEvent_t *event) {
  BNO08x *instance = static_cast<BNO08x*>(cookie);
  if (instance == nullptr) {
      std::cerr << "BNO08x - Error: cookie is null" << std::endl;
      return;
  }
  
  sh2_SensorValue_t sensor_value;
  int rc = sh2_decodeSensorEvent(&sensor_value, event);
  if (rc != SH2_OK) {
    std::cerr << "BNO08x - Error decoding sensor event" << std::endl;
    return;
  }

  instance->host_callback_(cookie, &sensor_value);
}

/**
 * @brief Callback for asynchronous events
 * This function is called when an asynchronous event is received
 * 
 * @param cookie The cookie to pass to the callback
 * @param pEvent The asynchronous event
 */
void BNO08x::hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
  BNO08x *instance = static_cast<BNO08x*>(cookie);
  if (instance == nullptr) {
      std::cerr << "BNO08x - Error: cookie is null" << std::endl;
      return;
  }

  if (pEvent->eventId == SH2_RESET) {
    instance->reset_occurred_ = true;
  }
}

/**
 * @brief Wrapper for the open function in the HAL
 * 
 * @param HAL The SH2 HAL struct
 */
inline int BNO08x::open_wrapper(sh2_Hal_t * HAL){
    return static_cast<BNO08x*>(HAL->cookie)->comm_->open();
}

/**
 * @brief Wrapper for the close function in the HAL
 * 
 * @param HAL The SH2 HAL struct
 */
inline void BNO08x::close_wrapper(sh2_Hal_t * HAL){
    static_cast<BNO08x*>(HAL->cookie)->comm_->close();
}

/**
 * @brief Wrapper for the read function in the HAL
 * 
 * @param HAL The SH2 HAL struct
 * @param pBuffer The buffer to read into
 * @param len The length of the buffer
 * @param t_us The timestamp
 */
inline int BNO08x::read_wrapper(sh2_Hal_t * HAL, uint8_t *pBuffer, unsigned len, uint32_t *t_us){
    return static_cast<BNO08x*>(HAL->cookie)->comm_->read(pBuffer, len, t_us);
}

/**
 * @brief Wrapper for the write function in the HAL
 * 
 * @param HAL The SH2 HAL struct
 * @param pBuffer The buffer to write
 * @param len The length of the buffer
 */
inline int BNO08x::write_wrapper(sh2_Hal_t * HAL, uint8_t *pBuffer, unsigned len){
    return static_cast<BNO08x*>(HAL->cookie)->comm_->write(pBuffer, len);
}

/**
 * @brief Reset the device using the Reset pin
 *
 */
void BNO08x::hardware_reset(void) { return; }

/**
 * @brief Check if a reset has occured
 *
 * @return true: a reset has occured false: no reset has occoured
 */
bool BNO08x::was_reset(void) {
  bool x = reset_occurred_;
  reset_occurred_ = false;

  return x;
}

/**
 * @brief Poll the sensor for new events
 * 
 * This function must be called periodically to get the buffered sensor events
 */
void BNO08x::poll() {
  sh2_service();
}

/**
 * @brief Enable the given report type
 *
 * @param sensorId The report ID to enable
 * @param interval_us The update interval for reports to be generated, in
 * microseconds
 * @return true: success false: failure
 */
bool BNO08x::enable_report(sh2_SensorId_t sensorId,
                                   uint32_t interval_us) {
  static sh2_SensorConfig_t config;

  // These sensor options are disabled or not used in most cases
  config.changeSensitivityEnabled = false;
  config.wakeupEnabled = false;
  config.changeSensitivityRelative = false;
  config.alwaysOnEnabled = false;
  config.changeSensitivity = 0;
  config.batchInterval_us = 0;
  config.sensorSpecific = 0;

  config.reportInterval_us = interval_us;
  int status = sh2_setSensorConfig(sensorId, &config);

  if (status != SH2_OK) {
    return false;
  }

  return true;
}