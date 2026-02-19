#include "bno08x_driver/bno08x_ros.hpp"
#include "bno08x_driver/i2c_interface.hpp"
#include "bno08x_driver/uart_interface.hpp"
#include "bno08x_driver/spi_interface.hpp"

constexpr uint8_t ROTATION_VECTOR_RECEIVED = 0x01;
constexpr uint8_t ACCELEROMETER_RECEIVED   = 0x02;
constexpr uint8_t GYROSCOPE_RECEIVED       = 0x04;

BNO08xROS::BNO08xROS()
    : Node("bno08x_ros")
{
    init_parameters();
    init_comms();
    init_sensor();

    accuracy_status_ = 0; // default to all sensors having accuracy status of 0 (0=unreliable)

    /*
      Note: using "this->" is optional in this context, but can help clarify that we're accessing member variables and functions.
            Some teams like:
                this->get_logger(), this->now(), this->create_publisher(), etc.  (ROS calls)
                but still no this-> for our own members (imu_msg_, frame_id_, etc.)
            This makes “ROS node calls” stand out while keeping your code readable.
            Using it when disambiguating calls to dependent base classes or in templates does not apply here (non-template node class)
            Our class variables have trailing underscores.
    */

    if (publish_imu_) {
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
        RCLCPP_INFO(this->get_logger(), "IMU Publisher created");
        RCLCPP_INFO(this->get_logger(), "IMU Rate: %d", imu_rate_);
    }

    if (publish_magnetic_field_) {
        mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>(
                             "/magnetic_field", 10);
        RCLCPP_INFO(this->get_logger(), "Magnetic Field Publisher created");
        RCLCPP_INFO(this->get_logger(), "Magnetic Field Rate: %d", magnetic_field_rate_);
    }

    calib_status_publisher_ = this->create_publisher<std_msgs::msg::String>("/imu/calib_status", 10);
    RCLCPP_INFO(this->get_logger(), "Calibration Status Publisher created");
    last_calib_status_publish_time_ = this->get_clock()->now();

    // Poll the sensor at the rate of the fastest sensor
    imu_received_flag_ = 0;

    // as we only fill diagonals, zero out the rest of covariances
    std::fill(std::begin(imu_msg_.orientation_covariance), std::end(imu_msg_.orientation_covariance), 0.0);
    std::fill(std::begin(imu_msg_.linear_acceleration_covariance), std::end(imu_msg_.linear_acceleration_covariance), 0.0);
    std::fill(std::begin(imu_msg_.angular_velocity_covariance), std::end(imu_msg_.angular_velocity_covariance), 0.0);

    std::fill(std::begin(mag_msg_.magnetic_field_covariance), std::end(mag_msg_.magnetic_field_covariance), 0.0);

    const int poll_hz = std::max(imu_rate_, magnetic_field_rate_);
    poll_timer_ = create_wall_timer(
                      std::chrono::duration<double>(1.0 / poll_hz),
                      std::bind(&BNO08xROS::poll_timer_callback, this));

    // Initialize the watchdog timer
    auto timeout = std::chrono::milliseconds(2000);
    watchdog_ = std::make_unique<Watchdog>();
    watchdog_->set_timeout(timeout);
    watchdog_->set_check_interval(timeout / 2);
    watchdog_->set_callback([this]() {
        RCLCPP_ERROR(this->get_logger(), "Watchdog timeout! No data received from sensor. Resetting...");
        reset();
    });
    watchdog_->start();

    RCLCPP_INFO(this->get_logger(), "BNO08X ROS Node started.");
}

BNO08xROS::~BNO08xROS() {
    if (watchdog_)
        watchdog_->stop();
    // unique_ptr will destroy things automatically in reverse member order.
}

/**
 * @brief Initialize the communication interface
 *
 * communication interface based on the parameters
 */
void BNO08xROS::init_comms() {
    bool i2c_enabled, uart_enabled, spi_enabled;
    this->get_parameter("i2c.enabled", i2c_enabled);
    this->get_parameter("uart.enabled", uart_enabled);
    this->get_parameter("spi.enabled", spi_enabled);

    if (i2c_enabled) {
        std::string device;
        std::string address;
        this->get_parameter("i2c.bus", device);
        this->get_parameter("i2c.address", address);
        RCLCPP_INFO(this->get_logger(), "Communication Interface: I2C");
        try {
            comm_interface_ = std::make_unique<I2CInterface>(device, std::stoi(address, nullptr, 16));
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to create I2CInterface: %s", e.what());
            throw std::runtime_error("I2CInterface creation failed");
        }
    } else if (uart_enabled) {
        RCLCPP_INFO(this->get_logger(), "Communication Interface: UART");
        std::string device;
        this->get_parameter("uart.device", device);
        try {
            comm_interface_ = std::make_unique<UARTInterface>(device);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                         "UART Interface not implemented: %s", e.what());
            throw std::runtime_error("UARTInterface creation failed");
        }
    } else if (spi_enabled) {
        RCLCPP_INFO(this->get_logger(), "Communication Interface: SPI");
        std::string device;
        this->get_parameter("spi.device", device);
        try {
            comm_interface_ = std::make_unique<SPIInterface>(device);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                         "SPI Interface not implemented: %s", e.what());
            throw std::runtime_error("SPIInterface creation failed");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "No communication interface enabled!");
        throw std::runtime_error("Communication interface setup failed");
    }
}

/**
 * @brief Initialize the parameters
 *
 * This function initializes the parameters for the node
 *
 */
void BNO08xROS::init_parameters() {
    this->declare_parameter<std::string>("frame_id", "bno085");

    this->declare_parameter<bool>("publish.magnetic_field.enabled", true);
    this->declare_parameter<int>("publish.magnetic_field.rate", 100);
    this->declare_parameter<bool>("publish.imu.enabled", true);
    this->declare_parameter<int>("publish.imu.rate", 100);

    this->declare_parameter<bool>("i2c.enabled", true);
    this->declare_parameter<std::string>("i2c.bus", "/dev/i2c-7");
    this->declare_parameter<std::string>("i2c.address", "0x4A");
    this->declare_parameter<bool>("uart.enabled", false);
    this->declare_parameter<std::string>("uart.device", "/dev/ttyACM0");
    this->declare_parameter<bool>("spi.enabled", false);
    this->declare_parameter<std::string>("spi.device", "/dev/spidev0.0");

    this->get_parameter("frame_id", frame_id_);

    this->get_parameter("publish.magnetic_field.enabled", publish_magnetic_field_);
    this->get_parameter("publish.magnetic_field.rate", magnetic_field_rate_);
    this->get_parameter("publish.imu.enabled", publish_imu_);
    this->get_parameter("publish.imu.rate", imu_rate_);

    // clamp to 1 Hz in case user set rates to 0 or negative
    imu_rate_ = std::max(1, imu_rate_);
    magnetic_field_rate_ = std::max(1, magnetic_field_rate_);

    this->declare_parameter<double>("imu.orientation_yaw_variance", 7.5e-3); //  default 0.0075 (≈5°) means pretty trustworthy

    this->get_parameter("imu.orientation_yaw_variance", orientation_yaw_variance_);
    this->declare_parameter<bool>("verbose", false);
    this->get_parameter("verbose", verbose_);

    this->declare_parameter<bool>("scale_covariance_by_calib", false);
    this->get_parameter("scale_covariance_by_calib", scale_covariance_by_calib_);
}

/**
 * @brief Initialize the sensor
 *
 * This function initializes the sensor and enables the required sensor reports
 *
 */
void BNO08xROS::init_sensor() {

    try {
        bno08x_ = std::make_unique<BNO08x>(
            comm_interface_.get(),  // raw pointer
            std::bind(&BNO08xROS::sensor_callback, this,
            std::placeholders::_1, std::placeholders::_2), this
        );
    } catch (const std::bad_alloc& e) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to allocate memory for BNO08x object: %s", e.what());
        throw std::runtime_error("BNO08x object allocation failed");
    }

    if (!bno08x_->begin()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize BNO08X sensor");
        throw std::runtime_error("BNO08x initialization failed");
    }

    if (publish_magnetic_field_) {
        if(!bno08x_->enable_report(SH2_MAGNETIC_FIELD_CALIBRATED,
                                   1000000/magnetic_field_rate_)) {  // Hz to us
            RCLCPP_ERROR(this->get_logger(), "Failed to enable magnetic field sensor");
        }
    }
    if (publish_imu_) {
        if(!bno08x_->enable_report(SH2_ROTATION_VECTOR,
                                   1000000/imu_rate_)) {             // Hz to us
            RCLCPP_ERROR(this->get_logger(), "Failed to enable rotation vector sensor");
        }
        if(!bno08x_->enable_report(SH2_ACCELEROMETER,
                                   1000000/imu_rate_)) {             // Hz to us
            RCLCPP_ERROR(this->get_logger(), "Failed to enable accelerometer sensor");
        }
        if(!bno08x_->enable_report(SH2_GYROSCOPE_CALIBRATED,
                                   1000000/imu_rate_)) {             // Hz to us
            RCLCPP_ERROR(this->get_logger(), "Failed to enable gyroscope sensor");
        }
    }
    if (!(publish_imu_ || publish_magnetic_field_)) {
        RCLCPP_ERROR(this->get_logger(), "No sensor reports enabled! Exiting...");
        throw std::runtime_error("No sensor reports enabled");
    }
}

/**
 * @brief Get the sensor name from the sensor ID
 *
 * @param sensor_id The sensor ID from the BNO08x sensor event
 * @return std::string The human-readable name of the sensor
 */
std::string BNO08xROS::sensor_name(uint8_t sensor_id)
{
    switch(sensor_id) {
    case SH2_MAGNETIC_FIELD_CALIBRATED:
        return "Calibrated Magnetic Field";
    case SH2_ROTATION_VECTOR:
        return "Rotation Vector";
    case SH2_ACCELEROMETER:
        return "Accelerometer";
    case SH2_GYROSCOPE_CALIBRATED:
        return "Calibrated Gyroscope";
    default:
        return "Unknown Sensor";
    }
}

/**
 * @brief Convert the accuracy status bitfield to a human-readable JSON string
 *
 * The accuracy_status_ variable is a bitfield where:
 *   bits 0-1: Mag accuracy
 *   bits 2-3: Accel accuracy
 *   bits 4-5: Gyro accuracy
 *   bits 6-7: Rotation Vector (system) accuracy
 *
 * Each sensor's accuracy is represented as:
 *   0 - Unreliable
 *   1 - Accuracy low
 *   2 - Accuracy medium
 *   3 - Accuracy high
 *
 * @return std::string A JSON-like string representing the accuracy of each sensor
 */
std::string BNO08xROS::accuracy_status_string()
{
    acc_stat_t orient = (accuracy_status_ >> 6) & 0x03; // bits 6-7
    acc_stat_t gyro = (accuracy_status_ >> 4) & 0x03;   // bits 4-5
    acc_stat_t accel = (accuracy_status_ >> 2) & 0x03;  // bits 2-3
    acc_stat_t mag = accuracy_status_ & 0x03;           // bits 0-1

    std::string result = "{";
    result += "\"sys\":" + std::to_string(orient) + ",";
    result += "\"gyro\":" + std::to_string(gyro) + ",";
    result += "\"accel\":" + std::to_string(accel) + ",";
    result += "\"mag\":" + std::to_string(mag);
    result += "}";

    return result;
}

/**
 * @brief Calculate covariance scaling factor based on sensor accuracy
 *
 * @param accuracy The accuracy status (0=Unreliable, 1=Low, 2=Medium, 3=High)
 * @return float Scaling factor to apply to base covariances
 */
float BNO08xROS::get_covariance_scaled(float base_variance, acc_stat_t accuracy) {

    if (!scale_covariance_by_calib_) {
        return base_variance;  // No scaling, return base variance
    }

    switch(accuracy) {
    case 3:
        return base_variance;            // High accuracy - base covariance (no scaling)
    case 2:
        return 5.0f * base_variance;     // Medium accuracy - 5x base covariance
    case 1:
        return 25.0f * base_variance;    // Low accuracy - 25x base covariance
    case 0:
    default:
        // Huge variance => EKF ignores it without special-case semantics.
        // Works better than -1 at [0] which can cause issues in some implementations.
        return 1e6f;
    }
}

/**
 * @brief Callback function for sensor events
 *
 * @param cookie Pointer to the object that called the function, not used here
 * @param sensor_value The sensor value from parsing the sensor event buffer
 *
 */
void BNO08xROS::sensor_callback(void *cookie, sh2_SensorValue_t *sensor_value) {
    DEBUG_LOG("Sensor Callback");
    watchdog_->reset();

    if (!rclcpp::ok()) return;

    const auto now = this->now();
    uint8_t sensor_id = sensor_value->sensorId;

    if (sensor_id == SH2_ROTATION_VECTOR || sensor_id == SH2_ACCELEROMETER || sensor_id == SH2_GYROSCOPE_CALIBRATED) {
        // one of the values that goes into imu_data bundle, so manage bundle timing and flags
        if (!imu_bundle_active_) {
            // Start bundle timing if this is the first component
            imu_bundle_active_ = true;
            imu_bundle_start_time_ = now;
            imu_bundle_stamp_ = now;
        } else if ((now - imu_bundle_start_time_).seconds() >= IMU_BUNDLE_TIMEOUT_SEC) {
            // If bundle takes too long, treat the current message as “first of a new bundle”
            RCLCPP_WARN(this->get_logger(), "IMU data bundle timeout. flag=0x%02x. Restarting bundle.", imu_received_flag_);
            imu_received_flag_ = 0;
            imu_bundle_active_ = true;
            imu_bundle_start_time_ = now;
            imu_bundle_stamp_ = now;
            // continue processing current message as first element of the new bundle
        }
    }

    /* Status of a sensor
    *   0 - Unreliable
    *   1 - Accuracy low
    *   2 - Accuracy medium
    *   3 - Accuracy high
    */
    acc_stat_t sensor_accuracy = static_cast<acc_stat_t>(sensor_value->status & 0x03); // Extract accuracy bits (1-0) for "accuracy_status_" mask updating and covariance scaling

    // Note: we must provide realistic covariances for all fields in the Imu message,
    //       see https://chatgpt.com/s/t_691b60f38e1c8191a0a309cbcf99e478

    switch(sensor_id) {
    case SH2_MAGNETIC_FIELD_CALIBRATED:
        accuracy_status_ = (accuracy_status_ & ~MAG_MASK) | (sensor_accuracy << 0); // Update bits 0-1 for Mag accuracy
        if (publish_magnetic_field_ && sensor_accuracy > 0) { // Only publish if magnetic field report is enabled and accuracy is not unreliable
            float to_tesla = 1e-6f; // Convert microTesla to Tesla
            mag_msg_.magnetic_field.x = sensor_value->un.magneticField.x * to_tesla;
            mag_msg_.magnetic_field.y = sensor_value->un.magneticField.y * to_tesla;
            mag_msg_.magnetic_field.z = sensor_value->un.magneticField.z * to_tesla;
            mag_msg_.header.frame_id = frame_id_;
            mag_msg_.header.stamp = now;
            // IMU will still return infrequent magnetic field reports even if the report
            // was not enabled, so check it was enabled before publishing.

            float base_mag_var = 1e-11f; // Base variance for magnetic field, 1e-11 (stddev ~3.2 µT).
            mag_msg_.magnetic_field_covariance[0] = get_covariance_scaled(base_mag_var, sensor_accuracy);
            mag_msg_.magnetic_field_covariance[4] = get_covariance_scaled(base_mag_var, sensor_accuracy);
            mag_msg_.magnetic_field_covariance[8] = get_covariance_scaled(base_mag_var, sensor_accuracy);

            mag_publisher_->publish(mag_msg_);
        }
        break;

    case SH2_ROTATION_VECTOR:
        accuracy_status_ = (accuracy_status_ & ~RV_MASK) | (sensor_accuracy << 6); // Update bits 6-7 for Rotation Vector accuracy
        // RAW quaternion from BNO08x (as ROS2 requires it, in REP-103 ENU reference frame):
        imu_msg_.orientation.x = sensor_value->un.rotationVector.i;
        imu_msg_.orientation.y = sensor_value->un.rotationVector.j;
        imu_msg_.orientation.z = sensor_value->un.rotationVector.k;
        imu_msg_.orientation.w = sensor_value->un.rotationVector.real;

        // Add orientation covariance scaled by accuracy:
        imu_msg_.orientation_covariance[0] = get_covariance_scaled(3e-4f, sensor_accuracy);  // roll
        imu_msg_.orientation_covariance[4] = get_covariance_scaled(3e-4f, sensor_accuracy);  // pitch
        imu_msg_.orientation_covariance[8] = get_covariance_scaled(static_cast<float>(orientation_yaw_variance_), sensor_accuracy);  // yaw

        imu_received_flag_ |= ROTATION_VECTOR_RECEIVED;
        break;

    case SH2_ACCELEROMETER: {
        accuracy_status_ = (accuracy_status_ & ~ACC_MASK) | (sensor_accuracy << 2); // Update bits 2-3 for Accel accuracy
        imu_msg_.linear_acceleration.x = sensor_value->un.accelerometer.x;
        imu_msg_.linear_acceleration.y = sensor_value->un.accelerometer.y;
        imu_msg_.linear_acceleration.z = sensor_value->un.accelerometer.z;

        // acceleration covariance scaled by accuracy
        float base_accel_var = 0.04f; // 0.04 (stddev ~0.2 m/s²) is reasonable.
        imu_msg_.linear_acceleration_covariance[0] = get_covariance_scaled(base_accel_var, sensor_accuracy);
        imu_msg_.linear_acceleration_covariance[4] = get_covariance_scaled(base_accel_var, sensor_accuracy);
        imu_msg_.linear_acceleration_covariance[8] = get_covariance_scaled(base_accel_var, sensor_accuracy);

        imu_received_flag_ |= ACCELEROMETER_RECEIVED;
        break;
    }

    case SH2_GYROSCOPE_CALIBRATED: {
        // TODO: it looks like gyro accuracy is always 0? does 0 indicate "unavailable"?
        accuracy_status_ = (accuracy_status_ & ~GYR_MASK) | (sensor_accuracy << 4); // Update bits 4-5 for Gyro accuracy
        imu_msg_.angular_velocity.x = sensor_value->un.gyroscope.x;
        imu_msg_.angular_velocity.y = sensor_value->un.gyroscope.y;
        imu_msg_.angular_velocity.z = sensor_value->un.gyroscope.z;

        // gyro covariance scaled by accuracy.
        // Hack: if gyro accuracy is unavailable (0), fall back to rotation-vector (system) accuracy, then accel, then mag.
        float base_gyro_var = 5e-4f; // (stddev ~0.022 rad/s) is reasonable;
        acc_stat_t eff_acc = sensor_accuracy;
        if (eff_acc == 0) {
            // try rotation vector (system) accuracy (bits 6-7)
            eff_acc = (accuracy_status_ >> 6) & 0x03;
        }
        if (eff_acc == 0) {
            // try accel accuracy (bits 2-3)
            eff_acc = (accuracy_status_ >> 2) & 0x03;
        }
        if (eff_acc == 0) {
            // try mag accuracy (bits 0-1)
            eff_acc = accuracy_status_ & 0x03;
        }

        // if (verbose_ && eff_acc != sensor_accuracy) {
        //     RCLCPP_INFO(this->get_logger(), "Gyro accuracy missing; falling back to accuracy=%d", eff_acc);
        // }

        imu_msg_.angular_velocity_covariance[0] = get_covariance_scaled(base_gyro_var, eff_acc);
        imu_msg_.angular_velocity_covariance[4] = get_covariance_scaled(base_gyro_var, eff_acc);
        imu_msg_.angular_velocity_covariance[8] = get_covariance_scaled(base_gyro_var, eff_acc);

        imu_received_flag_ |= GYROSCOPE_RECEIVED;
        break;
    }
    default:
        break;
    }

    // Publish only when all three reports are ready and are from the same bundle
    // (i.e. received since last publish and within a short time window of each other)
    if (publish_imu_
            && imu_publisher_
            && imu_received_flag_ == (ROTATION_VECTOR_RECEIVED | ACCELEROMETER_RECEIVED | GYROSCOPE_RECEIVED))
    {
        imu_msg_.header.frame_id = frame_id_;
        imu_msg_.header.stamp = imu_bundle_stamp_;  // time of the first report in the bundle, for better synchronization
        imu_publisher_->publish(imu_msg_);

        imu_received_flag_ = 0;
        imu_bundle_active_ = false;   // ready for next bundle

        // Publish calibration status approximately once per second using elapsed time.
        // We do it here after processing an IMU bundle to ensure we have the latest accuracy status.
        if ((now - last_calib_status_publish_time_).seconds() >= 1.0) {
            calib_msg_.data = accuracy_status_string();
            calib_status_publisher_->publish(calib_msg_);
            last_calib_status_publish_time_ = now;

            if(verbose_) {
                const int orient = (accuracy_status_ >> 6) & 0x03;  // bits 6-7
                const int gyro   = (accuracy_status_ >> 4) & 0x03;  // bits 4-5
                const int accel  = (accuracy_status_ >> 2) & 0x03;  // bits 2-3
                const int mag    =  accuracy_status_       & 0x03;  // bits 0-1

                if(orient == 0 || gyro == 0 || accel == 0 || mag == 0) {
                    RCLCPP_WARN(this->get_logger(), "IMU calibration status - Sys: %d, Gyro: %d, Accel: %d, Mag: %d (0=unreliable)", orient, gyro, accel, mag);
                }

                /*
                // Log warnings for any sensors that are currently unreliable:
                if(sensor_accuracy == 0) {
                    RCLCPP_WARN(this->get_logger(), "UNRELIABLE accuracy sensor ID: %s", sensor_name(sensor_id).c_str());
                } else if (sensor_accuracy == 1) {
                    RCLCPP_INFO(this->get_logger(), "LOW accuracy sensor ID: %s", sensor_name(sensor_id).c_str());
                //} else if (sensor_accuracy == 2) {
                //    RCLCPP_INFO(this->get_logger(), "MEDIUM accuracy sensor ID: %s", sensor_name(sensor_id).c_str());
                //} else if (sensor_accuracy == 3) {
                //    RCLCPP_INFO(this->get_logger(), "HIGH accuracy sensor ID: %s", sensor_name(sensor_id).c_str());
                }
                */
            }

            // reset accuracy status to 0 (unreliable) after publishing,
            // it will be updated by next sensor callbacks, and fully filled by the time IMU bundle is published.
            accuracy_status_ = 0;
        }
    }
}

/**
 * @brief Poll the sensor for new events
 *
 * This function is called periodically at the rate of the fastest sensor report
 * to get the buffered sensor events
 * called by the poll_timer_ timer
 */
void BNO08xROS::poll_timer_callback() {
    {
        std::lock_guard<std::mutex> lock(bno08x_mutex_);
        bno08x_->poll();
    }
}

void BNO08xROS::reset() {
    std::lock_guard<std::mutex> lock(bno08x_mutex_);
    imu_received_flag_ = 0;
    imu_bundle_active_ = false;
    accuracy_status_ = 0;
    bno08x_.reset();   // deletes current object (if any)
    init_sensor();     // will assign a new one via make_unique
}
