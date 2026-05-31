#include "bno08x_driver/bno08x_ros.hpp"
#include "bno08x_driver/i2c_interface.hpp"
#include "bno08x_driver/uart_interface.hpp"
#include "bno08x_driver/spi_interface.hpp"

#include <cmath>

constexpr uint8_t ROTATION_VECTOR_RECEIVED = 0x01;
constexpr uint8_t ACCELEROMETER_RECEIVED   = 0x02;
constexpr uint8_t GYROSCOPE_RECEIVED       = 0x04;

// SH-2 report status field: bits 1-0 carry the calibration accuracy level.
constexpr uint8_t SH2_STATUS_ACCURACY_MASK = 0x03;
constexpr uint8_t SH2_ACCURACY_UNRELIABLE  = 0;
constexpr uint8_t SH2_ACCURACY_LOW         = 1;
constexpr uint8_t SH2_ACCURACY_MEDIUM      = 2;
constexpr uint8_t SH2_ACCURACY_HIGH        = 3;

BNO08xROS::BNO08xROS()
    : Node("bno08x_ros")
{  
    this->init_parameters();
    this->init_comms();
    this->init_sensor();

    if (publish_imu_) {
        this->init_imu_covariance();
        this->imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
        RCLCPP_INFO(this->get_logger(), "IMU Publisher created");
        RCLCPP_INFO(this->get_logger(), "IMU Rate: %d", imu_rate_);
    }

    if (publish_magnetic_field_) {
        mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>(
                                                                        "/magnetic_field", 10);
        RCLCPP_INFO(this->get_logger(), "Magnetic Field Publisher created");
        RCLCPP_INFO(this->get_logger(), "Magnetic Field Rate: %d", magnetic_field_rate_);
    }

    if (publish_game_rv_) {
        // angular_velocity and linear_acceleration are not provided on this topic.
        // A covariance[0] = -1 signals "field not populated" per REP-145.
        game_rv_msg_.angular_velocity_covariance[0] = -1;
        game_rv_msg_.linear_acceleration_covariance[0] = -1;
        game_rv_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/game_rotation", 10);
        RCLCPP_INFO(this->get_logger(), "Game Rotation Vector Publisher created");
        RCLCPP_INFO(this->get_logger(), "Game Rotation Vector Rate: %d", game_rv_rate_);
    }

    if (publish_geo_rv_) {
        geo_rv_msg_.angular_velocity_covariance[0] = -1;
        geo_rv_msg_.linear_acceleration_covariance[0] = -1;
        geo_rv_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/geomagnetic", 10);
        RCLCPP_INFO(this->get_logger(), "Geomagnetic Rotation Vector Publisher created");
        RCLCPP_INFO(this->get_logger(), "Geomagnetic Rotation Vector Rate: %d", geo_rv_rate_);
    }

    // Poll at the fastest rate of all enabled sensor reports.
    this->imu_received_flag_ = 0;
    int poll_rate_hz = 0;
    if (publish_imu_)            poll_rate_hz = std::max(poll_rate_hz, imu_rate_);
    if (publish_magnetic_field_) poll_rate_hz = std::max(poll_rate_hz, magnetic_field_rate_);
    if (publish_game_rv_)        poll_rate_hz = std::max(poll_rate_hz, game_rv_rate_);
    if (publish_geo_rv_)         poll_rate_hz = std::max(poll_rate_hz, geo_rv_rate_);
    this->poll_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / poll_rate_hz),
        std::bind(&BNO08xROS::poll_timer_callback, this)
    );

    // Initialize the watchdog timer
    auto timeout = std::chrono::milliseconds(2000);
    watchdog_ = new Watchdog();
    watchdog_->set_timeout(timeout);
    watchdog_->set_check_interval(timeout / 2); 
    watchdog_->set_callback([this]() {
        RCLCPP_ERROR(this->get_logger(), "Watchdog timeout! No data received from sensor. Resetting...");
        watchdog_fire_count_++;
        this->reset();
    });
    watchdog_->start();

    diag_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10);
    diag_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&BNO08xROS::publish_diagnostics, this));

    save_cal_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/imu/save_calibration",
        std::bind(&BNO08xROS::save_calibration_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    tare_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/imu/tare",
        std::bind(&BNO08xROS::tare_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    clear_tare_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/imu/clear_tare",
        std::bind(&BNO08xROS::clear_tare_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    set_reorientation_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/imu/set_reorientation",
        std::bind(&BNO08xROS::set_reorientation_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "BNO08X ROS Node started.");
}

BNO08xROS::~BNO08xROS() {
    delete watchdog_;
    delete bno08x_;
    delete comm_interface_;
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
            comm_interface_ = new I2CInterface(device, std::stoi(address, nullptr, 16));
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), 
                    "Failed to create I2CInterface: %s", e.what());
            throw std::runtime_error("I2CInterface creation failed");
        }
    } else if (uart_enabled) {
        RCLCPP_INFO(this->get_logger(), "Communication Interface: UART");
        std::string device;
        this->get_parameter("uart.device", device);
        try{
            comm_interface_ = new UARTInterface(device);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), 
                    "UART Interface not implemented: %s", e.what());
            throw std::runtime_error("UARTInterface creation failed");
        }
    } else if (spi_enabled){
        RCLCPP_INFO(this->get_logger(), "Communication Interface: SPI");
        std::string device;
        this->get_parameter("spi.device", device);
        try {
            comm_interface_ = new SPIInterface(device);
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
    this->declare_parameter<bool>("publish.imu.linear_acceleration_compensated", true);
    this->declare_parameter<bool>("publish.game_rotation_vector.enabled", false);
    this->declare_parameter<int>("publish.game_rotation_vector.rate", 100);
    this->declare_parameter<bool>("publish.geomagnetic_rotation_vector.enabled", false);
    this->declare_parameter<int>("publish.geomagnetic_rotation_vector.rate", 100);
    this->declare_parameter<std::vector<double>>("publish.imu.orientation_covariance", this->default_orientation_covariance_);
    this->declare_parameter<std::vector<double>>("publish.imu.gyrometer_covariance", this->default_gyrometer_covariance_);
    this->declare_parameter<std::vector<double>>("publish.imu.linear_covariance", this->default_linear_covariance_);

    this->declare_parameter<bool>("calibration.auto_save", true);
    // Identity quaternion [x, y, z, w] — no rotation. Set before calling /imu/set_reorientation.
    this->declare_parameter<std::vector<double>>("tare.quaternion", {0.0, 0.0, 0.0, 1.0});

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
    this->get_parameter("publish.imu.linear_acceleration_compensated", linear_acceleration_compensated_);
    this->get_parameter("calibration.auto_save", auto_save_dcd_);
    this->get_parameter("publish.game_rotation_vector.enabled", publish_game_rv_);
    this->get_parameter("publish.game_rotation_vector.rate", game_rv_rate_);
    this->get_parameter("publish.geomagnetic_rotation_vector.enabled", publish_geo_rv_);
    this->get_parameter("publish.geomagnetic_rotation_vector.rate", geo_rv_rate_);

    this->get_parameter("publish.imu.orientation_covariance", orientation_covariance_);
    if (orientation_covariance_.size() != 9) {
        RCLCPP_WARN(this->get_logger(),
            "publish.imu.orientation_covariance must be a 9-element array, using defaults.");
        orientation_covariance_ = default_orientation_covariance_;
    }

    this->get_parameter("publish.imu.gyrometer_covariance", gyrometer_covariance_);
    if (gyrometer_covariance_.size() != 9) {
        RCLCPP_WARN(this->get_logger(),
            "publish.imu.gyrometer_covariance must be a 9-element array, using defaults.");
        gyrometer_covariance_ = default_gyrometer_covariance_;
    }

    this->get_parameter("publish.imu.linear_covariance", linear_covariance_);
    if (linear_covariance_.size() != 9) {
        RCLCPP_WARN(this->get_logger(),
            "publish.imu.linear_covariance must be a 9-element array, using defaults.");
        linear_covariance_ = default_linear_covariance_;
    }
}

/**
 * @brief Initialize the sensor
 * 
 * This function initializes the sensor and enables the required sensor reports
 * 
 */
void BNO08xROS::init_sensor() {

    try {
        bno08x_ = new BNO08x(comm_interface_, std::bind(&BNO08xROS::sensor_callback, this, 
                                        std::placeholders::_1, std::placeholders::_2), this);
    } catch (const std::bad_alloc& e) {
        RCLCPP_ERROR(this->get_logger(), 
                        "Failed to allocate memory for BNO08x object: %s", e.what());
        throw std::runtime_error("BNO08x object allocation failed");
    }

    if (!bno08x_->begin()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize BNO08X sensor");
        throw std::runtime_error("BNO08x initialization failed");
    }

    if (auto_save_dcd_) {
        if (!bno08x_->set_dcd_auto_save(true)) {
            RCLCPP_WARN(this->get_logger(), "Failed to enable DCD auto-save");
        }
    }

    if (publish_magnetic_field_) {
        if(!this->bno08x_->enable_report(SH2_MAGNETIC_FIELD_CALIBRATED, 
                                         1000000/this->magnetic_field_rate_)){   // Hz to us
            RCLCPP_ERROR(this->get_logger(), "Failed to enable magnetic field sensor");
        }
    }
    if (publish_imu_) {
        if(!this->bno08x_->enable_report(SH2_ROTATION_VECTOR, 
                                         1000000/this->imu_rate_)){              // Hz to us
            RCLCPP_ERROR(this->get_logger(), "Failed to enable rotation vector sensor");
        }
        sh2_SensorId_t accel_report = linear_acceleration_compensated_ ?
                                      SH2_LINEAR_ACCELERATION : SH2_ACCELEROMETER;
        if(!this->bno08x_->enable_report(accel_report,
                                         1000000/this->imu_rate_)){              // Hz to us
            RCLCPP_ERROR(this->get_logger(), "Failed to enable accelerometer sensor");
        }
        if(!this->bno08x_->enable_report(SH2_GYROSCOPE_CALIBRATED, 
                                         1000000/this->imu_rate_)){              // Hz to us
            RCLCPP_ERROR(this->get_logger(), "Failed to enable gyroscope sensor");
        }
    }
    if (publish_game_rv_) {
        if(!this->bno08x_->enable_report(SH2_GAME_ROTATION_VECTOR,
                                         1000000/this->game_rv_rate_)){            // Hz to us
            RCLCPP_ERROR(this->get_logger(), "Failed to enable game rotation vector sensor");
        }
    }
    if (publish_geo_rv_) {
        if(!this->bno08x_->enable_report(SH2_GEOMAGNETIC_ROTATION_VECTOR,
                                         1000000/this->geo_rv_rate_)){             // Hz to us
            RCLCPP_ERROR(this->get_logger(), "Failed to enable geomagnetic rotation vector sensor");
        }
    }
    if (!(publish_imu_ || publish_magnetic_field_ || publish_game_rv_ || publish_geo_rv_)) {
        RCLCPP_ERROR(this->get_logger(), "No sensor reports enabled! Exiting...");
        throw std::runtime_error("No sensor reports enabled");
    }
}

// Maps the SH-2 calibration accuracy level (SH2_ACCURACY_UNRELIABLE … SH2_ACCURACY_HIGH)
// to a diagonal variance value for sensor_msgs/Imu covariance matrices.
// Each step spans two decades so that robot_localization weights a fully calibrated
// reading ~1000x more heavily than an unreliable one.
double BNO08xROS::accuracy_to_variance(uint8_t accuracy)
{
    switch (accuracy) {
        case SH2_ACCURACY_HIGH:      return 1e-4;
        case SH2_ACCURACY_MEDIUM:    return 1e-3;
        case SH2_ACCURACY_LOW:       return 1e-2;
        case SH2_ACCURACY_UNRELIABLE:
        default:                     return 1e-1;
    }
}

void BNO08xROS::init_imu_covariance()
{
    std::copy(orientation_covariance_.begin(), orientation_covariance_.end(),
              imu_msg_.orientation_covariance.begin());
    std::copy(gyrometer_covariance_.begin(), gyrometer_covariance_.end(),
              imu_msg_.angular_velocity_covariance.begin());
    std::copy(linear_covariance_.begin(), linear_covariance_.end(),
              imu_msg_.linear_acceleration_covariance.begin());
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
	switch(sensor_value->sensorId){
		case SH2_MAGNETIC_FIELD_CALIBRATED:
			this->mag_msg_.magnetic_field.x = sensor_value->un.magneticField.x;
			this->mag_msg_.magnetic_field.y = sensor_value->un.magneticField.y;
			this->mag_msg_.magnetic_field.z = sensor_value->un.magneticField.z;
			this->mag_msg_.header.frame_id = this->frame_id_;
			this->mag_msg_.header.stamp.sec = this->get_clock()->now().seconds();
			this->mag_msg_.header.stamp.nanosec = this->get_clock()->now().nanoseconds();
			mag_accuracy_ = sensor_value->status & SH2_STATUS_ACCURACY_MASK;
			// IMU will still return infrequent magnetic field reports even if the report
			// was not enabled, so check it was enabled before publishing.
			if (publish_magnetic_field_) {
				this->mag_publisher_->publish(this->mag_msg_);
			}
			break;
		case SH2_ROTATION_VECTOR:
			this->imu_msg_.orientation.x = sensor_value->un.rotationVector.i;
			this->imu_msg_.orientation.y = sensor_value->un.rotationVector.j;
			this->imu_msg_.orientation.z = sensor_value->un.rotationVector.k;
			this->imu_msg_.orientation.w = sensor_value->un.rotationVector.real;
			orientation_accuracy_     = sensor_value->status & SH2_STATUS_ACCURACY_MASK;
			orientation_accuracy_rad_ = sensor_value->un.rotationVector.accuracy;
			imu_received_flag_ |= ROTATION_VECTOR_RECEIVED;
			break;
		case SH2_ACCELEROMETER:
			this->imu_msg_.linear_acceleration.x = sensor_value->un.accelerometer.x;
			this->imu_msg_.linear_acceleration.y = sensor_value->un.accelerometer.y;
			this->imu_msg_.linear_acceleration.z = sensor_value->un.accelerometer.z;
			accel_accuracy_ = sensor_value->status & SH2_STATUS_ACCURACY_MASK;
			imu_received_flag_ |= ACCELEROMETER_RECEIVED;
			break;
		case SH2_LINEAR_ACCELERATION:
			this->imu_msg_.linear_acceleration.x = sensor_value->un.linearAcceleration.x;
			this->imu_msg_.linear_acceleration.y = sensor_value->un.linearAcceleration.y;
			this->imu_msg_.linear_acceleration.z = sensor_value->un.linearAcceleration.z;
			accel_accuracy_ = sensor_value->status & SH2_STATUS_ACCURACY_MASK;
			imu_received_flag_ |= ACCELEROMETER_RECEIVED;
			break;
		case SH2_GYROSCOPE_CALIBRATED:
			this->imu_msg_.angular_velocity.x = sensor_value->un.gyroscope.x;
			this->imu_msg_.angular_velocity.y = sensor_value->un.gyroscope.y;
			this->imu_msg_.angular_velocity.z = sensor_value->un.gyroscope.z;
			gyro_accuracy_ = sensor_value->status & SH2_STATUS_ACCURACY_MASK;
			imu_received_flag_ |= GYROSCOPE_RECEIVED;
			break;
		case SH2_GEOMAGNETIC_ROTATION_VECTOR: {
			if (!publish_geo_rv_) break;
			float acc = sensor_value->un.geoMagRotationVector.accuracy;
			double ov = (acc > 0.0f) ? static_cast<double>(acc) * acc
			                         : accuracy_to_variance(SH2_ACCURACY_UNRELIABLE);
			geo_rv_msg_.orientation.x = sensor_value->un.geoMagRotationVector.i;
			geo_rv_msg_.orientation.y = sensor_value->un.geoMagRotationVector.j;
			geo_rv_msg_.orientation.z = sensor_value->un.geoMagRotationVector.k;
			geo_rv_msg_.orientation.w = sensor_value->un.geoMagRotationVector.real;
			geo_rv_msg_.orientation_covariance[0] = ov;
			geo_rv_msg_.orientation_covariance[4] = ov;
			geo_rv_msg_.orientation_covariance[8] = ov;
			geo_rv_msg_.header.frame_id = frame_id_;
			geo_rv_msg_.header.stamp = this->get_clock()->now();
			geo_rv_publisher_->publish(geo_rv_msg_);
			break;
		}
		case SH2_GAME_ROTATION_VECTOR: {
			if (!publish_game_rv_) break;
			double ov = accuracy_to_variance(sensor_value->status & SH2_STATUS_ACCURACY_MASK);
			game_rv_msg_.orientation.x = sensor_value->un.gameRotationVector.i;
			game_rv_msg_.orientation.y = sensor_value->un.gameRotationVector.j;
			game_rv_msg_.orientation.z = sensor_value->un.gameRotationVector.k;
			game_rv_msg_.orientation.w = sensor_value->un.gameRotationVector.real;
			game_rv_msg_.orientation_covariance[0] = ov;
			game_rv_msg_.orientation_covariance[4] = ov;
			game_rv_msg_.orientation_covariance[8] = ov;
			game_rv_msg_.header.frame_id = frame_id_;
			game_rv_msg_.header.stamp = this->get_clock()->now();
			game_rv_publisher_->publish(game_rv_msg_);
			break;
		}
		default:
			break;
	}

	if(imu_received_flag_ == (ROTATION_VECTOR_RECEIVED | ACCELEROMETER_RECEIVED | GYROSCOPE_RECEIVED)){
		// Orientation: use the continuous 1-sigma accuracy field from the rotation vector
		// report (radians), squaring it to get variance. Fall back to the UNRELIABLE level
		// when the field is zero (sensor not yet converged, not truly error-free).
		// Gyro and accel have no dedicated accuracy field — use the 4-level status bits.
		double ov = (orientation_accuracy_rad_ > 0.0f)
		            ? static_cast<double>(orientation_accuracy_rad_) * orientation_accuracy_rad_
		            : accuracy_to_variance(SH2_ACCURACY_UNRELIABLE);
		double gv = accuracy_to_variance(gyro_accuracy_);
		double av = accuracy_to_variance(accel_accuracy_);
		imu_msg_.orientation_covariance[0] = imu_msg_.orientation_covariance[4] = imu_msg_.orientation_covariance[8] = ov;
		imu_msg_.angular_velocity_covariance[0] = imu_msg_.angular_velocity_covariance[4] = imu_msg_.angular_velocity_covariance[8] = gv;
		imu_msg_.linear_acceleration_covariance[0] = imu_msg_.linear_acceleration_covariance[4] = imu_msg_.linear_acceleration_covariance[8] = av;

		this->imu_msg_.header.frame_id = this->frame_id_;
		this->imu_msg_.header.stamp.sec = this->get_clock()->now().seconds();
		this->imu_msg_.header.stamp.nanosec = this->get_clock()->now().nanoseconds();
		this->imu_publisher_->publish(this->imu_msg_);
		imu_received_flag_ = 0;
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
        this->bno08x_->poll();
    }
}

void BNO08xROS::reset() {
    std::lock_guard<std::mutex> lock(bno08x_mutex_);
    reset_count_++;
    delete bno08x_;
    this->init_sensor();
}

void BNO08xROS::tare_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    std::lock_guard<std::mutex> lock(bno08x_mutex_);
    bool ok = bno08x_->tare();
    response->success = ok;
    response->message = ok ? "Tare applied. Current orientation is now the zero reference."
                           : "Failed to apply tare.";
    if (ok) {
        RCLCPP_INFO(this->get_logger(), "Tare applied successfully.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to apply tare.");
    }
}

void BNO08xROS::clear_tare_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    std::lock_guard<std::mutex> lock(bno08x_mutex_);
    bool ok = bno08x_->clear_tare();
    response->success = ok;
    response->message = ok ? "Tare cleared. Orientation reference restored to default."
                           : "Failed to clear tare.";
    if (ok) {
        RCLCPP_INFO(this->get_logger(), "Tare cleared successfully.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to clear tare.");
    }
}

void BNO08xROS::publish_diagnostics()
{
    using DS  = diagnostic_msgs::msg::DiagnosticStatus;
    using KV  = diagnostic_msgs::msg::KeyValue;

    auto make_kv = [](const std::string& key, const std::string& val) {
        KV kv; kv.key = key; kv.value = val; return kv;
    };

    auto accuracy_label = [](uint8_t acc) -> std::string {
        switch (acc) {
            case SH2_ACCURACY_HIGH:       return "HIGH";
            case SH2_ACCURACY_MEDIUM:     return "MEDIUM";
            case SH2_ACCURACY_LOW:        return "LOW";
            case SH2_ACCURACY_UNRELIABLE:
            default:                      return "UNRELIABLE";
        }
    };

    auto accuracy_level = [](uint8_t acc) -> uint8_t {
        if (acc >= SH2_ACCURACY_MEDIUM) return DS::OK;
        if (acc == SH2_ACCURACY_LOW)    return DS::WARN;
        return DS::ERROR;
    };

    // Snapshot accuracy values and product ID under the mutex.
    uint8_t o_acc, g_acc, a_acc, m_acc;
    std::string part_number, fw_version;
    {
        std::lock_guard<std::mutex> lock(bno08x_mutex_);
        o_acc = orientation_accuracy_;
        g_acc = gyro_accuracy_;
        a_acc = accel_accuracy_;
        m_acc = mag_accuracy_;
        if (bno08x_ && bno08x_->prodIds.numEntries > 0) {
            const auto& e = bno08x_->prodIds.entry[0];
            part_number = std::to_string(e.swPartNumber);
            fw_version  = std::to_string(e.swVersionMajor) + "." +
                          std::to_string(e.swVersionMinor) + "." +
                          std::to_string(e.swVersionPatch);
        }
    }

    // ── Calibration status ────────────────────────────────────────────────────
    DS cal;
    cal.name        = "BNO08x/Calibration";
    cal.hardware_id = "bno08x";
    uint8_t worst   = std::min({o_acc, g_acc, a_acc, m_acc});
    cal.level       = accuracy_level(worst);
    cal.message     = "Worst accuracy: " + accuracy_label(worst);
    cal.values      = {
        make_kv("orientation",   accuracy_label(o_acc)),
        make_kv("gyroscope",     accuracy_label(g_acc)),
        make_kv("accelerometer", accuracy_label(a_acc)),
        make_kv("magnetometer",  accuracy_label(m_acc)),
    };

    // ── Driver health ─────────────────────────────────────────────────────────
    DS driver;
    driver.name        = "BNO08x/Driver";
    driver.hardware_id = "bno08x";
    uint32_t wdc       = watchdog_fire_count_.load();
    uint32_t rc        = reset_count_.load();
    driver.level       = (wdc > 0) ? DS::WARN : DS::OK;
    driver.message     = (wdc == 0) ? "OK" : "Watchdog has fired";
    driver.values      = {
        make_kv("watchdog_fires", std::to_string(wdc)),
        make_kv("sensor_resets",  std::to_string(rc)),
    };

    // ── Sensor info ───────────────────────────────────────────────────────────
    DS info;
    info.name        = "BNO08x/Sensor";
    info.hardware_id = "bno08x";
    info.level       = DS::OK;
    info.message     = "Sensor information";
    info.values      = {
        make_kv("part_number",      part_number),
        make_kv("firmware_version", fw_version),
    };

    diagnostic_msgs::msg::DiagnosticArray diag_array;
    diag_array.header.stamp    = this->get_clock()->now();
    diag_array.header.frame_id = frame_id_;
    diag_array.status          = {cal, driver, info};
    diag_publisher_->publish(diag_array);
}

void BNO08xROS::set_reorientation_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    std::vector<double> quat;
    this->get_parameter("tare.quaternion", quat);

    if (quat.size() != 4) {
        response->success = false;
        response->message = "tare.quaternion must be a 4-element array [x, y, z, w].";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }

    double norm = std::sqrt(quat[0]*quat[0] + quat[1]*quat[1] +
                            quat[2]*quat[2] + quat[3]*quat[3]);
    if (norm < 1e-9) {
        response->success = false;
        response->message = "tare.quaternion has zero norm, cannot normalize.";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }

    sh2_Quaternion_t orientation;
    orientation.x = quat[0] / norm;
    orientation.y = quat[1] / norm;
    orientation.z = quat[2] / norm;
    orientation.w = quat[3] / norm;

    std::lock_guard<std::mutex> lock(bno08x_mutex_);
    bool ok = bno08x_->set_reorientation(&orientation);
    response->success = ok;
    response->message = ok ? "Reorientation applied." : "Failed to apply reorientation.";
    if (ok) {
        RCLCPP_INFO(this->get_logger(),
            "Reorientation applied: [x=%.4f y=%.4f z=%.4f w=%.4f]",
            orientation.x, orientation.y, orientation.z, orientation.w);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to apply reorientation.");
    }
}

void BNO08xROS::save_calibration_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    std::lock_guard<std::mutex> lock(bno08x_mutex_);
    bool ok = bno08x_->save_dcd();
    response->success = ok;
    response->message = ok ? "Calibration data saved to flash."
                           : "Failed to save calibration data.";
    if (ok) {
        RCLCPP_INFO(this->get_logger(), "Calibration data saved to flash.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to save calibration data.");
    }
}
