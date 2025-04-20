#include "bno08x_driver/bno08x_ros.hpp"
#include "bno08x_driver/i2c_interface.hpp"
#include "bno08x_driver/uart_interface.hpp"

constexpr uint8_t ROTATION_VECTOR_RECEIVED = 0x01;
constexpr uint8_t ACCELEROMETER_RECEIVED   = 0x02;
constexpr uint8_t GYROSCOPE_RECEIVED       = 0x04;

BNO08xROS::BNO08xROS()
    : Node("bno08x_ros")
{  
    this->init_parameters();
    this->init_comms();
    this->init_sensor();

    if (publish_imu_) {
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

    // Poll the sensor at the rate of the fastest sensor
    this->imu_received_flag_ = 0;
    if(this->imu_rate_ < this->magnetic_field_rate_){
        this->poll_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000/this->magnetic_field_rate_), // Hz to ms
            std::bind(&BNO08xROS::poll_timer_callback, this)
        );
    } else {
        this->poll_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000/this->imu_rate_), // Hz to ms
            std::bind(&BNO08xROS::poll_timer_callback, this)
        );
    }

    // Initialize the watchdog timer
    auto timeout = std::chrono::milliseconds(2000);
    watchdog_ = new Watchdog();
    watchdog_->set_timeout(timeout);
    watchdog_->set_check_interval(timeout / 2); 
    watchdog_->set_callback([this]() {
        RCLCPP_ERROR(this->get_logger(), "Watchdog timeout! No data received from sensor. Resetting...");
        this->reset();
    });
    watchdog_->start();

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
    bool i2c_enabled, uart_enabled;
    this->get_parameter("i2c.enabled", i2c_enabled);
    this->get_parameter("uart.enabled", uart_enabled);

    if (i2c_enabled) {
        std::string device;
        std::string address;
        this->get_parameter("i2c.bus", device);
        this->get_parameter("i2c.address", address);
        RCLCPP_INFO(this->get_logger(), "Communication Interface: I2C");
        try {
            comm_interface_ = new I2CInterface(device, std::stoi(address, nullptr, 16));
        } catch (const std::bad_alloc& e) {
            RCLCPP_ERROR(this->get_logger(), 
                    "Failed to allocate memory for I2CInterface object: %s", e.what());
            throw std::runtime_error("I2CInterface object allocation failed");
        }
    } else if (uart_enabled) {
        RCLCPP_INFO(this->get_logger(), "Communication Interface: UART");
        std::string device;
        this->get_parameter("uart.device", device);
        try{
            comm_interface_ = new UARTInterface(device);
        } catch (const std::bad_alloc& e) {
            RCLCPP_ERROR(this->get_logger(), 
                    "Failed to allocate memory for UARTInterface object: %s", e.what());
            throw std::runtime_error("UARTInterface object allocation failed");
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

    this->get_parameter("frame_id", frame_id_);

    this->get_parameter("publish.magnetic_field.enabled", publish_magnetic_field_);
    this->get_parameter("publish.magnetic_field.rate", magnetic_field_rate_);
    this->get_parameter("publish.imu.enabled", publish_imu_);
    this->get_parameter("publish.imu.rate", imu_rate_);
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

    if(!this->bno08x_->enable_report(SH2_MAGNETIC_FIELD_CALIBRATED, 
                                      1000000/this->magnetic_field_rate_)){     // Hz to us
      RCLCPP_ERROR(this->get_logger(), "Failed to enable magnetic field sensor");
    }
    if(!this->bno08x_->enable_report(SH2_ROTATION_VECTOR, 
                                        1000000/this->imu_rate_)){              // Hz to us
      RCLCPP_ERROR(this->get_logger(), "Failed to enable rotation vector sensor");
    }
    if(!this->bno08x_->enable_report(SH2_ACCELEROMETER,
                                        1000000/this->imu_rate_)){              // Hz to us
        RCLCPP_ERROR(this->get_logger(), "Failed to enable accelerometer sensor");
    }
    if(!this->bno08x_->enable_report(SH2_GYROSCOPE_CALIBRATED, 
                                        1000000/this->imu_rate_)){              // Hz to us
        RCLCPP_ERROR(this->get_logger(), "Failed to enable gyroscope sensor");
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
	switch(sensor_value->sensorId){
		case SH2_MAGNETIC_FIELD_CALIBRATED:
			this->mag_msg_.magnetic_field.x = sensor_value->un.magneticField.x;
			this->mag_msg_.magnetic_field.y = sensor_value->un.magneticField.y;
			this->mag_msg_.magnetic_field.z = sensor_value->un.magneticField.z;
			this->mag_msg_.header.frame_id = this->frame_id_;
			this->mag_msg_.header.stamp.sec = this->get_clock()->now().seconds();
			this->mag_msg_.header.stamp.nanosec = this->get_clock()->now().nanoseconds();
			this->mag_publisher_->publish(this->mag_msg_);
			break;
		case SH2_ROTATION_VECTOR:
			this->imu_msg_.orientation.x = sensor_value->un.rotationVector.i;
			this->imu_msg_.orientation.y = sensor_value->un.rotationVector.j;
			this->imu_msg_.orientation.z = sensor_value->un.rotationVector.k;
			this->imu_msg_.orientation.w = sensor_value->un.rotationVector.real;
			imu_received_flag_ |= ROTATION_VECTOR_RECEIVED;
			break;
		case SH2_ACCELEROMETER:
			this->imu_msg_.linear_acceleration.x = sensor_value->un.accelerometer.x;
			this->imu_msg_.linear_acceleration.y = sensor_value->un.accelerometer.y;
			this->imu_msg_.linear_acceleration.z = sensor_value->un.accelerometer.z;
			imu_received_flag_ |= ACCELEROMETER_RECEIVED;
			break;
		case SH2_GYROSCOPE_CALIBRATED:
			this->imu_msg_.angular_velocity.x = sensor_value->un.gyroscope.x;
			this->imu_msg_.angular_velocity.y = sensor_value->un.gyroscope.y;
			this->imu_msg_.angular_velocity.z = sensor_value->un.gyroscope.z;
			imu_received_flag_ |= GYROSCOPE_RECEIVED;
			break;
		default:
			break;
	}

	if(imu_received_flag_ == (ROTATION_VECTOR_RECEIVED | ACCELEROMETER_RECEIVED | GYROSCOPE_RECEIVED)){
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
    delete bno08x_;
    this->init_sensor();
}
