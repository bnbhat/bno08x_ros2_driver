#pragma once

#include <mutex>
#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include "bno08x_driver/bno08x.hpp"
#include "bno08x_driver/logger.h"
#include "bno08x_driver/watchdog.hpp"
#include "sh2/sh2.h"

class BNO08xROS : public rclcpp::Node
{
public:
    BNO08xROS();
    ~BNO08xROS();
    void sensor_callback(void *cookie, sh2_SensorValue_t *sensor_value);

private:
    void init_comms();
    void init_parameters();
    void init_sensor();
    void poll_timer_callback();
    void reset();
    std::string sensor_name(uint8_t sensor_id);
    std::string accuracy_status_string();

    using acc_stat_t = uint8_t;
    float get_covariance_scaled(float base_variance, acc_stat_t accuracy);
    static constexpr acc_stat_t MAG_MASK   = 0x03; // bits 0-1
    static constexpr acc_stat_t ACC_MASK   = 0x0C; // bits 2-3
    static constexpr acc_stat_t GYR_MASK   = 0x30; // bits 4-5
    static constexpr acc_stat_t RV_MASK    = 0xC0; // bits 6-7

    acc_stat_t accuracy_status_{0};  // bits 0-1 Mag, bits 2-3 Accel, bits 4-5 Gyro, bits 6-7 Rotation Vector

    // ROS Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr calib_status_publisher_;

    sensor_msgs::msg::Imu imu_msg_;
    sensor_msgs::msg::MagneticField mag_msg_;
    std_msgs::msg::String calib_msg_;

    uint8_t imu_received_flag_{0};
    rclcpp::Time last_calib_status_publish_time_;

    // ROS Timer
    rclcpp::TimerBase::SharedPtr poll_timer_;

    // BNO08X Sensor Interface
    std::mutex bno08x_mutex_;
    std::unique_ptr<CommInterface> comm_interface_;  // must be defined before bno08x_ so it’s destroyed after bno08x_
    std::unique_ptr<BNO08x> bno08x_;                 // Destruction is reverse of declaration order.

    // Watchdog
    std::unique_ptr<Watchdog> watchdog_;

    // IMU data bundling for more consistent timestamps
    bool imu_bundle_active_{false};
    rclcpp::Time imu_bundle_stamp_;
    rclcpp::Time imu_bundle_start_time_;
    static constexpr double IMU_BUNDLE_TIMEOUT_SEC = 0.05;  // 50 ms

    // Parameters
    std::string frame_id_;
    bool publish_magnetic_field_;
    int magnetic_field_rate_;
    bool publish_imu_;
    int imu_rate_;
    double orientation_yaw_variance_;
    bool verbose_;
    bool scale_covariance_by_calib_;

    bool publish_orientation_;
    bool publish_acceleration_;
    bool publish_angular_velocity_;
};

