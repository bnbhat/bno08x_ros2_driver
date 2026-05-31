#pragma once

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "bno08x_driver/bno08x.hpp"
#include "bno08x_driver/watchdog.hpp"

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
    void init_imu_covariance();
    void poll_timer_callback();
    void reset();

    // ROS Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
    sensor_msgs::msg::Imu imu_msg_;
    sensor_msgs::msg::MagneticField mag_msg_;
    uint8_t imu_received_flag_;

    // ROS Timer
    rclcpp::TimerBase::SharedPtr poll_timer_;

    // BNO08X Sensor Interface
    BNO08x* bno08x_;
    std::mutex bno08x_mutex_;
    CommInterface* comm_interface_;

    // Watchdog
    Watchdog* watchdog_;

    // Parameters
    std::string frame_id_;
    bool publish_magnetic_field_;
    int magnetic_field_rate_;
    bool publish_imu_;
    int imu_rate_;

    bool publish_orientation_;
    bool publish_acceleration_;
    bool publish_angular_velocity_;

    std::vector<double> orientation_covariance_;
    std::vector<double> gyrometer_covariance_;
    std::vector<double> linear_covariance_;

    // Default covariance values derived from datasheet noise figures.
    // Orientation: 3.5 deg RMS -> variance in rad^2
    const std::vector<double> default_orientation_covariance_ = {
        pow(3.5 * M_PI / 180, 2), 0, 0,
        0, pow(3.5 * M_PI / 180, 2), 0,
        0, 0, pow(3.5 * M_PI / 180, 2)
    };
    // Gyroscope: 3.1 deg/s RMS -> variance in (rad/s)^2
    const std::vector<double> default_gyrometer_covariance_ = {
        pow(3.1 * M_PI / 180, 2), 0, 0,
        0, pow(3.1 * M_PI / 180, 2), 0,
        0, 0, pow(3.1 * M_PI / 180, 2)
    };
    // Accelerometer: 0.35 m/s^2 RMS -> variance in (m/s^2)^2
    const std::vector<double> default_linear_covariance_ = {
        pow(0.35, 2), 0, 0,
        0, pow(0.35, 2), 0,
        0, 0, pow(0.35, 2)
    };
};
