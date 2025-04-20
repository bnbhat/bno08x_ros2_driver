#pragma once

#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include "bno08x_driver/bno08x.hpp"
#include "bno08x_driver/logger.h"
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
    CommInterface* comm_interface_;

    // Parameters
    std::string frame_id_;
    bool publish_magnetic_field_;
    int magnetic_field_rate_;
    bool publish_imu_;
    int imu_rate_;

    bool publish_orientation_;
    bool publish_acceleration_;
    bool publish_angular_velocity_;

};

