#include <rclcpp/rclcpp.hpp>
#include "bno08x_driver/bno08x_ros.hpp"

int main(int argc, char **argv)
{   
    pthread_setname_np(pthread_self(), "main_thread");
    rclcpp::init(argc, argv);
    std::shared_ptr<BNO08xROS> bno08x_node = std::make_shared<BNO08xROS>();
    rclcpp::spin(bno08x_node);
    rclcpp::shutdown();

    return 0;
}
