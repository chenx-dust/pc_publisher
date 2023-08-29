#include "pc_publisher/LidarPub.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pc_publisher::PcLidarPublisher>());
    rclcpp::shutdown();
    return 0;
}
