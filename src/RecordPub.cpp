#include "pc_publisher/RecordPub.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pc_publisher::PcRecordPublisher>());
    rclcpp::shutdown();
    return 0;
}
