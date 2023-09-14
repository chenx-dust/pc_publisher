#pragma once

#include "pc_publisher/BasePub.hpp"

#include <boost/asio.hpp>

namespace pc_publisher {
class PcLidarPublisher final : public PcBasePublisher {
private:
    boost::asio::io_context ctx;
    std::optional<boost::asio::ip::udp::socket> socket;

    void recv_spin() {
        std::array<unsigned char, pc_msg_size> recv_buf;
        boost::system::error_code error;
        std::size_t recv_length = 0;
        while (rclcpp::ok() && socket->is_open()) {
            socket->async_receive(boost::asio::buffer(recv_buf),
                                  [&](const boost::system::error_code& error_, std::size_t length_) {
                                      error = error_;
                                      recv_length = length_;
                                  });
            ctx.restart();
            ctx.run_for(std::chrono::milliseconds(get_parameter("timeout_ms").as_int()));
            if (!ctx.stopped()) {
                socket->cancel();
                ctx.run();
            }
            if (error && error != boost::asio::error::message_size) {
                RCLCPP_ERROR(get_logger(), "Receiver error: %s", error.message().c_str());
                continue;
            }

            auto header = reinterpret_cast<livox_proto::header*>(recv_buf.data());
            if (header->data_type.value() == livox_proto::msg_type::MSG_IMU && header->length.value() == imu_msg_size) {
                if (!check_header_imu(*header)) {
                    RCLCPP_ERROR(get_logger(), "Wrong imu header");
                    return;
                }
                auto data = reinterpret_cast<livox_proto::imu*>(recv_buf.data() + sizeof(livox_proto::header));
                proccess_imu(*header, *data);
            } else if (header->data_type.value() == livox_proto::msg_type::MSG_PCD1 && header->length.value() == pc_msg_size) {
                if (!check_header_pcd1(*header)) {
                    RCLCPP_ERROR(get_logger(), "Wrong pcd1 header");
                    return;
                }
                auto data = reinterpret_cast<livox_proto::pcd1_span*>(recv_buf.data() + sizeof(livox_proto::header));
                proccess_pcd1(*header, *data);
            } else {
                RCLCPP_ERROR(get_logger(), "Wrong data_type: %d, length: %d", header->data_type.value(), header->length.value());
            }
        }
    }

public:
    PcLidarPublisher()
        : PcBasePublisher("pc_publisher") {
        RCLCPP_INFO(get_logger(), "PcLidarPublisher: Initializing");
        declare_parameter("listen_port", 57000);
        declare_parameter("timeout_ms", 1000);

        int listen_port = get_parameter("listen_port").as_int();
        int timeout_ms = get_parameter("timeout_ms").as_int();
        RCLCPP_INFO(get_logger(), "PcLidarPublisher: Params: listen_port: %d, timeout_ms: %d", listen_port, timeout_ms);

        socket.emplace(ctx, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), listen_port));
        recv_thread = std::thread(&PcLidarPublisher::recv_spin, this);
    }
};
}
