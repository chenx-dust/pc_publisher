#include "BasePub.hpp"

#include <boost/asio.hpp>

class PcLidarPublisher final : public PcBasePublisher {
private:
    boost::asio::io_context ctx;
    std::optional<boost::asio::ip::udp::socket> socket;

    void recv_spin() final
    {
        std::array<unsigned char, msg_size> recv_buf;
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
            if (recv_length != msg_size) {
                RCLCPP_ERROR(get_logger(), "Wrong receive length: %lu, error: %s (Timeout?)", recv_length, error.message().c_str());
                continue;
            }
            if (error && error != boost::asio::error::message_size) {
                RCLCPP_ERROR(get_logger(), "Receiver error: %s", error.message().c_str());
                continue;
            }

            auto header = reinterpret_cast<struct header*>(recv_buf.data());

            auto data = reinterpret_cast<pcd1_span*>(recv_buf.data() + sizeof(struct header));
            proccess_data(*header, *data);
        }
    }

public:
    PcLidarPublisher()
        : PcBasePublisher("pc_publisher")
    {
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

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PcLidarPublisher>());
    rclcpp::shutdown();
    return 0;
}
