#pragma once

#include "BasePub.hpp"

#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/filter/zstd.hpp>
#include <boost/iostreams/filtering_stream.hpp>

namespace pc_publisher {
class PcRecordPublisher final : public PcBasePublisher {
private:
    std::shared_ptr<boost::iostreams::filtering_istream> in;
    std::shared_ptr<boost::iostreams::file_source> file;

    void recv_spin()
    {
        std::array<unsigned char, 1380> recv_buf;
        while (!in->eof() && rclcpp::ok()) {
            in->read(reinterpret_cast<char*>(recv_buf.data()), recv_buf.size());
            auto header = reinterpret_cast<struct header*>(recv_buf.data());
            auto data = reinterpret_cast<pcd1_span*>(recv_buf.data() + sizeof(struct header));
            proccess_pcd1(*header, *data);

            if (header->time_type.value() == 0) {
                RCLCPP_WARN_ONCE(get_logger(), "timestamp not set");
                static std::chrono::nanoseconds last_time(0);
                constexpr std::chrono::nanoseconds interval(static_cast<size_t>(96.0 / 452000 * 1e9));
                auto now = std::chrono::steady_clock::now().time_since_epoch();
                auto diff = now - last_time;
                if (diff < interval && last_time != std::chrono::nanoseconds(0)) {
                    auto sleep_time = interval - diff;
                    std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_time));
                }
                last_time = std::chrono::steady_clock::now().time_since_epoch();

            } else {
                static std::chrono::nanoseconds init_time = std::chrono::steady_clock::now().time_since_epoch();
                static uint64_t init_timestamp = header->timestamp.value();
                auto now = std::chrono::steady_clock::now().time_since_epoch();
                auto diff = now - init_time;
                auto diff_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(diff);
                auto diff_timestamp = static_cast<uint64_t>(diff_ns.count());
                if (diff_timestamp < header->timestamp.value() - init_timestamp) {
                    auto sleep_time = header->timestamp.value() - init_timestamp - diff_timestamp;
                    std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_time));
                }
            }
        }
        RCLCPP_INFO(get_logger(), "PcRecordPublisher: EOF");
    }

public:
    PcRecordPublisher()
        : PcBasePublisher("pc_publisher")
    {
        RCLCPP_INFO(get_logger(), "PcRecordPublisher: Initializing");
        declare_parameter("record_file", "");
        declare_parameter("use_zstd", true);

        std::string record_filename = get_parameter("record_file").as_string();
        bool use_zstd = get_parameter("use_zstd").as_bool();

        RCLCPP_INFO(get_logger(), "PcRecordPublisher: Params: record_file: %s, use_zstd: %d",
            record_filename.c_str(), use_zstd);
        if (record_filename.empty()) {
            RCLCPP_ERROR(get_logger(), "record_file is empty");
            throw std::runtime_error("record_file is empty");
        }

        // 初始化
        in = std::make_shared<boost::iostreams::filtering_istream>();
        if (use_zstd)
            in->push(boost::iostreams::zstd_decompressor());
        file = std::make_shared<boost::iostreams::file_source>(record_filename, std::ios_base::in | std::ios_base::binary);
        in->push(*file);
        RCLCPP_INFO(get_logger(), "PcRecordPublisher: %s file opened", record_filename.c_str());
        recv_thread = std::thread(&PcRecordPublisher::recv_spin, this);
    }
};
}
