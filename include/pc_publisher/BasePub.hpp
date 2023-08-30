#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <radar_interface/livox_struct.hpp>

#include <array>
#include <boost/endian/arithmetic.hpp>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/filter/zstd.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <chrono>
#include <memory>
#include <optional>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <thread>

using namespace boost::endian;
using radar_interface::LivoxPointXyzrtlt;

// 以下是 Livox 激光雷达接受到的数据包结构
#pragma pack(push, 1)
struct header {
    little_uint8_t version;
    little_uint16_t length;
    little_uint16_t time_interval;
    little_uint16_t dot_num;
    little_uint16_t udp_cnt;
    little_uint8_t frame_cnt;
    little_uint8_t data_type;
    little_uint8_t time_type;
    little_uint8_t pack_info;
    little_uint8_t _padding[11];
    little_uint32_t crc32;
    little_uint64_t timestamp;
};
struct pcd1 {
    little_int32_t x;
    little_int32_t y;
    little_int32_t z;
    little_uint8_t reflectivity;
    little_uint8_t tag;
};
using pcd1_span = std::array<struct pcd1, 96>;
struct imu {
    little_float32_t gyro_x;
    little_float32_t gyro_y;
    little_float32_t gyro_z;
    little_float32_t acc_x;
    little_float32_t acc_y;
    little_float32_t acc_z;
};
#pragma pack(pop)

enum msg_type {
    MSG_IMU = 0,
    MSG_PCD1 = 1,
    MSG_PCD2 = 2,   // 不使用
};

using sensor_msgs::msg::Imu;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;
constexpr size_t pc_msg_size = 1380;
constexpr size_t imu_msg_size = 60;
constexpr size_t dot_num = 96;
const std::string frame_id = "livox_frame";
const std::string pc_topic = "pc_raw";
const std::string imu_topic = "imu_raw";

class PcBasePublisher : public rclcpp::Node {
protected:
    int line_num = 6;

    rclcpp::Publisher<PointCloud2>::SharedPtr pc_pub;
    rclcpp::Publisher<Imu>::SharedPtr imu_pub;

    rclcpp::TimerBase::SharedPtr timer;
    std::thread recv_thread;
    boost::lockfree::spsc_queue<LivoxPointXyzrtlt> pt_queue { 452000 }; // 1s buffer

    bool check_header_pcd1(const header& header)
    {
        // TODO: check crc32
        if (header.version.value() != 0) {
            RCLCPP_ERROR(get_logger(), "header: version is not 1");
            return false;
        }
        if (header.length.value() != pc_msg_size) {
            RCLCPP_ERROR(get_logger(), "header: length is not 1380");
            return false;
        }
        if (header.dot_num.value() != dot_num) {
            RCLCPP_ERROR(get_logger(), "header: dot_num is not 96");
            return false;
        }
        // TODO: support other data_type
        if (header.data_type.value() != 1) {
            RCLCPP_ERROR(get_logger(), "header: data_type is not 1");
            return false;
        }
        if ((header.pack_info.value() & 0x03) != 0) {
            RCLCPP_ERROR(get_logger(), "header: pack_info is not 0");
            return false;
        }
        return true;
    }

    bool check_header_imu(const header& header)
    {
        // TODO: check crc32
        if (header.version.value() != 0) {
            RCLCPP_ERROR(get_logger(), "header: version is not 1");
            return false;
        }
        if (header.length.value() != imu_msg_size) {
            RCLCPP_ERROR(get_logger(), "header: length is not 60");
            return false;
        }
        if (header.dot_num.value() != dot_num) {
            RCLCPP_ERROR(get_logger(), "header: dot_num is not 96");
            return false;
        }
        // TODO: support other data_type
        if (header.data_type.value() != 1) {
            RCLCPP_ERROR(get_logger(), "header: data_type is not 1");
            return false;
        }
        if ((header.pack_info.value() & 0x03) != 0) {
            RCLCPP_ERROR(get_logger(), "header: pack_info is not 0");
            return false;
        }
        return true;
    }

    void proccess_pcd1(const header& header, const pcd1_span& data)
    {
        for (size_t i = 0; i < dot_num; ++i) {
            pt_queue.push({
                data[i].x.value() / 1000.0f,
                data[i].y.value() / 1000.0f,
                data[i].z.value() / 1000.0f,
                static_cast<float>(data[i].reflectivity.value()),
                data[i].tag.value(),
                static_cast<uint8_t>(i % line_num),
                static_cast<double>(header.timestamp.value() + header.time_interval.value() * i),   // TODO: 需要验证时间戳含义
            });
        }
    }

    void proccess_imu(const header& header, const imu& data)
    {
        Imu imu_msg;
        imu_msg.header.frame_id.assign(frame_id);
        imu_msg.header.stamp = rclcpp::Time(header.timestamp.value());
        imu_msg.angular_velocity.x = data.gyro_x.value();
        imu_msg.angular_velocity.y = data.gyro_y.value();
        imu_msg.angular_velocity.z = data.gyro_z.value();
        imu_msg.linear_acceleration.x = data.acc_x.value();
        imu_msg.linear_acceleration.y = data.acc_y.value();
        imu_msg.linear_acceleration.z = data.acc_z.value();
        imu_pub->publish(imu_msg);
    }

    void pc2_init_header(PointCloud2& cloud)
    {
        cloud.header.frame_id.assign(frame_id);
        cloud.header.stamp = now();
        cloud.height = 1;
        cloud.width = 0;
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(7,
            "x", 1, PointField::FLOAT32,
            "y", 1, PointField::FLOAT32,
            "z", 1, PointField::FLOAT32,
            "intensity", 1, PointField::FLOAT32,
            "tag", 1, PointField::UINT8,
            "line", 1, PointField::UINT8,
            "timestamp", 1, PointField::FLOAT64);
    }

    void timer_callback()
    {
        PointCloud2 cloud;
        pc2_init_header(cloud);
        size_t size = pt_queue.read_available();
        cloud.width = size;
        cloud.row_step = cloud.width * cloud.point_step;
        cloud.is_bigendian = false;
        cloud.is_dense = true;

        cloud.data.resize(sizeof(LivoxPointXyzrtlt) * size);
        auto data = reinterpret_cast<LivoxPointXyzrtlt*>(cloud.data.data());
        for (size_t i = 0; i < size; ++i) {
            pt_queue.pop(data[i]);
        }

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");

        pc_pub->publish(cloud);
    }

public:
    PcBasePublisher(std::string node_name)
        : Node(node_name)
    {
        RCLCPP_INFO(get_logger(), "PcBasePublisher: Initializing");
        declare_parameter("pub_interval_ms", 20);
        declare_parameter("lidar_line", 6);

        int pub_interval_ms = get_parameter("pub_interval_ms").as_int();
        line_num = get_parameter("lidar_line").as_int();

        RCLCPP_INFO(get_logger(), "PcBasePublisher: Params: pub_interval_ms: %d, line_num: %d",
            pub_interval_ms, line_num);

        // 初始化
        pc_pub = create_publisher<PointCloud2>(pc_topic, rclcpp::SystemDefaultsQoS());
        timer = create_wall_timer(
            std::chrono::milliseconds(pub_interval_ms),
            std::bind(&PcBasePublisher::timer_callback, this));
    }
};
