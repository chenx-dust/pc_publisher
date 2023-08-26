#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <array>
#include <boost/endian/arithmetic.hpp>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/filter/zstd.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <chrono>
#include <memory>
#include <optional>
#include <thread>

using namespace boost::endian;

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

// 以下是用于对接 Livox 官方提供的现成代码的数据结构
typedef struct {
    float x; /**< X axis, Unit:m */
    float y; /**< Y axis, Unit:m */
    float z; /**< Z axis, Unit:m */
    float reflectivity; /**< Reflectivity   */
    uint8_t tag; /**< Livox point tag   */
    uint8_t line; /**< Laser line id     */
    double timestamp; /**< Timestamp of point*/
} LivoxPointXyzrtlt;
#pragma pack(pop)

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;
constexpr size_t msg_size = 1380;
constexpr size_t dot_num = 96;
const std::string frame_id = "livox_frame";
const std::string pc_topic = "/pc_raw";

class PcBasePublisher : public rclcpp::Node {
protected:
    int line_num = 6;

    rclcpp::Publisher<PointCloud2>::SharedPtr pub;

    rclcpp::TimerBase::SharedPtr timer;
    std::thread recv_thread;
    boost::lockfree::spsc_queue<LivoxPointXyzrtlt> pt_queue { 452000 }; // 1s buffer

    bool check_header(const header& header)
    {
        // TODO: check crc32
        if (header.version.value() != 0) {
            RCLCPP_ERROR(get_logger(), "header: version is not 1");
            return false;
        }
        if (header.length.value() != msg_size) {
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

    void proccess_data(const header& header, const pcd1_span& data)
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

    void pc2_init_header(PointCloud2& cloud)
    {
        cloud.header.frame_id.assign(frame_id);
        cloud.height = 1;
        cloud.width = 0;
        cloud.fields.resize(7);
        cloud.fields[0].offset = 0;
        cloud.fields[0].name = "x";
        cloud.fields[0].count = 1;
        cloud.fields[0].datatype = PointField::FLOAT32;
        cloud.fields[1].offset = 4;
        cloud.fields[1].name = "y";
        cloud.fields[1].count = 1;
        cloud.fields[1].datatype = PointField::FLOAT32;
        cloud.fields[2].offset = 8;
        cloud.fields[2].name = "z";
        cloud.fields[2].count = 1;
        cloud.fields[2].datatype = PointField::FLOAT32;
        cloud.fields[3].offset = 12;
        cloud.fields[3].name = "intensity";
        cloud.fields[3].count = 1;
        cloud.fields[3].datatype = PointField::FLOAT32;
        cloud.fields[4].offset = 16;
        cloud.fields[4].name = "tag";
        cloud.fields[4].count = 1;
        cloud.fields[4].datatype = PointField::UINT8;
        cloud.fields[5].offset = 17;
        cloud.fields[5].name = "line";
        cloud.fields[5].count = 1;
        cloud.fields[5].datatype = PointField::UINT8;
        cloud.fields[6].offset = 18;
        cloud.fields[6].name = "timestamp";
        cloud.fields[6].count = 1;
        cloud.fields[6].datatype = PointField::FLOAT64;
        cloud.point_step = sizeof(LivoxPointXyzrtlt);
    }

    virtual void recv_spin() = 0;

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

        pub->publish(cloud);
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
        pub = create_publisher<PointCloud2>(pc_topic, rclcpp::SensorDataQoS());
        timer = create_wall_timer(
            std::chrono::milliseconds(pub_interval_ms),
            std::bind(&PcBasePublisher::timer_callback, this));
    }
};
