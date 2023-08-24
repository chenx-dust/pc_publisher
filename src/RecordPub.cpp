#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <array>
#include <boost/asio.hpp>
#include <boost/endian/arithmetic.hpp>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/filter/zstd.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <chrono>
#include <memory>
#include <thread>
#include <optional>

using namespace boost::endian;
constexpr size_t msg_size = 1380;
constexpr size_t dot_num = 96;
constexpr size_t line_num = 6; // HAP
const std::string frame_id = "livox_frame";

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

class PcRecordPublisher : public rclcpp::Node {
private:
    rclcpp::Publisher<PointCloud2>::SharedPtr pub;
    std::shared_ptr<boost::iostreams::filtering_istream> in;
    std::shared_ptr<boost::iostreams::file_source> file;

    rclcpp::TimerBase::SharedPtr timer;
    std::thread recv_thread;
    boost::lockfree::spsc_queue<LivoxPointXyzrtlt> pt_queue {452000};   // 1s buffer

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

    void recv_spin()
    {
        std::array<unsigned char, 1380> recv_buf;
        while (!in->eof()) {
            in->read(reinterpret_cast<char*>(recv_buf.data()), recv_buf.size());
            auto header = reinterpret_cast<struct header*>(recv_buf.data());
            // TODO: check crc32
            if (header->version.value() != 0) {
                RCLCPP_ERROR(get_logger(), "header version is not 1");
                continue;
            }
            if (header->length.value() != msg_size) {
                RCLCPP_ERROR(get_logger(), "header length is not 1380");
                continue;
            }
            if (header->dot_num.value() != dot_num) {
                RCLCPP_ERROR(get_logger(), "header dot_num is not 96");
                continue;
            }
            // TODO: support other data_type
            if (header->data_type.value() != 1) {
                RCLCPP_ERROR(get_logger(), "header data_type is not 1");
                continue;
            }
            if ((header->pack_info.value() & 0x03) != 0) {
                RCLCPP_ERROR(get_logger(), "header pack_info is not 0");
                continue;
            }

            auto data = reinterpret_cast<pcd1_span*>(recv_buf.data() + sizeof(struct header));

            for (size_t i = 0; i < dot_num; ++i) {
                pt_queue.push({
                    data->at(i).x.value() / 1000.0f,
                    data->at(i).y.value() / 1000.0f,
                    data->at(i).z.value() / 1000.0f,
                    static_cast<float>(data->at(i).reflectivity.value()),
                    data->at(i).tag.value(),
                    static_cast<uint8_t>(i % line_num),
                    static_cast<double>(header->timestamp.value() + header->time_interval.value() * i), // XXX: 需要验证时间戳含义
                });
            }

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

        pub->publish(cloud);
    }

public:
    PcRecordPublisher() : Node("pc_record_pub")
    {
        RCLCPP_INFO(get_logger(), "Initializing");
        declare_parameter("record_file", "");
        declare_parameter("use_zstd", true);
        declare_parameter("pc_topic", "/pc_raw");
        declare_parameter("pub_interval_ms", 20);

        std::string record_filename = get_parameter("record_file").as_string();
        bool use_zstd = get_parameter("use_zstd").as_bool();
        std::string pc_topic = get_parameter("pc_topic").as_string();
        int pub_interval_ms = get_parameter("pub_interval_ms").as_int();

        RCLCPP_INFO(get_logger(), "Params: record_file: %s, use_zstd: %d, pc_topic: %s, pub_interval_ms: %d",
            record_filename.c_str(), use_zstd, pc_topic.c_str(), pub_interval_ms);
        if (record_filename.empty()) {
            RCLCPP_ERROR(get_logger(), "record_file is empty");
            throw std::runtime_error("record_file is empty");
        }

        // 初始化
        pub = create_publisher<PointCloud2>(pc_topic, 10);
        in = std::make_shared<boost::iostreams::filtering_istream>();
        if (use_zstd)
            in->push(boost::iostreams::zstd_decompressor());
        file = std::make_shared<boost::iostreams::file_source>(record_filename, std::ios_base::in | std::ios_base::binary);
        in->push(*file);
        RCLCPP_INFO(get_logger(), "%s file opened", record_filename.c_str());

        timer = create_wall_timer(
            std::chrono::milliseconds(get_parameter("pub_interval_ms").as_int()),
            std::bind(&PcRecordPublisher::timer_callback, this));
        recv_thread = std::thread(&PcRecordPublisher::recv_spin, this);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PcRecordPublisher>());
    rclcpp::shutdown();
    return 0;
}
