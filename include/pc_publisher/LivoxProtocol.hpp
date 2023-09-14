#pragma once
#ifndef PC_PUBLISHER_LIVOX_PROTOCOL_HPP
#define PC_PUBLISHER_LIVOX_PROTOCOL_HPP

#include <array>
#include <cstdint>

#include <boost/endian/arithmetic.hpp>

namespace pc_publisher::livox_proto {
// 以下是 Livox 激光雷达接受到的数据包结构
#pragma pack(push, 1)
struct header {
    boost::endian::little_uint8_t version;
    boost::endian::little_uint16_t length;
    boost::endian::little_uint16_t time_interval;
    boost::endian::little_uint16_t dot_num;
    boost::endian::little_uint16_t udp_cnt;
    boost::endian::little_uint8_t frame_cnt;
    boost::endian::little_uint8_t data_type;
    boost::endian::little_uint8_t time_type;
    boost::endian::little_uint8_t pack_info;
    boost::endian::little_uint8_t _padding[11];
    boost::endian::little_uint32_t crc32;
    boost::endian::little_uint64_t timestamp;
};

struct pcd1 {
    boost::endian::little_int32_t x;
    boost::endian::little_int32_t y;
    boost::endian::little_int32_t z;
    boost::endian::little_uint8_t reflectivity;
    boost::endian::little_uint8_t tag;
};
using pcd1_span = std::array<pcd1, 96>;

struct imu {
    boost::endian::little_float32_t gyro_x;
    boost::endian::little_float32_t gyro_y;
    boost::endian::little_float32_t gyro_z;
    boost::endian::little_float32_t acc_x;
    boost::endian::little_float32_t acc_y;
    boost::endian::little_float32_t acc_z;
};
#pragma pack(pop)

enum msg_type : std::uint8_t {
    MSG_IMU = 0,
    MSG_PCD1 = 1,
    MSG_PCD2 = 2,  // 不使用
};

}  // namespace pc_publisher::livox_proto
#endif
