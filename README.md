# PointCloudPublisher

Livox 激光雷达 ROS2 驱动节点，目前支持 Livox HAP 与 Livox Mid-360 两款激光雷达。

使用前请启动激光雷达，并将其设置为发送 Point Cloud Data 1（32 位直角坐标系，单位毫米，每包 96 点）数据。

配合控制程序 [shirok1/pylivox2](https://github.com/shirok1/pylivox2) 使用。

## 使用方式

launch 文件示例:

```yaml
launch:
- node:
    pkg: pc_publisher
    exec: pc_lidar_pub
    param:
    -
      name: lidar_line
      value: 6

- node:
    pkg: rviz2
    exec: rviz2
    args: --display-config /home/chenx/Source/radar_ros2_ws/src/pc_publisher/config/display_point_cloud.rviz
```

## ROS2 参数

### 通用

- `lidar_line`: 激光雷达线数，用于倒推数据中每一点所属的线编号，根据型号调整
  - Mid-360 使用 4
  - HAP 使用 6
- `pub_interval_ms`: 发布时间间隔，单位 ms，注意和频率是倒数关系
- `pc_topic`: 点云发布的话题，默认为 `livox/lidar`
- `imu_topic`: IMU 发布的话题，默认为 `livox/imu`
- `frame_id`: 所属的 tf2 frame，默认为 `livox_frame`。可能需要执行以下命令，使 RViz 认为 `livox_frame` 和 `map` 重合以便调试：

```shell
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map livox_frame
```

### 仅实时接收（运行 `pc_lidar_pub`）时使用

- `listen_port`: 监听端口，同时用于接收点云和 IMU 数据
- `timeout_ms`: 监听超时，单位 ms

### 仅重放 UDP 包（运行 `pc_record_pub`）时使用

- `record_file`: 文件路径
- `use_zstd`: 是否进行 Zstandard 解压缩
