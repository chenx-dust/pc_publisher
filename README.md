# PointCloudPublisher

Livox 激光雷达 ROS2 驱动程序

使用前请运行雷达并且发送 PCD1 数据

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

可传递参数:

- `lidar_line` (`int`): 雷达线数，根据型号调整

    ```c++
    // 摘抄自 Livox 官方代码
    const uint8_t kLineNumberDefault = 1;
    const uint8_t kLineNumberMid360 = 4;
    const uint8_t kLineNumberHAP = 6;
    ```

- `pub_interval_ms` (`int`): 发布时间间隔，单位 ms
- `listen_port` (`int`): 监听端口，一般不用改
- `timeout_ms` (`int`): 监听超时，单位 ms