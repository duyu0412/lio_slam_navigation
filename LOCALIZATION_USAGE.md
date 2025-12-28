# 定位模式使用说明

## 功能概述

已为 `localization.cpp` 添加了纯定位模式功能，支持：
- 加载预构建的 PCD 地图
- 通过 `/initialpose` 话题手动设置初始位姿
- 发布 `map -> camera_init -> aft_mapped` 完整的 TF 树
- 静态地图（不进行增量更新）
- 禁用后端优化和回环检测（仅使用前端里程计）

## TF 坐标系关系

定位模式下的 TF 树结构：
```
map (全局地图坐标系)
 └─> camera_init (里程计坐标系，Point-LIO 前端输出)
      └─> aft_mapped (机器人当前位姿)
```

- **map**: 预构建地图的全局坐标系
- **camera_init**: Point-LIO 前端里程计的原点（第一帧时的位置）
- **aft_mapped**: 机器人当前时刻的位姿

## 配置文件设置

在配置文件中启用定位模式（例如 `unilidar_l2_loc.yaml`）：

```yaml
localization:
  localization_mode: true  # 启用定位模式
  map_file_path: "/home/user/SLAM/src/point_lio_unilidar/PCD/optimized_map0.pcd"  # 地图文件路径
```

**注意**: 地图文件路径必须是绝对路径。

## 使用流程

### 1. 启动定位节点

```bash
roslaunch point_lio_unilidar localization_unilidar_l2.launch
```

节点启动后会：
- 加载预构建的 PCD 地图
- 构建静态 ikd-tree 用于匹配
- 等待初始位姿输入

### 2. 设置初始位姿

有两种方式设置初始位姿：

#### 方法 1: 使用 RViz "2D Pose Estimate" 工具

1. 在 RViz 中点击 "2D Pose Estimate" 按钮
2. 在地图上点击并拖拽设置机器人的初始位置和朝向
3. 这会发布一个 `/initialpose` 消息

#### 方法 2: 手动发布初始位姿

```bash
rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped '{
  header: {frame_id: "map"},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}' -1
```

### 3. 系统开始定位

收到初始位姿后：
- 系统计算 `map -> camera_init` 的静态变换
- 开始基于预加载地图进行点云匹配
- 持续发布完整的 TF 树和里程计信息

## 日志信息

### 启动时的日志

```
=== Localization Mode Enabled ===
Loading map from: /path/to/map.pcd
Map loaded successfully: 1234567 points
Map bounding box:
  X: [-50.00, 50.00] (100.00 m)
  Y: [-30.00, 30.00] (60.00 m)
  Z: [-2.00, 5.00] (7.00 m)
Subscribed to /initialpose topic for localization initialization
```

### 等待初始位姿

```
Waiting for initial pose on /initialpose topic...
```

### 收到初始位姿

```
Initial pose received in map frame:
  Position: [5.123, -2.456, 0.000]
  Orientation (quat): [1.000, 0.000, 0.000, 0.050]
Building ikd-tree from pre-loaded map (1234567 points)...
ikd-tree built successfully from map. Tree size: 1234567
Computed map to camera_init transformation:
  Translation: [5.100, -2.400, 0.050]
  Rotation (ZYX): [5.73, 0.12, -0.05] deg
```

## 话题说明

### 订阅的话题

- `/initialpose` (geometry_msgs/PoseWithCovarianceStamped): 初始位姿设置
- `/unilidar/cloud` (sensor_msgs/PointCloud2): 激光雷达点云
- `/unilidar/imu` (sensor_msgs/Imu): IMU 数据

### 发布的话题

- `/pointlio/odom` (nav_msgs/Odometry): 机器人位姿（在 camera_init 坐标系下）
- `/pointlio/path` (nav_msgs/Path): 机器人轨迹
- `/pointlio/cloud_registered` (sensor_msgs/PointCloud2): 配准后的点云（world 坐标系）
- `/pointlio/cloud_registered_body` (sensor_msgs/PointCloud2): 机器人坐标系点云
- `/pointlio/laser_map` (sensor_msgs/PointCloud2): 加载的静态地图（仅发布一次）

### TF 变换

定位模式下发布：
- `map -> camera_init`: 静态变换（基于初始位姿计算）
- `camera_init -> aft_mapped`: 动态变换（Point-LIO 前端输出）

## 注意事项

1. **地图格式**: 必须是 `.pcd` 格式的点云文件
2. **初始位姿**: 必须在开始定位前设置，否则系统会一直等待
3. **初始位姿精度**: 建议初始位姿误差在 ±2 米和 ±15 度以内
4. **地图加载**: 大地图可能需要几秒钟加载时间
5. **性能**: 定位模式下不进行地图更新，性能比建图模式更好

## 与 global_localization.py 配合使用

可以结合 `global_localization.py` 实现全局重定位：

```bash
# Terminal 1: 启动定位节点
roslaunch point_lio_unilidar localization_unilidar_l2.launch

# Terminal 2: 运行全局重定位
rosrun point_lio_unilidar global_localization.py

# Terminal 3: 运行变换融合
rosrun point_lio_unilidar transform_fusion.py
```

这样可以自动估计初始位姿，无需手动设置。

## 故障排查

### 问题: 无法加载地图

**症状**: 
```
Map file does not exist: /path/to/map.pcd
Failed to load map! Exiting...
```

**解决**: 
- 检查地图文件路径是否正确
- 确保使用绝对路径
- 确认文件存在且有读取权限

### 问题: 一直等待初始位姿

**症状**: 
```
Waiting for initial pose on /initialpose topic...
```

**解决**:
- 在 RViz 中使用 "2D Pose Estimate" 工具
- 或手动发布 `/initialpose` 话题

### 问题: TF 树不完整

**症状**: 在 RViz 中看不到 `map -> camera_init` 变换

**解决**:
- 确认已收到初始位姿（检查日志）
- 确认 `localization_mode` 设置为 `true`
- 检查系统是否正常运行（发布里程计话题）
