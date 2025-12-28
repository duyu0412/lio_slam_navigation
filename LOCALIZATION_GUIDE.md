# Point-LIO 定位模式使用指南

## 📋 目录
1. [建图模式](#建图模式)
2. [定位模式](#定位模式)
3. [模式切换](#模式切换)
4. [常见问题](#常见问题)

---

## 🗺️ 建图模式

### 第一步：配置建图参数

编辑 `config/unilidar_l2.yaml`：

```yaml
localization:
    localization_mode: false  # 建图模式

pcd_save:
    pcd_save_en: true         # 开启地图保存
    map_save_downsample_en: true   # 是否下采样（根据需求）
    map_save_resolution: 0.05      # 下采样分辨率

optimization:
    loop_closure_enable_flag: true  # 开启回环检测（建议）
```

### 第二步：运行建图

```bash
# 启动建图节点
roslaunch point_lio_unilidar mapping_unilidar_l2.launch

# 播放或运行数据源
rosbag play your_data.bag
# 或连接实际传感器
```

### 第三步：保存地图

建图完成后，按 `Ctrl+C` 停止节点，地图会自动保存到：
- `PCD/optimized_map.pcd` - 优化后的全局地图
- `PCD/scans.pcd` - 原始累积地图（可选）

---

## 🎯 定位模式

### 第一步：准备地图文件

确保已有地图文件（通过建图模式生成）：
```bash
ls ~/SLAM/src/point_lio_unilidar/PCD/optimized_map.pcd
```

### 第二步：配置定位参数

**方案A：使用专用配置文件**（推荐）

```bash
# 编辑 config/localization_unilidar_l2.yaml
localization:
    localization_mode: true
    map_file_path: "/home/user/SLAM/src/point_lio_unilidar/PCD/optimized_map.pcd"

optimization:
    loop_closure_enable_flag: false  # 定位模式关闭回环
```

**方案B：修改原有配置文件**

```bash
# 编辑 config/unilidar_l2.yaml
localization:
    localization_mode: true  # 切换到定位模式
    map_file_path: "/path/to/your/optimized_map.pcd"

optimization:
    loop_closure_enable_flag: false  # 关闭回环检测
```

### 第三步：运行定位

```bash
# 使用定位专用 launch 文件
roslaunch point_lio_unilidar localization_unilidar_l2.launch

# 或指定地图文件
roslaunch point_lio_unilidar localization_unilidar_l2.launch \
    map_file:=/path/to/custom_map.pcd
```

### 第四步：提供初始位姿

有三种方式提供初始位姿：

**方式1：RViz 手动设置**（最简单）
1. 在 RViz 中点击 `2D Pose Estimate`
2. 在地图上点击并拖动设置初始位置和方向

**方式2：发布初始位姿话题**
```bash
# 使用 rostopic pub
rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped \
'{header: {frame_id: "camera_init"}, 
  pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, 
                orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'
```

**方式3：从其他传感器获取**（如GPS、视觉定位等）

---

## 🔄 模式切换

### 建图模式 → 定位模式

```bash
# 1. 确保地图已保存
cd ~/SLAM/src/point_lio_unilidar/PCD/
ls optimized_map.pcd

# 2. 修改配置文件
# 编辑 config/unilidar_l2.yaml 或使用 localization_unilidar_l2.yaml

# 3. 启动定位模式
roslaunch point_lio_unilidar localization_unilidar_l2.launch
```

### 定位模式 → 建图模式

```bash
# 修改配置文件
# config/unilidar_l2.yaml:
localization:
    localization_mode: false

optimization:
    loop_closure_enable_flag: true

# 启动建图模式
roslaunch point_lio_unilidar mapping_unilidar_l2.launch
```

---

## ⚙️ 重要参数说明

### 建图模式参数

| 参数 | 说明 | 推荐值 |
|------|------|--------|
| `loop_closure_enable_flag` | 是否启用回环检测 | `true` |
| `keyframe_add_dist_threshold` | 关键帧距离阈值 | 1.0-2.0 m |
| `pcd_save_en` | 是否保存地图 | `true` |
| `map_save_downsample_en` | 保存时是否下采样 | 根据需求 |
| `map_save_resolution` | 下采样分辨率 | 0.05 m |

### 定位模式参数

| 参数 | 说明 | 推荐值 |
|------|------|--------|
| `localization_mode` | 定位模式开关 | `true` |
| `map_file_path` | 地图文件路径 | 绝对路径 |
| `loop_closure_enable_flag` | 回环检测（定位时关闭） | `false` |
| `max_retained_keyframes_cloud` | 保留关键帧数（减少内存） | 100 |

---

## ❓ 常见问题

### Q1: 定位模式下位姿漂移怎么办？

**可能原因：**
- 初始位姿不准确
- 地图质量差
- 传感器标定有误

**解决方案：**
1. 重新提供更准确的初始位姿
2. 使用更密集、质量更高的地图
3. 检查 IMU-LiDAR 外参标定

### Q2: 如何评估定位精度？

```bash
# 对比原始轨迹和优化轨迹
rostopic echo /pointlio/path       # 前端里程计轨迹
rostopic echo /pointlio/opt_path   # 优化后轨迹（定位模式下就是匹配结果）
```

### Q3: 定位失败/跳变怎么办？

**检查项：**
1. ✅ 地图是否完整覆盖当前区域
2. ✅ 初始位姿是否在地图范围内
3. ✅ 环境是否发生较大变化（动态物体、季节变化等）
4. ✅ LiDAR数据质量（点云密度、退化场景）

**优化方法：**
- 降低 `filter_size_map` 增加匹配点
- 调整 `lidar_meas_cov` 噪声模型
- 使用更高频率的IMU数据

### Q4: 地图文件太大怎么办？

```bash
# 方案1: 启用下采样保存
map_save_downsample_en: true
map_save_resolution: 0.1  # 增大分辨率（降低密度）

# 方案2: 使用 PCL 工具离线下采样
pcl_voxel_grid -leaf 0.1,0.1,0.1 optimized_map.pcd downsampled_map.pcd

# 方案3: 只保留感兴趣区域
# 使用 CloudCompare 或 PCL 裁剪
```

### Q5: 如何同时运行建图和定位？

不建议同时运行，但可以：
1. 使用两个不同的命名空间
2. 修改 topic 名称避免冲突
3. 一个节点建图，另一个节点用旧地图定位

---

## 📊 性能优化建议

### 建图模式优化
- 降低关键帧频率（增大 `keyframe_add_dist_threshold`）
- 限制保留的关键帧数量
- 在平坦/退化场景减少点云密度

### 定位模式优化
- 使用下采样后的地图
- 禁用回环检测和后端优化
- 减少 `max_retained_keyframes_cloud` 值
- 使用体素化的全局地图

---

## 🔗 相关话题

### 建图模式话题
- `/pointlio/path` - 原始里程计轨迹
- `/pointlio/opt_path` - 优化后轨迹
- `/pointlio/keyframe_path` - 关键帧轨迹
- `/pointlio/loop_closure_markers` - 回环可视化

### 定位模式话题
- `/pointlio/odom` - 定位结果（里程计）
- `/pointlio/path` - 定位轨迹
- `/pointlio/global_map` - 加载的全局地图
- `/initialpose` - 初始位姿输入

---

## 📝 示例工作流程

```bash
# ========== 第一天：建图 ==========
# 1. 准备配置
cd ~/SLAM/src/point_lio_unilidar
vim config/unilidar_l2.yaml  # 确保 localization_mode: false

# 2. 启动建图
roslaunch point_lio_unilidar mapping_unilidar_l2.launch

# 3. 采集数据
rosbag play mapping_data.bag

# 4. 保存地图（Ctrl+C 停止节点）
# 地图保存在 PCD/optimized_map.pcd

# ========== 第二天：定位 ==========
# 1. 启动定位模式
roslaunch point_lio_unilidar localization_unilidar_l2.launch

# 2. 设置初始位姿（RViz中使用 2D Pose Estimate）

# 3. 运行实时数据或rosbag
rosbag play localization_data.bag

# 4. 监控定位效果
rviz  # 查看 /pointlio/path 和 /pointlio/global_map
```

---

## 🛠️ 进阶：代码层面实现

如果需要在代码中实现更高级的定位功能，需要修改 `laserMapping.cpp`：

### 关键修改点
1. **加载预建地图**：在初始化时读取PCD文件到ikd-tree
2. **禁用建图逻辑**：跳过关键帧添加和地图扩展
3. **仅做位姿跟踪**：每帧与全局地图匹配
4. **初始位姿处理**：订阅 `/initialpose` 话题

这部分需要更深入的代码修改，建议先使用参数开关测试基本功能。

---

更多信息请参考：
- Point-LIO 原始仓库：https://github.com/hku-mars/Point-LIO
- FAST-LIO-LOCALIZATION：https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION
