#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
import tf
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class PCDLocalization:
    """
    基于点云地图和ICP算法的ROS定位节点。
    """
    def __init__(self):
        rospy.init_node('pcd_localization_node', anonymous=True)
        
        # --- 参数配置 ---
        self.map_pcd_path = rospy.get_param('~map_pcd_path', '/path/to/map.pcd')
        self.pointcloud_topic = rospy.get_param('~pointcloud_topic', '/points_raw')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.lidar_frame = rospy.get_param('~lidar_frame', 'lidar') # 接收点云的原始坐标系
        self.voxel_size = rospy.get_param('~voxel_size', 0.1)     # 体素下采样尺寸
        self.local_map_radius = rospy.get_param('~local_map_radius', 30.0) # 局部地图提取半径
        self.icp_max_iter = rospy.get_param('~icp_max_iter', 50)
        self.icp_threshold = rospy.get_param('~icp_threshold', 0.5) # ICP 距离阈值
        self.icp_fitness_threshold = rospy.get_param('~icp_fitness_threshold', 0.3) # 匹配质量阈值
        self.fov_angle = rospy.get_param('~fov_angle', 360.0)     # FOV角度 (360表示全向)
        
        # --- 状态变量 ---
        self.map_cloud = None
        self.map_kdtree = None
        self.current_pose = None
        self.initial_pose_received = False
        
        # --- TF监听和发布 ---
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # --- 发布器 ---
        self.pose_pub = rospy.Publisher(
            '/localized_pose',
            PoseStamped,
            queue_size=1
        )
        self.odom_pub = rospy.Publisher(
            '/localized_odom',
            Odometry,
            queue_size=1
        )
        self.map_pub = rospy.Publisher( # 地图可视化 (latch=True 确保 Rviz 能接收)
            '/map_pointcloud',
            PointCloud2,
            queue_size=1,
            latch=True
        )
        self.filtered_cloud_pub = rospy.Publisher( # 裁剪后的点云可视化
            '/filtered_pointcloud',
            PointCloud2,
            queue_size=1
        )
        
        # --- 加载地图 ---
        self.load_map()
        
        # --- 订阅器 ---
        self.pc_sub = rospy.Subscriber(
            self.pointcloud_topic,  
            PointCloud2,  
            self.pointcloud_callback,
            queue_size=1
        )
        self.initial_pose_sub = rospy.Subscriber(
            '/initialpose',
            PoseWithCovarianceStamped,
            self.initial_pose_callback,
            queue_size=1
        )
        
        rospy.loginfo("PCD Localization Node initialized and waiting for initial pose...")
        
    def load_map(self):
        """加载PCD地图文件, 进行预下采样并构建KD树"""
        try:
            rospy.loginfo(f"Loading map from {self.map_pcd_path}...")
            pcd = o3d.io.read_point_cloud(self.map_pcd_path)
            
            # 预下采样，提高KD树的效率
            if self.voxel_size > 0:
                pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)
            
            self.map_cloud = np.asarray(pcd.points)
            
            # 构建KD树用于快速最近邻搜索
            self.map_kdtree = cKDTree(self.map_cloud)
            
            rospy.loginfo(f"Map loaded successfully! Points: {len(self.map_cloud)}")
            
            # 发布地图点云用于可视化
            self.publish_map_pointcloud()
            
        except Exception as e:
            rospy.logerr(f"Failed to load map: {e}")
            rospy.signal_shutdown("Map loading failed")
            
    def publish_map_pointcloud(self):
        """发布地图点云用于RViz可视化"""
        if self.map_cloud is None:
            return
        
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.map_frame
        
        # 转换为PointCloud2消息
        points = [(p[0], p[1], p[2]) for p in self.map_cloud]
        pc_msg = pc2.create_cloud_xyz32(header, points)
        
        self.map_pub.publish(pc_msg)

    def publish_filtered_cloud(self, points, header):
        """发布FOV裁剪后的点云"""
        if len(points) == 0:
            return
        
        filtered_header = rospy.Header()
        filtered_header.stamp = header.stamp
        filtered_header.frame_id = self.base_frame # 假设发布到 base_frame
        
        # 转换为PointCloud2消息
        points_list = [(p[0], p[1], p[2]) for p in points]
        pc_msg = pc2.create_cloud_xyz32(filtered_header, points_list)
        
        self.filtered_cloud_pub.publish(pc_msg)
        
    def initial_pose_callback(self, msg):
        """接收RViz的初始位姿"""
        self.current_pose = msg.pose.pose
        self.initial_pose_received = True
        
        rospy.loginfo("Initial pose received from RViz. Starting localization loop.")
        
    def filter_fov(self, points):
        """根据FOV角度过滤点云（仅基于x-y平面角度）"""
        if self.fov_angle >= 360.0 or len(points) == 0:
            return points
        
        # 计算每个点的角度（相对于x轴）
        angles = np.arctan2(points[:, 1], points[:, 0])
        angles_deg = np.degrees(angles)
        
        # FOV过滤：假设FOV对称分布在x轴两侧
        half_fov = self.fov_angle / 2.0
        mask = np.abs(angles_deg) <= half_fov
        
        return points[mask]
        
    def pointcloud_callback(self, msg):
        """处理点云话题数据"""
        if not self.initial_pose_received:
            return
        if self.map_kdtree is None:
            rospy.logwarn_throttle(5.0, "Map not loaded or KD-Tree not ready.")
            return

        try:
            # 1. 转换PointCloud2为numpy数组
            points_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            if len(points_list) < 10:
                return
            scan_cloud = np.array(points_list)
            
            # 2. FOV过滤 (在LiDAR本地坐标系下进行)
            scan_cloud_filtered = self.filter_fov(scan_cloud)
            
            if len(scan_cloud_filtered) < 100:
                rospy.logwarn_throttle(2.0, "Not enough points after FOV filtering.")
                return

            # 3. TF 转换：将点云从其原始帧转换到 base_frame
            # ICP 需要将扫描点云转换到 robot base 坐标系 (即 base_frame)
            if msg.header.frame_id != self.base_frame:
                try:
                    self.tf_listener.waitForTransform(
                        self.base_frame, 
                        msg.header.frame_id, 
                        msg.header.stamp, 
                        rospy.Duration(0.1)
                    )
                    (trans, rot) = self.tf_listener.lookupTransform(
                        self.base_frame, 
                        msg.header.frame_id, 
                        msg.header.stamp
                    )
                    
                    # 构建变换矩阵
                    transform_matrix = self.tf_listener.fromTranslationRotation(trans, rot)
                    R = transform_matrix[:3, :3]
                    t = transform_matrix[:3, 3]
                    
                    # 应用变换
                    scan_cloud_for_icp = (R @ scan_cloud_filtered.T).T + t
                    
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logwarn_throttle(1.0, f"TF lookup failed for {msg.header.frame_id} to {self.base_frame}: {e}")
                    return
            else:
                scan_cloud_for_icp = scan_cloud_filtered

            # 4. 发布转换后的点云用于可视化 (发布在 base_frame)
            self.publish_filtered_cloud(scan_cloud_for_icp, msg.header)

            # 5. 体素下采样
            pcd_scan = o3d.geometry.PointCloud()
            pcd_scan.points = o3d.utility.Vector3dVector(scan_cloud_for_icp)
            if self.voxel_size > 0:
                pcd_scan = pcd_scan.voxel_down_sample(voxel_size=self.voxel_size)
            
            if len(pcd_scan.points) < 50:
                rospy.logwarn_throttle(2.0, "Not enough points after voxel downsampling.")
                return
            
            # 6. 执行ICP重定位
            self.perform_icp(pcd_scan, msg.header)
            
        except Exception as e:
            rospy.logerr(f"Error in pointcloud callback: {e}")
            
    def perform_icp(self, scan_pcd, header):
        """使用ICP进行点云配准"""
        # 从当前位姿提取变换矩阵 T_map_base (作为初始猜测)
        pos = self.current_pose.position
        ori = self.current_pose.orientation
        
        # 提取yaw角，构建初始变换矩阵 (忽略 roll 和 pitch)
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        
        init_transform = np.eye(4)
        init_transform[0:2, 0:2] = [[np.cos(yaw), -np.sin(yaw)], 
                                    [np.sin(yaw),  np.cos(yaw)]]
        init_transform[0:3, 3] = [pos.x, pos.y, pos.z]
        
        # 1. 从地图中提取局部子地图
        # scan_pcd 已经在 base_frame, 初始猜测 T_map_base 将其变换到 map 坐标系下的估计位置
        scan_center_base = np.mean(np.asarray(scan_pcd.points), axis=0)
        # 转换到地图坐标系的初始估计中心
        transformed_center = (init_transform @ np.append(scan_center_base, 1))[:3]
        
        # 查询附近的地图点
        local_map_indices = self.map_kdtree.query_ball_point(
            transformed_center, r=self.local_map_radius
        )
        
        if len(local_map_indices) < 100:
            rospy.logwarn_throttle(2.0, f"Not enough map points nearby ({len(local_map_indices)}).")
            return
        
        local_map_points = self.map_cloud[local_map_indices]
        pcd_map = o3d.geometry.PointCloud()
        pcd_map.points = o3d.utility.Vector3dVector(local_map_points)
        
        # 2. **改进点:** 对局部地图进行下采样
        if self.voxel_size > 0:
            pcd_map = pcd_map.voxel_down_sample(voxel_size=self.voxel_size)
        
        # 3. 执行ICP
        reg_result = o3d.pipelines.registration.registration_icp(
            source=scan_pcd, # base_frame 的点云
            target=pcd_map,  # map_frame 的点云
            max_correspondence_distance=self.icp_threshold,
            init=init_transform,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=self.icp_max_iter
            )
        )
        
        # 4. 更新位姿
        if reg_result.fitness > self.icp_fitness_threshold:
            # 最终变换矩阵 T_map_base_new
            transform = reg_result.transformation
            
            # 从变换矩阵提取新位姿
            new_x = transform[0, 3]
            new_y = transform[1, 3]
            new_z = transform[2, 3]
            new_yaw = np.arctan2(transform[1, 0], transform[0, 0])
            
            # 更新当前位姿
            self.current_pose.position.x = new_x
            self.current_pose.position.y = new_y
            self.current_pose.position.z = new_z
            
            quat = quaternion_from_euler(0, 0, new_yaw)
            self.current_pose.orientation.x = quat[0]
            self.current_pose.orientation.y = quat[1]
            self.current_pose.orientation.z = quat[2]
            self.current_pose.orientation.w = quat[3]
            
            # 发布结果
            self.publish_localization(header.stamp)
            
            rospy.loginfo_throttle(1.0, f"ICP Fitness: {reg_result.fitness:.3f} (Inliers: {reg_result.inlier_rmse:.3f}), "
                                      f"Pose: ({new_x:.2f}, {new_y:.2f}, {np.degrees(new_yaw):.1f}°)")
        else:
            rospy.logwarn_throttle(2.0, f"Poor ICP fitness: {reg_result.fitness:.3f}. Pose not updated.")
            
    def publish_localization(self, stamp):
        """发布重定位结果 (PoseStamped, Odometry, TF)"""
        if self.current_pose is None:
            return

        # 1. 发布 PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.map_frame
        pose_msg.pose = self.current_pose
        self.pose_pub.publish(pose_msg)
        
        # 2. 发布 Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.map_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose = self.current_pose
        # 注意: 协方差 (covariance) 字段在此简化版本中未填充
        self.odom_pub.publish(odom_msg)
        
        # 3. 发布 TF变换：map -> base_link
        pos = self.current_pose.position
        ori = self.current_pose.orientation
        self.tf_broadcaster.sendTransform(
            (pos.x, pos.y, pos.z),
            (ori.x, ori.y, ori.z, ori.w),
            stamp,
            self.base_frame,
            self.map_frame
        )
        
    def run(self):
        """主循环，持续发布TF变换"""
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            # 持续发布 TF，确保 map -> base_link 变换链的稳定
            if self.initial_pose_received and self.current_pose is not None:
                current_time = rospy.Time.now()
                pos = self.current_pose.position
                ori = self.current_pose.orientation
                
                # 发布 map -> base_link
                self.tf_broadcaster.sendTransform(
                    (pos.x, pos.y, pos.z),
                    (ori.x, ori.y, ori.z, ori.w),
                    current_time,
                    self.base_frame,
                    self.map_frame
                )
                
                # 注意: 如果需要 lidar_frame 的 TF (base_link -> lidar_frame)
                # 这个变换通常由机器人的 URDF 或另一个节点发布。
                
            rate.sleep()


if __name__ == '__main__':
    try:
        node = PCDLocalization()
        node.run()
    except rospy.ROSInterruptException:
        pass