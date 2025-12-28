#!/usr/bin/python3
# coding=utf8

import copy
import threading
import time
import os

import open3d as o3d
import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import numpy as np
import tf
import tf.transformations
import sys


SCAN_VOXEL_SIZE = 0.1
MAP_VOXEL_SIZE = 0.5


def voxel_down_sample(pcd, voxel_size):
    """
    安全地对 Open3D 点云进行体素降采样。
    pcd 必须是 open3d.geometry.PointCloud 类型。
    """
    return pcd.voxel_down_sample(voxel_size=voxel_size)


def registration_at_scale(pc_scan, pc_map, initial, scale):
    # scan_down = voxel_down_sample(pc_scan, SCAN_VOXEL_SIZE * scale)
    # map_down = voxel_down_sample(pc_map, MAP_VOXEL_SIZE * scale)
    scan_down = pc_scan
    map_down = pc_map
    result_icp = o3d.pipelines.registration.registration_icp(
        source=scan_down,
        target=map_down,
        max_correspondence_distance=1.0 * scale,
        init=initial,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20)
    )
    return result_icp.transformation, result_icp.fitness


def load_cloud():
    """
    加载 scan 和 global map 点云。
    返回: (scan_pcd, map_pcd) 均为 open3d.geometry.PointCloud
    """
    scan_cloud_path = os.path.expanduser("~/pcd_debug/scan.pcd")
    map_cloud_path = os.path.expanduser("~/pcd_debug/global_cloud.pcd")

    if not os.path.exists(scan_cloud_path):
        raise FileNotFoundError(f"Scan point cloud not found: {scan_cloud_path}")
    if not os.path.exists(map_cloud_path):
        raise FileNotFoundError(f"Map point cloud not found: {map_cloud_path}")

    scan_pcd = o3d.io.read_point_cloud(scan_cloud_path)
    map_pcd = o3d.io.read_point_cloud(map_cloud_path)

    if len(scan_pcd.points) == 0:
        raise ValueError("Scan point cloud is empty!")
    if len(map_pcd.points) == 0:
        raise ValueError("Map point cloud is empty!")

    print(f"Loaded scan: {len(scan_pcd.points)} points")
    print(f"Loaded map:  {len(map_pcd.points)} points")

    return scan_pcd, map_pcd


def transform_to_pose_msg(T):
    """将 4x4 变换矩阵转为 geometry_msgs/Pose"""
    pose = Pose()
    pose.position.x = T[0, 3]
    pose.position.y = T[1, 3]
    pose.position.z = T[2, 3]

    # 从旋转矩阵提取四元数
    q = tf.transformations.quaternion_from_matrix(T)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose


def main():
    rospy.init_node('scan_to_map_localizer', anonymous=True)

    # 可选：发布定位结果
    pose_pub = rospy.Publisher("/icp_localization/pose", PoseWithCovarianceStamped, queue_size=1)

    try:
        scan_pcd, map_pcd = load_cloud()
    except Exception as e:
        rospy.logerr(f"Failed to load point clouds: {e}")
        return

    # 初始猜测：单位矩阵（或可从 IMU/Odometry 获取）
    T = np.eye(4)

    # 多尺度策略：从粗到精
    scales = [4.0, 2.0, 1.0]

    for scale in scales:
        rospy.loginfo(f"Running ICP at scale = {scale}")
        try:
            T, fitness = registration_at_scale(scan_pcd, map_pcd, T, scale)
            rospy.loginfo(f"Scale {scale}: fitness = {fitness:.4f}")
        except Exception as e:
            rospy.logwarn(f"ICP failed at scale {scale}: {e}")
            continue

    # 输出最终结果
    rospy.loginfo("Final transformation:\n{}".format(T))

    # 转为 ROS Pose 消息
    pose_msg = transform_to_pose_msg(T)

    # 发布一次（或持续发布）
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.pose.pose = pose_msg
    # 可选：填充协方差（这里设为未知）
    msg.pose.covariance = [0.0] * 36
    pose_pub.publish(msg)

    rospy.loginfo("Published ICP localization result to /icp_localization/pose")

    # 保持节点运行几秒以便消息被接收
    rospy.sleep(2.0)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass