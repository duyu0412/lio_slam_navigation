#!/usr/bin/python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import copy
import threading
import time

import open3d as o3d
import rospy
import ros_numpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import numpy as np
import tf
import tf.transformations
import sys
import os
import signal

global_map = None
initialized = False
T_map_to_odom = np.eye(4)
cur_odom = None
cur_scan = None
shutdown_flag = False

def signal_handler(sig, frame):
    global shutdown_flag
    rospy.loginfo('Received shutdown signal, exiting...')
    shutdown_flag = True
    sys.exit(0)


def pose_to_mat(pose_msg):
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
    )


def msg_to_array(pc_msg):
    pc_array = ros_numpy.numpify(pc_msg)
    pc = np.zeros([len(pc_array), 3])
    pc[:, 0] = pc_array['x']
    pc[:, 1] = pc_array['y']
    pc[:, 2] = pc_array['z']
    return pc


def registration_at_scale(pc_scan, pc_map, initial, scale):
    scan_down = voxel_down_sample(pc_scan, SCAN_VOXEL_SIZE * scale)
    map_down = voxel_down_sample(pc_map, MAP_VOXEL_SIZE * scale)
    # rospy.loginfo('Registration at scale {}: scan {} pts, map {} pts'.format(
    #     scale, len(scan_down.points), len(map_down.points)))
    result_icp = o3d.pipelines.registration.registration_icp(
        source=scan_down,
        target=map_down,
        max_correspondence_distance=1.0 * scale,
        init=initial,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20)
    )
    return result_icp.transformation, result_icp.fitness


def inverse_se3(trans):
    trans_inverse = np.eye(4)
    # R
    trans_inverse[:3, :3] = trans[:3, :3].T
    # t
    trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
    return trans_inverse


def publish_point_cloud(publisher, header, pc):
    data = np.zeros(len(pc), dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('intensity', np.float32),
    ])
    data['x'] = pc[:, 0]
    data['y'] = pc[:, 1]
    data['z'] = pc[:, 2]
    if pc.shape[1] == 4:
        data['intensity'] = pc[:, 3]
    msg = ros_numpy.msgify(PointCloud2, data)
    msg.header = header
    publisher.publish(msg)


def crop_global_map_in_FOV(global_map, T_map_to_base_link):
    """
    T_map_to_base_link: 机器人(Lidar)在地图坐标系下的真实位姿 (4x4 Matrix)
    """
    T_base_link_to_map = inverse_se3(T_map_to_base_link)
    global_map_points = np.array(global_map.points)
    points_homo = np.column_stack([global_map_points, np.ones(len(global_map_points))])
    global_map_in_base_link = np.matmul(T_base_link_to_map, points_homo.T).T

    x = global_map_in_base_link[:, 0]
    y = global_map_in_base_link[:, 1]
    z = global_map_in_base_link[:, 2]
    dist_sq = x**2 + y**2 + z**2
    # angle = np.arctan2(x, z)
    
    if FOV >= 2 * np.pi - 1e-3:
        mask = dist_sq < FOV_FAR**2
    else:
        half_fov = FOV / 2.0
        tan_half_fov = np.tan(half_fov)
        # 后向 FOV: z < 0 且 |y| < (-z) * tan(FOV/2)
        fov_mask = (z > 0) & (np.abs(y) < z * tan_half_fov)
        mask = (dist_sq < FOV_FAR**2) & fov_mask

    indices = np.where(mask)[0]
    
    global_map_in_FOV = o3d.geometry.PointCloud()
    global_map_in_FOV.points = o3d.utility.Vector3dVector(global_map_points[indices])

    header = rospy.Header()
    header.frame_id = 'map'
    header.stamp = rospy.Time.now()
    publish_point_cloud(pub_submap, header, np.asarray(global_map_in_FOV.points))

    return global_map_in_FOV


def global_localization(pose_estimation):
    global global_map, cur_scan, cur_odom, T_map_to_odom
    # 用icp配准
    # print(global_map, cur_scan, T_map_to_odom)
    pose_estimation = np.eye(4)
    # rospy.loginfo('Global localization by scan-to-map matching......')

    # TODO 这里注意线程安全
    scan_tobe_mapped = copy.copy(cur_scan)
    odom_now = copy.copy(cur_odom)
    # tic = time.time()
    T_odom_to_base_link = pose_to_mat(odom_now)

    T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
    global_map_in_FOV = crop_global_map_in_FOV(global_map, T_map_to_base_link)

    # 粗配准
    
    transformation_0, fitness_0 = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=4)

    # 精配准
    transformation_1, fitness_1 = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=transformation_0,
                                                    scale=1)
    fitness = fitness_0 if fitness_0 >= fitness_1 else fitness_1
    transformation = transformation_0 if fitness_0 >= fitness_1 else transformation_1
    # if fitness < LOCALIZATION_TH:
    #     save_dir = os.path.expanduser("~/pcd_debug")
    #     os.makedirs(save_dir, exist_ok=True)
    #     # 合并两个点云（global_map_in_FOV + scan_tobe_mapped）
    #     combined_pcd = global_map_in_FOV + scan_tobe_mapped
    #     # 保存到同一个文件
    #     combined_path = os.path.join(save_dir, "combined_global_and_scan.pcd")
    #     global_cloud = os.path.join(save_dir, "global_cloud.pcd")
    #     scan_cloud = os.path.join(save_dir, "scan.pcd")
    #     o3d.io.write_point_cloud(combined_path, combined_pcd, write_ascii=False)
    #     o3d.io.write_point_cloud(global_cloud, global_map_in_FOV, write_ascii=False)
    #     o3d.io.write_point_cloud(scan_cloud, scan_tobe_mapped, write_ascii=False)
    #     rospy.loginfo(f"Saved combined point cloud to: {combined_path}")
    # toc = time.time()
    # rospy.loginfo('Time: {}'.format(toc - tic))
    # rospy.loginfo('')

    # 当全局定位成功时才更新map2odom
    if fitness > LOCALIZATION_TH:
        rospy.loginfo('Global localization success with fitness: {}'.format(fitness))
        # T_map_to_odom = np.matmul(transformation, pose_estimation)
        T_map_to_odom = transformation

        # 发布map_to_odom
        map_to_odom = Odometry()
        xyz = tf.transformations.translation_from_matrix(T_map_to_odom)
        quat = tf.transformations.quaternion_from_matrix(T_map_to_odom)
        map_to_odom.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
        map_to_odom.header.stamp = cur_odom.header.stamp
        map_to_odom.header.frame_id = 'map'
        pub_map_to_odom.publish(map_to_odom)
        return True
    else:
        rospy.logwarn('Not match!!!!')
        rospy.logwarn('{}'.format(transformation))
        rospy.logwarn('fitness score:{}'.format(fitness))
        return False


def voxel_down_sample(pcd, voxel_size):
    try:
        pcd_down = pcd.voxel_down_sample(voxel_size)
    except:
        # for opend3d 0.7 or lower
        pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
    return pcd_down


def initialize_global_map(pc_msg):
    global global_map

    global_map = o3d.geometry.PointCloud()
    pc_array = msg_to_array(pc_msg)[:, :3]
    global_map.points = o3d.utility.Vector3dVector(pc_array)
    
    global_map = voxel_down_sample(global_map, MAP_VOXEL_SIZE)
    rospy.loginfo('Global map downsampled. Final points: {}'.format(len(global_map.points)))


def cb_save_cur_odom(odom_msg):
    global cur_odom
    cur_odom = odom_msg


def cb_save_cur_scan(pc_msg):
    global cur_scan
    # 注意这里直接将scan转到odom系下了 不是lidar局部系
    pc_msg.header.frame_id = 'camera_init'
    pc_msg.header.stamp = rospy.Time().now()
    pub_pc_in_map.publish(pc_msg)

    # 转换为pcd
    # fastlio给的field有问题 处理一下
    pc_msg.fields = [pc_msg.fields[0], pc_msg.fields[1], pc_msg.fields[2],
                     pc_msg.fields[4], pc_msg.fields[5], pc_msg.fields[6],
                     pc_msg.fields[3], pc_msg.fields[7]]
    pc = msg_to_array(pc_msg)

    cur_scan = o3d.geometry.PointCloud()
    cur_scan.points = o3d.utility.Vector3dVector(pc[:, :3])


def thread_localization():
    global T_map_to_odom, shutdown_flag
    while not rospy.is_shutdown() and not shutdown_flag:
        # 每隔一段时间进行全局定位
        try:
            rospy.sleep(1 / FREQ_LOCALIZATION)
            # TODO 由于这里Fast lio发布的scan是已经转换到odom系下了 所以每次全局定位的初始解就是上一次的map2odom 不需要再拿odom了
            global_localization(T_map_to_odom)
        except rospy.ROSInterruptException:
            break


if __name__ == '__main__':
    MAP_VOXEL_SIZE = 0.8
    SCAN_VOXEL_SIZE = 0.1

    # Global localization frequency (HZ)
    FREQ_LOCALIZATION = 0.5

    # The threshold of global localization,
    # only those scan2map-matching with higher fitness than LOCALIZATION_TH will be taken
    LOCALIZATION_TH = 0.9

    # FOV(rad), modify this according to your LiDAR type
    FOV = 3.13

    # The farthest distance(meters) within FOV
    FOV_FAR = 15

    # 设置信号处理
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    rospy.init_node('fast_lio_localization')
    rospy.loginfo('Localization Node Inited...')

    # publisher
    pub_pc_in_map = rospy.Publisher('/cur_scan_in_map', PointCloud2, queue_size=1)
    pub_submap = rospy.Publisher('/submap', PointCloud2, queue_size=1)
    pub_map_to_odom = rospy.Publisher('/map_to_odom', Odometry, queue_size=1)

    rospy.Subscriber('/pointlio/cloud_registered', PointCloud2, cb_save_cur_scan, queue_size=1)
    rospy.Subscriber('/pointlio/odom', Odometry, cb_save_cur_odom, queue_size=1)

    # 初始化全局地图
    rospy.logwarn('Waiting for global map......')
    initialize_global_map(rospy.wait_for_message('/map', PointCloud2))

    # 初始化
    while not initialized:
        rospy.logwarn('Waiting for initial pose....')
        try:
            pose_msg = rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped)
        except rospy.ROSInterruptException:
            rospy.loginfo("Node shutdown while waiting for /initialpose.")
            sys.exit(0)
        # 等待初始位姿
        initial_pose = pose_to_mat(pose_msg)
        if cur_scan:
            initialized = global_localization(initial_pose)
        else:
            rospy.logwarn('First scan not received!!!!!')

    rospy.loginfo('')
    rospy.loginfo('Initialize successfully!!!!!!')
    rospy.loginfo('')
    # 开始定期全局定位
    loc_thread = threading.Thread(target=thread_localization)
    loc_thread.daemon = True  # 设置为daemon线程，主线程退出时自动结束
    loc_thread.start()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down global localization node...')
    finally:
        shutdown_flag = True
