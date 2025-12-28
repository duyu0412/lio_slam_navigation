#!/usr/bin/python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import copy
import threading
import time

import numpy as np
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import signal
import sys

cur_odom_to_baselink = None
cur_map_to_odom = None
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


def transform_fusion():
    global cur_odom_to_baselink, cur_map_to_odom, shutdown_flag

    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown() and not shutdown_flag:
        try:
            time.sleep(1 / FREQ_PUB_LOCALIZATION)
        except (KeyboardInterrupt, SystemExit):
            break

        # TODO 这里注意线程安全
        cur_odom = copy.copy(cur_odom_to_baselink)
        if cur_map_to_odom is not None:
            T_map_to_odom = pose_to_mat(cur_map_to_odom)
        else:
            T_map_to_odom = np.eye(4)

        # 使用当前时间 + 小偏移，确保TF在未来，避免TF_OLD_DATA警告
        current_time = rospy.Time.now()

        # 发布 map -> camera_init TF
        br.sendTransform(tf.transformations.translation_from_matrix(T_map_to_odom),
                         tf.transformations.quaternion_from_matrix(T_map_to_odom),
                         current_time,
                         'camera_init', 'map')
        
        if cur_odom is not None:
            T_odom_to_base_link = pose_to_mat(cur_odom)
            
            # 不再重复发布 camera_init -> aft_mapped TF
            # 这个变换已经由 laserMapping 发布，避免冲突
            # xyz_odom = tf.transformations.translation_from_matrix(T_odom_to_base_link)
            # quat_odom = tf.transformations.quaternion_from_matrix(T_odom_to_base_link)
            # br.sendTransform(xyz_odom, quat_odom,
            #                current_time,
            #                'aft_mapped', 'camera_init')
            
            # 发布全局定位的odometry (map -> aft_mapped)
            localization = Odometry()
            # 这里T_map_to_odom短时间内变化缓慢 暂时不考虑与T_odom_to_base_link时间同步
            T_map_to_base_link = np.matmul(T_map_to_odom, T_odom_to_base_link)
            xyz = tf.transformations.translation_from_matrix(T_map_to_base_link)
            quat = tf.transformations.quaternion_from_matrix(T_map_to_base_link)
            localization.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
            localization.twist = cur_odom.twist

            localization.header.stamp = cur_odom.header.stamp  # Odometry使用原始时间戳
            localization.header.frame_id = 'map'
            localization.child_frame_id = 'aft_mapped'
            # rospy.loginfo_throttle(1, '{}'.format(np.matmul(T_map_to_odom, T_odom_to_base_link)))
            pub_localization.publish(localization)


def cb_save_cur_odom(odom_msg):
    global cur_odom_to_baselink
    cur_odom_to_baselink = odom_msg


def cb_save_map_to_odom(odom_msg):
    global cur_map_to_odom
    cur_map_to_odom = odom_msg


if __name__ == '__main__':
    # tf and localization publishing frequency (HZ)
    FREQ_PUB_LOCALIZATION = 50
    
    # 设置信号处理
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    rospy.init_node('transform_fusion')
    rospy.loginfo('Transform Fusion Node Inited...')

    rospy.Subscriber('/pointlio/odom', Odometry, cb_save_cur_odom, queue_size=1)
    rospy.Subscriber('/map_to_odom', Odometry, cb_save_map_to_odom, queue_size=1)

    pub_localization = rospy.Publisher('/localization', Odometry, queue_size=1)

    # 发布定位消息
    fusion_thread = threading.Thread(target=transform_fusion)
    fusion_thread.daemon = True  # 设置为daemon线程，主线程退出时自动结束
    fusion_thread.start()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down transform fusion node...')
    finally:
        shutdown_flag = True
