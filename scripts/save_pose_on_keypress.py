#!/usr/bin/python3
# coding=utf8
"""
实时保存aft_mapped位姿
按 S 键保存当前位姿到文件
按 Q 键退出程序
"""
from __future__ import print_function, division, absolute_import

import rospy
from nav_msgs.msg import Odometry
import sys
import os
import termios
import tty
import select
import threading
from datetime import datetime

cur_odom = None
pose_count = 0
save_file = None


def get_key():
    """非阻塞获取键盘输入"""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def odom_callback(msg):
    """订阅里程计消息"""
    global cur_odom
    cur_odom = msg


def save_current_pose():
    """保存当前位姿到文件"""
    global cur_odom, pose_count, save_file
    
    if cur_odom is None:
        print("\r[WARN] No odometry data received yet!                    ")
        return
    
    pose_count += 1
    timestamp = cur_odom.header.stamp.to_sec()
    
    # 位置
    x = cur_odom.pose.pose.position.x
    y = cur_odom.pose.pose.position.y
    z = cur_odom.pose.pose.position.z
    
    # 四元数
    qx = cur_odom.pose.pose.orientation.x
    qy = cur_odom.pose.pose.orientation.y
    qz = cur_odom.pose.pose.orientation.z
    qw = cur_odom.pose.pose.orientation.w
    
    # 写入文件：时间戳 x y z qx qy qz qw
    line = f"{timestamp:.6f} {x:.6f} {y:.6f} {z:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n"
    save_file.write(line)
    save_file.flush()  # 立即写入磁盘
    
    print(f"\r[{pose_count}] Saved pose: x={x:.3f}, y={y:.3f}, z={z:.3f}                    ")


def show_current_pose():
    """显示当前位姿（不保存）"""
    global cur_odom
    
    if cur_odom is None:
        return
    
    x = cur_odom.pose.pose.position.x
    y = cur_odom.pose.pose.position.y
    z = cur_odom.pose.pose.position.z
    
    # 使用\r实现原地刷新显示
    sys.stdout.write(f"\rCurrent pose: x={x:7.3f}, y={y:7.3f}, z={z:7.3f} | Press 'S' to save, 'Q' to quit")
    sys.stdout.flush()


def keyboard_listener():
    """键盘监听线程"""
    global save_file
    
    # 保存终端设置
    old_settings = termios.tcgetattr(sys.stdin)
    
    try:
        # 设置为非规范模式
        tty.setcbreak(sys.stdin.fileno())
        
        print("\n" + "="*70)
        print("Pose Saver Started!")
        print("="*70)
        print("Press 'S' to save current pose")
        print("Press 'Q' to quit")
        print("="*70 + "\n")
        
        rate = rospy.Rate(10)  # 10Hz显示刷新
        
        while not rospy.is_shutdown():
            # 显示当前位姿
            show_current_pose()
            
            # 检查键盘输入
            key = get_key()
            if key:
                key = key.lower()
                if key == 's':
                    print("\r[INFO] Saving pose...                              ")
                    save_current_pose()
                elif key == 'q':
                    print("\r[INFO] Quitting...                                 ")
                    break
            
            rate.sleep()
            
    except KeyboardInterrupt:
        print("\n[INFO] Keyboard interrupt detected")
    finally:
        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        if save_file:
            save_file.close()
            print(f"\n[INFO] Saved {pose_count} poses to {save_file.name}")
        print("[INFO] Shutting down...")
        rospy.signal_shutdown("User requested shutdown")


if __name__ == '__main__':
    try:
        rospy.init_node('pose_saver', anonymous=True)
        
        # 创建保存文件
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        save_dir = os.path.expanduser("~/SLAM/saved_poses")
        os.makedirs(save_dir, exist_ok=True)
        save_filename = os.path.join(save_dir, f"poses_{timestamp_str}.txt")
        save_file = open(save_filename, 'w')
        
        # 写入文件头
        save_file.write("# Saved poses from /pointlio/odom (aft_mapped frame)\n")
        save_file.write("# Format: timestamp x y z qx qy qz qw\n")
        save_file.write("# " + "="*60 + "\n")
        
        print(f"[INFO] Saving poses to: {save_filename}")
        
        # 订阅里程计话题
        rospy.Subscriber('/pointlio/odom', Odometry, odom_callback, queue_size=1)
        
        # 等待第一条消息
        print("[INFO] Waiting for odometry data...")
        while cur_odom is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        if not rospy.is_shutdown():
            print("[INFO] Odometry data received, ready to save poses!")
            # 启动键盘监听
            keyboard_listener()
        
    except Exception as e:
        print(f"[ERROR] {e}")
        if save_file:
            save_file.close()
