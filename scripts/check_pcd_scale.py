#!/usr/bin/python3
# coding=utf8
"""检查PCD文件的尺度和统计信息"""

import open3d as o3d
import numpy as np
import sys

def check_pcd(pcd_file):
    print(f"\n检查PCD文件: {pcd_file}")
    print("=" * 60)
    
    # 加载PCD
    pcd = o3d.io.read_point_cloud(pcd_file)
    points = np.asarray(pcd.points)
    
    if len(points) == 0:
        print("错误: PCD文件为空!")
        return
    
    # 统计信息
    print(f"点数: {len(points)}")
    
    # 边界信息
    min_pt = np.min(points, axis=0)
    max_pt = np.max(points, axis=0)
    center = np.mean(points, axis=0)
    range_pt = max_pt - min_pt
    
    print(f"\n边界框:")
    print(f"  最小值: [{min_pt[0]:.2f}, {min_pt[1]:.2f}, {min_pt[2]:.2f}]")
    print(f"  最大值: [{max_pt[0]:.2f}, {max_pt[1]:.2f}, {max_pt[2]:.2f}]")
    print(f"  中心点: [{center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f}]")
    print(f"  范围:   [{range_pt[0]:.2f}, {range_pt[1]:.2f}, {range_pt[2]:.2f}]")
    
    # 采样一些点
    print(f"\n前10个点样本:")
    for i in range(min(10, len(points))):
        print(f"  点{i}: [{points[i,0]:.4f}, {points[i,1]:.4f}, {points[i,2]:.4f}]")
    
    # 检查是否有异常的单位转换（如m->mm）
    if np.max(np.abs(points)) > 1000:
        print(f"\n警告: 检测到大数值点云 (max={np.max(np.abs(points)):.2f})")
        print("可能存在单位问题 (米->毫米 转换?)")
    
    print("=" * 60)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: ./check_pcd_scale.py <pcd_file1> [pcd_file2] ...")
        sys.exit(1)
    
    for pcd_file in sys.argv[1:]:
        try:
            check_pcd(pcd_file)
        except Exception as e:
            print(f"错误读取 {pcd_file}: {e}")
