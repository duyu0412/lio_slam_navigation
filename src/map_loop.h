// #ifndef LOOP_H
// #define LOOP_H
#pragma once

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "parameters.h"
#include "map_loop.h"

extern gtsam::NonlinearFactorGraph gtSAMgraph;
extern gtsam::Values initialEstimate;
extern gtsam::ISAM2 *isam;
extern gtsam::Values isamCurrentEstimate;
extern gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
extern gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;
extern gtsam::noiseModel::Diagonal::shared_ptr loopNoise;

extern vector<PointCloudXYZI::Ptr> keyframeCloudQueue;      // 世界坐标系点云（用于回环检测）
extern vector<PointCloudXYZI::Ptr> keyframeCloudBodyQueue;  // 激光雷达坐标系点云（用于保存优化后地图）
extern vector<gtsam::Pose3> keyframePoseQueue;
extern vector<double> keyframeTimeQueue;  // 关键帧时间戳队列
extern deque<int> recentKeyframeIds;

// Loop closure queues (thread-safe)
extern std::mutex mtxLoopInfo;
extern std::deque<std::pair<int, int>> loopIndexQueue;
extern std::deque<gtsam::Pose3> loopPoseQueue;
extern std::deque<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
extern std::map<int, int> loopIndexContainer;  // {current_id: history_id}
// Copy of keyframe data for thread safety
extern pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
extern std::vector<gtsam::Pose3> copy_keyframePoseQueue;
extern ros::Publisher pubLoopClosureMarkersGlobal;

extern bool loopClosureActive;
extern int latestKeyframeId;
extern int closestHistoryKeyframeId;
extern void performLoopClosure();
