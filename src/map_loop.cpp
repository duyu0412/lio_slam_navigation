// GTSAM
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
#include "Estimator.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
// Backend Optimization and Loop Closure Variables
gtsam::NonlinearFactorGraph gtSAMgraph;
gtsam::Values initialEstimate;
gtsam::ISAM2 *isam;
gtsam::Values isamCurrentEstimate;
gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;
gtsam::noiseModel::Diagonal::shared_ptr loopNoise;

vector<PointCloudXYZI::Ptr> keyframeCloudQueue;      // 世界坐标系点云（用于回环检测）
vector<PointCloudXYZI::Ptr> keyframeCloudBodyQueue;  // 激光雷达坐标系点云（用于保存优化后地图）
vector<gtsam::Pose3> keyframePoseQueue;
vector<double> keyframeTimeQueue;  // 关键帧时间戳队列
deque<int> recentKeyframeIds;

// Loop closure queues (thread-safe)
std::mutex mtxLoopInfo;
std::deque<std::pair<int, int>> loopIndexQueue;
std::deque<gtsam::Pose3> loopPoseQueue;
std::deque<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
std::map<int, int> loopIndexContainer;  // {current_id: history_id}
// Copy of keyframe data for thread safety
pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D(new pcl::PointCloud<PointType>());
std::vector<gtsam::Pose3> copy_keyframePoseQueue;
bool loopClosureActive = false;
int latestKeyframeId = 0;
int closestHistoryKeyframeId = -1;
void performLoopClosure();
// Global publishers for loop closure visualization
ros::Publisher pubLoopClosureMarkersGlobal;

void visualizeLoopClosureConstraints();
void addKeyframeAndDetectLoop();
void detectLoopClosure();
void performLoopClosure();
// 参考 LIO-SAM 实现：发布所有回环的可视化（使用优化后的位姿）
void visualizeLoopClosureConstraints()
{
    if (loopIndexContainer.empty())
        return;
    
    visualization_msgs::MarkerArray markerArray;
    
    // Loop nodes (sphere markers at loop closure keyframes)
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = "odom";
    markerNode.header.stamp = ros::Time::now();
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode.ns = "loop_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.3; 
    markerNode.scale.y = 0.3; 
    markerNode.scale.z = 0.3;
    markerNode.color.r = 0; 
    markerNode.color.g = 0.8; 
    markerNode.color.b = 1;
    markerNode.color.a = 1;
    
    // Loop edges (lines connecting loop closure pairs)
    visualization_msgs::Marker markerEdge;
    markerEdge.header.frame_id = "odom";
    markerEdge.header.stamp = ros::Time::now();
    markerEdge.action = visualization_msgs::Marker::ADD;
    markerEdge.type = visualization_msgs::Marker::LINE_LIST;  // LINE_LIST: 每两个点一条线
    markerEdge.ns = "loop_edges";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.1;
    markerEdge.color.r = 0.9; 
    markerEdge.color.g = 0.9; 
    markerEdge.color.b = 0;
    markerEdge.color.a = 1;
    
    // 遍历所有回环对，使用最新的优化后位姿
    for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
    {
        int key_cur = it->first;
        int key_pre = it->second;
        
        if (key_cur >= keyframePoseQueue.size() || key_pre >= keyframePoseQueue.size())
            continue;
        
        geometry_msgs::Point p;
        // Current keyframe position
        p.x = keyframePoseQueue[key_cur].x();
        p.y = keyframePoseQueue[key_cur].y();
        p.z = keyframePoseQueue[key_cur].z();
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
        
        // History keyframe position
        p.x = keyframePoseQueue[key_pre].x();
        p.y = keyframePoseQueue[key_pre].y();
        p.z = keyframePoseQueue[key_pre].z();
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
    }
    
    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
    

    pubLoopClosureMarkersGlobal.publish(markerArray);
}

// ========== 修改 addKeyframeAndDetectLoop 函数 ==========
void addKeyframeAndDetectLoop()
{
    // Get current pose
    gtsam::Pose3 currentPose;
    if (!use_imu_as_input)
    {
        currentPose = gtsam::Pose3(gtsam::Rot3(kf_output.x_.rot.normalized()), 
                                   gtsam::Point3(kf_output.x_.pos));
    }
    else
    {
        currentPose = gtsam::Pose3(gtsam::Rot3(kf_input.x_.rot.normalized()), 
                                   gtsam::Point3(kf_input.x_.pos));
    }

    // Add first keyframe
    if (keyframePoseQueue.empty())
    {
        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(
            gtsam::Symbol('x', 0), currentPose, priorNoise));
        initialEstimate.insert(gtsam::Symbol('x', 0), currentPose);
        latestKeyframeId = 0;
        printf("[Backend] First keyframe added at (%.2f, %.2f, %.2f)\n", 
               currentPose.x(), currentPose.y(), currentPose.z());
    }
    else
    {
        gtsam::Pose3 lastPose = keyframePoseQueue.back();
        
        // Check if this is a new keyframe
        if (currentPose.range(lastPose) < keyframe_add_dist_threshold &&
            gtsam::Rot3::Logmap(lastPose.rotation().between(currentPose.rotation())).norm() 
            < keyframe_add_angle_threshold)
        {
            return; // Not a keyframe
        }
        
        latestKeyframeId++;
        
        // Add odometry factor
        gtsam::Pose3 odometry = lastPose.between(currentPose);
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
            gtsam::Symbol('x', latestKeyframeId - 1), 
            gtsam::Symbol('x', latestKeyframeId), 
            odometry, odometryNoise));
        initialEstimate.insert(gtsam::Symbol('x', latestKeyframeId), currentPose);
        
        if (latestKeyframeId % 10 == 0)
        {
            printf("[Backend] Keyframe %d added. Total: %zu, Pos: (%.2f, %.2f, %.2f)\n", 
                   latestKeyframeId, keyframePoseQueue.size() + 1,
                   currentPose.x(), currentPose.y(), currentPose.z());
        }
    }

    // ========== Update ISAM2 (odometry factors only) ==========
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    
    gtSAMgraph.resize(0);
    initialEstimate.clear();

    // ========== Save keyframe data ==========
    // 保存世界坐标系点云（用于回环检测）
    PointCloudXYZI::Ptr thisKeyframeCloud(new PointCloudXYZI());
    pcl::copyPointCloud(*feats_down_world, *thisKeyframeCloud);
    keyframeCloudQueue.push_back(thisKeyframeCloud);
    
    // 保存激光雷达坐标系点云（用于保存优化后地图）
    PointCloudXYZI::Ptr thisKeyframeCloudBody(new PointCloudXYZI());
    pcl::copyPointCloud(*feats_down_body, *thisKeyframeCloudBody);
    keyframeCloudBodyQueue.push_back(thisKeyframeCloudBody);
    
    keyframePoseQueue.push_back(currentPose);
    keyframeTimeQueue.push_back(lidar_end_time);
    recentKeyframeIds.push_back(latestKeyframeId);
    
    // Limit recent keyframe queue size
    while (recentKeyframeIds.size() > 100)
    {
        recentKeyframeIds.pop_front();
    }

    // ========== Detect loop closure ==========
    if (loop_closure_enable_flag)
    {
        detectLoopClosure();
    }

    // ========== Process newly detected loop closures immediately ==========
    bool hasNewLoopClosure = false;
    mtxLoopInfo.lock();
    while (!loopIndexQueue.empty())
    {
        int loopKeyCur = loopIndexQueue.front().first;
        int loopKeyPre = loopIndexQueue.front().second;
        gtsam::Pose3 loopPose = loopPoseQueue.front();
        gtsam::noiseModel::Diagonal::shared_ptr loopNoise = loopNoiseQueue.front();
        
        loopIndexQueue.pop_front();
        loopPoseQueue.pop_front();
        loopNoiseQueue.pop_front();
        
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
            gtsam::Symbol('x', loopKeyCur),
            gtsam::Symbol('x', loopKeyPre),
            loopPose,
            loopNoise));
        
        printf("\033[1;33m[Backend] Loop closure factor added: %d -> %d\033[0m\n", 
               loopKeyCur, loopKeyPre);
        
        hasNewLoopClosure = true;
    }
    mtxLoopInfo.unlock();

    // ========== Update ISAM2 and correct poses if new loop closure ==========
    if (hasNewLoopClosure)
    {
        // Update with loop closure factors
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        
        gtSAMgraph.resize(0);
        initialEstimate.clear();
        
        // Get optimized poses
        isamCurrentEstimate = isam->calculateEstimate();
        
        // Update all keyframe poses including the latest one
        mtxLoopInfo.lock();
        for(int i = 0; i < keyframePoseQueue.size(); ++i)
        {
            if (isamCurrentEstimate.exists(gtsam::Symbol('x', i)))
            {
                keyframePoseQueue[i] = isamCurrentEstimate.at<gtsam::Pose3>(gtsam::Symbol('x', i));
                copy_keyframePoseQueue[i] = keyframePoseQueue[i];
            }
        }
        mtxLoopInfo.unlock();

        // Feedback to frontend
        gtsam::Pose3 latest_corrected_pose = keyframePoseQueue.back();
        
        printf("\033[1;33m[Backend] Pose correction applied to %zu keyframes\033[0m\n", 
               keyframePoseQueue.size());
        printf("[Backend] Corrected pose: (%.2f, %.2f, %.2f)\n", 
               latest_corrected_pose.x(), latest_corrected_pose.y(), 
               latest_corrected_pose.z());
        
        if (!use_imu_as_input)
        {
            kf_output.x_.pos = latest_corrected_pose.translation();
            kf_output.x_.rot = latest_corrected_pose.rotation().matrix();
        }
        else
        {
            kf_input.x_.pos = latest_corrected_pose.translation();
            kf_input.x_.rot = latest_corrected_pose.rotation().matrix();
        }
        
        // 重新发布所有回环的可视化（使用优化后的位姿）
        visualizeLoopClosureConstraints();
        printf("\033[1;32m[Backend] Loop closure visualization updated with optimized poses\033[0m\n");
    }

    // ========== Memory management ==========
    // 只清理用于回环检测的世界坐标系点云，保留 body frame 点云用于最终地图保存
    if (keyframeCloudQueue.size() > max_retained_keyframes_cloud)
    {
        for (int i = 0; i < keyframeCloudQueue.size() - max_retained_keyframes_cloud; ++i)
        {
            if (i < latestKeyframeId - history_search_num * 2)
            {
                if (keyframeCloudQueue[i] != nullptr)
                {
                    keyframeCloudQueue[i].reset();
                }
            }
        }
    }
}

// ========== 修改 detectLoopClosure 函数 ==========
void detectLoopClosure()
{
    closestHistoryKeyframeId = -1;
    
    if (keyframeTimeQueue.empty() || keyframePoseQueue.size() < 2)
        return;
    
    // Skip if current keyframe already has loop closure
    if (loopIndexContainer.find(latestKeyframeId) != loopIndexContainer.end())
        return;
    
    // Make thread-safe copy
    mtxLoopInfo.lock();
    copy_keyframePoseQueue = keyframePoseQueue;
    copy_cloudKeyPoses3D->clear();
    for (const auto& pose : keyframePoseQueue)
    {
        PointType p;
        p.x = pose.x();
        p.y = pose.y();
        p.z = pose.z();
        copy_cloudKeyPoses3D->push_back(p);
    }
    mtxLoopInfo.unlock();
    
    double currentTime = keyframeTimeQueue.back();
    
    // Build search candidates
    std::vector<int> candidateIds;
    for(int i = 0; i < (int)keyframeTimeQueue.size() - 1; ++i)
    {
        // Time threshold check
        if ((currentTime - keyframeTimeQueue[i]) < loop_closure_time_threshold)
            continue;
        
        // Index distance check
        if (abs(latestKeyframeId - i) < history_search_num)
            continue;
        
        // Check if cloud is available
        if (keyframeCloudQueue[i] != nullptr)
        {
            candidateIds.push_back(i);
        }
    }

    if (candidateIds.empty()) 
        return;

    // KD-tree search for nearest keyframe
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyframes(new pcl::KdTreeFLANN<PointType>());
    pcl::PointCloud<PointType>::Ptr cloudHistoryKeyframes(new pcl::PointCloud<PointType>());

    for (int id : candidateIds)
    {
        cloudHistoryKeyframes->push_back(copy_cloudKeyPoses3D->points[id]);
    }
    kdtreeHistoryKeyframes->setInputCloud(cloudHistoryKeyframes);

    PointType currentPose = copy_cloudKeyPoses3D->points.back();

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    kdtreeHistoryKeyframes->radiusSearch(currentPose, loop_closure_search_radius, 
                                        pointSearchInd, pointSearchSqDis);

    if (pointSearchInd.empty())
        return;

    // Find the best candidate
    for (int i = 0; i < pointSearchInd.size(); ++i)
    {
        int id = candidateIds[pointSearchInd[i]];
        closestHistoryKeyframeId = id;
        double distance = sqrt(pointSearchSqDis[i]);
        printf("[Loop] Candidate found! KeyframeID: %d, Distance: %.2f m\n", 
               id, distance);
        break;
    }

    if (closestHistoryKeyframeId != -1)
    {
        performLoopClosure();
    }
}

// ========== 重写 performLoopClosure 函数 ==========
void performLoopClosure()
{
    if (keyframeCloudQueue.empty() || closestHistoryKeyframeId == -1)
        return;

    int loopKeyCur = latestKeyframeId;        // Current keyframe
    int loopKeyPre = closestHistoryKeyframeId; // History keyframe

    printf("\n\033[1;36m====== Loop Closure Detection ======\033[0m\n");
    printf("[Loop] Current keyframe: %d, History keyframe: %d\n", loopKeyCur, loopKeyPre);

    // ========== Step 1: Extract clouds ==========
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
    
    // Extract current keyframe cloud (no neighbors for front LiDAR)
    {
        if (keyframeCloudQueue[loopKeyCur] != nullptr)
        {
            *cureKeyframeCloud = *keyframeCloudQueue[loopKeyCur];
        }
    }
    
    // Extract history keyframe cloud with neighbors
    {
        int nearby_range = 5; // historyKeyframeSearchNum
        for (int i = std::max(0, loopKeyPre - nearby_range); 
             i <= std::min((int)keyframeCloudQueue.size() - 1, loopKeyPre + nearby_range); 
             ++i)
        {
            if (keyframeCloudQueue[i] != nullptr)
            {
                *prevKeyframeCloud += *keyframeCloudQueue[i];
            }
        }
    }
    
    // Check cloud size
    if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
    {
        // printf("[Loop] Insufficient points - Current: %zu, History: %zu\n",
        //        cureKeyframeCloud->size(), prevKeyframeCloud->size());
        return;
    }
    
    printf("[Loop] Cloud size - Current: %zu, History: %zu\n",
           cureKeyframeCloud->size(), prevKeyframeCloud->size());

    // ========== Step 2: Downsample ==========
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    downSizeFilterICP.setLeafSize(0.1, 0.1, 0.1);
    
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloudDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloudDS(new pcl::PointCloud<PointType>());
    
    downSizeFilterICP.setInputCloud(cureKeyframeCloud);
    downSizeFilterICP.filter(*cureKeyframeCloudDS);
    
    downSizeFilterICP.setInputCloud(prevKeyframeCloud);
    downSizeFilterICP.filter(*prevKeyframeCloudDS);
    
    printf("[Loop] After downsample - Current: %zu, History: %zu\n",
           cureKeyframeCloudDS->size(), prevKeyframeCloudDS->size());

    // 保存原始点云到/home/user/SLAM/src/point_lio_unilidar/PCD/loop_closure_failure/以供调试
    // {
    //     static pcl::PCDWriter pcd_writer;
    //     std::string save_dir = "/home/user/SLAM/src/point_lio_unilidar/PCD/loop_closure_failure/";
    //     std::string source_file = save_dir + "source_kf_" + std::to_string(loopKeyCur) + ".pcd";
    //     std::string target_file = save_dir + "target_kf_" + std::to_string(loopKeyPre) + ".pcd";
    //     pcd_writer.writeBinary(source_file, *cureKeyframeCloudDS);
    //     pcd_writer.writeBinary(target_file, *prevKeyframeCloudDS);
    //     printf("[Loop] Saved debug clouds: %s, %s\n", source_file.c_str(), target_file.c_str());
    // }

    // ========== Step 3: Multi-Yaw ICP (for front LiDAR) ==========
    // printf("[Loop] Running Multi-Yaw ICP...\n");
    
    static pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(loop_closure_search_radius * 2);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);
    
    std::vector<float> yaw_angles = {0, 90, 180, 270};
    
    float bestScore = std::numeric_limits<float>::max();
    Eigen::Matrix4f bestTransform = Eigen::Matrix4f::Identity();
    bool bestConverged = false;
    float bestYaw = 0;
    
    for (float yaw_deg : yaw_angles)
    {
        // Create initial guess with yaw rotation
        float yaw_rad = yaw_deg * M_PI / 180.0f;
        Eigen::Matrix4f initialGuess = Eigen::Matrix4f::Identity();
        initialGuess(0,0) = cos(yaw_rad);  
        initialGuess(0,1) = -sin(yaw_rad);
        initialGuess(1,0) = sin(yaw_rad);  
        initialGuess(1,1) = cos(yaw_rad);
        
        // Transform source cloud with initial guess
        pcl::PointCloud<PointType>::Ptr sourceTransformed(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*cureKeyframeCloudDS, *sourceTransformed, initialGuess);
        
        // Run ICP
        icp.setInputSource(sourceTransformed);
        icp.setInputTarget(prevKeyframeCloudDS);

        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);
        
        float score = icp.getFitnessScore();
        bool converged = icp.hasConverged();
        
        // printf("[Loop]   Yaw=%3.0f°: Score=%.4f, Converged=%s", 
        //        yaw_deg, score, converged ? "Yes" : "No");
        
        if (converged && score < bestScore)
        {
            bestScore = score;
            // Combine initial guess with ICP result
            bestTransform = icp.getFinalTransformation() * initialGuess;
            bestConverged = true;
            bestYaw = yaw_deg;
            printf(" (NEW BEST)");
        }
        printf("\n");
    }

    // ========== Step 4: Check ICP result ==========
    if (!bestConverged || bestScore > history_fitness_score)
    {
        printf("\033[1;31m[Loop] ICP FAILED! Score: %.4f (threshold: %.4f)\033[0m\n", 
               bestScore, history_fitness_score);
        printf("\033[1;36m=====================================\033[0m\n\n");
        return;
    }
    
    printf("\033[1;32m[Loop] ICP SUCCESS! Score: %.4f (Best Yaw: %.0f°)\033[0m\n", 
           bestScore, bestYaw);
    
    // ========== Step 5: 计算相对位姿 (参考LIO-SAM) ==========
    mtxLoopInfo.lock();
    gtsam::Pose3 poseCurrent = copy_keyframePoseQueue[loopKeyCur];  // tWrong
    gtsam::Pose3 poseHistory = copy_keyframePoseQueue[loopKeyPre];
    mtxLoopInfo.unlock();
    
    // correctionLidarFrame: ICP在世界坐标系的修正
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = bestTransform;
    
    // tWrong: 当前帧的（有漂移的）世界位姿
    Eigen::Affine3f tWrong;
    tWrong.matrix() = poseCurrent.matrix().cast<float>();
    
    // tCorrect: 修正后的当前帧世界位姿
    // pre-multiplying: 在固定(世界)坐标系下连续变换
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
    
    // 提取修正后的位姿
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseCorrected = gtsam::Pose3(
        gtsam::Rot3::RzRyRx(roll, pitch, yaw), 
        gtsam::Point3(x, y, z));
    
    // 计算相对位姿: 从修正后的当前帧到历史帧
    gtsam::Pose3 relativePose = poseCorrected.between(poseHistory);
    
    printf("[Loop] Current pose (wrong):     (%.2f, %.2f, %.2f)\n", 
           poseCurrent.x(), poseCurrent.y(), poseCurrent.z());
    printf("[Loop] Current pose (corrected): (%.2f, %.2f, %.2f)\n", 
           poseCorrected.x(), poseCorrected.y(), poseCorrected.z());
    printf("[Loop] History pose:             (%.2f, %.2f, %.2f)\n", 
           poseHistory.x(), poseHistory.y(), poseHistory.z());
    printf("[Loop] Relative pose (cur->pre): trans=(%.3f, %.3f, %.3f)\n",
           relativePose.x(), relativePose.y(), relativePose.z());

    // ========== Step 6: Create constraint noise model ==========
    gtsam::Vector Vector6(6);
    float noiseScore = bestScore;
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = 
        gtsam::noiseModel::Diagonal::Variances(Vector6);

    // ========== Step 7: Add to loop closure queue ==========
    mtxLoopInfo.lock();
    loopIndexQueue.push_back(std::make_pair(loopKeyCur, loopKeyPre));
    loopPoseQueue.push_back(relativePose);  // T_pre_cur
    loopNoiseQueue.push_back(constraintNoise);
    loopIndexContainer[loopKeyCur] = loopKeyPre;
    mtxLoopInfo.unlock();

    printf("\033[1;32m[Loop] Loop constraint added to queue!\033[0m\n");
    printf("\033[1;36m=====================================\033[0m\n\n");
    
    // Visualize (will be updated after optimization)
    visualizeLoopClosureConstraints();
}