/**
 * @file loop_closure_test.cpp
 * @brief 回环检测ICP匹配验证工具
 * 
 * 用于测试保存的局部坐标系下的source和target点云是否能够成功匹配
 * 
 * 编译方法:
 * cd /home/user/SLAM/src/point_lio_unilidar
 * mkdir -p build && cd build
 * cmake .. -DCMAKE_BUILD_TYPE=Release
 * make loop_closure_test
 * 
 * 或者直接使用g++:
 * g++ -std=c++17 -O3 -o loop_closure_test loop_closure_test.cpp \
 *     -I/usr/include/pcl-1.10 -I/usr/include/eigen3 \
 *     -lpcl_common -lpcl_io -lpcl_registration -lpcl_filters -lpcl_visualization
 * 
 * 使用方法:
 * ./loop_closure_test <source_pcd> <target_pcd> [--visualize]
 * 
 * 示例:
 * ./loop_closure_test ../PCD/loop_closure_failure/source_merged.pcd ../PCD/loop_closure_failure/target_merged.pcd --visualize
 */

#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <iomanip>
#include <limits>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloud;

// 前向声明
void visualize(PointCloud::Ptr source, PointCloud::Ptr target, 
               PointCloud::Ptr aligned, const std::string& title);

// 颜色定义
#define RED     "\033[1;31m"
#define GREEN   "\033[1;32m"
#define YELLOW  "\033[1;33m"
#define CYAN    "\033[1;36m"
#define RESET   "\033[0m"

void printUsage()
{
    std::cout << "\n用法: loop_closure_test <source_pcd> <target_pcd> [选项]\n\n";
    std::cout << "选项:\n";
    std::cout << "  --visualize, -v    可视化结果\n";
    std::cout << "  --merge-source     合并多个source点云 (使用通配符)\n";
    std::cout << "  --merge-target     合并多个target点云 (使用通配符)\n";
    std::cout << "  --downsample SIZE  下采样体素大小 (默认: 0.2)\n";
    std::cout << "  --method METHOD    匹配方法: gicp, icp, ndt (默认: gicp)\n";
    std::cout << "  --score-thresh T   匹配成功阈值 (默认: 0.5)\n";
    std::cout << "  --test-yaw-flip    测试180°翻转假设 (针对前向LiDAR回环检测)\n";
    std::cout << "  --test-multi-yaw   测试多个yaw角度初始猜测 (0°, 90°, 180°, 270°)\n";
    std::cout << "\n示例:\n";
    std::cout << "  ./loop_closure_test source_merged.pcd target_merged.pcd -v\n";
    std::cout << "  ./loop_closure_test source.pcd target.pcd --test-yaw-flip -v\n";
    std::cout << "  ./loop_closure_test --merge-source source_kf_*.pcd --merge-target target_kf_*.pcd\n\n";
}

PointCloud::Ptr loadAndMergePointClouds(const std::vector<std::string>& files)
{
    PointCloud::Ptr merged(new PointCloud());
    
    for (const auto& file : files)
    {
        PointCloud::Ptr cloud(new PointCloud());
        if (pcl::io::loadPCDFile<PointType>(file, *cloud) == -1)
        {
            std::cerr << RED << "无法加载文件: " << file << RESET << std::endl;
            continue;
        }
        *merged += *cloud;
        std::cout << "  加载: " << file << " (" << cloud->size() << " 点)\n";
    }
    
    return merged;
}

std::vector<std::string> globFiles(const std::string& pattern)
{
    std::vector<std::string> files;
    std::filesystem::path dir = std::filesystem::path(pattern).parent_path();
    std::string filename_pattern = std::filesystem::path(pattern).filename().string();
    
    if (dir.empty()) dir = ".";
    
    // 简单的通配符匹配
    for (const auto& entry : std::filesystem::directory_iterator(dir))
    {
        if (entry.is_regular_file())
        {
            std::string fname = entry.path().filename().string();
            // 检查是否匹配pattern (简化版本：检查前缀和后缀)
            size_t star_pos = filename_pattern.find('*');
            if (star_pos != std::string::npos)
            {
                std::string prefix = filename_pattern.substr(0, star_pos);
                std::string suffix = filename_pattern.substr(star_pos + 1);
                if (fname.substr(0, prefix.size()) == prefix &&
                    fname.size() >= suffix.size() &&
                    fname.substr(fname.size() - suffix.size()) == suffix)
                {
                    files.push_back(entry.path().string());
                }
            }
        }
    }
    
    std::sort(files.begin(), files.end());
    return files;
}

PointCloud::Ptr downsample(PointCloud::Ptr cloud, float leaf_size)
{
    PointCloud::Ptr filtered(new PointCloud());
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.setInputCloud(cloud);
    voxel.filter(*filtered);
    return filtered;
}

struct MatchResult
{
    bool converged;
    float fitness_score;
    Eigen::Matrix4f transformation;
    double match_time_ms;
    float initial_yaw_deg;  // 使用的初始yaw角
};

// 生成绕Z轴旋转的变换矩阵
Eigen::Matrix4f createYawRotation(float yaw_deg)
{
    float yaw_rad = yaw_deg * M_PI / 180.0f;
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T(0,0) = cos(yaw_rad);  T(0,1) = -sin(yaw_rad);
    T(1,0) = sin(yaw_rad);  T(1,1) = cos(yaw_rad);
    return T;
}

MatchResult runGICPWithInitialGuess(PointCloud::Ptr source, PointCloud::Ptr target, 
                                     const Eigen::Matrix4f& initial_guess,
                                     float max_correspondence_dist = 2.0, int max_iterations = 100)
{
    MatchResult result;
    result.initial_yaw_deg = 0;
    
    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
    gicp.setMaxCorrespondenceDistance(max_correspondence_dist);
    gicp.setMaximumIterations(max_iterations);
    gicp.setTransformationEpsilon(1e-6);
    gicp.setEuclideanFitnessEpsilon(1e-6);
    gicp.setRANSACIterations(0);
    
    gicp.setInputSource(source);
    gicp.setInputTarget(target);
    
    PointCloud::Ptr aligned(new PointCloud());
    
    auto start = std::chrono::high_resolution_clock::now();
    gicp.align(*aligned, initial_guess);
    auto end = std::chrono::high_resolution_clock::now();
    
    result.converged = gicp.hasConverged();
    result.fitness_score = gicp.getFitnessScore();
    result.transformation = gicp.getFinalTransformation();
    result.match_time_ms = std::chrono::duration<double, std::milli>(end - start).count();
    
    return result;
}

MatchResult runGICP(PointCloud::Ptr source, PointCloud::Ptr target, 
                    float max_correspondence_dist = 2.0, int max_iterations = 100)
{
    return runGICPWithInitialGuess(source, target, Eigen::Matrix4f::Identity(), 
                                    max_correspondence_dist, max_iterations);
}

MatchResult runICP(PointCloud::Ptr source, PointCloud::Ptr target,
                   float max_correspondence_dist = 2.0, int max_iterations = 100)
{
    MatchResult result;
    
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(max_correspondence_dist);
    icp.setMaximumIterations(max_iterations);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    
    icp.setInputSource(source);
    icp.setInputTarget(target);
    
    PointCloud::Ptr aligned(new PointCloud());
    
    auto start = std::chrono::high_resolution_clock::now();
    icp.align(*aligned);
    auto end = std::chrono::high_resolution_clock::now();
    
    result.converged = icp.hasConverged();
    result.fitness_score = icp.getFitnessScore();
    result.transformation = icp.getFinalTransformation();
    result.match_time_ms = std::chrono::duration<double, std::milli>(end - start).count();
    
    return result;
}

MatchResult runNDT(PointCloud::Ptr source, PointCloud::Ptr target,
                   float resolution = 1.0, int max_iterations = 50)
{
    MatchResult result;
    
    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    ndt.setResolution(resolution);
    ndt.setMaximumIterations(max_iterations);
    ndt.setStepSize(0.1);
    ndt.setTransformationEpsilon(0.01);
    
    ndt.setInputSource(source);
    ndt.setInputTarget(target);
    
    PointCloud::Ptr aligned(new PointCloud());
    
    auto start = std::chrono::high_resolution_clock::now();
    ndt.align(*aligned);
    auto end = std::chrono::high_resolution_clock::now();
    
    result.converged = ndt.hasConverged();
    result.fitness_score = ndt.getFitnessScore();
    result.transformation = ndt.getFinalTransformation();
    result.match_time_ms = std::chrono::duration<double, std::milli>(end - start).count();
    
    return result;
}

void printTransformation(const Eigen::Matrix4f& T, const std::string& name)
{
    Eigen::Vector3f trans = T.block<3,1>(0,3);
    Eigen::Matrix3f rot = T.block<3,3>(0,0);
    Eigen::Vector3f euler = rot.eulerAngles(2, 1, 0); // ZYX顺序: yaw, pitch, roll
    
    std::cout << name << ":\n";
    std::cout << "  平移: (" << trans.x() << ", " << trans.y() << ", " << trans.z() << ") m\n";
    std::cout << "  平移模长: " << trans.norm() << " m\n";
    std::cout << "  旋转(YPR): (" 
              << euler.x() * 180.0 / M_PI << ", " 
              << euler.y() * 180.0 / M_PI << ", " 
              << euler.z() * 180.0 / M_PI << ") deg\n";
}

// 测试多个yaw角初始猜测，返回最佳结果
MatchResult runMultiYawGICP(PointCloud::Ptr source, PointCloud::Ptr target,
                            const std::vector<float>& yaw_angles_deg,
                            float max_correspondence_dist = 5.0, int max_iterations = 150)
{
    MatchResult best_result;
    best_result.fitness_score = std::numeric_limits<float>::max();
    best_result.converged = false;
    
    std::cout << CYAN << "\n===== 多角度初始猜测测试 =====" << RESET << "\n";
    std::cout << "测试角度: ";
    for (float yaw : yaw_angles_deg) std::cout << yaw << "° ";
    std::cout << "\n\n";
    
    for (float yaw_deg : yaw_angles_deg)
    {
        Eigen::Matrix4f initial_guess = createYawRotation(yaw_deg);
        
        MatchResult result = runGICPWithInitialGuess(source, target, initial_guess,
                                                      max_correspondence_dist, max_iterations);
        result.initial_yaw_deg = yaw_deg;
        
        std::cout << "  Yaw=" << std::setw(4) << yaw_deg << "°: ";
        if (result.converged)
        {
            std::cout << "Score=" << std::fixed << std::setprecision(4) << result.fitness_score;
            if (result.fitness_score < best_result.fitness_score)
            {
                std::cout << GREEN << " (新最佳)" << RESET;
                best_result = result;
            }
        }
        else
        {
            std::cout << RED << "未收敛" << RESET;
        }
        std::cout << " 耗时=" << result.match_time_ms << "ms\n";
    }
    
    std::cout << "\n";
    if (best_result.converged)
    {
        std::cout << GREEN << "最佳初始角度: " << best_result.initial_yaw_deg << "°" << RESET << "\n";
        std::cout << GREEN << "最佳匹配分数: " << best_result.fitness_score << RESET << "\n";
        printTransformation(best_result.transformation, "最佳变换");
    }
    else
    {
        std::cout << RED << "所有角度均未能成功匹配" << RESET << "\n";
    }
    
    return best_result;
}

// 专门测试180°翻转假设
void testYawFlipHypothesis(PointCloud::Ptr source, PointCloud::Ptr target,
                           float score_threshold, bool do_visualize)
{
    std::cout << CYAN << "\n========================================" << RESET << "\n";
    std::cout << CYAN << "  180° 翻转假设测试 (前向LiDAR回环)" << RESET << "\n";
    std::cout << CYAN << "========================================" << RESET << "\n\n";
    
    std::cout << "假设说明:\n";
    std::cout << "  由于LiDAR仅覆盖前方180°视野，当机器人从相反方向\n";
    std::cout << "  回到同一位置时，点云会呈现'背面相对'的情况。\n";
    std::cout << "  此时需要在ICP初始猜测中加入180°的yaw旋转。\n\n";
    
    // 测试0°和180°
    std::cout << YELLOW << "对比测试: 0° vs 180° 初始猜测" << RESET << "\n\n";
    
    MatchResult result_0deg = runGICPWithInitialGuess(source, target, 
                                createYawRotation(0), 10.0, 200);
    result_0deg.initial_yaw_deg = 0;
    
    MatchResult result_180deg = runGICPWithInitialGuess(source, target, 
                                  createYawRotation(180), 10.0, 200);
    result_180deg.initial_yaw_deg = 180;
    
    std::cout << "  初始猜测 0°:\n";
    std::cout << "    收敛: " << (result_0deg.converged ? "是" : "否") << "\n";
    std::cout << "    Score: " << result_0deg.fitness_score << "\n";
    if (result_0deg.converged)
        printTransformation(result_0deg.transformation, "    变换");
    
    std::cout << "\n  初始猜测 180°:\n";
    std::cout << "    收敛: " << (result_180deg.converged ? "是" : "否") << "\n";
    std::cout << "    Score: " << result_180deg.fitness_score << "\n";
    if (result_180deg.converged)
        printTransformation(result_180deg.transformation, "    变换");
    
    // 判断结果
    std::cout << "\n" << CYAN << "===== 分析结论 =====" << RESET << "\n";
    
    bool flip_better = result_180deg.converged && 
                       (!result_0deg.converged || result_180deg.fitness_score < result_0deg.fitness_score);
    
    if (flip_better)
    {
        std::cout << GREEN << "✓ 180°翻转假设成立!" << RESET << "\n";
        std::cout << "  机器人可能从相反方向回到了同一位置。\n";
        std::cout << "  建议: 在回环检测中同时测试0°和180°初始猜测。\n";
        
        float improvement = result_0deg.fitness_score - result_180deg.fitness_score;
        std::cout << "  分数提升: " << improvement << " (" 
                  << (improvement / result_0deg.fitness_score * 100) << "%)\n";
    }
    else if (result_0deg.converged && result_0deg.fitness_score <= score_threshold)
    {
        std::cout << YELLOW << "○ 0°初始猜测已足够" << RESET << "\n";
        std::cout << "  标准ICP匹配成功，无需180°翻转。\n";
    }
    else
    {
        std::cout << RED << "✗ 两种假设均失败" << RESET << "\n";
        std::cout << "  可能原因:\n";
        std::cout << "  - 点云重叠区域不足\n";
        std::cout << "  - 环境变化太大\n";
        std::cout << "  - 需要尝试更多角度\n";
    }
    
    // 可视化最佳结果
    if (do_visualize)
    {
        MatchResult best = flip_better ? result_180deg : result_0deg;
        if (best.converged)
        {
            PointCloud::Ptr aligned(new PointCloud());
            pcl::transformPointCloud(*source, *aligned, best.transformation);
            
            std::string title = "180° Flip Test - Best: " + 
                               std::to_string((int)best.initial_yaw_deg) + "° (Score: " +
                               std::to_string(best.fitness_score) + ")";
            visualize(source, target, aligned, title);
        }
    }
}

void visualize(PointCloud::Ptr source, PointCloud::Ptr target, 
               PointCloud::Ptr aligned, const std::string& title)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(title));
    viewer->setBackgroundColor(0.1, 0.1, 0.1);
    
    // Target点云 - 绿色
    pcl::visualization::PointCloudColorHandlerCustom<PointType> target_color(target, 0, 255, 0);
    viewer->addPointCloud<PointType>(target, target_color, "target");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target");
    
    // Source点云 - 红色 (匹配前)
    pcl::visualization::PointCloudColorHandlerCustom<PointType> source_color(source, 255, 0, 0);
    viewer->addPointCloud<PointType>(source, source_color, "source");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
    
    // Aligned点云 - 蓝色 (匹配后)
    if (aligned && !aligned->empty())
    {
        pcl::visualization::PointCloudColorHandlerCustom<PointType> aligned_color(aligned, 0, 0, 255);
        viewer->addPointCloud<PointType>(aligned, aligned_color, "aligned");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "aligned");
    }
    
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    
    std::cout << "\n" << CYAN << "可视化说明:" << RESET << "\n";
    std::cout << "  " << RED << "红色" << RESET << ": Source点云 (匹配前)\n";
    std::cout << "  " << GREEN << "绿色" << RESET << ": Target点云\n";
    std::cout << "  蓝色: Aligned点云 (匹配后)\n";
    std::cout << "\n按 'q' 退出可视化\n";
    
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
}

int main(int argc, char** argv)
{
    std::cout << CYAN << "\n===== 回环检测ICP匹配验证工具 =====" << RESET << "\n\n";
    
    if (argc < 3)
    {
        printUsage();
        return 1;
    }
    
    // 解析参数
    std::string source_file, target_file;
    std::string method = "gicp";
    float downsample_size = 0.2;
    float score_threshold = 0.5;
    bool do_visualize = false;
    bool merge_source = false, merge_target = false;
    bool test_yaw_flip = false;
    bool test_multi_yaw = false;
    
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--visualize" || arg == "-v")
        {
            do_visualize = true;
        }
        else if (arg == "--merge-source" && i + 1 < argc)
        {
            merge_source = true;
            source_file = argv[++i];
        }
        else if (arg == "--merge-target" && i + 1 < argc)
        {
            merge_target = true;
            target_file = argv[++i];
        }
        else if (arg == "--downsample" && i + 1 < argc)
        {
            downsample_size = std::stof(argv[++i]);
        }
        else if (arg == "--method" && i + 1 < argc)
        {
            method = argv[++i];
        }
        else if (arg == "--score-thresh" && i + 1 < argc)
        {
            score_threshold = std::stof(argv[++i]);
        }
        else if (arg == "--test-yaw-flip")
        {
            test_yaw_flip = true;
        }
        else if (arg == "--test-multi-yaw")
        {
            test_multi_yaw = true;
        }
        else if (source_file.empty())
        {
            source_file = arg;
        }
        else if (target_file.empty())
        {
            target_file = arg;
        }
    }
    
    // 加载点云
    PointCloud::Ptr source_cloud(new PointCloud());
    PointCloud::Ptr target_cloud(new PointCloud());
    
    std::cout << "加载Source点云...\n";
    if (merge_source)
    {
        auto files = globFiles(source_file);
        std::cout << "  找到 " << files.size() << " 个文件\n";
        source_cloud = loadAndMergePointClouds(files);
    }
    else
    {
        if (pcl::io::loadPCDFile<PointType>(source_file, *source_cloud) == -1)
        {
            std::cerr << RED << "无法加载source文件: " << source_file << RESET << std::endl;
            return -1;
        }
        std::cout << "  加载: " << source_file << " (" << source_cloud->size() << " 点)\n";
    }
    
    std::cout << "\n加载Target点云...\n";
    if (merge_target)
    {
        auto files = globFiles(target_file);
        std::cout << "  找到 " << files.size() << " 个文件\n";
        target_cloud = loadAndMergePointClouds(files);
    }
    else
    {
        if (pcl::io::loadPCDFile<PointType>(target_file, *target_cloud) == -1)
        {
            std::cerr << RED << "无法加载target文件: " << target_file << RESET << std::endl;
            return -1;
        }
        std::cout << "  加载: " << target_file << " (" << target_cloud->size() << " 点)\n";
    }
    
    std::cout << "\n" << CYAN << "点云统计:" << RESET << "\n";
    std::cout << "  Source: " << source_cloud->size() << " 点\n";
    std::cout << "  Target: " << target_cloud->size() << " 点\n";
    
    // 下采样
    std::cout << "\n下采样 (体素大小: " << downsample_size << ")...\n";
    PointCloud::Ptr source_ds = downsample(source_cloud, downsample_size);
    PointCloud::Ptr target_ds = downsample(target_cloud, downsample_size);
    std::cout << "  Source下采样后: " << source_ds->size() << " 点\n";
    std::cout << "  Target下采样后: " << target_ds->size() << " 点\n";
    
    // 计算点云统计信息
    Eigen::Vector4f source_centroid, target_centroid;
    pcl::compute3DCentroid(*source_ds, source_centroid);
    pcl::compute3DCentroid(*target_ds, target_centroid);
    
    std::cout << "\n" << CYAN << "点云质心:" << RESET << "\n";
    std::cout << "  Source: (" << source_centroid.x() << ", " << source_centroid.y() << ", " << source_centroid.z() << ")\n";
    std::cout << "  Target: (" << target_centroid.x() << ", " << target_centroid.y() << ", " << target_centroid.z() << ")\n";
    
    Eigen::Vector3f centroid_diff(target_centroid.x() - source_centroid.x(),
                                   target_centroid.y() - source_centroid.y(),
                                   target_centroid.z() - source_centroid.z());
    std::cout << "  质心差异: (" << centroid_diff.x() << ", " << centroid_diff.y() << ", " << centroid_diff.z() << ")\n";
    std::cout << "  质心距离: " << centroid_diff.norm() << " m\n";
    
    // ===== 180°翻转假设测试 =====
    if (test_yaw_flip)
    {
        testYawFlipHypothesis(source_ds, target_ds, score_threshold, do_visualize);
        std::cout << CYAN << "===== 测试完成 =====" << RESET << "\n\n";
        return 0;
    }
    
    // ===== 多角度测试 =====
    if (test_multi_yaw)
    {
        std::vector<float> yaw_angles = {0, 45, 90, 135, 180, 225, 270, 315};
        MatchResult best = runMultiYawGICP(source_ds, target_ds, yaw_angles, 10.0, 200);
        
        if (do_visualize)
        {
            PointCloud::Ptr aligned(new PointCloud());
            std::string title;
            
            if (best.converged)
            {
                pcl::transformPointCloud(*source_ds, *aligned, best.transformation);
                title = "Multi-Yaw Test - Best: " + 
                        std::to_string((int)best.initial_yaw_deg) + "° (Score: " +
                        std::to_string(best.fitness_score) + ")";
            }
            else
            {
                // 即使没有收敛，也显示原始点云用于调试
                *aligned = *source_ds;  // 不变换，显示原始位置
                title = "Multi-Yaw Test - ALL FAILED (showing original)";
            }
            
            visualize(source_ds, target_ds, aligned, title);
        }
        
        std::cout << CYAN << "===== 测试完成 =====" << RESET << "\n\n";
        return 0;
    }
    
    // ===== 标准匹配测试 =====
    std::cout << "\n" << CYAN << "===== 开始匹配测试 (使用multi-yaw方法) =====" << RESET << "\n\n";
    
    MatchResult best_result;
    best_result.fitness_score = std::numeric_limits<float>::max();
    best_result.converged = false;
    
    // 多角度初始猜测 - 与 runMultiYawGICP 使用相同参数
    std::vector<float> yaw_angles = {0, 45, 90, 135, 180, 225, 270, 315};
    
    if (method == "gicp" || method == "all")
    {
        // 直接使用 runMultiYawGICP，它使用更大的对应距离(20.0)和迭代次数(200)
        best_result = runMultiYawGICP(source_ds, target_ds, yaw_angles, 20.0, 200);
    }
    
    if (method == "icp" || method == "all")
    {
        std::cout << YELLOW << "测试ICP方法..." << RESET << "\n";
        MatchResult result = runICP(source_ds, target_ds, 10.0, 200);
        std::cout << "  ICP:  ";
        if (result.converged && result.fitness_score <= score_threshold)
        {
            std::cout << GREEN << "成功!" << RESET;
        }
        else
        {
            std::cout << RED << "失败" << RESET;
        }
        std::cout << " Score=" << result.fitness_score 
                  << " 收敛=" << (result.converged ? "是" : "否")
                  << " 耗时=" << result.match_time_ms << "ms\n";
        
        if (result.converged && result.fitness_score < best_result.fitness_score)
        {
            best_result = result;
        }
    }
    
    if (method == "ndt" || method == "all")
    {
        std::cout << YELLOW << "测试NDT方法..." << RESET << "\n";
        MatchResult result = runNDT(source_ds, target_ds, 1.0, 200);
        std::cout << "  NDT:  ";
        if (result.converged && result.fitness_score <= score_threshold)
        {
            std::cout << GREEN << "成功!" << RESET;
        }
        else
        {
            std::cout << RED << "失败" << RESET;
        }
        std::cout << " Score=" << result.fitness_score 
                  << " 收敛=" << (result.converged ? "是" : "否")
                  << " 耗时=" << result.match_time_ms << "ms\n";
        
        if (result.converged && result.fitness_score < best_result.fitness_score)
        {
            best_result = result;
        }
    }
    
    // 显示最佳结果
    if (best_result.converged)
    {
        std::cout << GREEN << "===== 最佳匹配结果 =====" << RESET << "\n";
        std::cout << "  最佳初始角度: " << best_result.initial_yaw_deg << "°\n";
        std::cout << "  最佳匹配分数: " << best_result.fitness_score << "\n";
        printTransformation(best_result.transformation, "  最佳变换");
        std::cout << "\n";
    }
    else
    {
        std::cout << RED << "所有配置均未能成功匹配" << RESET << "\n\n";
    }
    
    // 可视化
    if (do_visualize)
    {
        PointCloud::Ptr aligned(new PointCloud());
        std::string title;
        
        if (best_result.converged)
        {
            pcl::transformPointCloud(*source_ds, *aligned, best_result.transformation);
            title = "Best Match - Yaw: " + std::to_string((int)best_result.initial_yaw_deg) + 
                    "° (Score: " + std::to_string(best_result.fitness_score) + ")";
        }
        else
        {
            *aligned = *source_ds;
            title = "ALL FAILED (showing original)";
        }
        
        visualize(source_ds, target_ds, aligned, title);
    }
    
    std::cout << CYAN << "===== 测试完成 =====" << RESET << "\n\n";
    
    return 0;
}
