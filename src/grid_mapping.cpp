#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>  
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
using namespace message_filters;

class OctoMapBuilder {
public:
    using PointCloud2 = sensor_msgs::PointCloud2;
    using Odometry = nav_msgs::Odometry;
    using ApproximateSyncPolicy = sync_policies::ApproximateTime<PointCloud2, Odometry>;

    OctoMapBuilder() : nh_("~") {
        // Parameters
        double resolution = 0.1;
        max_range_ = 10.0;
        int prune_every = 50;
        int sor_mean_k = 6;
        double sor_stddev = 1.0;
        int publish_every = 5;
        double min_update_distance = 0.5;
        double ground_dist_thresh = 0.05; // default 5 cm
        int accumulation_frames = 5;
        nh_.param("accumulation_frames", accumulation_frames, accumulation_frames);

        nh_.param("ground_distance_threshold", ground_dist_thresh, ground_dist_thresh);
        nh_.param("sor_mean_k", sor_mean_k, sor_mean_k);
        nh_.param("sor_stddev", sor_stddev, sor_stddev);
        nh_.param("resolution", resolution, resolution);
        nh_.param("max_range", max_range_, max_range_);
        nh_.param("prune_every", prune_every, prune_every);
        nh_.param("publish_every_", publish_every, publish_every);
        nh_.param("min_update_distance_", min_update_distance, min_update_distance);
        nh_.param("z_min", min_z_, -10.0);   // 注意：参数名建议不用下划线结尾
        nh_.param("z_max", max_z_, 10.0);
        tree_ = std::make_shared<octomap::OcTree>(resolution);

        // Optional: set log-odds thresholds
        tree_->setProbHit(0.7);
        tree_->setProbMiss(0.4);
        tree_->setClampingThresMin(0.12);
        tree_->setClampingThresMax(0.97);

        // Subscribers
        pointcloud_sub_.subscribe(nh_, "/pointlio/cloud_registered", 1);
        odom_sub_.subscribe(nh_, "/pointlio/odom", 1);

        sync_ = std::make_shared<Synchronizer<ApproximateSyncPolicy>>(
            ApproximateSyncPolicy(10), pointcloud_sub_, odom_sub_);
        sync_->registerCallback(boost::bind(&OctoMapBuilder::callback, this, _1, _2));

        octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("octomap_full", 1, true);
        accumulation_frames_ = accumulation_frames;
        ground_distance_threshold_ = ground_dist_thresh;
        prune_every_ = prune_every;
        sor_mean_k_ = sor_mean_k;
        sor_stddev_ = sor_stddev;
        min_update_distance_ = min_update_distance;
        publish_every_ = publish_every;
        ROS_INFO("OctoMap builder started (res=%.3fm, max_range=%.1fm)", resolution, max_range_);
    }

    void callback(const PointCloud2::ConstPtr& cloud_msg,
                const Odometry::ConstPtr& odom_msg) {

        latest_sensor_origin_ = octomap::point3d(
            odom_msg->pose.pose.position.x,
            odom_msg->pose.pose.position.y,
            odom_msg->pose.pose.position.z
        );

        // 转换点云（已是全局坐标）
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        try {
            pcl::fromROSMsg(*cloud_msg, *cloud);
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to convert PointCloud2: %s", e.what());
            return;
        }

        if (cloud->empty()) return;

        // 累积点云
        accumulated_clouds_.push_back(cloud);

        // 未达到累积帧数，暂不处理
        if (static_cast<int>(accumulated_clouds_.size()) < accumulation_frames_) {
            return;
        }

        // 合并所有累积的点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& c : accumulated_clouds_) {
            *merged_cloud += *c;
        }

        // 清空缓冲区
        accumulated_clouds_.clear();

        // ===== 后续处理流程 =====

        // 1. SOR 滤波
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        // sor.setInputCloud(merged_cloud);
        // sor.setMeanK(sor_mean_k_);
        // sor.setStddevMulThresh(sor_stddev_);
        // sor.filter(*cloud_sor);

        // if (cloud_sor->empty()) {
        //     ROS_WARN_THROTTLE(1.0, "Merged cloud empty after SOR");
        //     return;
        // }

        // 2. 地面分割
        // pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices);
        // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        // pcl::SACSegmentation<pcl::PointXYZ> seg;
        // seg.setOptimizeCoefficients(true);
        // seg.setModelType(pcl::SACMODEL_PLANE);
        // seg.setMethodType(pcl::SAC_RANSAC);
        // seg.setDistanceThreshold(ground_distance_threshold_);
        // seg.setInputCloud(cloud_sor);
        // seg.segment(*ground_inliers, *coefficients);

        // // 可选：检查法向量是否接近垂直（防墙面误判）
        // bool is_ground_plane = true;
        // if (coefficients->values.size() >= 4) {
        //     float nz = std::abs(coefficients->values[2]);
        //     if (nz < 0.7) {
        //         ROS_DEBUG("Detected plane is not horizontal (nz=%.2f), skipping ground removal", nz);
        //         is_ground_plane = false;
        //     }
        // }

        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZ>);
        // if (is_ground_plane && !ground_inliers->indices.empty()) {
        //     pcl::ExtractIndices<pcl::PointXYZ> extract;
        //     extract.setInputCloud(cloud_sor);
        //     extract.setIndices(ground_inliers);
        //     extract.setNegative(true);
        //     extract.filter(*cloud_no_ground);
        // } else {
        //     // 无法可靠分割地面，保留全部点（或可选跳过）
        //     cloud_no_ground = cloud_sor;
        // }

        // 3. Z 范围过滤（兜底）
        octomap::Pointcloud octo_cloud;
        for (const auto& pt : merged_cloud->points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
                continue;
            if (pt.z > max_z_ || pt.z < min_z_)
                continue;
            octo_cloud.push_back(pt.x, pt.y, pt.z);
        }

        if (octo_cloud.size() == 0) {
            ROS_WARN_THROTTLE(1.0, "No valid points after filtering");
            return;
        }

        // 4. 插入 OctoMap（使用最新一帧的 sensor origin）
        tree_->insertPointCloud(octo_cloud, latest_sensor_origin_, max_range_);

        frame_count_++;
        if (frame_count_ % prune_every_ == 0) {
            tree_->prune();
        }
        if (frame_count_ % publish_every_ == 0) {
            publishOctoMap(cloud_msg->header.stamp, cloud_msg->header.frame_id);
        }
    }
    void publishOctoMap(const ros::Time& stamp, const std::string& frame_id) {
        octomap_msgs::Octomap msg;
        if (octomap_msgs::fullMapToMsg(*tree_, msg)) {
            msg.header.frame_id = frame_id; // Use consistent frame
            msg.header.stamp = stamp;
            octomap_pub_.publish(msg);
        }
    }

    ~OctoMapBuilder() {
        std::string filename = "/home/user/SLAM/src/point_lio_unilidar/Octomap/pointlio_octomap.bt";
        tree_->updateInnerOccupancy();
        if (tree_->writeBinary(filename)) {
            ROS_INFO("OctoMap saved to %s", filename.c_str());
        } else {
            ROS_ERROR("Failed to save OctoMap!");
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher octomap_pub_;

    message_filters::Subscriber<PointCloud2> pointcloud_sub_;
    message_filters::Subscriber<Odometry> odom_sub_;
    std::shared_ptr<Synchronizer<ApproximateSyncPolicy>> sync_;

    std::shared_ptr<octomap::OcTree> tree_;
    geometry_msgs::Point last_position_;
    double min_update_distance_; // 米
    double max_range_;
    int prune_every_;
    int frame_count_ = 0;
    int sor_mean_k_;
    double sor_stddev_;
    int publish_every_;
    double min_z_,max_z_;
    double ground_distance_threshold_; // e.g., 0.05 meters
    std::string cloud_frame_id_ = "odom"; // You can also get from cloud_msg->header.frame_id
    // Accumulation
    int accumulation_frames_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> accumulated_clouds_;
    octomap::point3d latest_sensor_origin_; // 用于 insertPointCloud
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointlio_octomap_builder");
    OctoMapBuilder builder;
    ros::spin();
    return 0;
}