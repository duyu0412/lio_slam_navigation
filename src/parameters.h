// #ifndef PARAM_H
// #define PARAM_H
#pragma once
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <cstring>
#include "preprocess.h"

extern bool is_first_frame;
extern double lidar_end_time, first_lidar_time, time_con;
extern double last_timestamp_lidar, last_timestamp_imu;
extern int pcd_index;

extern std::string lid_topic, imu_topic;
extern bool prop_at_freq_of_imu, check_satu, con_frame, cut_frame;
extern bool use_imu_as_input, space_down_sample;
extern bool extrinsic_est_en, publish_odometry_without_downsample;
extern int  init_map_size, con_frame_num;
extern double match_s, satu_acc, satu_gyro, cut_frame_time_interval;
extern float  plane_thr;
extern double filter_size_surf_min, filter_size_map_min, fov_deg;
extern double cube_len; 
extern float  DET_RANGE;
extern bool   imu_en, gravity_align, non_station_start;
extern double imu_time_inte;
extern double laser_point_cov, acc_norm;
extern double acc_cov_input, gyr_cov_input, vel_cov;
extern double gyr_cov_output, acc_cov_output, b_gyr_cov, b_acc_cov;
extern double imu_meas_acc_cov, imu_meas_omg_cov; 
extern int    lidar_type, pcd_save_interval;
extern std::vector<double> gravity_init, gravity;
extern std::vector<double> extrinT;
extern std::vector<double> extrinR;
extern Eigen::Matrix3d rot_init;
extern bool   runtime_pos_log, pcd_save_en, path_en;
extern bool   scan_pub_en, scan_body_pub_en;
extern shared_ptr<Preprocess> p_pre;
extern double time_lag_imu_to_lidar;

// Working mode
extern bool localization_mode;  // true: localization only, false: mapping mode
extern std::string map_file_path;  // path to pre-built map for localization

// Backend optimization parameters
extern bool loop_closure_enable_flag;  // enable/disable loop closure detection
extern double keyframe_add_dist_threshold;
extern double keyframe_add_angle_threshold;
extern int history_search_num;
extern double history_fitness_score;
extern int loop_closure_search_radius;
extern double loop_closure_time_threshold;
extern int max_retained_keyframes_cloud;

// ISAM2 parameters
extern double isam2_relinearize_threshold;
extern int isam2_relinearize_skip;

// Noise model parameters (rotation: roll, pitch, yaw; translation: x, y, z)
extern double prior_noise_rotation;
extern double prior_noise_translation;
extern double odometry_noise_rotation;
extern double odometry_noise_translation;
extern double loop_noise_rotation;
extern double loop_noise_translation;

// Map save parameters
extern bool map_save_downsample_en;
extern double map_save_resolution;

void readParameters(ros::NodeHandle &n);
