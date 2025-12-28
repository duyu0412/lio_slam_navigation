// ...existing code...
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <fstream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "bt_to_projected_map");
    ros::NodeHandle nh;

    // 参数：.bt 文件路径
    ros::NodeHandle private_nh("~"); // 私有句柄
    std::string bt_file;
    if (!private_nh.getParam("bt_file", bt_file)) {
        ROS_ERROR("Please set ~bt_file parameter!");
        return -1;
    }

    double resolution = 0.2;
    private_nh.param("resolution", resolution, resolution);

    double min_z = -100.0, max_z = 100.0;
    private_nh.param("min_z", min_z, min_z);
    private_nh.param("max_z", max_z, max_z);

    std::string frame_id = "map";
    private_nh.param("frame_id", frame_id, frame_id);

    // 加载 .bt 文件
    std::unique_ptr<octomap::OcTree> tree(new octomap::OcTree(resolution));
    if (!tree->readBinary(bt_file)) {
        ROS_ERROR("Failed to read .bt file: %s", bt_file.c_str());
        return -1;
    }

    ROS_INFO("Loaded OctoMap with resolution=%.3f, size=%zu nodes", 
             tree->getResolution(), tree->size());

    // 统计 occupied
    size_t occupied_count = 0;
    double min_x = 1e9, min_y = 1e9, max_x = -1e9, max_y = -1e9;

    for (octomap::OcTree::iterator it = tree->begin(), end = tree->end(); it != end; ++it) {
        if (tree->isNodeOccupied(*it)) {
            occupied_count++;
            double x = it.getX(), y = it.getY(), z = it.getZ();
            if (z >= min_z && z <= max_z) {
                min_x = std::min(min_x, x);
                min_y = std::min(min_y, y);
                max_x = std::max(max_x, x);
                max_y = std::max(max_y, y);
            }
        }
    }

    ROS_INFO("Occupied nodes: %zu (within z=[%.1f, %.1f]: used for projection)", 
             occupied_count, min_z, max_z);

    if (occupied_count == 0) {
        ROS_WARN("No occupied nodes! Map will be empty.");
    }

    // 创建 OccupancyGrid
    nav_msgs::OccupancyGrid grid;
    grid.header.frame_id = frame_id;
    grid.header.stamp = ros::Time::now();

    // 计算地图尺寸（加一点边界）
    double margin = 0.5;
    min_x -= margin; min_y -= margin;
    max_x += margin; max_y += margin;

    int width = std::ceil((max_x - min_x) / resolution);
    int height = std::ceil((max_y - min_y) / resolution);

    grid.info.resolution = resolution;
    grid.info.width = width;
    grid.info.height = height;
    grid.info.origin.position.x = min_x;
    grid.info.origin.position.y = min_y;
    grid.info.origin.orientation.w = 1.0;

    // 初始化为 unknown (-1)
    grid.data.assign(width * height, -1);

    // 投影：对每个 (x,y) 列，检查是否有 occupied 且 z in [min_z, max_z]
    for (int i = 0; i < width; ++i) {
        for (int j = 0; j < height; ++j) {
            double x = min_x + (i + 0.5) * resolution;
            double y = min_y + (j + 0.5) * resolution;

            bool has_occupied = false;
            // 查询该 (x,y) 列在 [min_z, max_z] 内是否有 occupied
            for (double z = min_z; z <= max_z; z += resolution) {
                octomap::point3d query(x, y, z);
                octomap::OcTreeNode* node = tree->search(query);
                if (node && tree->isNodeOccupied(node)) {
                    has_occupied = true;
                    break;
                }
            }

            int idx = j * width + i;
            grid.data[idx] = has_occupied ? 100 : 0; // 100=occupied, 0=free
        }
    }

    // 发布一次（latched）
    ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("/projected_map", 1, true);
    ros::Publisher octomap_pub = nh.advertise<octomap_msgs::Octomap>("/octomap", 1, true);

    pub.publish(grid);

    // 发布 octomap
    octomap_msgs::Octomap octomap_msg;
    octomap_msg.header.frame_id = frame_id;
    octomap_msg.header.stamp = ros::Time::now();
    if (octomap_msgs::fullMapToMsg(*tree, octomap_msg)) {
        octomap_pub.publish(octomap_msg);
        ROS_INFO("Published /octomap topic.");
    } else {
        ROS_WARN("Failed to convert octomap to message!");
    }

    ROS_INFO("Published /projected_map with %d x %d cells (origin: %.2f, %.2f)",
             width, height, min_x, min_y);

    // 保存2D地图为PGM和YAML
    std::string map_name = "/home/user/map";
    std::ofstream pgm(map_name + ".pgm");
    pgm << "P2\n" << width << " " << height << "\n255\n";
    for (int j = 0; j < height; ++j) {
        for (int i = 0; i < width; ++i) {
            int idx = j * width + i;
            int val = grid.data[idx];
            int gray = 205; // unknown
            if (val == 100) gray = 0; // occupied
            else if (val == 0) gray = 254; // free
            pgm << gray << " ";
        }
        pgm << "\n";
    }
    pgm.close();

    std::ofstream yaml(map_name + ".yaml");
    yaml << "image: " << map_name << ".pgm\n";
    yaml << "resolution: " << resolution << "\n";
    yaml << "origin: [" << min_x << ", " << min_y << ", 0.0]\n";
    yaml << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
    yaml.close();
    ROS_INFO("Saved 2D map to %s.pgm and %s.yaml", map_name.c_str(), map_name.c_str());

    // 保持运行以便 RViz 订阅
    ros::spin();

    return 0;
}