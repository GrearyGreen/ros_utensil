#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/filesystem.hpp>
#include <vector>
#include <algorithm>
#include <boost/regex.hpp>
namespace fs = boost::filesystem;

// 函数：获取指定文件夹中的所有 PCD 文件路径，按文件名排序
// std::vector<std::string> getPCDFiles(const std::string& folder_path) {
//     std::vector<std::string> pcd_files;

//     if (!fs::exists(folder_path) || !fs::is_directory(folder_path)) {
//         ROS_ERROR("Invalid folder path: %s", folder_path.c_str());
//         return pcd_files;
//     }

//     for (const auto& entry : fs::directory_iterator(folder_path)) {
//         if (fs::is_regular_file(entry) && entry.path().extension() == ".pcd") {
//             pcd_files.push_back(entry.path().string());
//         }
//     }

//     // 按文件名排序
//     std::sort(pcd_files.begin(), pcd_files.end());
//     return pcd_files;
// }

// namespace fs = std::filesystem;

// 提取文件名中的数字部分
// int extractNumber(const std::string& str) {
//     int number = 0;
//     bool number_started = false;

//     for (char c : str) {
//         if (std::isdigit(c)) {
//             number = number * 10 + (c - '0');
//             number_started = true;
//         } else if (number_started) {
//             // 当数字部分结束后就停止
//             break;
//         }
//     }

//     return number;
// }

// 自定义比较函数：按文件名中的数字排序
// bool comparePCDFiles(const std::string& a, const std::string& b) {
//     int num_a = extractNumber(a);
//     int num_b = extractNumber(b);

//     if (num_a != num_b) {
//         return num_a < num_b;
//     }
//     return a < b;  // 如果数字相同，按字典顺序排序
// }

// 自定义比较函数：按文件名中的数字排序
bool comparePCDFiles(const std::string& a, const std::string& b) {
    // std::regex re(R"(\d+)");  // 匹配文件名中的数字部分
    boost::regex re(R"(\d+)");  // 匹配文件名中的数字部分
    // std::smatch match_a, match_b;
    boost::smatch match_a, match_b;

    // 提取第一个文件名中的数字
    // std::regex_search(a, match_a, re);
    boost::regex_search(a, match_a, re);
    // std::regex_search(b, match_b, re);
    boost::regex_search(b, match_b, re);

    int num_a = (match_a.empty()) ? 0 : std::stoi(match_a.str());
    int num_b = (match_b.empty()) ? 0 : std::stoi(match_b.str());

    // 比较数字大小
    if (num_a != num_b) {
        return num_a < num_b;
    }
    return a < b; // 如果数字相同，则按字典顺序排序
}

// 函数：获取指定文件夹中的所有 PCD 文件路径，按文件名中的数字排序
std::vector<std::string> getPCDFiles(const std::string& folder_path) {
    std::vector<std::string> pcd_files;

    if (!fs::exists(folder_path) || !fs::is_directory(folder_path)) {
        std::cerr << "Invalid folder path: " << folder_path << std::endl;
        return pcd_files;
    }

    // 获取所有 .pcd 文件
    for (const auto& entry : fs::directory_iterator(folder_path)) {
        if (fs::is_regular_file(entry) && entry.path().extension() == ".pcd") {
            pcd_files.push_back(entry.path().string());
        }
    }

    // 按文件名中的数字排序
    std::sort(pcd_files.begin(), pcd_files.end(), comparePCDFiles);
    return pcd_files;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcd_to_rosbag");
    ros::NodeHandle nh;  // 确保初始化了 NodeHandle

    ros::Time time_now = ros::Time::now();
    ROS_INFO("Current time: %f", time_now.toSec());

    // 指定文件夹路径和输出 bag 文件路径
    std::string folder_path = "/home/john/rosbag/locally/pcd_to_bag_less";  // 替换为实际 PCD 文件夹路径
    std::string bag_file = "/home/john/rosbag/locally/03small_less_change.bag";             // 输出 bag 文件名

    // 获取 PCD 文件列表
    std::vector<std::string> pcd_files = getPCDFiles(folder_path);
    if (pcd_files.empty()) {
        ROS_ERROR("No PCD files found in folder: %s", folder_path.c_str());
        return -1;
    }

    // 创建 ROS bag 文件
    rosbag::Bag bag;
    bag.open(bag_file, rosbag::bagmode::Write);

    for (size_t i = 0; i < pcd_files.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        if (pcl::io::loadPCDFile(pcd_files[i], cloud) == -1) {
            ROS_ERROR("Failed to load PCD file: %s", pcd_files[i].c_str());
            continue;
        }

        // 转换为 sensor_msgs::PointCloud2
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        cloud_msg.header.frame_id = "map";  // 替换为适当的 frame_id
        cloud_msg.header.stamp = ros::Time::now();

        // 写入 ROS bag 文件
        // bag.write("point_cloud_topic", cloud_msg.header.stamp, cloud_msg);
        bag.write("/lidar_points", cloud_msg.header.stamp, cloud_msg);
        ROS_INFO("Added %s to rosbag", pcd_files[i].c_str());
    }

    bag.close();
    ROS_INFO("ROS bag file saved: %s", bag_file.c_str());

    return 0;
}
