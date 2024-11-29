#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
// #include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <iostream>

// 设置

int main(int argc, char** argv) {
    std::string bag_file = "/home/john/rosbag/locally/03small.bag";  // 输入 rosbag 文件
    std::string topic_name = "/livox/lidar";   // 点云话题
    std::string output_dir = "/home/john/rosbag/locally/bag_to_pcd";  // 输出目录

    // 创建输出目录
    if (!boost::filesystem::exists(output_dir)) {
        boost::filesystem::create_directory(output_dir);
    }

    // 打开 rosbag
    rosbag::Bag bag;
    bag.open(bag_file, rosbag::bagmode::Read);

    // 过滤话题
    rosbag::View view(bag, rosbag::TopicQuery(topic_name));
    int seq = 0;

    // 遍历消息
    for (const rosbag::MessageInstance& msg : view) {
        sensor_msgs::PointCloud2::ConstPtr cloud_msg = msg.instantiate<sensor_msgs::PointCloud2>();
        if (cloud_msg) {
            // 转换为 PCL 点云格式
            pcl::PointCloud<pcl::PointXYZ> cloud;
            // 中间转换部分
            pcl::PCLPointCloud2 pcl_cloud2;
            // 1.74版本没有fromROSMsg函数，用pcl::fromPCLPointCloud2代替
            // pcl::fromROSMsg(*cloud_msg, cloud);
            pcl_conversions::toPCL(*cloud_msg, pcl_cloud2);
            pcl::fromPCLPointCloud2(pcl_cloud2, cloud);

            // 自定义文件名
            std::stringstream ss;
            // ss << output_dir << "/cloud_" << seq << "_" << cloud_msg->header.stamp.toSec() << ".pcd";
            ss << output_dir << "/cloud_" << seq << ".pcd";

            // 保存点云到 PCD 文件
            // pcl::io::savePCDFile(ss.str(), *cloud);
            pcl::io::savePCDFile(ss.str(), cloud);
            std::cout << "Saved: " << ss.str() << std::endl;
            seq++;
        }
    }

    bag.close();
    return 0;
}