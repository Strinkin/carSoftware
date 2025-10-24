#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h> // pcl <-> ros
#include <pcl/visualization/pcl_visualizer.h>

void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

int main(int argc, char **argv) {
    setlocale(LC_ALL, ""); // 设置locale，防止输出中文乱码

    ros::init(argc, argv, "pointcloud_reader_node");
    ros::NodeHandle nh;

    ros::Subscriber pointcloud_sub = nh.subscribe("/rslidar_points", 50, pointcloudCallback);

    ROS_INFO("waiting for pointcloud data...");
    ros::Rate loop_rate(20);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

// 读取点云数据并可视化
void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // 读取点云数据保存为pcl格式
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    // 可视化点云数据
    visualizePointCloud(pcl_cloud);
}

void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    // 1. 创建可视化对象
    static pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PointCloud Viewer"));
    viewer->removeAllPointClouds();
    // 2. 设置背景颜色
    viewer->setBackgroundColor(0, 0, 0); // 黑色背景
    
    // 3. 添加点云到可视化窗口
    // 使用强度值（intensity）作为颜色映射
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_color(cloud, "intensity");
    viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_color, "cloud");
    
    // 4. 设置点云大小
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    
    // 5. 添加坐标系
    viewer->addCoordinateSystem(5.0); // 坐标系大小(米)
    viewer->setPosition(960, 0); // 坐标系位置(像素)
    viewer->spinOnce(10); // 非阻塞刷新
}