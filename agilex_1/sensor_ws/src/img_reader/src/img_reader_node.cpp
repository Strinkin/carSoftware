#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

void colorImageCallback(sensor_msgs::Image::ConstPtr msg);
void depthImageCallback(sensor_msgs::Image::ConstPtr msg);

int main(int argc, char **argv) {
    setlocale(LC_ALL, ""); // 防中文乱码
    cv::namedWindow("color", cv::WINDOW_NORMAL);
    cv::namedWindow("depth", cv::WINDOW_NORMAL);
    cv::moveWindow("color", 0, 0);
    cv::moveWindow("depth", 0, 480);
    ros::init(argc, argv, "img_reader_node");
    ros::NodeHandle nh;

    ros::Subscriber color_sub = nh.subscribe("/camera/color/image_raw", 10, colorImageCallback);
    ros::Subscriber depth_sub = nh.subscribe("/camera/depth/image_rect_raw", 10, depthImageCallback);

    ROS_INFO("wait for camera topic...");
    ros::Rate rate(30); // 30hz
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    cv::destroyAllWindows();
    return 0;
}

void colorImageCallback(sensor_msgs::Image::ConstPtr msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try { // 防止解码失败
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        cv::imshow("color", cv_ptr->image);
        cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void depthImageCallback(sensor_msgs::Image::ConstPtr msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        
        // 将16位深度图像归一化到0-255范围（8位）
        double min_val, max_val;
        cv::minMaxLoc(cv_ptr->image, &min_val, &max_val); // 获取图像的最小值和最大值
        cv::Mat normalized_depth;
        cv_ptr->image.convertTo(normalized_depth, CV_8UC1, 255.0/(max_val - min_val), -min_val * 255.0/(max_val - min_val));
        
        // 应用Jet颜色映射
        cv::Mat color_depth;
        cv::applyColorMap(normalized_depth, color_depth, cv::COLORMAP_TURBO);
        
        cv::imshow("depth", color_depth);
        cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}