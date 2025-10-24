#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

void colorImageCallback(sensor_msgs::Image::ConstPtr msg);

int main(int argc, char **argv) {
    setlocale(LC_ALL, ""); // 防中文乱码
    cv::namedWindow("color", cv::WINDOW_NORMAL);
    cv::resizeWindow("color", 640, 480);
    cv::moveWindow("color", 0, 0);
    ros::init(argc, argv, "img_reader_node");
    ros::NodeHandle nh;

    ros::Subscriber color_sub = nh.subscribe("/camera/color/image_raw", 10, colorImageCallback);

    ROS_INFO("waiting for camera topic...");
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

