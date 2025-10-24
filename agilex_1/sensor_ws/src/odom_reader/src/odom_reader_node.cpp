#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

int main(int argc, char **argv) {
    setlocale(LC_ALL, ""); // 防中文乱码

    ros::init(argc, argv, "odom_reader_node");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);

    ROS_INFO("waiting for odom topic...");
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    system("clear");
    ROS_INFO("线速度: [x=%.5f, y=%.5f, z=%.5f]", msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    ROS_INFO("角速度: [x=%.5f, y=%.5f, z=%.5f]", msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    // ROS_INFO("--- --- --- --- --- ---");
}