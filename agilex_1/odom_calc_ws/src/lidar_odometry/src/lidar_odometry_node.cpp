#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

void lidarOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);

int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "lidar_odometry_node");
    ros::NodeHandle nh;
    ros::Subscriber lidar_odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 10, lidarOdomCallback);

    ROS_INFO("waiting for lidar odometry topic...");
    ros::Rate loop_rate(10); // 10 Hz
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void lidarOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    static double last_time = 0;
    static double accumulated_distance_trap = 0;  // 梯形法累积距离
    // static double accumulated_distance_rect = 0;  // 矩形法累积距离
    static double previous_velocity = 0;

    double current_time = msg->header.stamp.toSec();
    double current_velocity = msg->twist.twist.linear.x;
    if (current_velocity < 0) {
        current_velocity = 0;
    }
    // 初始化检查
    if (last_time == 0) {
        last_time = current_time;
        previous_velocity = current_velocity;
        return;
    }

    // 计算时间差
    double delta_time = current_time - last_time;

    // 有效时间差检查 (0 < dt <= 0.5秒)
    if (delta_time <= 0 || delta_time > 0.5) {
        ROS_WARN("无效时间差: %.6f 秒，跳过积分", delta_time);
        last_time = current_time;
        previous_velocity = current_velocity;
        return;
    }

    // 矩形法积分（普通积分）
    // double rect_distance = current_velocity * delta_time;
    // accumulated_distance_rect += rect_distance;

    // 梯形法积分
    double average_velocity = (previous_velocity + current_velocity) / 2.0;
    double trap_distance = average_velocity * delta_time;
    accumulated_distance_trap += trap_distance;

    // 更新状态
    previous_velocity = current_velocity;
    last_time = current_time;

    // 输出信息
    ROS_INFO("时间差(dt): %.6f 秒", delta_time);
    ROS_INFO("前进速度为: %.2f 米/秒.", msg->twist.twist.linear.x);
    // ROS_INFO("矩形法累积距离: %.6f 米", accumulated_distance_rect);
    ROS_INFO("梯形法累积距离: %.6f 米", accumulated_distance_trap);
    // ROS_INFO("前进距离为: %.2f 米.", msg->pose.pose.position.x);
    ROS_INFO("--- --- --- --- --- --- --- --- --- ---");
}

