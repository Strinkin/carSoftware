#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);

int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "imu_odometry_node");
    ros::NodeHandle nh;

    ros::Subscriber imu_sub = nh.subscribe("/imu/data_raw", 10, imuCallback);

    ROS_INFO("waiting for imu topic...");
    ros::Rate rate(200);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}
/*
（2）编程实现用里程计（odometry）计算小车移动距离
    • 小车靜止不动，读取里程计数据，记为a，控制小车前进n米距离(n=1、2、3均可），读取里程计数据，记为b
    • 建立小车移动距离与里程计读数a、b之间的关系模型（ 前两步应进行多次以拟合更精确的模型）
    • 控制小车移动，利用上一步构建的模型，计算小车移动的距离，并将计算结果与实际值进行对比
*/
void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    static double time = 0;        // 时间戳
    static double v_x = 0;         // 速度
    static double s_x = 0;         // 位移
    static bool is_static = false; // 静止状态标志
    const double static_threshold = 0.03; // 静止检测阈值(m/s^2)

    // 解析数据包
    double pack_acc_x = msg->linear_acceleration.x;
    double pack_time = msg->header.stamp.toSec();
    
    // 首次接收到数据包时初始化
    if (time == 0) {
        time = pack_time;
        return;
    }

    // 计算时间差
    double dt = pack_time - time;
    
    // 零速检测：当加速度绝对值小于阈值时认为静止
    if (fabs(pack_acc_x) < static_threshold) {
        if (!is_static) {
            // 刚进入静止状态，重置速度和位移
            v_x = 0;
            is_static = true;
            ROS_INFO("velocity reset to 0");
        }
    } else {
        is_static = false;
        // 速度积分: v = v0 + a * dt
        double prev_v_x = v_x;
        v_x += fabs(pack_acc_x) * dt;
        
        // 位移积分: s = s0 + (v0 + v1)/2 * dt (梯形积分法)
        s_x += (prev_v_x + v_x) / 2.0 * dt;
    }
    
    // 打印当前速度和位移
    ROS_INFO("dt = %.5f s", dt);
    ROS_INFO("a_x = %.5f m/s^2", pack_acc_x);
    ROS_INFO("v_x = %.5f m/s", v_x);
    ROS_INFO("s_x = %.5f m",s_x);

    ROS_INFO("--- --- --- --- --- --- --- --- --- ---");
    
    time = pack_time; // 更新时间戳
}