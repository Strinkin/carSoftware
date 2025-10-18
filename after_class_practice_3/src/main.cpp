#include <iostream>
#include <string>
#include <thread>
#include "car_parts/lidar.hpp"
#include "car_parts/chassis.hpp"
#include "topic.hpp"    

using namespace std;

void lidarThread() {
    // 获取话题单例对象
    Topic<std::string>& topic = Topic<std::string>::getInstance(); 
    Lidar lidar;  // 创建Lidar雷达对象                                                    
    while (1) {
        lidar.updateData("front"); // 更新雷达数据
        topic.push(lidar.getData()); // 获取数据并推送到话题
        std::this_thread::sleep_for(std::chrono::seconds(1)); 
        lidar.updateData("front_right"); 
        topic.push(lidar.getData());
        std::this_thread::sleep_for(std::chrono::seconds(1));
        lidar.updateData("front_left");
        topic.push(lidar.getData());
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void chassisThread() {
    // 获取话题单例对象
    Topic<std::string>& topic = Topic<std::string>::getInstance();
    Chassis chassis; // 创建Chassis底盘对象
    while (1) {
        chassis.updateAction(topic.pop()); // 从话题获取数据并更新底盘动作(打印底盘行为)
    }
}

int main() {
    thread lidar_thread(lidarThread); // 启动Lidar雷达线程
    std::this_thread::sleep_for(std::chrono::seconds(1)); // 确保Lidar线程先运行
    thread chassis_thread(chassisThread); // 启动Chassis底盘线程

    lidar_thread.join(); 
    chassis_thread.join();
    return 0;
}