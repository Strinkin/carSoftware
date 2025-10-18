#include <iostream>
#include <string>
#include <thread>
#include "car_parts/lidar.hpp"
#include "car_parts/chassis.hpp"
#include "topic.hpp"    

using namespace std;

void lidarThread() {
    Topic<std::string>& topic = Topic<std::string>::getInstance();
    Lidar lidar;
    while (1) {
        lidar.updateData("front");
        topic.push(lidar.getData());
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
    Topic<std::string>& topic = Topic<std::string>::getInstance();
    Chassis chassis;
    while (1) {
        chassis.updateAction(topic.pop());
    }
}

int main() {
    thread lidar_thread(lidarThread);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    thread chassis_thread(chassisThread);
    

    lidar_thread.join();
    chassis_thread.join();
    return 0;
}