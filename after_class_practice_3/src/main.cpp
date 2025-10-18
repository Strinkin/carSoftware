#include <iostream>
#include <string>
#include <thread>
#include "car_parts/lidar.hpp"
#include "car_parts/chassis.hpp"
#include "topic.hpp"    

using namespace std;

void lidarThread() {
    Topic<std::string>& topic = Topic<std::string>::getInstance();
    topic.push("LIDAR data");
}

int main() {
    thread lidar_thread(lidarThread);
    lidar_thread.join();
    Topic<std::string>& topic = Topic<std::string>::getInstance();
    string data = topic.pop();
    cout << "Received from LIDAR thread: " << data << endl;
    return 0;
}