#ifndef CARINFO_HPP
#define CARINFO_HPP

#include <string>
#include "car_parts/chassis.hpp"
#include "car_parts/agx.hpp"
#include "car_parts/camera.hpp"
#include "car_parts/lidar.hpp"
#include "car_parts/imu.hpp"
#include "car_parts/screen.hpp"
#include "car_parts/battery.hpp"
#include "studentInfo.hpp"
#include <iostream>

class CarInfo {
public:
    CarInfo();
    void setID(const std::string &id);
    std::string getID() const;
    void print(std::ostream &os = std::cout) const;
    void save(const std::string &path) const; // save to file path

    // keep components public so main can configure them easily
    Chassis chassis;
    AGX agx;
    Camera camera;
    Lidar lidar;
    IMU imu;
    Screen screen;
    Battery battery;

private:
    std::string serial_number;
};

#endif // CARINFO_HPP
