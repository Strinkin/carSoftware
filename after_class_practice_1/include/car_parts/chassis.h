// chassis.h
#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include <string>
#include "tire.h"  // 包含轮胎定义

struct Chassis {
    std::string serial_number;           // 编号: dp打头的8位数字+字母
    std::string model;                   // 型号: SCOUT MINI
    double wheel_base;                   // 轴距: 451mm
    double track_width;                  // 轮距: 490mm
    double min_ground_clearance;         // 最小离地间隙: 115mm
    double min_turning_radius;           // 最小转弯半径: 0m
    std::string drive_type;              // 驱动形式: 四轮四驱
    double max_travel_distance;          // 最大行程:  10km
    Tire tire;                           // 轮胎(4个)
};

#endif // __CHASSIS_H__