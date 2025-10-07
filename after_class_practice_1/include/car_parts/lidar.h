// lidar.h
#ifndef __LIDAR_H__
#define __LIDAR_H__

#include <string>

struct Lidar {
    std::string model;       // 型号: RS-Helios-16
    int channels;            // 通道数: 16
    double range;            // 测试范围: 100m
    double power;            // 功耗: 8W
};

#endif // __LIDAR_H__