#ifndef __CARINFO_H__
#define __CARINFO_H__

#include <string>
#include "car_parts/chassis.h"
#include "car_parts/agx.h"
#include "car_parts/camera.h"
#include "car_parts/lidar.h"
#include "car_parts/imu.h"
#include "car_parts/screen.h"
#include "car_parts/battery.h"

using namespace std;

// 智能小车信息
struct CarInfo {
    string serial_number;       // 1. 编号: 自定义，cqusn打头的16位数字+字母
    Chassis chassis;            // 2. 底盘
    AGX agx;                    // 3. AGX套件(1个)
    Camera camera;              // 4. 双目摄像头(1个)
    Lidar lidar;                // 5. 多线激光雷达(1个)
    IMU imu;                    // 6. 9轴陀螺仪(1个)
    Screen screen;              // 7. 液晶显示屏(1个)
    Battery battery;            // 8. 电池模块(1个)
};

/*
// 2. 底盘信息
struct Chassis {
    string serial_number;           // 编号: dp打头的8位数字+字母
    string model;                   // 型号: SCOUT MINI
    double wheel_base;              // 轴距: 451mm
    double track_width;             // 轮距: 490mm
    double min_ground_clearance;    // 最小离地间隙: 115mm
    double min_turning_radius;      // 最小转弯半径: 0m
    string drive_type;              // 驱动形式: 四轮四驱
    double max_travel_distance;     // 最大行程:  10km
    Tire tire;                      // 轮胎(4个)
};

// 2.1 轮胎信息
struct Tire {
    TireModel model;    // 轮胎型号: 公路轮、麦克纳姆轮
    double size;        // 尺寸: 172mm
};

// 轮胎型号枚举
enum TireModel {
    ROAD_TIRE,      // 公路轮
    MECANUM_TIRE    // 麦克纳姆轮
};

// 3. AGX套件信息
struct AGX {
    string model;           // 型号: AGX Xavier
    double AI;              // AI算力: 32 TOPS
    double cuda_cores;      // CUDA核心数: 512
    double tensor_cores;    // Tensor核心数: 64
    double memory;          // 内存: 32GB
    double storage;         // 存储: 32GB
};  

// 4. 双目摄像头信息
struct Camera {
    string model;               // 型号: RealSense D435i
    string rgb_camera;          // 摄像头: D430
    Size2D rgb_resolution;      // RGB分辨率: 1920*1080
    double rgb_fps;             // RGB帧率: 30fps
    Size2D fov;                 // 视场角: 87*58度
    double depth_fps;           // 深度帧率: 90fps
};

// 5. 多线激光雷达信息
struct Lidar {
    string model;       // 型号: RS-Helios-16
    int channels;       // 通道数: 16
    double range;       // 测试范围: 100m
    double power;       // 功耗: 8W
};

// 6. 9轴惯导信息
struct IMU {
    string model;           // 型号: CH110
    string manufacturer;    // 厂家: NXP
};

// 7. 液晶屏信息
struct Screen {
    double size;    // 11.6
    string model;   // 型号: super
};

// 8. 电池模块信息
struct Battery {
    double voltage;     // 电压: 24V
    double capacity;    // 容量: 15Ah
    double charge_time; // 充电时长: 2H
};
*/
#endif // __CARINFO_H__