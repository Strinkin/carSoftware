// camera.h
#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <string>
#include "common.h"  // 包含 Size2D 定义

struct Camera {
    std::string model;               // 型号: RealSense D435i
    std::string rgb_camera;          // 摄像头: D430
    Size2D rgb_resolution;           // RGB分辨率: 1920 * 1080
    double rgb_fps;                  // RGB帧率: 30fps
    Size2D fov;                      // 视场角: 87 * 58度
    double depth_fps;                // 深度帧率: 90fps
};

#endif // __CAMERA_H__