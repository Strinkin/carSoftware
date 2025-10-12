#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <string>
#include "common.h"
#include <iostream>

class Camera {
public:
    Camera();
    void setModel(const std::string &m);
    void setRgbCamera(const std::string &r);
    void setRgbResolution(int w, int h);
    void setRgbFps(double f);
    void setFov(int w, int h);
    void setDepthFps(double f);

    void print(std::ostream &os = std::cout) const;
    void save(std::ostream &ofs) const;

    std::string model;
    std::string rgb_camera;
    Size2D rgb_resolution;
    double rgb_fps;
    Size2D fov;
    double depth_fps;
};

#endif // CAMERA_HPP
