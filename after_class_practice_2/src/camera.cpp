#include "../include/car_parts/camera.hpp"
#include <ostream>

Camera::Camera(): rgb_fps(0), depth_fps(0) { rgb_resolution.width = rgb_resolution.height = 0; fov.width = fov.height = 0; }
void Camera::setModel(const std::string &m) { model = m; }
void Camera::setRgbCamera(const std::string &r) { rgb_camera = r; }
void Camera::setRgbResolution(int w, int h) { rgb_resolution.width = w; rgb_resolution.height = h; }
void Camera::setRgbFps(double f) { rgb_fps = f; }
void Camera::setFov(int w, int h) { fov.width = w; fov.height = h; }
void Camera::setDepthFps(double f) { depth_fps = f; }
void Camera::print(std::ostream &os) const { os << "Camera: model=" << model << ", rgb_camera=" << rgb_camera << ", resolution=" << rgb_resolution.width << "x" << rgb_resolution.height << ", rgb_fps=" << rgb_fps << ", fov=" << fov.width << "x" << fov.height << ", depth_fps=" << depth_fps << "\n"; }
void Camera::save(std::ostream &ofs) const { ofs << "Camera\n"; ofs << "model:" << model << "\n"; ofs << "rgb_camera:" << rgb_camera << "\n"; ofs << "rgb_resolution:" << rgb_resolution.width << "," << rgb_resolution.height << "\n"; ofs << "rgb_fps:" << rgb_fps << "\n"; ofs << "fov:" << fov.width << "," << fov.height << "\n"; ofs << "depth_fps:" << depth_fps << "\n"; }
