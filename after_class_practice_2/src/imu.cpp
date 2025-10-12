#include "../include/car_parts/imu.hpp"
#include <ostream>

IMU::IMU(): model(""), manufacturer("") {}
void IMU::setModel(const std::string &m) { model = m; }
void IMU::setManufacturer(const std::string &mfr) { manufacturer = mfr; }
void IMU::print(std::ostream &os) const { os << "IMU: model=" << model << ", manufacturer=" << manufacturer << "\n"; }
void IMU::save(std::ostream &ofs) const { ofs << "IMU\n"; ofs << "model:" << model << "\n"; ofs << "manufacturer:" << manufacturer << "\n"; }
