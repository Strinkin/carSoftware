#include "../include/carInfo.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>

CarInfo::CarInfo(): serial_number("") {}
void CarInfo::setID(const std::string &id) { serial_number = id; }
std::string CarInfo::getID() const { return serial_number; }

void CarInfo::print(std::ostream &os) const {
    os << "CarInfo\n";
    os << "serial_number:" << serial_number << "\n";
    chassis.print(os);
    agx.print(os);
    camera.print(os);
    lidar.print(os);
    imu.print(os);
    screen.print(os);
    battery.print(os);
}

void CarInfo::save(const std::string &path) const {
    std::ofstream ofs(path);
    if (!ofs) return;
    ofs << "CarInfo\n";
    ofs << "serial_number:" << serial_number << "\n";
    chassis.save(ofs);
    agx.save(ofs);
    camera.save(ofs);
    lidar.save(ofs);
    imu.save(ofs);
    screen.save(ofs);
    battery.save(ofs);
    ofs << "\n";
    ofs.close();
}