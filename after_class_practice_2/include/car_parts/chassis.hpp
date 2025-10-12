#ifndef CHASSIS_HPP
#define CHASSIS_HPP

#include <string>
#include "tire.hpp"
#include <iostream>

class Chassis {
public:
    Chassis();
    // setters
    void setSerialNumber(const std::string &s);
    void setModel(const std::string &m);
    void setWheelBase(double wb);
    void setTrackWidth(double tw);
    void setMinGroundClearance(double mg);
    void setMinTurningRadius(double mr);
    void setDriveType(const std::string &d);
    void setMaxTravelDistance(double md);
    void setTire(const Tire &t);

    // print and save
    void print(std::ostream &os = std::cout) const;
    void save(std::ostream &ofs) const;

    // attributes
    std::string serial_number;
    std::string model;
    double wheel_base;
    double track_width;
    double min_ground_clearance;
    double min_turning_radius;
    std::string drive_type;
    double max_travel_distance;
    Tire tire;
};

#endif // CHASSIS_HPP
