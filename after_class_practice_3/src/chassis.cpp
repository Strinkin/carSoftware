#include "../include/car_parts/chassis.hpp"
#include <ostream>

Chassis::Chassis()
    : wheel_base(0), track_width(0), min_ground_clearance(0), min_turning_radius(0), max_travel_distance(0) {}

void Chassis::setSerialNumber(const std::string &s) { serial_number = s; }
void Chassis::setModel(const std::string &m) { model = m; }
void Chassis::setWheelBase(double wb) { wheel_base = wb; }
void Chassis::setTrackWidth(double tw) { track_width = tw; }
void Chassis::setMinGroundClearance(double mg) { min_ground_clearance = mg; }
void Chassis::setMinTurningRadius(double mr) { min_turning_radius = mr; }
void Chassis::setDriveType(const std::string &d) { drive_type = d; }
void Chassis::setMaxTravelDistance(double md) { max_travel_distance = md; }
void Chassis::setTire(const Tire &t) { tire = t; }

void Chassis::print(std::ostream &os) const {
    os << "Chassis:\n";
    os << "  serial_number: " << serial_number << "\n";
    os << "  model: " << model << "\n";
    os << "  wheel_base: " << wheel_base << "\n";
    os << "  track_width: " << track_width << "\n";
    os << "  min_ground_clearance: " << min_ground_clearance << "\n";
    os << "  min_turning_radius: " << min_turning_radius << "\n";
    os << "  drive_type: " << drive_type << "\n";
    os << "  max_travel_distance: " << max_travel_distance << "\n";
    os << "  tire.model: " << (tire.model==ROAD_TIRE?"ROAD_TIRE":"MECANUM_TIRE") << "\n";
    os << "  tire.size: " << tire.size << "\n";
}

void Chassis::save(std::ostream &ofs) const {
    ofs << "Chassis\n";
    ofs << "serial_number:" << serial_number << "\n";
    ofs << "model:" << model << "\n";
    ofs << "wheel_base:" << wheel_base << "\n";
    ofs << "track_width:" << track_width << "\n";
    ofs << "min_ground_clearance:" << min_ground_clearance << "\n";
    ofs << "min_turning_radius:" << min_turning_radius << "\n";
    ofs << "drive_type:" << drive_type << "\n";
    ofs << "max_travel_distance:" << max_travel_distance << "\n";
    ofs << "tire_model:" << (tire.model==ROAD_TIRE?"ROAD_TIRE":"MECANUM_TIRE") << "\n";
    ofs << "tire_size:" << tire.size << "\n";
}

void Chassis::updateAction(std::string lidar_data) {
// （1）	障碍状态为“前方“，则底盘执行”后退“执行
// （2）	障碍状态为“右前方“，则底盘执行”左转“执行
// （3）	障碍状态为“左前方“，则底盘执行”右转“执行
    if (lidar_data == "front") {
        std::cout << "Chassis action: move backward" << std::endl;
    } 
    else if (lidar_data == "front_right") {
        std::cout << "Chassis action: turn left" << std::endl;
    } 
    else if (lidar_data == "front_left") {
        std::cout << "Chassis action: turn right" << std::endl;
    }
}