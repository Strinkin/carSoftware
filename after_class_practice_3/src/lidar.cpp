#include "../include/car_parts/lidar.hpp"
#include <ostream>

Lidar::Lidar(): channels(0), range(0), power(0) {}
void Lidar::setModel(const std::string &m) { model = m; }
void Lidar::setChannels(int c) { channels = c; }
void Lidar::setRange(double r) { range = r; }
void Lidar::setPower(double p) { power = p; }
void Lidar::print(std::ostream &os) const { os << "Lidar: model=" << model << ", channels=" << channels << ", range=" << range << "m, power=" << power << "W\n"; }
void Lidar::save(std::ostream &ofs) const { ofs << "Lidar\n"; ofs << "model:" << model << "\n"; ofs << "channels:" << channels << "\n"; ofs << "range:" << range << "\n"; ofs << "power:" << power << "\n"; }

void Lidar::updateData(const std::string &d) { data = d; }
std::string Lidar::getData() const { return data; }