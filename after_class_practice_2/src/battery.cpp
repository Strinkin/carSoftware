#include "../include/car_parts/battery.hpp"
#include <ostream>

Battery::Battery(): voltage(0), capacity(0), charge_time(0) {}
void Battery::setVoltage(double v) { voltage = v; }
void Battery::setCapacity(double c) { capacity = c; }
void Battery::setChargeTime(double t) { charge_time = t; }
void Battery::print(std::ostream &os) const { os << "Battery: voltage=" << voltage << "V, capacity=" << capacity << "Ah, charge_time=" << charge_time << "h\n"; }
void Battery::save(std::ostream &ofs) const { ofs << "Battery\n"; ofs << "voltage:" << voltage << "\n"; ofs << "capacity:" << capacity << "\n"; ofs << "charge_time:" << charge_time << "\n"; }
