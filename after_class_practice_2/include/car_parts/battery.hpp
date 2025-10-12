#ifndef BATTERY_HPP
#define BATTERY_HPP

#include <iostream>

class Battery {
public:
    Battery();
    void setVoltage(double v);
    void setCapacity(double c);
    void setChargeTime(double t);
    void print(std::ostream &os = std::cout) const;
    void save(std::ostream &ofs) const;

    double voltage;
    double capacity;
    double charge_time;
};

#endif // BATTERY_HPP
