#ifndef TIRE_HPP
#define TIRE_HPP

#include <iostream>

enum TireModel { ROAD_TIRE, MECANUM_TIRE };

class Tire {
public:
    Tire();
    void setModel(TireModel m);
    void setSize(double s);
    void print(std::ostream &os = std::cout) const;
    void save(std::ostream &ofs) const;

    TireModel model;
    double size;
};

#endif // TIRE_HPP
