#ifndef IMU_HPP
#define IMU_HPP

#include <string>
#include <iostream>

class IMU {
public:
    IMU();
    void setModel(const std::string &m);
    void setManufacturer(const std::string &mfr);
    void print(std::ostream &os = std::cout) const;
    void save(std::ostream &ofs) const;

    std::string model;
    std::string manufacturer;
};

#endif // IMU_HPP
