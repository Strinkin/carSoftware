#ifndef LIDAR_HPP
#define LIDAR_HPP

#include <string>
#include <iostream>

class Lidar {
public:
    Lidar();
    void setModel(const std::string &m);
    void setChannels(int c);
    void setRange(double r);
    void setPower(double p);
    void print(std::ostream &os = std::cout) const;
    void save(std::ostream &ofs) const;
    void updateData(const std::string &d);
    std::string getData() const;

    std::string model;
    int channels;
    double range;
    double power;
private:
    std::string data;
};

#endif // LIDAR_HPP
