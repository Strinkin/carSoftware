#ifndef SCREEN_HPP
#define SCREEN_HPP

#include <string>
#include <iostream>

class Screen {
public:
    Screen();
    void setSize(double s);
    void setModel(const std::string &m);
    void print(std::ostream &os = std::cout) const;
    void save(std::ostream &ofs) const;

    double size;
    std::string model;
};

#endif // SCREEN_HPP
