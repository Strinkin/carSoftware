#include "../include/car_parts/screen.hpp"
#include <ostream>

Screen::Screen(): size(0) {}
void Screen::setSize(double s) { size = s; }
void Screen::setModel(const std::string &m) { model = m; }
void Screen::print(std::ostream &os) const { os << "Screen: model=" << model << ", size=" << size << "\n"; }
void Screen::save(std::ostream &ofs) const { ofs << "Screen\n"; ofs << "model:" << model << "\n"; ofs << "size:" << size << "\n"; }
