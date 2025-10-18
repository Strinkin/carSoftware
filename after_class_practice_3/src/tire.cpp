#include "../include/car_parts/tire.hpp"
#include <ostream>

Tire::Tire(): model(ROAD_TIRE), size(0) {}
void Tire::setModel(TireModel m) { model = m; }
void Tire::setSize(double s) { size = s; }
void Tire::print(std::ostream &os) const { os << "Tire: model=" << (model==ROAD_TIRE?"ROAD_TIRE":"MECANUM_TIRE") << ", size=" << size << "\n"; }
void Tire::save(std::ostream &ofs) const { ofs << "Tire\n"; ofs << "model:" << (model==ROAD_TIRE?"ROAD_TIRE":"MECANUM_TIRE") << "\n"; ofs << "size:" << size << "\n"; }
