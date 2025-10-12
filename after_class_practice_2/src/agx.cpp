#include "../include/car_parts/agx.hpp"
#include <ostream>

AGX::AGX(): AI(0), cuda_cores(0), tensor_cores(0), memory(0), storage(0) {}
void AGX::setModel(const std::string &m) { model = m; }
void AGX::setAI(double a) { AI = a; }
void AGX::setCudaCores(double c) { cuda_cores = c; }
void AGX::setTensorCores(double t) { tensor_cores = t; }
void AGX::setMemory(double m) { memory = m; }
void AGX::setStorage(double s) { storage = s; }
void AGX::print(std::ostream &os) const { os << "AGX: model=" << model << ", AI=" << AI << " TOPS, cuda=" << cuda_cores << ", tensor=" << tensor_cores << ", memory=" << memory << "GB, storage=" << storage << "GB\n"; }
void AGX::save(std::ostream &ofs) const { ofs << "AGX\n"; ofs << "model:" << model << "\n"; ofs << "AI:" << AI << "\n"; ofs << "cuda_cores:" << cuda_cores << "\n"; ofs << "tensor_cores:" << tensor_cores << "\n"; ofs << "memory:" << memory << "\n"; ofs << "storage:" << storage << "\n"; }
