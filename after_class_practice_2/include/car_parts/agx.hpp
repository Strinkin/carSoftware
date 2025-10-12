#ifndef AGX_HPP
#define AGX_HPP

#include <string>
#include <iostream>

class AGX {
public:
    AGX();
    void setModel(const std::string &m);
    void setAI(double a);
    void setCudaCores(double c);
    void setTensorCores(double t);
    void setMemory(double m);
    void setStorage(double s);

    void print(std::ostream &os = std::cout) const;
    void save(std::ostream &ofs) const;

    std::string model;
    double AI;
    double cuda_cores;
    double tensor_cores;
    double memory;
    double storage;
};

#endif // AGX_HPP
