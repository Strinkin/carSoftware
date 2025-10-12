#ifndef STUDENTINFO_HPP
#define STUDENTINFO_HPP

#include <string>
#include <iostream>
#include <fstream>

class StudentInfo {
public:
    StudentInfo();
    void setId(const std::string &s);
    void setName(const std::string &n);
    void print(std::ostream &os = std::cout) const;
    void save(std::ofstream &ofs) const;

    std::string id;
    std::string name;
};

#endif // STUDENTINFO_HPP
