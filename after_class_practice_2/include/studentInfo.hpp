#ifndef __STUDENTINFO_HPP__
#define __STUDENTINFO_HPP__

#include <string>
#include <iostream>
#include <fstream>

class StudentInfo {
public:
    StudentInfo();
    void setId(const std::string &s);
    void setName(const std::string &n);
    void print(std::ostream &os = std::cout) const;
    void save(std::string &path) const;

    std::string id;
    std::string name;
};

#endif // __STUDENTINFO_HPP__
