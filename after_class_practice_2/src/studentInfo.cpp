#include "../include/studentInfo.hpp"
#include <ostream>

StudentInfo::StudentInfo(): id(""), name("") {}
void StudentInfo::setId(const std::string &s) { id = s; }
void StudentInfo::setName(const std::string &n) { name = n; }
void StudentInfo::print(std::ostream &os) const { os << "Student: id=" << id << ", name=" << name << "\n"; }
void StudentInfo::save(std::ofstream &ofs) const { ofs << "Student\n"; ofs << "id:" << id << "\n"; ofs << "name:" << name << "\n"; }
