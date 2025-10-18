#include "../include/studentInfo.hpp"
#include <ostream>
#include <string>
StudentInfo::StudentInfo(): id(""), name("") {}
void StudentInfo::setId(const std::string &s) { id = s; }
void StudentInfo::setName(const std::string &n) { name = n; }
void StudentInfo::print(std::ostream &os) const { os << "Student: id=" << id << ", name=" << name << "\n"; }
void StudentInfo::save(std::string &path) const { 
    std::ofstream ofs(path);
    if (!ofs) return;
    ofs << "Student\n"; ofs << "id:" << id << "\n"; ofs << "name:" << name << "\n";
    ofs.close();
}
