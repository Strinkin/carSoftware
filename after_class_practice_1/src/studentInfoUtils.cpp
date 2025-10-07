#include "studentInfoUtils.h"
#include <iostream>
#include <fstream>

void printStudentInfo(const StudentInfo& student) {
    std::cout << "学号: " << student.id << ", 姓名: " << student.name << std::endl;
}

void saveStudentInfo(const StudentInfo& student, const std::string& path) {
    std::ofstream ofs(path);
    if (!ofs) return;
    ofs << "学号: " << student.id << std::endl;
    ofs << "姓名: " << student.name << std::endl;
    ofs.close();
}

void saveStudentInfoJson(const StudentInfo& student, const std::string& path) {
    std::ofstream ofs(path);
    if (!ofs) return;
    ofs << "{\n";
    ofs << "  \"id\": \"" << student.id << "\",\n";
    ofs << "  \"name\": \"" << student.name << "\"\n";
    ofs << "}\n";
    ofs.close();
}

void printStudentInfoJson(const StudentInfo& student) {
    std::cout << "{\n";
    std::cout << "  \"id\": \"" << student.id << "\",\n";
    std::cout << "  \"name\": \"" << student.name << "\"\n";
    std::cout << "}\n";
}
