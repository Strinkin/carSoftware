#ifndef __STUDENTINFOUTILS_H__
#define __STUDENTINFOUTILS_H__

#include <string>
#include "studentInfo.h"

// 打印学生信息
void printStudentInfo(const StudentInfo& student);
// 保存学生信息为文本
void saveStudentInfo(const StudentInfo& student, const std::string& path);
// 保存学生信息为JSON
void saveStudentInfoJson(const StudentInfo& student, const std::string& path);
// 打印学生信息为JSON
void printStudentInfoJson(const StudentInfo& student);

#endif // __STUDENTINFOUTILS_H__