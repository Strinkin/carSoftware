#ifndef __ADD_FIXED_PARAM_H__
#define __ADD_FIXED_PARAM_H__

#include <string>
#include "carInfo.hpp"
#include "studentInfo.hpp"

void addCarParam(CarInfo &car, std::string car_sn, std::string chassis_sn);
void addStudentParam(StudentInfo &student, std::string id, std::string name);

#endif // __ADD_FIXED_PARAM_H__
