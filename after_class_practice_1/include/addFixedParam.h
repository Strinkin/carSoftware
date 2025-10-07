#ifndef __ADD_FIXED_PARAM_H__
#define __ADD_FIXED_PARAM_H__

#include <string>
#include "carInfo.h"
#include "studentInfo.h"

void addCarParam(CarInfo &car, string car_sn, string chassis_sn);
Chassis addChassisParam(string chassis_sn);
Tire addTireParam();
AGX addAGXParam();
Camera addCameraParam();
Lidar addLidarParam();
IMU addIMUParam();
Screen addScreenParam();
Battery addBatteryParam();

void addStudentParam(StudentInfo &student, string id, string name);

#endif // __ADD_FIXED_PARAM_H__