#ifndef __CARINFOUTILS_H__
#define __CARINFOUTILS_H__

#include "carInfo.h"
#include <string>

void printCarInfo(const CarInfo& car);
void saveCarInfo(const CarInfo& car, const std::string& path);
void saveCarInfoJson(const CarInfo& car, const std::string& path);
void printCarInfoJson(const CarInfo& car);

#endif // __CARINFOUTILS_H__
