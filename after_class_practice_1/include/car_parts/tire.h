// tire.h
#ifndef __TIRE_H__
#define __TIRE_H__

enum TireModel {
    ROAD_TIRE,      // 公路轮
    MECANUM_TIRE    // 麦克纳姆轮
};

struct Tire {
    TireModel model;    // 轮胎型号
    double size;        // 尺寸: 172mm
};

#endif // __TIRE_H__