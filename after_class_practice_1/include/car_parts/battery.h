// battery.h
#ifndef __BATTERY_H__
#define __BATTERY_H__

struct Battery {
    double voltage;     // 电压: 24V
    double capacity;    // 容量: 15Ah
    double charge_time; // 充电时长: 2H
};

#endif // __BATTERY_H__