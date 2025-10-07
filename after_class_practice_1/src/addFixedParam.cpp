#include "CarInfo.h"
#include "StudentInfo.h"
#include "addFixedParam.h"

void addCarParam(CarInfo &car, string car_sn, string chassis_sn) {
    car.serial_number = car_sn;
    car.chassis = addChassisParam(chassis_sn);
    car.agx = addAGXParam();
    car.camera = addCameraParam();
    car.lidar = addLidarParam();
    car.imu = addIMUParam();
    car.screen = addScreenParam();
    car.battery = addBatteryParam();
}

Chassis addChassisParam(string chassis_sn) {
    Chassis chassis;
    chassis.serial_number = chassis_sn;
    chassis.model = "SCOUT MINI";
    chassis.wheel_base = 451.0;
    chassis.track_width = 490.0;
    chassis.min_ground_clearance = 115.0;
    chassis.min_turning_radius = 0.0;
    chassis.drive_type = "四轮四驱";
    chassis.max_travel_distance = 10.0;
    chassis.tire = addTireParam();
    return chassis;
}

Tire addTireParam() {
    Tire tire;
    tire.model = ROAD_TIRE;
    tire.size = 172.0;
    return tire;
}

AGX addAGXParam() {
    AGX agx;
    agx.model = "AGX Xavier";
    agx.AI = 32.0;
    agx.cuda_cores = 512.0;
    agx.tensor_cores = 64.0;
    agx.memory = 32.0;
    agx.storage = 32.0;
    return agx;
}

Camera addCameraParam() {
    Camera camera;
    camera.model = "RealSense D435i";
    camera.rgb_camera = "D430";
    camera.rgb_resolution = {1920, 1080};
    camera.rgb_fps = 30.0;
    camera.fov = {87, 58};
    camera.depth_fps = 90.0;
    return camera;
}

Lidar addLidarParam() {
    Lidar lidar;
    lidar.model = "RS-Helios-16";
    lidar.channels = 16;
    lidar.range = 100.0;
    lidar.power = 8.0;
    return lidar;
}

IMU addIMUParam() {
    IMU imu;
    imu.model = "CH110";
    imu.manufacturer = "NXP";
    return imu;
}

Screen addScreenParam() {
    Screen screen;
    screen.size = 11.6;
    screen.model = "super";
    return screen;
}

Battery addBatteryParam() {
    Battery battery;
    battery.voltage = 24.0;
    battery.capacity = 15.0;
    battery.charge_time = 2.0;
    return battery;
}

void addStudentParam(StudentInfo &student, string id, string name) {
    student.id = id;
    student.name = name;
}
