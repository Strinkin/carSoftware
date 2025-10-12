#include "../include/addFixedParam.h"

void addStudentParam(StudentInfo &student, std::string id, std::string name) {
    student.setId(id);
    student.setName(name);
}

void addCarParam(CarInfo &car, std::string car_sn, std::string chassis_sn) {
    car.setID(car_sn);

    // chassis
    car.chassis.setSerialNumber(chassis_sn);
    car.chassis.setModel("SCOUT MINI");
    car.chassis.setWheelBase(451.0);
    car.chassis.setTrackWidth(490.0);
    car.chassis.setMinGroundClearance(115.0);
    car.chassis.setMinTurningRadius(0.0);
    car.chassis.setDriveType("4WD");
    car.chassis.setMaxTravelDistance(10.0);
    Tire t; t.setModel(ROAD_TIRE); t.setSize(172.0);
    car.chassis.setTire(t);

    // agx
    car.agx.setModel("AGX Xavier");
    car.agx.setAI(32.0);
    car.agx.setCudaCores(512);
    car.agx.setTensorCores(64);
    car.agx.setMemory(32);
    car.agx.setStorage(32);

    // camera
    car.camera.setModel("RealSense D435i");
    car.camera.setRgbCamera("D430");
    car.camera.setRgbResolution(1920, 1080);
    car.camera.setRgbFps(30);
    car.camera.setFov(87, 58);
    car.camera.setDepthFps(90);

    // lidar
    car.lidar.setModel("RS-Helios-16");
    car.lidar.setChannels(16);
    car.lidar.setRange(100);
    car.lidar.setPower(8);

    // imu
    car.imu.setModel("CH110");
    car.imu.setManufacturer("NXP");

    // screen
    car.screen.setModel("super");
    car.screen.setSize(11.6);

    // battery
    car.battery.setVoltage(24);
    car.battery.setCapacity(15);
    car.battery.setChargeTime(2);
}
