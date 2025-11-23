#include <unistd.h>

#include <memory>
#include <iostream>

#include "ugv_sdk/mobile_robot/scout_robot.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"

using namespace westonrobot;

int main(int argc, char **argv) {
  std::string device_name = "can0"; // can0
  std::cout << "Selected interface " << device_name << std::endl;

  std::unique_ptr<ScoutMiniOmniRobot> scout;

  ProtocolDetector detector;
  detector.Connect(device_name);
  auto proto = detector.DetectProtocolVersion(5); // time_out
  if (proto == ProtocolVersion::AGX_V1) {
    std::cout << "Detected protocol: AGX_V1" << std::endl;
    scout = std::unique_ptr<ScoutMiniOmniRobot>(
        new ScoutMiniOmniRobot(ProtocolVersion::AGX_V1));

  } else if (proto == ProtocolVersion::AGX_V2) {
    std::cout << "Detected protocol: AGX_V2" << std::endl;
    scout = std::unique_ptr<ScoutMiniOmniRobot>(
        new ScoutMiniOmniRobot(ProtocolVersion::AGX_V2));
  } else {
    std::cout << "Detected protocol: UNKONWN" << std::endl;
    return -1;
  }

  if (scout == nullptr)
    std::cout << "Failed to create robot object" << std::endl;

  scout->Connect(device_name);

  if (scout->GetParserProtocolVersion() == ProtocolVersion::AGX_V2) {
    scout->EnableCommandedMode();
  }

  // light control
  std::cout << "Light: const off" << std::endl;
  scout->SetLightCommand(CONST_OFF, 0, CONST_OFF, 0);

  int count = 0;
  int light = 0; // 0: OFF, 1: ON
  while (true) {
    int _ = system("clear");
    // light control
    if (count % 50 == 0 && light == 0) {
      // std::cout << "Light: ON" << std::endl;
      scout->SetLightCommand(CONST_ON, 0, CONST_ON, 0);
      light = 1;
    } else if (count % 50 == 0 && light == 1) {
      // std::cout << "Light: OFF" << std::endl;
      scout->SetLightCommand(CONST_OFF, 0, CONST_OFF, 0);
      light = 0;
    }
    
    // get robot state
    auto state = scout->GetRobotState();
    std::cout << "-------------------------------" << std::endl;
    std::cout << "count: " << count << std::endl;

    if (light == 0)
        std::cout << "Light: OFF" << std::endl;
    else
        std::cout << "Light: ON" << std::endl;

    std::cout << ", battery voltage: " << state.system_state.battery_voltage << " V"
              << std::endl;
    std::cout << "velocity (linear, angular, lateral): ["
              << state.motion_state.linear_velocity << ", "
              << state.motion_state.angular_velocity << ", "
              << state.motion_state.lateral_velocity << "]"
	      << std::endl;

    auto actuator = scout->GetActuatorState(); // 获取执行器状态

    for (int i = 0; i < 4; ++i) {
      printf("motor %d: current %.2f, rpm %d, driver temp %.2f, motor temp %.2f\n",
             actuator.actuator_hs_state[i].motor_id,
             actuator.actuator_hs_state[i].current,
             actuator.actuator_hs_state[i].rpm,
             actuator.actuator_ls_state[i].driver_temp,
             actuator.actuator_ls_state[i].motor_temp);
    }
    std::cout << "-------------------------------" << std::endl;

    usleep(20*1000);
    ++count;
  }

  return 0;
}