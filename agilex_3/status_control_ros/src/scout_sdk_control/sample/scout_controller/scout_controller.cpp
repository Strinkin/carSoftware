#include <unistd.h>
#include <thread>
#include <memory>
#include <iostream>

#include "ugv_sdk/mobile_robot/scout_robot.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"

using namespace westonrobot;

char getch_timeout_select() {
    fd_set fds;
    struct timeval tv;
    char buf = 0;
    struct termios old = {0};
    
    // 保存当前终端设置
    if (tcgetattr(0, &old) < 0) {
        perror("tcgetattr()");
        return 0;
    }
    
    // 设置非规范模式和关闭回显
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    
    // 应用新的终端设置
    if (tcsetattr(0, TCSANOW, &old) < 0) {
        perror("tcsetattr()");
        return 0;
    }
    
    // 使用select设置100ms超时
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100ms = 100,000 微秒
    
    int ready = select(1, &fds, NULL, NULL, &tv);
    
    if (ready > 0) {
        // 有数据可读
        if (read(0, &buf, 1) < 0) {
            perror("read()");
            buf = 0;
        }
    } else if (ready == 0) {
        // 超时
        buf = 0; // 返回0表示超时
    } else {
        // select错误
        perror("select()");
        buf = 0;
    }
    
    // 恢复原始终端设置
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) {
        perror("tcsetattr restore()");
    }
    
    return buf;
}

int main(int argc, char **argv) {
  std::string device_name = "can0"; // can0
  std::cout << "Selected interface " << device_name << std::endl;

  std::unique_ptr<ScoutMiniOmniRobot> scout;

  ProtocolDetector detector;
  detector.Connect(device_name);
  // auto proto = detector.DetectProtocolVersion(5); // time_out 5
  scout = std::unique_ptr<ScoutMiniOmniRobot>(
        new ScoutMiniOmniRobot(ProtocolVersion::AGX_V2));

  scout->Connect(device_name);

  if (scout->GetParserProtocolVersion() == ProtocolVersion::AGX_V2) {
    scout->EnableCommandedMode();
  }

  int count = 0;
  double linear_vel = 0.0;
  double angular_vel = 0.0;
  double lateral_velocity = 0.0;
  double step = 0.1;
  while (true) {

   
    if (true) { // 非阻塞检测
      char key = getch_timeout_select();
      switch (key) {
        case 'w':
          linear_vel = step;
          angular_vel = 0.0;
          std::cout << "Pressed w: linear_vel = 0.1" << std::endl;
          break;
        case 's':
          linear_vel = -step;
          angular_vel = 0.0;
          std::cout << "Pressed s: linear_vel = -0.1" << std::endl;
          break;
        case 'a':
          linear_vel = 0.0;
          angular_vel = step;
          std::cout << "Pressed a: angular_vel = 0.1" << std::endl;
          break;
        case 'd':
          linear_vel = 0.0;
          angular_vel = -step;
          std::cout << "Pressed d: angular_vel = -0.1" << std::endl;
          break;
        case '=':
          step += 0.1;
          break;
        case '-':
          step -= 0.1;
          break;
        case ' ':
          linear_vel = 0.0;
          angular_vel = 0.0;
          lateral_velocity = 0.0;
          std::cout << "Pressed space: stop all movement" << std::endl;
          break;
      }
    }
    int _ = system("clear");
    // motion control
    std::cout << "Setting motion command: " << linear_vel << " " << angular_vel << " " << lateral_velocity << std::endl;
    scout->SetMotionCommand(linear_vel, angular_vel, lateral_velocity);

    auto state = scout->GetRobotState();
    std::cout << "-------------------------------" << std::endl;
    std::cout << "count: " << count << std::endl;
    std::cout << "velocity (linear, angular, lateral): ["
              << state.motion_state.linear_velocity << ", "
              << state.motion_state.angular_velocity << ", "
              << state.motion_state.lateral_velocity << "]"
	      << std::endl;

    auto actuator = scout->GetActuatorState(); // 获取执行器状态
    for (int i = 0; i < 4; ++i) { // 打印4个电机编号、电流、转速、驱动器温度、电机温度
      printf("motor %d: current %f, rpm %d, driver temp %f, motor temp %f\n",
             actuator.actuator_state[i].motor_id,
             actuator.actuator_state[i].current,
             actuator.actuator_state[i].rpm,
             actuator.actuator_state[i].driver_temp,
             actuator.actuator_state[i].motor_temp);
    }
    std::cout << "-------------------------------" << std::endl;

    //usleep(20*1000);
    ++count;
  }

  return 0;
}
