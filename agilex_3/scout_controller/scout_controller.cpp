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
        case 'q':
          linear_vel = step;
          angular_vel = step;
          break;
        case 'e':
          linear_vel = step;
          angular_vel = -step;
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