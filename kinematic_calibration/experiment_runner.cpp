#include <iostream>
#include <array>
#include "robot_backend_ros.h"

int main() {
  try {
    RobotBackendROS backend;

    std::cout << "=== Step 1: HOME ===" << std::endl;
    backend.home();

    std::cout << "=== Step 2: MOVE to test target ===" << std::endl;
    std::array<float, 4> target = {10.0f, 10.0f, 10.0f, 10.0f};
    backend.move_raw(target, 1.0f);

    std::cout << "Robot reached target. Press ENTER to home again..." << std::endl;
    std::cin.get();

    std::cout << "=== Step 3: HOME again ===" << std::endl;
    backend.home();

    std::cout << "Done." << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "ERROR: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
