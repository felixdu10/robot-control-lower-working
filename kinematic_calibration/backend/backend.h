#ifndef BACKEND
#define BACKEND
#include "atracsys_functions.h"
#include "robot_controller.h"

class Backend {
public:
  bool isConnectedToRobot = false;
  bool isConnectedToAtracsys = false;

  AtracsysTracker<double> a;
  void hello();
  RobotController robot;
  void sendAtracsysMeasurement();
private:
};

#endif
