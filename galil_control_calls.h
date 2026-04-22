#include "gclib.h"
#include "gclibo.h"
#include "kinematic_calibration/kinematics.h"
namespace External {
#include "kinematic_structs.h"
}
#include <chrono>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
using namespace std::chrono_literals;

#ifndef GALIL_CALLS
#define GALIL_CALLS
const std::string galil_dir = "/home/tyler/projects/robot_control_code/Galil/";

enum init_types {

  home = 1,
  reho = 2,
  up_or_down = 3,
  test = 4,
  move = 5,
};

static GCon g = 0;
const GCStringIn GALIL_IP_STRING = "192.168.1.10";
static int init_mode = -1;
inline void stop_galil() {
  if (g)       // don't call close on a nullptr
    GClose(g); // Don't forget to close!
  init_mode = -1;
}

void check(GReturn rc) {
  if (rc != G_NO_ERROR) {
    std::cout << "ERROR:" << rc << std::endl;
    stop_galil();
    throw std::runtime_error("galil error");
  }
}
std::string to_string(double d) {
  std::stringstream ss;
  ss << d;
  return ss.str();
}

void init_galil(int home_reho_or_updown) {
  if (init_mode == home_reho_or_updown) {
    return;
  }
  if (init_mode != -1) {
    stop_galil();
  }
  char buf[1024]; // traffic buffer

  check(GVersion(buf, sizeof(buf)));
  printf("version: %s\n", buf); // Print the library version

  char addresses[1024];
  GAddresses(addresses, sizeof(addresses));
  std::cout << "Devices found: " << addresses << std::endl;

  check(GOpen("192.168.1.10 --direct",
              &g)); // Open a connection to Galil, store the identifier in g.

  check(GInfo(g, buf, sizeof(buf)));
  printf("info: %s\n", buf); // Print the connection info

  check(GCommand(g, "MG TIME", buf, sizeof(buf),
                 0)); // Send MG TIME. Because response is ASCII, don't care
                      // about bytes read.
  printf("response: %s\n", buf); // Print the response
  if (home_reho_or_updown == 1)
    check(GProgramDownloadFile(g, (galil_dir + "home.dmc").c_str(), "--max 4"));
  if (home_reho_or_updown == 2)
    check(GProgramDownloadFile(g, (galil_dir + "rc.dmc").c_str(), "--max 4"));
  if (home_reho_or_updown == 3) {
    check(GProgramDownloadFile(g, (galil_dir + "test.dmc").c_str(), "--max 4"));
    // char program[G_HUGE_BUFFER];
    // GProgramUpload(g, program, G_HUGE_BUFFER);
    // printf("%s\n", program);
  }
  if (home_reho_or_updown == 4) {
    check(
        GProgramDownloadFile(g, (galil_dir + "spinA.dmc").c_str(), "--max 4"));
  }
  if (home_reho_or_updown == 5) {
    check(GProgramDownloadFile(g, (galil_dir + "moveAll.dmc").c_str(),
                               "--max 4"));
  }
  init_mode = home_reho_or_updown;
}

int GoToLowBlocking(double left, double right) {
  char buf[G_SMALL_BUFFER]; // traffic buffer
  GSize read_bytes = 0;     // bytes read in GCommand
  int value = 0;
  GCommand(g, ("tgtMmB = " + to_string(right)).c_str(), buf, G_SMALL_BUFFER,
           &read_bytes);
  GCommand(g, ("tgtMmE = " + to_string(left)).c_str(), buf, G_SMALL_BUFFER,
           &read_bytes);
  GCommand(g, "XQ #GoToLow, 7", buf, G_SMALL_BUFFER, &read_bytes);
  std::this_thread::sleep_for(100ms);
  do {
    GCmdI(g, "dMotion = ?", &value);
    std::this_thread::sleep_for(500ms);
  } while (!value);
  std::this_thread::sleep_for(1750ms);
  return 0;
}

// int GoToBothBlocking(slider_positions sliders) {
//   char buf[G_SMALL_BUFFER];
//   char buf2[G_SMALL_BUFFER]; // traffic buffer
//   GSize read_bytes = 0;
//   GSize read_bytes2 = 0; // bytes read in GCommand
//   int valueU = 0;
//   int valueL = 0;
//   GCommand(g, ("tgtMmB = " +
//   to_string(sliders.right_middle_slider_y)).c_str(),
//            buf, G_SMALL_BUFFER, &read_bytes);
//   GCommand(g, ("tgtMmE = " +
//   to_string(sliders.left_middle_slider_y)).c_str(),
//            buf, G_SMALL_BUFFER, &read_bytes);
//   GCommand(g, ("tgtMmA = " + to_string(sliders.right_slider_y)).c_str(), buf,
//            G_SMALL_BUFFER, &read_bytes);
//   GCommand(g, ("tgtMmF = " + to_string(sliders.left_slider_y)).c_str(), buf,
//            G_SMALL_BUFFER, &read_bytes);
//   GCommand(g, "XQ #GoToLow, 3", buf2, G_SMALL_BUFFER, &read_bytes2);
//   GCommand(g, "XQ #GoToUp, 7", buf, G_SMALL_BUFFER, &read_bytes);
//   std::this_thread::sleep_for(100ms);
//   do {
//     GCmdI(g, "dMotionL = ?", &valueL);
//     GCmdI(g, "dMotionU = ?", &valueU);
//
//     std::this_thread::sleep_for(500ms);
//   } while (!(valueL && valueU));
//   std::this_thread::sleep_for(1750ms);
//   return 0;
// }

int GoToUpBlocking(double left, double right) {
  char buf[G_SMALL_BUFFER]; // traffic buffer
  GSize read_bytes = 0;     // bytes read in GCommand
  int value = 0;
  GCommand(g, ("tgtMmA = " + to_string(right)).c_str(), buf, G_SMALL_BUFFER,
           &read_bytes);
  GCommand(g, ("tgtMmF = " + to_string(left)).c_str(), buf, G_SMALL_BUFFER,
           &read_bytes);
  GCommand(g, "XQ #GoToUp, 7", buf, G_SMALL_BUFFER, &read_bytes);
  std::this_thread::sleep_for(300ms);
  do {
    GCmdI(g, "dMotion = ?", &value);
    std::this_thread::sleep_for(500ms);
  } while (!value);
  std::this_thread::sleep_for(1750ms);
  return 0;
}

int GoToPosBlocking(double left, double right) {
  char buf[G_SMALL_BUFFER]; // traffic buffer
  GSize read_bytes = 0;     // bytes read in GCommand
  int value = 0;
  GCommand(g, ("tgtMmB = " + to_string(right)).c_str(), buf, G_SMALL_BUFFER,
           &read_bytes);
  GCommand(g, ("tgtMmE = " + to_string(left)).c_str(), buf, G_SMALL_BUFFER,
           &read_bytes);
  GCommand(g, "XQ #GoToPos, 7", buf, G_SMALL_BUFFER, &read_bytes);
  std::this_thread::sleep_for(100ms);
  do {
    GCmdI(g, "dMotion = ?", &value);
    std::this_thread::sleep_for(500ms);
  } while (!value);
  std::this_thread::sleep_for(1750ms);
  return 0;
}

int HomeUpBlocking(bool msideA, bool msideF) {
  char buf[G_SMALL_BUFFER]; // traffic buffer
  GSize read_bytes = 0;     // bytes read in GCommand
  int value = 0;
  GCommand(g, ("msideA = " + to_string((int)msideA)).c_str(), buf,
           G_SMALL_BUFFER, &read_bytes);
  GCommand(g, ("msideF = " + to_string((int)msideF)).c_str(), buf,
           G_SMALL_BUFFER, &read_bytes);
  GCommand(g, "XQ #homeUp, 0", buf, G_SMALL_BUFFER, &read_bytes);
  std::this_thread::sleep_for(100ms);
  do {
    GCmdI(g, "dHome = ?", &value);
    std::this_thread::sleep_for(500ms);
  } while (!value);
  std::this_thread::sleep_for(1750ms);

  return 0;
}

int HomeLowBlocking(bool msideB, bool msideE) {
  char buf[G_SMALL_BUFFER]; // traffic buffer
  GSize read_bytes = 0;     // bytes read in GCommand
  int value = 0;
  GCommand(g, ("msideB = " + to_string((int)msideB)).c_str(), buf,
           G_SMALL_BUFFER, &read_bytes);
  GCommand(g, ("msideE = " + to_string((int)msideE)).c_str(), buf,
           G_SMALL_BUFFER, &read_bytes);
  GCommand(g, "XQ #homeLow, 0", buf, G_SMALL_BUFFER, &read_bytes);
  std::this_thread::sleep_for(100ms);
  do {
    GCmdI(g, "dHome = ?", &value);
    std::this_thread::sleep_for(500ms);
  } while (!value);
  std::this_thread::sleep_for(1750ms);
  return 0;
}

Thetas<double> home_positions = {
    External::BASE_TO_SLIDER_MAX - 60,    //- HALF_SLIDER_WIDTH,
    External::BASE_TO_SLIDER_MAX - 60,    //- HALF_SLIDER_WIDTH,
    External::BASE_TO_SLIDER_MAX - 26.79, // - HALF_SLIDER_WIDTH,
    External::BASE_TO_SLIDER_MAX - 23.48, //- HALF_SLIDER_WIDTH,
    0};

struct encoder_error_struct {
  double mmErrLeft;
  double mmErrRight;
  double mmErrLeftMiddle;
  double mmErrRightMiddle;
};
/**
 * @brief use the slider positions to move the robot into a desired position
 *
 * @param positions
 */
encoder_error_struct
move_robot_with_slider_positions(Thetas<double> positions) {
  std::string command;
  char buf[G_SMALL_BUFFER];
  GSize read_bytes = 0;
  double left = positions.theta_1 - home_positions.theta_1;
  double right = positions.theta_2 - home_positions.theta_2;
  double left_middle = positions.theta_3 - home_positions.theta_3;
  double right_middle = positions.theta_4 - home_positions.theta_4;
  // std::cout << left << " " << left_middle << " " << right_middle << " " <<
  // right
  //           << std::endl;
  int encoder_countA, encoder_countF, encoder_countE, encoder_countB = 0;
  check(GCmdI(g, "MG _TPA", &encoder_countA));
  check(GCmdI(g, "MG _TPF", &encoder_countF));
  check(GCmdI(g, "MG _TPE", &encoder_countE));
  check(GCmdI(g, "MG _TPB", &encoder_countB));
  double ctsPermmA = 947.419292;
  double ctsPermmF = 948.586925;
  double ctsPermmB = 942.555021;
  double ctsPermmE = 944.160381;
  double desired_ctsA = ctsPermmA * right;
  double desired_ctsF = ctsPermmF * left;
  double desired_ctsE = ctsPermmE * left_middle;
  double desired_ctsB = ctsPermmB * right_middle;
  int cts_diffA = encoder_countA + int(desired_ctsA);
  int cts_diffF = encoder_countF + int(desired_ctsF);
  int cts_diffE = encoder_countE + int(desired_ctsE);
  int cts_diffB = encoder_countB + int(desired_ctsB);
  // std::cout << "beginning motion" << std::endl;
  check(GCommand(g, ("AtgtCT = " + to_string(cts_diffA)).c_str(), buf,
                 G_SMALL_BUFFER, &read_bytes));
  check(GCommand(g, ("FtgtCT = " + to_string(cts_diffF)).c_str(), buf,
                 G_SMALL_BUFFER, &read_bytes));
  check(GCommand(g, ("EtgtCT = " + to_string(cts_diffE)).c_str(), buf,
                 G_SMALL_BUFFER, &read_bytes));
  check(GCommand(g, ("BtgtCT = " + to_string(cts_diffB)).c_str(), buf,
                 G_SMALL_BUFFER, &read_bytes));
  GCommand(g, "XQ #MoveAll, 0", buf, G_SMALL_BUFFER, &read_bytes);
  int done = 0;
  int time = 0;
  do {
    GCmdI(g, "dMotion= ?", &done);
    std::this_thread::sleep_for(500ms);
    time += 500;
    if (time > 1 * 1000) {
      std::cout << "motors stalled, trying to get them going" << std::endl;
      GCommand(g, ("STA; SHA; BGA;" + to_string(cts_diffA)).c_str(), buf,
               G_SMALL_BUFFER, &read_bytes);
      GCommand(g, ("STF; SHF; BGF;" + to_string(cts_diffA)).c_str(), buf,
               G_SMALL_BUFFER, &read_bytes);
      GCommand(g, ("STE; SHE; BGE;" + to_string(cts_diffA)).c_str(), buf,
               G_SMALL_BUFFER, &read_bytes);
      GCommand(g, ("STB; SHB; BGB;" + to_string(cts_diffA)).c_str(), buf,
               G_SMALL_BUFFER, &read_bytes);
      time = 0;
    }
  } while (!done);

  // std::cout << "finished motion" << std::endl;
  check(GCmdI(g, "MG _TPA", &encoder_countA));
  check(GCmdI(g, "MG _TPF", &encoder_countF));
  check(GCmdI(g, "MG _TPE", &encoder_countE));
  check(GCmdI(g, "MG _TPB", &encoder_countB));
  double AmmErr = (-encoder_countA - desired_ctsA) / ctsPermmA;
  double FmmErr = (-encoder_countF - desired_ctsF) / ctsPermmF;
  double EmmErr = (-encoder_countE - desired_ctsE) / ctsPermmE;
  double BmmErr = (-encoder_countB - desired_ctsB) / ctsPermmB;
  /*std::cout << "A: desired ct: " << desired_ctsA
            << " actual ct: " << encoder_countA << " error in mm: " << AmmErr
            << std::endl;
  std::cout << "F: desired ct: " << desired_ctsF
            << " actual ct: " << encoder_countF << " error in mm: " << FmmErr
            << std::endl;
  std::cout << "E: desired ct: " << desired_ctsE
            << " actual ct: " << encoder_countE << " error in mm: " << EmmErr
            << std::endl;
  std::cout << "B: desired ct: " << desired_ctsB
            << " actual ct: " << encoder_countB << " error in mm: " << BmmErr
            << std::endl;*/
  return {AmmErr, FmmErr, EmmErr, BmmErr};
}
static Thetas<double> last_position;

Thetas<double> get_encoder_positions() {

  Thetas<double> ret_thetas;
  int encoder_countA, encoder_countF, encoder_countE, encoder_countB = 0;
  check(GCmdI(g, "MG _TPA", &encoder_countA));
  check(GCmdI(g, "MG _TPF", &encoder_countF));
  check(GCmdI(g, "MG _TPE", &encoder_countE));
  check(GCmdI(g, "MG _TPB", &encoder_countB));
  double ctsPermmA = 947.419292;
  double ctsPermmF = 948.586925;
  double ctsPermmB = 942.555021;
  double ctsPermmE = 944.160381;
  ret_thetas.theta_1 = (ctsPermmF * -encoder_countF) + home_positions.theta_1;
  ret_thetas.theta_2 = (ctsPermmA * -encoder_countA) + home_positions.theta_2;
  ret_thetas.theta_3 = (ctsPermmE * -encoder_countE) + home_positions.theta_3;
  ret_thetas.theta_4 = (ctsPermmB * -encoder_countB) + home_positions.theta_4;
};

void sendTgtPositions(int encoder_countA, int encoder_countF,
                      int encoder_countB, int encoder_countE) {
  char buf[G_SMALL_BUFFER]; // traffic buffer
  GSize read_bytes = 0;     // bytes read in GCommand
  check(GCommand(g, ("tgtA = " + to_string(encoder_countA)).c_str(), buf,
                 G_SMALL_BUFFER, &read_bytes));
  check(GCommand(g, ("tgtF = " + to_string(encoder_countF)).c_str(), buf,
                 G_SMALL_BUFFER, &read_bytes));
  check(GCommand(g, ("tgtB = " + to_string(encoder_countB)).c_str(), buf,
                 G_SMALL_BUFFER, &read_bytes));
  check(GCommand(g, ("tgtE = " + to_string(encoder_countE)).c_str(), buf,
                 G_SMALL_BUFFER, &read_bytes));
}

#endif
