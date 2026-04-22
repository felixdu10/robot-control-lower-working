#include "backend.h"
#include "Templated_Point.h"
#include "Templated_Transform.h"
#include <iostream>
#include <vector>
#include <fstream>

void Backend::hello() { std::cout << "hello from backend" << std::endl; }

void Backend::sendAtracsysMeasurement() {
  if (!isConnectedToAtracsys) {
    AtracsysTracker<double> a("geometry100000.ini", "geometry999.ini");
    isConnectedToAtracsys = true;
  }
  Measurement<double> atracsys_measurement;
  a.getMeasurement(BOTH, atracsys_measurement);
  std::string M1_full_path =
      "/home/tyler/Downloads/fusionTrack_x64_public_v4.10.1/"
      "fusionTrack_SDK-v4.10.1-linux64/data/geometry100000.ini";
  std::vector<Point<double>> M1_fids = loadFiducials(M1_full_path);
  for (int i = 0; i < M1_fids.size(); i++) {
    Point<double> p = {M1_fids[i].x, M1_fids[i].y,
                       M1_fids[i].z};
    Point<double> global_loc = atracsys_measurement.F_OM1 * p;
  }
}

std::vector<Point<double>> Backend::loadFiducials(const std::string &file) {
  std::ifstream in(file);
  std::string line;
  bool inFiducial = false;
  std::vector<Point<double>> fiducials;
  Point<double> pos;
  std::vector<Point<double>> ret;
  while (std::getline(in, line)) {
    if (line.rfind("[fiducial", 0) == 0) {
      inFiducial = true;
    } else if (line.find("x=") == 0) {
      pos.x = (std::stof(line.substr(2)));
    } else if (line.find("y=") == 0) {
      pos.y = (std::stof(line.substr(2)));
    } else if (line.find("z=") == 0) {
      pos.z = (std::stof(line.substr(2)));
    } else if (line.find("[geometry]") == 0) {
      break;
    }

    if (inFiducial && !line.empty() && line.find("z=") == 0) {
      ret.push_back(pos);
      inFiducial = false;
    }
  }
  return ret;
}
