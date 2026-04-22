#ifndef ATRACSYS_FUNCTIONS
#define ATRACSYS_FUNCTIONS
#include <limits>
#include "ftkInterface.h"
#include <stdexcept>
#include <thread>
#include <chrono>
#include "geometryHelper.hpp"
#include "helpers.hpp"
#include "kinematics.h"
#include "templated_classes/Templated_Transform.h"
#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <fstream>

float MAX_REGISTRATION_ERROR = .4;
const int MAX_FIDUCIALS_IN_MARKER = 4;

struct frame_errors {
  double translation;
  double angular;
};
std::vector<Point<double>> loadFiducials(const std::string &file) {
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
enum atracsys_bitmask { BOTH, FOM1, FOM2 };

// static std::string M1_file = "../scripts/geometry100000.ini";
// static std::string M2_file = "../scripts/geometry999.ini";

template <typename T>
T atracsys_transform_to_templated_transform(ftkMarker *marker,
                                            Transform<T> &transform) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      transform.R.matrix[i][j] = T(marker->rotation[i][j]);
    }
  }
  transform.p.x = T(marker->translationMM[0]);
  transform.p.y = T(marker->translationMM[1]);
  transform.p.z = T(marker->translationMM[2]);
  return marker->registrationErrorMM;
}

template <typename T> class AtracsysTracker {
public:
  std::vector<Point<double>> M1_fids;
  std::vector<Point<double>> M2_fids;
  AtracsysTracker(const std::string &M1_file, const std::string &M2_file)
      : M1_file(M1_file), M2_file(M2_file) {
    ftkBuffer buffer;
    lib = ftkInitExt(nullptr, &buffer);
    if (lib == nullptr) {
      throw std::runtime_error(std::string("Failed to init Atracsys: ") +
                               buffer.data);
    }

    device = DeviceData(retrieveLastDevice(lib, true, false, true));
    sn = device.SerialNumber;

    if (ftkEnumerateOptions(lib, sn, optionEnumerator, &options) !=
            ftkError::FTK_OK ||
        options.empty()) {
      throw std::runtime_error(
          "Cannot retrieve any options from Atracsys device.");
    }
    // Build full path for registration
    std::string M1_full_path =
        "/home/tyler/Downloads/fusionTrack_x64_public_v4.10.1/"
        "fusionTrack_SDK-v4.10.1-linux64/data/" +
        M1_file;
    if (loadRigidBody(lib, M1_file, M1) < 0) {
      throw std::runtime_error("Couldn't load M1 file: " + M1_file);
    }
    // Register M1 with device using full path
    if (ftkSetRigidBody(lib, sn, &M1) != ftkError::FTK_OK) {
      throw std::runtime_error("Couldn't register M1 with device: " +
                               M1_full_path);
    }

    // Load M2 using just filename
    if (loadRigidBody(lib, M2_file, M2) < 0) {
      throw std::runtime_error("Couldn't load M2 file: " + M2_file);
    }

    // Build full path for registration
    std::string M2_full_path =
        "/home/tyler/Downloads/fusionTrack_x64_public_v4.10.1/"
        "fusionTrack_SDK-v4.10.1-linux64/data/" +
        M2_file;

    // Register M2 with device using full path
    if (ftkSetRigidBody(lib, sn, &M2) != ftkError::FTK_OK) {
      throw std::runtime_error("Couldn't register M2 with device: " +
                               M2_full_path);
    }
    M1_fids = loadFiducials(M1_full_path);
    M2_fids = loadFiducials(M2_full_path);
  }
  void reload() {
    std::cout << "reloading" << std::endl;
    ftkBuffer buffer;
    lib = ftkInitExt(nullptr, &buffer);
    if (lib == nullptr) {
      throw std::runtime_error(std::string("Failed to init Atracsys: ") +
                               buffer.data);
    }

    device = DeviceData(retrieveLastDevice(lib, true, false, true));
    sn = device.SerialNumber;

    if (ftkEnumerateOptions(lib, sn, optionEnumerator, &options) !=
            ftkError::FTK_OK ||
        options.empty()) {
      throw std::runtime_error(
          "Cannot retrieve any options from Atracsys device.");
    }
    // Build full path for registration
    std::string M1_full_path =
        "/home/tyler/Downloads/fusionTrack_x64_public_v4.10.1/"
        "fusionTrack_SDK-v4.10.1-linux64/data/" +
        M1_file;
    if (loadRigidBody(lib, M1_file, M1) < 0) {
      throw std::runtime_error("Couldn't load M1 file: " + M1_file);
    }
    // Register M1 with device using full path
    if (ftkSetRigidBody(lib, sn, &M1) != ftkError::FTK_OK) {
      throw std::runtime_error("Couldn't register M1 with device: " +
                               M1_full_path);
    }

    // Load M2 using just filename
    if (loadRigidBody(lib, M2_file, M2) < 0) {
      throw std::runtime_error("Couldn't load M2 file: " + M2_file);
    }

    // Build full path for registration
    std::string M2_full_path =
        "/home/tyler/Downloads/fusionTrack_x64_public_v4.10.1/"
        "fusionTrack_SDK-v4.10.1-linux64/data/" +
        M2_file;

    // Register M2 with device using full path
    if (ftkSetRigidBody(lib, sn, &M2) != ftkError::FTK_OK) {
      throw std::runtime_error("Couldn't register M2 with device: " +
                               M2_full_path);
    }
  }
  int getMeasurement(enum atracsys_bitmask mask, Measurement<T> &measurement,
                     frame_errors *M1_errs = nullptr,
                     frame_errors *M2_errs = nullptr) {
    ftkFrameQuery *frame = ftkCreateFrame();
    if (!frame) {
      std::cerr << "Cannot create frame instance" << std::endl;
      return -1;
    }

    if (ftkSetFrameOptions(false, false, 0u, 0u, 100u, 4u, frame) !=
        ftkError::FTK_OK) {
      ftkDeleteFrame(frame);
      std::cerr << "Could not initialize frame." << std::endl;
      return -1;
    }

    int tries = 0;
    while (tries++ < 10) {
      //std::cout << "try number " << tries << std::endl;
      if (ftkGetLastFrame(lib, sn, frame, 1000) == ftkError::FTK_OK &&
          frame->markersCount > 0) {
        break;
      }
      //std::cerr << "frame not found" << std::endl;
      if (tries == 9) {
        reload();
        return -1;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    ftkMarker *markerOne = nullptr;
    ftkMarker *markerTwo = nullptr;

    for (int i = 0; i < frame->markersCount; ++i) {
      if ((mask == BOTH || mask == FOM1) &&
          frame->markers[i].geometryId == M1.geometryId) {
        markerOne = &frame->markers[i];
      }
      if ((mask == BOTH || mask == FOM2) &&
          frame->markers[i].geometryId == M2.geometryId) {
        markerTwo = &frame->markers[i];
      }
    }
    if (!markerOne) {
      std::cerr << "Warning: M1 marker not visible this frame\n";
      return -1;
    }
    if (!markerTwo) {
      std::cerr << "Warning: M2 marker not visible this frame\n";
      return -1;
    }

    if ((mask == FOM1 && !markerOne) || (mask == FOM2 && !markerTwo) ||
        (mask == BOTH && !markerOne) || (mask == BOTH && !markerTwo)) {
      ftkDeleteFrame(frame);
      return -1;
    }
    float err1 = 0.f, err2 = 0.f;
    if (mask == FOM1 || mask == BOTH && markerOne != nullptr) {
      err1 = atracsys_transform_to_templated_transform(markerOne,
                                                       measurement.F_OM1);
      if (M1_errs != nullptr) {
        try {
          get_frame_errors(frame, markerOne, measurement.F_OM1, M1_fids,
                           M1_errs);
        } catch (std::runtime_error &e) {
          std::cout << "error in frame errors" << std::endl;
          std::cerr << e.what() << std::endl;
          return -1;
        }
      }
    }
    if (mask == FOM2 || mask == BOTH && markerTwo != nullptr) {
      err2 = atracsys_transform_to_templated_transform(markerTwo,
                                                       measurement.F_OM2);
      if (M2_errs != nullptr) {
        try {
          get_frame_errors(frame, markerTwo, measurement.F_OM2, M2_fids,
                           M2_errs);
        } catch (std::runtime_error &e) {
          std::cout << "error in frame errors" << std::endl;
          std::cerr << e.what() << std::endl;
          return -1;
        }
      }
    }

    ftkDeleteFrame(frame);

    if (err1 > MAX_REGISTRATION_ERROR || err2 > MAX_REGISTRATION_ERROR) {
      std::cerr << "error too high on marker reading" << std::endl;
      return -1;
    }

    return 0;
  }
  void get_frame_errors(const ftkFrameQuery *frame, const ftkMarker *marker,
                        const Transform<double> &computed_transform,
                        std::vector<Point<double>> fids, frame_errors *ret) {

    double max_double_value = std::numeric_limits<double>::max();

    Point<double> alpha = {max_double_value, max_double_value,
                           max_double_value};
    Point<double> epsilon = {max_double_value, max_double_value,
                             max_double_value};
    for (int n = 0; n < MAX_FIDUCIALS_IN_MARKER; n++) {
      uint32 fidId = marker->fiducialCorresp[n];
      if (fidId != INVALID_ID &&
          frame->threeDFiducialsStat == ftkQueryStatus::QS_OK &&
          fidId < frame->threeDFiducialsCount) {

        ftk3DFiducial &fid = frame->threeDFiducials[fidId];
        Point<double> fid_point = {fid.positionMM.x, fid.positionMM.y,
                                   fid.positionMM.z};
        Point<double> b = fids[n];
        double e = fid.triangulationErrorMM;
        update_error_vec(computed_transform.R, epsilon, e);
        if (b.magnitude() >= .001) {
          update_error_vec(computed_transform.R * skew(-1 * b), alpha, e);
        }
      } else {
        std::cout << "Marker " << n << " not found" << std::endl;
        throw std::runtime_error("marker not found");
      }
    }
    ret->translation = epsilon.magnitude();
    ret->angular = alpha.magnitude();
  }

private:
  void update_error_vec(const Rotation<double> R, Point<double> &error_vec,
                        double e) {
    double R_max =
        std::max(std::max(std::abs(R.matrix[0][0]), std::abs(R.matrix[1][0])),
                 std::abs(R.matrix[2][0]));
    error_vec.x = std::min(e / R_max, std::abs(error_vec.x));
    R_max =
        std::max(std::max(std::abs(R.matrix[0][1]), std::abs(R.matrix[1][1])),
                 std::abs(R.matrix[2][1]));
    error_vec.y = std::min(e / R_max, std::abs(error_vec.y));
    R_max =
        std::max(std::max(std::abs(R.matrix[0][2]), std::abs(R.matrix[1][2])),
                 std::abs(R.matrix[2][2]));
    error_vec.z = std::min(e / R_max, std::abs(error_vec.z));
  }
  Rotation<double> skew(const Point<double> p) {
    Rotation<double> R;
    for (int i = 0; i < 3; i++) {
      R.matrix[i][i] = 0;
    }
    R.matrix[0][1] = -p.z;
    R.matrix[1][0] = p.z;
    R.matrix[0][2] = p.y;
    R.matrix[2][0] = -p.y;
    R.matrix[1][2] = -p.x;
    R.matrix[2][1] = p.x;
    return R;
  }

  ftkLibrary lib;
  DeviceData device;
  uint64 sn;
  ftkRigidBody M1, M2;
  std::map<std::string, uint32> options;
  std::string M1_file, M2_file;
};

#endif // !define ATRACSYS_FUNCTIONS
