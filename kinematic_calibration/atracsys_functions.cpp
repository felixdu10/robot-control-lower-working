#include "atracsys_functions.h"
#include "ftkInterface.h"
#include "kinematics.h"
#include "geometryHelper.hpp"
#include "helpers.hpp"
#include <cmath>
#include <iostream>
#include <map>
#include <string>

float MAX_REGISTRATION_ERROR = .2;

static std::string M1_file = "../scripts/geometry100000.ini";
static std::string M2_file = "../scripts/geometry999.ini";

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

template<typename T>
int get_atracsys_measurement(enum atracsys_bitmask mask,
                             Measurement<T> &atracsys_measurement) {
  const bool isNotFromConsole = isLaunchedFromExplorer();
  ftkBuffer buffer;
  ftkLibrary lib(ftkInitExt(nullptr, &buffer));
  if (lib == nullptr) {
    std::cerr << buffer.data << std::endl;
    std::cerr << "Cannot initialize driver" << !isNotFromConsole << std::endl;
    return -1;
  }

  DeviceData device(retrieveLastDevice(lib, true, false, !isNotFromConsole));
  uint64 sn(device.SerialNumber);

  std::map<std::string, uint32> options{};

  ftkError err(ftkEnumerateOptions(lib, sn, optionEnumerator, &options));
  if (options.empty()) {
    std::cout << "Cannot retrieve any options." << !isNotFromConsole
              << std::endl;
    return -1;
  }
  ftkRigidBody M1{};
  ftkRigidBody M2{};
  if (mask == BOTH || mask == FOM1) {
    if (loadRigidBody(lib, M1_file, M1) != 1) {
      std::cout << "Couldn't load M2 file." << std::endl;
      return -1;
    }
  }
  if (mask == BOTH || mask == FOM2) {

    if (loadRigidBody(lib, M2_file, M2) != 1) {
      std::cout << "Couldn't load M2 file." << std::endl;
      return -1;
    }
  }

  ftkFrameQuery *frame = ftkCreateFrame();
  if (frame == nullptr) {
    std::cout << "Cannot create frame instance" << std::endl;
    return -1;
  }

  if (ftkError::FTK_OK !=
      ftkSetFrameOptions(false, false, 0u, 0u, 0u, 4u, frame)) {
    ftkDeleteFrame(frame);
    std::cout << "Could not initialize frame." << std::endl;
    return -1;
  }

  ftkGetLastFrame(lib, sn, frame, 200);

  ftkMarker *markerOne = 0, *markerTwo = 0;
  int i, k;
  for (i = 0u; i < frame->markersCount; ++i) {
    if ((mask == BOTH || mask == FOM1) &&
        frame->markers[i].geometryId == M1.geometryId) {
      markerOne = &(frame->markers[i]);
    } else if ((mask == BOTH || mask == FOM1) &&
               frame->markers[i].geometryId == M2.geometryId) {
      markerTwo = &(frame->markers[i]);
    }
  }

  if ((mask == FOM1 && markerOne == 0) || (mask == FOM2 && markerTwo == 0)) {
    std::cout << "At least one marker is missing from data." << std::endl;
    sleep(1000L);
    err = ftkGetLastFrame(lib, sn, frame, 1000);
    return -1;
  }
  Measurement<double> M;
  float err1, err2 = 0;
  if ((mask == FOM1 || mask == BOTH)) {
    err1 = atracsys_transform_to_templated_transform(
        markerOne, atracsys_measurement.F_OM1);
  }
  if ((mask == FOM2 || mask == BOTH)) {
    err2 = atracsys_transform_to_templated_transform(
        markerTwo, atracsys_measurement.F_OM2);
  }
  if (err1 > MAX_REGISTRATION_ERROR) {
    std::cout << "Marker one registration error too high. " << err1 << " > "
              << MAX_REGISTRATION_ERROR << std::endl;
    return -1;
  }
	if (err2 > MAX_REGISTRATION_ERROR) {
    std::cout << "Marker two registration error too high. " << err2 << " > "
              << MAX_REGISTRATION_ERROR << std::endl;
    return -1;
  }
  ftkDeleteFrame(frame);
  return 0;
}
