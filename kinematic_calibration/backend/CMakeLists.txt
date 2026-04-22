set(robot_control_code_dir /home/tyler/projects/robot_control_code/)

include_directories(robot_control_code_dir)
include_directories(${robot_control_code_dir}/kinematic_calibration)

include_directories(${robot_control_code_dir}/dependencies/eigen-3.4.0)
include_directories(${robot_control_code_dir}/dependencies)
include_directories(${robot_control_code_dir}/dependencies/eigen-3.4.0/Eigen)
include_directories(${robot_control_code_dir}/kinematic_calibration)
include_directories(${robot_control_code_dir}/kinematic_calibration/templated_classes/auto_diff_class)
find_package(Ceres REQUIRED)
# Atracsys SDK paths
set(ATRACSYS_INCLUDE_DIR /home/tyler/Downloads/fusionTrack_x64_public_v4.10.1/fusionTrack_SDK-v4.10.1-linux64/include)
set(ATRACSYS_SDK_INSTALL_PATH ${ATRACSYS_INCLUDE_DIR}/..)
#set(ATRACSYS_LIB_DIR /home/tyler/Downloads/fusionTrack_x64_public_v4.10.1/fusionTrack_SDK-v4.10.1-linux64/lib)
# Atracsys SDK libraries

# Atracsys SDK API This will be only used to display the headers in Visual Studio
set(api_SRCS
    "${ATRACSYS_SDK_INSTALL_PATH}/include/ftkErrors.h" "${ATRACSYS_SDK_INSTALL_PATH}/include/ftkTypes.h"
    "${ATRACSYS_SDK_INSTALL_PATH}/include/ftkErrors.h" "${ATRACSYS_SDK_INSTALL_PATH}/include/ftkInterface.h"
    "${ATRACSYS_SDK_INSTALL_PATH}/include/ftkPlatform.h" "${ATRACSYS_SDK_INSTALL_PATH}/include/ftkOptions.h")

# Helpers dedicated to the samples
set(additional_SRCS
    "${ATRACSYS_SDK_INSTALL_PATH}/samples/helpers.cpp" "${ATRACSYS_SDK_INSTALL_PATH}/samples/helpers.hpp"
    "${ATRACSYS_SDK_INSTALL_PATH}/samples/geometryHelper.hpp"
    "${ATRACSYS_SDK_INSTALL_PATH}/samples/geometryHelper.cpp")

set(Atracsys_DIR /home/tyler/Downloads/fusionTrack_x64_public_v4.10.1/fusionTrack_SDK-v4.10.1-linux64/cmake/Atracsys/)
set(ATRACSYS_LIBS fusionTrack64 calibrationClient64 device64 AtracsysDataExchangeLib64 atnetLib64)
include_directories(${ATRACSYS_INCLUDE_DIR})
include_directories(/home/tyler/Downloads/fusionTrack_x64_public_v4.10.1/fusionTrack_SDK-v4.10.1-linux64/samples)
link_directories(${ATRACSYS_LIB_DIR})
set(CMAKE_BUILD_RPATH ${ATRACSYS_LIB_DIR})
find_package(Atracsys REQUIRED COMPONENTS SDK AdvancedAPI)

