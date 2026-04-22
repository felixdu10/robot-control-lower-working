cmake_minimum_required(VERSION 3.18)
project(kinematics_project LANGUAGES CXX)

# Set C++ standard and flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(project_dir /${CMAKE_CURRENT_SOURCE_DIR}) # Include Eigen
include_directories(${project_dir}/dependencies/eigen-3.4.0)
include_directories(${project_dir}/dependencies)
include_directories(${project_dir}/dependencies/eigen-3.4.0/Eigen)
include_directories(${project_dir}/kinematic_calibration)
include_directories(${project_dir}/kinematic_calibration/templated_classes/auto_diff_class)
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
# Add optional linking libraries
set(POST_LIBS gclib pthread gclibo)

# Common source files reused across targets
set(SRC_SHARED
    forward_kinematics.cpp
    inverse_kinematics.cpp
    Matrix.cpp
    Transform.cpp
)
set(SRC_SIMULATION
        ${project_dir}/kinematic_calibration/templated_classes/Templated_Point.h
        ${project_dir}/kinematic_calibration/templated_classes/Templated_Transform.h
        ${project_dir}/kinematic_calibration/jacobian.h
        ${project_dir}/kinematic_calibration/templated_classes/auto_diff_class/auto.h
        ${project_dir}/kinematic_calibration/atracsys_functions.h
	inverse_kinematics.cpp
	Matrix.cpp
	Transform.cpp
)

# If Pivot.cpp is needed
set(SRC_PIVOT
    Pivot.cpp
)

# --- Kinematics core library (for external projects) ---
add_library(kinematics_core STATIC
    ${SRC_SIMULATION}
    ${SRC_SHARED}
    ${SRC_PIVOT}
    ${api_SRCS}
    ${additional_SRCS}
)

# Include paths required by this library
target_include_directories(kinematics_core PUBLIC
    ${project_dir}
    ${project_dir}/kinematic_calibration
    ${project_dir}/kinematic_calibration/templated_classes
    ${project_dir}/kinematic_calibration/templated_classes/auto_diff_class
    ${project_dir}/dependencies/eigen-3.4.0
    ${project_dir}/dependencies/eigen-3.4.0/Eigen
    ${ATRACSYS_INCLUDE_DIR}
)

# Link dependencies needed by the core functionality
target_link_libraries(kinematics_core PUBLIC
    ceres
    Atracsys::AdvancedAPI
    ${POST_LIBS}
)
# Targets

add_executable(test_auto
    ${project_dir}/kinematic_calibration/test/test_auto.cpp
    ${SRC_SIMULATION}
)

add_executable(inverse_kinematics_test
    inverse_kinematics_test.cpp
    ${SRC_SHARED}
)

add_executable(kinematics_test
    kinematics_test.cpp
    ${SRC_SHARED}
)

add_executable(kinematics_end_effector_analysis_test
    kinematics_end_effector_analysis_test.cpp
    ${SRC_SHARED}
)

add_executable(get_test_grid
    get_test_grid.cpp
    ${SRC_SHARED}
)

add_executable(forward_inverse_kinematic_comparison
    forward_inverse_kinematic_comparison.cpp
    ${SRC_SHARED}
)

add_executable(parse_results
    parse_results.cpp
    Matrix.cpp
    Transform.cpp
)

add_executable(Parse_home
    Parse_home.cpp
    Matrix.cpp
    Transform.cpp
)

add_executable(kinematic_grid_error_analysis
    kinematic_grid_error_analysis.cpp
    ${SRC_SHARED}
)

add_executable(galil_control_test
    galil_control_test.cpp
    ${SRC_SHARED}
)
target_link_libraries(galil_control_test PRIVATE ${POST_LIBS})

add_executable(Relative_test
    Relative_test.cpp
    ${SRC_SHARED}
    ${SRC_PIVOT}
)
target_link_libraries(Relative_test PRIVATE ${POST_LIBS})

add_executable(Benchtop_Accuracy_test
    Benchtop_Accuracy_test.cpp
    ${SRC_SHARED}
    ${SRC_PIVOT}
)
target_link_libraries(Benchtop_Accuracy_test PRIVATE ${POST_LIBS})

add_executable(pivot
    ${project_dir}/kinematic_calibration/pivot_calibration.cpp
    ${SRC_SHARED}
    ${SRC_PIVOT}
    ${SRC_SIMULATION}
)
target_link_libraries(pivot PRIVATE ${POST_LIBS})


add_executable(parse_angle
    parse_angle.cpp
    Matrix.cpp
    Transform.cpp
)

add_executable(Home_test
    Home_test.cpp
    ${SRC_SHARED}
    ${SRC_PIVOT}
)
target_link_libraries(Home_test PRIVATE ${POST_LIBS})

add_executable(simulation
    ${project_dir}/kinematic_calibration/simulation.cpp
    ${SRC_SIMULATION}
    ${SRC_SHARED}
    ${SRC_PIVOT}
    ${api_SRCS}
    ${additional_SRCS}
)

target_link_libraries(simulation PRIVATE ${POST_LIBS} ceres Atracsys::AdvancedAPI)

add_executable(test_forward_kinematics
        ${project_dir}/kinematic_calibration/test/test_forward_kinematics.cpp
	${SRC_SIMULATION}
	${SRC_SHARED}
)

target_link_libraries(test_forward_kinematics ceres)

add_executable(test_inverse_kinematics
        ${project_dir}/kinematic_calibration/test/test_inverse_kinematics.cpp
	${SRC_SIMULATION}
	${SRC_SHARED}
)

target_link_libraries(test_inverse_kinematics ceres)



add_executable(data_collection
    ${project_dir}/kinematic_calibration/data_collection.cpp
    ${SRC_SIMULATION}
    ${SRC_SHARED}
    ${SRC_PIVOT}
    ${api_SRCS}
    ${additional_SRCS}
)

target_link_libraries(data_collection PRIVATE ${POST_LIBS} ceres Atracsys::AdvancedAPI)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
add_executable(z_graph
    ${project_dir}/kinematic_calibration/z_graph.cpp
    ${SRC_SIMULATION}
    ${SRC_SHARED}
    ${SRC_PIVOT}
    ${api_SRCS}
    ${additional_SRCS}
)

target_link_libraries(z_graph ${POST_LIBS} ceres Atracsys::AdvancedAPI)

ament_target_dependencies(z_graph
  rclcpp
  rclcpp_action
  std_msgs
  action_msgs
  my_robot_interfaces
)

add_executable(precision
    ${project_dir}/kinematic_calibration/precision.cpp
    ${SRC_SIMULATION}
    ${SRC_SHARED}
    ${SRC_PIVOT}
    ${api_SRCS}
    ${additional_SRCS}
)

target_link_libraries(precision PRIVATE ${POST_LIBS} ceres Atracsys::AdvancedAPI)
add_executable(absolute_accuracy
    ${project_dir}/kinematic_calibration/absolute_accuracy.cpp
    ${SRC_SIMULATION}
    ${SRC_SHARED}
    ${SRC_PIVOT}
    ${api_SRCS}
    ${additional_SRCS}
)

target_link_libraries(absolute_accuracy PRIVATE ${POST_LIBS} ceres Atracsys::AdvancedAPI)

add_executable(read_csv
    ${project_dir}/kinematic_calibration/read_csv.cpp
    ${SRC_SIMULATION}
)


install(TARGETS kinematics_core EXPORT kinematicsTargets)
install(EXPORT kinematicsTargets FILE kinematicsTargets.cmake NAMESPACE kinematics:: DESTINATION lib/cmake/kinematics)
install(DIRECTORY kinematic_calibration/ DESTINATION include/kinematic_calibration)
