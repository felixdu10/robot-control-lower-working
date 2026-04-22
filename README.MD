# Program Overview
This program includes all of the code used to control the robot, do kinematics and analysis.
# Files
 - Slicer_Extension/CIS2_Extension/load_robot/load_robot.py: main extension file
 - Slicer_Extension/CIS2_Extension/load_robot/Resources/UI/load_robot.ui: extension UI file
 - Slicer_Extension/CIS2_Extension/load_robot/load_robot.py
 - scripts/
   - Makefile: makes executables
   - geometry2000.ini: defines back marker
   - geometry99900000.ini: defines needle marker
   - geometryOriginal.ini: original needle marker before orientation;
   - get_rot.cpp: tests pivot calibration to make sure that the y direcion is forward
   - img.csv: kinematic error analysis based on slider offset data
   - ini_helpers.h: scripts helper function
   - ini_to_desmos.cpp: turn an ini file to desmos points
   - map2.py: generate error heatmap script
   - orient_ini.cpp: rotate needle marker to 
 - 3D_slicer_interface.h: Gets and sends information from and to 3D slicer.
 - 3D_Slicer_Transforms.h and 3D_Slicer_Transforms.cpp: Defines and implements classes related to positioning STL files in 3D slicer.
 - Benchtop_Accuracy_test.cpp: main file for running the experiment and collecting data.
    - Runs a pivot calibration.
    - Homes the robot.
    - Reads in a list of positions to go to.
    - Goes to a set of positions.
    - Takes measurements at each position.
        - Relative frames.
        - expected vs. measured needle position.
        - positional error. (absolute error is meaningless in the most recent results)
 - Home_test.cpp: run home while tracking
 - Kinematics.pdf: a description of the kinematics of the robot, along with the associated error.
 - Makefile: make all the executables.
 - Matrix-test.h: a module that tests the matrix class.
 - Matrix.cpp: a module that implements the matrix class.
 - Matrix.h: a module that defines the matrix class.
 - NewTransform.h: Define the NewTransform class (relative to old one from CIS1).
 - NewTransform_test.cpp: test file for NewTransform.
 - Pivot-test.h: defines testPivot function which random pivots and then checks if the algorithm can make sense of them.
 - Pivot.cpp: implementation for Pivot class.
 - Pivot.h: define the Pivot class.
 - PointCloudTransform.h: implements computing a point cloud registration with SVD.
 - Robot.h: defines a robot class, which holds all the information about robot state based on forawrd and inverse kinematics.
 - Transform-test.h: defines tests for Frame transformations.
 - Transform.cpp: implements frame Transformation functionality.
 - Transform.h: definies the transform class.
 - forward_inverse_kinematic_comparison.cpp: tests whether forward kinematics and inverse kinematics give the same values when reversed.
 - forward_kinematics.cpp: Implements forward kinematic functions.
 - forward_kinematics.h: Defines forward kinematic mathematical functions.
 - galil_control_calls.cpp: Implements functions that send commands to the robot through galil.
 - get_frames_in_output_file.cpp: gets frames of markers and puts them in out.txt.
 - get_test_grid.cpp: gets a test grid of a specified solution. Also outputs slider positions in sliders.txt.
 - helperFunctions.h: a set of helper functions used throughout the program.
 - inverse_kinematics.cpp: Implements inverse kinematic mathematical functions.
 - inverse_kinematics.h: Defines inverse kinematic functions.
 - inverse_kinematics_test.cpp: Tests whether the inverse kinematic code is working correctly.
 - kinematic_grid_error_analysis.cpp: gets the maximum error based on slider diffs at each grid position.
 - kinematic_structs.h: Defines kinematic structs passed into and returned from functions.
 - kinematics_end_effector_analysis_test.cpp: A test script to get certain slider positions under certain conditions and to output kinematic error over distance.
 - kinematics_movement_test.cpp: A test script to move the robot to a certain position.
 - kinematics_test.cpp: Test the forward kinematics.
 - make.sh: a script that makes Benchtop_Accuracy_test executable.
 - restricted_plane_pivot.h: implements the lower pivot to get the robot marker transform.
 - test.cpp: runs all tests.
 - Point.h: Define the point struct and implement its functions.
 - scripts/Makefile: Make ini_to_desmos and orient_ini.
 - scripts/ini_helpers.h: implements a basic ini parser.
 - scripts/ini_to_desmos.cpp: turns an ini file to a desmos coordinate set.
 - scripts/pivot_needle.h: calls pivot calibration on transformlist.
 - scripts/pivot_robot.h: depracated--used to use the plane of the lower linkage to calibrate back marker;
 - scripts/map2.py: generates a heatmap based on error.
 - scripts/orient_ini.cpp: orients a four pronged marker to be aligned with the z-x axis.
 - Parse_home.cpp: parse home results
 - angular_results_pitch.txt, angular_results_yaw.txt: USED TEST DATA
 - Eigen/Dense: in dependencies, there is a folder called eigen-3.4.0. It contains a library that I use for SVD. Here is the link: https://eigen.tuxfamily.org/dox/GettingStarted.html

 # instructions
First ensure that the cmake file in $ATRACSYS_SDK_HOME/samples can target get_frames_in_output_file.cpp exactly like sample5.

```
cd robot_control_code
make
./make.sh
cd scripts
make
cp get_frames_in_output_file.cpp $ATRACSYS_SDK_HOME/samples
cd $ATRACSYS_SDK_HOME/samples
make
./get_frames_in_output_file -g [geometry1.ini] [geometry2.ini]
```
