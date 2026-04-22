#!/bin/bash

var scripts_dir = "/home/tyler/cis2/robot_control_code/scripts"

read -p "enter the 4 fiducial file path:" needle_marker_file
needle_marker_file="${needle_marker_file:geometry9990000.ini}"

read -p "enter the robot marker file path:" robot_marker_file
robot_marker_file="${robot_marker_file:-/geometry9990000\.ini}"
./orient_ini $robot_marker_file
cp $robot_marker_file $ATRACSYS_SDK_HOME/data
cd ..
cp ./get_frames_in_output_file.cpp $ATRACSYS_SDK_HOME/samples
cd $ATRACSYS_SDK_HOME/samples
make
./get_frames_in_output_file -g $needle_marker_file $robot_marker_file 
cd $scripts_dir/..
./Benchtop_Accuracy_test


