#!/bin/bash

g++ -g -I ./dependencies/eigen-3.4.0/ -c -o Benchtop_Accuracy_test.o Benchtop_Accuracy_test.cpp 
g++ -g -c -o parse_results.o parse_results.cpp
g++ -g -I ./dependencies/eigen-3.4.0/ -c -o inverse_kinematics.o inverse_kinematics.cpp
g++ -g -I ./dependencies/eigen-3.4.0/ -c -o forward_kinematics.o forward_kinematics.cpp 
g++ -g -I ./dependencies/eigen-3.4.0/ -c -o Pivot.o Pivot.cpp
g++ -g -I ./dependencies/eigen-3.4.0/ -c -o Matrix.o Matrix.cpp
g++ -g -I ./dependencies/eigen-3.4.0/ -c -o Transform.o Transform.cpp
g++ -g -I ./dependencies/eigen-3.4.0/  Benchtop_Accuracy_test.o forward_kinematics.o inverse_kinematics.o Matrix.o Transform.o Pivot.o -o Benchtop_Accuracy_test -L/usr/lib/ -lgclib -lpthread -lgclibo
g++ -g parse_results.o Matrix.o Transform.o -o parse_results