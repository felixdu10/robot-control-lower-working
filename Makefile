CXX = g++
CXXFLAGS = -g -I ./dependencies/eigen-3.4.0/

POST_FLAGS = -L/usr/lib/ -lgclib -lpthread -lgclibo

# Source files
SRC = inverse_kinematics_test.cpp \
      kinematics_end_effector_analysis_test.cpp \
      Benchtop_Accuracy_test.cpp \
      forward_kinematics.cpp \
      inverse_kinematics.cpp \
      forward_inverse_kinematic_comparison.cpp \
      get_test_grid.cpp \
      parse_results.cpp \
      Parse_home.cpp \
      kinematic_grid_error_analysis.cpp \
      galil_control_test.cpp \
      Matrix.cpp \
      Transform.cpp \
      Home_test.cpp \
      parse_angle.cpp \


# Header files
HEADERS = forward_kinematics.h \
          inverse_kinematics.h \
          kinematic_structs.h \
          Robot.h \
          Point.h \
          NewTransform.h \
          ../scripts/pivot_needle.h \
          ../scripts/pivot_robot.h \
          galil_control_calls.h \

# Object files
OBJ = $(SRC:.cpp=.o)

# Output executables
TARGETS = forward_inverse_kinematic_comparison get_test_grid parse_results#inverse_kinematics_test kinematics_end_effector_analysis_test
TARGETS = forward_inverse_kinematic_comparison get_test_grid kinematics_end_effector_analysis_test kinematic_grid_error_analysis  inverse_kinematics_test galil_control_test parse_results Relative_test Home_test parse_angle

all: $(TARGETS)


inverse_kinematics_test: inverse_kinematics_test.o forward_kinematics.o inverse_kinematics.o
	$(CXX) $(CXXFLAGS) -o $@ $^

kinematics_end_effector_analysis_test: kinematics_end_effector_analysis_test.o forward_kinematics.o inverse_kinematics.o
	$(CXX) $(CXXFLAGS) -o $@ $^

get_test_grid: get_test_grid.o forward_kinematics.o inverse_kinematics.o
	$(CXX) $(CXXFLAGS) -o $@ $^



forward_inverse_kinematic_comparison: forward_inverse_kinematic_comparison.o forward_kinematics.o inverse_kinematics.o
	$(CXX) $(CXXFLAGS) -o $@ $^

parse_results: parse_results.o Matrix.o Transform.o
	$(CXX) $(CXXFLAGS) -o $@ $^

Parse_home: Parse_home.o Matrix.o Transform.o
	$(CXX) $(CXXFLAGS) -o $@ $^

kinematic_grid_error_analysis: kinematic_grid_error_analysis.o forward_kinematics.o inverse_kinematics.o
	$(CXX) $(CXXFLAGS) -o $@ $^

galil_control_test: galil_control_test.o Matrix.o Transform.o inverse_kinematics.o forward_kinematics.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(POST_FLAGS)

Relative_test: Relative_test.o Matrix.o Transform.o Pivot.o forward_kinematics.o inverse_kinematics.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(POST_FLAGS)

Benchtop_Accuracy_test: Benchtop_Accuracy_test.o Matrix.o Transform.o Pivot.o forward_kinematics.o inverse_kinematics.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(POST_FLAGS)

parse_angle: parse_angle.o Matrix.o Transform.o
	$(CXX) $(CXXFLAGS) -o $@ $^

Home_test: Home_test.o Matrix.o Transform.o Pivot.o forward_kinematics.o inverse_kinematics.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(POST_FLAGS)

clean:
	rm -f $(OBJ) $(TARGETS) *.o
