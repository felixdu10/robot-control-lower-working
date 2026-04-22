# Makefile for ini_to_desmos.cpp

PATHS = ../dependencies/eigen-3.4.0 

CXXFLAGS = -std=c++17 -g -I $(PATHS) -I ../

# Project structure
SRC_DIR = .

# Source files
SRCS = ../Matrix.cpp \
	   ../Transform.cpp \
	   ../Pivot.cpp \

       


# Header files
HEADERS = ../Point.h \
          ../NewTransform.h \
		  ../Matrix.h \
		  ../Transform.h \
		  ../ini_helpers.h \
		  ../helperFunctions.h \
		  ../Pivot.h \


# Object files
OBJS = $(SRCS:.cpp=.o)

# Output binary
TARGETS = orient_ini ini_to_desmos get_rot

# Build target
all: $(TARGETS)

orient_ini: orient_ini.o $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^


ini_to_desmos: ini_to_desmos.o $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

get_rot: get_rot.o $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compile source to object
%.o: %.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

	

# Clean up
clean:
	rm -f $(TARGETS) $(OBJS) *.o

.PHONY: all clean
