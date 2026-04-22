/**
 * @file 3D_slicer_interface.h
 * @author Tyler Lehrfeld
 * @brief This file is the file that communicates with 3D slicer to move the stl files according to their kinematically determined transforms.
 * @version 0.1
 * @date 2025-03-08
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef SLICER_INTERFACE
#define SLICER_INTERFACE

#include "kinematic_structs.h"


inline approach_definition get_approach_from_3D_slicer_UI() {
    //TODO
    return {.target = {.x=0,.y=0,.z=0}, .theta = 0, .phi = 0};
};

#endif