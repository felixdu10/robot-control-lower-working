/**
 * @file orient_ini.cpp
 * @author Tyler Lehrfeld (tlehrfe2@jhu.edu)
 * @brief This file orients a certain marker file (.ini)
 *  such that its rotation frame corresponds with the needle frame
 * @version 0.1
 * @date 2025-04-05
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "ini_helpers.h"
#include <cmath>
#include "../Matrix.h"
#include "../helperFunctions.h"




int main(int argc, char **argv)
{
    std::string filepath = std::string(argv[1]);
    // We want fiducials 2 and 4 to be oriented upwards. Fiducial 2 should be at the origin.
    std::vector<Point> fiducials = parse_ini_file(filepath);
    std::vector<Matrix> Matrix_fiducials;
    //Point rotation_axis_point = {0,0,1};
    //double theta = 0;
    //Matrix R = rotation_from_axis_and_angle(rotation_axis_point, theta); 
    //Point middle = fiducials[1];
    //for (int i = 0; i < fiducials.size(); i++)
    //{
    //    fiducials[i] = fiducials[i] - middle;
    //    Matrix_fiducials.push_back(fiducials[i].to_matrix());
    //}
    //for(int i = 0; i < Matrix_fiducials.size(); i++) {
    //    Matrix_fiducials[i] = R*Matrix_fiducials[i];
    //}
    //put fiducial 2 at origin
    Point middle = fiducials[1];
    for (int i = 0; i < fiducials.size(); i++)
    {
        fiducials[i] = fiducials[i] - middle;
        Matrix_fiducials.push_back(fiducials[i].to_matrix());
    }
    // rotate fiducials such that fiducial 4 has an x and y of zero or (fiducial 4 is above 2).
    Point normalized_4 = fiducials[0].normalize();
    Point rotation_axis_point = cross(normalized_4, {0, 0, 1});
    if(rotation_axis_point.magnitude() == 0) {
        rotation_axis_point = {0, 0, 1};
    }
    double theta = acos(normalized_4 * rotation_axis_point);
    
    Matrix R = rotation_from_axis_and_angle(rotation_axis_point, theta); 
    for(int i = 0; i < Matrix_fiducials.size(); i++) {
        Matrix_fiducials[i] = R*Matrix_fiducials[i];
    }
    //Rotate fiducials around the z axis such that fiducials 1 and 3 are as close as possible to the x-z plane.
    double x1 = Matrix_fiducials[3].matrixArray[0];
    double x2 = Matrix_fiducials[3].matrixArray[1];
    double y1 = Matrix_fiducials[2].matrixArray[0];
    double y2 = Matrix_fiducials[2].matrixArray[1];

    double B = x1 * x2 + y1 * y2;
    double A = x1*x1 - x2*x2 + y1 * y1 - y2 * y2;
    double theta_z = 0.5 * std::atan(-2*B/A) + M_PI/2;
    R = rotation_from_axis_and_angle({0,0,1}, theta_z);
    for(int i = 0; i < Matrix_fiducials.size(); i++) {
        Matrix_fiducials[i] = R*Matrix_fiducials[i];
    }
    std::ofstream file(filepath);
    for (size_t i = 0; i < Matrix_fiducials.size(); ++i) {
        Matrix f = Matrix_fiducials[i];
        double x = (abs(f.matrixArray[0]) < 10e-3 ? 0 : f.matrixArray[0]);
        double y = (abs(f.matrixArray[1]) < 10e-3 ? 0 : f.matrixArray[1]);
        double z = (abs(f.matrixArray[2]) < 10e-3 ? 0 : f.matrixArray[2]);
        
        file << "[fiducial"<< i << "]" << std::endl;
        file << "x=" << x << std::endl << "y=" << y << std::endl << "z=" << z << std::endl;
    }
    file << "[geometry]" <<std::endl << "count=4" << std::endl << "id=" << filepath.substr(8, filepath.length() - std::string("geometry.ini").length()) << std::endl;
}
