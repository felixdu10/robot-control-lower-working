#include <iostream>
#include <string>
#include <vector>
#include <regex>
#include "../Point.h"
#include "../NewTransform.h"
#include "ini_helpers.h"


int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: ./parse_ini path_to_ini_file.ini" << std::endl;
        return 1;
    }
    
    std::string filepath = std::string(argv[1]);
    std::vector<Point> fiducials = parse_ini_file(filepath);

    for (size_t i = 0; i < fiducials.size(); ++i) {
        Point f = fiducials[i];
        std::cout << "(" << f.x << ", " << f.y << ", " << f.z << ")\n";
    }

    return 0;
}
