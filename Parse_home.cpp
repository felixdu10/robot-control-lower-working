#include <fstream>
#include "NewTransform.h"
#include "Matrix.h"


NewTransform get_transform(std::ifstream& file, int num_strs) {
    std::string str = "";
    double num;
    int count = 0;
    while(str == "" || str == " " || count < num_strs) {
        file >> str;
        if(!(str == "" || str == " "))
            count++;
    }
    NewTransform T;
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            file >> num;
            T.matrix[i][j] = num;
        }
        
    }
    return T;
}

Point get_point(std::ifstream& file, int num_strs) {
    Point p;
    std::string str = "";
    double num;
    int count = 0;
    while(str == "" || str == " " || count < num_strs) {
        file >> str;
        if(!(str == "" || str == " "))
            count++;
    }
    file >> p.x;
    file >> p.y;
    file >> p.z;
    return p;
    
}

NewTransform parse_beginning(std::ifstream& file) {
 get_transform(file,1);
 NewTransform T = get_transform(file, 1);
 get_point(file,2);
 return T;
 
}

int main() {
    std::ifstream results("home_results.txt");
    int num_results = 20;
    std::string line;
    NewTransform F_OM1 = parse_beginning(results);
    for(int i = 0; i < num_results; i++) {
        NewTransform F_OM1 = get_transform(results, 1);
        Point difference_vec = get_point(results, 2);
        double diff_mag;
        results >> line;
        results >>line;
        results >> diff_mag;
        std::cout << difference_vec.to_string(true);
        //std::cout << diff_mag << ";"<< std::endl;
    }
}