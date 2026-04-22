#include <vector>
#include <tuple>
#ifndef MATRIX
#define MATRIX

static const double MAX_ERROR = .00001;
/**
 * @brief determines if a value is within a suitable range of error. Used for matrix comparisons
 * 
 * @param a 
 * @param b 
 * @return true 
 * @return false 
 */
static bool isCloseTo(double a, double b) {
    return (a - MAX_ERROR) < b && (a + MAX_ERROR) > b;
}

using std::vector;

/**
 * @brief A matrix class for any math needed
 * 
 */
class Matrix {
public:
    int ROWS;
    int COLUMNS;
    Matrix(int r, int c, vector<double> data);
    Matrix(vector<Matrix> columns);
    Matrix();
    Matrix inverse();
    ~Matrix();
    Matrix operator*(Matrix m2);
    Matrix operator+(Matrix m2);
    Matrix operator*(double scalar);
    Matrix adjunct();
    Matrix transpose();
    Matrix renormalize();
    Matrix cross(Matrix b);
    void switchRow(int row1, int row2);
    double divideRow(int row);
    void divideRow(int row, double divisor);
    double subtractRow(int row1, int row2, int column);
    void subtractRow(int row1, int row2, int column, double multiple);
    friend Matrix operator*(double scalar, Matrix m);
    bool operator==(Matrix b);
    double det();
    double magnitude();
    void print();
    void print_desmos();
    std::tuple<vector<double>, vector<Matrix>> eigenValuesAndVectors();
    double trace();
    vector<double> matrixArray;
};

#endif