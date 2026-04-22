//Eigen/Dense: https://github.com/libigl/eigen/tree/master
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "Transform.h"

#ifndef PCLOUDTRANSFORM
#define PCLOUDTRANSFORM

class PointCloudTransform {
public:
    /**
     * @brief Get the centroid
     * 
     * @param Pcloud 
     * @return Matrix: Centroid matrix of a point cloud 
     */
    Matrix getCenter(vector<Matrix> Pcloud) {
        Matrix center(3, 1, {0, 0, 0});
        for(Matrix i : Pcloud) {
            center = center + i;
        }
        center = center * (1 / (double)Pcloud.size());
        return center;
    }

    /**
     * @brief Single Value Decomposition of a Matrix M
     * 
     * @param M 
     * @return vector<Matrix>: returns the U and V^T components of a SVD in a list
     */
    vector<Matrix> SVD(Matrix M) {
        // only for 3x3 matrices
        if(M.COLUMNS != 3 || M.ROWS != 3) {
            throw std::domain_error("SVD only implemented with 3x3 matrices");
        }
        vector<Matrix> USV;
        Eigen::MatrixXd M_Eigen(3, 3);
        M_Eigen << M.matrixArray[0], M.matrixArray[1], M.matrixArray[2],
            M.matrixArray[3], M.matrixArray[4], M.matrixArray[5],
            M.matrixArray[6], M.matrixArray[7], M.matrixArray[8];

        // Compute the SVD
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(
            M_Eigen, Eigen::ComputeThinU | Eigen::ComputeThinV);

        // Get the singular values, U and V matrices
        Eigen::MatrixXd U_Eigen = svd.matrixU();
        Eigen::MatrixXd V_Eigen = svd.matrixV();
        Matrix U(
            3, 3,
            {static_cast<double> (U_Eigen(0, 0)),static_cast<double> (U_Eigen(0, 1)),
             static_cast<double> (U_Eigen(0, 2)), static_cast<double> (U_Eigen(1, 0)),
             static_cast<double> (U_Eigen(1, 1)), static_cast<double> (U_Eigen(1, 2)),
             static_cast<double> (U_Eigen(2, 0)), static_cast<double> (U_Eigen(2, 1)),
             static_cast<double> (U_Eigen(2, 2))});
        Matrix V(
            3, 3,
            {static_cast<double> (V_Eigen(0, 0)), static_cast<double> (V_Eigen(0, 1)),
             static_cast<double> (V_Eigen(0, 2)), static_cast<double> (V_Eigen(1, 0)),
             static_cast<double> (V_Eigen(1, 1)), static_cast<double> (V_Eigen(1, 2)),
             static_cast<double> (V_Eigen(2, 0)), static_cast<double> (V_Eigen(2, 1)),
             static_cast<double> (V_Eigen(2, 2))});
        USV.push_back(U);
        USV.push_back(V.transpose());

        return USV;
    }

    PointCloudTransform() {
    }

    // compute a transformation F_BA
    // using kabsch algorithm: Here is what I used to do this https://en.wikipedia.org/wiki/Kabsch_algorithm
    /**
     * @brief Compute a point cloud registration F_BA between point cloud A and point cloud B.
     * 
     * @param PcloudA 
     * @param PcloudB 
     * @return Transform: F_BA
     */
    Transform compute(vector<Matrix> PcloudA, vector<Matrix> PcloudB) {
        size_t numPoints = PcloudA.size();
        if(numPoints != PcloudB.size()) {
            throw std::invalid_argument(
                "Point clouds must be of equivalent size");
        }

        // get centers of the point clouds and compute new clouds that are on
        // top of each other
        Matrix centroidA = getCenter(PcloudA);
        Matrix centroidB = getCenter(PcloudB);
        vector<Matrix> AdjustedA;
        vector<Matrix> AdjustedB;
        for(size_t i = 0; i < numPoints; i++) {
            AdjustedA.push_back(PcloudA[i] + -1 * centroidA);
            AdjustedB.push_back(PcloudB[i] + -1 * centroidB);
        }
        // construct A and B matrices out of columns
        Matrix A(AdjustedA);
        Matrix B(AdjustedB);
        // F_BA -> F_PQ
        Matrix Q = A.transpose();
        Matrix P = B.transpose();
        Matrix H = P.transpose() * Q;
        vector<Matrix> USV = SVD(H);
        double d = (USV[0] * USV[1]).det();
        if(d <= 0) {
            d = -1;
        }
        Matrix R = USV[0] * Matrix(3, 3, {1, 0, 0, 0, 1, 0, 0, 0, d}) * USV[1]; 
        return Transform(
            R,
            centroidB + -1 * R * centroidA);
    }
};

#endif