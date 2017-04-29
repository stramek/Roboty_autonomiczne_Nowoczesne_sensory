//
// Created by mordimer on 26.02.17.
//

#ifndef PLANEPCA_H
#define PLANEPCA_H

#include "include/utils/constants.h"
#include "include/models/MyPlane.h"

#include <iostream>
#include <Eigen/Dense>
#include <vector>

using namespace Eigen;
using namespace std;


class PlanePca {
private:
    static void pointsVectorToMatrix(const vector<Vector3f> &pointsVector, MatrixXf &matrix);

    static MatrixXf computeCovMatrix(const MatrixXf &matrix);

    static MyPlane computePlane(const vector<Vector3f> &pointsVector);

public:
    static MyPlane getPlane(const vector<Vector3f> &pointsVector);
};


#endif
