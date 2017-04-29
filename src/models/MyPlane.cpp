//
//  MyPlane.cpp
//  ProjektMagisterski
//
//  Created by Marcin Stramowski on 11.02.2017.
//
//

#include "include/models/MyPlane.h"

MyPlane::MyPlane() {}

MyPlane::MyPlane(Eigen::Vector3d point1, Eigen::Vector3d point2, Eigen::Vector3d point3) {

}

MyPlane::MyPlane(std::array<Eigen::Vector3d, 3>) {

}

MyPlane::MyPlane(Eigen::Vector3f normalVec, Eigen::Vector3f point) {
    A = normalVec(0);
    B = normalVec(1);
    C = normalVec(2);
    D = A * point(0) + B * point(1) + C * point(2);
    valid = true;
}

double MyPlane::getA() {
    return A;
}

double MyPlane::getB() {
    return B;
}

double MyPlane::getC() {
    return C;
}

double MyPlane::getD() {
    return D;
}

double MyPlane::getDistanceFromPoint(Eigen::Vector3d point) {
    return std::abs(A * point(0) + B * point(1) + C * point(2) + D)
           / sqrt(pow(A, 2) + pow(B, 2) + pow(C, 2));
}

void MyPlane::computePlaneEquation(Eigen::Vector3d point1, Eigen::Vector3d point2, Eigen::Vector3d point3) {
    Eigen::Vector3d v = point1 - point2;
    Eigen::Vector3d w = point1 - point3;
    Eigen::Vector3d planeParameters = v.cross(w);
    A = planeParameters(0);
    B = planeParameters(1);
    C = planeParameters(2);
    D = A * point1(0) + B * point1(1) + C * point1(2);
    valid = true;
}

bool MyPlane::isValid() const {
    return valid;
}