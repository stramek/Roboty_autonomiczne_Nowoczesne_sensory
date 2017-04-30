//
// Created by stramek on 26.02.17.
//
#ifndef PROJEKTMAGISTERSKI_CONSTANTS_H
#define PROJEKTMAGISTERSKI_CONSTANTS_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace Eigen;
using namespace cv;

const static float PCA_MAX_ACCEPTED_DISTANCE = 0.0002f;

const static float FOCAL_LENGTH_X = 525;
const static float FOCAL_LENGTH_Y = 525;
const static float OPTICAL_CENTER_X = 319.5;
const static float OPTICAL_CENTER_Y = 239.5;

#endif //PROJEKTMAGISTERSKI_CONSTANTS_H