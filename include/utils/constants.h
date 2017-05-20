//
// Created by stramek on 26.02.17.
//
#ifndef PROJEKTMAGISTERSKI_CONSTANTS_H
#define PROJEKTMAGISTERSKI_CONSTANTS_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace Eigen;
//using namespace cv;

const static float PCA_MAX_ACCEPTED_DISTANCE = 0.0002f;

const static float FOCAL_LENGTH_X = 525;
const static float FOCAL_LENGTH_Y = 525;
const static float OPTICAL_CENTER_X = 319.5;
const static float OPTICAL_CENTER_Y = 239.5;

// LCCPSegmentation Stuff
const static float CONCAVITY_TOLERANCE_THRESHOLD = 10;
const static float SMOOTHNESS_THRESHOLD = 0.1;
const static uint32_t MIN_SEGMENT_SIZE = 5;
const static bool USE_EXTENDED_CONVEXITY = true;
const static bool USE_SANITY_CRITERION = true;
const static unsigned int K_FACTOR = 0; // or change to 1

#endif //PROJEKTMAGISTERSKI_CONSTANTS_H