

#ifndef Plane_h
#define Plane_h
#ifdef __GNUC__
// Avoid tons of warnings with root code
#pragma GCC system_header
#endif

#include "include/models/PlanePca.h"

#include <opencv2/opencv.hpp>
#include <array>
#include <vector>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


using namespace pcl;
using namespace std;
using namespace Eigen;

class CloudPlaneDetector {
public:
    CloudPlaneDetector(PointCloud<PointXYZL> &inputCloud);

    PointCloud<PointXYZRGB>::Ptr getPointCloud();

private:
    vector<PointXYZL> points;
    vector<vector<Vector3f>> voxels;

    struct sortPointsByLabel {
        inline bool operator()(const PointXYZL &point1, const PointXYZL &point2) {
            return (point1.label < point2.label);
        }
    };
};


#endif /* Plane_h */
