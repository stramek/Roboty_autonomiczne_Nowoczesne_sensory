

#ifndef Plane_h
#define Plane_h
#ifdef __GNUC__
// Avoid tons of warnings with root code
#pragma GCC system_header
#endif

#include "include/models/PlanePca.h"

#include <array>
#include <vector>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "include/models/StatisticsModule.h"
#include <opencv2/opencv.hpp>

//using namespace pcl;
//using namespace std;
//using namespace Eigen;

class CloudPlaneDetector {
public:
    CloudPlaneDetector(pcl::PointCloud<pcl::PointXYZL> &inputCloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloud();

    const vector<vector<Vector3f>> &getVoxels() const;

private:
    vector<pcl::PointXYZL> points;
    vector<vector<Vector3f>> voxels;

    struct sortPointsByLabel {
        inline bool operator()(const pcl::PointXYZL &point1, const pcl::PointXYZL &point2) {
            return (point1.label < point2.label);
        }
    };
};


#endif /* Plane_h */
