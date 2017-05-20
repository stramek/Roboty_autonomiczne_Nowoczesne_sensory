//
// Created by stramek on 20.05.17.
//

#ifndef ROBOTY_AUTONOMICZNE_NOWOCZESNE_SENSORY_STATISTICSMODULE_H
#define ROBOTY_AUTONOMICZNE_NOWOCZESNE_SENSORY_STATISTICSMODULE_H

#include <Eigen/Dense>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include "include/models/PlanePca.h"
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

class StatisticsModule {
public:
    StatisticsModule();
    void appendSceneVoxels(const std::vector<std::vector<Eigen::Vector3f>> &voxels);
    void calculateAndPrint();
    double getPlanesByScene() const;
    double getAverageArea() const;
    double getAverageCurvature() const;
    void clearScenes();
private:
    static const int MINIMUM_POINTS_VALUE = 600;
    std::vector<std::vector<std::vector<Eigen::Vector3f>>> scenes;
    double planesByScene = -1;
    double averageArea = -1;
    double averageCurvature = -1;
    void printOutput();
    double calculateTotalArea(pcl::PointCloud<pcl::PointXYZ>::Ptr voxelCloud);
    double calculateCurvatures(pcl::PointCloud<pcl::PointXYZ>::Ptr vortexCloud);
    void printProgressBar(const int firstCount = 0, const int firstMax = 1, const int secondCount = 0, const int secondMax = 1);
};


#endif //ROBOTY_AUTONOMICZNE_NOWOCZESNE_SENSORY_STATISTICSMODULE_H
