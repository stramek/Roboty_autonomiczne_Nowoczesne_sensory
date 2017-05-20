//
// Created by stramek on 20.05.17.
//

#include "include/models/StatisticsModule.h"

StatisticsModule::StatisticsModule() {}

void StatisticsModule::appendSceneVoxels(const std::vector<std::vector<Eigen::Vector3f>> &voxels) {
    scenes.push_back(voxels);
}

void StatisticsModule::calculateAndPrint() {

    double areaSum = 0;
    double curvaturesSum = 0;
    long planes = 0;

    std::cout<<"Calculation started..."<<std::endl;
    for (std::vector<std::vector<Eigen::Vector3f>> voxels : scenes) {
        for (std::vector<Eigen::Vector3f> voxel : voxels) {
            MyPlane plane = PlanePca::getPlane(voxel);
            if (plane.isValid() && voxel.size() > MINIMUM_POINTS_VALUE) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr voxelCloud(new pcl::PointCloud<pcl::PointXYZ>());
                for (Eigen::Vector3f point : voxel) {
                    voxelCloud.get()->points.push_back(pcl::PointXYZ(point[0], point[1], point[2]));
                }
                planes++;
                areaSum += calculateTotalArea(voxelCloud);
                curvaturesSum += calculateCurvatures(voxelCloud);
            }
        }
    }
    planesByScene = planes / scenes.size();
    averageArea = areaSum / planes;
    averageCurvature = curvaturesSum / planes;
    std::cout<<std::endl;
    printOutput();
}

void StatisticsModule::printOutput() {
    cout<<"----------------------------------STATISTICS----------------------------------------"<<endl;
    cout<<"Number of scenes: "<<scenes.size()<<endl;
    cout<<"Planes by scene with minimum number of points of "<<MINIMUM_POINTS_VALUE<<": "<<planesByScene<<endl;
    cout<<"Average area: "<<averageArea<<endl;
    cout<<"Average Curvature: "<<averageCurvature<<endl;
    cout<<"------------------------------------------------------------------------------------"<<endl;
}

double StatisticsModule::calculateTotalArea(pcl::PointCloud<pcl::PointXYZ>::Ptr voxelCloud) {
    pcl::ConvexHull<pcl::PointXYZ> cHull;
    pcl::PointCloud<pcl::PointXYZ> cHull_points;
    cHull.setInputCloud(voxelCloud);
    cHull.setComputeAreaVolume(true);
    cHull.setDimension(3);
    cHull.reconstruct(cHull_points);
    return cHull.getTotalArea();
}

double StatisticsModule::calculateCurvatures(pcl::PointCloud<pcl::PointXYZ>::Ptr voxelCloud) {

    // TODO: CURVATURES LOGIC GOES HERE

    return -1;
}

double StatisticsModule::getPlanesByScene() const {
    return planesByScene;
}

double StatisticsModule::getAverageArea() const {
    return averageArea;
}

double StatisticsModule::getAverageCurvature() const {
    return averageCurvature;
}

void StatisticsModule::clearScenes() {
    scenes.clear();
}
