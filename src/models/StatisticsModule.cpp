//
// Created by stramek on 20.05.17.
//

#include "include/models/StatisticsModule.h"

StatisticsModule::StatisticsModule() {}

void StatisticsModule::printProgressBar(const int firstCount, const int firstMax, const int secondCount, const int secondMax) {

    float progress = firstCount / (float) firstMax;
    float progress2 = (secondCount / (float) secondMax) + (progress / secondMax);

    int barWidth = 15;

    cout << "Scene: [";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) cout << "=";
        else if (i == pos) cout << ">";
        else cout << " ";
    }
    cout << "] " << int(progress * 100.0) << " %  ";

    cout << "Overall: [";
    int pos2 = barWidth * progress2;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos2) cout << "=";
        else if (i == pos2) cout << ">";
        else cout << " ";
    }
    cout << "] " << int(progress2 * 100.0) << " %    \r";
    cout.flush();
}

void StatisticsModule::appendSceneVoxels(std::vector<std::vector<Eigen::Vector3f>> voxels) {
    scenes.push_back(voxels);
}

Eigen::Vector3f StatisticsModule::getNormalFromPlaneEquation(Eigen::Vector4f &planeEquation){
    return Vector3f(planeEquation(0), planeEquation(1), planeEquation(2));
}

bool StatisticsModule::isNotParallel(Eigen::Vector3f firstNormal, Eigen::Vector3f secondNormal){
    firstNormal.normalize();
    secondNormal.normalize();
    float cosBetweenNormals = firstNormal.dot(secondNormal);
    if(cosBetweenNormals < 0.98 && cosBetweenNormals > -0.98) return true;
    return false;
}

bool StatisticsModule::isMoreThenTwoNonParallelPlanes(){
    for(auto normalA : planesNormals){
        for(auto normalB : planesNormals){
            if(isNotParallel(normalA, normalB)){
                for(auto normalC : planesNormals){
                    if(isNotParallel(normalA, normalC) && isNotParallel(normalC, normalB))
                        return true;
                }
            }
        }
    }
    return false;
}


void StatisticsModule::calculateAndPrint() {

    double areaSum = 0;
    double curvaturesSum = 0;
    long planes = 0;

    std::cout<<"Calculation started..."<<std::endl<<std::endl;
    printProgressBar();
    int sceneNumber = 0;
    for (std::vector<std::vector<Eigen::Vector3f>> voxels : scenes) {
        planesNormals.clear();
        int voxelNo = 0;
        std::vector<std::vector<Eigen::Vector3f>> validVoxels;
        for (std::vector<Eigen::Vector3f> voxel : voxels) {
            MyPlane plane = PlanePca::getPlane(voxel);
            if (plane.isValid() && voxel.size() > MINIMUM_POINTS_VALUE) {
                validVoxels.push_back(voxel);
            }
        }

        for (std::vector<Eigen::Vector3f> voxel : validVoxels) {
            ++voxelNo;
            pcl::PointCloud<pcl::PointXYZ>::Ptr voxelCloud(new pcl::PointCloud<pcl::PointXYZ>());
            for (Eigen::Vector3f point : voxel) {
                voxelCloud.get()->points.push_back(pcl::PointXYZ(point[0], point[1], point[2]));
            }
            double calculatedArea = calculateTotalArea(voxelCloud);
            pair<double, Eigen::Vector4f> pointNormal = calculatePointNormal(voxelCloud);

            // TODO: Do something with "pointNormal.second" -> Eigen::Vector4f
            Eigen::Vector3f normalVec = getNormalFromPlaneEquation(pointNormal.second);
            planesNormals.push_back(normalVec);

            if (!isnan(calculatedArea) && !isnan(pointNormal.first)) {
                areaSum += calculatedArea;
                curvaturesSum += pointNormal.first;
                planes++;
            }
            printProgressBar(voxelNo, (int) validVoxels.size(), sceneNumber, (int) scenes.size());
        }
        if(isMoreThenTwoNonParallelPlanes()) ++numberOfScenes;
        ++sceneNumber;
    }
    planesByScene = planes / scenes.size();
    averageArea = areaSum / planes;
    averageCurvature = curvaturesSum / planes;
    std::cout<<std::endl<<std::endl;
    printOutput();
}

void StatisticsModule::printOutput() {
    cout<<"-----------------------------STATISTICS-----------------------------------"<<endl;
    cout<<"Number of scenes: "<<scenes.size()<<endl;
    cout<<"Planes by scene with minimum number of points of "<<MINIMUM_POINTS_VALUE<<": "<<planesByScene<<endl;
    cout<<"Average area: "<<averageArea<<endl;
    cout<<"Average Curvature: "<<averageCurvature<<endl;
    cout<<"Number of scenes with at least three non parallel planes " << numberOfScenes<<endl;
    cout<<"--------------------------------------------------------------------------"<<endl;
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

pair<double, Eigen::Vector4f> StatisticsModule::calculatePointNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr voxelCloud) {
    Eigen::Vector4f planeParameters;
    float curvature;
    pcl::computePointNormal(*voxelCloud, planeParameters, curvature);
    return pair<double, Eigen::Vector4f>(curvature, planeParameters);
}