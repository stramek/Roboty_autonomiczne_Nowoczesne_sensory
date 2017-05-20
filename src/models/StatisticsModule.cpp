//
// Created by stramek on 20.05.17.
//

#include "include/models/StatisticsModule.h"

StatisticsModule::StatisticsModule() {}

void StatisticsModule::printProgressBar(const int firstCount, const int firstMax, const int secondCount, const int secondMax) {

    float progress = firstCount / (float) firstMax;
    float progress2 = secondCount / (float) secondMax;

    int barWidth = 40;

    cout << "voxels: [";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) cout << "=";
        else if (i == pos) cout << ">";
        else cout << " ";
    }
    cout << "] " << int(progress * 100.0) << " %  ";

    cout << "scenes: [";
    int pos2 = barWidth * progress2;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos2) cout << "=";
        else if (i == pos2) cout << ">";
        else cout << " ";
    }
    cout << "] " << int(progress2 * 100.0) << " %\r";

    cout.flush();
}

void StatisticsModule::appendSceneVoxels(const std::vector<std::vector<Eigen::Vector3f>> &voxels) {
    scenes.push_back(voxels);
}

void StatisticsModule::calculateAndPrint() {

    double areaSum = 0;
    double curvaturesSum = 0;
    long planes = 0;

    std::cout<<"Calculation started..."<<std::endl<<std::endl;
    printProgressBar();
    int sceneNumber = 0;
    for (std::vector<std::vector<Eigen::Vector3f>> voxels : scenes) {
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
            double calculatedCuravture = calculateCurvatures(voxelCloud);
            if (!isnan(calculatedArea) && !isnan(calculatedCuravture)) {
                areaSum += calculatedArea;
                curvaturesSum += calculatedCuravture;
                planes++;
            }
            printProgressBar(voxelNo, (int) validVoxels.size(), sceneNumber, (int) scenes.size());
        }
        ++sceneNumber;
    }
    planesByScene = planes / scenes.size();
    averageArea = areaSum / planes;
    averageCurvature = curvaturesSum / planes;
    printProgressBar(1, 1, sceneNumber, (int) scenes.size());
    std::cout<<std::endl<<std::endl;
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

    // Compute the normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud (voxelCloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    normal_estimation.setSearchMethod (tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::Normal>);

    normal_estimation.setRadiusSearch (0.03);

    normal_estimation.compute (*cloud_with_normals);

    // Setup the principal curvatures computation
    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principal_curvatures_estimation;

    // Provide the original point cloud (without normals)
    principal_curvatures_estimation.setInputCloud (voxelCloud);

    // Provide the point cloud with normals
    principal_curvatures_estimation.setInputNormals (cloud_with_normals);

    // Use the same KdTree from the normal estimation
    principal_curvatures_estimation.setSearchMethod (tree);
    principal_curvatures_estimation.setRadiusSearch (0.5);

    // Actually compute the principal curvatures
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
    principal_curvatures_estimation.compute (*principal_curvatures);

    pcl::PrincipalCurvatures descriptor = principal_curvatures->points[0];
    cout<<"Wartość: "<<descriptor.pc1<<endl;
    return descriptor.pc1;
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
