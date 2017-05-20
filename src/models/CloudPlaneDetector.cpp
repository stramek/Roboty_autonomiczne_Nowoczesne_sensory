#include "include/models/CloudPlaneDetector.h"

CloudPlaneDetector::CloudPlaneDetector(pcl::PointCloud<pcl::PointXYZL> &inputCloud) {

    if (inputCloud.points.size() == 0) {
        throw runtime_error("Passed cloud is empty!");
    }

    for (pcl::PointXYZL point : inputCloud) {
        points.push_back(point);
    }

    sort(points.begin(), points.end(), sortPointsByLabel());

    uint32_t lastPoint = inputCloud.at(0).label;
    vector<Vector3f> voxel;
    for (pcl::PointXYZL point : points) {
        if (point.label != lastPoint) {
            voxels.push_back(voxel);
            voxel.clear();
        }
        Vector3f vector3f(point.x, point.y, point.z);
        voxel.push_back(vector3f);

        lastPoint = point.label;
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudPlaneDetector::getPointCloud() {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGB>());


    for (vector<Vector3f> voxel : voxels) {
        MyPlane plane = PlanePca::getPlane(voxel);
        bool isPlane = plane.isValid();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr localCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

        for (Vector3f point : voxel) {
            pcl::PointXYZRGB pointXYZRGB = isPlane ? pcl::PointXYZRGB(0, 255, 0) : pcl::PointXYZRGB(255, 0, 0);
            pointXYZRGB.x = point[0];
            pointXYZRGB.y = point[1];
            pointXYZRGB.z = point[2];
            coloredCloud.get()->points.push_back(pointXYZRGB);
            localCloud.get()->points.push_back(pointXYZRGB);
        }

    }

    return coloredCloud;
}

const vector<vector<Vector3f>> &CloudPlaneDetector::getVoxels() const {
    return voxels;
}
