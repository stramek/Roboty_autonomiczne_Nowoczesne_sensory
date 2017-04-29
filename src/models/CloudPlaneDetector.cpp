#include "include/models/CloudPlaneDetector.h"

CloudPlaneDetector::CloudPlaneDetector(PointCloud<PointXYZL> &inputCloud) {

    if (inputCloud.points.size() == 0) {
        throw runtime_error("Passed cloud is empty!");
    }

    for (PointXYZL point : inputCloud) {
        points.push_back(point);
    }

    sort(points.begin(), points.end(), sortPointsByLabel());

    uint32_t lastPoint = inputCloud.at(0).label;
    vector<Vector3f> voxel;
    for (PointXYZL point : points) {
        if (point.label != lastPoint) {
            voxels.push_back(voxel);
            voxel.clear();
        }
        Vector3f vector3f(point.x, point.y, point.z);
        voxel.push_back(vector3f);

        lastPoint = point.label;
    }
}

uint8_t getCC(int tagValue) {
    return (uint8_t) (tagValue % 255);
}

PointCloud<PointXYZRGB>::Ptr CloudPlaneDetector::getPointCloud() {

    PointCloud<PointXYZRGB>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGB>());


    for (vector<Vector3f> voxel : voxels) {
        MyPlane plane = PlanePca::getPlane(voxel);



        bool isPlane = plane.isValid();
        for (Vector3f point : voxel) {
            PointXYZRGB pointXYZRGB = isPlane ? PointXYZRGB(0, 255, 0) : PointXYZRGB(255, 0, 0);
            pointXYZRGB.x = point[0];
            pointXYZRGB.y = point[1];
            pointXYZRGB.z = point[2];
            coloredCloud.get()->points.push_back(pointXYZRGB);
        }
    }

    return coloredCloud;
}
