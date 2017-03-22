#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

cv::Mat color, depth;
cv::Mat cameraMatrixColor;
cv::Mat lookupX, lookupY;

const double FOCAL_LENGTH_X = 525;
const double FOCAL_LENGTH_Y = 525;
const double OPTICAL_CENTER_X = 319.5;
const double OPTICAL_CENTER_Y = 239.5;

void createLookup(size_t width, size_t height) {
    const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
    const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
    const float cx = cameraMatrixColor.at<double>(0, 2);
    const float cy = cameraMatrixColor.at<double>(1, 2);
    float *it;

    lookupY = cv::Mat(1, height, CV_32F);
    it = lookupY.ptr<float>();
    for (size_t r = 0; r < height; ++r, ++it) {
        *it = (r - cy) * fy;
    }

    lookupX = cv::Mat(1, width, CV_32F);
    it = lookupX.ptr<float>();
    for (size_t c = 0; c < width; ++c, ++it) {
        *it = (c - cx) * fx;
    }
}

void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) {
    const float badPoint = std::numeric_limits<float>::quiet_NaN();

    //#pragma omp parallel for
    for (int r = 0; r < depth.rows; ++r) {
        pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
        const uint16_t *itD = depth.ptr<uint16_t>(r);
        const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
        const float y = lookupY.at<float>(0, r);
        const float *itX = lookupX.ptr<float>();

        for (size_t c = 0; c < (size_t) depth.cols; ++c, ++itP, ++itD, ++itC, ++itX) {
            register const float depthValue = *itD / 1000.0f;
            // Check for invalid measurements
            if (*itD == 0) {
                // not valid
                itP->x = itP->y = itP->z = badPoint;
                itP->rgba = 0;
                continue;
            }
            itP->z = depthValue;
            itP->x = *itX * depthValue;
            itP->y = y * depthValue;
            itP->b = itC->val[0];
            itP->g = itC->val[1];
            itP->r = itC->val[2];
            itP->a = 255;
        }
    }
}

int main(int argc, char **argv) {
    color = imread("../images//rgb//0.png");
    depth = imread("../images//depth//0.png", CV_LOAD_IMAGE_ANYDEPTH);

    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
    cloud->height = color.rows;
    cloud->width = color.cols;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);

    cameraMatrixColor = (Mat_<double>(3, 3) <<
                                            1 / FOCAL_LENGTH_X, 0, -OPTICAL_CENTER_X / FOCAL_LENGTH_X,
                                            0, 1 / FOCAL_LENGTH_Y, -OPTICAL_CENTER_Y / FOCAL_LENGTH_Y,
                                            0, 0, 1);

    createLookup(color.cols, color.rows);

    createCloud(depth, color, cloud);
    const std::string cloudName = "rendered";

    visualizer->addPointCloud(cloud, cloudName);
    visualizer->setBackgroundColor(0, 0, 0);
    visualizer->setShowFPS(true);
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
    visualizer->addCoordinateSystem(1.0);
    visualizer->initCameraParameters();

    while (!visualizer->wasStopped()) {
        visualizer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}
