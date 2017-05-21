#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/lccp_segmentation.h>
#include "include/models/StatisticsModule.h"

#include "../../include/utils/edited_lccp_segmentation.h"
#include "../../src/utils/edited_lccp_segmentation.cpp"

//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>

#include <opencv2/opencv.hpp>
#include <include/models/CloudPlaneDetector.h>
#include <pcl/io/ply_io.h>

using namespace std;
using namespace cv;

// Types
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

cv::Mat color, depth;
cv::Mat lookupX, lookupY;
StatisticsModule statisticsModule;

const Eigen::Matrix<double, 3, 3> PHCP_MODEL = [] {
    Eigen::Matrix<double, 3, 3> matrix;
    matrix << 1 / FOCAL_LENGTH_X, 0, -OPTICAL_CENTER_X / FOCAL_LENGTH_X,
            0, 1 / FOCAL_LENGTH_Y, -OPTICAL_CENTER_Y / FOCAL_LENGTH_Y,
            0, 0, 1;
    return matrix;
}();

// use single camera transform

float voxel_resolution = 0.0075f;

float seed_resolution = 0.03f;

float color_importance = 1.0f;

float spatial_importance = 1.0f;

float normal_importance = 4.0f;


void getPoint(unsigned int u, unsigned int v, float depth, Eigen::Vector3d &point3D) {
    Eigen::Vector3d point(u, v, 1);
    point3D = depth * PHCP_MODEL * point;
}

void createLookup(size_t width, size_t height) {
    float *it;

    lookupY = cv::Mat(1, height, CV_32F);
    it = lookupY.ptr<float>();
    for (size_t r = 0; r < height; ++r, ++it) {
        *it = (r - OPTICAL_CENTER_Y) * FOCAL_LENGTH_Y;
    }

    lookupX = cv::Mat(1, width, CV_32F);
    it = lookupX.ptr<float>();
    for (size_t c = 0; c < width; ++c, ++it) {
        *it = (c - OPTICAL_CENTER_X) * FOCAL_LENGTH_X;
    }
}

void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) {
    const float badPoint = std::numeric_limits<float>::quiet_NaN();
    Eigen::Vector3d point;
    //#pragma omp parallel for
    for (int r = 0; r < depth.rows; ++r) {
        pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
        const uint16_t *itD = depth.ptr<uint16_t>(r);
        const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
        const float y = lookupY.at<float>(0, r);
        const float *itX = lookupX.ptr<float>();

        for (size_t c = 0; c < (size_t) depth.cols; ++c, ++itP, ++itD, ++itC, ++itX) {
            register const float depthValue = *itD /5000.0f;
            getPoint(c, r, depthValue, point);
            // Check for invalid measurements
            if (*itD == 0) {
                // not valid
                itP->x = itP->y = itP->z = badPoint;
                itP->rgba = 0;
                continue;
            }
            itP->z = point(2);
            itP->x = -point(0);
            itP->y = -point(1);
            itP->b = itC->val[0];
            itP->g = itC->val[1];
            itP->r = itC->val[2];
            itP->a = 255;
        }
    }
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr createEmptyCloud(const Mat &color, const Mat &depth) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    cloud->height = color.rows;
    cloud->width = color.cols;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);
    return cloud;
}

pcl::SupervoxelClustering<PointT> getSuperVoxelClustering(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {
    pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution);
    super.setUseSingleCameraTransform (false);
    super.setInputCloud (cloud);
    super.setColorImportance (color_importance);
    super.setSpatialImportance (spatial_importance);
    super.setNormalImportance (normal_importance);
    return super;
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> createAndSetupVisualizer(string windowName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer (windowName));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud(cloud, "maincloud");
    return viewer;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> createAndSetupVisualizer(string windowName, pcl::PointCloud<pcl::PointXYZL>::Ptr cloud) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer (windowName));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud(cloud, "maincloud");
    return viewer;
}

void relabelCloud(map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxelClusters,
                  multimap<uint32_t, uint32_t> supervoxelAdjecency,
                  pcl::PointCloud<pcl::PointXYZL>::Ptr cloudToRelabel) {
    PCL_INFO ("Starting Segmentation\n");
    pcl_edited::LCCPSegmentation<PointT> lccp;
    lccp.setConcavityToleranceThreshold (CONCAVITY_TOLERANCE_THRESHOLD);
    lccp.setSanityCheck (USE_SANITY_CRITERION);
    lccp.setSmoothnessCheck (true, voxel_resolution, seed_resolution, SMOOTHNESS_THRESHOLD);
    lccp.setKFactor (K_FACTOR);
    lccp.setInputSupervoxels (supervoxelClusters, supervoxelAdjecency);
    lccp.setMinSegmentSize (MIN_SEGMENT_SIZE);
    lccp.segment();
    lccp.relabelCloud(*cloudToRelabel);
}

multimap<uint32_t, uint32_t> getSupervoxelAdjacency(pcl::SupervoxelClustering<PointT> super, map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters) {
    pcl::console::print_highlight ("Getting supervoxel adjacency\n");
    multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency (supervoxel_adjacency);
    multimap<uint32_t, uint32_t>::iterator label_itr = supervoxel_adjacency.begin();
    for (; label_itr != supervoxel_adjacency.end();) {
        //First get the label
        uint32_t supervoxel_label = label_itr->first;
        //Now get the supervoxel corresponding to the label
        pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);

        //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
        PointCloudT adjacent_supervoxel_centers;
        multimap<uint32_t,uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first;
        for ( ; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr) {
            pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at (adjacent_itr->second);
            adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
        }
        label_itr = supervoxel_adjacency.upper_bound(supervoxel_label);
    }
    return supervoxel_adjacency;
}

int main(int argc, char **argv) {

    ///////

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PLYReader Reader;
    Reader.read("../cloud_merged.ply", *cloudRGB);

    pcl::PointCloud<pcl::PointXYZRGBA> cloudRGBA;
    pcl::PointCloud<pcl::PointXYZRGB> cloudTmp = *cloudRGB;
    pcl::copyPointCloud (cloudTmp, cloudRGBA);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(&cloudRGBA);

    pcl::SupervoxelClustering<PointT> super = getSuperVoxelClustering(cloud);


    ////////////

    map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;
    super.extract(supervoxel_clusters);

    multimap<uint32_t, uint32_t> supervoxel_adjacency = getSupervoxelAdjacency(super, supervoxel_clusters);

    pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
    pcl::PointCloud<pcl::PointXYZL>::Ptr lccpLabeledCloud = sv_labeled_cloud->makeShared();
    relabelCloud(supervoxel_clusters, supervoxel_adjacency, lccpLabeledCloud);

    CloudPlaneDetector cloudPlaneDetector(*lccpLabeledCloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelsCloud = cloudPlaneDetector.getPointCloud();

    boost::shared_ptr<pcl::visualization::PCLVisualizer> planeViewer = createAndSetupVisualizer("Planes", lccpLabeledCloud);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> voxelViewer = createAndSetupVisualizer("Voxels", voxelsCloud);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizerReadCloud = createAndSetupVisualizer("Cloud Viewer", cloudRGB);

    while (!planeViewer->wasStopped() || !voxelViewer->wasStopped()) {
        planeViewer->spinOnce(100);
        voxelViewer->spinOnce(100);
    }
    return 0;
}