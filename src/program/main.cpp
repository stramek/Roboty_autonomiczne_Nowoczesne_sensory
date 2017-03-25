#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>

//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// Types
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

cv::Mat color, depth;
cv::Mat cameraMatrixColor;
cv::Mat lookupX, lookupY;

const float FOCAL_LENGTH_X = 525;
const float FOCAL_LENGTH_Y = 525;
const float OPTICAL_CENTER_X = 319.5;
const float OPTICAL_CENTER_Y = 239.5;

const Eigen::Matrix<double, 3, 3> PHCP_MODEL = [] {
    Eigen::Matrix<double, 3, 3> matrix;
    matrix << 1 / FOCAL_LENGTH_X, 0, -OPTICAL_CENTER_X / FOCAL_LENGTH_X,
            0, 1 / FOCAL_LENGTH_Y, -OPTICAL_CENTER_Y / FOCAL_LENGTH_Y,
            0, 0, 1;
    return matrix;
}();

float voxel_resolution = 0.008f;

float seed_resolution = 0.1f;

float color_importance = 0.2f;

float spatial_importance = 0.4f;

float normal_importance = 1.0f;


void getPoint(unsigned int u, unsigned int v, float depth, Eigen::Vector3d &point3D) {
    Eigen::Vector3d point(u, v, 1);
    point3D = depth * PHCP_MODEL * point;
}

void createLookup(size_t width, size_t height) {
    const float fx = 1.0f / cameraMatrixColor.at<float>(0, 0);
    const float fy = 1.0f / cameraMatrixColor.at<float>(1, 1);
    const float cx = cameraMatrixColor.at<float>(0, 2);
    const float cy = cameraMatrixColor.at<float>(1, 2);
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

void
addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
                                  PointCloudT &adjacent_supervoxel_centers,
                                  std::string supervoxel_name,
                                  boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
    vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();

    //Iterate through all adjacent points, and add a center point to adjacent point pair
    PointCloudT::iterator adjacent_itr = adjacent_supervoxel_centers.begin ();
    for ( ; adjacent_itr != adjacent_supervoxel_centers.end (); ++adjacent_itr)
    {
        points->InsertNextPoint (supervoxel_center.data);
        points->InsertNextPoint (adjacent_itr->data);
    }
    // Create a polydata to store everything in
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
    // Add the points to the dataset
    polyData->SetPoints (points);
    polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
    for(unsigned int i = 0; i < points->GetNumberOfPoints (); i++)
        polyLine->GetPointIds ()->SetId (i,i);
    cells->InsertNextCell (polyLine);
    // Add the lines to the dataset
    polyData->SetLines (cells);
    viewer->addModelFromPolyData (polyData,supervoxel_name);
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

    //////////////////////////////  //////////////////////////////
    ////// This is how to use supervoxels
    //////////////////////////////  //////////////////////////////

    pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution);
    super.setUseSingleCameraTransform (false);
    super.setInputCloud (cloud);
    super.setColorImportance (color_importance);
    super.setSpatialImportance (spatial_importance);
    super.setNormalImportance (normal_importance);

    std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;

    pcl::console::print_highlight ("Extracting supervoxels!\n");
    super.extract (supervoxel_clusters);
    pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);

    PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud ();
    viewer->addPointCloud (voxel_centroid_cloud, "voxel centroids");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2.0, "voxel centroids");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.95, "voxel centroids");

    PointLCloudT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud ();
    viewer->addPointCloud (labeled_voxel_cloud, "labeled voxels");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "labeled voxels");

    PointNCloudT::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);
    //We have this disabled so graph is easy to see, uncomment to see supervoxel normals
    //viewer->addPointCloudNormals<PointNormal> (sv_normal_cloud,1,0.05f, "supervoxel_normals");

    pcl::console::print_highlight ("Getting supervoxel adjacency\n");
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency (supervoxel_adjacency);
    //To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
    std::multimap<uint32_t,uint32_t>::iterator label_itr = supervoxel_adjacency.begin ();
    for ( ; label_itr != supervoxel_adjacency.end (); )
    {
        //First get the label
        uint32_t supervoxel_label = label_itr->first;
        //Now get the supervoxel corresponding to the label
        pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);

        //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
        PointCloudT adjacent_supervoxel_centers;
        std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first;
        for ( ; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
        {
            pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at (adjacent_itr->second);
            adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
        }
        //Now we make a name for this polygon
        std::stringstream ss;
        ss << "supervoxel_" << supervoxel_label;
        //This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
        addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), viewer);
        //Move iterator forward to next label
        label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
    }

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
    }

/*    visualizer->addPointCloud(cloud, cloudName);
    visualizer->setBackgroundColor(0, 0, 0);
    visualizer->setShowFPS(true);
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
    //visualizer->addCoordinateSystem(1.0);
    visualizer->initCameraParameters();

    while (!visualizer->wasStopped()) {
        visualizer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }*/

    return 0;
}
