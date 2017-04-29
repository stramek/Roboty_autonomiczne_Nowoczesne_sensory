#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/lccp_segmentation.h>

//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>

#include <opencv2/opencv.hpp>
#include <include/models/CloudPlaneDetector.h>

using namespace std;
using namespace cv;

// Types
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;
typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;
typedef boost::graph_traits<SuperVoxelAdjacencyList>::vertex_iterator VertexIterator;

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
            register const float depthValue = *itD / 5000.0f;
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
addSupervoxelConnectionsToViewer(PointT &supervoxel_center,
                                 PointCloudT &adjacent_supervoxel_centers,
                                 std::string supervoxel_name,
                                 boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer) {
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();

    //Iterate through all adjacent points, and add a center point to adjacent point pair
    PointCloudT::iterator adjacent_itr = adjacent_supervoxel_centers.begin();
    for (; adjacent_itr != adjacent_supervoxel_centers.end(); ++adjacent_itr) {
        points->InsertNextPoint(supervoxel_center.data);
        points->InsertNextPoint(adjacent_itr->data);
    }
    // Create a polydata to store everything in
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    // Add the points to the dataset
    polyData->SetPoints(points);
    polyLine->GetPointIds()->SetNumberOfIds(points->GetNumberOfPoints());
    for (unsigned int i = 0; i < points->GetNumberOfPoints(); i++)
        polyLine->GetPointIds()->SetId(i, i);
    cells->InsertNextCell(polyLine);
    // Add the lines to the dataset
    polyData->SetLines(cells);
    //viewer->addModelFromPolyData (polyData,supervoxel_name);
}

uint8_t getC(int tagValue) {
    return (uint8_t) (tagValue % 255);
}

void test(pcl::PointCloud<pcl::PointXYZL> &labeled_cloud_arg, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target) {

    for (PointXYZL point : labeled_cloud_arg) {
        uint32_t label = point.label;
        pcl::PointXYZRGB pointToAdd = pcl::PointXYZRGB(getC(label), getC(label), getC(label));
        pointToAdd.x = point.x;
        pointToAdd.y = point.y;
        pointToAdd.z = point.z;

        target.get()->points.push_back(pointToAdd);
    }
}

int main(int argc, char **argv) {
    color = imread("../images//rgb//0.png");
    depth = imread("../images//depth//0.png", CV_LOAD_IMAGE_ANYDEPTH);

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

    pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
    super.setUseSingleCameraTransform(true);
    super.setInputCloud(cloud);
    super.setColorImportance(color_importance);
    super.setSpatialImportance(spatial_importance);
    super.setNormalImportance(normal_importance);

    std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;

    pcl::console::print_highlight("Extracting supervoxels!\n");
    super.extract(supervoxel_clusters);
    pcl::console::print_info("Found %d supervoxels\n", supervoxel_clusters.size());

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer(
            new pcl::visualization::PCLVisualizer("Cloud Viewer"));
    visualizer->setBackgroundColor(0, 0, 0);

    PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud();
    /*visualizer->addPointCloud(voxel_centroid_cloud, "voxel centroids");
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "voxel centroids");
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.95, "voxel centroids");
*/
    PointLCloudT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud();
    /*visualizer->addPointCloud(labeled_voxel_cloud, "labeled voxels");
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, "labeled voxels");
*/
    PointNCloudT::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud(supervoxel_clusters);
    //We have this disabled so graph is easy to see, uncomment to see supervoxel normals
    //visualizer->addPointCloudNormals<PointNormal> (sv_normal_cloud,1,0.05f, "supervoxel_normals");

    pcl::console::print_highlight("Getting supervoxel adjacency\n");
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency(supervoxel_adjacency);
    //To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
    std::multimap<uint32_t, uint32_t>::iterator label_itr = supervoxel_adjacency.begin();

    for (; label_itr != supervoxel_adjacency.end();) {
        //First get the label
        uint32_t supervoxel_label = label_itr->first;
        //Now get the supervoxel corresponding to the label
        pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at(supervoxel_label);

        //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
        PointCloudT adjacent_supervoxel_centers;
        std::multimap<uint32_t, uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range(
                supervoxel_label).first;
        for (; adjacent_itr != supervoxel_adjacency.equal_range(supervoxel_label).second; ++adjacent_itr) {
            pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at(adjacent_itr->second);
            adjacent_supervoxel_centers.push_back(neighbor_supervoxel->centroid_);
        }
        //Now we make a name for this polygon
        std::stringstream ss;
        ss << "supervoxel_" << supervoxel_label;
        //This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
        addSupervoxelConnectionsToViewer(supervoxel->centroid_, adjacent_supervoxel_centers, ss.str(), visualizer);
        //Move iterator forward to next label
        label_itr = supervoxel_adjacency.upper_bound(supervoxel_label);
    }

    // LCCPSegmentation Stuff
    float concavity_tolerance_threshold = 10;
    float smoothness_threshold = 0.1;
    uint32_t min_segment_size = 5;
    bool use_extended_convexity = true;
    bool use_sanity_criterion = true;
    unsigned int k_factor = 0; // or change to 1

    pcl::console::parse(argc, argv, "-ctt", concavity_tolerance_threshold);
    pcl::console::parse(argc, argv, "-st", smoothness_threshold);
    pcl::console::parse(argc, argv, "-mss", min_segment_size);
    pcl::console::parse(argc, argv, "-uec", use_extended_convexity);
    pcl::console::parse(argc, argv, "-usc", use_sanity_criterion);
    pcl::console::parse(argc, argv, "-kf", k_factor);
    cout << "smoothness_threshold: " << smoothness_threshold << endl;

    PCL_INFO ("Starting Segmentation\n");
    pcl::LCCPSegmentation<PointT> lccp;
    lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
    lccp.setSanityCheck(use_sanity_criterion);
    lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
    lccp.setKFactor(k_factor);
    lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
    lccp.setMinSegmentSize(min_segment_size);
    lccp.segment();


    PCL_INFO ("Interpolation voxel cloud -> input cloud and relabeling\n");

    pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
    pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
    lccp.relabelCloud(*lccp_labeled_cloud);
    visualizer->addPointCloud(lccp_labeled_cloud);
    viewer->setBackgroundColor(0, 0, 0);
    //viewer->addPointCloud(lccp_labeled_cloud, "maincloud");

//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target(new pcl::PointCloud<pcl::PointXYZRGB>());
//    test(*lccp_labeled_cloud, target);

    CloudPlaneDetector cloudPlaneDetector(*lccp_labeled_cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target = cloudPlaneDetector.getPointCloud();


    viewer->addPointCloud(target, "maincloud");
//    std::map<uint32_t, std::set<uint32_t>> segmentMap;
//    lccp.getSegmentToSupervoxelMap(segmentMap);
//
//    for (std::map<uint32_t, std::set<uint32_t>>::iterator it = segmentMap.begin(); it != segmentMap.end(); ++it) {
//        //it->second.Method();
//        uint32_t mapKey = it->first;
//        std::set<uint32_t> voxelSet = it->second;
//        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr finalCloud1(new pcl::PointCloud<pcl::PointXYZRGBA>());
//
//        for (uint32_t voxel : voxelSet) {
//            pcl::Supervoxel<PointT>::Ptr point = supervoxel_clusters.at(voxel);
//            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = point.get()->voxels_;
//            *finalCloud1 += *cloud;
//        }
//        cout<<"Cloud size: "<<finalCloud1->points.size()<<endl;
//        viewer->addPointCloud(finalCloud1, to_string(it->first));
//    }


    while (!viewer->wasStopped() || !visualizer->wasStopped()) {
        viewer->spinOnce(100);
        visualizer->spinOnce(100);
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
