#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "convert.h"
#include <iostream>
#include <string>
#include <opencv/cv.h>
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <string>
#include <iostream>
//#include "pclwindow.h"
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


class Visualizer
{
public:
    Visualizer();
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;    // viewer = displayPointCloudColor || displaypolygonMesh
    bool displayPoly;
    bool displayPoints;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> displayPointCloudColor (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> displayPolyMesh (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,pcl::PolygonMesh triangles);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> displayLocatorObject (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> displayPointCloudColorNormal (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals1, pcl::PointCloud<pcl::Normal>::ConstPtr normals2);
};

#endif // VISUALIZER_H
