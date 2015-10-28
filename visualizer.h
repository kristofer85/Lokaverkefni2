#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <iostream>

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
