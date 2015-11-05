#ifndef CONVERT_H
#define CONVERT_H

#include <opencv/cv.h>
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <string>
#include <iostream>
#include <pcl/surface/poisson.h>
#include <pcl/surface/boost.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/TextureMesh.h>
#include <pcl/surface/texture_mapping.h>
//#include "pclwindow.h"
#include "visualizer.h"
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include "dataholder.h"
#include "stereocalibrate.h"
#include <QApplication>
#include <opencv/cv.h>
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "stereoscopicimage.h"
#include "convert.h"
//#include "pclwindow.h"
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>
#include <pcl/point_cloud.h>
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
#include "visualizer.h"
#include <pcl/surface/mls.h>
#include <pcl/stereo/disparity_map_converter.h>
class Convert
{
public:
    Convert();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr matToCloud(cv::Mat rgb,cv::Mat disp,cv::Mat Q,pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr SOR_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    pcl::PolygonMesh triangulate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr curveNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr disparityToPointCloud(std::string disparity);


};

#endif // CONVERT_H
