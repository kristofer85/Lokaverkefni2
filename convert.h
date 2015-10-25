#ifndef CONVERT_H
#define CONVERT_H

#include <opencv/cv.h>
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <string>
#include <iostream>
//#include "pclwindow.h"
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
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
#include "dataholder.h"
class Convert
{
public:
    Convert(cv::Mat rgb,cv::Mat disp);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr matToCloud(cv::Mat rgb,cv::Mat disp,cv::Mat Q,pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

//signals:

//public slots:
};

#endif // CONVERT_H
