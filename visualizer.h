#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <pcl/visualization/pcl_visualizer.h>
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

class Visualizer
{
public:
    Visualizer();
    boost::shared_ptr<pcl::visualization::PCLVisualizer> pointCloudVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
};

#endif // VISUALIZER_H
