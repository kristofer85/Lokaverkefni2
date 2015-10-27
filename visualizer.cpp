#include "visualizer.h"

Visualizer::Visualizer()
{

}

boost::shared_ptr<pcl::visualization::PCLVisualizer> pointCloudVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  boost::shared_ptr<PCLVisualizer> viewer (new PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  PointCloudColorHandlerRGBField<PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<PointXYZRGB> (cloud, rgb, "reconstruction");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "reconstruction");
  viewer->addCoordinateSystem ( 1.0 );
  viewer->initCameraParameters ();
  return (viewer);
}
