#include "visualizer.h"
using namespace pcl;
using namespace pcl::visualization;
Visualizer::Visualizer()
{
    displayPoly = false;
    displayPoints = true;
}

/***********Display point cloud***********
 *  Loop untils pcl viewer is turned off *
******************************************/
boost::shared_ptr<pcl::visualization::PCLVisualizer> Visualizer::displayPointCloudColor (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "reconstruction");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "reconstruction");
    viewer->addCoordinateSystem ( 1.0 );
    //viewer->initCameraParameters ();
    return (viewer);
}

/***********Display polygon mesh**********
 *  Loop untils pcl viewer is turned off *
******************************************/

boost::shared_ptr<pcl::visualization::PCLVisualizer> Visualizer::displayPolyMesh (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,pcl::PolygonMesh triangles)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPolygonMesh(triangles,"triangulation.vtk");
    viewer->addCoordinateSystem ( 1.0 );
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0,"normals");
    //viewer->initCameraParameters ();
    return (viewer);
}

/***Open 3D viewer and add point cloud and normals**
****************************************************/
boost::shared_ptr<pcl::visualization::PCLVisualizer> displayPointCloudColorNormal (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals1, pcl::PointCloud<pcl::Normal>::ConstPtr normals2)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();

    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor (0, 0, 0, v1);
    viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud1", v1);

    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
    viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "sample cloud2", v2);

    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
    viewer->addCoordinateSystem (1.0);

    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals1, 10, 0.05, "normals1", v1);
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals2, 10, 0.05, "normals2", v2);

    return (viewer);
}

/******Compare locations of camera*********
 *  Test visualizer: adds small spheres   *
 *  with specific transform and rotate.   *
 *  Compare positions from the camera     *
 *  matrix from stereo calibration with   *
 *  transformed cubes. This will hopfully *
 *  tell something about the quality of   *
 *  the stereo calibration.               *
*******************************************/
boost::shared_ptr<pcl::visualization::PCLVisualizer> Visualizer::displayLocatorObject (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{

  /****Open 3D viewer and add point cloud****
  *******************************************/
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();


  /*******Add shapes at cloud points*********
  *******************************************/
  //viewer->addLine<pcl::PointXYZRGB> (cloud->points[0],cloud->points[(cloud->size() - 1) + 38.7], "line");
  viewer->addSphere (cloud->points[0], 0.4, 0.7, 0.7, 0.0, "sphere");
  viewer->addSphere (cloud->points[(cloud->size() - 1)], 0.4, 0.7, 0.7, 0.0, "sphere2");

  /*******Add shapes at other locations*********
  *******************************************/

  viewer->addCoordinateSystem (1.0,0.0,0.0,12.0);
  //viewer->setCameraParameters();
  //viewer->setCameraPosition(-1.6, 2.18, 8.6, 0.0, 1.0, 0.0, 0);
  viewer->resetCamera();
  //int leftViewport(0);
  //viewer->createViewPort(0.0, 0.0, 0.5, 1.0, leftViewport);
  //viewer->setBackgroundColor (0, 0, 0, leftViewport);
  //
  //int rightViewport(0);
  //viewer->createViewPort(0.0, 0.0, 0.5, 1.0, leftViewport);
  //viewer->setBackgroundColor (1, 1, 1, leftViewport);
  return (viewer);
}


