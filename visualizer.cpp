#include "visualizer.h"
#include "convert.h"
using namespace cv;
using namespace std;
using namespace pcl;
using namespace pcl::visualization;
Visualizer::Visualizer()
{
    displayPoly = false;
    displayPoints = true;
    renderFrameWidth = 1024;
    renderFrameHeight = 1024;
}





//pcl::PCLPointCloud2 cloud_blob;
//  loadPCDFile (argv[1], cloud_blob);
//  fromPCLPointCloud2 (cloud_blob, *cloud);



/***********Display point cloud***********
 *  Loop untils pcl viewer is turned off *
******************************************/
boost::shared_ptr<PCLVisualizer> Visualizer::displayPointCloudColor (PointCloud<PointXYZRGB>::ConstPtr cloud,Mat color)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    viewer->setBackgroundColor (0, 0, 0);
    viewer->getRenderWindow();
    viewer->setSize(color.size().width,color.size().height);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "3D Viewer");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "3D Viewer");
    viewer->setCameraPosition( 2.56012, -10.2816, 61.2513, 1.498, -9.69651, 82.6992,0.0311421, -0.9991, 0.0287981);
    viewer->setCameraClipDistances(2.35827, 35.2884);
    viewer->setPosition(color.size().width/2,color.size().height/2);
    viewer->getRenderWindow();
    //viewer->setSize(getRenderWindowWidth,getRenderWindowHeight);
    //viewer->initCameraParameters ();
        viewer->saveCameraParameters("frameCamera2.cam");

    return (viewer);
}

/***********Display polygon mesh**********
 *  Loop untils pcl viewer is turned off *
******************************************/

boost::shared_ptr<pcl::visualization::PCLVisualizer> Visualizer::displayPolyMesh (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,pcl::PolygonMesh triangles,Mat  color)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->getRenderWindow();
    viewer->setSize(color.size().width,color.size().height);
    viewer->addPolygonMesh(triangles);
    //pcl::PointXYZ center (0, 0, 0);
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0,0,"3D Viewer");
    viewer->setCameraPosition( 2.56012, -10.2816, 61.2513, 1.498, -9.69651, 82.6992,0.0311421, -0.9991, 0.0287981);
    viewer->setCameraClipDistances(2.35827, 35.2884);
    viewer->setPosition(color.size().width/2,color.size().height/2);
    viewer->addCoordinateSystem(1.0);
    viewer->saveCameraParameters("frameCamera2.cam");


    return (viewer);
}



