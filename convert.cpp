#include "convert.h"
using namespace cv;
using namespace std;
using namespace boost;
using namespace pcl;
using namespace pcl::visualization;

Convert::Convert()
{
}

shared_ptr<PCLVisualizer> Convert::createVisualizer (PointCloud<PointXYZRGB>::ConstPtr cloud)
{
  shared_ptr<PCLVisualizer> viewer (new PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  PointCloudColorHandlerRGBField<PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<PointXYZRGB> (cloud, rgb, "reconstruction");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "reconstruction");
  viewer->addCoordinateSystem ( 1.0 );
  viewer->initCameraParameters ();
  return (viewer);
}

shared_ptr<PCLVisualizer> Convert::polyMeshVisualizer (PointCloud<PointXYZRGB>::ConstPtr cloud,pcl::PolygonMesh triangles)
{
  shared_ptr<PCLVisualizer> viewer (new PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  PointCloudColorHandlerRGBField<PointXYZRGB> rgb(cloud);
  viewer->addPolygonMesh(triangles,"triangulation.vtk");
  //viewer->addPointCloud<PointXYZRGB> (cloud, rgb, "reconstruction");
  //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "reconstruction");
  viewer->addCoordinateSystem ( 1.0 );
  viewer->initCameraParameters ();
  return (viewer);
}
//make a function to convert a cv mat into a point cloud
PointCloud<PointXYZRGB>::Ptr Convert::matToCloud(Mat rgb,Mat disp,Mat Q,PointCloud<PointXYZRGB>::Ptr Cloud)
{
    double px, py, pz;
    uchar pr, pg, pb;
    double Q03, Q13, Q23, Q32, Q33;
    Q03 = Q.at<double>(0,3);
    Q13 = Q.at<double>(1,3);
    Q23 = Q.at<double>(2,3);
    Q32 = Q.at<double>(3,2);
    Q33 = Q.at<double>(3,3);

    for (int i = 0; i < rgb.rows; i++)
    {
        uchar* rgb_ptr = rgb.ptr<uchar>(i);
        uchar* disp_ptr = disp.ptr<uchar>(i);
        for (int j = 0; j < rgb.cols; j++)
        {
            uchar d = disp_ptr[j];
            if ( d == 0 ) continue; //Discard bad pixels
            double pw = -1.0 * static_cast<double>(d) * Q32 + Q33;
            px = static_cast<double>(j) + Q03;
            py = static_cast<double>(i) + Q13;
            pz = Q23;

            px = px/pw;
            py = py/pw;
            pz = pz/pw;
            //Get RGB info
            pb = rgb_ptr[3*j];
            pg = rgb_ptr[3*j+1];
            pr = rgb_ptr[3*j+2];

            //Insert info into point cloud structure
            PointXYZRGB point;
            point.x = px;
            point.y = py;
            point.z = pz;
            uint32_t rgb = (static_cast<uint32_t>(pr) << 16 |
                    static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
            point.rgb = *reinterpret_cast<float*>(&rgb);
            Cloud->points.push_back (point);
        }
    }
    Cloud->width = (int) Cloud->points.size();
    Cloud->height = 1;
    return Cloud;
}



pcl::PointCloud<pcl::PointXYZRGB>::Ptr Convert::SOR_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filter (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*filter);
    return filter;

}

void Convert::triangulate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PolygonMesh triangles)
{
    /*
    // Normal estimation*
     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
     pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
     tree->setInputCloud (cloud);
     n.setInputCloud (cloud);
     n.setSearchMethod (tree);
     n.setKSearch (20);
     n.compute (*normals);
     //* normals should not contain the point normals + surface curvatures

     // Concatenate the XYZ and normal fields*
     pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
     pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
     //* cloud_with_normals = cloud + normals

     // Create search tree*
     pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
     tree2->setInputCloud (cloud_with_normals);

     // Initialize objects
     pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;


     // Set the maximum distance between connected points (maximum edge length)
     gp3.setSearchRadius (0.025);

     // Set typical values for the parameters
     gp3.setMu (2.5);
     gp3.setMaximumNearestNeighbors (100);
     gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
     gp3.setMinimumAngle(M_PI/18); // 10 degrees
     gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
     gp3.setNormalConsistency(false);

     // Get result
     gp3.setInputCloud (cloud_with_normals);
     gp3.setSearchMethod (tree2);
     gp3.reconstruct (triangles);

     // Additional vertex information
     std::vector<int> parts = gp3.getPartIDs();
     std::vector<int> states = gp3.getPointStates();

     pcl::io::saveVTKFile("triangulation.vtk", triangles);
     // Use for viewing triangulated point cloud || polymesh
     viewer = polyMeshVisualizer();
    */
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Convert::pointXYZRGB(cv::Mat rgb,cv::Mat disp,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    Mat Q;
    string file = "Q.xml";  // moved files to debug & release folder "relative path"
    //Load Matrix Q
    FileStorage fs(file, cv::FileStorage::READ);

    fs["Q"] >> Q;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

    point_cloud_ptr = matToCloud(rgb,disp,Q,point_cloud_ptr);
    cloud_filtered = SOR_filter(point_cloud_ptr);

    // Use for viewing point cloud with pointXYZRGB
    //viewer = createVisualizer( cloud_filtered );


    return cloud_filtered;

}


