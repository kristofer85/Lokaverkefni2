#include "convert.h"
using namespace cv;
using namespace std;
using namespace boost;
using namespace pcl;
using namespace pcl::visualization;

Convert::Convert(Mat image, Mat disp)
{
    Mat Q;
    //our Q matrix untill sterio calibration is done a test Q matrix will be used
/*
    DataHolder dataHolder;
    dataHolder.fs1.open("stereoCalibration.yml", FileStorage::READ);
    dataHolder.fs1["Q"] >> Q;
*/

    string file = "C:/Users/Notandi/Documents/QtProjects/OpenCVMatToPCL/Q.xml";
    //Load Matrix Q
    FileStorage fs(file, cv::FileStorage::READ);

    fs["Q"] >> Q;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

    point_cloud_ptr = matToCloud(image,disp,Q,point_cloud_ptr);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (point_cloud_ptr);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);



    //viewer = createVisualizer( point_cloud_ptr );
    viewer = createVisualizer( cloud_filtered );

    //Main loop
    while ( !viewer->wasStopped())
    {

      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

shared_ptr<PCLVisualizer> Convert::createVisualizer (PointCloud<PointXYZRGB>::ConstPtr cloud)
{
  shared_ptr<PCLVisualizer> viewer (new PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  PointCloudColorHandlerRGBField<PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<PointXYZRGB> (cloud, rgb, "reconstruction");
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


