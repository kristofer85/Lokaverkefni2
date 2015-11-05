//#include <pcl/pcl_tests.h>
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
#include <iostream>
#include <string>
#include <iostream>
#include "reprojectimageto3d.h"
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
#include "dataholder.h"
#include <boost/thread/thread.hpp>
#include "visualizer.h"
#include "test.h"
#include "point_2d.h"
#include "imageprocessing.h"

using namespace cv;
using namespace std;


int main(int argc, char *argv[])
{

 //StereoCalibrate cc;
   ////     //cc.findAndDrawChessBoardCorners();
   //    cc.findAndDrawChessBoardCorners("Y.xml");
   ////     //// Read multiple image for stereo calibration &
   ////     //// findChessBoard corners
   ////     ////cc.findAndDrawChessBoardCorners("Y.xml");
   //  cc.CalibrateStereoCamera();
   //
   //     StereoScopicImage ssi;
   //     ssi.rectifyCamera();
//cc.initUndistort();
   //     //
   //     //
   //     //
   //     //
        //
        //
        //
        //
        //
        ////cc.rectifyCamera();
        //
        //ssi.rectifyCamera();
        ////ssi.disparityMap();



        //ssi.disparityMap("Y.xml");


    Convert utilities;
    Visualizer visualizer;
    DataHolder dataHolder;
    // PCL variables & other temp location*****
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mainCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr normal (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PolygonMesh triangles;
    //dataHolder.fs1.open("stereoCalibration.yml", FileStorage::READ);
    Mat Q;
    string file = "Q.xml";  // moved files to debug & release folder "relative path"
    FileStorage fs = FileStorage(file, FileStorage::READ);
    fs["Q"] >> Q;                                            //Load Matrix Q
    //ReprojectImageTo3d rProj3d;
    //Mat d = imread("d.jpg",CV_LOAD_IMAGE_GRAYSCALE);
    //Mat outDisp;
    //rProj3d.reproject(d,Q,outDisp);
    //string f = "testReproject.txt";
    //rProj3d.save(outDisp,f);
    Mat img_rgb = imread("c.jpg", CV_LOAD_IMAGE_COLOR);              // moved files to debug & release folder "relative path"
    Mat img_disparity = imread("d2.jpg", CV_LOAD_IMAGE_GRAYSCALE);    // moved files to debug & release folder "relative path"
    /*point cloud 2 xyzrgb, filers,triangulate*
     *  point cloud from rgb image, depth     *
     *  image & an empty point cloud of       *
     *  pointXYZRGB.                          *
     *  Point cloud filters applied           *
     *****************************************/
    //ImageProcessing j;
    //j.selectContinuousPixel(0,0,img_disparity);

    mainCloud = utilities.matToCloud(img_rgb,img_disparity,Q,mainCloud);
    cloud_filtered = utilities.SOR_filter(mainCloud);
    //
    triangles = utilities.triangulate(cloud_filtered);
    //normal = utilities.curveNormals(mainCloud);

    if(visualizer.displayPoly == false && visualizer.displayPoints == true)
        visualizer.viewer = visualizer.displayPointCloudColor(cloud_filtered);      // view point cloud
    else if(visualizer.displayPoly == true && visualizer.displayPoints == false)
        visualizer.viewer = visualizer.displayPolyMesh(cloud_filtered,triangles); // view polyMesh
    else
        visualizer.viewer = visualizer.displayLocatorObject(cloud_filtered);

    /***********Main render loop**************
     *  Loop untils pcl viewer is turned off *
    ******************************************/
    while ( !visualizer.viewer->wasStopped())
    {
      visualizer.viewer->spinOnce(100);
      //visualizer.viewer->saveScreenshot("screenshot.png");
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    return 0;
}
