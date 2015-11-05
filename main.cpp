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
#include "utils.h"

using namespace cv;
using namespace std;
void bleh()
{
    Mat fullImg,img1,img2;

    string bleh = "C:/Users/notandi/Pictures/kula_calib_myndir2/calibMyndir_fixed/calib4_fixed.jpg";
    fullImg = imread(bleh,IMREAD_COLOR);
    Size imSize = fullImg.size();

    double zoom = 0;
    StereoCalibrate c2;
    img1 = fullImg(Range(0, imSize.height),Range(0, imSize.width/2)).clone();

    img2 = fullImg(Range(0, imSize.height),Range(imSize.width/2, imSize.width)).clone();
    matPair pair;
    pair.left = img1;
    pair.right = img2;
    pair = c2.undestort(pair);
    pair = BorderRemoveal(pair);
    //ult.splitImage(fullImg,img1,img2);
    namedWindow( "Left Window",WINDOW_NORMAL|WINDOW_KEEPRATIO);
    namedWindow( "Right Window",WINDOW_NORMAL|WINDOW_KEEPRATIO);
    imshow("Left Window", pair.left);
    imshow("Right Window", pair.right);

    waitKey(0);
}


/*
 *
 *
 *
 */



int main(int argc, char *argv[])
{

    StereoCalibrate cc;
    //cc.findAndDrawChessBoardCorners();
    //bleh();


    cc.findAndDrawChessBoardCorners("../Lokaverkefni2/Y.xml");
    string bleh = "C:/Users/notandi/Pictures/kula_calib_myndir2/calibMyndir_fixed/calib4_fixed.jpg";
    Mat fullImg, NewImg;
    fullImg = imread(bleh,IMREAD_COLOR);
    NewImg = cc.undestortZoom(fullImg,bleh);

    Size imSize = fullImg.size();

    Mat img1 = fullImg(Range(0, imSize.height),Range(0, imSize.width/2)).clone();

    Mat img2 = fullImg(Range(0, imSize.height),Range(imSize.width/2, imSize.width)).clone();
    matPair pair;
    pair.left = img1;
    pair.right = img2;
    pair = cc.undestort(pair);
    pair = BorderRemoveal(pair);
    cc.img1 = pair.left;
    cc.img2 = pair.right;
    cc.CalibrateStereoCamera();
    cc.initUndistort();



    //cc.findAndDrawChessBoardCorners("X.xml");

    string g = "Y.xml";
    //cc.findAndDrawChessBoardCorners(g);

    //cc.CalibrateStereoCamera();
    //cc.initUndistort();
    //cc.rectifyCamera();
    //StereoScopicImage ssi;
    //ssi.rectifyCamera();
    //ssi.disparityMap();



    //ssi.disparityMap("C:/Users/Notandi/Documents/GitHub/Lokaverkefni2/Y.xml");

    //Convert
    //untill sterio calibration is complete use these test images
    Mat img_rgb = imread("../Lokaverkefni2/myndir/check.jpg", CV_LOAD_IMAGE_COLOR);
    Mat img_disparity = imread("../Lokaverkefni2/myndir/checkdisp2.jpg", CV_LOAD_IMAGE_GRAYSCALE);



    //ssi.disparityMap("X.xml");


    //ssi.disparityMap("X.xml");

    //Convert

    // declare classes
    Convert utilities;
    Visualizer visualizer;
    DataHolder dataHolder;

    // PCL variables & other temp location*****
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mainCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr triangulate_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    pcl::PolygonMesh triangles;

    Mat Q;
    string file = "../Lokaverkefni2/Q.xml";  // moved files to debug & release folder "relative path"
    FileStorage fs(file, cv::FileStorage::READ);
    fs["Q"] >> Q;                                           //Load Matrix Q


    // PCL variables & other temp location*****

    /*point cloud 2 xyzrgb, filers,triangulate*
     *  point cloud from rgb image, depth     *
     *  image & an empty point cloud of       *
     *  pointXYZRGB.                          *
     *  Point cloud filters applied           *
     *****************************************/


    mainCloud = utilities.matToCloud(img_rgb,img_disparity,Q,mainCloud);
    cloud_filtered = utilities.SOR_filter(mainCloud);
    pcl::PCDWriter writer;
    //writer.write<pcl::PointXYZRGB> ("triangulate.pcd", *cloud_filtered, false);
    pcl::io::savePCDFileBinaryCompressed ("triangulate.pcd", *cloud_filtered);
    pcl::io::loadPCDFile("triangulate.pcd", *mainCloud);
    triangles = utilities.triangulate(mainCloud);


    /***********viewport setup******************
     * default viewport is from left side.     *
     * The right side can be found by          *
     * mirroring the left camera over the left *
     * camera Y-up axis & the left camera      *
     * view-Z axis with an offset of half the  *
     * distance in X-axis between the mirrors  *
     ******************************************/
     /***viewport is split screen not useful****
     *******************************************/






    if(visualizer.displayPoly == false && visualizer.displayPoints == true)
        visualizer.viewer = visualizer.displayPointCloudColor(cloud_filtered);      // view point cloud
    else if(visualizer.displayPoly == true && visualizer.displayPoints == false)
    {
        pcl::io::saveVTKFile("triangulation.vtk", triangles);                   // save polygon Mesh to file
        visualizer.viewer = visualizer.displayPolyMesh(cloud_filtered,triangles); // view polyMesh
    }
    else
    {
        // view point cloud with objects(used for locating the point cloud
        visualizer.viewer = visualizer.displayLocatorObject(cloud_filtered);
    }



    /***********Main render loop**************
     *  Loop untils pcl viewer is turned off *
    ******************************************/

    while ( !visualizer.viewer->wasStopped())
    {
      visualizer.viewer->spinOnce(100);
      visualizer.viewer->saveScreenshot("screenshot.png");
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }



    return 0;
}
