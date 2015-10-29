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
#include <opencv/cv.h>
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
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


using namespace cv;
using namespace std;
void bleh()
{
    Mat fullImg;
    string bleh = "C:/Users/kristinn/Pictures/kula_calib_myndir/calibMyndir_fixed/test/calib4_fixed.jpg";
    fullImg = imread(bleh,IMREAD_COLOR);
    Size imSize = fullImg.size();
    Mat img1 = fullImg(Range(300, imSize.height-800),Range(650, imSize.width/2)).clone();

    Mat img2 = fullImg(Range(300, imSize.height-800),Range(imSize.width/2, imSize.width-650)).clone();
    namedWindow( "Left Window",WINDOW_NORMAL|WINDOW_KEEPRATIO);
    namedWindow( "Right Window",WINDOW_NORMAL|WINDOW_KEEPRATIO);
    imshow("Left Window", img1);
    imshow("Right Window", img2);

    waitKey(0);
}


/*
 *
 *
 *
 */



int main(int argc, char *argv[])
{
    //StereoCalibrate cc;
    //cc.findAndDrawChessBoardCorners();
    //bleh();


    string g = "Y.xml";
    //cc.findAndDrawChessBoardCorners(g);
    //cc.CalibrateStereoCamera();
    //cc.initUndistort();
    //cc.rectifyCamera();
    //StereoScopicImage ssi;
    //ssi.rectifyCamera();
    //ssi.disparityMap();

    //ssi.disparityMap("C:/Users/Notandi/Documents/GitHub/Lokaverkefni2/Y.xml");
/*
    //Convert
    //untill sterio calibration is complete use these test images
    Mat img_rgb = imread("C:/Users/Notandi/Pictures/Screenshots/left.png", CV_LOAD_IMAGE_COLOR);
    Mat img_disparity = imread("C:/Users/Notandi/Pictures/Screenshots/disp.jpg", CV_LOAD_IMAGE_GRAYSCALE);
    Convert con(img_rgb,img_disparity);
*/

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
    string file = "Q.xml";  // moved files to debug & release folder "relative path"
    FileStorage fs(file, cv::FileStorage::READ);
    fs["Q"] >> Q;                                           //Load Matrix Q
    Mat img_rgb = imread("left.png", CV_LOAD_IMAGE_COLOR);              // moved files to debug & release folder "relative path"
    Mat img_disparity = imread("disp.jpg", CV_LOAD_IMAGE_GRAYSCALE);    // moved files to debug & release folder "relative path"

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
    /****viewport is split screen not useful****
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
      //visualizer.viewer->saveScreenshot("screenshot.png");
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    return 0;
}
