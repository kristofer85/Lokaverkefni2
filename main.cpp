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
    namedWindow( "Left Window",WINDOW_NORMAL| WINDOW_KEEPRATIO );
    namedWindow( "Right Window",WINDOW_NORMAL| WINDOW_KEEPRATIO );
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


    //cc.findAndDrawChessBoardCorners("X.xml");
    //cc.CalibrateStereoCamera();
    //cc.initUndistort();
    //cc.rectifyCamera();
    //StereoScopicImage ssi;
    //ssi.rectifyCamera();
    //ssi.disparityMap();
    //ssi.disparityMap("X.xml");

    //Convert
    //untill sterio calibration is complete use these test images
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mainCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr triangulate_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PolygonMesh triangles;
    Mat img_rgb = imread("left.png", CV_LOAD_IMAGE_COLOR);              // moved files to debug & release folder "relative path"
    Mat img_disparity = imread("disp.jpg", CV_LOAD_IMAGE_GRAYSCALE);    // moved files to debug & release folder "relative path"
    Convert utilities;
    triangulate_cloud = utilities.pointXYZRGB(img_rgb,img_disparity,mainCloud);
    utilities.viewer = utilities.createVisualizer( triangulate_cloud );
    //utilities.triangulate(mainCloud,triangles);
    while ( !utilities.viewer->wasStopped())
    {

      utilities.viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    return 0;
}
