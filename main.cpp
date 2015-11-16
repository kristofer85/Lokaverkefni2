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
#include "opencv2/features2d/features2d.hpp"
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
#include "stereocalibrate.h"
#include "test.h"
#include "point_2d.h"
#include "imageprocessing.h"
#include "depthmap.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"



using namespace cv;
using namespace std;
int main(int argc, char *argv[])
{

    StereoCalibrate stereoCalibrate;
    stereoCalibrate.findAndDrawChessBoardCorners("Y.xml");
    stereoCalibrate.findAndDrawChessBoardCorners("../lokaverkefni2/Y.xml");
    stereoCalibrate.CalibrateStereoCamera();
    stereoCalibrate.rectifyCamera();
    DepthMap depthMap;
    depthMap.run();
    //stereoCalibrate.rectifyCamera();
    stereoCalibrate.initUndistort();

    //DepthMap depthMap;
    //depthMap.run();
/*
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

    string file = "stereoCalibration.yml";  // moved files to debug & release folder "relative path"
    FileStorage fs = FileStorage(file, FileStorage::READ);
    fs["Q"] >> Q;                                            //Load Matrix Q
    fs["Q"] >> Q;
*/                                   //Load Matrix Q
    //ReprojectImageTo3d rProj3d;
    //Mat d = imread("d.jpg",CV_LOAD_IMAGE_GRAYSCALE);
    //Mat outDisp;
    //rProj3d.reproject(d,Q,outDisp);
    //string f = "testReproject.txt";
    //rProj3d.save(outDisp,f);








    return 0;
}
