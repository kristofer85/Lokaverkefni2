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
#include "convert.h"
#include <iostream>
#include <string>
#include <iostream>
#include "opencv2/features2d/features2d.hpp"
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
#include <boost/thread/thread.hpp>
#include "visualizer.h"
#include "stereocalibrate.h"
#include "depthmap.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <pcl/io/pcd_io.h>

using namespace cv;
using namespace std;
int main(int argc, char *argv[])
{

    //StereoCalibrate stereoCalibrate;
    //stereoCalibrate.findAndDrawChessBoardCorners("Y.xml");
    //stereoCalibrate.CalibrateStereoCamera();
    //stereoCalibrate.rectifyCamera();
    //DepthMap depthMap;
    //depthMap.run();
    Convert utilities;
    Visualizer visualizer;

    // PCL variables & other temp location*****
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mainCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointNormal>::Ptr pointNormal (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr normal (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PolygonMesh triangles;
    //dataHolder.fs1.open("stereoCalibration.yml", FileStorage::READ);
    Mat Q;                                     //Load Matrix Q
    string cameraCal = "stereoCalibration.yml";
    FileStorage fs = FileStorage(cameraCal, FileStorage::READ);
    fs["Q"] >> Q;
    Mat color = imread("im0.png", CV_LOAD_IMAGE_COLOR);
    Mat disparity = imread("disp8SGBM.png", CV_LOAD_IMAGE_GRAYSCALE);

    /*point cloud 2 xyzrgb, filers,triangulate*
     *  point cloud from rgb image, depth     *
     *  image & an empty point cloud of       *
     *  pointXYZRGB.                          *
     *  Point cloud filters applied           *
     *****************************************/
    utilities.matToCloud(color,disparity,Q,mainCloud);

    mainCloud = utilities.SOR_filter(mainCloud);

    utilities.smoothNormals(mainCloud);
    //mainCloud.reset();
    //pcl::PCLPointCloud2 cloud_blob;
    //pcl::io::loadPCDFile ("SmoothNormals.pcd", cloud_blob);
    //fromPCLPointCloud2 (cloud_blob, *mainCloud);
    triangles = utilities.triangulate(mainCloud);
    //normal = utilities.curveNormals(mainCloud);


    if(visualizer.displayPoly == false && visualizer.displayPoints == true)
        visualizer.viewer = visualizer.displayPointCloudColor(mainCloud,color);      // view point cloud
    else if(visualizer.displayPoly == true && visualizer.displayPoints == false)
        visualizer.viewer = visualizer.displayPolyMesh(mainCloud,triangles,color); // view polyMesh

    /***********Main render loop**************
     *  Loop untils pcl viewer is turned off *
    ******************************************/
    while ( !visualizer.viewer->wasStopped())
    {
      visualizer.viewer->spinOnce(100);
      //viewer->saveScreenshot("screenshot.png");
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    return 0;
}
