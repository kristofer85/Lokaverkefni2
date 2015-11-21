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
#include <pcl/filters/sampling_surface_normal.h>
#include "imageprossessing.h"
#include "harrisdetector.h"
#include "selectcomponents.h"
#include "utils.h"
#include "loadimage.h"
using namespace cv;
using namespace std;

int main(int argc, char *argv[])
{

    //StereoCalibrate stereoCalibrate;
    ////stereoCalibrate.findAndDrawChessBoardCorners("Y.xml");
    ////stereoCalibrate.CalibrateStereoCamera();
    ////stereoCalibrate.rectifyCamera();
    //stereoCalibrate.initUndistort();
    DepthMap depthMap;
    depthMap.run();
    Convert utilities;
    Visualizer visualizer;
    //loadImage load;
    //load.loadImagesForDepthMap();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mainCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PolygonMesh triangles;
    Mat Q;
    string cameraCal = "stereoCalibration.yml";
    FileStorage fs = FileStorage(cameraCal, FileStorage::READ);
    fs["Q"] >> Q;
    Mat color = imread(depthMap.leftImage, CV_LOAD_IMAGE_COLOR);
    Mat disparity = imread(depthMap.disparityImage, CV_LOAD_IMAGE_GRAYSCALE);
    //******************************************************




    //img = splitt(img);

    //imwrite("tyt.png",img);
//    Mat image = img.clone();
//    cvtColor(image,image,CV_RGB2GRAY);
//    medianBlur(image,image,3);
//    threshold(image,image,1,255,CV_THRESH_BINARY_INV);
//     imwrite("ty.png",image);
//    vector<vector<Point> > contours;
//    Mat contourOutput = image.clone();
//    vector<Vec4i> hierarchy;
//    int largest_area=0;
//    findContours( image, contours, hierarchy, CV_RETR_LIST , CV_CHAIN_APPROX_SIMPLE,Point(-1,-1));
//    Rect bounding_rect;
//    Mat src,dst;
//    int largest_contour_index=0;
//    for( int i = 0; i< contours.size(); i++ ) // iterate through each contour.
//          {
//           int a=contourArea(contours[i],false);  //  Find the area of contour
//           if(a > 200)
//  {
//               largest_area=a;
//               largest_contour_index=i;                //Store the index of largest contour
//               bounding_rect=boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
//
//              rectangle(image, bounding_rect,  Scalar(255,0,0),10, 8,0);
//                   cout << a << "  area     " << i << " counter     " << bounding_rect.x << " x     " << bounding_rect.y  << "y     "<< bounding_rect.width << " with     " << bounding_rect.height<< "height     "<< endl;
//           }
//          }
//     src = img(Rect(0,0,1606,2848)).clone();
//imwrite("yubu.png",src);
//    Mat regions(image.size(), CV_8UC1);
//    dst = image.clone();
     // Draw the largest contour using previously stored index.
    //drawContours( image, contours,largest_contour_index, Scalar(255,255,255), 5,8,hierarchy);





//
//    Mat src = imread("FindContour.png",IMREAD_COLOR);
//    ImageProssessing imgPro;
//    disparity = imgPro.cannyEdges(color,2,5,100,3,3);
//    vector<int> borders;
//    Mat contour = src = imread("test.png",IMREAD_COLOR);
//    contour = imgPro.FindingContour(src);
//    contour = imgPro.FindingLargestContour(src);
//
//    HarrisDetector harris;
//    Mat srcGray;
//
//
// threshold(harris.cornerStrength,cornerHarris(color,harris.cornerStrength,3,3,0.01),threshold,255,THRESH_BINARY);
////Create Harris detector instance
//    srcGray.create(color.size(),color.type());
//    // Compute Harris values
//    harris.detect(color);
//    // Detect Harris corners
//    std::vector<cv::Point> pts;
//    harris.getCorners(pts,0.01);
//    // Draw Harris corners
//    harris.drawOnImage(color,pts);
//    //std::vector<cv::Point2f> corners;
//    //cv::goodFeaturesToTrack(color, corners, 500, 0.01, 10);
//    Mat cornerStrength;
//    Mat harrisCorner;
//    //threshold(cornerStrength, harrisCorners, 0.00001, 255, THRESH_BINARY);
//    imwrite("h1.png",harrisCorner);
//    cornerHarris(color, cornerStrength, 2, 3, 0.01);
//    imwrite("h2.png",cornerStrength);
//   // threshold( InputArray _src,OutputArray _dst, double thresh, double maxval, int type );
//   // adaptiveThreshold( InputArray_src, OutputArray _dst, double maxValue,int method, int type, int blockSize, double delta );
//
//
//    cornerHarris(color,srcGray,3,3,0.01,0);
//    imwrite("s.png",color);
//
//
//    src = imread("k.jpg",IMREAD_COLOR);
//    if (!src.data)
//        return -1;
//
//    Mat dst;
//    imgPro.autocrop(src, dst);
//
//    imwrite("src.png", src);
//    imwrite("dst.png", dst);
//    cout << dst.size() << dst.cols << dst.rows << endl;

    //******************************************************
    utilities.matToCloud(color,disparity,Q,mainCloud);
    if(visualizer.displayPoly == false && visualizer.displayPoints == true)
    {
        //mainCloud = utilities.SOR_filter(mainCloud);
        utilities.smoothNormals(mainCloud);
        visualizer.viewer = visualizer.displayPointCloudColor(mainCloud,color); // view point cloud
    }

    else if(visualizer.displayPoly == true && visualizer.displayPoints == false)
    {
        triangles = utilities.triangulate(mainCloud);
        //triangles = utilities.possitionMesh(mainCloud);
         visualizer.viewer = visualizer.displayPolyMesh(mainCloud,triangles,color); // view polyMesh
    }

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
