#ifndef HARRISDETECTOR_H
#define HARRISDETECTOR_H
#include <iostream>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>


using namespace cv;
using namespace std;
class HarrisDetector
{

private:

    // 32-bit float image of corner strength
    Mat cornerStrength;
    // 32-bit float image of thresholded corners
    cv::Mat cornerTh;
    // image of local maxima (internal)
    cv::Mat localMax;
    // size of neighborhood for derivatives smoothing
    int neighbourhood;
    // aperture for gradient computation
    int aperture;
    // Harris parameter
    double k;
    // maximum strength for threshold computation
    double maxStrength;
    // calculated threshold (internal)
    double threshold;
    // size of neighborhood for non-max suppression
    int nonMaxSize;
    // kernel for non-max suppression
    cv::Mat kernel;


public:


    cv::Mat harrisCorners;
    HarrisDetector();
    void drawOnImage(cv::Mat &image,const std::vector<cv::Point> &points);
    cv::Mat getCornerMap(double qualityLevel);
    void getCorners(std::vector<cv::Point> &points,const cv::Mat& cornerMap);
    void getCorners(std::vector<cv::Point> &points,double qualityLevel);
    void detect(const cv::Mat& image);
    void myCornerHarris( cv::Mat src,cv::Mat dst, int blockSize, int ksize, double k, int borderType );
    void cornerEigenValsVecs( const cv::Mat& src,cv::Mat& eigenv, int block_size, int aperture_size, int op_type, double k=0., int borderType=0 );
    void on_CornerHarris(  );
    cv::Mat srcGray;


};


double cv::pointPolygonTest( cv::InputArray contour, cv::Point2f pt, bool measureDist );
#endif // HARRISDETECTOR_H
