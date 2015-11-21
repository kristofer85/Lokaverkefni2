#ifndef UTILS_H
#define UTILS_H
#include "defines.h"
#include <string>
#include <iostream>
#include <opencv/cv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "json/json.h"
#include "json/value.h"
#include <QDebug>
#include <vector>
#include <cmath>

#include "exiv2/exiv2.hpp"
#include <vector>
//#include "lensfun.h"
#include "lensfun/lensfun.h"
#include <locale.h>
//#include <glib.h>
#include <getopt.h>
#define DISTORTION 0.0694
struct matPair
{
    cv::Mat full;
   cv::Mat left;
   cv::Mat right;
   cv::Mat disparity;
   std::string fullImage,leftImage,rightImage,disparityImage;
   matPair()
   {

   }
};

double limit_precision(double val, int precision);
float limit_precision2(float val, int precision);

cv::Mat limit_precision_mat(cv::Mat M, int precision);
cv::Mat limit_precision_matF(cv::Mat M, int precision);

    //matPair splitImage(cv::Mat fullImage);
    matPair splitImage(matPair pair);
    //this only works for nikon D90 to do implement lensfun
    //to do implement lensfun and exiv2
    bool getDistCoeffs(cv::Mat &distCoeffs, float zoom, std::string filename);
    bool getDistortionParameters(std::string lens, double focal, double &k1, double &k2, double &k3);
    double getFocalResolution(std::string imagePath);
    void tangent_distortion_correction(cv::Mat src_mat, cv::Mat * dst_mat, float left, float right);
    int findSideBox(const cv::Mat &image, double maxStdev,int numberOfLines, float maxSizeRatio, int maxColorDiff, double grayScaleSize, char
     selection);
    matPair BorderRemoveal(matPair pair);
    cv::Mat cropImageBorders(cv::Mat image, int top, int right, int bottom, int left);
    cv::Mat getCameraMatrix(float zoom, int width, int height);
    Exiv2::Image::AutoPtr unicodeExiv2Open(QString srcPath, QString *linkPath);
    float getZoomValue(std::string imagePath);
    void keystone(cv::Mat src, cv::Mat dst);
    cv::Ptr<cv::StereoMatcher> createRightMatcher2(cv::Ptr<cv::StereoMatcher> matcher_left);
    void shift_image(cv::Mat src_mat, cv::Mat * dst_mat, float left, float right);
    void proccess(std::string imagepath);
    matPair undestort(matPair mats);
    cv::Mat undestortZoom(cv::Mat image,std::string file);
    matPair Proccess_image(std::string impath);
    //std::string cameraName;
    //std::string lens;
    cv::Mat mySplitImage(cv::Mat& src,std::string name,cv::Point& s);


#endif // UTILS_H
