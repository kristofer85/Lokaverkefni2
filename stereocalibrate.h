#ifndef STEREOCALIBRATE_H
#define STEREOCALIBRATE_H

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <string>
#include <limits.h>
#include "opencv2/hal/intrin.hpp"
#include <opencv2/stereo.hpp>

#include "utils.h"

class StereoCalibrate
{
public:
    int numBoards;
    int board_w;
    int board_h;
    cv::Size board_sz;
    int board_n;
    double patternSize;

    size_t fullResImage;
    size_t leftAndRightImages;

    std::vector<std::vector<cv::Point3f> > objectpoints;
    std::vector<std::vector<cv::Point2f> > imagePoints1, imagePoints2;
    std::vector<cv::Point2f> corners1, corners2;
    std::vector<cv::Point3f> obj;

    cv::Mat ChessHd,img1, img2, gray1, gray2, foundImages;

    cv::Mat R1, R2, P1, P2, Q;
    cv::Mat H1, H2;
    cv::Mat map1x, map1y, map2x, map2y;
    cv::Mat imgU1, imgU2;

    cv::Mat CM1;
    cv::Mat CM2;
    cv::Mat D1, D2;
    cv::Mat R, T, E, F;
    cv::Size imSize,cutSize,distortionSize,distortionCutSize;
    float zoom_valueC,focalResC;

    StereoCalibrate();
    void findAndDrawChessBoardCorners(std::string filename);
    void CalibrateStereoCamera();
    void rectifyCamera();
    void initUndistort(cv::Mat& l,cv::Mat& r);

};
#endif // STEREOCALIBRATE_H
