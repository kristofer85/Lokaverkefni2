#ifndef STEREOCALIBRATE_H
#define STEREOCALIBRATE_H

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <string>
#include "dataholder.h"

//using namespace cv;
//using namespace std;

class StereoCalibrate
{
public:
    int numBoards;
    int board_w;
    int board_h;
    cv::Size board_sz;
    int board_n;
    std::vector<std::vector<cv::Point3f> > object_points,object_points1,object_points2;
    std::vector<std::vector<cv::Point2f> > imagePoints1, imagePoints2;
    std::vector<cv::Point2f> corners1, corners2;
    std::vector<cv::Point3f> obj;
    cv::Mat img1, img2, gray1, gray2;
    cv::Mat R1, R2, P1, P2, Q;
    cv::Mat map1x, map1y, map2x, map2y;
    cv::Mat imgU1, imgU2;
    cv::Mat CM1;// = Mat(3, 3, CV_32FC1);
    cv::Mat CM2;// = Mat(3, 3, CV_32FC1);
    cv::Mat D1, D2;
    cv::Mat R, T, E, F;
    cv::Size imgSize;

    StereoCalibrate();
    void findAndDrawChessBoardCorners();
    void findAndDrawChessBoardCorners(std::string images);
    void CalibrateStereoCamera();
    void rectifyCamera();
    void initUndistort();
};
#endif // STEREOCALIBRATE_H
