#ifndef STEREOCALIBRATE_H
#define STEREOCALIBRATE_H

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>
#include "dataholder.h"

using namespace cv;
using namespace std;

class StereoCalibrate
{
public:
    int numBoards;
    int board_w;
    int board_h;
    Size board_sz;
    int board_n;
    vector<vector<Point3f> > object_points;
    vector<vector<Point2f> > imagePoints1, imagePoints2;
    vector<Point2f> corners1, corners2;
    vector<Point3f> obj;
    Mat img1, img2, gray1, gray2;
    Mat R1, R2, P1, P2, Q;
    Mat map1x, map1y, map2x, map2y;
    Mat imgU1, imgU2;
    Mat CM1;// = Mat(3, 3, CV_32FC1);
    Mat CM2;// = Mat(3, 3, CV_32FC1);
    Mat D1, D2;
    Mat R, T, E, F;

    StereoCalibrate();
    void findAndDrawChessBoardCorners();
    void CalibrateStereoCamera();
    void rectifyCamera();
    void initUndistort();
};
#endif // STEREOCALIBRATE_H
