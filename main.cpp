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
#include <iostream>
#include <string>
using namespace cv;
using namespace std;

int main(int argc, char *argv[])
{
    StereoCalibrate cc;
    cc.findAndDrawChessBoardCorners();
    cc.CalibrateStereoCamera();
    //cc.initUndistort();
    //cc.rectifyCamera();
    StereoScopicImage ssi;
    ssi.rectifyCamera();
    ssi.disparityMap();

    return 0;
}
