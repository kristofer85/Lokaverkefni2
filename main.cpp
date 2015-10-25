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

using namespace cv;
using namespace std;
void bleh()
{
    Mat fullImg;
    string bleh = "C:/Users/Notandi/Pictures/kula_calib_myndir/calibMyndir_fixed/test/calib4_fixed.jpg";
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

int main(int argc, char *argv[])
{
    StereoCalibrate cc;
    //cc.findAndDrawChessBoardCorners();
    //bleh();


    cc.findAndDrawChessBoardCorners("C:/Users/Notandi/Documents/GitHub/Lokaverkefni2/X.xml");
    cc.CalibrateStereoCamera();
    //cc.initUndistort();
    //cc.rectifyCamera();
    StereoScopicImage ssi;
    ssi.rectifyCamera();
    //ssi.disparityMap();
    //ssi.disparityMap("C:/Users/Notandi/Documents/GitHub/Lokaverkefni2/X.xml");

    //Convert
    //untill sterio calibration is complete use these test images
    Mat img_rgb = imread("C:/Users/Notandi/Pictures/Screenshots/left.png", CV_LOAD_IMAGE_COLOR);
    Mat img_disparity = imread("C:/Users/Notandi/Pictures/Screenshots/disp.jpg", CV_LOAD_IMAGE_GRAYSCALE);
    Convert con(img_rgb,img_disparity);

    return 0;
}
