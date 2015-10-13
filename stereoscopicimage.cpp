#include "stereoscopicimage.h"

StereoScopicImage::StereoScopicImage()
{

}

void StereoScopicImage::rectifyCamera()
{
    Mat img1 = imread("C:/Users/kristinn/Documents/leftChessbord.png",IMREAD_COLOR);
    Mat img2 = imread("C:/Users/kristinn/Documents/rightChessbord.png",IMREAD_COLOR);
    Mat CM1, D1, CM2, D2,R, T, R1, P1, R2, P2;
    Rect roi1, roi2;
    Mat Q;
    DataHolder dataHolder;
    dataHolder.fs1.open("stereoCalibration.yml", FileStorage::READ);
    dataHolder.fs1["CM1"] >> CM1;
    dataHolder.fs1["D1"] >> D1;
    dataHolder.fs1["CM2"] >> CM2;
    dataHolder.fs1["D2"] >> D2;
    dataHolder.fs1["R"] >> R;
    dataHolder.fs1["T"] >> T;
    stereoRectify( CM1, D1, CM2, D2, img1.size(), R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img1.size(), &roi1, &roi2 );
    dataHolder.fs1.open("stereoCalibration.yml", FileStorage::APPEND);
    dataHolder.fs1 << "R1" << R1;
    dataHolder.fs1 << "R2" << R2;
    dataHolder.fs1 << "P1" << P1;
    dataHolder.fs1 << "P2" << P2;
    dataHolder.fs1 << "Q" << Q;
}

void StereoScopicImage::disparityMap()
{
    Mat img1 = imread("C:/Users/kristinn/Pictures/stereoscopicLeft.jpg",IMREAD_COLOR);
    Mat img2 = imread("C:/Users/kristinn/Pictures/stereoscopicRight.jpg",IMREAD_COLOR);
    Mat g1, g2, disp, disp8;
    cvtColor(img1, g1, CV_BGR2GRAY);
    cvtColor(img2, g2, CV_BGR2GRAY);

    sgbm->setBlockSize(3);
    sgbm->setDisp12MaxDiff(1);
    sgbm->setMode(StereoSGBM::MODE_HH);
    sgbm->setMinDisparity(-64);
    sgbm->setNumDisparities(192);
    sgbm->setP1(600);
    sgbm->setP2(2400);
    sgbm->setPreFilterCap(4);
    sgbm->setSpeckleRange(32);


    sgbm->compute(g1, g2, disp);
    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);
    namedWindow("disp");

    imshow("disp", disp8);
    waitKey(0);
}
