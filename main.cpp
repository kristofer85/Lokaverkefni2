#include "mainwindow.h"
#include <QApplication>

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
    ssi.convertTo3D();
    return 0;
}
