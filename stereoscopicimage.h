#ifndef STEREOSCOPICIMAGE_H
#define STEREOSCOPICIMAGE_H
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>
#include "dataholder.h"

#include <limits.h>
#include "opencv2/hal/intrin.hpp"
using namespace cv;
using namespace std;

class StereoScopicImage
{
public:

    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);
    StereoScopicImage();
    void rectifyCamera();
    void disparityMap();
};

#endif // STEREOSCOPICIMAGE_H
