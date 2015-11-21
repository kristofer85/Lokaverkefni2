#ifndef LOADIMAGE_H
#define LOADIMAGE_H
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <utils.h>
#include "stereocalibrate.h"
class loadImage
{
public:
    std::string fullImage,leftImage,rightImage,disparityImage;
    matPair pair;
    cv::Point s;
    StereoCalibrate stereoCalib;
    std::vector<cv::Mat> imagePair;
    loadImage();
    void loadImagesForDepthMap();

};

#endif // LOADIMAGE_H
