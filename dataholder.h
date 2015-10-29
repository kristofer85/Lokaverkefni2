#ifndef DATAHOLDER_H
#define DATAHOLDER_H
#include "defines.h"
#include <pcl/point_types.h>
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>


class DataHolder
{
public:
    DataHolder();
    //void setLeftImage(Mat left);
    //void setRightImage(Mat right);
    //void setDispImage(Mat disp);
    //Mat getLeftImage(Mat left);
    //Mat getRightImage(Mat right);
    //Mat getDispImage(Mat disp);


    cv::FileStorage fs1;
private:
    //Mat left, right, disp;

};

#endif // DATAHOLDER_H
