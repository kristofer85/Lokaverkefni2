#ifndef DATAHOLDER_H
#define DATAHOLDER_H
#include "defines.h"
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>


class DataHolder
{
public:
    DataHolder();

    cv::FileStorage fs1;

};

#endif // DATAHOLDER_H
