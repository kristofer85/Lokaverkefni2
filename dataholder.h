#ifndef DATAHOLDER_H
#define DATAHOLDER_H
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>

using namespace cv;
using namespace std;
class DataHolder
{
public:
    DataHolder();

    FileStorage fs1;

};

#endif // DATAHOLDER_H
