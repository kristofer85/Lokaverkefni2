#ifndef DEPTHMAP_H
#define DEPTHMAP_H
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/core_c.h>
#include <opencv2/ccalib.hpp>
#include <opencv2/core/utility.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <limits.h>
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/hal/intrin.hpp"
#include <opencv2/stereo.hpp>
#include "opencv2/hal/intrin.hpp"
#include <opencv2/stereo.hpp>
#include "utils.h"
#include <limits.h>
#include "opencv2/stereo.hpp"
using namespace cv;
using namespace std;
using namespace cv::ximgproc;
class DepthMap
{
public:
    Mat left,right,g1,g2,disp,disp8,disp12,dispf,dispf8;
    string leftImage,rightImage,disparityImage;
    Size imSize;
    DepthMap();
    void run();
    void SGBMdisparityCalc(Mat l, Mat r);
    void BMdisparityCalc(Mat l, Mat r);
    void DisparityFilter(Mat l,Mat r);
    void SGBMBinary(Mat l,Mat r);
    Rect computeROI(Size2i src_sz, Ptr<StereoMatcher> matcher_instance);

};
#endif // DEPTHMAP_H
