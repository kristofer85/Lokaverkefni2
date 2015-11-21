#ifndef IMAGEPROSSESSING_H
#define IMAGEPROSSESSING_H
#include <iostream>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>

class ImageProssessing
{
public:
    ImageProssessing();
    cv::Mat cannyEdges(cv::Mat src, int edgeThresh, int lowThreshold, int const max_lowThreshold, int ratio, int kernel_size);
    cv::Mat FindingContour(cv::Mat src);
    cv::Mat FindingLargestContour(cv::Mat src);
    bool isBorder(cv::Mat& edge, cv::Vec3b color);
    void autocrop(cv::Mat& src, cv::Mat& dst);
    cv::Mat srcGray;
};

#endif // IMAGEPROSSESSING_H
