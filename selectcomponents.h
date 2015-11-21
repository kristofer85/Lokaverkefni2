#ifndef SELECTCOMPONENTS_H
#define SELECTCOMPONENTS_H
#include <iostream>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>

class SelectComponents
{
public:
    cv::Mat* image;
    cv::Vec2i selColor;
    cv::Vec2i* colorAt;
    bool crop;
    Rect top,right,bottom,left;
    int xMin, xMax, yMin, yMax;
    int* ptr;
    int startPos;
    SelectComponents();
    //SelectComponents(cv::Mat src,cv::Vec colorAt);
    bool isTop(cv::Mat src, cv::Vec3b colorAt);
    bool isRight(cv::Mat src, cv::Vec3b colorAt);
    bool isBottom(cv::Mat src, cv::Vec3b colorAt);
    bool isLeft(cv::Mat src, cv::Vec3b colorAt);
};

#endif // SELECTCOMPONENTS_H
