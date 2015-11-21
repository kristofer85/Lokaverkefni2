#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include "imageprossessing.h"
using namespace cv;
using namespace std;
ImageProssessing::ImageProssessing()
{

}

Mat ImageProssessing::cannyEdges(Mat src, int edgeThresh, int lowThreshold, int const max_lowThreshold, int ratio, int kernel_size)
{
    if(src.empty())
        cout<< "src image is empty";

    Mat srcGray, srcGrayInvert;
    Mat dst, detectEdges;

    dst.create(src.size(),src.type());
    if(dst.empty())
    {
        cout<< "dst image is empty";
        cout << src.channels() << src.type() << src.depth() <<src.size() << endl;
        cout << dst.channels() << dst.type() << dst.depth() <<dst.size() << endl;
    }
    cvtColor( src, srcGray, CV_BGR2GRAY );

    blur( srcGray, detectEdges, Size(3,3) );
    cv::threshold(srcGray,srcGray,128,255, cv::THRESH_BINARY_INV);
    Canny(detectEdges,detectEdges,lowThreshold,lowThreshold*ratio,kernel_size);

    dst = Scalar::all(0) - detectEdges;
    src.copyTo( dst, detectEdges);
    imwrite("a.png",dst);
    return dst;
}

Mat ImageProssessing::FindingContour(Mat src)
{
    Mat srcGray;
    cvtColor( src, srcGray, CV_BGR2GRAY );
    Canny(srcGray, srcGray, 100, 200, 3);

    vector<Vec4i> hierarchy;
    vector<vector<Point> > contours;
    vector<int> borders;

    RNG rng(12345);
    int largest_area=0;
    int largest_contour_index=0;
    Rect boundingRect;

    Mat draw = Mat::zeros(srcGray.size(),CV_8UC3);

    findContours( srcGray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    for( int i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( draw, contours, i, color, 2, 8, hierarchy, 0, Point() );
    }
    return src;
}


Mat ImageProssessing::FindingLargestContour(Mat src)
{
    if(src.empty() || (src.channels() == 1))
        cout << "src either empty or black&white image" << endl;
    cvtColor(src,srcGray, CV_BGR2GRAY);
    int largestArea = 0;
    int largestContourIndex = 0;
    vector<Vec4i> hierarchy;
    vector<vector<Point> > contours;
    vector<int> borders;
    Rect boundingRectangle;
    findContours( srcGray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    for( int i = 0; i< contours.size(); i++ )
    {
        //  Find the area of contour
        double a=contourArea( contours[i],false);
        if(a>largestArea){
            largestArea = a;
            cout<<i<<" area  "<< a << endl;
            // Store the index of largest contour
            largestContourIndex =i ;
            // Find the bounding rectangle for biggest contour
            boundingRectangle=boundingRect(contours[i]);
        }
    }
    Scalar color( 255,255,255);  // color of the contour in the
    //Draw the contour and rectangle
    drawContours( src, contours,largestContourIndex, color, CV_FILLED,8,hierarchy);
   rectangle(src, boundingRectangle,  Scalar(56,255,255),2, 8,0);
    imwrite("test.png",src);
    return src;
}

bool ImageProssessing::isBorder(Mat& edge, Vec3b color)
{
//    Mat image = edge.clone().reshape(0,1);
//
//    bool result = true;
//    for (int i = 0; i < image.cols; i++)
//        result &= (color == image.at<Vec3b>(0,i));
//
//    return result;
}

/**
 * Function to auto-cropping image
 *
 * Parameters:
 *   src   The source image
 *   dst   The destination image
 */
void ImageProssessing::autocrop(Mat& src, Mat& dst)
{
    //Rect win(0, 0, src.cols, src.rows);
    //
    //std::vector<Rect> edges;
    //edges.push_back(Rect(0, 0, src.cols, 1));
    //edges.push_back(Rect(src.cols-2, 0, 1, src.rows));
    //edges.push_back(Rect(0, src.rows-2, src.cols, 1));
    //edges.push_back(Rect(0, 0, 1, src.rows));
//    //
//    //Mat edge;
//    //int nborder = 0;
//    //Vec3b color = src.at<Vec3b>(0,0);
//    //
//    //for (int i = 0; i < edges.size(); ++i)
//    //{
//    //    edge = src(edges[i]);
//    //    nborder += is_border(edge, color);
//    //}
//    //
//    //if (nborder < 4)
//    //{
//    //    src.copyTo(dst);
//    //    return;
//    //}
//    //
//    bool next;
//
//    do {
//        edge = src(Rect(win.x, win.height-2, win.width, 1));
//        if (next = isBorder(edge, color))
//            win.height--;
//    }
//    while (next && win.height > 0);
//
//    do {
//        edge = src(Rect(win.width-2, win.y, 1, win.height));
//        if (next = isBorder(edge, color))
//            win.width--;
//    }
//    while (next && win.width > 0);
//
//    do {
//        edge = src(Rect(win.x, win.y, win.width, 1));
//        if (next = isBorder(edge, color))
//            win.y++, win.height--;
//    }
//    while (next && win.y <= src.rows);
//
//    do {
//        edge = src(Rect(win.x, win.y, 1, win.height));
//        if (next = isBorder(edge, color))
//            win.x++, win.width--;
//    }
//    while (next && win.x <= src.cols);
//
//    dst = src(win);
}
