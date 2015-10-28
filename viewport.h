#ifndef VIEWPORT_H
#define VIEWPORT_H

#include <pcl/visualization/pcl_visualizer.h>

class ViewPort
{
public:
    ViewPort();
    pcl::visualization::PCLVisualizer::Ptr leftView;
    pcl::visualization::PCLVisualizer::Ptr rightView;
    pcl::visualization::PCLVisualizer::Ptr centerView;
    pcl::visualization::PCLVisualizer::Ptr anyView;
    double xmin,xmax,ymin,ymax;
    int* viewport;

    void createAndPositionViewPort();
    void createCameraInViewPort();
};

#endif // VIEWPORT_H

