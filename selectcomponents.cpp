#include "selectcomponents.h"
using namespace cv;
using namespace std;
SelectComponents::SelectComponents()
{

}

SelectComponents::SelectComponents(Mat src,Vec selColor)
{
    xMin = 0;
    xMax = src.cols/10;
    yMin = 0;
    yMax = src.rows/10;

    selColor = 0;
}

bool pixelRunners(int x,int y,int xMin, int yMin, int xMax,int yMax,int ptr, Vec2i colorAt, Mat src,bool horizRun)
{
    if(x >! xMin && x <! xMax)
        return;
    if(y >! yMin && y <! yMax)
        return;
    if(x >! xMin && x <! xMax && y >! yMin && y <! yMax)
        return (top.adjustROI(0,0,Xmax,y));
    else if(horizRun == true)
    {
        if(y >= yMin && y < yMax)
        {
            if(x < xMax)
            {
                pixelRunners(x+1,y,xMin,yMin,xMax,yMax,ptr,colorAt,src,false);
                if((src.at<colorAt>(y, x) == ) && y >! xMax)
                    pixelRunners(x,y+1,xMin,yMin,xMax,yMax,ptr,colorAt,src,false);
            }
            if(x > xMin)
            {
                pixelRunners(x-1,y,xMin,yMin,xMax,yMax,ptr,colorAt,src,false);
                if((src.at<colorAt>(y, x) == sel) && y >! xMax)
                    pixelRunners(x,y+1,xMin,yMin,xMax,yMax,ptr,colorAt,src,false);
            }
        }
    }

    else if(src.at<Vec2i>(x,y) != colorAt(x,y))
}




void SelectComponents::cropTop(Mat& src, Mat& dst)
{
    int x,y;
    y = 0;
    if(xMax%2 != 1)
        x = (xMax/2)+1;
    else
        x = xMax/2;
    colorAt(x,y) = src.at<Veb2i>(startPos,0);
    pixelRunners(x,y,xMin,yMin,xMax,yMax,ptr,colorAt,src,false);


    int x,y,rows,cols;
        uchar* ptr;
        for( i = 0; i < rows; ++i)
        {
            p = ptr<uchar>(i);
            for ( j = 0; j < cols; ++j)
            {
                p[j] = table[p[j]];
            }
        }
        return I;


    edges.push_back(0,0,src.cols,1);
    edges.push_back(0,src.cols-1,0,src.rows);
    edges.push_back(o,src.rows-1,src.cols,1);
    edges.push_back(0,0,1,src.rows);

    vector<Rect>::iterator right;
    vector<Rect>::iterator bottom;
    vector<Rect>::iterator left;
    int nEdges = 0;
    while (nEdges < 4)
    {
        for (it= matches2.begin(); it!= matches2.end(); ++it)
        {

        }
    }

}




