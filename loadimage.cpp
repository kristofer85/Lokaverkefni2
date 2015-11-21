#include "loadimage.h"
#include "stereocalibrate.h"
using namespace cv;
using namespace std;
loadImage::loadImage()
{
    pair.fullImage = "orginal.JPG";
    pair.leftImage = "leftImage.png";
    pair.rightImage = "rightImage.png";
    pair.disparityImage = "dispImage.png";
    s.x = 0;
    s.y = 0;
}

void loadImage::loadImagesForDepthMap()
{
    pair.full = imread(pair.fullImage);
    pair.left = imread(pair.leftImage);
    pair.right = imread(pair.rightImage);
    pair.disparity = imread(pair.disparityImage);
    pair = splitImage(pair);
    stereoCalib.initUndistort(pair.left, pair.right);

    mySplitImage(pair.left,pair.leftImage,s);

    cout << "   x    "  << s.x << "   y     "  <<  s.y << endl;
    mySplitImage(pair.right,pair.rightImage,s);


}
