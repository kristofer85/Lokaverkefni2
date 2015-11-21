#include "harrisdetector.h"
using namespace cv;
using namespace std;
HarrisDetector::HarrisDetector()
{
    neighbourhood = 3;
    aperture = 3;
    k = 0.01;
    maxStrength = 0.0;
    threshold = 0.01;
    nonMaxSize = 3;
    //setLocalMaxWindowSize(nonMaxSize);
}

// Compute Harris corners
void HarrisDetector::detect(const Mat& image)
{
    // Harris computation
    cvtColor( image, srcGray, CV_BGR2GRAY );
    cornerHarris(image,cornerStrength,neighbourhood,aperture, k);
    // internal threshold computation
    //minMaxLoc(cornerStrength,&minStrength,&maxStrength);
    // local maxima detection
    Mat dilated; // temporary image
    dilate(cornerStrength,dilated,Mat());
    compare(cornerStrength,dilated,
    localMax,CMP_EQ);
}

// Get the corner map from the computed Harris values
Mat HarrisDetector::getCornerMap(double qualityLevel)
{
    Mat cornerMap;
    // thresholding the corner strength
    threshold= qualityLevel*maxStrength;
    //threshold(cornerStrength,cornerTh,threshold,255,THRESH_BINARY);
    // convert to 8-bit image
    cornerTh.convertTo(cornerMap,CV_8U);
    // non-maxima suppression
    bitwise_and(cornerMap,localMax,cornerMap);
    return cornerMap;
}

// Get the feature points from the computed Harris values
void HarrisDetector::getCorners(std::vector<Point> &points,double qualityLevel)
{
    // Get the corner map
    Mat cornerMap= getCornerMap(qualityLevel);
    // Get the corners
    getCorners(points, cornerMap);
}
// Get the feature points from the computed corner map

void HarrisDetector::getCorners(std::vector<Point> &points,const Mat& cornerMap)
{
    // Iterate over the pixels to obtain all features
    for( int y = 0; y < cornerMap.rows; y++ )
    {
        const uchar* rowPtr = cornerMap.ptr<uchar>(y);
        for( int x = 0; x < cornerMap.cols; x++ )
        {
            // if it is a feature point
            if (rowPtr[x])
            {
                points.push_back(Point(x,y));
            }
        }
    }
}

// Draw circles at feature point locations on an image
void HarrisDetector::drawOnImage(Mat &image,const std::vector<Point> &points)
{
    std::vector<Point>::const_iterator it=points.begin();
    // for all corners
    while (it!=points.end())
    {
        // draw a circle at each corner location
        circle(image,*it,3,(255,255,255),1);
        ++it;
    }
}

/*
void HarrisDetector::myCornerHarris( Mat src,Mat dst, int blockSize, int ksize, double k, int borderType )
{
    src.getMat();
   dst.create( src.size(), CV_32F );
   dst.getMat();
   cornerEigenValsVecs( src, dst, blockSize, ksize, HARRIS, k, borderType);
}

void HarrisDetector::cornerEigenValsVecs( const Mat& src,Mat& eigenv, int block_size, int aperture_size, int op_type, double k = 0, int borderType=0 )
{
#ifdef HAVE_TEGRA_OPTIMIZATION
   if (tegra::cornerEigenValsVecs(src, eigenv, block_size, aperture_size,op_type, k, borderType))
       return;
#endif

   int depth = src.depth();
   double scale = (double)(1 << ((aperture_size > 0 ?aperture_size : 3) - 1)) * block_size;
   if( aperture_size < 0 )
       scale *= 2.;
   if( depth == CV_8U )
       scale *= 255.;
   scale = 1./scale;

   CV_Assert( src.type() == CV_8UC1 || src.type() == CV_32FC1 );

   Mat Dx, Dy;
   if( aperture_size > 0 )
    {
       Sobel( src, Dx, CV_32F, 1, 0, aperture_size, scale, 0, 0 );
       Sobel( src, Dy, CV_32F, 0, 1, aperture_size, scale, 0, 0 );
    }
   else
    {
       Scharr( src, Dx, CV_32F, 1, 0, scale, 0, 0 );
       Scharr( src, Dy, CV_32F, 0, 1, scale, 0, 0 );
    }

   Size size = src.size();
   Mat cov( size, CV_32FC3 );
   int i, j;

   for( i = 0; i < size.height; i++ )
    {
       float* cov_data = (float*)(cov.data + i*cov.step);
       const float* dxdata = (const float*)(Dx.data + i*Dx.step);
       const float* dydata = (const float*)(Dy.data + i*Dy.step);

        for( j = 0; j < size.width; j++ )
       {
           float dx = dxdata[j];
           float dy = dydata[j];

           cov_data[j*3] = dx*dx;
           cov_data[j*3+1] = dx*dy;
           cov_data[j*3+2] = dy*dy;
       }
    }

   boxFilter(cov, cov, cov.depth(), Size(block_size, block_size),Point(-1,-1), false, 0 );

   if( op_type == MINEIGENVAL )
       calcMinEigenVal( cov, eigenv );
   else if( op_type == HARRIS )
       calcHarris( cov, eigenv, k );
   else if( op_type == EIGENVALSVECS )
       calcEigenValsVecs( cov, eigenv );
}


threshold( Mat src,Mat dst, double thresh, double maxval, int type )
{
   src = _src.getMat();
   bool use_otsu = (type & THRESH_OTSU) != 0;
    type &= THRESH_MASK;

   if( use_otsu )
    {
       CV_Assert( src.type() == CV_8UC1 );
       thresh = getThreshVal_Otsu_8u(src);
    }

   dst.create( src.size(), src.type() );
   dst = dst.getMat();

   if( src.depth() == CV_8U )
    {
        int ithresh = cvFloor(thresh);
       thresh = ithresh;
       int imaxval = cvRound(maxval);
       if( type == THRESH_TRUNC )
           imaxval = ithresh;
       imaxval = saturate_cast<uchar>(imaxval);

       if( ithresh < 0 || ithresh >= 255 )
       {
           if( type == THRESH_BINARY || type == THRESH_BINARY_INV ||
                ((type == THRESH_TRUNC || type== THRESH_TOZERO_INV) && ithresh < 0) ||
                (type == THRESH_TOZERO&& ithresh >= 255) )
           {
               int v = type ==THRESH_BINARY ? (ithresh >= 255 ? 0 : imaxval) :
                        type ==THRESH_BINARY_INV ? (ithresh >= 255 ? imaxval : 0) :
                        //type == THRESH_TRUNC? imaxval :// 0;
                dst.setTo(v);
            }
           else
                src.copyTo(dst);
           return thresh;
       }
       thresh = ithresh;
       maxval = imaxval;
    }
   else if( src.depth() == CV_16S )
    {
       int ithresh = cvFloor(thresh);
       thresh = ithresh;
       int imaxval = cvRound(maxval);
       if( type == THRESH_TRUNC )
           imaxval = ithresh;
       imaxval = saturate_cast<short>(imaxval);

       if( ithresh < SHRT_MIN || ithresh >= SHRT_MAX )
       {
           if( type == THRESH_BINARY || type == THRESH_BINARY_INV ||
               ((type == THRESH_TRUNC || type== THRESH_TOZERO_INV) && ithresh < SHRT_MIN) ||
               (type == THRESH_TOZERO&& ithresh >= SHRT_MAX) )
           {
                int v = type == THRESH_BINARY ?(ithresh >= SHRT_MAX ? 0 : imaxval) :
                type == THRESH_BINARY_INV ?(ithresh >= SHRT_MAX ? imaxval : 0) :
                //type == THRESH_TRUNC ?imaxval :// 0;
                dst.setTo(v);
           }
           else
               src.copyTo(dst);
           return thresh;
       }
       thresh = ithresh;
       maxval = imaxval;
    }
   else if( src.depth() == CV_32F )
       ;
   else
       CV_Error( CV_StsUnsupportedFormat, "" );

   parallel_for_(Range(0, dst.rows),
                  ThresholdRunner(src, dst,thresh, maxval, type),
                 dst.total()/(double)(1<<16));
   return thresh;
}
/*
void HarrisDetector::adaptiveThreshold( Mat src, Mat dst, double maxValue,int method, int type, int blockSize, double delta )
{
   Mat src = _src.getMat();
   CV_Assert( src.type() == CV_8UC1 );
   CV_Assert( blockSize % 2 == 1 && blockSize > 1 );
   Size size = src.size();

  dst.create( size, src.type() );
   Mat dst = dst.getMat();

   if( maxValue < 0 )
    {
       dst = Scalar(0);
       return;
    }

   Mat mean;

   if( src.data != dst.data )
       mean = dst;

   if( method == ADAPTIVE_THRESH_MEAN_C )
       boxFilter( src, mean, src.type(), Size(blockSize, blockSize),
                   Point(-1,-1), true,BORDER_REPLICATE );
   else if( method == ADAPTIVE_THRESH_GAUSSIAN_C )
       GaussianBlur( src, mean, Size(blockSize, blockSize), 0, 0,BORDER_REPLICATE );
   else
       CV_Error( CV_StsBadFlag, "Unknown/unsupported adaptive thresholdmethod" );

   int i, j;
   uchar imaxval = saturate_cast<uchar>(maxValue);
   int idelta = type == THRESH_BINARY ? cvCeil(delta) : cvFloor(delta);
   uchar tab[768];

   if( type == CV_THRESH_BINARY )
       for( i = 0; i < 768; i++ )
           tab[i] = (uchar)(i - 255 > -idelta ? imaxval : 0);
   else if( type == CV_THRESH_BINARY_INV )
       for( i = 0; i < 768; i++ )
           tab[i] = (uchar)(i - 255 <= -idelta ? imaxval : 0);
   else
       CV_Error( CV_StsBadFlag, "Unknown/unsupported threshold type");

   if( src.isContinuous() && mean.isContinuous() &&dst.isContinuous() )
    {
       size.width *= size.height;
       size.height = 1;
    }

   for( i = 0; i < size.height; i++ )
    {
       const uchar* sdata = src.data + src.step*i;
       const uchar* mdata = mean.data + mean.step*i;
       uchar* ddata = dst.data + dst.step*i;

       for( j = 0; j < size.width; j++ )
           ddata[j] = tab[sdata[j] - mdata[j] + 255];
    }
}


void HarrisDetector::on_CornerHarris(  )
{
    //---------------------------【1】定义一些局部变量-----------------------------
    Mat dstImage;//目标图
    Mat normImage;//归一化后的图
    Mat scaledImage;//线性变换后的八位无符号整型的图

    //---------------------------【2】初始化---------------------------------------
    //置零当前需要显示的两幅图，即清除上一次调用此函数时他们的值
    dstImage = Mat::zeros( color.size(), CV_32FC1 );
    color=color.clone( );

    //---------------------------【3】正式检测-------------------------------------
    //进行角点检测
    cornerHarris( g_grayImage, dstImage, 2, 3, 0.04, BORDER_DEFAULT );

    // 归一化与转换
    normalize( dstImage, normImage, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
    convertScaleAbs( normImage, scaledImage );//将归一化后的图线性变换成8位无符号整型

    //---------------------------【4】进行绘制-------------------------------------
    // 将检测到的，且符合阈值条件的角点绘制出来
    for( int j = 0; j < normImage.rows ; j++ )
    { for( int i = 0; i < normImage.cols; i++ )
    {
        if( (int) normImage.at<float>(j,i) > thresh+80 )
        {
            circle( color, Point( i, j ), 5,  Scalar(10,10,255), 2, 8, 0 );
            circle( scaledImage, Point( i, j ), 5,  Scalar(0,10,255), 2, 8, 0 );
        }
    }
    }

}
*/




