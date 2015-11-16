#include "utils.h"
using namespace cv;
using namespace std;
//a helper class containing functions from kula code that i modified to work with our project



void splitImage(cv::Mat fullImage,cv::Mat leftImg,cv::Mat rightImg)
{
    float midper = 0.1;
    Size imSize = fullImage.size();
    int midArea = imSize.width * midper;

    leftImg = fullImage(Range(0, imSize.height),Range(0, imSize.width/2 + midArea)).clone();

    rightImg = fullImage(Range(0, imSize.height),Range(imSize.width/2 - midArea, imSize.width)).clone();
}

double getFocalResolution(string imagePath)
{
    Exiv2::Image::AutoPtr image = Exiv2::ImageFactory::open(imagePath);
    if(image.get() == 0)
    {
            return 0;
    }
    image->readMetadata();

    Exiv2::ExifData &exifData = image->exifData();
    if (exifData.empty())
    {
            return 0;
    }
    Exiv2::ExifData::const_iterator end = exifData.end();
    for (Exiv2::ExifData::const_iterator i = exifData.begin(); i != end; ++i) {
            if(i->groupName() == "Photo" && i->tagName() == "FocalPlaneXResolution")
            {
                    return i->value().toFloat(0);
            }
    }
    return 0;
}

void tangent_distortion_correction(Mat src_mat, Mat * dst_mat, float left, float right)
{
        Mat map_x, map_y;
        map_x.create( src_mat.size(), CV_32FC1 );
        map_y.create( src_mat.size(), CV_32FC1 );
        for( int i = 0; i < src_mat.cols; i++ )
        {
                for( int j = 0; j < src_mat.rows; j++ )
                {
                        map_x.at<float>(j,i) = i;
                        map_y.at<float>(j,i) = ((j-0.5*src_mat.rows)*(left+(((right-left)*i)/src_mat.cols)))+(0.5*src_mat.rows);
                }
        }
        remap( src_mat, *dst_mat, map_x, map_y, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0, 0) );

}


int findSideBox(const Mat &image, double maxStdev,int numberOfLines, float maxSizeRatio, int maxColorDiff, double grayScaleSize, char
 selection)
{
       //selections: 0: top, 1: right, 2: bottom, 3: left.
       int firstRow, lastRow, firstColumn, lastColumn,numberOfPixels,outerBegin,outerEnd,innerBegin,innerEnd,incrementor;
       double meanRed,meanGreen,meanBlue,stdevRed,stdevGreen,stdevBlue;
       long sampleSumRed = 0;
       long sampleSumGreen = 0;
       long sampleSumBlue = 0;
       long sampleSumRed2 = 0;
       long sampleSumGreen2 = 0;
       long sampleSumBlue2 = 0;

       switch(selection)
       {
              //Find out wich case we are in and select the rows and columns of the
              //sample lines.
              case 0:
                    firstRow = 0;
                    lastRow = numberOfLines;
                    firstColumn = 0;
                    lastColumn = image.size().width-1;
                    outerBegin = 0;
                    outerEnd = image.size().height-1;
                    innerBegin = 0;
                    innerEnd = image.size().width-1;
                    incrementor = 1;
                    break;
              case 1:
                    firstRow = 0;
                    lastRow = image.size().height-1;
                    firstColumn = image.size().width-numberOfLines-1;
                    lastColumn =image.size().width -1;
                    outerBegin = image.size().width-1;
                    outerEnd = 0;
                    innerBegin = 0;
                    innerEnd = image.size().height-1;
                    incrementor = -1;
                    break;
              case 2:
                    firstRow = image.size().height - numberOfLines -1;
                    lastRow = image.size().height - 1;
                    firstColumn = 0;
                    lastColumn = image.size().width-1;
                    outerBegin = image.size().height-1;
                    outerEnd = 0;
                    innerBegin = 0;
                    innerEnd = image.size().width-1;
                    incrementor = -1;
                    break;
              case 3:
                    firstRow = 0;
                    lastRow = image.size().height-1;
                    firstColumn = 0;
                    lastColumn = numberOfLines;
                    outerBegin = 0;
                    outerEnd = image.size().width-1;
                    innerBegin = 0;
                    innerEnd = image.size().height-1;
                    incrementor = 1;
                    break;

       }
       //qDebug()<<"first loop of finding box";
       for(int n = firstRow; n <= lastRow;n++)
       {
              //Collect the sum and squared sum of the elements in the rows and columns we
              //selected.
             // qDebug()<<"inner loop 1 and n ="<<n;
              for(int m = firstColumn; m <= lastColumn; m++)
              {
                    Vec3b pointVector = image.at<Vec3b>(n,m);
                    sampleSumBlue += pointVector[0];
                    sampleSumBlue2 += pointVector[0]*pointVector[0];
                    sampleSumGreen += pointVector[1];
                    sampleSumGreen2 += pointVector[1]*pointVector[1];
                    sampleSumRed += pointVector[2];
                    sampleSumRed2 += pointVector[2]*pointVector[2];
              }
        }
        if (selection % 2 == 0)
        {
              //we are in case 0 or 2 so EF24-70mm f/2.8L USMwe are going verticaly
              numberOfPixels = image.size().width*(numberOfLines+1);
        }
        else
        {
              numberOfPixels = image.size().height*(numberOfLines+1);
        }
    //qDebug()<<"Second loop";
        for(int n = outerBegin; abs(n-outerBegin)<=abs(outerBegin - outerEnd)*maxSizeRatio; n+=incrementor)
        {
              long sumRed = sampleSumRed;
              long sumGreen = sampleSumGreen;
              long sumBlue = sampleSumBlue;
              long sumRed2 = sampleSumRed2;
              long sumGreen2 = sampleSumGreen2;
              long sumBlue2 = sampleSumBlue2;
             // qDebug()<<"mode:"<<(int)selection<<"second inner loop and n ="<<n;
              for(int m = 0; abs(m-innerBegin)<=abs(innerBegin - innerEnd);m++)
              {
                   // if( selection == 3) qDebug()<<"mode"<<selection<<"inside second inner loop"<<n<<m;
                    Vec3b pointVector;
                    if(selection % 2 == 0)
                    {
                          pointVector = image.at<Vec3b>(n,m);
                    }
                    else
                    {
                          pointVector = image.at<Vec3b>(m,n);
                    }
                    sumBlue += pointVector[0];
                    sumBlue2 += pointVector[0]*pointVector[0];
                    sumGreen += pointVector[1];
                    sumGreen2 += pointVector[1]*pointVector[1];
                    sumRed += pointVector[2];
                    sumRed2 += pointVector[2]*pointVector[2];
              }
              meanRed = sumRed/(double)numberOfPixels;
              meanGreen = sumGreen/(double)numberOfPixels;
              meanBlue = sumBlue/(double)numberOfPixels;

              stdevRed = sqrt((sumRed2/(double)numberOfPixels)-(meanRed*meanRed));
              stdevGreen = sqrt((sumGreen2/(double)numberOfPixels)-(meanGreen*meanGreen));
              stdevBlue = sqrt((sumBlue2/(double)numberOfPixels)-(meanBlue*meanBlue));

              if((stdevRed>maxStdev) || (stdevGreen>maxStdev) || (stdevBlue>maxStdev)
                || (max(max(meanRed,meanGreen),meanBlue)-min(min(meanRed,meanGreen),meanBlue)) > maxColorDiff
                ||  (meanRed+meanGreen+meanBlue)/3 > grayScaleSize*255)
              {
                      if (selection==0 || selection == 3)
                      {
                            return n;
                      }
                      if (selection == 1)
                      {
                            return image.size().width - n;
                      }
                      else return image.size().height - n;
              }
        }
        return 0;
}

matPair BorderRemoveal(matPair pair)
{

    int topL,bottomL,topR,bottomR,leftL,leftR,rightL,rightR,left,right,top,bottom;

    topL =    findSideBox(pair.left,5,0,0.4,10,0.5,0);
    bottomL = findSideBox(pair.left,5,0,0.4,10,0.5,2);
    leftL =   findSideBox(pair.left,5,0,0.4,10,0.5,3);
    rightL =  findSideBox(pair.left,5,0,0.4,10,0.5,1);
    topR =    findSideBox(pair.right,5,0,0.4,10,0.5,0);
    bottomR = findSideBox(pair.right,5,0,0.4,10,0.5,2);
    leftR =   findSideBox(pair.right,5,0,0.4,10,0.5,3);
    rightR =  findSideBox(pair.right,5,0,0.4,10,0.5,1);


    bottom = max(bottomL,bottomR);
    top = max(topL,topR);
    left = max(leftL,leftR);
    right = max(rightL,rightR);

    pair.left = cropImageBorders(pair.left,top,right,bottom,left);
    pair.right = cropImageBorders(pair.right,top,right,bottom,left);
    return pair;

}

Mat cropImageBorders(Mat image, int top, int right, int bottom, int left)
{
        qDebug()<<"croping borders";
        Mat subImage;
        subImage =image(Range(top,image.size().height-bottom), Range(left, image.size().width-right)).clone();
        return subImage;
}

Mat getCameraMatrix(float zoom, int width, int height)
{
       Mat cameraMatrix = Mat::zeros(3,3, CV_64F);
       cameraMatrix.at<double>(2,2) = 1.0;
       double fx,fy,cx,cy;
       //double focalResMm = 155.21693;
       fx = zoom;
       cx = width/2.0;
       cy = height/2.0;
       fy = fx;


       cameraMatrix.at<double>(0,0) = fx;
       cameraMatrix.at<double>(0,2) = cx;
       cameraMatrix.at<double>(1,1) = fy;
       cameraMatrix.at<double>(1,2) = cy;

       return cameraMatrix;
}

bool getDistCoeffs(Mat &distCoeffs, float zoom, string filename)
{
       distCoeffs = Mat::zeros(5,1, CV_64F);

       //distCoeffs.at<double>(0,0) = 3.6562459162927829e-01;
       //distCoeffs.at<double>(1,0) = 1.9073063052260267e+01;
       //distCoeffs.at<double>(2,0) = 0.0;
       //distCoeffs.at<double>(3,0) = 0.0;
       //distCoeffs.at<double>(4,0) = 1.9073063052260267e+01;
       distCoeffs.at<double>(2,0) = 0.0;
       distCoeffs.at<double>(3,0) = 0.0;
       double k1, k2, k3;
       string lens = "Sigma 17-35mm f/2.8-4 EX DG";


       //if(!getDistortionParameters(getLensName(filename),zoom,k1,k2,k3))
       if(!getDistortionParameters(lens,zoom,k1,k2,k3))
       {
               qDebug()<<"getDistortionParameters returned false";
               return false;
       }
       distCoeffs.at<double>(0,0) = k1;
       distCoeffs.at<double>(1,0) = k2;
       distCoeffs.at<double>(4,0) = k3;
       qDebug()<<"Found Coeffs k1:"<<k1<<", k2:"<<k2<<", k3:"<<k3;

       return true;
}

bool getDistortionParameters(string cameraName, double focal, double &k1, double &k2, double &k3)
{

        setlocale(LC_ALL, "");
        struct lfDatabase *ldb;
        ldb = lf_db_new();
        ldb->Load();
        cout << ldb->HomeDataDir << endl;

        const lfLens **lenses = ldb->FindLenses (NULL, NULL, cameraName.c_str());
        if(!lenses)
        {
                qDebug()<<"WARNING: did not find the lens used";
                return false;
        }
        lfLensCalibDistortion results;
        lenses[0]->InterpolateDistortion(focal, results);
        k1 = results.Terms[0];
        k2 = results.Terms[1];
        k3 = results.Terms[2];
        ldb->Destroy();
        qDebug()<<"Found distortion coeffs";
        return true;

}

Exiv2::Image::AutoPtr unicodeExiv2Open(QString srcPath, QString *linkPath)
{

        qDebug()<<"Exif: Opening image:"<<srcPath;
        qDebug()<<"linkPath:"<<*linkPath;
        return Exiv2::ImageFactory::open(srcPath.toStdString());
        /*
        if(stringIsAscii(srcPath))
        {
                *linkPath = "";
                return Exiv2::ImageFactory::open(srcPath.toStdString());
        }
        else
        {
                qDebug()<<"Exif: image path not ascii";
                QUuid imageUuid = QUuid::createUuid();
                *linkPath = QDir::temp().absoluteFilePath(imageUuid).append(".jpg");
                qDebug()<<"Linking image to:";
                qDebug()<<*linkPath;
                windowsSafeLink(QFileInfo(srcPath).absoluteFilePath(), *linkPath);
                Exiv2::Image::AutoPtr returnVal = Exiv2::ImageFactory::open(linkPath->toStdString());
                //QFile::remove(QDir::temp().absoluteFilePath(imageUuid)).append(".jpg");
                //QFile::remove(*linkPath);
                qDebug()<<"Exif: opened non ascii path";
                return returnVal;
        }
        */
}

float getZoomValue(string imagePath)
{
        qDebug()<<"getting zoom value";
        QString * linkPath = new QString("");
        Exiv2::Image::AutoPtr image = unicodeExiv2Open(imagePath.c_str(), linkPath);
        float zoom = -1.0;
        if(image.get() == 0)
        {
                qDebug()<<"Could not load image";
        }
        else
        {
                image->readMetadata();

                Exiv2::ExifData &exifData = image->exifData();
                if (exifData.empty()) {
                        qDebug()<<"No exif data found";
                }
                else
                {
                        Exiv2::ExifData::const_iterator end = exifData.end();
                        for (Exiv2::ExifData::const_iterator i = exifData.begin(); i != end; ++i) {
                                if(i->groupName() == "Photo" && i->tagName() == "FocalLength")
                                {
                                        qDebug()<<"Success";
                                        zoom = i->value().toFloat(0);
                                }
                        }
                }
        }
        /*
        if(*linkPath != "")
        {
                QFile::remove(*linkPath);
        }
*/
        return zoom;
}



