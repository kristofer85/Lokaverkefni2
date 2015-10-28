#include "stereoscopicimage.h"
using namespace cv;
using namespace std;

StereoScopicImage::StereoScopicImage()
{

}


/*******************************************
 * The functions in this section use a     *
 * so-called pinhole camera model. In this *
 * model, a scene view is formed by        *
 * projecting 3D points into the image     *
 * plane using a perspective               *
 * transformation.                         *
 ******************************************/
void StereoScopicImage::rectifyCamera()
{
    string img1Path = CALIBFOLDER;
    img1Path.append("leftChessbord.png");
    Mat img1 = imread(img1Path,IMREAD_COLOR);
    img1Path = CALIBFOLDER;
    img1Path.append("rightChessbord.png");
    Mat img2 = imread(img1Path,IMREAD_COLOR);
    Mat CM1, D1, CM2, D2,R, T, R1, P1, R2, P2;
    Rect roi1, roi2;
    Mat Q;
    DataHolder dataHolder;
    dataHolder.fs1.open("stereoCalibration.yml", FileStorage::READ);
    dataHolder.fs1["CM1"] >> CM1;
    dataHolder.fs1["D1"] >> D1;
    dataHolder.fs1["CM2"] >> CM2;
    dataHolder.fs1["D2"] >> D2;
    dataHolder.fs1["R"] >> R;
    dataHolder.fs1["T"] >> T;
    stereoRectify( CM1, D1, CM2, D2, img1.size(), R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img1.size(), &roi1, &roi2 );
    dataHolder.fs1.open("stereoCalibration.yml", FileStorage::APPEND);
    dataHolder.fs1 << "R1" << R1;
    dataHolder.fs1 << "R2" << R2;
    dataHolder.fs1 << "P1" << P1;
    dataHolder.fs1 << "P2" << P2;
    dataHolder.fs1 << "Q" << Q;
}

void StereoScopicImage::disparityMap()
{
    string img1Path = CALIBFOLDER;
    img1Path.append("leftChessbord.png");
    //Mat img1 = imread("C:/Users/Notandi/Pictures/kula calib myndir/stereoscopicLeft.jpg",IMREAD_COLOR);
    Mat img1 = imread(img1Path,IMREAD_COLOR);
    img1Path = CALIBFOLDER;
    img1Path.append("rightChessbord.png");
    //Mat img2 = imread("C:/Users/Notandi/Pictures/kula calib myndir/stereoscopicRight.jpg",IMREAD_COLOR);
    Mat img2 = imread(img1Path,IMREAD_COLOR);
    Mat g1, g2, disp, disp8;
    cvtColor(img1, g1, CV_BGR2GRAY);
    cvtColor(img2, g2, CV_BGR2GRAY);

    sgbm->setBlockSize(3);
    sgbm->setDisp12MaxDiff(1);
    sgbm->setMode(StereoSGBM::MODE_HH);
    sgbm->setMinDisparity(-64);
    sgbm->setNumDisparities(192);
    sgbm->setP1(600);
    sgbm->setP2(2400);
    sgbm->setPreFilterCap(4);
    sgbm->setSpeckleRange(32);


    sgbm->compute(g1, g2, disp);
    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);
    namedWindow("disp");

    imshow("disp", disp8);
    waitKey(0);
}

void StereoScopicImage::disparityMap(string filename)
{
    //configure the sgbm

    //read the file
    int SADWindowSize = 9, numberOfDisparities = 0;
    FileStorage fs;
    fs.open(filename, FileStorage::READ);

    if (!fs.isOpened())
    {
        cerr << "Failed to open " << filename << endl;
        //help(av);
        //return 0;
    }

    FileNode n = fs["images"];                         // Read string sequence - Get node
    if (n.type() != FileNode::SEQ)
    {
        cerr << "strings is not a sequence! FAIL" << endl;
        //return 1;
    }
    string bleh;
    Mat fullImg;
    Mat img1, img2;
    Mat g1, g2, disp, disp8;
    namedWindow("disp",WINDOW_NORMAL| WINDOW_KEEPRATIO);
    Size imgSize;
    FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
    for (; it != it_end; ++it)
    {
        //reed sterio pic from file
        //cout << (string)*it << endl;
        bleh = CALIBFOLDERFIXED;
        bleh.append((string)*it);
        //cout << bleh << endl;
        fullImg = imread(bleh,IMREAD_COLOR);
        //split sterio pic into two images
        Size imSize = fullImg.size();
        /*
        img1 = fullImg(Range(0, imSize.height),Range(0, imSize.width/2)).clone();

        img2 = fullImg(Range(0, imSize.height),Range(imSize.width/2, imSize.width)).clone();
        */
        Mat img1 = fullImg(Range(300, imSize.height-800),Range(650, imSize.width/2)).clone();

        Mat img2 = fullImg(Range(300, imSize.height-800),Range(imSize.width/2, imSize.width-650)).clone();
        //std::cout << "full width =" << fullImg.size().width << std::endl;
        //std::cout << "left width = " << img1.size().width << std::endl;
        //std::cout << "right width = " << img2.size().width << std::endl;
        imgSize = img1.size();

        cvtColor(img1, g1, CV_BGR2GRAY);
        cvtColor(img2, g2, CV_BGR2GRAY);

        //int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
        int sgbmWinSize = 9;
                      sgbm->setBlockSize(sgbmWinSize);

                        int cn = img1.channels();

        //sgbm->setBlockSize(3);
        sgbm->setDisp12MaxDiff(1);
        sgbm->setUniquenessRatio(1);
        sgbm->setMode(StereoSGBM::MODE_SGBM);
        sgbm->setMinDisparity(-64);
        sgbm->setNumDisparities(768);
        //sgbm->setP1(600);
        //sgbm->setP2(2400);
        sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
        sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
        sgbm->setPreFilterCap(4);
        sgbm->setSpeckleRange(32);
        sgbm->setSpeckleWindowSize(200);

        sgbm->compute(g1, g2, disp);
        //disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
        normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);


        imshow("disp", disp8);

        namedWindow("g1",WINDOW_NORMAL| WINDOW_KEEPRATIO);
        namedWindow("g2",WINDOW_NORMAL| WINDOW_KEEPRATIO);
        imshow("g1", g1);
        imshow("g2", g2);
        waitKey(0);
    }
}
