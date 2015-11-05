#include "stereocalibrate.h"
#include <opencv2/core/utility.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/core_c.h>
#include <opencv2/ccalib.hpp>
#include <opencv2/stereo/stereo.hpp>
using namespace cv;
using namespace std;
using namespace cv::ximgproc;


StereoCalibrate::StereoCalibrate()
{
    //fs1 = FileStorage("stereoCalibration.yml", FileStorage::WRITE);
}


void StereoCalibrate::findAndDrawChessBoardCorners(string filename)
{
    numBoards = 1;
    board_w = 7;
    board_h = 10;
    board_sz = Size(board_w, board_h);
    board_n = board_w*board_h;
    for (int j=0; j<board_n; j++)
    {
        obj.push_back(Point3f(j/board_w, j%board_w, 0.0f));
    }

    //namedWindow( "Left Window",WINDOW_NORMAL| WINDOW_KEEPRATIO );
    //namedWindow( "Right Window",WINDOW_NORMAL| WINDOW_KEEPRATIO );
    cout << endl << "Reading calib pics: " << endl;
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
    FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
    for (; it != it_end; ++it)
    {
        //reed sterio pic from file
        //cout << (string)*it << endl;
        bleh = CALIBFOLDERFIXED;
        //bleh = CALIBFOLDER;
        bleh.append((string)*it);
        //cout << bleh << endl;
        fullImg = imread(bleh,IMREAD_COLOR);
        pyrDown(fullImg,fullImg,Size(fullImg.cols/2,fullImg.rows/2));
        //split sterio pic into two images
        Size imSize = fullImg.size();

        img1 = fullImg(Range(0, imSize.height),Range(0, imSize.width/2)).clone();

        img2 = fullImg(Range(0, imSize.height),Range(imSize.width/2, imSize.width)).clone();

        /*
        Mat img1 = fullImg(Range(300, imSize.height-800),Range(650, imSize.width/2)).clone();

        Mat img2 = fullImg(Range(300, imSize.height-800),Range(imSize.width/2, imSize.width-650)).clone();
        */
        std::cout << "full width =" << fullImg.size().width << std::endl;
        std::cout << "left width = " << img1.size().width << " left height "<< img1.size().height << std::endl;
        std::cout << "right width = " << img2.size().width << std::endl;
        imgSize = img1.size();
        int success = 0, k = 0;
        bool found1 = false, found2 = false;

        while (success < numBoards)
        {
            cvtColor(img1, gray1, CV_BGR2GRAY);
            cvtColor(img2, gray2, CV_BGR2GRAY);

            //found1 = findChessboardCorners(img1, board_sz, corners1, CV_CALIB_CB_ADAPTIVE_THRESH  + CV_CALIB_CB_FAST_CHECK + CV_CALIB_CB_NORMALIZE_IMAGE + CV_CALIB_CB_FILTER_QUADS);
            //found2 = findChessboardCorners(img2, board_sz, corners2, CV_CALIB_CB_ADAPTIVE_THRESH  + CV_CALIB_CB_FAST_CHECK + CV_CALIB_CB_NORMALIZE_IMAGE + CV_CALIB_CB_FILTER_QUADS);
            found1 = findChessboardCorners(img1, board_sz, corners1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
            found2 = findChessboardCorners(img2, board_sz, corners2, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

            if (found1)
            {
                cornerSubPix(gray1, corners1, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
                drawChessboardCorners(gray1, board_sz, corners1, found1);
            }

            if (found2)
            {
                cornerSubPix(gray2, corners2, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
                drawChessboardCorners(gray2, board_sz, corners2, found2);
            }
            //for debuging
/*
            cout << (string)*it << endl;
            imshow("Left Window", gray1);
            imshow("Right Window", gray2);

            k = waitKey(10);
            if (found1 && found2)
            {
                k = waitKey(0);
            }
            if (k == 27)
            {
                break;
            }
*/
            if (found1 !=0 && found2 != 0)
            {
                //cout << "found" << endl;
                imagePoints1.push_back(corners1);
                imagePoints2.push_back(corners2);
                object_points.push_back(obj);
                success++;

                if (success >= numBoards)
                {

                    break;
                }
            }
        }
    }
    destroyAllWindows();
}


void StereoCalibrate::findAndDrawChessBoardCorners()
{
    numBoards = 1;
    board_w = 16;
    board_h = 9;
    board_sz = Size(board_w, board_h);
    board_n = board_w*board_h;
    for (int j=0; j<board_n; j++)
    {
        obj.push_back(Point3f(j/board_w, j%board_w, 0.0f));
    }


    string img1Path = CALIBFOLDER;
    img1Path.append("leftChessbord.png");
    cout << "bleh path = " << img1Path << endl;
    img1 = imread(img1Path,IMREAD_COLOR);
    img1Path = CALIBFOLDER;
    img1Path.append("rightChessbord.png");
    img2 = imread(img1Path,IMREAD_COLOR);
    int success = 0, k = 0;
    bool found1 = false, found2 = false;

    while (success < numBoards)
    {
        cvtColor(img1, gray1, CV_BGR2GRAY);
        cvtColor(img2, gray2, CV_BGR2GRAY);

        found1 = findChessboardCorners(img1, board_sz, corners1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
        found2 = findChessboardCorners(img2, board_sz, corners2, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

        if (found1)
        {
            cornerSubPix(gray1, corners1, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            drawChessboardCorners(gray1, board_sz, corners1, found1);
        }

        if (found2)
        {
            cornerSubPix(gray2, corners2, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            drawChessboardCorners(gray2, board_sz, corners2, found2);
        }

        //imshow("image1", gray1);
        //imshow("image2", gray2);

        k = waitKey(10);
        if (found1 && found2)
        {
            k = waitKey(0);
        }
        if (k == 27)
        {
            break;
        }
        if (found1 !=0 && found2 != 0)
        {
            imagePoints1.push_back(corners1);
            imagePoints2.push_back(corners2);
            object_points.push_back(obj);
            success++;

            if (success >= numBoards)
            {
                destroyAllWindows();
                break;
            }
        }
    }
}

void StereoCalibrate::CalibrateStereoCamera()
{
    CM1 = getCameraMatrix(zoom_valueC*focalResC, imgSize.width, imgSize.height);
    CM2 = getCameraMatrix(zoom_valueC*focalResC, imgSize.width, imgSize.height);
    /*
    cout << "starting camera calibration" << endl;
    CM1 = Mat::zeros(3, 3, CV_64FC1);
    CM2 = Mat::zeros(3, 3, CV_64FC1);
    D1 = Mat::zeros(8, 1, CV_64F);
    D2 = Mat::zeros(8, 1, CV_64F);
    Mat D11 = Mat::zeros(8, 1, CV_64F);
    Mat D22 = Mat::zeros(8, 1, CV_64F);

    //CM1 = getDefaultNewCameraMatrix(CM1,imgSize,false);
    //CM2 = getDefaultNewCameraMatrix(CM2,imgSize,false);
    cout << "starting CM1" << CM1 << endl;
    cout << "starting CM1" << CM2 << endl;

    //string img1Path = CALIBFOLDER;
    //img1Path.append("leftChessbord.png");
    //cout << "bleh path = " << img1Path << endl;
    //img1 = imread(img1Path,IMREAD_COLOR);

    string bleh = CALIBFOLDERFIXED;
    bleh.append("calib2_fixed.JPG");
    Mat fullImg = imread(bleh,IMREAD_COLOR);
    pyrDown(fullImg,fullImg,Size(fullImg.cols/2,fullImg.rows/2));
    //split sterio pic into two images
    //Size imSize = fullImg.size();
    //img1 = fullImg(Range(0, imSize.height),Range(0, imSize.width/2)).clone();
    Size imageSize = imgSize;

      double apertureWidth = 4.0;
      double apertureHeight = 4.0;
      double fieldOfViewX;
      double fieldOfViewY;
      double focalLength = 25.0;
      cv::Point2d principalPoint;
      double aspectRatio;
      double fx = (imageSize.width*25.0)/23.6;
      double fy = (imageSize.height*25.0)/15.8;
      CM1.at<double>(0,0) = fx;
      //CM1.at<double>(0,0) = 10.0;
      CM1.at<double>(1,1) = fy;
      //CM1.at<double>(1,1) = 5.0;
      CM1.at<double>(0,2) = imageSize.width/2;
      CM1.at<double>(1,2) = imageSize.height/2;
      CM1.at<double>(2,2) = 1;

      CM2.at<double>(0,0) = fx;
      //CM1.at<double>(0,0) = 10.0;
      CM2.at<double>(1,1) = fy;
      //CM1.at<double>(1,1) = 5.0;
      CM2.at<double>(0,2) = imageSize.width/2;
      CM2.at<double>(1,2) = imageSize.height/2;
      CM2.at<double>(2,2) = 1;
      cout << CM1 << endl;
      cout << CM2 << endl;

      DataHolder dataHolder2;
      dataHolder2.fs1 = FileStorage("cam.yml", FileStorage::WRITE);
      dataHolder2.fs1 << "CM1 before" << CM1;
      dataHolder2.fs1 << "CM2 before" << CM2;
      //calibrationMatrixValues(CM1, imageSize, apertureWidth, apertureHeight, fieldOfViewX, fieldOfViewY, focalLength, principalPoint, aspectRatio);
      //calibrationMatrixValues(CM2, imageSize, apertureWidth, apertureHeight, fieldOfViewX, fieldOfViewY, focalLength, principalPoint, aspectRatio);

      dataHolder2.fs1 << "CM1 after" << CM1;
      dataHolder2.fs1 << "CM2 after" << CM2;
*/
    //vector<Mat> rvecs,rvecs2, tvecs,tvecs2;
    //calibrateCamera(object_points, imagePoints1,imgSize,CM1, D11,rvecs,tvecs,CV_CALIB_ZERO_TANGENT_DIST |CV_CALIB_USE_INTRINSIC_GUESS);
    //calibrateCamera(object_points, imagePoints2,imgSize,CM2, D22,rvecs2,tvecs2,CV_CALIB_ZERO_TANGENT_DIST |CV_CALIB_USE_INTRINSIC_GUESS );
    //calibrateCamera(object_points, imagePoints1,imgSize,CM2, D11,rvecs,tvecs,CV_CALIB_ZERO_TANGENT_DIST |CV_CALIB_USE_INTRINSIC_GUESS);
    //calibrateCamera(object_points, imagePoints2,imgSize,CM1, D22,rvecs2,tvecs2,CV_CALIB_ZERO_TANGENT_DIST |CV_CALIB_USE_INTRINSIC_GUESS );

    stereoCalibrate(object_points, imagePoints1, imagePoints2,CM1, D1, CM2, D2, imgSize, R, T, E, F, CV_CALIB_FIX_FOCAL_LENGTH+CV_CALIB_ZERO_TANGENT_DIST+CV_CALIB_USE_INTRINSIC_GUESS,cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5));
    //stereoCalibrate(object_points, imagePoints2, imagePoints1,CM1, D1, CM2, D2, imgSize, R, T, E, F, CV_CALIB_FIX_INTRINSIC,cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5));


    //stereoCalibrate(object_points, imagePoints1, imagePoints2,CM1, D1, CM2, D2, img1.size(), R, T, E, F, CV_CALIB_USE_INTRINSIC_GUESS,cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5));
      //stereoCalibrate(object_points, imagePoints1, imagePoints2,CM1, D1, CM2, D2, img1.size(), R, T, E, F,CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST | CV_CALIB_FIX_ASPECT_RATIO | CV_CALIB_FIX_INTRINSIC ,cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5));
      //stereoCalibrate(object_points, imagePoints1, imagePoints2,CM1, D1, CM2, D2, img1.size(), R, T, E, F,CV_CALIB_SAME_FOCAL_LENGTH ,cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5));
      cout << CM1 << endl;
      cout << CM2 << endl;


//*******************************
// ToDo Write Data to DATA_HOLDER
//*******************************
    DataHolder dataHolder;
    dataHolder.fs1 = FileStorage("stereoCalibration.yml", FileStorage::WRITE);
    dataHolder.fs1 << "CM1" << CM1;
    dataHolder.fs1 << "CM2" << CM2;
    dataHolder.fs1 << "D1" << D1;
    dataHolder.fs1 << "D2" << D2;
    dataHolder.fs1 << "R" << R;
    dataHolder.fs1 << "T" << T;
    dataHolder.fs1 << "E" << E;
    dataHolder.fs1 << "F" << F;
}

void StereoCalibrate::rectifyCamera()
{
    stereoRectify(CM1, D1, CM2, D2, img1.size(), R, T, R1, R2, P1, P2, Q,CALIB_ZERO_DISPARITY);
//*******************************
// ToDo Write Data to DATA_HOLDER
//*******************************
    DataHolder dataHolder;
    dataHolder.fs1 = FileStorage("stereoCalibration.yml", FileStorage::WRITE);
    dataHolder.fs1 << "R1" << R1;
    dataHolder.fs1 << "R2" << R2;
    dataHolder.fs1 << "P1" << P1;
    dataHolder.fs1 << "P2" << P2;
    dataHolder.fs1 << "Q" << Q;
    dataHolder.fs1.release();
}
/*
void StereoCalibrate::initUndistort()
{
    initUndistortRectifyMap(CM1, D1, R1, P1, img1.size(), CV_32FC1, map1x, map1y);
    initUndistortRectifyMap(CM2, D2, R2, P2, img1.size(), CV_32FC1, map2x, map2y);

    remap(img1, imgU1, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
    remap(img2, imgU2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());

    imshow("image1", imgU1);
    imshow("image2", imgU2);
    string img1Path = CALIBFOLDER;
    img1Path.append("leftChessbordW.png");
    imwrite(img1Path,imgU1);
    img1Path = CALIBFOLDER;
    img1Path.append("rightChessbordW.png");
    imwrite(img1Path,imgU2);
    int k = waitKey(0);

    if(k==27)
    {
        destroyAllWindows();
    }

}
*/
Rect computeROI(Size2i src_sz, Ptr<StereoMatcher> matcher_instance)
{
    int min_disparity = matcher_instance->getMinDisparity();
    int num_disparities = matcher_instance->getNumDisparities();
    int block_size = matcher_instance->getBlockSize();

    int bs2 = block_size/2;
    int minD = min_disparity, maxD = min_disparity + num_disparities - 1;

    int xmin = maxD + bs2;
    int xmax = src_sz.width + minD - bs2;
    int ymin = bs2;
    int ymax = src_sz.height - bs2;

    Rect r(xmin, ymin, xmax - xmin, ymax - ymin);
    return r;
}

void StereoCalibrate::initUndistort()
{


    cout << "starting remaping" << endl;

    string bleh = CALIBFOLDERFIXED;
    //string bleh = CALIBFOLDER;
    bleh.append("calib3_fixed.JPG");
    //bleh.append("hestur.JPG");
    Mat fullImg = imread(bleh,IMREAD_COLOR);
    //pyrDown(fullImg,fullImg,Size(fullImg.cols/2,fullImg.rows/2));
    //split sterio pic into two images
    Size imSize = fullImg.size();

    Mat img1 = fullImg(Range(0, imSize.height),Range(0, imSize.width/2)).clone();
    pyrDown(img1,img1,Size(img1.cols/2,img1.rows/2));

    Mat img2 = fullImg(Range(0, imSize.height),Range(imSize.width/2, imSize.width)).clone();
    pyrDown(img2,img2,Size(img2.cols/2,img2.rows/2));

    /*
    Mat img1 = fullImg(Range(300, imSize.height-800),Range(650, imSize.width/2)).clone();

    Mat img2 = fullImg(Range(300, imSize.height-800),Range(imSize.width/2, imSize.width-650)).clone();
    */
    //std::cout << "full width =" << fullImg.size().width << std::endl;
    //std::cout << "left width = " << img1.size().width << std::endl;
    //std::cout << "right width = " << img2.size().width << std::endl;
    imgSize = img1.size();

    cout << "CM1"<< CM1 << endl;
    cout << "CM2"<< CM2 << endl;
    //stereoRectify(CM1, D1, CM2, D2, img1.size(), R, T, R1, R2, P1, P2, Q,CV_CALIB_ZERO_DISPARITY);
    rectifyCamera();
    initUndistortRectifyMap(CM1, D1, R1, P1, img1.size(), CV_32FC1, map1x, map1y);
    initUndistortRectifyMap(CM2, D2, R2, P2, img1.size(), CV_32FC1, map2x, map2y);

    remap(img1, imgU1, map1x, map1y, INTER_CUBIC, BORDER_CONSTANT, Scalar());
    remap(img2, imgU2, map2x, map2y, INTER_CUBIC, BORDER_CONSTANT, Scalar());

    namedWindow("image1",WINDOW_AUTOSIZE);
    namedWindow("image2",WINDOW_AUTOSIZE);


    namedWindow("disp",WINDOW_KEEPRATIO);


    imshow("image1", imgU1);
    imshow("image2", imgU2);

    /*
    string img1Path = CALIBFOLDER;
    img1Path.append("leftChessbordW.jpg");
    imwrite(img1Path,imgU1);
    img1Path = CALIBFOLDER;
    img1Path.append("rightChessbordW.jpg");
    imwrite(img1Path,imgU2);
    */

    int ndisp = (int) ((float) img1.size().width / 3.14f);
    if ((ndisp & 1) != 0)
    {
        ndisp++;
    }
    cout << "estimated nr of disparencys Disparities " << ndisp << endl;

    imwrite("../Lokaverkefni2/myndir/test.jpg",img1);
    Mat g1,g2,disp,disp8,disp12,disp2,disp28;
    cvtColor(img1, g1, CV_BGR2GRAY);
    cvtColor(img2, g2, CV_BGR2GRAY);
    imshow("image1", g1);
    imshow("image2", g2);

    cv::Ptr<cv::stereo::StereoBinarySGBM> sgbm2 = cv::stereo::StereoBinarySGBM::create(0,16,3);
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0,16,3);
    int sgbmWinSize = 3;
                  sgbm2->setBlockSize(sgbmWinSize);

                    int cn = img1.channels();
    cout << "channels "<< cn << endl;
    //sgbm->setBlockSize(3);
    sgbm2->setDisp12MaxDiff(5);
    sgbm2->setUniquenessRatio(5);
    //sgbm->setMode(StereoSGBM::MODE_SGBM);
    //sgbm2->setMode(StereoSGBM::MODE_SGBM_3WAY);
    sgbm2->setMode(stereo::StereoBinarySGBM::MODE_SGBM);

    sgbm2->setMinDisparity(0);
    sgbm2->setNumDisparities(192);
    //sgbm->setP1(600);
    //sgbm->setP2(2400);
    //sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    //sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    sgbm2->setP1(24*sgbmWinSize*sgbmWinSize);
    sgbm2->setP2(96*sgbmWinSize*sgbmWinSize);
    sgbm2->setPreFilterCap(64);
    sgbm2->setSpeckleRange(1);
    sgbm2->setSpeckleWindowSize(20);

    sgbm2->compute(g1, g2, disp);

    cv::Ptr<cv::stereo::StereoBinarySGBM> sgbm3 = cv::stereo::StereoBinarySGBM::create(0,16,3);
    //cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0,16,3);

                  sgbm3->setBlockSize(sgbmWinSize);



    //sgbm->setBlockSize(3);
    sgbm3->setDisp12MaxDiff(5);
    sgbm3->setUniquenessRatio(5);
    //sgbm->setMode(StereoSGBM::MODE_SGBM);
    //sgbm2->setMode(StereoSGBM::MODE_SGBM_3WAY);
    sgbm3->setMode(stereo::StereoBinarySGBM::MODE_SGBM);

    sgbm3->setMinDisparity(0);
    sgbm3->setNumDisparities(384);
    //sgbm->setP1(600);
    //sgbm->setP2(2400);
    //sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    //sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    sgbm3->setP1(24*sgbmWinSize*sgbmWinSize);
    sgbm3->setP2(96*sgbmWinSize*sgbmWinSize);
    sgbm3->setPreFilterCap(64);
    sgbm3->setSpeckleRange(2);
    sgbm3->setSpeckleWindowSize(20);

    sgbm3->compute(g1, g2, disp2);

    //disp.convertTo(disp12, CV_8U, 255/(960*16));
    normalize(disp2, disp28, 0, 255, CV_MINMAX, CV_8U);
    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

    string img1Path = CALIBFOLDER;
    img1Path.append("disp5.jpg");
    imwrite(img1Path,disp);
    img1Path = CALIBFOLDER;
    img1Path.append("disp58.jpg");
    imwrite(img1Path,disp8);
    Mat raw_disp_vis2 ,raw_disp_vis22;
    getDisparityVis(disp2,raw_disp_vis22,1.0);
    getDisparityVis(disp,raw_disp_vis2,1.0);

    //imshow("disp raw",disp);
    imshow("disp norm", disp8);
    //imshow("disp vis", raw_disp_vis2);
    //imshow("disp raw2",disp2);
    imshow("disp2 norm", disp28);
    //imshow("disp vis2", raw_disp_vis22);


    //test filter

    Ptr<DisparityWLSFilter> wls_filter;


    Mat left_for_matcher, right_for_matcher;
        Mat left_disp,right_disp;
        Mat filtered_disp;
        Mat conf_map = Mat(g1.rows,g1.cols,CV_8U);
        conf_map = Scalar(255);
        Rect ROI;

        left_for_matcher  = g1.clone();
        right_for_matcher = g2.clone();
        //left_for_matcher  = img1.clone();
        //right_for_matcher = img2.clone();

        String filter = "wls_conf";


            int max_disp = 192;
            double lambda = 8000.0;
            double sigma  = 1.5;
            double vis_mult = 1.0;
            int wsize = 3;

        //Ptr<stereo::StereoBinarySGBM> left_matcher  = stereo::StereoBinarySGBM::create(0,max_disp,wsize);
        Ptr<StereoSGBM> left_matcher  = StereoSGBM::create(0,max_disp,wsize);

        left_matcher->setP1(24*wsize*wsize);
        left_matcher->setP2(96*wsize*wsize);
        left_matcher->setPreFilterCap(64);
        left_matcher->setUniquenessRatio(1);
        //left_matcher->setSpeckleRange(2);
        //left_matcher->setSpeckleWindowSize(20);
        left_matcher->setMinDisparity(0);
        left_matcher->setDisp12MaxDiff(1);
        left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
        //left_matcher->setMode(stereo::StereoBinarySGBM::MODE_SGBM);
        wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);

        //
        /*
        Ptr<StereoSGBM> right_matcher  = StereoSGBM::create(0,max_disp,wsize);

        right_matcher->setP1(3*24*wsize*wsize);
        right_matcher->setP2(3*96*wsize*wsize);
        right_matcher->setPreFilterCap(192);
        right_matcher->setUniquenessRatio(2);
        right_matcher->setSpeckleRange(2);
        right_matcher->setSpeckleWindowSize(20);
        right_matcher->setMinDisparity(0);
        right_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
        */
        //

        Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

        left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
        cout << "nr disparity " << right_matcher->getNumDisparities() << " block "<< right_matcher->getBlockSize() << endl;
        right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);

        Mat raw_disp_visL,raw_disp_visR;
        getDisparityVis(left_disp,raw_disp_visL,1.0);
        //normalize(left_disp, left_disp, 0, 255, CV_MINMAX, CV_8U);
        imshow("left disp", left_disp);
        getDisparityVis(right_disp,raw_disp_visR,1.0);
        imshow("right disp", right_disp);
        imshow("right disp vis", right_disp);
        imshow("right disp vis", right_disp);

                    //! [filtering]
        wls_filter->setLambda(lambda);
        wls_filter->setSigmaColor(sigma);
        wls_filter->filter(left_disp,left_for_matcher,filtered_disp,right_disp,ROI,right_for_matcher);
                            //! [filtering]
        conf_map = wls_filter->getConfidenceMap();
        Ptr<DisparityWLSFilter> wls_filter2 = cv::ximgproc::createDisparityWLSFilterGeneric(true);
        wls_filter2->setLambda(lambda);
        wls_filter2->setSigmaColor(sigma);

                            // Get the ROI that was used in the last filter call:
        ROI = wls_filter->getROI();
        Mat filtered_disp2;
        wls_filter2->filter(left_disp,left_for_matcher,filtered_disp2,right_disp,ROI,right_for_matcher);
        // upscale raw disparity and ROI back for a proper comparison:





                        //! [visualization]
                        Mat raw_disp_vis;
                        getDisparityVis(left_disp,raw_disp_vis,vis_mult);
                        namedWindow("raw disparity", WINDOW_NORMAL| WINDOW_KEEPRATIO);
                        imshow("raw disparity", raw_disp_vis);
                        Mat filtered_disp_vis,filtered_disp_vis2;
                        getDisparityVis(filtered_disp2,filtered_disp_vis2,vis_mult);
                        getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
                        namedWindow("filtered disparity", WINDOW_NORMAL| WINDOW_KEEPRATIO);
                        namedWindow("filtered disparity2", WINDOW_NORMAL| WINDOW_KEEPRATIO);

                        filtered_disp_vis2.convertTo(filtered_disp_vis2,-1,1,1);

                        normalize(filtered_disp_vis, disp8, 0, 255, CV_MINMAX, CV_8U);
                        normalize(filtered_disp_vis2, disp12, 0, 255, CV_MINMAX, CV_8U);

                        imshow("filtered disparity", filtered_disp_vis);
                        imshow("filtered disparity2", filtered_disp_vis2);
                        Mat disp_color;
                        applyColorMap(left_disp,disp_color,COLORMAP_JET);

                        //imshow("bleh",disp8);
                        //imshow("blah",disp12);
                        imshow("bleh2",disp_color);
                        imshow("blurg",conf_map);

                        string img2Path = CALIBFOLDER;
                        string img2Path2 = CALIBFOLDER;
                        img2Path.append("disp5v.jpg");
                        img2Path2.append("disp5v2.jpg");
                        imwrite(img2Path,filtered_disp_vis);
                        imwrite(img2Path2,filtered_disp_vis2);

    //end test filter

    waitKey(0);
    destroyAllWindows();

}

matPair StereoCalibrate::undestort(matPair pair)
{

    Mat left_tangent= Mat::zeros(pair.left.size().height, pair.left.size().width, pair.left.type());
    Mat right_tangent = Mat::zeros(pair.right.size().height, pair.right.size().width, pair.right.type());
    tangent_distortion_correction(pair.left, &left_tangent, 1.0, 1.0-DISTORTION); // magic number found by mesuring images taken by deeper
    tangent_distortion_correction(pair.right, &right_tangent, 1.0-DISTORTION, 1.0);//really just an educated guess.
    //Mats tangent_rectified;
    /* for debuging
    namedWindow("orginal",WINDOW_NORMAL);
    namedWindow("tangentdestort",WINDOW_NORMAL);
    imshow("orginal",left);
    imshow("tangentdestort",left_tangent);
    */
    pair.left = left_tangent;
    pair.right = right_tangent;
    img1 = pair.left;
    img2 = pair.right;
    //waitKey(0);
    return pair;
}

Mat StereoCalibrate::undestortZoom(cv::Mat image,string file_name)
{
    qDebug()<<"Undistort via zoom info";
    Mat distCoeffs;
    float zoom_value = getZoomValue(file_name);
    if(!getDistCoeffs(distCoeffs, zoom_value, file_name))
    {
            qDebug()<<"Failed to get distortion coeffs";
            return image;
    }
    double focalRes = getFocalResolution(file_name);

    if (focalRes*zoom_value < 1)
    {
            qDebug()<<"No focal resolution found using a standin value";
            focalRes = 23.6;
    }
    Mat cameraMatrix = getCameraMatrix(zoom_value*focalRes, image.cols, image.rows);
    D1 = distCoeffs;
    D2 = distCoeffs;
    Mat undistorted, map1, map2;
    focalResC = focalRes;
    zoom_valueC = zoom_value;
    qDebug()<<"Image size is"<<image.size().width<<"x"<<image.size().height;
    qDebug()<<"cols x rows"<<image.cols<<image.rows;
    initUndistortRectifyMap(cameraMatrix,distCoeffs, Mat(),
                            cameraMatrix,
                            image.size(), CV_16SC2, map1, map2);
    remap(image, undistorted, map1, map2, INTER_LINEAR);
    //char buffer[256];
    //sprintf(buffer, "Zoom %4.2f", zoom_value);
    //putText(undistorted,buffer ,Point(100,200),FONT_HERSHEY_SIMPLEX,5.0, Scalar(255.0,0.0,0.0), 5);
    image = undistorted;
    return image;
}
