#include "stereocalibrate.h"
using namespace cv;
using namespace std;


StereoCalibrate::StereoCalibrate()
{
    //fs1 = FileStorage("stereoCalibration.yml", FileStorage::WRITE);
}


void StereoCalibrate::findAndDrawChessBoardCorners(string filename)
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
        //bleh = CALIBFOLDERFIXED;
        bleh = CALIBFOLDER;
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
        int success = 0, k = 0;
        bool found1 = false, found2 = false;

        while (success < numBoards)
        {
            cvtColor(img1, gray1, CV_BGR2GRAY);
            cvtColor(img2, gray2, CV_BGR2GRAY);

            found1 = findChessboardCorners(img1, board_sz, corners1, CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_FAST_CHECK + CV_CALIB_CB_NORMALIZE_IMAGE + CV_CALIB_CB_FILTER_QUADS);
            found2 = findChessboardCorners(img2, board_sz, corners2, CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_FAST_CHECK + CV_CALIB_CB_NORMALIZE_IMAGE + CV_CALIB_CB_FILTER_QUADS);

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
    cout << "starting camera calibration" << endl;
    CM1 = Mat(3, 3, CV_64FC1);
    CM2 = Mat(3, 3, CV_64FC1);
    D1 = Mat::zeros(8, 1, CV_64F);
    D2 = Mat::zeros(8, 1, CV_64F);
    Mat D11 = Mat::zeros(8, 1, CV_64F);
    Mat D22 = Mat::zeros(8, 1, CV_64F);

    //CM1 = getDefaultNewCameraMatrix(CM1,imgSize,false);
    //CM2 = getDefaultNewCameraMatrix(CM2,imgSize,false);
    cout << CM1 << endl;
    cout << CM2 << endl;

    //string img1Path = CALIBFOLDER;
    //img1Path.append("leftChessbord.png");
    //cout << "bleh path = " << img1Path << endl;
    //img1 = imread(img1Path,IMREAD_COLOR);

    string bleh = CALIBFOLDERFIXED;
    bleh.append("calib2_fixed.JPG");
    Mat fullImg = imread(bleh,IMREAD_COLOR);
    //split sterio pic into two images
    //Size imSize = fullImg.size();
    //img1 = fullImg(Range(0, imSize.height),Range(0, imSize.width/2)).clone();
    Size imageSize = imgSize;

    double apertureWidth = 4.0;
      double apertureHeight = 4.0;
      double fieldOfViewX;
      double fieldOfViewY;
      double focalLength = 18.0;
      cv::Point2d principalPoint;
      double aspectRatio;
      double fx = (imageSize.width*18.0)/23.6;
      double fy = (imageSize.height*18.0)/15.8;
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

    vector<Mat> rvecs,rvecs2, tvecs,tvecs2;
    calibrateCamera(object_points, imagePoints1,imgSize,CM1, D11,rvecs,tvecs,CV_CALIB_ZERO_TANGENT_DIST |CV_CALIB_USE_INTRINSIC_GUESS);
    calibrateCamera(object_points, imagePoints2,imgSize,CM2, D22,rvecs2,tvecs2,CV_CALIB_ZERO_TANGENT_DIST |CV_CALIB_USE_INTRINSIC_GUESS );
    //stereoCalibrate(object_points, imagePoints1, imagePoints2,CM1, D1, CM2, D2, imgSize, R, T, E, F, CV_CALIB_FIX_INTRINSIC,cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5));

    //stereoCalibrate(object_points, imagePoints1, imagePoints2,CM1, D1, CM2, D2, img1.size(), R, T, E, F, CV_CALIB_USE_INTRINSIC_GUESS,cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5));
      stereoCalibrate(object_points, imagePoints1, imagePoints2,CM1, D1, CM2, D2, img1.size(), R, T, E, F,CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST | CV_CALIB_FIX_ASPECT_RATIO | CV_CALIB_FIX_INTRINSIC ,cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5));

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
    stereoRectify(CM1, D1, CM2, D2, img1.size(), R, T, R1, R2, P1, P2, Q);
//*******************************
// ToDo Write Data to DATA_HOLDER
//*******************************
    //dataHolder.fs1 = FileStorage("stereoCalibration.yml", FileStorage::WRITE);
    //dataHolder.fs1 << "R1" << R1;
    //dataHolder.fs1 << "R2" << R2;
    //dataHolder.fs1 << "P1" << P1;
    //dataHolder.fs1 << "P2" << P2;
    //dataHolder.fs1 << "Q" << Q;
    //dataHolder.fs1.release();
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

void StereoCalibrate::initUndistort()
{
    cout << "starting remaping" << endl;
    //string bleh = CALIBFOLDERFIXED;
    string bleh = CALIBFOLDER;
    //bleh.append("calib2_fixed.JPG");
    bleh.append("DSC_0025.JPG");
    Mat fullImg = imread(bleh,IMREAD_COLOR);
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
    cout << "CM1"<< CM1 << endl;
    cout << "CM3"<< CM2 << endl;
    //stereoRectify(CM1, D1, CM2, D2, img1.size(), R, T, R1, R2, P1, P2, Q,CV_CALIB_ZERO_DISPARITY);

    initUndistortRectifyMap(CM1, D1, R1, P1, img1.size(), CV_32FC1, map1x, map1y);
    initUndistortRectifyMap(CM2, D2, R2, P2, img1.size(), CV_32FC1, map2x, map2y);

    remap(img1, imgU1, map1x, map1y, INTER_CUBIC, BORDER_CONSTANT, Scalar());
    remap(img2, imgU2, map2x, map2y, INTER_CUBIC, BORDER_CONSTANT, Scalar());
    namedWindow("image1",WINDOW_NORMAL| WINDOW_KEEPRATIO);
    namedWindow("image2",WINDOW_NORMAL| WINDOW_KEEPRATIO);
    imshow("image1", imgU1);
    imshow("image2", imgU2);
    string img1Path = CALIBFOLDER;
    img1Path.append("leftChessbordW.jpg");
    imwrite(img1Path,imgU1);
    img1Path = CALIBFOLDER;
    img1Path.append("rightChessbordW.jpg");
    imwrite(img1Path,imgU2);
    int k = waitKey(0);

    if(k==27)
    {
        destroyAllWindows();
    }

}

