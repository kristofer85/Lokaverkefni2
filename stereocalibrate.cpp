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
#include "utils.h"
#include "string"
using namespace cv;
using namespace std;

StereoCalibrate::StereoCalibrate()
{
    patternSize = 2.4;
    numBoards = 12;
    board_w = 7;
    board_h = 10;
    board_sz = Size(board_w, board_h);
    board_n = board_w*board_h;
}


void StereoCalibrate::findAndDrawChessBoardCorners(string filename)
{
    bool found1 = false;
    bool found2 = false;
    String l = "left_color_536x712.png";
    String r = "right_color_536x712.png";
    String l_gGray = "left_color_536x712.png";
    String r_gGray = "right_color_536x712.png";

    FileStorage fs;
    fs.open(filename, FileStorage::READ);
    if (!fs.isOpened()){cerr << "Failed to open " << filename << endl;}
    FileNode n = fs["images"];                         // Read string sequence - Get node
    if (n.type() != FileNode::SEQ){cerr << "strings is not a sequence! FAIL" << endl;}
    string ChessboardImageList,left, right;
    FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
    int success = 0;

    for (int i=0; i<board_h; i++)
        for (int j=0; j<board_w; j++)
            obj.push_back(Point3f(i * patternSize, j *patternSize, 0.0f));
    for (; it != it_end; ++it)
    {
        ChessboardImageList = CHESSBOARDIMAGES;
        ChessboardImageList.append((string)*it);
        ChessHd = imread(ChessboardImageList,IMREAD_COLOR);
        imSize = ChessHd.size();
        img1 = ChessHd(Range(0, imSize.height),Range(0, imSize.width/2)).clone();
        img2 = ChessHd(Range(0, imSize.height),Range(imSize.width/2, imSize.width)).clone();
        //resize(img1,img1,Size(),0.25,0.25);
        //resize(img2,img2,Size(),0.25,0.25);
        imwrite("leftFound"+(string)*it,img1);
        imwrite("rightFound"+(string)*it,img2);
        cutSize = img1.size();

        left = LEFTRIGHTSCALED;
        left.append(l);
        imwrite(left,img1);         // Left Color image used for coloring the polyMesh
        right = LEFTRIGHTSCALED;
        right.append(r);
        imwrite(right,img2);        // Right Color not as inportant.
        cvtColor(img1,gray1,CV_RGB2GRAY);
        cvtColor(img2,gray2,CV_RGB2GRAY);
        left = LEFTRIGHTSCALED;
        left.append(l_gGray);
        imwrite(l_gGray,gray1);            // Left B&W Image used for comarison in depth mapping
        right = LEFTRIGHTSCALED;
        right.append(r_gGray);
        imwrite(r_gGray,gray2);           // Left B&W Image, the image left compares itself with.

        found1 = false;
        found2 = false;
        found1 = findChessboardCorners(img1, board_sz, corners1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
        found2 = findChessboardCorners(img2, board_sz, corners2, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

        if (found1)
        {
            cornerSubPix(gray1, corners1, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.01));
            drawChessboardCorners(gray1, board_sz, corners1, found1);
        }

        if (found2)
        {
            cornerSubPix(gray2, corners2, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            drawChessboardCorners(gray2, board_sz, corners2, found2);
        }

        if (found1 !=0 && found2 != 0)
        {
            imagePoints1.push_back(corners1);
            imagePoints2.push_back(corners2);
            objectpoints.push_back(obj);
            success++;
            ChessboardImageList = CHESSBOARDIMAGES;
            ChessboardImageList.append("leftFound"+(string)*it);
            imwrite(ChessboardImageList,gray1);
            ChessboardImageList = CHESSBOARDIMAGES;
            ChessboardImageList.append("rightFound"+(string)*it);
            imwrite(ChessboardImageList+(string)*it,gray2);
        }
    }

}

void StereoCalibrate::CalibrateStereoCamera()
{
    FileStorage stereoCalibrationStored = FileStorage("stereoCalibration.yml", FileStorage::WRITE);
    double rms = stereoCalibrate(objectpoints,
                                 imagePoints1,
                                 imagePoints2,
                                 CM1, D1, CM2, D2,
                                 cutSize,
                                 R, T, E, F,
                                 CV_CALIB_FIX_ASPECT_RATIO,
                                 cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5));
    cout << "done with RMS error=" << rms << endl;
    stereoCalibrationStored << "CM1" << CM1;
    stereoCalibrationStored << "CM2" << CM2;
    stereoCalibrationStored << "D1" << D1;
    stereoCalibrationStored << "D2" << D2;
    stereoCalibrationStored << "R" << R;
    stereoCalibrationStored << "T" << T;
    stereoCalibrationStored << "E" << E;
    stereoCalibrationStored << "F" << F;

    vector<Point3f> lines[2];

    double avgErr = 0;
    int nframes = (int)objectpoints.size();

    for(int i = 0; i < nframes; i++ )
    {
        vector<Point2f>& pt0 = imagePoints1[i];
        vector<Point2f>& pt1 = imagePoints2[i];

        undistortPoints(pt0, pt0, CM1, D1, Mat(), CM1);
        undistortPoints(pt1, pt1, CM2, D2, Mat(), CM2);
        computeCorrespondEpilines(pt0, 1, F, lines[0]);
        computeCorrespondEpilines(pt1, 2, F, lines[1]);

        for(int j = 0; j < 70; j++ )
        {
            double err = fabs(pt0[j].x*lines[1][j].x +
                pt0[j].y*lines[1][j].y + lines[1][j].z)
                + fabs(pt1[j].x*lines[0][j].x +
                pt1[j].y*lines[0][j].y + lines[0][j].z);
            avgErr += err;
        }
    }

    cout << "avg err = " << avgErr/(nframes*70) << endl;
    nframes = (int)objectpoints.size();
    vector<Point2f> allpoints[2];
    for(int i = 0; i < nframes; i++ )
    {
        copy(imagePoints1[i].begin(), imagePoints1[i].end(), back_inserter(allpoints[0]));
        copy(imagePoints2[i].begin(), imagePoints2[i].end(), back_inserter(allpoints[1]));
    }
    F = findFundamentalMat(allpoints[0], allpoints[1], FM_RANSAC, 3, 0.99);
    stereoRectifyUncalibrated(allpoints[0], allpoints[1], F, cutSize, H1, H2, 3);
    R1 = CM1.inv()*H1*CM1;
    R2 = CM2.inv()*H2*CM2;
}

void StereoCalibrate::rectifyCamera()
{

    FileStorage stereoCalibrationStored = FileStorage("stereoCalibration.yml", FileStorage::READ);
    stereoCalibrationStored["CM1"] >> CM1;
    stereoCalibrationStored["D1"] >> D1;
    stereoCalibrationStored["CM2"] >> CM2;
    stereoCalibrationStored["D2"] >> D2;
    stereoCalibrationStored["R"] >> R;
    stereoCalibrationStored["T"] >> T;

    stereoRectify( CM1, D1, CM2, D2, img1.size(), R, T, R1, R2, P1, P2, Q, CV_CALIB_FIX_ASPECT_RATIO );
//*******************************
// ToDo Write Data to DATA_HOLDER
//*******************************

    stereoCalibrationStored = FileStorage("stereoCalibration.yml", FileStorage::APPEND);
    stereoCalibrationStored << "R1" << R1;
    stereoCalibrationStored << "R2" << R2;
    stereoCalibrationStored << "P1" << P1;
    stereoCalibrationStored << "P2" << P2;
    stereoCalibrationStored << "Q" << Q;
    stereoCalibrationStored.release();
}

void StereoCalibrate::initUndistort(Mat& l, Mat& r)
{
    distortionCutSize = l.size();
    //R1 = imread("R1.png",IMREAD_UNCHANGED);
    //R2 = imread("R2.png",IMREAD_UNCHANGED);
    FileStorage stereoCalibrationStored = FileStorage("stereoCalibration.yml", FileStorage::READ);
    stereoCalibrationStored["CM1"] >> CM1;
    stereoCalibrationStored["D1"] >> D1;
    stereoCalibrationStored["CM2"] >> CM2;
    stereoCalibrationStored["D2"] >> D2;
    stereoCalibrationStored.release();
    FileStorage stereo = FileStorage("test.yml", FileStorage::READ);
    stereo["R1"] >> R1;
    stereo["R2"] >> R2;
    initUndistortRectifyMap(CM1, D1, R1, P1, distortionCutSize, CV_16SC2, map1x, map1y);
    initUndistortRectifyMap(CM2, D2, R2, P2, distortionCutSize, CV_16SC2, map2x, map2y);

    remap(l, imgU1, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
    remap(r, imgU2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());

    imwrite("leftRemap.jpg",imgU1);
    imwrite("rightRemap.jpg",imgU2);
    l = imgU1.clone();
    r = imgU2.clone();
}
