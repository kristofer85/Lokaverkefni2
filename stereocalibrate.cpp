#include "stereocalibrate.h"

StereoCalibrate::StereoCalibrate()
{
    //fs1 = FileStorage("stereoCalibration.yml", FileStorage::WRITE);
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


    img1 = imread("C:/Users/kristinn/Documents/leftChessbord.png",IMREAD_COLOR);
    img2 = imread("C:/Users/kristinn/Documents/rightChessbord.png",IMREAD_COLOR);
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

        imshow("image1", gray1);
        imshow("image2", gray2);

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
    stereoCalibrate(object_points, imagePoints1, imagePoints2,CM1, D1, CM2, D2, img1.size(), R, T, E, F,CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST),cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5);

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

void StereoCalibrate::initUndistort()
{
    initUndistortRectifyMap(CM1, D1, R1, P1, img1.size(), CV_32FC1, map1x, map1y);
    initUndistortRectifyMap(CM2, D2, R2, P2, img1.size(), CV_32FC1, map2x, map2y);

    remap(img1, imgU1, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
    remap(img2, imgU2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());

    imshow("image1", imgU1);
    imshow("image2", imgU2);
    imwrite("C:/Users/kristinn/Documents/leftChessbord1.png",imgU1);
    imwrite("C:/Users/kristinn/Documents/rightChessbord1.png",imgU2);
    int k = waitKey(0);

    if(k==27)
    {
        destroyAllWindows();
    }

}

