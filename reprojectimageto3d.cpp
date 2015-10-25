#include "reprojectimageto3d.h"

ReprojectImageTo3d::ReprojectImageTo3d()
{

}


void ReprojectImageTo3d::reproject(const cv::Mat& disparity, const cv::Mat& Q, cv::Mat& out3D)
{


    out3D = cv::Mat::zeros(disparity.size(), CV_32FC3);

    float Q03 = Q.at<float>(0, 3);
    float Q13 = Q.at<float>(1, 3);
    float Q23 = Q.at<float>(2, 3);
    float Q32 = Q.at<float>(3, 2);
    float Q33 = Q.at<float>(3, 3);

    for (int i = 0; i < disparity.rows; i++)
    {
        const float* disp_ptr = disparity.ptr<float>(i);
        cv::Vec3f* out3D_ptr = out3D.ptr<cv::Vec3f>(i);

        for (int j = 0; j < disparity.cols; j++)
        {
            const float pw = 1.0f / (disp_ptr[j] * Q32 + Q33);

            cv::Vec3f& point = out3D_ptr[j];
            point[0] = (static_cast<float>(j)+Q03) * pw;
            point[1] = (static_cast<float>(i)+Q13) * pw;
            point[2] = Q23 * pw;
        }
    }
}

void ReprojectImageTo3d::save(const Mat& image3D, const string& fileName)
{


    ofstream outFile("geggjad.pcd");

    if (!outFile.is_open())
    {
        std::cerr << "ERROR: Could not open " << fileName << std::endl;
        return;
    }

    for (int i = 0; i < image3D.rows; i++)
    {
        const cv::Vec3f* image3D_ptr = image3D.ptr<cv::Vec3f>(i);

        for (int j = 0; j < image3D.cols; j++)
        {
            outFile << image3D_ptr[j][0] << " " << image3D_ptr[j][1] << " " << image3D_ptr[j][2] << std::endl;
        }
    }

    outFile.close();
}
