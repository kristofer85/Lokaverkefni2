#include "depthmap.h"
#include "defines.h"
DepthMap::DepthMap()
{
    leftImage = "leftImage.png";
    rightImage = "rightImage.png";
    disparityImage = "dispImage.png";

}
Rect DepthMap::computeROI(Size2i src_sz, Ptr<StereoMatcher> matcher_instance)
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
void DepthMap::run()
{
    //Mat stereo = imread("kula.jpg",IMREAD_COLOR);
    //imSize = stereo.size();
    //left = stereo(Range(0, imSize.height),Range(0, imSize.width/2)).clone();
    //right = stereo(Range(0, imSize.height),Range(imSize.width/2, imSize.width)).clone();

    //resize(left ,left ,Size(),0.5,0.5);
    //resize(right,right,Size(),0.5,0.5);
    //cvtColor(left, g1, CV_BGR2GRAY);
    //cvtColor(right, g2, CV_BGR2GRAY);
    //imwrite("leftDepthMap.png",left);
    //imwrite("rightDepthMap.png",right);
    //
    //imwrite("leftGray.png", g1);
    //imwrite("rightGray.png", g2);
    left = imread(leftImage,IMREAD_COLOR);
    right = imread(rightImage,IMREAD_COLOR);
    resize(left ,left ,Size(),0.25,0.25);
    resize(right,right,Size(),0.25,0.25);
    imwrite(leftImage,left);
    imwrite(rightImage,right);
    cvtColor(left, g1, CV_BGR2GRAY);
    cvtColor(right, g2, CV_BGR2GRAY);
    //threshold(g1,g1,100,255,CV_THRESH_BINARY);
    //threshold(g2,g2,120,255,CV_THRESH_BINARY);
   imwrite("leftGray.png",g1);
   imwrite("rightGray.png",g2);


    //cvtColor(left, g1, CV_BGR2GRAY);
    //cvtColor(right, g2, CV_BGR2GRAY);
    //imwrite(leftImage,left);
    //imwrite(rightImage,right);
    //SGBMBinary(g1,g2);
    SGBMdisparityCalc(g1,g2);
    //BMdisparityCalc(g1,g2);

    //DisparityFilter(left,right);
}

void DepthMap::DisparityFilter(Mat l,Mat r)
{


    Mat left_for_matcher, right_for_matcher;
    Mat left_disp,right_disp;
    Mat filtered_disp;
    Mat conf_map = Mat(left.rows,left.cols,CV_8U);
    conf_map = Scalar(255);
    Rect ROI;
    Ptr<DisparityWLSFilter> wls_filter;
    double matching_time, filtering_time;



    left_for_matcher  = l.clone();
    right_for_matcher = r.clone();
    int wsize = 3;
    Ptr<StereoSGBM> left_matcher  = StereoSGBM::create(0,48,wsize);
    left_matcher->setP1(24*wsize*wsize);
    left_matcher->setP2(96*wsize*wsize);
    left_matcher->setPreFilterCap(5);
    left_matcher->setMinDisparity(-32);
    left_matcher->setBlockSize(3);
    left_matcher->setDisp12MaxDiff(12);
    left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
    wls_filter = createDisparityWLSFilter(left_matcher);
    Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);
    matching_time = (double)getTickCount();
    left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
    right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
    matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();

    //! [filtering]
    wls_filter->setLambda(8000);
    wls_filter->setSigmaColor(1.5);
    filtering_time = (double)getTickCount();
    wls_filter->filter(left_disp,left,filtered_disp,right_disp);
    filtering_time = ((double)getTickCount() - filtering_time)/getTickFrequency();
    //! [filtering]
    conf_map = wls_filter->getConfidenceMap();
    //
    //// Get the ROI that was used in the last filter call:
    ROI = wls_filter->getROI();
    left_matcher->save("g.txt");

    cout.precision(2);
    cout<<"Matching time:  "<<matching_time<<"s"<<endl;
    cout<<"Filtering time: "<<filtering_time<<"s"<<endl;
    cout<<endl;




    Mat filtered_disp_vis;
    getDisparityVis(filtered_disp,filtered_disp_vis,1.0);
    imwrite("dst_path.jpg",filtered_disp_vis);



    Mat raw_disp_vis;
    getDisparityVis(left_disp,raw_disp_vis,1.0);
    imwrite("dst_raw_path.jpg",raw_disp_vis);



    imwrite("dst_conf_path.png",conf_map);


}

void DepthMap::BMdisparityCalc(Mat g1,Mat g2)
{
    cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(16,9);

    int SADWindowSize = 9;
    bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
    bm->setDisp12MaxDiff(64);
    bm->setUniquenessRatio(5);
    bm->setMinDisparity(0);
    bm->setNumDisparities(112);
    bm->setPreFilterCap(5);
    bm->setSpeckleRange(20);
    bm->setSpeckleWindowSize(0);
    bm->compute(g1, g2, disp);
    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_16S);
    imwrite("dispBM.png", disp);
    imwrite("disp8BM.png", disp8);
}

void DepthMap::SGBMdisparityCalc(Mat g1,Mat g2)
{
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0,16,7);
    int sgbmWinSize = 3;
    sgbm->setBlockSize(3);
    sgbm->setDisp12MaxDiff(2);
    sgbm->setUniquenessRatio(10);
    sgbm->setMode(StereoSGBM::MODE_SGBM);
    sgbm->setMinDisparity(-32);
    sgbm->setNumDisparities(64);
    sgbm->setP1(8*g1.channels()*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32*g2.channels()*sgbmWinSize*sgbmWinSize);
    sgbm->setPreFilterCap(12);
    sgbm->setSpeckleRange(2);
    sgbm->setSpeckleWindowSize(200);
    sgbm->compute(g1, g2, disp);
    //sgbm->save("j.txt");
    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);
    imwrite("dispSGBM.png", disp);
    imwrite(disparityImage, disp8);

     //bilateralFilter(g1,g2,0,6.0,6.0);
    medianBlur(disp,dispf,5);
    medianBlur(disp8,dispf8,5);

    imwrite("dispSGBMMedian.png", dispf);
    imwrite("disp8SGBMMedian.png", dispf8);


}

void DepthMap::SGBMBinary(Mat l,Mat r)
{
    Mat imgDisparity16S2 = Mat(l.rows, l.cols, CV_16S);
    Mat imgDisparity8U2 = Mat(l.rows, l.cols, CV_8UC1);
    int kernel_size = 10, number_of_disparities = 64, aggregation_window = 0, P1 = 2400, P2 = 600;
    int binary_descriptor_type = 0;
    cv::Ptr<cv::stereo::StereoBinarySGBM> sgbm = cv::stereo::StereoBinarySGBM::create(0, number_of_disparities, kernel_size);
            // setting the penalties for sgbm
            sgbm->setP1(P1);
            sgbm->setP2(P2);
            sgbm->setMinDisparity(0);
            sgbm->setUniquenessRatio(5);
            sgbm->setSpeckleWindowSize(30);
            sgbm->setSpeckleRange(0);
            sgbm->setDisp12MaxDiff(1);
            //sgbm->setsetBinaryKernelType(binary_descriptor_type);
            //sgbm->setSpekleRemovalTechnique(CV_SPECKLE_REMOVAL_AVG_ALGORITHM);
            //sgbm->setSubPixelInterpolationMethod(CV_SIMETRICV_INTERPOLATION);
            sgbm->compute(l, r, imgDisparity16S2);
            /*Alternative for scalling
            imgDisparity16S2.convertTo(imgDisparity8U2, CV_8UC1, scale);
            */

            double minVal = 0.0; double maxVal = 100.0;
            minMaxLoc(imgDisparity16S2, &minVal, &maxVal);
            //imgDisparity16S2.convertTo(imgDisparity8U2, CV_8UC1, 255 / (maxVal - minVal));
            //show the disparity image
            imwrite("Windowsgm.jpg", imgDisparity16S2);
}

