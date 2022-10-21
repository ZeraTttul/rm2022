//
// Change by JYSimilar on 2022/9/25
//
// 该文件已整理完成

#include "opencv2/opencv.hpp"
#include "../include/rgb.h"

using namespace std;
using namespace cv;

//图片准备
Mat Rgb::imagePreprocess(const cv::Mat &src, bool flag) {

    _src = src;

    _splitSrc.clear();

    cv::split(_src, _splitSrc);

    if (flag == 1) {
        //识别红色

        //削减亮度
        Mat BrightnessLut(1, 256, CV_8UC1);

        for (int i = 0; i < 256; i++) {
            //you can change "_para.imageBright_RED" to adjust bright level(in the file rgb.h)
            BrightnessLut.at<uchar>(i) = saturate_cast<uchar>((double)i + _para.imageBright_RED);
        }

        LUT(_src, BrightnessLut, _src);

        //分离色彩通道
        cv::cvtColor(_src, _graySrc, cv::COLOR_BGR2GRAY);

        //高斯滤波
        GaussianBlur( _graySrc,
                      _graySrc,
                      Size(11, 11),
                      0,
                      0);

        //灰度二值化
        cv::threshold(_graySrc,
                      _graySrc,
                      _para.grayThreshold_RED,
                      255,
                      cv::THRESH_BINARY);

        //红蓝通道相减
        cv::subtract(_splitSrc[2],
                     _splitSrc[0],
                    _separationSrc);

        //红蓝二值化
        cv::threshold(_separationSrc,
                      _separationSrc,
                      _para.separationThreshold_RED,
                      255,
                      cv::THRESH_BINARY);

        //红绿通道相减
        cv::subtract(_splitSrc[2],
                     _splitSrc[1],
                     _separationSrcGreen);

        cv::erode(_separationSrcGreen,
                  _separationSrcGreen,
                  Rgb:: structuringElement2());

        cv::dilate(_separationSrcGreen,
                   _separationSrcGreen,
                   Rgb:: structuringElement3());

        imshow("1",_separationSrc);

        //逻辑与获得最终二值化图像
        _maxColor = _separationSrc & _graySrc;

        cv::dilate(_maxColor, _maxColor,Rgb::structuringElement3());

    } else {
        //识别蓝色

        //削减亮度
        Mat BrightnessLut(1, 256, CV_8UC1);

        for (int i = 0; i < 256; i++) {
            //you can change "_para.imageBright_BLUE" to adjust bright level(in the file rgb.h)
            BrightnessLut.at<uchar>(i) = saturate_cast<uchar>((double)i + _para.imageBright_BLUE);
        }

        LUT(_src, BrightnessLut, _src);

<<<<<<< HEAD
=======
        //高斯滤波
        GaussianBlur( _src,
                      _src,
                      Size(11, 11),
                      0,
                      0 );

>>>>>>> bef8c8c906c207319740c4f400c857de90997db8
        //分离色彩通道
        cv::cvtColor(_src, _graySrc, cv::COLOR_BGR2GRAY);

        //高斯滤波
<<<<<<< HEAD
        GaussianBlur( _graySrc,
                      _graySrc,
                      Size(11, 11),
                      0,
                      0 );
=======
        // GaussianBlur( _graySrc,
        //               _graySrc,
        //               Size(11, 11),
        //               0,
        //               0 );
        
        //imshow("graysrc", _graySrc);

        //直方图均质化 能把灰度图像几乎变回没灰度的时候
        // Mat _equalseparaSrc;
        // _equalseparaSrc = EqualHist(_graySrc);
>>>>>>> bef8c8c906c207319740c4f400c857de90997db8

        //灰度二值化
        cv::threshold(_graySrc,
                      _graySrc,
                      _para.grayThreshold_BLUE,
                      255,
                      cv::THRESH_BINARY);

        //红蓝通道相减
        cv::subtract(_splitSrc[0],
                     _splitSrc[2],
                     _separationSrc);

<<<<<<< HEAD
        Mat _grayseparaSrc;
        cvtColor(_separationSrc, _grayseparaSrc, cv::COLOR_BGR2GRAY);
        imshow("_grayseparasrc",_grayseparaSrc);
        // Mat hist;
        // hist = GetHist(_separationSrc);
        // imshow("hist",hist);
=======
        //Mat _grayseparaSrc;
        // equalizeHist(_grayseparaSrc, _equalseparaSrc);
        //imshow("_separationSrc", _separationSrc);
>>>>>>> bef8c8c906c207319740c4f400c857de90997db8

        //红蓝二值化
        cv::threshold(_separationSrc,
                      _separationSrc,
                      _para.separationThreshold_BLUE,
                      255,
                      cv::THRESH_BINARY);
<<<<<<< HEAD
        //imshow("separationsrc",_separationSrc);
=======

        //imshow("separationsrc",_separationSrc);

>>>>>>> bef8c8c906c207319740c4f400c857de90997db8
        cv::subtract(_splitSrc[0], _splitSrc[1], _separationSrcGreen);
        cv::erode(_separationSrcGreen, _separationSrcGreen, Rgb::structuringElement3());
        cv::dilate(_separationSrcGreen, _separationSrcGreen, Rgb::structuringElement5());

        //逻辑与获得最终二值化图像
<<<<<<< HEAD
        _maxColor = _separationSrc & _graySrc ;
=======
        _maxColor = _separationSrc & _graySrc & _separationSrcGreen;
>>>>>>> bef8c8c906c207319740c4f400c857de90997db8

        //膨胀
        cv::dilate(_maxColor, _maxColor, Rgb::structuringElement5());

    }
    return _maxColor;
}

<<<<<<< HEAD
Mat Rgb:: GetHist(Mat& image)
{
    Mat hist;
    const int channels[] = {0};
    int dims = 1;
    int histSize[] = {256};
    float granges[] = {0, 255};
    const float *ranges[] = {granges};
    calcHist(&image, 1, channels, Mat(), hist, dims, histSize, ranges);
    equalizeHist(hist,hist);
    return hist;
=======
Mat Rgb:: EqualHist(Mat image)
{
    Mat equalImg;
    Mat grayImage;
    equalizeHist(image, equalImg);
    imshow("equalImg", equalImg);
    return equalImg;
>>>>>>> bef8c8c906c207319740c4f400c857de90997db8
}