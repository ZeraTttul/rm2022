#ifndef RM2022_KALMAN_CPP
#define RM2022_KALMAN_CPP

#include "opencv2/opencv.hpp"
#include "../include/Solution.h"
#include "../include/Kalman.h"

using namespace std;
using namespace cv;

void kalman :: init(KalmanFilter KF) {
    //将下面几个矩阵设置为对角阵
    setIdentity(KF.measurementMatrix);                     //测量矩阵 H
    setIdentity(KF.processNoiseCov, Scalar::all(1e-3));    //过程噪声 Q
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-2));//测量噪声 R
    setIdentity(KF.errorCovPost, Scalar::all(1));          //最小均方误差 Pt
    //randn( KF.statePost, Scalar::all(0), Scalar::all(0.1) );
    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1)); //x(0)初始化
}

Point2f kalman::kal(float x,float y)
{
    Point2f center;   
    prediction = KF.predict();
    measurement.at<float>(0) = x;
	measurement.at<float>(1) = y;		
    for(int i=1;i<=5;i++)
         {
            KF.correct(measurement);
            prediction = KF.predict();
            measurement.at<float>(0) = prediction.at<float>(0);
		    measurement.at<float>(1) = prediction.at<float>(1);
        }
        center.x=prediction.at<float>(0);
        center.y=prediction.at<float>(1);
    return center;
}
#endif