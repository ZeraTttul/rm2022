#ifndef RM2022_KALMAN_H
#define RM2022_KALMAN_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include "../include/Solution.h"
#include "../include/SolvePnP.h"
#include <queue>
#include <vector>
#include <iostream>

using namespace std;
using namespace cv;

class kalman
{
public:
    const int stateNum = 4;                          //状态值4×1向量(x,y,△x,△y)
    const int measureNum = 2;                        //测量值2×1向量(x,y)
    KalmanFilter KF;
    Mat measurement;                           //测量值
    Mat prediction;
public:
void init(KalmanFilter KF);
Point kal(float x,float y);
};
#endif