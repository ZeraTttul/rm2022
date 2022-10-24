//
// Created by JYSimilar on 2022/9/25.
//

#ifndef RM2022_SOLUTION_H
#define RM2022_SOLUTION_H

#include <ctime>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <iostream>
#include <cmath>
#include <fcntl.h>
#include <cerrno>
#include <cstring>
#include <cstdio>
#include "SolvePnP.h"
#include "Kalman.h"
#include "rgb.h"

//#define NX
//#define RED
#define BLUE
#define IMSHOW //提高效率可以把这两个注释掉
//#define CLOCK
//#define PREDICT 10.12 卡尔曼不可用

#ifdef NX
#include "serial.h"
#include "hikvision_camera.h"
using namespace camera;
#endif

using namespace std;
using namespace cv;

class Solution {
private:

    clock_t start, finish;
    double totaltime, heights[32],sum=0;
    int hi = 0;
    int times = 0;

    Mat frame, binary, frame1;
    RotatedRect Select_contours[32];
    RotatedRect RA[32], R[32];

    int stateNum = 4;                          //状态值4×1向量(x,y,△x,△y)
    int measureNum = 2;                        //测量值2×1向量(x,y)
    KalmanFilter KF;
    Mat measurement;                           //测量值
    
    float xishu;
    const double P = 3.1415926;

public:
    void sol();
    void init(KalmanFilter &KF);
    void selectRightContours(vector<vector<Point>> &contours,
                             vector<vector<Point>> &select_contours);
    void chooseNearest();
    kalman k;
};

#endif //RM2022_SOLUTION_H
