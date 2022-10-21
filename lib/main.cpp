//
// Created by JYSimilar on 2022/9/25.
//

#include "../include/Solution.h"

/****************************
*
* 记得给串口赋予权限 chmod 777 /dev/ttyUSB0
*
* **************************/

#ifdef NX
#include "serial.h"
#endif

using namespace std;
using namespace cv;



int main () {

    #ifdef NX
    Serial uart(BR115200, WORDLENGTH_8B, STOPBITS_1, PARITY_NONE, HWCONTROL_NONE);
    uart.sOpen("/dev/ttyUSB0");
    VideoCapture capture(0);
    #endif

    Solution solve;
    solve.sol();
    //

    /*******equalhist test***********/
    // Rgb rgb;
    // Mat img = imread("E:/VSCode/Picture/blue7.jpg");
    // cvtColor(img, img, COLOR_BGR2GRAY);
    // Mat hist;
    // Mat equalImg;
    // imshow("img", img);
    // equalizeHist(img, equalImg);
    // imshow("equalImg", equalImg);
    // const int channels[] = {0};
    // int dims = 1;
    // int histSize[] = {256};
    // float granges[] = {0, 255};
    // const float *ranges[] = {granges};
    // calcHist(&equalImg, 1, channels, Mat(), hist, dims, histSize, ranges);
    //equalizeHist(hist, _hist);
    // waitKey(0);

    return 0;
}