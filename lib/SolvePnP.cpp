
#ifndef RM2022_SOLVEPNP_CPP
#define RM2022_SOLVEPNP_CPP

#include "opencv2/opencv.hpp"
#include "../include/SolvePnP.h"

using namespace std;
using namespace cv;

float SOLVEPNP::PNP(int flag)
{
    if(flag==0)
    {
            model_points.push_back(Point3d(-66.75f, -24.25f, 0));
            model_points.push_back(Point3d(+66.75f, -24.25f, 0));
            model_points.push_back(Point3d(-66.75f, +24.25f, 0));
            model_points.push_back(Point3d(+66.75f, +24.25f, 0));
    }
    if(flag==1)
    {
            model_points.push_back(Point3d(-114.0f, -24.25f, 0));
            model_points.push_back(Point3d(+114.0f, -24.25f, 0));
            model_points.push_back(Point3d(-114.0f, +24.25f, 0));
            model_points.push_back(Point3d(+114.0f, +24.25f, 0));
    }
    solvePnP(model_points, picture_points, camera_matrix, dist_coeffs,
        rotation_vector, translation_vector, 0, SOLVEPNP_ITERATIVE);
    // 默认ITERATIVE方法，可尝试修改为EPNP（CV_EPNP）,P3P（CV_P3P）

    Mat Rvec;
    Mat_<float> Tvec;
    rotation_vector.convertTo(Rvec, CV_32F);  // 旋转向量转换格式
    translation_vector.convertTo(Tvec, CV_32F); // 平移向量转换格式 

    Mat_<float> rotMat(3, 3);
    Rodrigues(Rvec, rotMat);
    // 旋转向量转成旋转矩阵

    Mat P_oc;
    P_oc = -rotMat.inv() * Tvec;
    // 求解相机的世界坐标，得出p_oc的第三个元素即相机到物体的距离即深度信息，单位是mm

    //迭代器版
    // MatIterator_<float> it = P_oc.begin<float>();
    // MatIterator_<float> it_end = P_oc.end<float>();
    // for(int i = 1;it != it_end;it++,i++)
    // {
    //     if(i == 3)
    //     {
    //          distance = (float)(*it);
    //          break;
    //     }

    // }
    //at版
    distance = P_oc.at<float>(2,0);
    distance /= 10; 

    //cout << "P_oc " << endl << P_oc << endl;
    // cout << "distance " << abs(distance)<< endl;
    return abs(distance);
}
#endif
