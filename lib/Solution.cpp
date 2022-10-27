//
// Created by JYSimilar on 2022/9/25.
//

#ifndef RM2022_SOLUTION_CPP
#define RM2022_SOLUTION_CPP

#include "../include/Solution.h"

void Solution :: sol() {
    //定义KalmanFilter类并初始化
#ifdef PREDICT
    KalmanFilter KK(k.stateNum,k.measureNum,0);
    k.KF=KK;
    //定义测量值
    k.measurement = Mat::zeros(k.measureNum,
                                 1,
                                 CV_32F);
                      

    //转移矩阵 A
    k.KF.transitionMatrix = (Mat_<float>(k.stateNum,
                                       k.stateNum) <<
            1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1);

  
  
   // 1.初始化
        k.init(k.KF);
#endif

#ifdef NX
	HikCamera MVS_cap; // 初始化相机
    MVS_cap.CamInfoShow(); // 显示图像参数信息
	MVS_cap.ReadImg(frame1); // 相机取图
    if (frame1.empty()) { // 相机开启线程需要一定时间
        continue;
    }
#endif 
    // VideoCapture cap ("E:\\VSCode\\production\\Rgb\\Video\\blueVideo3.mp4");

    while (true) {
        //再次确保不会爆数组
        hi = 0;

#ifdef CLOCK
        start = clock();
#endif

        frame = imread("E:/VSCode/Picture/red4.jpg");

        // cap.read(frame1);
	    // resize(frame1,frame,frame.size(),0.5,0.5);
        frame.copyTo(binary);//展示效果
        frame.copyTo(frame1);

        Rgb rgb;

#ifdef BLUE
        frame = rgb.imagePreprocess(frame, 0);//flag=1识别红色 else 识别蓝色
#endif
#ifdef RED
        frame = rgb.imagePreprocess(frame, 1);//flag=1识别红色 else 识别蓝色
#endif

#ifdef IMSHOW
        imshow("double", frame);
#endif

        // 标记装甲板
        vector<vector<Point> > contours;
        findContours(frame,
                     contours,
                     RETR_LIST,
                     CHAIN_APPROX_NONE);
        vector<vector<Point>> select_contours;

        // select the right contours
        selectRightContours(contours, select_contours);

        //选择最近的装甲板进行击打
        chooseNearest();

#ifdef IMSHOW
        imshow("okey", binary);
#endif
        waitKey(30);
#ifdef CLOCK
        finish = clock();
        totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
        sum += totaltime;
        times+=1;
        if (sum > 1)
        {
            cout <<"times"<< times << endl;
            times = 0;
            sum = 0;
        }
        std::cout << "Time" << totaltime << endl;
#endif
        hi = 0;//这个很重要！！！ 让RotatedRect数组重新从0开始存储 不然会爆数组
    }
}

void Solution :: init(KalmanFilter &KF) {
    //将下面几个矩阵设置为对角阵
    setIdentity(KF.measurementMatrix);                     //测量矩阵 H
    setIdentity(KF.processNoiseCov, Scalar::all(1));    //过程噪声 Q
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));//测量噪声 R
    setIdentity(KF.errorCovPost, Scalar::all(1));          //最小均方误差 Pt
    //randn( state, Scalar::all(0), Scalar::all(0.1) );
    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1)); //x(0)初始化
}

void Solution :: selectRightContours(vector<vector<Point>> &contours, vector<vector<cv::Point>> &select_contours) {

    for (size_t i = 0; i < contours.size(); i++) {

        float light_contour_area = contourArea(contours[i]);

        if (light_contour_area < 20 || contours[i].size() <= 5){
            continue;
        }

        drawContours(frame,
                     contours,
                     -1,
                     Scalar(255),
                     -1);

        // RotatedRect rec = fitEllipse(contours[i]); //椭圆拟合
        RotatedRect rec = minAreaRect(contours[i]); //最小外接矩阵拟合
        //采用最小外接矩阵拟合比椭圆拟合效果要好，因为有的装甲板两条灯条亮度不一样，用椭圆拟合出来的矩形亮的比暗的大很多

        cout << "rec_angle " << rec.angle << endl;
        float angle = abs(rec.angle);
        if (angle > 45){
            angle = 90 - angle;
        }
        if (angle > 30){
            continue;
        }

        const float width_height_ratio_max = 5;
        if (abs(rec.angle) < 45){
            if ((rec.size.height / rec.size.width) > width_height_ratio_max){
                continue;
            }
        } else {
            if ((rec.size.width / rec.size.height) > width_height_ratio_max) {
                continue;
            }
        }
        select_contours.push_back(contours[i]);
    }

    for (int i = 0; i < select_contours.size(); i++) {

        drawContours(frame, 
                     select_contours, 
                     -1, 
                     Scalar(0), 
                     4);

        if (i == select_contours.size() - 1) {
            continue;
        }
        RotatedRect rect = minAreaRect(select_contours[i]);

        for (int j = i + 1; j < select_contours.size(); j++) {

            RotatedRect rectA = minAreaRect(select_contours[j]);

            float r_angle, rA_angle, angle_diff;
            float r_height, r_width, rA_height, rA_width;
            float r_ratio, rA_ratio;

            // cout << "r_angle " << rect.angle << endl;
            // cout << "rA_angle " << rectA.angle << endl;

            // if((rect.angle > 0 && rect.angle < 30) && (rectA.angle > 60 && rectA.angle < 90)) 
            // {
            //     cout<< "1" <<endl;
            //     continue;
            // }

            if (rect.angle < -45){
                r_angle = 90 + rect.angle;
                r_width = rect.size.height;
                r_height = rect.size.width;
            } else {
                r_angle = rect.angle;

                r_height = rect.size.height;
                r_width = rect.size.width;
            }

            if (rectA.angle < -45) {
                rA_angle = 90 + rectA.angle;
                rA_width = rectA.size.height;
                rA_height = rectA.size.width;
            } else {
                rA_angle = rectA.angle;
                rA_height = rectA.size.height;
                rA_width = rectA.size.width;
            }

            r_ratio = r_height / r_width;
            rA_ratio = rA_height / rA_width;

            // if (r_ratio > 8 || r_ratio < 1 || rA_ratio> 8 || rA_ratio < 1) {
            //     //cout << r_ratio << "   " << rA_ratio << endl;;
            //     continue;
            // }

            angle_diff = abs(r_angle - rA_angle);
            if (angle_diff > 10 && angle_diff < 70) {
                continue;
            }

            float d = sqrt((rect.center.x - rectA.center.x) * (rect.center.x - rectA.center.x) +
                           (rect.center.y - rectA.center.y) * (rect.center.y - rectA.center.y));
            if (abs(rect.center.x - rectA.center.x) < 0.90 * d) {
                continue;
            }

            const float twobar_height_ratio_max = 1.5;
            float twobar_height_ratio = max(max(rect.size.width, rect.size.height),
                                            max(rectA.size.width, rectA.size.height)) / min(max(rect.size.width, rect.size.height),
                                            max(rectA.size.width, rectA.size.height));
            float twobar_width_ratio = max(min(rect.size.width, rect.size.height),
                                           min(rectA.size.width, rectA.size.height)) / min(min(rect.size.width, rect.size.height),
                                           min(rectA.size.width, rectA.size.height));;

            if (twobar_width_ratio > twobar_height_ratio_max
                || twobar_height_ratio > twobar_height_ratio_max) {
                continue;
            }

            if (rect.size.area() / rectA.size.area() > 2) {
                continue;
            }

            float board_width = abs(rect.center.x - rectA.center.x);
            float board_height = (max(rect.size.width, rect.size.height) + max(rectA.size.width, rectA.size.height)) / 2;
            float board_ratio = board_width / board_height;
            const float board_ratio_max = 4;
            const float board_ratio_min = 2;

            if (board_ratio > board_ratio_max || board_ratio < board_ratio_min) {
                continue;
            }

            float board_width_difference = abs(rect.center.x - rectA.center.x);
            float board_height_difference = abs(rect.center.y - rectA.center.y);

            const float board_height_difference_max = 200;
            const float board_width_difference_min = 30;
            if (board_width_difference < board_width_difference_min || board_height_difference>board_height_difference_max) {
                continue;
            }

#ifdef IMSHOW
            cv::Point2f* vertices1 = new cv::Point2f[4];
            rect.points(vertices1);

            for (int j = 0; j < 4; j++) {
                cv::line(binary,
                         vertices1[j],
                         vertices1[(j + 1) % 4],
                         cv::Scalar(0, 255, 0),
                         4);
            }

            cv::Point2f* vertices2 = new cv::Point2f[4];
            rectA.points(vertices2);

            for (int j = 0; j < 4; j++){
                cv::line(binary,
                         vertices2[j],
                         vertices2[(j + 1) % 4],
                         cv::Scalar(0, 255, 0),
                         4);
            }
#endif

            heights[hi] = max(rect.size.height,rect.size.width)+ max(rectA.size.height,rectA.size.width) / 2;
            R[hi] = rect;
            RA[hi] = rectA;
            hi++;//统计找到的装甲板个数
        }
    }
}

void Solution ::chooseNearest() {

#ifdef NX
    Serial uart(BR115200, WORDLENGTH_8B, STOPBITS_1, PARITY_NONE, HWCONTROL_NONE);
    uart.sOpen("/dev/ttyTHS2");
#endif
    double maxh = 0;
    int mark;
    for (int i = 0; i < hi; i++) {
        if (heights[i] >= maxh) {
            maxh = heights[i];
            mark = i;
        }
    }

    if (hi != 0) 
    {
        cv::circle(binary,
                   Point((R[mark].center.x + RA[mark].center.x) / 2,
                   (R[mark].center.y + RA[mark].center.y) / 2),
                   15, cv::Scalar(0, 0, 255), 4);

        double center_x = (R[mark].center.x + RA[mark].center.x) / 2;
        double center_y = (R[mark].center.y + RA[mark].center.y) / 2;
        
        center_y = (R[mark].center.y + RA[mark].center.y) / 2;

        Point2f verticesR[4];
        R[mark].points(verticesR);
        Point2f verticesRA[4];
        RA[mark].points(verticesRA);

        if (abs(R[mark].angle) > 45) {
            x1 = (verticesR[2].x + verticesR[3].x) / 2;
            y1 = (verticesR[2].y + verticesR[3].y) / 2;
            x4 = (verticesR[1].x + verticesR[0].x) / 2;
            y4 = (verticesR[1].y + verticesR[0].y) / 2;
        } else {
            x1 = (verticesR[1].x + verticesR[2].x) / 2;
            y1 = (verticesR[1].y + verticesR[2].y) / 2;
            x4 = (verticesR[0].x + verticesR[3].x) / 2;
            y4 = (verticesR[0].y + verticesR[3].y) / 2;
        }

        if (abs(RA[mark].angle) > 45) {
            x2 = (verticesRA[2].x + verticesRA[3].x) / 2;
            y2 = (verticesRA[2].y + verticesRA[3].y) / 2;
            x3 = (verticesRA[1].x + verticesRA[0].x) / 2;
            y3 = (verticesRA[1].y + verticesRA[0].y) / 2;
        } else {
            x2 = (verticesRA[1].x + verticesRA[2].x) / 2;
            y2 = (verticesRA[1].y + verticesRA[2].y) / 2;
            x3 = (verticesRA[0].x + verticesRA[3].x) / 2;
            y3 = (verticesRA[0].y + verticesRA[3].y) / 2;
        }
    }

#ifdef PREDICT
        queue<Point2f> que;
        float pointx, pointy;
        if(hi == 0)    
        {
            cout << "1111111111111111111111111111 " <<endl;
            if(!que.empty())
            {
                que.front().x = pointx;
                que.front().y = pointy;
                cv::circle(binary,
                        Point(pointx, pointy),
                        3, cv::Scalar(255, 0, 0), 4);
                que.push(k.kal(pointx, pointy));
                que.pop();
            }
        }
        else
        {
            while(!que.empty()) que.pop();
            Point2f predict_pt = k.kal((float)center_x,(float)center_y);
            que.push(predict_pt);
            circle(binary, predict_pt, 3, Scalar(34, 255, 255), -1);
        }

        if(que.size() > 4) que.pop();


#endif

#ifdef IMSHOW
        // vector<cv::Point2f> imagePoints;
        // imagePoints.push_back(Point2f(x1, y1));
        // imagePoints.push_back(Point2f(x2, y2));
        // imagePoints.push_back(Point2f(x3, y3));
        // imagePoints.push_back(Point2f(x4, y4));

       // for (int n = 0; n < imagePoints.size(); n++) {
         //   circle(binary, imagePoints[n], 3, Scalar(255, 0, 0), -1, 8);
        //}
#endif
    if(hi != 0)
    {
        float boardw_up = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
        float boardw_down = sqrt((x3 - x4) * (x3 - x4) + (y3 - y4) * (y3 - y4));
        float boardw = (boardw_up + boardw_down) / 2;
        float boardh_left = sqrt((x1 - x4) * (x1 - x4) + (y1 - y4) * (y1 - y4));
        float boardh_right = sqrt((x2 - x3) * (x2 - x3) + (y2 - y3) * (y2 - y3));
        float boardh = (boardh_left + boardh_right) / 2;
        float board_width = abs(R[mark].center.x - RA[mark].center.x);
        float board_height = (max(R[mark].size.width, R[mark].size.height) + max(RA[mark].size.width, RA[mark].size.height)) / 2;
        float board_ratio = board_width / board_height;

        SOLVEPNP pnp;
        static float final_distance;
        float tmp;

        pnp.picture_points.push_back(Point2f(x2, y2));
        pnp.picture_points.push_back(Point2f(x1, y1));
        pnp.picture_points.push_back(Point2f(x3, y3));
        pnp.picture_points.push_back(Point2f(x4, y4));

        //判断是大装甲板还是小装甲板
        if (board_ratio < 4) {
            //小装甲板            
            // cout<<"small "<<endl;
            xishu = (13.5 / boardw + 5.4 / boardh) / 2;
                //世界坐标
            tmp = pnp.PNP(0);
            if(tmp > 10) final_distance = tmp;

        } else {
            //大装甲板
            // cout<<"big "<<endl;
            xishu = (23.5 / boardw + 5.4 / boardh) / 2;
            tmp = pnp.PNP(1);
            if(tmp > 10) final_distance = tmp;
        }
        double distance_to_midboard_x, distance_to_midboard_y;
        distance_to_midboard_x = xishu * (center_x - 320);
        distance_to_midboard_y = xishu * (center_y - 160);

        double angle_x = atan2(distance_to_midboard_x, final_distance);
        double angle_y = atan2(distance_to_midboard_y, final_distance);

        double final_angle_x = angle_x / CV_PI * 180;
        double final_angle_y = angle_y / CV_PI * 180;
        cout << "final_distance  " << final_distance <<endl;
#ifdef NX
        if(tmp > 10)
            uart.sSendData(final_angle_x, final_angle_y,final_distance,1);

#endif
    }
}


#endif //RM2022_SOLUTION_CPP
