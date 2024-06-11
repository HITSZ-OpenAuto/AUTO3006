#include <stdlib.h>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <math.h>
#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <iostream>
#include <cstdlib>

using namespace cv;

long long GetTimens(void)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
}

typedef struct
{
    int x;
    int y;
    int deg;
    float score = -100;
} Posture;

// 在图像选定的roi中, 对特定范围角度旋转的模板进行匹配
Posture matchTemplatewithRotation(Mat image, Mat temp,
                                  int roi_x1, int roi_y1, int roi_x2, int roi_y2,
                                  int angle_min, int angle_step, int angle_max)
{
    // 截取图像roi, 加速匹配
    Mat image_roi = image(Rect(roi_x1, roi_y1, roi_x2 - roi_x1, roi_y2 - roi_y1));

    Posture max_posture;

    for (int angle = angle_min; angle < angle_max; angle += angle_step)
    {
        /* 将模板图片进行旋转, 并在选取的roi中进行匹配, 得到最高score与对应的x,y坐标 */
    }

    return max_posture;
}

// 绘制旋转矩形
void rotateRectangle(Mat &image, int cent_x, int cent_y, int width, int height, int angle)
{
    // 计算旋转前左上,右下顶点
    float x1 = cent_x - width / 2;
    float y1 = cent_y - height / 2;
    float x2 = cent_x + width / 2;
    float y2 = cent_y + height / 2;

    // 计算旋转后的4顶点
    float x1_rot = (x1 - cent_x) * cos(angle * CV_PI / 180) - (y1 - cent_y) * sin(angle * CV_PI / 180) + cent_x;
    float y1_rot = (x1 - cent_x) * sin(angle * CV_PI / 180) + (y1 - cent_y) * cos(angle * CV_PI / 180) + cent_y;
    float x2_rot = (x2 - cent_x) * cos(angle * CV_PI / 180) - (y1 - cent_y) * sin(angle * CV_PI / 180) + cent_x;
    float y2_rot = (x2 - cent_x) * sin(angle * CV_PI / 180) + (y1 - cent_y) * cos(angle * CV_PI / 180) + cent_y;
    float x3_rot = (x2 - cent_x) * cos(angle * CV_PI / 180) - (y2 - cent_y) * sin(angle * CV_PI / 180) + cent_x;
    float y3_rot = (x2 - cent_x) * sin(angle * CV_PI / 180) + (y2 - cent_y) * cos(angle * CV_PI / 180) + cent_y;
    float x4_rot = (x1 - cent_x) * cos(angle * CV_PI / 180) - (y2 - cent_y) * sin(angle * CV_PI / 180) + cent_x;
    float y4_rot = (x1 - cent_x) * sin(angle * CV_PI / 180) + (y2 - cent_y) * cos(angle * CV_PI / 180) + cent_y;

    // 绘制旋转后的矩形
    line(image, Point(x1_rot, y1_rot), Point(x2_rot, y2_rot), Scalar(255, 255, 255), 1);
    line(image, Point(x2_rot, y2_rot), Point(x3_rot, y3_rot), Scalar(255, 255, 255), 1);
    line(image, Point(x3_rot, y3_rot), Point(x4_rot, y4_rot), Scalar(255, 255, 255), 1);
    line(image, Point(x4_rot, y4_rot), Point(x1_rot, y1_rot), Scalar(255, 255, 255), 1);

    // line(image, Point(x1, y1), Point(x1, y2), Scalar(255, 255, 255), 2);
    // line(image, Point(x1, y2), Point(x2, y2), Scalar(255, 255, 255), 2);
    // line(image, Point(x2, y2), Point(x2, y1), Scalar(255, 255, 255), 2);
    // line(image, Point(x2, y1), Point(x1, y1), Scalar(255, 255, 255), 2);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exp2_node"); // 初始化 ROS 节点
    ros::NodeHandle n;
    long long tim1, tim2; // 计时, 单位为ns
    tim1 = GetTimens();

    // 创建4层图像金字塔
    Mat temp[4];
    temp[0] = imread("src/exp2/src/pattern.bmp");
    cvtColor(temp[0], temp[0], COLOR_BGR2GRAY);

    /* 将模板进行高斯滤波后, 长宽分别缩小至1/2, 即为下一层金字塔 */

    tim2 = GetTimens();
    std::cout << "time:" << (tim2 - tim1) / 1000000.0 << "ms" << std::endl;

    for (int image = 1; image <= 1; image++)
    {
        std::cout << "--- image" << image << " ---" << std::endl;

        tim1 = GetTimens();
        Mat img[4];
        img[0] = imread("src/exp2/src/IMAGEB" + std::to_string(image) + ".bmp");
        cvtColor(img[0], img[0], COLOR_BGR2GRAY);

        /* 图像金字塔创建方法同上 */


        /* 每对一层进行匹配后, 将得到的坐标与角度用于下一层匹配 */
        std::cout << "angle:" << post0.deg << " x:" << post0.x + temp[0].cols / 2 << " y:" << post0.y + temp[0].rows / 2 << " score:" << post0.score << std::endl;

        // 亚像素求解
        float sub_x, sub_y, sub_deg;
        Mat rot_temp;
        Mat rot_matrix;
        Mat result;
        Mat score;

        /* 3*3邻域求score */

        Mat A = Mat::zeros(9, 6, CV_32FC1);
        Mat F = Mat::zeros(9, 1, CV_32FC1);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                // [x^2 y^2 xy x y 1]
                A.at<float>(i * 3 + j, 0) = (j - 1) * (j - 1);
                A.at<float>(i * 3 + j, 1) = (i - 1) * (i - 1);
                A.at<float>(i * 3 + j, 2) = (i - 1) * (j - 1);
                A.at<float>(i * 3 + j, 3) = j - 1;
                A.at<float>(i * 3 + j, 4) = i - 1;
                A.at<float>(i * 3 + j, 5) = 1;
                F.at<float>(i * 3 + j, 0) = score.at<float>(i, j);
            }
        }

        /* 最小二乘法拟合出曲面系数, 之后求解得到x,y */

        /* ±1°求score, 并使用二次曲线插值 */

        std::cout << "angle:" << post0.deg + sub_deg << " x:" << sub_x + post0.x + temp[0].cols / 2 << " y:" << sub_y + post0.y + temp[0].rows / 2 << std::endl;

        // 绘制像素精度匹配结果
        rotateRectangle(img[0], post0.x + temp[0].cols / 2, post0.y + temp[0].rows / 2, 95, 140, -post0.deg - 47);
        imshow("result0", img[0]);

        // 运行时间
        tim2 = GetTimens();
        std::cout << "time:" << (tim2 - tim1) / 1000000.0 << "ms" << std::endl;
    }

    while (ros::ok())
    {
        ros::spinOnce();
        waitKey(1);
    }
    return 0;
}
