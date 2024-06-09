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

Posture matchTemplatewithRotation(Mat image, Mat temp,
                                  int roi_x1, int roi_y1, int roi_x2, int roi_y2,
                                  int angle_min, int angle_step, int angle_max)
{
    // 截取图像roi, 加速匹配
    Mat image_roi = image(Rect(roi_x1, roi_y1, roi_x2 - roi_x1, roi_y2 - roi_y1));

    Posture max_posture;

    for (int angle = angle_min; angle < angle_max; angle += angle_step)
    {
        // 旋转模板
        Mat rot_temp;
        Mat rot_matrix = getRotationMatrix2D(Point2f(temp.cols / 2, temp.rows / 2), angle, 1);
        warpAffine(temp, rot_temp, rot_matrix, temp.size(), INTER_CUBIC, BORDER_REPLICATE, Scalar(0, 0, 0));
        // 匹配模板
        Mat result;
        matchTemplate(image_roi, rot_temp, result, TM_CCOEFF_NORMED);
        // 获取最大匹配分数
        double minVal, maxVal;
        Point minLoc, maxLoc;
        minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
        // std::cout << angle << " " << maxLoc.x << " " << maxLoc.y << " " << maxVal << std::endl;
        if (maxVal > max_posture.score)
        {
            max_posture.score = maxVal;
            max_posture.x = maxLoc.x + roi_x1;
            max_posture.y = maxLoc.y + roi_y1;
            max_posture.deg = angle;
        }
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

    for (int i = 0; i < 3; i++)
    {
        GaussianBlur(temp[i], temp[i + 1], Size(3, 3), 0, 0, BORDER_DEFAULT);
        resize(temp[i + 1], temp[i + 1], Size(temp[i + 1].cols / 2, temp[i + 1].rows / 2));
    }

    tim2 = GetTimens();
    std::cout << "time:" << (tim2 - tim1) / 1000000.0 << "ms" << std::endl;

    for (int image = 1; image <= 1; image++)
    {
        std::cout << "--- image" << image << " ---" << std::endl;

        tim1 = GetTimens();
        Mat img[4];
        img[0] = imread("src/exp2/src/IMAGEB" + std::to_string(image) + ".bmp");
        cvtColor(img[0], img[0], COLOR_BGR2GRAY);

        for (int i = 0; i < 3; i++)
        {
            GaussianBlur(img[i], img[i + 1], Size(3, 3), 0, 0, BORDER_DEFAULT);
            resize(img[i + 1], img[i + 1], Size(img[i + 1].cols / 2, img[i + 1].rows / 2));
        }

        // 匹配余量
        int rest_pix = 24;
        int rest_angle = 16;
        Posture post3 = matchTemplatewithRotation(img[3], temp[3], 0, 0, img[3].cols, img[3].rows, -180, 8, 180);

        Posture post2 = matchTemplatewithRotation(img[2], temp[2],
                                                  max(2 * post3.x - rest_pix / 4, 0), max(2 * post3.y - rest_pix / 4, 0),
                                                  min(2 * post3.x + temp[2].cols + rest_pix / 4, img[2].cols), min(2 * post3.y + temp[2].rows + rest_pix / 4, img[2].rows),
                                                  post3.deg - rest_angle, 4, post3.deg + rest_angle);

        Posture post1 = matchTemplatewithRotation(img[1], temp[1],
                                                  max(2 * post2.x - rest_pix / 2, 0), max(2 * post2.y - rest_pix / 2, 0),
                                                  min(2 * post2.x + temp[1].cols + rest_pix / 2, img[1].cols), min(2 * post2.y + temp[1].rows + rest_pix / 2, img[1].rows),
                                                  post2.deg - rest_angle / 2, 2, post2.deg + rest_angle / 2);

        Posture post0 = matchTemplatewithRotation(img[0], temp[0],
                                                  max(2 * post1.x - rest_pix, 0), max(2 * post1.y - rest_pix, 0),
                                                  min(2 * post1.x + temp[0].cols + rest_pix, img[0].cols), min(2 * post1.y + temp[0].rows + rest_pix, img[0].rows),
                                                  post1.deg - rest_angle / 4, 1, post1.deg + rest_angle / 4);
        std::cout << "angle:" << post0.deg << " x:" << post0.x + temp[0].cols / 2 << " y:" << post0.y + temp[0].rows / 2 << " score:" << post0.score << std::endl;

        // 亚像素求解
        float sub_x, sub_y, sub_deg;
        Mat rot_temp;
        Mat rot_matrix;
        Mat result;
        Mat score;

        // 计算位置
        score = Mat::zeros(3, 3, CV_32FC1);
        // 对得到angle进行模板匹配, 取3*3邻域匹配结果
        rot_matrix = getRotationMatrix2D(Point2f(temp[0].cols / 2, temp[0].rows / 2), post0.deg, 1);
        warpAffine(temp[0], rot_temp, rot_matrix, temp[0].size(), INTER_CUBIC, BORDER_REPLICATE, Scalar(0, 0, 0));
        matchTemplate(img[0], rot_temp, result, TM_CCOEFF_NORMED);
        // 获取邻域匹配分数
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                score.at<float>(i + 1, j + 1) = result.at<float>(post0.y + i, post0.x + j);
            }
        }

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

        Mat k = (A.t() * A).inv() * A.t() * F;
        // s = k1x^2 + k2y^2 + k3xy + k4x + k5y + k6
        sub_x = (2 * k.at<float>(1, 0) * k.at<float>(3, 0) - k.at<float>(2, 0) * k.at<float>(4, 0)) / (4 * k.at<float>(0, 0) * k.at<float>(1, 0) - k.at<float>(2, 0) * k.at<float>(2, 0));
        sub_y = (2 * k.at<float>(0, 0) * k.at<float>(4, 0) - k.at<float>(2, 0) * k.at<float>(3, 0)) / (4 * k.at<float>(0, 0) * k.at<float>(1, 0) - k.at<float>(2, 0) * k.at<float>(2, 0));

        // 计算角度
        score = Mat::zeros(1, 3, CV_32FC1);
        for (int i = -1; i <= 1; i++)
        {
            // 对得到x,y进行模板匹配, 取±1°匹配结果
            rot_matrix = getRotationMatrix2D(Point2f(temp[0].cols / 2, temp[0].rows / 2), post0.deg + i, 1);
            warpAffine(temp[0], rot_temp, rot_matrix, temp[0].size(), INTER_CUBIC, BORDER_REPLICATE, Scalar(0, 0, 0));
            matchTemplate(img[0], rot_temp, result, TM_CCOEFF_NORMED);
            score.at<float>(0, i + 1) = result.at<float>(post0.y, post0.x);
        }
        sub_deg = (score.at<float>(0, 0) - score.at<float>(0, 2)) / (score.at<float>(0, 0) + score.at<float>(0, 2) - 2 * score.at<float>(0, 1)) / 2;

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
