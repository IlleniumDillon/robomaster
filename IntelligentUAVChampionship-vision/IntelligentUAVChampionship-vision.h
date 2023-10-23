// IntelligentUAVChampionship-vision.h: 标准系统包含文件的包含文件
// 或项目特定的包含文件。

#pragma once

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

//#include <Eigen/Dense>
// TODO: 在此处引用程序需要的其他标头。

#define IF_COUT
#define IF_DRAW
#define M_PI       3.14159265358979323846   // pi

extern int picCount;

void imageMain(cv::Mat leftImage,cv::Mat rightImage);
void imageProcess(cv::Mat originImage, std::vector<cv::Point>& trueContours);
inline void color_process(cv::Mat color, cv::Mat& dstBlue, cv::Mat& dstRed);
cv::Point3f uv2xyz(cv::Point2f uvLeft, cv::Point2f uvRight);
cv::Mat calculatePlaneNormalVector(const cv::Point3f& point1, const cv::Point3f& point2, const cv::Point3f& point3);

constexpr int PICSTART = 0;
constexpr int PICEND = 385;

class Circle{
public:
	cv::Point2f center;
	cv::Mat color,binary,roi;
	std::vector<cv::Point> contours;
	float radius;
	bool ifCheck;
};