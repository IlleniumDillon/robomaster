#include <iostream>
#include "IntelligentUAVChampionship-vision.h"

int picCount = 0;

cv::Mat getPic(int picNum);

cv::Mat getPic(int picNum)
{
	char path[1000]; //定义图片路径，供读取图片使用
	std::sprintf(path, "E:/QQData/1273797180/FileRecv/imgRing/imgRing/image_%d0.jpg", picNum);
	cv::Mat getImage = cv::imread(path);
	if (getImage.empty())
	{
		std::cout << "get Image Fail,Please check the path of picture or picture list is end!" << std::endl;
	}
	return getImage;
}

#define RUN(picNum,key) 		if (key == 'a'){\
	picNum--;\
	if (picNum == PICSTART - 1) picNum = PICSTART;}\
else if (key == 'd'){\
	picNum++;\
	if (picNum == PICEND + 1) picNum = PICSTART;}\
else if (key == 'q') break;\

int main(void)
{
	int picNum = 0;
	cv::Mat originImage;
	cv::Mat secondImage;
	while (1)
	{
		int key;
		key = cv::waitKey(0);
		RUN(picNum, key)
		originImage = getPic(picNum);
		secondImage = getPic(198 + picNum);
		imageMain(originImage,secondImage);
		//std::vector<cv::Point> cccc;
		//cv::imshow("show", originImage);
		//imageProcess(originImage,cccc);
	}
	return 0;
}