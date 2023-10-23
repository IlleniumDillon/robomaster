// IntelligentUAVChampionship-vision.cpp: 定义应用程序的入口点。
//

#include "IntelligentUAVChampionship-vision.h"

Circle red, yellow;

//左相机内参数矩阵
float leftIntrinsic[3][3] = { 1.700301517410894e+03,			 0.326893673245990,		6.270573283828736e+02,
                             0,	1.700233516756168e+03,		5.160954119157802e+02,
                             0,			 0,				1 };
//左相机旋转矩阵
float leftRotation[3][3] = {0.912333,		-0.211508,		 0.350590,
                            0.023249,		-0.828105,		-0.560091,
                            0.408789,		 0.519140,		-0.750590};
//左相机平移向量
float leftTranslation[1][3] = {-127.199992, 28.190639, 1471.356768};

//右相机内参数矩阵
float rightIntrinsic[3][3] = { 1.722799249643528e+03,			 -0.922575632391725,		6.167081176419775e+02,
                              0,	1.723235893974104e+03,		5.073103330322222e+02,
                              0,			 0,				1 };
//右相机旋转矩阵
float rightRotation[3][3] = { 0.998977143902391,		 -0.002835217348746,		0.045129009552698,
                             0.003223939331885,		 0.999958309786264,		-0.008543120307155,
                             -0.045102906511737,		0.008679875113363,		0.998944636900469 };
//右相机平移向量
float rightTranslation[1][3] = { -5.001160296664356e+02, -6.098120049765349, 29.410374114366885 };

inline void color_process(cv::Mat color, cv::Mat& dstBlue, cv::Mat& dstRed) {
    std::vector<cv::Mat> channelsBGR;
    cv::split(color, channelsBGR);
    cv::Mat imageBlueChannel1 = channelsBGR.at(0);
    cv::Mat imageGreenChannel1 = channelsBGR.at(1);
    cv::Mat imageRedChannel1 = channelsBGR.at(2);
    /////////////////////////////////////Blue/////////////////////////////////////
    auto b_imageBlueChannel1 = imageBlueChannel1 - imageGreenChannel1 - imageRedChannel1;
    dstBlue = b_imageBlueChannel1 * 3;
    auto r_imageRedChannel = imageRedChannel1 - imageGreenChannel1 - imageBlueChannel1;
    //cv::imshow("splitred", r_imageRedChannel);
    dstRed = r_imageRedChannel * 3;
    //    cv::imwrite("imgBlue.jpg", dstBlue);
    //////////////////////////////////Yellow////////////////////////////////////////
    //    cv::Mat imgYCB, out;
    //    cv::cvtColor(color, imgYCB, cv::COLOR_BGR2YCrCb);
    //    cv::Scalar lower(82, 113, 60);
    //    cv::Scalar upper(240, 155, 105);
    //    cv::inRange(imgYCB, lower, upper, out);
    //    dstRed = dstRed - out;

}


inline void binProcess(cv::Mat imgColor, cv::Mat& imgBin, float thresh) {
    double maxValue_gary;
    cv::Mat imgBin1, imgBin2, imgBin3;
    cv::Mat hchannel, schannel, vchannel;
    //cv::cvtColor(imgColor, imgColor, cv::COLOR_BGR2HSV);
    cv::minMaxLoc(imgColor, 0, &maxValue_gary, 0, 0);
    cv::threshold(imgColor, imgBin, maxValue_gary * thresh, 255, cv::THRESH_BINARY);
    //cv::imshow("高斯滤波前", imgBin);
    cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(imgBin, imgBin, cv::MORPH_OPEN, element);
    //cv::imshow("腐蚀前", imgBin);s
    // 定义核（结构元素）用于形态学操作
    //int kernelSize = 5; // 核的大小，可以根据需要调整
    //cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));
    //cv::erode(imgBin, imgBin, kernel);
    //cv::imshow("二值化",imgBin);
    //    cv::imwrite("imgBin.jpg", imgBin);
}


void detectRing(cv::Mat binaryImage,cv::Mat drawImage, std::vector<cv::Point2f>& center, std::vector<float>& radius)
{
    // 使用Hough变换检测圆环
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(binaryImage, circles, cv::HOUGH_GRADIENT, 1, binaryImage.rows / 8, 100, 20, 10, 100);

    // 在原始图像上绘制检测到的圆环

    // 将检测到的圆环信息保存到输出向量中
    for (size_t i = 0; i < circles.size(); i++)
    {
        cv::Point center_point(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius_value = cvRound(circles[i][2]);

        center.push_back(center_point);
        radius.push_back(radius_value);
        // 绘制圆心
        cv::circle(drawImage, center_point, 3, cv::Scalar(0, 0, 255), -1, 8, 0);

        // 绘制圆的边界
        cv::circle(drawImage, center_point, radius_value, cv::Scalar(0, 255, 0), 2, 8, 0);
    }
    // 显示可视化结果
    cv::imshow("Detected Rings", drawImage);
    printf("the ring number is %d\n", center.size());
}

void contoursFilter(cv::Mat imgbin,cv::Mat& imgNewBin, bool& flag, cv::Point2f& centerReal,
    float& radiusTrue, std::vector<cv::Point>& contoursReal) {
    //cv::GaussianBlur(imgbin, imgbin, cv::Size(3, 3), 5);
    //cv::Canny(imgbin, imgbin, 100, 200, 3);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Point2f> centerList;
    std::vector<float> radiusList;
    std::vector<std::vector<cv::Point>> contoursList;
    findContours(imgbin, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    flag = 1;
    for (int i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]); //计算轮廓面积
        if (area < 300)
            continue;
        std::cout << "I had check it!" << std::endl;
        cv::Point2f center; //初步检测的圆的坐标
        float radius;
        cv::minEnclosingCircle(contours[i], center, radius);
        auto len = arcLength(contours[i], true);           //计算轮廓周长
        auto roundness = (4 * CV_PI * area) / (len * len); //圆形度
        if (imgbin.at<uchar>(center.y, center.x) != 255)
            continue;
        printf("The roundness is %d\n", roundness);
        if (roundness < 0.8425)
            continue;
        /*if(radius / centerTakiZ < 0.003 || radius / centerTakiZ >=1)
            continue;*/
        flag = 0;
        drawContours(imgNewBin, contours, i, cv::Scalar(255), -1);
        //        cv::imwrite("imgNewBin.jpg", imgNewBin);
        contoursList.push_back(contours[i]);
        centerList.push_back(center);
        radiusList.push_back(radius);
    }
    if (centerList.size() == 1) {
        centerReal.x = centerList[0].x;
        centerReal.y = centerList[0].y;
        //        centerReal.z = depth.at<ushort>(centerReal.y, centerReal.x);
        radiusTrue = radiusList[0];
        contoursReal = contoursList[0];
    }
}

/**
 * @brief 在获得的图像中获得最合适的轮廓
 * @param[in] src           输入的图像
 * @return std::vector<cv::Point> 最合适的椭圆轮廓
 * @author Cantorxu (1273797180@qq.com)
 */
std::vector<cv::Point> getMaxcontour(const cv::Mat& src,cv::Mat& copy_image)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierachy;
    std::vector<cv::Point> max, dst_con;
    float max_area = 0;
    bool ifOut = 0;
    // cv::Mat kernel3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,
    // 5));
    cv::findContours(src, contours, hierachy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    float contoursArea, rectArea, contoursLength, rectLength, maxcontoursArea = 0, maxcontoursArea2 = 0;
    cv::Rect rect;
    cv::RotatedRect rRect;
    int count1 = 0, count2 = 0, ith1 = -1, ith2 = -1; //最大的两个连通区域
    float rate1, rate2, rate3;
    // fitEllipseRansac ransac;
    for (std::size_t i = 0; i < contours.size(); i++)
    {
        contoursArea = cv::contourArea(contours[i]);
        //直接求面积最大的两个连通区域
        if (contoursArea < 300)
        {
            continue;
        }
        printf("\033[31m"
            "轮廓面积判断通过！\n"
            "\033[0m");
        // else if (contoursArea > maxcontoursArea)
        // {
        //     maxcontoursArea2 = maxcontoursArea;
        //     maxcontoursArea = contoursArea;
        //     ith2 = ith1;
        //     ith1 = i;
        // }
        // else if (contoursArea > maxcontoursArea2)
        // {
        //     maxcontoursArea2 = contoursArea;
        //     ith2 = i;
        // }
        contoursLength = cv::arcLength(contours[i], true);
        rRect = cv::fitEllipse(contours[i]);
        rectArea = M_PI * rRect.size.width * rRect.size.height / 4;
        rate1 = contoursArea / rectArea;
        std::cout << "The rate of area: " << rate1 << std::endl;
        rRect.size.width > rRect.size.height
            ? (rectLength = M_PI * rRect.size.height + 2 * (rRect.size.width - rRect.size.height))
            : (rectLength = M_PI * rRect.size.width + 2 * (rRect.size.height - rRect.size.width));
        // 求椭圆周长
        rate2 = contoursLength / rectLength;
        std::cout << "The rate of length: " << rate2 << std::endl;
        std::cout << "椭圆角度为：" << rRect.angle << std::endl;
        if (abs(rate1-1) > 0.65 || abs(rate2 - 1) > 0.2) continue;
        cv::Point start, end;
        cv::Point2f vertices[4];
        rRect.points(vertices);
        for (std::size_t z = 0; z < 4; z++)
        {
            if (vertices[z].x <= 0 || vertices[z].y <= 0)
            {
                ifOut = 1;
                printf("\033[31m"
                    "椭圆拟合越界！\n"
                    "\033[0m");
                continue;
            }
        }
        if (ifOut)
            return max;
        float k1, b1;
        if (rRect.size.width < rRect.size.height)
        {
            start = (vertices[3] + vertices[0]) / 2;
            end = (vertices[1] + vertices[2]) / 2;
        }
        else
        {
            end = (vertices[3] + vertices[0]) / 2;
            start = (vertices[1] + vertices[2]) / 2;
        }
        if (start.y == end.y) //当外接矩形平躺着时
        {
            int xStart = 0, xEnd = 0, y0 = start.y;
            start.x < end.x ? (xStart = start.x, xEnd = end.x) : (xStart = end.x, xEnd = start.x);
            for (int x = xStart; x < xEnd; x++)
            {
                count1++;
                if (src.at<uchar>(y0, x) == 255)
                    count2++;
            }
        }
        else if (start.x == end.x) //当外接矩形站着时
        {
            int yStart = 0, yEnd = 0, x0 = start.x;
            start.y < end.y ? (yStart = start.y, yEnd = end.y) : (yStart = end.y, yEnd = start.y);
            for (int y = yStart; y < yEnd; y++)
            {
                count1++;
                if (src.at<uchar>(y, x0) == 255)
                    count2++;
            }
        }
        else
        {
            k1 = (float)(end.y - start.y) / (float)(end.x - start.x);
            b1 = rRect.center.y - k1 * rRect.center.x;
            int xStart, xEnd, y0;
            end.x < start.x ? (xStart = end.x, xEnd = start.x) : (xStart = start.x, xEnd = end.x);
            for (int x = xStart; x < xEnd; x++)
            {
                y0 = k1 * x + b1;
                count1++;
                if (src.at<uchar>(y0, x) == 255)
                    count2++;
            }
        }
        rate3 = float(count2) / float(count1);
         std::cout << "rate3: " << rate3 << std::endl;
   /*     if (rate3 <= 0.93 || contoursArea < max_area)
        {
            continue;
        }*/
        max_area = contoursArea;
        max = contours[i];
    }
    // max.clear();
    // if (ith1 == -1)
    // {
    //     return max;
    // }
    // else if (ith2 == -1)
    // {
    //     max.assign(contours[ith1].begin(), contours[ith1].end());
    //     return max;
    // }
    // else
    // { //寻找面积/周长最大的轮廓
    //     double rate1 = cv::contourArea(contours[ith1]) / cv::arcLength(contours[ith1], true);
    //     double rate2 = cv::contourArea(contours[ith2]) / cv::arcLength(contours[ith2], true);
    //     if (rate1 >= rate2)
    //     {
    //         return contours[ith1];
    //     }
    //     else
    //     {
    //         return contours[ith2];
    //     }
    // }
    // ifArcContour(max);
    return max;
}

int coordinate_Transformation() 
{
    // 相机内部参数
    double fx = 1000.0;  // 焦距
    double fy = 1000.0;
    double cx = 320.0;   // 光学中心
    double cy = 240.0;

    // 像素坐标
    double u = 400.0;
    double v = 300.0;

    // 深度信息
    double Z = 2.0;

    // 归一化坐标
    double x_norm = (u - cx) / fx;
    double y_norm = (v - cy) / fy;

    // 相机坐标系下的点
    double X_c = x_norm * Z;
    double Y_c = y_norm * Z;

    // 外部参数 - 旋转矩阵（假设为单位矩阵）和平移向量
    //Eigen::Matrix3d R = Eigen::Matrix3d::Identity(); // 单位矩阵
    //Eigen::Vector3d T(0.0, 0.0, 0.0); // 平移向量

    // 相机坐标系下的点转换到世界坐标系下
    //Eigen::Vector3d camera_point(X_c, Y_c, Z);
    //Eigen::Vector3d world_point = R * camera_point + T;
    
    // 计算yaw角度（假设使用欧拉角表示）
    //double yaw = std::atan2(R(1, 0), R(0, 0));

    // 输出结果
    //std::cout << "X: " << world_point(0) << std::endl;
    //std::cout << "Y: " << world_point(1) << std::endl;
    //std::cout << "Z: " << world_point(2) << std::endl;
    //std::cout << "Yaw: " << yaw << std::endl;

    return 0;
}

//************************************
// Description: 根据左右相机中成像坐标求解空间坐标
// Method:    uv2xyz
// FullName:  uv2xyz
// Access:    public
// Parameter: Point2f uvLeft
// Parameter: Point2f uvRight
// Returns:   cv::Point3f
// Author:    小白
// Date:      2017/01/10
// History:
//************************************
cv::Point3f uv2xyz(cv::Point2f uvLeft, cv::Point2f uvRight)
{
    //  [u1]      |X|					  [u2]      |X|
    //Z*|v1| = Ml*|Y|					Z*|v2| = Mr*|Y|
    //  [ 1]      |Z|					  [ 1]      |Z|
    //			  |1|								|1|
    cv::Mat mLeftRotation = cv::Mat(3, 3, CV_32F, leftRotation);
    cv::Mat mLeftTranslation = cv::Mat(3, 1, CV_32F, leftTranslation);
    cv::Mat mLeftRT = cv::Mat(3, 4, CV_32F);//左相机M矩阵
    hconcat(mLeftRotation, mLeftTranslation, mLeftRT);
    cv::Mat mLeftIntrinsic = cv::Mat(3, 3, CV_32F, leftIntrinsic);
    cv::Mat mLeftM = mLeftIntrinsic * mLeftRT;
    //cout<<"左相机M矩阵 = "<<endl<<mLeftM<<endl;

    cv::Mat mRightRotation = cv::Mat(3, 3, CV_32F, rightRotation);
    cv::Mat mRightTranslation = cv::Mat(3, 1, CV_32F, rightTranslation);
    cv::Mat mRightRT = cv::Mat(3, 4, CV_32F);//右相机M矩阵
    hconcat(mRightRotation, mRightTranslation, mRightRT);
    cv::Mat mRightIntrinsic = cv::Mat(3, 3, CV_32F, rightIntrinsic);
    cv::Mat mRightM = mRightIntrinsic * mRightRT;
    //cout<<"右相机M矩阵 = "<<endl<<mRightM<<endl;

    //最小二乘法A矩阵
    cv::Mat A = cv::Mat(4, 3, CV_32F);
    A.at<float>(0, 0) = uvLeft.x * mLeftM.at<float>(2, 0) - mLeftM.at<float>(0, 0);
    A.at<float>(0, 1) = uvLeft.x * mLeftM.at<float>(2, 1) - mLeftM.at<float>(0, 1);
    A.at<float>(0, 2) = uvLeft.x * mLeftM.at<float>(2, 2) - mLeftM.at<float>(0, 2);

    A.at<float>(1, 0) = uvLeft.y * mLeftM.at<float>(2, 0) - mLeftM.at<float>(1, 0);
    A.at<float>(1, 1) = uvLeft.y * mLeftM.at<float>(2, 1) - mLeftM.at<float>(1, 1);
    A.at<float>(1, 2) = uvLeft.y * mLeftM.at<float>(2, 2) - mLeftM.at<float>(1, 2);

    A.at<float>(2, 0) = uvRight.x * mRightM.at<float>(2, 0) - mRightM.at<float>(0, 0);
    A.at<float>(2, 1) = uvRight.x * mRightM.at<float>(2, 1) - mRightM.at<float>(0, 1);
    A.at<float>(2, 2) = uvRight.x * mRightM.at<float>(2, 2) - mRightM.at<float>(0, 2);

    A.at<float>(3, 0) = uvRight.y * mRightM.at<float>(2, 0) - mRightM.at<float>(1, 0);
    A.at<float>(3, 1) = uvRight.y * mRightM.at<float>(2, 1) - mRightM.at<float>(1, 1);
    A.at<float>(3, 2) = uvRight.y * mRightM.at<float>(2, 2) - mRightM.at<float>(1, 2);

    //最小二乘法B矩阵
    cv::Mat B = cv::Mat(4, 1, CV_32F);
    B.at<float>(0, 0) = mLeftM.at<float>(0, 3) - uvLeft.x * mLeftM.at<float>(2, 3);
    B.at<float>(1, 0) = mLeftM.at<float>(1, 3) - uvLeft.y * mLeftM.at<float>(2, 3);
    B.at<float>(2, 0) = mRightM.at<float>(0, 3) - uvRight.x * mRightM.at<float>(2, 3);
    B.at<float>(3, 0) = mRightM.at<float>(1, 3) - uvRight.y * mRightM.at<float>(2, 3);

    cv::Mat XYZ = cv::Mat(3, 1, CV_32F);
    //采用SVD最小二乘法求解XYZ
    cv::solve(A, B, XYZ, cv::DECOMP_SVD);

    //std::cout<<"空间坐标为 = "<<std::endl<< XYZ <<std::endl;

    //世界坐标系中坐标
    cv::Point3f world;
    world.x = XYZ.at<float>(0, 0);
    world.y = XYZ.at<float>(1, 0);
    world.z = XYZ.at<float>(2, 0);

    return world;
}

cv::Mat calculatePlaneNormalVector(const cv::Point3f& point1, const cv::Point3f& point2, const cv::Point3f& point3) 
{
    cv::Mat vector1 = cv::Mat(point2 - point1);
    cv::Mat vector2 = cv::Mat(point3 - point1);

    cv::Mat normalVector = vector1.cross(vector2);  // 使用叉乘计算法向量

    // 归一化法向量
    double norm = cv::norm(normalVector);
    if (norm > 0) {
        normalVector /= norm;
    }

    return normalVector;
}

void imageProcess(cv::Mat originImage,std::vector<cv::Point>& trueContours) 
{
    picCount++;
#ifdef IF_COUT
    printf("\033[36m"
        "--------   "
        "\033[33m"
        "%d"
        "\033[36m"
        "   --------\n"
        "\033[0m",
        picCount);
#endif
    std::vector<cv::Point2f> centerVector;
    std::vector<float> radiusVector;
    cv::imshow("origin", originImage);
    red.roi = originImage.clone();
    std::vector<cv::Point> contoursReal;
    //cv::namedWindow("origin");
    //cv::namedWindow("red");
    color_process(originImage, yellow.color, red.color);
    //cv::imshow("red", red.color);
    binProcess(red.color, red.binary, 0.3);
    //cv::imshow("binary", red.binary);
    //detectRing(red.binary, red.roi, centerVector, radiusVector);
    trueContours=getMaxcontour(red.binary, red.roi);
    if (trueContours.empty())
    {
        std::cout << "Fail!" << std::endl;
        return;
    }
#ifdef IF_DRAW
    cv::Rect lRect = cv::boundingRect(trueContours);
    cv::Point leftPointL = cv::Point(lRect.x, lRect.y + lRect.height / 2);
    cv::Point leftPointR = cv::Point(lRect.x + lRect.width, lRect.y + lRect.height / 2);
    cv::line(red.roi, leftPointL, leftPointR, cv::Scalar(199, 194, 0), 2, 8, 0);
    cv::imshow("check", red.roi);
#endif // IF_DRAW

	return;
}

void imageMain(cv::Mat leftImage, cv::Mat rightImage)
{
    std::vector<cv::Point>  leftContours, rightContours;
    imageProcess(leftImage, leftContours);
    imageProcess(rightImage, rightContours);
    if (!leftContours.empty() && !rightContours.empty())//找到轮廓后开始锁定三点计算平面
    {
        cv::Rect lRect = cv::boundingRect(leftContours);
        cv::Point leftPointL = cv::Point(lRect.x, lRect.y + lRect.height / 2);
        //cv::Point rightPointL = cv::Point(lRect.x + lRect.width, lRect.y + lRect.height / 2);
        cv::Point upPointL = cv::Point(lRect.x + lRect.width / 2, lRect.y);
        cv::Point centerL = cv::Point(lRect.x + lRect.width / 2, lRect.y + lRect.height / 2);
        cv::Rect rRect = cv::boundingRect(rightContours);
        cv::Point leftPointR = cv::Point(rRect.x, rRect.y + rRect.height / 2);
        //cv::Point rightPointR = cv::Point(rRect.x + rRect.width, rRect.y + rRect.height / 2);
        cv::Point upPointR = cv::Point(rRect.x + rRect.width / 2, rRect.y);
        cv::Point centerR = cv::Point(rRect.x + rRect.width / 2, rRect.y + rRect.height / 2);

        cv::Point3f left3DPoint = uv2xyz(leftPointL, leftPointR);
        //cv::Point3f right3DPoint = uv2xyz(rightPointL, rightPointR);
        cv::Point3f up3DPoint = uv2xyz(upPointL, upPointR);
        cv::Point3f center3DPoint = uv2xyz(centerL, centerR);

        cv::Mat normalVector = calculatePlaneNormalVector(left3DPoint, up3DPoint, center3DPoint);
        std::cout << "三维法向量为：" << normalVector << std::endl;
    }
}
