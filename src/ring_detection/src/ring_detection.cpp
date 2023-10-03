/**
 * @file ring_detection.cpp
 * @brief get the ring position and yaw angle from the image
 * @author yi 1574069331@qq.com
 * @date 26 9月 2023
 */
#include "ring_detection.h"

Ring::Ring(ros::NodeHandle* nh)
{
  // 创建图像传输控制句柄
  it = std::make_unique<image_transport::ImageTransport>(*nh);
  front_left_img = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0));

  front_View_suber = it->subscribe(
      "airsim_node/drone_1/front_left/Scene", 1,
      std::bind(&Ring::front_view_cb, this, std::placeholders::_1));

  front_View_suber_right = it->subscribe(
      "airsim_node/drone_1/front_right/Scene", 1,
      std::bind(&Ring::front_view_right_cb, this, std::placeholders::_1));

  circles_suber = nh->subscribe<airsim_ros::CirclePoses>(
      "/airsim_node/drone_1/circle_poses", 1,
      std::bind(&Ring::circles_cb, this,
                std::placeholders::_1)); // 障碍圈估值数据
  circles_gt_suber = nh->subscribe<airsim_ros::CirclePoses>(
      "/airsim_node/drone_1/debug/circle_poses_gt", 1,
      std::bind(&Ring::circles_gt_cb, this,
                std::placeholders::_1)); // 障碍圈真值数据

  ros::spin();
}

void Ring::front_view_cb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_front_left_ptr =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
  front_left_img = cv_front_left_ptr->image;
  //  cv::imshow("front_left", cv_front_left_ptr->image);
  color_process(front_left_img, imgColor); // 颜色滤波
  binProcess(imgColor, mask);
  contoursFilter(mask, center, radius);
  cv::imshow("imgColor", imgColor);
  cv::imshow("mask", mask);
  cv::waitKey(10);
}

void Ring::front_view_right_cb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_front_right_ptr =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
  front_right_img = cv_front_right_ptr->image;
  color_process(front_right_img, imgColor_right); // 颜色滤波
  //  cv::imshow("front_right", cv_front_right_ptr->image);
  //  cv::waitKey(10);
}

void Ring::circles_cb(const airsim_ros::CirclePoses::ConstPtr& msg)
{
  //  ROS_INFO("Get circle poses real data.");
  //  for (int i = 0; i < msg->poses.size(); i++)
  //  {
  //    ROS_INFO("circle %d: x: %f, y: %f, z: %f, yaw: %f", i,
  //             msg->poses[i].position.x, msg->poses[i].position.y,
  //             msg->poses[i].position.z, msg->poses[i].yaw);
  //  }
  // 共有17个圆环
  // part1 0-4 红色圆环
}

void Ring::circles_gt_cb(const airsim_ros::CirclePoses::ConstPtr& msg)
{
  //  ROS_INFO("Get circle poses real data.");
  //  for (int i = 0; i < msg->poses.size(); i++)
  //  {
  //    ROS_INFO("circle real%d: x: %f, y: %f, z: %f, yaw: %f", i,
  //             msg->poses[i].position.x, msg->poses[i].position.y,
  //             msg->poses[i].position.z, msg->poses[i].yaw);
  //  }
  // 共有17个圆环
  // part1 0-4 红色圆环
}

Ring::~Ring()
{
  // cv::destroyAllWindows();
}

void Ring::readImage()
{
  const std::string imagePath =
      "/home/yi/CLionProjects/robomaster/src/pos_ctrl/image/image_200.jpg";
  front_left_img = cv::imread(imagePath);
  if (front_left_img.empty())
  {
    std::cout << "Failed to read image: " << imagePath << std::endl;
    return;
  }
}

void Ring::color_process(cv::Mat color, cv::Mat& dstRed)
{
  int g_nHm = 9;
  std::vector<cv::Mat> channelsBGR;
  cv::split(color, channelsBGR);
  cv::Mat imageBlueChannel = channelsBGR.at(0);
  cv::Mat imageGreenChannel = channelsBGR.at(1);
  cv::Mat imageRedChannel = channelsBGR.at(2);
  auto r_imageRedChannel =
      imageRedChannel - imageGreenChannel - imageBlueChannel;
  dstRed = r_imageRedChannel * 3;

  //  cv::imshow("dstRed", dstRed);
  //  cv::waitKey(10);
}

void Ring::front_view_write()
{
  // 存储图像的频率（以帧为单位）
  int saveFrequency = 10;

  // 存储图像的路径和文件名
  std::string savePath =
      "/home/yi/CLionProjects/robomaster/src/pos_ctrl/imgColor";
  std::string fileName = "image";

  // 检查是否需要存储图像
  static int frameCount = 0;
  if (frameCount % saveFrequency == 0)
  {
    std::string filePath =
        savePath + "/" + fileName + "_" + std::to_string(frameCount) + ".jpg";
    cv::imwrite(filePath, imgColor);
  }
  frameCount++;
}

void Ring::contoursFilter(cv::Mat imgbin, cv::Point2f& centerReal,
                          float& radiusTrue)
{
  // cv::GaussianBlur(imgbin, imgbin, cv::Size(3, 3), 5);
  // cv::Canny(imgbin, imgbin, 100, 200, 3);
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  std::vector<cv::Point2f> centerList;
  std::vector<float> radiusList;
  std::vector<std::vector<cv::Point>> contoursList;
  findContours(imgbin, contours, hierarchy, cv::RETR_EXTERNAL,
               cv::CHAIN_APPROX_SIMPLE);
  // 创建一个空白图像用于绘制轮廓
  cv::Mat contourImage = cv::Mat::zeros(imgbin.size(), CV_8UC3);
  // 绘制轮廓
  cv::drawContours(contourImage, contours, -1, cv::Scalar(0, 0, 255), 2);

  //  flag = 1;
  for (int i = 0; i < contours.size(); i++)
  {
    double area = cv::contourArea(contours[i]); // 计算轮廓面积
    //      std::cout<<"area*********************************"<<area<<std::endl;
    if (area < 300) continue;

    cv::Point2f center; // 初步检测的圆的坐标
    float radius;
    cv::minEnclosingCircle(contours[i], center, radius);
    if (imgbin.at<uchar>(center.y, center.x) != 0) continue;
    // 显示原始图像和绘制轮廓后的图像
    cv::circle(contourImage, center, 5, cv::Scalar(0, 255, 0), -1);
    cv::imshow("Original Image", imgbin);
    cv::imshow("Contours", contourImage);
    //    cv::waitKey(10);
  }
}

cv::Point3d Ring::calculateWorldCoordinate(cv::Point2d leftPoint,
                                           cv::Point2d rightPoint)
{
  // TODO:以下参数待重新标定
  cameraMatrix_left =
      (cv::Mat_<double>(3, 3) << 87.3275594874194, 0, 400.263391651105, 0,
       71.5629185891200, 291.471402908258, 0, 0, 1);
  cameraMatrix_right =
      (cv::Mat_<double>(3, 3) << 82.0326157178800, 0, 389.792187255525, 0,
       75.7458515405673, 289.034180725912, 0, 0, 1);
  cv::Mat distCoeffs_left =
      (cv::Mat_<double>(1, 5) << 0.0109638715157618, -0.000348234482810868,
       -0.00151188351400409, 0.0228993508197386, 7.44235141199170e-06);
  cv::Mat distCoeffs_right =
      (cv::Mat_<double>(1, 5) << -0.000689843468851084, 0.000199239812283484,
       -0.00417451170840566, 0.00983138841966482, -3.57835528361395e-06);
  R = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1); // 安装尺寸计算获得
  T = (cv::Mat_<double>(3, 1) << -118.049376282093, 0.196071535276959,
       9.77032520934541);
  // 去畸变
  cv::Mat matLeft(1, 1, CV_64FC2), matRight(1, 1, CV_64FC2);
  matLeft.at<cv::Vec2d>(0, 0) = cv::Vec2d(leftPoint.x, leftPoint.y);
  matRight.at<cv::Vec2d>(0, 0) = cv::Vec2d(rightPoint.x, rightPoint.y);

  cv::Mat leftUndistorted, rightUndistorted;
  cv::undistortPoints(matLeft, leftUndistorted, cameraMatrix_left,
                      distCoeffs_left);
  cv::undistortPoints(matRight, rightUndistorted, cameraMatrix_right,
                      distCoeffs_right);

  // 构建左右目摄像头的投影矩阵
  cv::Mat leftProjectionMatrix = cv::Mat::eye(3, 4, CV_64F);
  cv::Mat rightProjectionMatrix = cv::Mat::eye(3, 4, CV_64F);
  R.copyTo(leftProjectionMatrix(cv::Rect(0, 0, 3, 3)));
  T.copyTo(rightProjectionMatrix(cv::Rect(3, 0, 1, 3)));
  cv::Mat rightRotationMatrix;
  cv::Rodrigues(R, rightRotationMatrix);
  rightRotationMatrix.copyTo(rightProjectionMatrix(cv::Rect(0, 0, 3, 3)));

  // 三角化
  cv::Mat points4D;
  cv::triangulatePoints(leftProjectionMatrix, rightProjectionMatrix,
                        leftUndistorted, rightUndistorted, points4D);

  // 归一化坐标
  cv::Mat normalizedPoints;
  cv::convertPointsFromHomogeneous(points4D.t(), normalizedPoints);

  return cv::Point3f(normalizedPoints.at<float>(0, 0),
                     normalizedPoints.at<float>(0, 1),
                     normalizedPoints.at<float>(0, 2));
}

void Ring::binProcess(cv::Mat image, cv::Mat& mask)
{
  // 对图片进行二值化处理
  cv::Mat binary_image;
  cv::threshold(image, binary_image, 239, 255, cv::THRESH_BINARY);

  // 寻找连通域
  cv::Mat labels, stats, centroids;
  int num_labels = cv::connectedComponentsWithStats(binary_image, labels, stats,
                                                    centroids, 8);

  // 找到最大连通域
  int largest_component_label = 1;
  int max_area = stats.at<int>(1, cv::CC_STAT_AREA);
  for (int i = 2; i < num_labels; i++)
  {
    int area = stats.at<int>(i, cv::CC_STAT_AREA);
    if (area > max_area)
    {
      max_area = area;
      largest_component_label = i;
    }
  }

  // 创建一个与原始图像大小相同的掩码图像
  mask = cv::Mat::zeros(image.size(), CV_8UC1);

  // 将最大连通域的像素设置为白色
  for (int i = 0; i < mask.rows; i++)
  {
    for (int j = 0; j < mask.cols; j++)
    {
      if (labels.at<int>(i, j) == largest_component_label)
      {
        mask.at<uchar>(i, j) = 255;
      }
    }
  }

  /*TODO:之后对ROI区域进行分析以减少运算量
  // 获取最大连通域的边界框
  cv::Rect roi = cv::boundingRect(mask);

  // 提取最大连通域所在的ROI
  cv::Mat roi_image = image(roi);*/

  // 显示原始图片和最大连通域
  //  cv::imshow("Original Image", image);
  //  cv::imshow("Binarized Image", binary_image);
  //  cv::imshow("Largest Connected Component", mask);
  //  cv::imshow("ROI", roi_image);
  //  cv::waitKey(0);
}
