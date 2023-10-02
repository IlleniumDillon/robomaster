/**
 * @file ring_detection.h
 * @brief get the ring position and yaw angle from the image
 * @author yi 1574069331@qq.com
 * @date 26 9月 2023
 */
#ifndef ROBOMASTER_RING_DETECTION_H
#define ROBOMASTER_RING_DETECTION_H
#include <airsim_ros/SetLocalPosition.h>
#include <image_transport/image_transport.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <time.h>

#include <boost/thread/thread.hpp>

#include "Eigen/Dense"
#include "airsim_ros/Circle.h"
#include "airsim_ros/CirclePoses.h"
#include "airsim_ros/GPSYaw.h"
#include "airsim_ros/Land.h"
#include "airsim_ros/PoseCmd.h"
#include "airsim_ros/Reset.h"
#include "airsim_ros/Takeoff.h"
#include "airsim_ros/VelCmd.h"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/Imu.h"
#endif // ROBOMASTER_RING_DETECTION_H

class Ring
{
 private:
  cv_bridge::CvImageConstPtr cv_front_left_ptr, cv_front_right_ptr;
  cv::Mat front_left_img, front_right_img;               // 原始图像

  // 相机内参和畸变参数
  cv::Mat cameraMatrix_left, cameraMatrix_right; // 相机内参
  cv::Mat distCoeffs_left, distCoeffs_right;     // 畸变参数
  // 摄像头相对位置
  cv::Mat R, T;

  cv::Mat imgColor, imgColor_right, imgBin, imgBin_right;
  cv::Point2f center, center_right, center_out;
  //  pcl::PointXYZ center3D;
  float angle, radius,radius_right;
  std::vector<cv::Point> contours;

  std::unique_ptr<image_transport::ImageTransport> it;

  ros::Subscriber circles_suber;    // 障碍圈参考位置
  ros::Subscriber circles_gt_suber; // 障碍圈真值
  image_transport::Subscriber front_View_suber,
      front_View_suber_right; // 订阅双目图像

  void front_view_cb(const sensor_msgs::ImageConstPtr &msg);
  void front_view_right_cb(const sensor_msgs::ImageConstPtr &msg);
  void circles_cb(const airsim_ros::CirclePoses::ConstPtr &msg);
  void circles_gt_cb(const airsim_ros::CirclePoses::ConstPtr &msg);

  // 视觉识别部分
  //
  void readImage();

  void color_process(cv::Mat color, cv::Mat &dstRed);

  void contoursFilter(cv::Mat imgbin, cv::Point2f &centerReal,
                      float &radiusTrue);

  Eigen::Vector3f coordinate_Transformation(Eigen::Vector3f input,
                                            float m_anglex, float m_angley,
                                            float m_anglez);

  cv::Point3d calculateWorldCoordinate(cv::Point2d leftPoint,
                                       cv::Point2d rightPoint);

 public:
  Ring(ros::NodeHandle *nh);
  ~Ring();
};