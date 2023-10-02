/**
 * @file ring_detection_node.cpp
 * @brief Entry point for ring detection ROS node
 * @author yi 1574069331@qq.com
 * @date 26 9月 2023
 */
#include "ring_detection.h"

int main(int argc, char** argv)
{

  ros::init(argc, argv, "ring_detection"); // 初始化ros 节点，命名为 basic
  ros::NodeHandle n; // 创建node控制句柄
  Ring ring(&n);
  return 0;
}