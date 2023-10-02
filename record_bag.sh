#!/bin/bash

# 定义保存的bag文件名和话题列表
bag_file="recorded_topics.bag"
topics=(
    "/airsim_node/drone_1/circle_poses"
    "/airsim_node/drone_1/debug/circle_poses_gt"
    "/airsim_node/drone_1/debug/pose_gt"
    "/airsim_node/drone_1/debug/tree_poses_gt"
    "/airsim_node/drone_1/front_left/Scene"
    "/airsim_node/drone_1/front_left/Scene/camera_info"
    "/airsim_node/drone_1/front_right/Scene"
    "/airsim_node/drone_1/front_right/Scene/camera_info"
    "/airsim_node/drone_1/imu/imu"
    "/airsim_node/drone_1/pose"
)

# 录制bag包
rosbag record -O $bag_file ${topics[@]}
