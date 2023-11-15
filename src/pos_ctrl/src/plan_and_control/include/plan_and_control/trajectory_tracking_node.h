#ifndef TRAJECTORY_TRACKING_NODE_H
#define TRAJECTORY_TRACKING_NODE_H

#include <ros/ros.h>
#include <uav_msgs/AngleRateThrottle.h>
#include <uav_msgs/DesiredStates.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>

#include "tracking_controller/geometric_controller.h"

using namespace std;

class GeometricControllerNode {
private:
    //ros related
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher control_cmd_pub_;
    ros::Subscriber odom_sub_, desired_states_sub_;

    ros::Timer cmd_pub_timer_;
    ros::Time prev_time_;

    //parameters for controller
    Eigen::Vector3d odom_pos_, odom_vel_,current_pos,prev_pos_;
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d desired_pos_, desired_vel_, desired_acc_;
    Eigen::Quaterniond desired_orient_; 
    double desired_yaw_;

    GeometricController geometricController_;
    bool states_cmd_updated_, states_cmd_init_;

public:
    GeometricControllerNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    ~GeometricControllerNode();
    
    void odomCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void desiredStatesCallback(const uav_msgs::DesiredStatesConstPtr &msg);
    void pubRrPrYrTCmd();

};

#endif
