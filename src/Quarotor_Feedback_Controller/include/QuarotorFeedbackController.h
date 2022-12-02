#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


class QuarotorFeedbackController
{
private:
    //data
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped current_local;
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    ros::Publisher local_pos_pub;
    ros::ServiceClient arming_client; 
    ros::Subscriber state_sub;
    ros::ServiceClient set_mode_client;
    ros::Subscriber current_sub;

public:
    //function
    QuarotorFeedbackController(ros::NodeHandle nh);
    ~QuarotorFeedbackController();
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void control(geometry_msgs::PoseStamped pose);
    void local_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
};


