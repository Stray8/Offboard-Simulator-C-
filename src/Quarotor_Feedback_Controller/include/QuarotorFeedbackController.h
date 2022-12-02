#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


class QuarotorFeedbackController
{
private:
    //data which I don't know what to do
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;   

    ros::ServiceClient arming_client; 
    ros::ServiceClient set_mode_client; 

    //callback_data
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped current_local;
    geometry_msgs::PoseStamped current_pose;

    //Subscriber_data
    ros::Subscriber current_sub;
    ros::Subscriber state_sub;
    ros::Subscriber vrpn_sub;

    //Publisher_data
    ros::Publisher local_pos_pub;
    ros::Publisher vrpn_pos_pub;

public:
    //function
    QuarotorFeedbackController(ros::NodeHandle nh);
    ~QuarotorFeedbackController();
    //Callback_function
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void local_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void vrpn_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    //main_function
    void control(geometry_msgs::PoseStamped pose);
};


