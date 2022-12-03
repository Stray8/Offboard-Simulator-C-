#pragma once

#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <Eigen/Dense>


using namespace Eigen;

struct PID{
    
};

class QuarotorFeedbackController
{
private:
    //data
    mavros_msgs::State current_state;
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    // geometry_msgs::PoseStamped current_local;
    nav_msgs::Odometry current_local;


    geometry_msgs::PoseStamped position_setpoint;
    geometry_msgs::TwistStamped velocity_setpoint;

    ros::Publisher local_pos_pub;
    ros::Publisher attitude_pub;

    mavros_msgs::AttitudeTarget thrust_attitude_cmd;

    ros::Subscriber state_sub;   
    ros::Subscriber current_sub;

    ros::ServiceClient set_mode_client;
    ros::ServiceClient arming_client; 

    std::string current_status;

    // controller params
    double loop_frequency;

    // hover PID params
    double kp_hover_x, kp_hover_y, kp_hover_z, kp_hover_vx, kp_hover_vy, kp_hover_vz;
    double ki_hover_x, ki_hover_y, ki_hover_z, ki_hover_vx, ki_hover_vy, ki_hover_vz;
    double kd_hover_x, kd_hover_y, kd_hover_z, kd_hover_vx, kd_hover_vy, kd_hover_vz;

    //thrust eval
    double thrust_eval[5];
    int eval_ptr;

    Vector3d current_position, current_velocity, current_attitude;
    Vector3d position_error_sum, velocity_error_sum;

    PID pid;

public:
    //function
    QuarotorFeedbackController(ros::NodeHandle nh);
    ~QuarotorFeedbackController();
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void control();
    // void local_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void local_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void is_at_setpoint();
    void position_control_feedback();
    void velocity_control_feedback();
    void topic_publish();
};


