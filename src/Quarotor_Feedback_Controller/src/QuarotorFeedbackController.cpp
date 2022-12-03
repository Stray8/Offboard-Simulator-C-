#include <QuarotorFeedbackController.h>

QuarotorFeedbackController::QuarotorFeedbackController(ros::NodeHandle nh){
  // ros::NodeHandle nh;

  state_sub = nh.subscribe<mavros_msgs::State>
      ("mavros/state", 10, &QuarotorFeedbackController::state_cb, this);
  local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
          ("mavros/setpoint_position/local", 10);

  attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>
          ("mavros/setpoint_raw/attitude", 10);

  //in reality
  // current_sub = nh.subscribe<geometry_msgs::PoseStamped>
  //         ("mavros/local_position/pose", 10, &QuarotorFeedbackController::local_cb, this);
  // in simulator
  current_sub = nh.subscribe<nav_msgs::Odometry>
          ("mavros/local_position/odom", 10, &QuarotorFeedbackController::local_cb, this);

  arming_client = nh.serviceClient<mavros_msgs::CommandBool>
          ("mavros/cmd/arming");
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
          ("mavros/set_mode");

  offb_set_mode.request.custom_mode = "OFFBOARD";
  arm_cmd.request.value = true;
  current_status = "Hover";
  loop_frequency = 100.0;

  // thrust eval
  eval_ptr = 0;

  // hover PID params init
  kp_hover_x = 2;
  kp_hover_y = 2;
  kp_hover_z = 1.3;
  kp_hover_vx = 0.6;
  kp_hover_vy = -0.5;
  kp_hover_vz = 0.2;

  ki_hover_x = 0;
  ki_hover_y = 0;
  ki_hover_z = 0.02;
  ki_hover_vx = 0;
  ki_hover_vy = -0.1;
  ki_hover_vz = 0.1;

  kd_hover_x = 0;
  kd_hover_y = 0;
  kd_hover_z = 0;
  kd_hover_vx = 0;
  kd_hover_vy = 0;
  kd_hover_vz = 0;
  
}

QuarotorFeedbackController::~QuarotorFeedbackController(){

}

void QuarotorFeedbackController::state_cb(const mavros_msgs::State::ConstPtr& msg){
  printf("in state callback!\n");
  current_state = *msg;
}

void QuarotorFeedbackController::local_cb(const nav_msgs::Odometry::ConstPtr& msg){
  printf("in local callback!\n");
  current_local = *msg;
  Vector3d cur_position(current_local.pose.pose.position.x, current_local.pose.pose.position.y, current_local.pose.pose.position.z);
  Vector3d cur_velocity(current_local.twist.twist.linear.x, current_local.twist.twist.linear.y, current_local.twist.twist.linear.z);
  current_position = cur_position;
  current_velocity = cur_velocity;
  current_attitude = (Quaterniond(
                      current_local.pose.pose.orientation.w, 
                      current_local.pose.pose.orientation.x, 
                      current_local.pose.pose.orientation.y, 
                      current_local.pose.pose.orientation.z)).matrix().eulerAngles(0,1,2);

}

void QuarotorFeedbackController::is_at_setpoint(){
  position_setpoint.pose.position.x = 3;
  position_setpoint.pose.position.y = 1;
  position_setpoint.pose.position.z = 2;

  Vector3d hp(position_setpoint.pose.position.x, position_setpoint.pose.position.y, position_setpoint.pose.position.z);
  Vector3d dis = current_position - hp;

  if(dis.norm() < 0.1)
    current_status = "Hover"; // enter planning
  else
    current_status = "Hover";
}

void QuarotorFeedbackController::position_control_feedback(){
  Vector3d position_cmd(position_setpoint.pose.position.x, position_setpoint.pose.position.y, position_setpoint.pose.position.z);
  Vector3d position_error = position_cmd - current_position;

  if(current_state.mode == "OFFBOARD")
    position_error_sum += position_error / loop_frequency;
  else
    position_error_sum = Vector3d(0,0,0);

  double position_i_z = ki_hover_z * position_error_sum[2]; 
  if(position_i_z > 2)
    position_i_z = 2;
  if(position_i_z < -2)
    position_i_z = -2;

  velocity_setpoint.twist.linear.x = kp_hover_x * position_error[0];
  velocity_setpoint.twist.linear.y = kp_hover_y * position_error[1];
  velocity_setpoint.twist.linear.z = kp_hover_z * position_error[2] + position_i_z;

  position_setpoint.header.stamp = ros::Time::now();
  position_setpoint.header.frame_id = "odom";
  velocity_setpoint.header.stamp = ros::Time::now();
  velocity_setpoint.header.frame_id = "odom";
}

void QuarotorFeedbackController::velocity_control_feedback(){
  Vector3d velocity_cmd(velocity_setpoint.twist.linear.x, velocity_setpoint.twist.linear.y, velocity_setpoint.twist.linear.z);
  Vector3d velocity_error  = velocity_cmd - current_velocity;

  double psi = current_attitude[2];
  Matrix3d R_E_B;
  R_E_B << cos(psi),sin(psi),0,
          -sin(psi),cos(psi),0,
            0,0,1;
  velocity_cmd = R_E_B * velocity_cmd;
  
  if(current_state.mode == "OFFBOARD")
    velocity_error_sum += velocity_error / loop_frequency;
  else
    velocity_error_sum = Vector3d(0,0,0); 

  double velocity_i_y = ki_hover_vy * velocity_error_sum[1]; 
  if(velocity_i_y > 0.4)
    velocity_i_y = 0.4;
  if(velocity_i_y < -0.4)
    velocity_i_y = -0.4; 

  double velocity_i_z = ki_hover_vz * velocity_error_sum[2]; 
  if(velocity_i_z > 0.2)
    velocity_i_z = 0.2;
  if(velocity_i_z < -0.2)
    velocity_i_z = -0.2;

  double thrust_cmd = 0.2 + kp_hover_vz * velocity_error[2] + velocity_i_z;
  if(thrust_cmd >= 0.95)
    thrust_cmd = 0.95;
  if(thrust_cmd <= 0.01)
    thrust_cmd = 0;
  printf("thrust_cmd: %lf \n", thrust_cmd);

  double theta_cmd = kp_hover_vx * velocity_error[0];
  double phi_cmd = kp_hover_vy * velocity_error[1] +  velocity_i_y;
  double psi_cmd = 0;
  if(theta_cmd > 0.2)
    theta_cmd = 0.2;
  if(theta_cmd < -0.2)
    theta_cmd = -0.2;
  if(phi_cmd > 0.2)
    phi_cmd = 0.2;
  if(phi_cmd < -0.2)
    phi_cmd = -0.2;

  thrust_eval[eval_ptr] = thrust_cmd;
  if(eval_ptr == 4)
    eval_ptr = 0;
  else
    eval_ptr += 1;

  Quaterniond oq = AngleAxisd(phi_cmd, Vector3d::UnitX()) *
                    AngleAxisd(theta_cmd, Vector3d::UnitY()) *
                    AngleAxisd(psi_cmd, Vector3d::UnitZ());
  thrust_attitude_cmd.orientation.w = oq.w();
  thrust_attitude_cmd.orientation.x = oq.x();
  thrust_attitude_cmd.orientation.y = oq.y();
  thrust_attitude_cmd.orientation.z = oq.z();
  thrust_attitude_cmd.thrust = thrust_cmd;
  // thrust_attitude_cmd.body_rate = Vector3d(0,0,0);
  thrust_attitude_cmd.type_mask = 7;
}

void QuarotorFeedbackController::topic_publish(){
  thrust_attitude_cmd.header.frame_id = "base_footprint";
  thrust_attitude_cmd.header.stamp = ros::Time::now();
  attitude_pub.publish(thrust_attitude_cmd);
}

void QuarotorFeedbackController::control(){
  ros::Rate rate(loop_frequency);  

  while (ros::ok()){
      if(current_state.mode != "OFFBOARD"){
        if(set_mode_client.call(offb_set_mode) &&
        offb_set_mode.response.mode_sent){
          // ROS_INFO("OFFBOARD enabled");
        }
      }else{
        if(!current_state.armed){
          if(arming_client.call(arm_cmd) &&
          arm_cmd.response.success){
            // ROS_INFO("Vehicle armed");
          }
        }
      }
      if(current_status == "Hover"){
        printf("enter hover!!\n");
        is_at_setpoint();
        printf("111111111111111");
        position_control_feedback();
        printf("222222222222222");
        velocity_control_feedback();
        topic_publish();
      }
      ros::spinOnce();
      rate.sleep();
    }
}
