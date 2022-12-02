#include <QuarotorFeedbackController.h>

QuarotorFeedbackController::QuarotorFeedbackController(ros::NodeHandle nh){
  // ros::NodeHandle nh;

  state_sub = nh.subscribe<mavros_msgs::State>
      ("mavros/state", 10, &QuarotorFeedbackController::state_cb, this);
  local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
          ("mavros/setpoint_position/local", 10);
  current_sub = nh.subscribe<geometry_msgs::PoseStamped>
          ("mavros/local_position/pose", 10, &QuarotorFeedbackController::local_cb, this);
  arming_client = nh.serviceClient<mavros_msgs::CommandBool>
          ("mavros/cmd/arming");
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
          ("mavros/set_mode");

  offb_set_mode.request.custom_mode = "OFFBOARD";
  arm_cmd.request.value = true;
}

QuarotorFeedbackController::~QuarotorFeedbackController(){

}

void QuarotorFeedbackController::state_cb(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
  // printf("Hello!\n");
}

void QuarotorFeedbackController::local_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  current_local = *msg;
  // printf("Hello!\n");
  printf("current_position: [%f %f %f]\n", current_local.pose.position.x, current_local.pose.position.y, current_local.pose.position.z);
}

void QuarotorFeedbackController::control(geometry_msgs::PoseStamped pose){
  ros::Rate rate(100.0);  

  while (ros::ok()){
      if(current_state.mode != "OFFBOARD"){
        if(set_mode_client.call(offb_set_mode) &&
        offb_set_mode.response.mode_sent){
          ROS_INFO("OFFBOARD enabled");
        }
      }else{
        if(!current_state.armed){
          if(arming_client.call(arm_cmd) &&
          arm_cmd.response.success){
            ROS_INFO("Vehicle armed");
          }
        }
      }
      local_pos_pub.publish(pose);

      ros::spinOnce();
      rate.sleep();
    }
}
