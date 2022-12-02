#include <QuarotorFeedbackController.h>

QuarotorFeedbackController::QuarotorFeedbackController(ros::NodeHandle nh){
  // ros::NodeHandle nh;

  state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, &QuarotorFeedbackController::state_cb, this);

  current_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/local_position/pose", 10, &QuarotorFeedbackController::local_cb, this);
        
  vrpn_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("vrpn_client_node/danzhe/pose", 10, &QuarotorFeedbackController::vrpn_cb, this);

  // arming_client = nh.serviceClient<mavros_msgs::CommandBool>
  //         ("mavros/cmd/arming");
  // set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
  //         ("mavros/set_mode");

  local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
          ("mavros/setpoint_position/local", 10);

  vrpn_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
          ("mavros/vision_pose/pose", 10);
          
  // offb_set_mode.request.custom_mode = "OFFBOARD";
  // arm_cmd.request.value = true;
}

QuarotorFeedbackController::~QuarotorFeedbackController(){

}

void QuarotorFeedbackController::state_cb(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
}

void QuarotorFeedbackController::local_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  current_local = *msg;
  printf("current_position: [%f %f %f]\n", current_local.pose.position.x, current_local.pose.position.y, current_local.pose.position.z);
}

void QuarotorFeedbackController::vrpn_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  current_pose = *msg;
  current_pose.header.stamp = ros::Time::now();
  vrpn_pos_pub.publish(current_pose);
}

void QuarotorFeedbackController::control(geometry_msgs::PoseStamped pose){
  ros::Rate rate(100.0);  

  while (ros::ok()){
      // if(current_state.mode != "OFFBOARD"){
      //   if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
      //     // ROS_INFO("OFFBOARD enabled");
      //   }
      // }else{
      //   if(!current_state.armed){
      //     // if(arming_client.call(arm_cmd) &&
      //     // arm_cmd.response.success){
      //     //   // ROS_INFO("Vehicle armed");
      //     // }
      //   }
      // }
      pose.header.stamp = ros::Time::now();
      local_pos_pub.publish(pose);

      ros::spinOnce();
      rate.sleep();
    }
}
