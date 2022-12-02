#include <QuarotorFeedbackController.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  QuarotorFeedbackController c1(nh);

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 2;

  c1.control(pose);

  return 0;
}

