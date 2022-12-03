#include "QuarotorFeedbackController.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  QuarotorFeedbackController c1(nh);

  c1.control();

  return 0;
}

