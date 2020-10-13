#include "six_wheel_odom/six_wheel_odom.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "six_wheel_odom_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  SIXWHEELODOM::six_wheel_odom six_wheel_odom(nh, private_nh);

  ros::spin();

  return 0;
}
