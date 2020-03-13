/*  CANYONERO  */

// ROS Drivers

// Sets up GPIO subscriber.

#include "canyonero_ros_driver/canyonero_ros_driver.h"

using namespace std;

int main(int argc, char** argv)
{
  // Start Constructors
  ros::init(argc, argv, "canyonero_feedback_node");
  ros::NodeHandle nh;

  canyonero_ros_driver::DriverClass driver(nh);
  driver.initSubscribers();

  ros::spin();
  return 0;
}
