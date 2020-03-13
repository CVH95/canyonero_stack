/*  CANYONERO  */

// ROS Drivers

// Sets up the webCam streaming.

#include "canyonero_ros_driver/canyonero_ros_driver.h"

int main(int argc, char** argv)
{
  // Start Constructors
  ros::init(argc, argv, "canyonero_feedback_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport itp;

  canyonero_ros_driver::DriverClass driver(nh, itp);
  driver.initPublishers();

  cv::VideoCapture video;
  if (!video.open(0))
  {
    ROS_ERROR("Could not access the camera");
  }
  else
  {
    ROS_INFO("Camera was accessed correctly. Starting stream!");
  }

  while (ros::ok())
  {
    cv::Mat frame;
    video >> frame;
    driver.publishImages(frame);
    driver.publishFeedback();

    ros::spinOnce();
  }

  return 0;
}
