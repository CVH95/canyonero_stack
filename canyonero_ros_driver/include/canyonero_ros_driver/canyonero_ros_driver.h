/*  CANYONERO  */

// ROS Interface

// MSG generation to communicate cameras and GPIOs

#ifndef CANYONERO_ROS_DRIVER_H
#define CANYONERO_ROS_DRIVER_H

// C++
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <memory>

// OpenCV
#include <opencv2/opencv.hpp>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// ROS_MSGS
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include "canyonero_msgs/canyonero_state.h"
#include "canyonero/gpio_control.h"

namespace canyonero_ros_driver
{
class DriverClass
{
private:
  // Attributes
  ros::NodeHandle nh_;
  image_transport::ImageTransport itp_;

  // Private variables
  image_transport::Publisher img_publisher_;
  ros::Publisher state_publisher_;
  ros::Subscriber sub_direction_;

  std::unique_ptr<canyonero::GpioControl> gpio_;

  // Params
  int width_;
  int height_;

  // Private Methods
  void directionCallback(const canyonero_msgs::canyonero_state msg);
  bool readParams();
  void controlMotion(std::string movement, int speed);
  void controlPlatform(std::string state, double pan, double tilt);
  void controlLeds(std::string state);

public:
  DriverClass(ros::NodeHandle node_handle);
  DriverClass(ros::NodeHandle node_handle, image_transport::ImageTransport img_transport);
  ~DriverClass();

  // Initializers
  void initPublishers();
  void initSubscribers();

  void publishFeedback();
  void publishImages(cv::Mat frame);
};
}  // namespace canyonero_ros_driver

#endif  // CANYONERO_ROS_DRIVER_H
