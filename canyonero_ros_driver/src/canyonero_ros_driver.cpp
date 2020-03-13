/*  CANYONERO  */

// ROS Interface

// MSG generation to communicate cameras and GPIOs

#include "canyonero_ros_driver/canyonero_ros_driver.h"

namespace canyonero_ros_driver
{
DriverClass(ros::NodeHandle node_handle) : nh_(node_handle)
{
  gpio_.reset(new canyonero::GpioControl());

  gpio_->setSpeed(50);
  gpio_->setPan(7, 0);
  gpio_->setTilt(11, 0);

  ROS_INFO("Canyonero ROS Driver started (command node)!");
}

DriverClass::DriverClass(ros::NodeHandle node_handle, image_transport::ImageTransport img_transport)
  : nh_(node_handle), itp_(img_transport)
{
  if (!readParams())
  {
    ROS_ERROR("Could not retrieve streaming size params. Setting default...");
    width_ = 640;
    height_ = 480;
  }

  gpio_.reset(new canyonero::GpioControl());

  gpio_->setSpeed(50);
  gpio_->setPan(7, 0);
  gpio_->setTilt(11, 0);

  ROS_INFO("Canyonero ROS Driver started (feedback node)!");
}

// Destructor
DriverClass::~DriverClass()
{
  ROS_INFO("Driver was terminated");
  ros::shutdown();
}

// GPIO Initializer
void DriverClass::initSubscriber()
{
  // Subscribers
  sub_direction_ = nh_.subscribe("joint_direction", 1, &DriverClass::directionCallback, this);

  rosSpin();
}

// Camera Initializer
void DriverClass::initPublishers()
{
  state_publisher_ = nh_.advertise<canyonero_msgs::canyonero_state>("joint_states", 1);
  img_publisher_ = itp_.advertise("eye/image_raw", 1);
}

bool readParams()
{
  bool success = true;
  success = success && ros::param::get("image_width", width_);
  success = success && ros::param::get("image_height", height_);
  return success;
}

// Direction Callback
void DriverClass::directionCallback(const canyonero_msgs::canyonero_state msg)
{
  std::string direction = msg.direction;
  std::string lights = msg.lights;
  std::string platform = msg.platform;
  int speed = msg.speed;
  double pan = msg.pan;
  double tilt = msg.tilt;

  // Send values to robot
  controlMotion(direction, speed);
  controlPlatform(platform, pan, tilt);
  controlLeds(lights);
}

void DriverClass::controlMotion(std::string movement, int speed)
{
  gpio_->setSpeed(speed);
  switch (movement)
  {
    case "FORWARD":
      gpio->moveForward();
      break;
    case "BACKWARD":
      gpio->moveBackward();
      break;
    case "RIGHT":
      gpio->turnRightOnSpot();
      break;
    case "LEFT":
      gpio->turnLeftOnSpot();
      break;
    case "STOPPED":
      gpio->stopRobot();
      break;
    default:
      gpio_->stopRobot();
      break;
  }
}

void DriverClass::controlPlatform(std::string state, double pan, double tilt)
{
  if (state == "LOCKED")
  {
    lockPlatform();
    return;
  }
  else
  {
    openPlatform();
  }
  setPan(pan, 1);
  setTilt(tilt, 1);
}

void DriverClass::controlLeds(std::string state)
{
  if (state == "ON")
  {
    gpio_->lightsOn();
  }
  else
  {
    gpio_->lightsOff();
  }
}

// Sending feedback
void DriverClass::publishFeedback()
{
  canyonero_msgs::canyonero_state msg;

  // Get values
  msg.direction = gpio_->getDirection();
  msg.speed = gpio_->getSpeed();
  msg.pan = gpio_->getPan();
  msg.tilt = gpio_->getTilt();
  msg.lights = gpio_->getLightsState();
  msg.platform = gpio_->getPlatform();

  state_publisher_.pub(msg);
}

void DriverClass::publishImages(cv::Mat frame)
{
  sensor_msgs::ImagePtr msg;
  if (!frame.empty())
  {
    ROS_ERROR("Missed frame");
    return;
  }

  cv::resize(frame, frame, Size(width_, height_));
  msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
  img_publisher.publish(msg);
  cv::waitKey(1);
}
}  // namespace canyonero_ros_driver
