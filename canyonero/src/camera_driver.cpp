/*  CANYONERO  */

// Camera Interface

// Video Stream
// Configure video broadcast server

#include "camera_driver.h"

namespace canyonero
{
// Constructor
CameraDriver::CameraDriver(int width, int height) : width_(width), height_(height)
{
  if (width_ != 0 && height_ != 0)
  {
    width_ = 600;
    height_ = 480;
  }
}

// Destructor
CameraDriver::~CameraDriver()
{
  cout << "Canyonero's webcam is out" << endl;
}

// Video Streaming
void CameraDriver::streaming()
{
  // Create  Mat container for video frames
  for (;;)
  {
    Mat frame;
    cap >> frame;

    // Resize frames
    resize(frame, frame, Size(width_, height_));

    if (frame.empty())
    {
      cout << "Signal went missing" << endl;
    }

    imshow("Canyonero View", frame);

    waitKey(30);
  }
}

// Saves frames in order to be read by webServer
void CameraDriver::onlineStreaming()
{
  // Create  Mat container for video frames
  for (;;)
  {
    Mat frame;
    cap >> frame;

    // Resize frames
    resize(frame, frame, Size(width_, height_));

    if (frame.empty())
    {
      cout << "Signal went missing" << endl;
    }

    // imshow("Canyonero View", frame);
    imwrite("/home/pi/Workspace/Canyonero/code/webServer/FlaskApp/templates/frame.png", frame);

    waitKey(30);
  }
}

// Obtain frame for ROS control
Mat CameraDriver::getFrame()
{
  Mat frame;
  cap >> frame;

  // Resize frames
  resize(frame, frame, Size(width_, height_));

  if (frame.empty())
  {
    cout << "Signal went missing" << endl;
  }

  return frame;
}

// Print Resolution
void CameraDriver::getResolution()
{
  cout << "Stream resolution: " << width_ << "x" << height_ << "." << endl;
}
}  // namespace canyonero
