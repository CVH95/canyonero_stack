/*  CANYONERO  */

// Camera Interface

// Video Stream
// Configure video broadcast server

#ifndef CAMERA_DRIVER_H
#define CAMERA_DRIVER_H

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

namespace canyonero
{
class CameraDriver
{
public:
  CameraDriver(int width, int height);
  ~CameraDriver();

  void streaming();
  void onlineStreaming();
  void getResolution();
  Mat getFrame();

  // Public Variables
  VideoCapture cap;

private:
  // Private Variables
  int width_;
  int height_;
};
}  // namespace canyonero

#endif  // CAMERA_DRIVER_H
