/*  CANYONERO  */

// GPIO Interface

// Access GPIOs
// Configure PWMs

#ifndef GPIOCONTROL_H
#define GPIOCONTROL_H

#include <iostream>
#include <unistd.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <curses.h>
#include <string.h>
#include <cmath>

using namespace std;

// DC Motor #1
#define Motor1 17
#define Motor2 22
#define Enable1 16

// DC Motor #2
#define Motor3 23
#define Motor4 24
#define Enable2 20

// LED lights
#define led_R 21
#define led_L 13

// Camera platform
#define PAN 18
#define TILT 25

// Ultrasonic HC_SR04
#define TRIGGER 12
#define ECHO 6

namespace canyonero
{
class GpioControl
{
public:
  // Constructor
  GpioControl();

  // Destructor
  ~GpioControl();

  // Public Methods

  // Robot Control
  void moveForward();
  void moveBackward();
  void turnRightOnSpot();
  void turnLeftOnSpot();
  void stopRobot();
  void increaseSpeed();
  void decreaseSpeed();
  void setSpeed(int value);

  // Camera platform control
  void rotateUp();
  void rotateDown();
  void rotateRight();
  void rotateLeft();
  double panToDutyCycle(double deg);
  double panToDeg(int dut);
  double tiltToDutyCycle(double deg);
  double tiltToDeg(int dut);
  void setPan(double value, int unit);
  void setTilt(double value, int unit);
  void openPlatform();
  void lockPlatform();

  // Illumination
  void lightsOn();
  void lightsOff();

  // General Purpose
  std::string getDirection();
  std::string getLightsState();
  std::string getPlatform();
  int getSpeed();
  double getPan();
  double getTilt();

  void sleep(unsigned int seconds);
  // void CommandLineInterface();
  // bool keyboardRemoteControl();
  // void ros_gpio_interface(int D);
  // void startNcurses();
  // void endNcurses();

private:
  // Private Variables
  std::string direction_ = "STOP";
  std::string led_state_ = "OFF";
  std::string platform_state_ = "LOCKED";

  int duty_cycle_ = 50;
  int pan_DTC_ = 15;
  int tilt_DTC_ = 15;
  int direction_ = 0;

  bool are_on_ = false;
  bool is_locked_ = true;

  // WINDOW* win;
};
}  // namespace canyonero

#endif  // GPIOCONTROL_H
