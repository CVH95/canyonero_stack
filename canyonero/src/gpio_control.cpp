/*  CANYONERO  */

// GPIO Interface

// Access GPIOs
// Configure PWMs

#include "GpioControl.h"

namespace canyonero
{
// Constructor
GpioControl::GpioControl()
{
  if (wiringPiSetupGpio() == -1)
  {
    std::cout << "WiringPi failed to set up GPIOs" << std::endl;
    // return 1;
  }

  // This has to do with permissions to run hardware PWM
  putenv("WIRINGPI_GPIOMEM=1");

  // DC Motor Pins
  pinMode(Motor1, OUTPUT);
  pinMode(Motor2, OUTPUT);
  pinMode(Motor3, OUTPUT);
  pinMode(Motor4, OUTPUT);

  // DC Motor Enable Pins
  pinMode(Enable1, OUTPUT);
  pinMode(Enable2, OUTPUT);

  // LED Light Pins
  pinMode(led_R, OUTPUT);
  pinMode(led_L, OUTPUT);
  are_on_ = false;

  // Create PWMs
  softPwmCreate(Enable1, 0, 100);
  softPwmCreate(Enable2, 0, 100);

  // Initiate (all pins LOW)
  digitalWrite(Motor1, 0);
  digitalWrite(Motor2, 0);
  digitalWrite(Motor3, 0);
  digitalWrite(Motor4, 0);
  digitalWrite(led_R, 0);
  digitalWrite(led_L, 0);
}

// Destructor (clean up)
GpioControl::~GpioControl()
{
  // Set all Values to low

  // DC Motors OFF
  digitalWrite(Motor1, 0);
  digitalWrite(Motor2, 0);
  digitalWrite(Motor3, 0);
  digitalWrite(Motor4, 0);

  // Lights OFF
  digitalWrite(led_R, 0);
  digitalWrite(led_L, 0);

  // Set back to input mode
  pinMode(Motor1, INPUT);
  pinMode(Motor2, INPUT);
  pinMode(Motor3, INPUT);
  pinMode(Motor4, INPUT);

  // DC Motor Enable Pins
  pinMode(Enable1, INPUT);
  pinMode(Enable2, INPUT);

  // LED Light Pins
  pinMode(led_R, INPUT);
  pinMode(led_L, INPUT);

  std::cout << "Canyonero is out!" << std::endl;
}

// Move forward
void GpioControl::moveForward()
{
  direction_ = "FORWARD";
  digitalWrite(Motor1, 1);
  digitalWrite(Motor2, 0);
  digitalWrite(Motor3, 1);
  digitalWrite(Motor4, 0);
}

// Move Backward
void GpioControl::moveBackward()
{
  direction_ = "BACKWARD";
  digitalWrite(Motor1, 0);
  digitalWrite(Motor2, 1);
  digitalWrite(Motor3, 0);
  digitalWrite(Motor4, 1);
}

// Turn Right
void GpioControl::turnRightOnSpot()
{
  direction_ = "RIGTH";
  digitalWrite(Motor1, 1);
  digitalWrite(Motor2, 0);
  digitalWrite(Motor3, 0);
  digitalWrite(Motor4, 1);
}

// Turn Left
void GpioControl::turnLeftOnSpot()
{
  direction_ = "LEFT";
  digitalWrite(Motor1, 0);
  digitalWrite(Motor2, 1);
  digitalWrite(Motor3, 1);
  digitalWrite(Motor4, 0);
}

// Stop
void GpioControl::stopRobot()
{
  direction_ = "STOPPED";
  digitalWrite(Motor1, 0);
  digitalWrite(Motor2, 0);
  digitalWrite(Motor3, 0);
  digitalWrite(Motor4, 0);
}

// Add speed
void GpioControl::increaseSpeed()
{
  duty_cycle_ = duty_cycle_ + 10;
  if (duty_cycle_ > 100)
  {
    duty_cycle_ = 100;
  }
  softPwmWrite(Enable1, duty_cycle_);
  softPwmWrite(Enable2, duty_cycle_);
}

// Reduce speed
void GpioControl::decreaseSpeed()
{
  duty_cycle_ = duty_cycle_ - 10;
  if (duty_cycle_ < 10)
  {
    duty_cycle_ = 10;
  }
  softPwmWrite(Enable1, duty_cycle_);
  softPwmWrite(Enable2, duty_cycle_);
}

// Set exact speed (duty cycle)
void GpioControl::setSpeed(int value)
{
  duty_cycle_ = value;
  softPwmWrite(Enable1, duty_cycle_);
  softPwmWrite(Enable2, duty_cycle_);
}

// Rotate Camera up
void GpioControl::rotateUp()
{
  if (platform_state_ == "LOCKED")
  {
    return;
  }

  tilt_DTC_ = tilt_DTC_ - 1;
  if (tilt_DTC_ < 1)
  {
    tilt_DTC_ = 1;
  }

  softPwmWrite(TILT, tilt_DTC_);
}

// Rotate Camera Down
void GpioControl::rotateDown()
{
  if (platform_state_ == "LOCKED")
  {
    return;
  }

  tilt_DTC_ = tilt_DTC_ + 1;
  if (tilt_DTC_ > 199)
  {
    tilt_DTC_ = 199;
  }

  softPwmWrite(TILT, tilt_DTC_);
}

// Rotate Camera to the right
void GpioControl::rotateRight()
{
  if (platform_state_ == "LOCKED")
  {
    return;
  }

  pan_DTC_ = pan_DTC_ + 1;
  if (pan_DTC_ > 199)
  {
    pan_DTC_ = 199;
  }

  softPwmWrite(PAN, pan_DTC_);
}

// Rotate Camera to the left
void GpioControl::rotateLeft()
{
  if (platform_state_ == "LOCKED")
  {
    return;
  }

  pan_DTC_ = pan_DTC_ - 1;
  if (pan_DTC_ < 1)
  {
    pan_DTC_ = 1;
  }

  softPwmWrite(PAN, pan_DTC_);
}

// Convert from duty cycle to deg for the PAN servo range
double GpioControl::panToDutyCycle(double deg)
{
  float m = 23 / 180;
  double dtc = (double)m * deg + 3;
  return dtc;
}

// Convert from deg to duty cycle for the PAN servo range
double GpioControl::panToDeg(int dut)
{
  float m = 180 / 23;
  double deg = (double)(dut - 3) * m;
  return deg;
}

// Convert from deg to duty cycle for the TILT servo range
double GpioControl::tiltToDutyCycle(double deg)
{
  float m = 11 / 180;
  double dtc = (double)m * deg + 14;
  return dtc;
}

// Convert from deg to duty cycle for the TILT servo range
double GpioControl::tiltToDeg(int dut)
{
  float m = 180 / 11;
  double deg = (double)(dut - 14) * m;
  return deg;
}

// Set exact pan joint angle
void GpioControl::setPan(double value, int unit)
{
  if (platform_state_ == "LOCKED")
  {
    return;
  }

  // In ms
  if (unit == 0)
  {
    pan_DTC_ = (int)round(value);
  }
  // In deg
  else
  {
    double dtc = pan_toDutyCycle(value);
    pan_DTC_ = (int)round(dtc);
  }

  softPwmWrite(PAN, pan_DTC_);
}

// Set exact tilt joint angle
void GpioControl::setTilt(double value, int unit)
{
  if (platform_state_ == "LOCKED")
  {
    return;
  }

  // In ms
  if (unit == 0)
  {
    tilt_DTC_ = (int)round(value);
  }
  // In deg
  else
  {
    double dtc = tilt_toDutyCycle(value);
    tilt_DTC_ = (int)round(dtc);
  }

  softPwmWrite(TILT, tilt_DTC_);
}

// Create platform object
void GpioControl::openPlatform()
{
  is_locked_ = false;
  platform_state_ = "OPENED";

  // Set pan/tilt GPIOs
  pinMode(PAN, OUTPUT);
  pinMode(TILT, OUTPUT);

  softPwmCreate(PAN, 0, 200);
  softPwmCreate(TILT, 0, 200);

  setPan(pan_DTC_, 0);
  setTilt(tilt_DTC_, 0);
}

// Destroy platform object
void GpioControl::lockPlatform()
{
  pinMode(PAN, INPUT);
  pinMode(TILT, INPUT);

  is_locked_ = true;
  platform_state_ = "LOCKED";
}

// Switch lights ON
void GpioControl::lightsOn()
{
  digitalWrite(led_R, 1);
  digitalWrite(led_L, 1);
  are_on_ = true;
  led_state_ = "ON";
}

// Siwtch lights OFF
void GpioControl::lightsOff()
{
  digitalWrite(led_R, 0);
  digitalWrite(led_L, 0);
  are_on_ = false;
  led_state_ = "OFF";
}

// Get speed value
int GpioControl::getSpeed()
{
  int value = duty_cycle_;
  return value;
}

// Get PAN position in deg
double GpioControl::getPan()
{
  int d = pan_DTC_;
  double value = pan_toDeg(d);
  return value;
}

// Get PAN position in deg
double GpioControl::getTilt()
{
  int d = tilt_DTC_;
  double value = tilt_toDeg(d);
  return value;
}

std::string GpioControl::getDirection()
{
  return direction_;
}

std::string GpioControl::getLightsState()
{
  return led_state_;
}

std::string GpioControl::getPlatform()
{
  return platform_state_;
}

// Sleep Function
void GpioControl::sleep(unsigned int seconds)
{
  unsigned int ti = seconds * 1000000;
  usleep(ti);
}

// Info message with the instructions to run the keyboard teleoperation
// SHOULD NOT BE HERE!!!
/*void GpioControl::CommandLineInterface()
{
  move(0,0);
  printw("... Runing Keyboard Teleoperation ...");
  move(2,0);
  printw("MOTION:");
  move(4,0);
  printw("  >> Forward: w");
  move(5,0);
  printw("  >> Backward: s");
  move(6,0);
  printw("  >> Turn right (on spot): d");
  move(7,0);
  printw("  >> Turn left (on spot): a");
  move(8,0);
  printw("  >> Stop: b");
  move(9,0);
  printw("  >> Increase speed: x");
  move(10,0);
  printw("  >> Reduce speed: z");
  move(11,0);
  printw("  >> Switch OFF Canyonero: p");

  move(14,0);
  printw("VISION PLATFORM:");
  move(16,0);
  printw("  >> Unlock/Lock platform: t");
  move(17,0);
  printw("  >> Rotate up: i");
  move(18,0);
  printw("  >> Rotate Down: k");
  move(19,0);
  printw("  >> Rotate right: l");
  move(20,0);
  printw("  >> Rotate left:j");
  move(21,0);
  printw("  >> Lights: m");

  move(23,0);
  printw("ROBOT STATE:");
  move(25,0);
  printw("  * Direction = %s", dir.c_str());
  move(26,0);
  printw("  * Current speed = %d (%)", duty_cycle_);
  move(27,0);
  printw("  * Lights = %d", are_on_);
  move(28,0);
  printw("  * Camera (PAN, TILT) = (%.2f, %.2f) [deg]", pan_toDeg(pan_DTC_), tilt_toDeg(tilt_DTC_));
  move(29,0);
  printw("  * Platform = %s, %d", platform_state_.c_str(), is_locked_);
}*/

// Should not be here!!!
// Keyboard remote control
/*bool GpioControl::keyboardRemoteControl()
{
  int cht = 0;

  bool running = true;
  int sp = duty_cycle_;

  info_teleop();

  cht = getch();

  switch(cht)
  {
    case 'w':
      move_forward();
      dir = "FWRD";
      running = true;
      break;
    case 's':
      move_backward();
      dir = "BACK";
      running = true;
      break;
    case 'd':
      turn_right_onSpot();
      dir = "RGHT";
      running = true;
      break;
    case 'a':
      turn_left_onSpot();
      dir = "LEFT";
      running = true;
      break;
    case 'b':
      stop_robot();
      dir = "STOP";
      running = true;
      break;
    case 'x':
      increaseSpeed();
      running = true;
      break;
    case 'z':
      decreaseSpeed();
      running = true;
      break;
    case 'm':
      if(!are_on_){lights_on();}
      else{lights_off();}
      break;
    case 'i':
      if(!is_locked_){rotate_up();}
      else{platform_state_="LOCKED";}
      running = true;
      break;
    case 'k':
      if(!is_locked_){rotate_down();}
      else{platform_state_="LOCKED";}
      running = true;
      break;
    case 'l':
      if(!is_locked_){rotate_right();}
      else{platform_state_="LOCKED";}
      running = true;
      break;
    case 'j':
      if(!is_locked_){rotate_left();}
      else{platform_state_="LOCKED";}
      running = true;
      break;
    case 't':
      if(!is_locked_){lock_platform();}
      else{open_platform();}
      running = true;
      break;
    case 'p':
      stop_robot();
      move(15, 0);
      printw("Shutting down Canyonero");
      SleeP(1);
      //cout << endl << "Shutting down Canyonero" << endl;
      running = false;
      break;
  }

  wrefresh(win);

  return running;
}*/

// SHOULD NOT BE HERE!!!
// GPIO interface
/*void GpioControl::ros_gpio_interface(int D)
{
  switch(D)
  {
    case 8:
      move_forward();
      state = "FORWARD";
      break;
    case 2:
      move_backward();
      state = "BACKWARD";
      break;
    case 6:
      turn_right_onSpot();
      state = "RIGHT";
      break;
    case 4:
      turn_left_onSpot();
      state = "LEFT";
      break;
    case 0:
      stop_robot();
      state = "STOPPED";
      break;
    case 9:
      increaseSpeed();
      break;
    case 7:
      decreaseSpeed();
      break;
    case 11:
      if(!are_on_){lights_on();}
      else{lights_off();}
      break;
    case 21:
      if(!is_locked_){rotate_up();}
      else{platform_state_="LOCKED";}
      break;
    case 22:
      if(!is_locked_){rotate_down();}
      else{platform_state_="LOCKED";}
      break;
    case 23:
      if(!is_locked_){rotate_right();}
      else{platform_state_="LOCKED";}
      break;
    case 24:
      if(!is_locked_){rotate_left();}
      else{platform_state_="LOCKED";}
      break;
    case 32:
      if(!is_locked_){lock_platform();}
      else{open_platform();}
      break;
    case 5:
      stop_robot();
      state = "STOPPED";
      break;
  }
}*/

/*// Initial config for ncurses
void GpioControl::startNcurses()
{
  // Start ncurses
  initscr();

  // No echo (not printing keys pressed)
  noecho();

  // One key at a time
  cbreak();

  // Create window
  win = newwin(30, 50, 0, 0);
}


// Final config for ncurses
void GpioControl::endNcurses()
{
    endwin();
}*/
}  // namespace canyonero