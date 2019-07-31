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

class gpioControl
{

    public:
    
        // Constructor
        gpioControl();
        
        // Destructor
        ~gpioControl();
        
        // Public Methods
        
        // Robot Control
        void move_forward();
        void move_backward();
        void turn_right_onSpot();    
        void turn_left_onSpot();
        void stop_robot();
        void increaseSpeed();
        void decreaseSpeed();
        void setSpeed(int value);
        
        // Camera platform control
        void rotate_up();
        void rotate_down();
        void rotate_right();
        void rotate_left();
        double pan_toDutyCycle(double deg);
        double pan_toDeg(int dut);
        double tilt_toDutyCycle(double deg);
        double tilt_toDeg(int dut);
        void setPan(double value, int unit);
        void setTilt(double value, int unit);
        
        // Illumination
        void lights_on();
        void lights_off();
        
        // General Purpose
        int getSpeed();        
        void SleeP(unsigned int seconds);
        void info_teleop();
        bool keyboard_remote_control();
        void ros_gpio_interface(int D);
        void start_ncurses();
        void end_ncurses();
        
        // Public variables
        string state ="STOP";
        string led_state = "OFF";
        
        
    private:
        
        // Private Variables
        int dutyCycleValue = 50;
        int pan_DTC = 7;
        int tilt_DTC = 11;
        int Direction = 0;
        string dir = "STOP";
        WINDOW * win;
        bool are_on = false;
};

#endif //GPIOCONTROL_H
