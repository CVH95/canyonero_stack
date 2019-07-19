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

using namespace std;

#define Motor1 17
#define Motor2 22
#define Enable1 16

#define Motor3 23
#define Motor4 24
#define Enable2 20

class gpioControl
{

    public:
    
        // Constructor
        gpioControl();
        
        // Destructor
        ~gpioControl();
        
        // Public Methods
        void move_forward();
        void move_backward();
        void turn_right_onSpot();    
        void turn_left_onSpot();
        void stop_robot();
        
        void increaseSpeed();
        void decreaseSpeed();
        void setSpeed(int value);
        int getSpeed();
        
        void SleeP(unsigned int seconds);
        
        void info_teleop();
        bool keyboard_remote_control();
        void ros_gpio_interface(int D);
        void start_ncurses();
        void end_ncurses();
        
        // Public variables
        string state ="STOP";
        
        
    private:
        
        // Private Variables
        int dutyCycleValue = 50;
        int Direction = 0;
        string dir = "STOP";
        WINDOW * win;
};

#endif //GPIOCONTROL_H
