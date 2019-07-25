/*  CANYONERO  */


// GPIO Interface 

// Access GPIOs
// Configure PWMs

#include "gpioControl.h"


// Constructor
gpioControl::gpioControl()
{
	if(wiringPiSetupGpio() == -1)
	{
		cout << "WiringPi failed to set up GPIOs" << endl;
		//return 1;
	}
	
	// Motor Pins
	pinMode(Motor1, OUTPUT);
	pinMode(Motor2, OUTPUT);
	pinMode(Motor3, OUTPUT);
	pinMode(Motor4, OUTPUT);
	
	// Enable Pins
	pinMode(Enable1, OUTPUT);
	pinMode(Enable2, OUTPUT);
	
	// Create PWMs
	softPwmCreate(Enable1, 0 , 100);
	softPwmCreate(Enable2, 0 , 100);
	
	// Initiate (all motors LOW)
	digitalWrite(Motor1, 0);
	digitalWrite(Motor2, 0);
	digitalWrite(Motor3, 0);
	digitalWrite(Motor4, 0);
}


// Destructor
gpioControl::~gpioControl()
{
	digitalWrite(Motor1, 0);
	digitalWrite(Motor2, 0);
	digitalWrite(Motor3, 0);
	digitalWrite(Motor4, 0);
	cout << "Canyonero is out" << endl;
}


// PUBLIC METHODS


// Move forward
void gpioControl::move_forward()
{
	digitalWrite(Motor1, 1);
	digitalWrite(Motor2, 0);
	digitalWrite(Motor3, 1);
	digitalWrite(Motor4, 0);
}


// Move Backward
void gpioControl::move_backward()
{
	digitalWrite(Motor1, 0);
	digitalWrite(Motor2, 1);
	digitalWrite(Motor3, 0);
	digitalWrite(Motor4, 1);
}


// Turn Right
void gpioControl::turn_right_onSpot()
{
	digitalWrite(Motor1, 1);
	digitalWrite(Motor2, 0);
	digitalWrite(Motor3, 0);
	digitalWrite(Motor4, 1);
}


// Turn Left
void gpioControl::turn_left_onSpot()
{
	digitalWrite(Motor1, 0);
	digitalWrite(Motor2, 1);
	digitalWrite(Motor3, 1);
	digitalWrite(Motor4, 0);
}


// Stop
void gpioControl::stop_robot()
{
	digitalWrite(Motor1, 0);
	digitalWrite(Motor2, 0);
	digitalWrite(Motor3, 0);
	digitalWrite(Motor4, 0);
}


// Add speed
void gpioControl::increaseSpeed()
{
	dutyCycleValue = dutyCycleValue + 10;
	if(dutyCycleValue > 100)
	{
		dutyCycleValue = 100;	
	}
	softPwmWrite(Enable1, dutyCycleValue);
	softPwmWrite(Enable2, dutyCycleValue);
}


// Reduce speed
void gpioControl::decreaseSpeed()
{
	dutyCycleValue = dutyCycleValue - 10;
	if(dutyCycleValue < 10)
	{
		dutyCycleValue = 10;	
	}
	softPwmWrite(Enable1, dutyCycleValue);
	softPwmWrite(Enable2, dutyCycleValue);
}


// Set exact speed (duty cycle)
void gpioControl::setSpeed(int value)
{
	dutyCycleValue = value;
	softPwmWrite(Enable1, dutyCycleValue);
	softPwmWrite(Enable2, dutyCycleValue);
}


// Get speed value
int gpioControl::getSpeed()
{
	int value = dutyCycleValue;
	return value;
}


// Sleep Function
void gpioControl::SleeP(unsigned int seconds)
{
	unsigned int ti = seconds * 1000000;
	usleep(ti);
}


// Info message with the instructions to run the keyboard teleoperation
void gpioControl::info_teleop()
{
	move(0,0);
	printw("... Runing Keyboard Teleoperation ...");
	move(2,0);
	printw("  >> Forward: w");
	move(3,0);
	printw("  >> Backward: s");
	move(4,0);
	printw("  >> Turn right (on spot): d");
	move(5,0);
	printw("  >> Turn left (on spot): a");
	move(6,0);
	printw("  >> Stop: b");
	move(7,0);
	printw("  >> Increase speed: k");
	move(8,0);
	printw("  >> Reduce speed: j");
	move(9,0);
	printw("  >> Exit program: p");
	move(13,0);
	printw("Direction = %s", dir.c_str());
	move(14,0);
	printw("Current speed = %d (%)", dutyCycleValue);
}


// Keyboard remote control
bool gpioControl::keyboard_remote_control()
{		
	int cht = 0;
	
	bool running = true;
	int sp = dutyCycleValue;
	
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
		case 'k':
			increaseSpeed();
			running = true;
			break;
		case 'j':
			decreaseSpeed();
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
}


// GPIO interface
void gpioControl::ros_gpio_interface(int D)
{
	switch(D)
	{
		case 8:
			move_forward();
			state = "MOVING FORWARD";
			break;
		case 2:
			move_backward();
			state = "MOVING BACKWARD";
			break;
		case 6:
			turn_right_onSpot();
			state = "TURNING RIGHT";
			break;
		case 4:
			turn_left_onSpot();
			state = "TURNING LEFT";
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
		case 5:
			stop_robot();
			state = "STOPPED";
			break;
	}
}


// Initial config for ncurses
void gpioControl::start_ncurses()
{
	// Start ncurses
	initscr();
		
	// No echo (not printing keys pressed)
	noecho();
		
	// One key at a time
	cbreak();
	
	// Create window
	win = newwin(16, 50, 0, 0);
} 


// Final config for ncurses
void gpioControl::end_ncurses()
{
		endwin();
}
