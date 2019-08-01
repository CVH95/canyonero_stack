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
	are_on = false;
	
	// Create PWMs
	softPwmCreate(Enable1, 0 , 100);
	softPwmCreate(Enable2, 0 , 100);
	
	// Initiate (all pins LOW)
	digitalWrite(Motor1, 0);
	digitalWrite(Motor2, 0);
	digitalWrite(Motor3, 0);
	digitalWrite(Motor4, 0);
	digitalWrite(led_R, 0);
	digitalWrite(led_L, 0);
}


// Destructor (clean up)
gpioControl::~gpioControl()
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
	
	
	cout << "Canyonero is out" << endl;
}


//-----------------------------------------------------------------------------------------------


// PUBLIC METHODS


// DC MOTOR Driving Methods


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


//-----------------------------------------------------------------------------------------------


// PAN/TILT CAMERA PLATFORM


// Rotate Camera up
void gpioControl::rotate_up()
{
	tilt_DTC = tilt_DTC - 1;
	if(tilt_DTC < 1)
	{
		tilt_DTC = 1;	
	}
	
	softPwmWrite(TILT, tilt_DTC);
}


// Rotate Camera Down
void gpioControl::rotate_down()
{
	tilt_DTC = tilt_DTC + 1;
	if(tilt_DTC > 199)
	{
		tilt_DTC = 199;	
	}
	
	softPwmWrite(TILT, tilt_DTC);
}


// Rotate Camera to the right
void gpioControl::rotate_right()
{
	pan_DTC = pan_DTC + 1;
	if(pan_DTC > 199)
	{
		pan_DTC = 199;	
	}
	
	softPwmWrite(PAN, pan_DTC);
}


// Rotate Camera to the left
void gpioControl::rotate_left()
{
	pan_DTC = pan_DTC - 1;
	if(pan_DTC < 1)
	{
		pan_DTC = 1;	
	}
	
	softPwmWrite(PAN, pan_DTC);
}


// Convert from duty cycle to deg for the PAN servo range
double gpioControl::pan_toDutyCycle(double deg)
{
	float m = 23/180;
	double dtc = (double) m*deg + 3;
	return dtc;
}


// Convert from deg to duty cycle for the PAN servo range
double gpioControl::pan_toDeg(int dut)
{
	float m = 180/23;
	double deg = (double) (dut - 3) * m;
	return deg;
}


// Convert from deg to duty cycle for the TILT servo range
double gpioControl::tilt_toDutyCycle(double deg)
{
	float m = 11/180;
	double dtc = (double) m*deg + 14;
	return dtc;
}


// Convert from deg to duty cycle for the TILT servo range
double gpioControl::tilt_toDeg(int dut)
{
	float m = 180/11;
	double deg = (double) (dut - 14) * m;
	return deg;
}


// Set exact pan joint angle
void gpioControl::setPan(double value, int unit)
{
	// In ms
	if(unit == 0)
	{
		pan_DTC = (int) round(value);
	}
	// In deg
	else
	{
		double dtc = pan_toDutyCycle(value);
		pan_DTC = (int) round(dtc); 
	}
	
	softPwmWrite(PAN, pan_DTC);
}


// Set exact tilt joint angle
void gpioControl::setTilt(double value, int unit)
{
	// In ms
	if(unit == 0)
	{
		tilt_DTC = (int) round(value);
	}
	// In deg
	else
	{
		double dtc = tilt_toDutyCycle(value);
		tilt_DTC = (int) round(dtc); 
	}
	
	softPwmWrite(TILT, tilt_DTC);
}


// Create platform object
void gpioControl::open_platform()
{
	is_locked = false;
	platform_state = "OPENED";
	
	// Set pan/tilt GPIOs
	pinMode(PAN, OUTPUT);
	pinMode(TILT, OUTPUT);
	
	softPwmCreate(PAN, 0 , 200);
	softPwmCreate(TILT, 0 , 200);
	
	setPan(15, 0);
	setTilt(15, 0);
}


// Destroy platform object
void gpioControl::lock_platform()
{
	pinMode(PAN, INPUT);
	pinMode(TILT, INPUT);
	
	is_locked = true;
	platform_state = "LOCKED";
}


//-----------------------------------------------------------------------------------------------


// LIGHTS


// Switch lights ON
void gpioControl::lights_on()
{
	digitalWrite(led_R, 1);
	digitalWrite(led_L, 1);
	are_on = true;
	led_state = "ON";
}


// Siwtch lights OFF
void gpioControl::lights_off()
{
	digitalWrite(led_R, 0);
	digitalWrite(led_L, 0);
	are_on = false;
	led_state = "OFF";
}

//-----------------------------------------------------------------------------------------------


// General Purpose Methods


// Get speed value
int gpioControl::getSpeed()
{
	int value = dutyCycleValue;
	return value;
}


// Get PAN position in deg
double gpioControl::get_pan()
{
	int d = pan_DTC;
	double value = pan_toDeg(d);
	return value;
}


// Get PAN position in deg
double gpioControl::get_tilt()
{
	int d = tilt_DTC;
	double value = tilt_toDeg(d);
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
	
	move(13,0);
	printw("VISION PLATFORM:");
	move(15,0);
	printw("  >> Enable platform: t");
	move(16,0);
	printw("  >> Disable platform: y");
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
	printw("  * Current speed = %d (%)", dutyCycleValue);
	move(27,0);
	printw("  * Lights = %d", are_on);
	move(28,0);
	printw("  * Camera (PAN, TILT) = (%.2f, %.2f) [deg]", pan_toDeg(pan_DTC), tilt_toDeg(tilt_DTC));
	move(29,0);
	printw("  * Platform = %s, %d", platform_state.c_str(), is_locked);
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
		case 'x':
			increaseSpeed();
			running = true;
			break;
		case 'z':
			decreaseSpeed();
			running = true;
			break;
		case 'm':
			if(!are_on){lights_on();}
			else{lights_off();}
			break;
		case 'i':
			if(!is_locked){rotate_up();}
			else{platform_state="LOCKED";}
			running = true;
			break;
		case 'k':
			if(!is_locked){rotate_down();}
			else{platform_state="LOCKED";}
			running = true;
			break;
		case 'l':
			if(!is_locked){rotate_right();}
			else{platform_state="LOCKED";}
			running = true;
			break;
		case 'j':
			if(!is_locked){rotate_left();}
			else{platform_state="LOCKED";}
			running = true;
			break;
		case 't':
			open_platform();
			running = true;
			break;
		case 'y':
			lock_platform();
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
			if(!are_on){lights_on();}
			else{lights_off();}
			break;
		case 21:
			if(!is_locked){rotate_up();}
			else{platform_state="LOCKED";}
			break;
		case 22:
			if(!is_locked){rotate_down();}
			else{platform_state="LOCKED";}
			break;
		case 23:
			if(!is_locked){rotate_right();}
			else{platform_state="LOCKED";}
			break;
		case 24:
			if(!is_locked){rotate_left();}
			else{platform_state="LOCKED";}
			break;
		case 31:
			open_platform();
			break;
		case 32:
			lock_platform();
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
	win = newwin(30, 50, 0, 0);
} 


// Final config for ncurses
void gpioControl::end_ncurses()
{
		endwin();
}
