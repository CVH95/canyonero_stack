/*  CANYONERO  */


// Keyboard Teleoperation

#include "gpioControl.h"

int main()
{
	cout << "CANYONERO" << endl << endl;
	
	gpioControl * Canyonero = new gpioControl();
	
	Canyonero->setSpeed(50);
	//Canyonero->setPan(15, 0);
	//Canyonero->setTilt(15, 0);
	
	cout << "Loading drivers and interface..." << endl;
	Canyonero->SleeP(3);
	
	Canyonero->start_ncurses();
	
	while(Canyonero->keyboard_remote_control()){}
	
	Canyonero->end_ncurses();
	
	cout << endl << "Canyonero out" << endl;
	
	return 0;
}
