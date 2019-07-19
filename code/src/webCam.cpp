/*  CANYONERO  */

// Onboard WebCam streaming

#include "camera_driver.h"

int main(int argc, char** argv)
{
	cout << "CANYONERO" << endl << endl;
	cout << "... Onboard Video Streaming ..." << endl;
	
	camera_driver * wcam = new camera_driver(0, 0);
	
	// Check if the camera is working
	if(!wcam->cap.open(0))
	{
		cout << "Error opening Canyonero's webcam" << endl;
		return 0;
	}
	
	wcam->get_resolution();
	
	// Streaming Loop
	//while(wcam->streamVideo()){}
	wcam->streaming();
	//wcam->online_streaming();
	
	return 0;
}
