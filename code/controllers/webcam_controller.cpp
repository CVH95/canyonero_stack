/*  CANYONERO  */

// Onboard WebCam streaming

#include "webcamControl.h"

int main(int argc, char** argv)
{
	cout << "CANYONERO" << endl << endl;
	cout << "... Onboard Video Streaming ..." << endl;
	
	webcamControl * wcam = new webcamControl(0, 0);
	
	// Check if the camera is working
	if(!wcam->cap.open(0))
	{
		cout << "Error opening Canyonero's webcam" << endl;
		return 0;
	}
	
	wcam->getResolution();
	
	// Streaming Loop
	//while(wcam->streamVideo()){}
	wcam->streaming();
	
	return 0;
}
