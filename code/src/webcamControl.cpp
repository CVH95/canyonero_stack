/*  CANYONERO  */


// Camera Interface

// Video Stream
// Configure video broadcast server

#include "webcamControl.h"


// Constructor
webcamControl::webcamControl(int length, int height)
{
	if(length != 0 && height != 0)
	{
		size_length = length;
		size_height = height;
	}
	else
	{
		size_length = 600;
		size_height = 480;
	}
}


// Destructor
webcamControl::~webcamControl()
{
	cout << "Canyonero's webcam is out" << endl;
}


//  Video streaming
bool webcamControl::streamVideo()
{
	bool isStreaming = true;
	
	// Create  Mat container for video frames
	Mat frame;
	cap >> frame;
	
	// Resize frames
	resize(frame, frame, Size(size_length, size_height));
	
	if(frame.empty())
	{
		return false;
	}
	
	imshow("Canyonero View", frame);
	
	waitKey(10000);
	
	return isStreaming;
}


// Video Streaming 
void webcamControl::streaming()
{	
	// Create  Mat container for video frames
	for(;;)
	{
		Mat frame;
		cap >> frame;
		
		// Resize frames
		resize(frame, frame, Size(size_length, size_height));
		
		if(frame.empty())
		{
			cout << "Signal went missing" << endl;
		}
		
		imshow("Canyonero View", frame);
		
		waitKey(30);
	}	
}


// Print Resolution
void webcamControl::getResolution()
{
	cout << "Stream resolution: " << size_length << "x" << size_height << "." << endl; 
}
