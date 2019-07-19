/*  CANYONERO  */


// Camera Interface

// Video Stream
// Configure video broadcast server

#include "camera_driver.h"


// Constructor
camera_driver::camera_driver(int length, int height)
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
camera_driver::~camera_driver()
{
	cout << "Canyonero's webcam is out" << endl;
}


// Video Streaming 
void camera_driver::streaming()
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


// Saves frames in order to be read by webServer
void camera_driver::online_streaming()
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
		
		//imshow("Canyonero View", frame);
		imwrite("/home/pi/Workspace/Canyonero/code/webServer/FlaskApp/templates/frame.png", frame);
		
		waitKey(30);
	}	
}


// Obtain frame for ROS control
Mat camera_driver::get_frame()
{
	Mat frame;
	cap >> frame;
	
	// Resize frames
	resize(frame, frame, Size(size_length, size_height));
	
	if(frame.empty())
	{
		cout << "Signal went missing" << endl;
	}
	
	return frame;
}


// Print Resolution
void camera_driver::get_resolution()
{
	cout << "Stream resolution: " << size_length << "x" << size_height << "." << endl; 
}
