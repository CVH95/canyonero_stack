/*  CANYONERO  */


// Camera Interface

// Video Stream
// Configure video broadcast server

#ifndef WEBCAMCONTROL_H
#define WEBCAMCONTROL_H

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class camera_driver
{
	public:
	
		// Constructor
		camera_driver(int length, int height);
		
		// Destructor
		~camera_driver();
		
		// Public methods
		void streaming();
		void online_streaming();
		void get_resolution();
		Mat get_frame();
		
		// Public Variables
		VideoCapture cap;
				
	private:
	
		// Private Variables
		int size_length = 600;
		int size_height = 480;	
};

#endif //WEBCAMCONTROL_H
