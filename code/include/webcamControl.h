/*  CANYONERO  */


// Camera Interface

// Video Stream
// Configure video broadcast server

#ifndef WEBCAMCONTROL_H
#define WEBCAMCONTROL_H

#include<iostream>
#include<string>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class webcamControl
{
		public:
		
			// Constructor
			webcamControl(int length, int height);
			
			// Destructor
			~webcamControl();
			
			// Public methods
			bool streamVideo();
			void streaming();
			void getResolution();
			
			// Public Variables
			VideoCapture cap;
					
		private:
		
			// Private Variables
			int size_length = 600;
			int size_height = 480;
			
			
		
};

#endif //WEBCAMCONTROL_H
