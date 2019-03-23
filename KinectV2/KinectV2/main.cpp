#include <iostream>
#include <sstream>
#include <opencv2/opencv.hpp>

#include "NtKinect.h"

using namespace std;

/*
void doJob() {
	cv::VideoCapture cap(0);
	cv::Mat image;
	while (1) {
		cap >> image;
		cv::imshow("video", image);
		auto key = cv::waitKey(1);
		if (key == 'q') break;
	}
	cv::destroyAllWindows();
}*/


void doJob() {
	NtKinect kinect;
	while (1) {
		


		kinect.setRGB();
		kinect.setDepth();
		kinect.setInfrared();
	
		//*Here starts James's attempts to create a depth frame 
		cv::Mat frame(kinect.infraredImage);
		//frame.convertTo(frame, CV_8U, 255.0 / 65536.0);//255/65536 is the scaling factor for converting from 16bit to 8bit
		cv::imwrite("InfraredV2.tif", frame);
		//*///Here ends James's attempts to create a depth frame

		cv::imshow("rgb", kinect.rgbImage);
		cv::imshow("depth", kinect.depthImage);
		cv::imshow("infrared", kinect.infraredImage);

	
		/*
		for (int y = 0; y < kinect.depthImage.rows; y++) {
			for (int x = 0; x < kinect.depthImage.cols; x++) {
				UINT16 d = kinect.depthImage.at<UINT16>(y, x);
				DepthSpacePoint dp; dp.X = x; dp.Y = y;
				ColorSpacePoint cp;
				HRESULT hr = kinect.coordinateMapper->MapDepthPointToColorSpace(dp, d, &cp);
				
				


				if (hr != S_OK) continue;
				if (d > 3000 || d < 500) {
					int ax = (int)cp.X;
					int ay = (int)cp.Y;
					cv::rectangle(kinect.rgbImage, cv::Rect(ax - 2, ay - 2, 4, 4), cv::Scalar(255, 0, 0), 2);
				}
			}
		}


		cv::imshow("rbg near", kinect.rgbImage);
		
		*/
		auto key = cv::waitKey(1);
		if (key == 'q') break;
	}
	cv::destroyAllWindows();
}

int main(int argc, char** argv) {
	try {
		doJob();
	}
	catch (exception &ex) {
		cout << ex.what() << endl;
		string s;
		cin >> s;
	}
	return 0;
}