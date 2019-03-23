#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <OpenNI.h>
#include <NiTE.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

using namespace std;
using namespace cv;
using namespace openni;

//Prototypes
bool HandleStatus(Status);
char ReadLastCharOfLine();


int main(int argv, char* argc)
{
	//Print OpenNI version number
	printf("OpenNi Version is %d.%d.%d.%d", OpenNI::getVersion().major, OpenNI::getVersion().minor, OpenNI::getVersion().maintenance, OpenNI::getVersion().build);

	//Initializes OpenNI status to scan for devices and check for errors
	printf("\nScanning machine for devices and loading modules/drivers...\r\n");
	Status status = STATUS_OK;
	status = OpenNI::initialize();

	//Outputs error if there is one and terminates program Calls user-defined HandleStatus function
	if (!HandleStatus(status)) return 1;
	
	printf("\nCompleted. \r\n");
	
	//Creates an array of DeviceInfo objects called listOfDevices
	Array<DeviceInfo> listOfDevices;
	OpenNI::enumerateDevices(&listOfDevices);	//enumerates the devices from the array of devices

	int numberOfDevices = listOfDevices.getSize();

	//Executes if devices are found, iterates through the listed devices and shows information
	if (numberOfDevices > 0) {
		printf("%d Device(s) are available to use. \r\n\r\n", numberOfDevices);

		for (int i = 0; i < numberOfDevices; i++) {
			DeviceInfo device = listOfDevices[i];
			printf("%d. %s->%s (VID: %d | PID: %d) is connected at %s\r\n", i, device.getVendor(), device.getName(), device.getUsbVendorId(), device.getUsbProductId(), device.getUri());
		}
	}
	else {
		printf("No device connected to this machine.");
	}
	
	Device v1;
	DeviceInfo v1Info;

	VideoStream *depth_stream_ = new openni::VideoStream();

	v1.open(ANY_DEVICE);
	VideoStream vid1;
	vid1.create(v1, SENSOR_DEPTH);



	VideoCapture capture(CAP_OPENNI_DEPTH_MAP);

	Mat depthMap;
	capture.grab();

	capture.retrieve(depthMap, CAP_OPENNI);

	//cout << depthMap;

	

	

	//Stops program from terminating right away
	printf("Press ENTER to exit. \r\n");
	ReadLastCharOfLine();
	OpenNI::shutdown();

	return 0;
}

//Function to check for errors
bool HandleStatus(Status status) {
	if (status == STATUS_OK)
		return true;
	printf("ERROR: #%d, %s", status, OpenNI::getExtendedError());
	ReadLastCharOfLine();
	return false;
}

//Function to wait for userinput before terminating program
char ReadLastCharOfLine()
{
	int newChar = 0;
	int lastChar;
	fflush(stdout);
	do
	{
		lastChar = newChar;
		newChar = getchar();
	} while ((newChar != '\n') && (newChar != EOF));
	return (char)lastChar;

}