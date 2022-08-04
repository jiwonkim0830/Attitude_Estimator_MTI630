//  Copyright (c) 2003-2021 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

//--------------------------------------------------------------------------------
// Xsens device API C++ example MTi receive data.
//--------------------------------------------------------------------------------
#include "CallbackHandler.hpp"
#include "Ros_publisher.hpp"
//--------------------------------------------------------------------------------
int main(int argc, char** argv)
{
// ################################################ Scanning & Setting ################################################
	cout << "Creating XsControl object..." << endl;
	XsControl* control = XsControl::construct();
	assert(control != nullptr);

	XsVersion version;
	xdaVersion(&version);
	cout << "Using XDA version: " << version.toString().toStdString() << endl;


	//Lambda function for error handling
	auto handleError = [=](string errorString)
	{
		control->destruct();
		cout << errorString << endl;
		cout << "Press [ENTER] to continue." << endl;
		cin.get();
		return -1;
	};

	cout << "Scanning for devices..." << endl;
	XsPortInfoArray portInfoArray = XsScanner::scanPorts();

	// Find an MTi device
	XsPortInfo mtPort;
	for (auto const &portInfo : portInfoArray)
	{
		if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
		{
			mtPort = portInfo;
			break;
		}
	}

	if (mtPort.empty())
		return handleError("No MTi device found. Aborting.");

	cout << "Found a device with ID: " << mtPort.deviceId().toString().toStdString() << " @ port: " << mtPort.portName().toStdString() << ", baudrate: " << mtPort.baudrate() << endl;

	cout << "Opening port..." << endl;
	if (!control->openPort(mtPort.portName().toStdString(), mtPort.baudrate()))
		return handleError("Could not open port. Aborting.");

	// Get the device object
	XsDevice* device = control->device(mtPort.deviceId());
	assert(device != nullptr);

	cout << "Device: " << device->productCode().toStdString() << ", with ID: " << device->deviceId().toString() << " opened." << endl;

	// Create and attach callback handler to device
	CallbackHandler callback;
	device->addCallbackHandler(&callback);


//#################################################### Configuration ###################################################
	// Put the device into configuration mode before configuring the device
	cout << "Putting device into configuration mode..." << endl;
	if (!device->gotoConfig())
		return handleError("Could not put device into configuration mode. Aborting.");

	cout << "Configuring the device..." << endl;
	XsOutputConfigurationArray configArray;
	configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
	configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));

	if (device->deviceId().isVru() || device->deviceId().isAhrs())
	{
		//configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 100));
        configArray.push_back(XsOutputConfiguration(XDI_AccelerationHR, 0xFFFF));
		configArray.push_back(XsOutputConfiguration(XDI_RateOfTurnHR, 0xFFFF));
		configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 100));
	}

	else
	{
		return handleError("Unknown device while configuring. Aborting.");
	}

	if (!device->setOutputConfiguration(configArray))
		return handleError("Could not configure MTi device. Aborting.");

	cout << "Creating a log file..." << endl;
	string logFileName = "logfile.mtb";
	if (device->createLogFile(logFileName) != XRV_OK)
		return handleError("Failed to create a log file. Aborting.");
	else
		cout << "Created a log file: " << logFileName.c_str() << endl;

// ##################################################### For ROS Publish ###############################################
	ros::init(argc, argv, "visualize_node");
	Ros_Publisher ros_publisher;
// ##################################################### Measurement #################################################
	cout << "Putting device into measurement mode..." << endl;
	if (!device->gotoMeasurement())
		return handleError("Could not put device into measurement mode. Aborting.");

	XsVector accHR;
	XsVector gyrHR;
	XsVector mag;

	cout << "\nMain loop. Recording data for 10 seconds." << endl;
	cout << string(79, '-') << endl;

	//int64_t startTime = XsTime::timeStampNow();
	vector<double> acc_vec;

	//while (XsTime::timeStampNow() - startTime <= 10000)
	while (1)
	{
		if (callback.packetAvailable())
		{
			cout << setw(5) << fixed << setprecision(2);

			// Retrieve a packet
			XsDataPacket packet = callback.getNextPacket();
            
            // if (packet.containsSampleTimeFine())
            // {
            //     cout << endl;
            //     uint32_t sampletimefine = packet.sampleTimeFine();
            //     cout << "\r";
            //     printf("%d", sampletimefine);
            // }
            
            if (packet.containsAccelerationHR())
			{
				cout << endl; 
				accHR = packet.accelerationHR();
				acc_vec = accHR.toVector();
				cout << "\r"
					<< "AccHR X:" << accHR[0]
					<< ", AccHR Y:" << accHR[1]
					<< ", AccHR Z:" << accHR[2];
			}

			if (packet.containsRateOfTurnHR())
			{

				cout << endl; 
				gyrHR = packet.rateOfTurnHR();
				//gyro_vec = gyrHR.toVector();
				cout << "GyrHR X:" << gyrHR[0]
					<< ", GyrHR Y:" << gyrHR[1]
					<< ", GyrHR Z:" << gyrHR[2];
			}

			if (packet.containsCalibratedMagneticField())
			{
				cout << endl; 
				mag = packet.calibratedMagneticField();
				//mag_vec = mag.toVector();
				cout << "Mag X:" << mag[0]
					<< ", Mag Y:" << mag[1]
					<< ", Mag Z:" << mag[2];
			}

			cout << flush;
		}
		XsTime::msleep(0);
		ros_publisher.RosPublic(accHR, gyrHR, mag);//, gyrHR, mag);
	}
	
	cout << "\n" << string(79, '-') << "\n";
	cout << endl;

	cout << "Stopping recording..." << endl;
	if (!device->stopRecording())
		return handleError("Failed to stop recording. Aborting.");

	cout << "Closing log file..." << endl;
	if (!device->closeLogFile())
		return handleError("Failed to close log file. Aborting.");

	cout << "Closing port..." << endl;
	control->closePort(mtPort.portName().toStdString());

	cout << "Freeing XsControl object..." << endl;
	control->destruct();

	cout << "Successful exit." << endl;

	cout << "Press [ENTER] to continue." << endl;
	cin.get();

	return 0;
}
