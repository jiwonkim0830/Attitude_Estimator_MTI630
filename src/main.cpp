#include "CallbackHandler.hpp"
#include "RosRawDataPublisher.hpp"
#include <Eigen/Dense>
#include "Mahonyfilter.hpp"
#include "EKF.hpp"
#include <chrono>
using namespace std;
using namespace Eigen;


int main(int argc, char** argv)
{
// ################################################ Scanning & Setting ################################################
	cout<<fixed; cout.precision(6);
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
		cout << portInfo << endl;
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


//######################################################## Configuration #########################################################
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
		configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 0xFFFF));
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


	ros::init(argc, argv, "visualize_node");
	RosRawDataPublisher ros_raw_data_publisher;

	cout << "Putting device into measurement mode..." << endl;
	if (!device->gotoMeasurement())
		return handleError("Could not put device into measurement mode. Aborting.");

	cout << "\nMain loop." << endl;
	cout << string(79, '-') << endl;

	XsVector accHR, gyrHR, mag;
	Vector3d AccMeasured = Vector3d::Zero();
	Vector3d GyrMeasured = Vector3d::Zero();
	Vector3d MagMeasured = Vector3d::Zero();
//##################################################### for filter initialize ###################################################
	bool isAccInitialized = false;
	bool isMagInitialized = false;
	bool isGyrInitialized = false;
	Vector3d firstAcc;
	Vector3d firstMag;
	Vector3d firstGyr;

	while (!isAccInitialized || !isMagInitialized || !isGyrInitialized)
	{
		if (callback.packetAvailable())
		{
			XsDataPacket packet = callback.getNextPacket();

			if (packet.containsAccelerationHR())
			{
				accHR = packet.accelerationHR();
				cout << "\r"
					<< "AccHR X:" << accHR[0]
					<< ", AccHR Y:" << accHR[1]
					<< ", AccHR Z:" << accHR[2] << endl;
				firstAcc << accHR[0], accHR[1], accHR[2];
				AccMeasured << accHR[0], accHR[1], accHR[2];
				isAccInitialized = true;
			}

			if (packet.containsCalibratedMagneticField())
			{
				mag = packet.calibratedMagneticField();
				cout << "Mag X:" << mag[0]
					<< ", Mag Y:" << mag[1]
					<< ", Mag Z:" << mag[2] << endl;
				firstMag << mag[0], mag[1], mag[2];
				MagMeasured << mag[0], mag[1], mag[2];
				isMagInitialized = true;
			}

			if (packet.containsRateOfTurnHR())
			{
				gyrHR = packet.rateOfTurnHR();
				cout << "GyrHR X:" << gyrHR[0]
					<< ", GyrHR Y:" << gyrHR[1]
					<< ", GyrHR Z:" << gyrHR[2] << endl;
				GyrMeasured << gyrHR[0], gyrHR[1], gyrHR[2];
				isGyrInitialized = true;
			}
		}
	}
	cout << "Filter Initialized !" << endl;
	cout << "FIrst acc : " << firstAcc[0] << " " << firstAcc[1] << " " << firstAcc[2] << endl;
	cout << "FIrst mag : " << firstMag[0] << " " << firstMag[1] << " " << firstMag[2] << endl;
	cout << "FIrst gyr : " << GyrMeasured[0] << " " << GyrMeasured[1] << " " << GyrMeasured[2] << endl;
	
//############################################## Choose the filter ############################################################
	//MahonyFilter mahony(1, 0.3, 1, 1, 1e-3, firstAcc, firstMag, 0.5, 0.1, 5); //gain tuning is needed !!
	EKF ekf(1e-3, 1e-2, 5e-2, 1e-2, 0.0, 0.0, 0.2, 0.1, 0.17, firstAcc, firstMag); //gain tuning is needed !!

//############################################## Loop time check ###############################################################
	using Framerate = chrono::duration<chrono::steady_clock::rep, std::ratio<1, 1000>>;
	chrono::steady_clock::time_point start = chrono::steady_clock::now();
	chrono::steady_clock::time_point next = chrono::steady_clock::now(); + Framerate{1};
	chrono::duration<double, nano> duration;

//#################################################### Measurement  ############################################################
	// int64_t startTime = XsTime::timeStampNow();
	//while (XsTime::timeStampNow() - startTime <= 10000)
	// int count =  0;
	// while (count < 10)
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
				cout << "\r"
					<< "AccHR X:" << accHR[0]
					<< ", AccHR Y:" << accHR[1]
					<< ", AccHR Z:" << accHR[2];
				AccMeasured(0) = accHR[0]; AccMeasured(1) = accHR[1]; AccMeasured(2) = accHR[2];
			}

			if (packet.containsRateOfTurnHR())
			{

				cout << endl; 
				gyrHR = packet.rateOfTurnHR();
				cout << "GyrHR X:" << gyrHR[0]
					<< ", GyrHR Y:" << gyrHR[1]
					<< ", GyrHR Z:" << gyrHR[2];
				GyrMeasured(0) = gyrHR[0]; GyrMeasured(1) = gyrHR[1]; GyrMeasured(2) = gyrHR[2];
			}

			if (packet.containsCalibratedMagneticField())
			{
				cout << endl; 
				mag = packet.calibratedMagneticField();
				cout << "Mag X:" << mag[0]
					<< ", Mag Y:" << mag[1]
					<< ", Mag Z:" << mag[2];
				MagMeasured(0) = mag[0]; MagMeasured(1) = mag[1]; MagMeasured(2) = mag[2];
			}
		}
//###################################################### Filtering ############################################################
		// mahony.Estimate(GyrMeasured, AccMeasured, MagMeasured);
		// mahony.RosPublishQuaternion();
		// mahony.PrintQuaternion();
		ekf.Estimate(GyrMeasured, AccMeasured, MagMeasured);
		ekf.RosPublishQuaternion();

		//to make loop Hz constant (1000Hz)
		while (chrono::steady_clock::now() < next);
		duration = chrono::steady_clock::now() - start;
		cout << endl << "duration time " << duration.count();
		start = chrono::steady_clock::now();
		next += Framerate{1};
		cout << flush;
		
		XsTime::msleep(0);
		ros_raw_data_publisher.RosRawDataPublish(accHR, gyrHR, mag);//, gyrHR, mag);
		//++count;
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
