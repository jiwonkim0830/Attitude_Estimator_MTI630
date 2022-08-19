#pragma once

#include "CallbackHandler.hpp"
#include <Eigen/Dense>
#include <chrono>
using namespace std;
using namespace Eigen;

// #define MAHONY
// #define EKF_
#define XSENS_SDK

class Initialize_IMU
{
private:
    XsControl* control;
    XsPortInfo mtPort;
    bool isAccInitialized;
	bool isMagInitialized;
	bool isGyrInitialized;

	#ifdef XSENS_SDK
	Vector4d xsens_sdk_orientation = Vector4d::Zero();
	#endif

public:
    XsDevice* device;
    CallbackHandler callback;
    Vector3d firstAcc;
	Vector3d firstMag;
	Vector3d firstGyr;

    Initialize_IMU()
    : isAccInitialized(false), isMagInitialized(false), isGyrInitialized(false)
    {
    }
    
    ~Initialize_IMU()
    {     
        cout << "\n" << string(79, '-') << "\n";
        cout << endl;

        cout << "Closing port..." << endl;
        control->closePort(mtPort.portName().toStdString());

        cout << "Freeing XsControl object..." << endl;
        control->destruct();

        cout << "Successful exit." << endl;

        cout << "Press [ENTER] to continue." << endl;
        cin.get();
    }

    void initialize(XsVector &accHR, XsVector &gyrHR, XsVector &mag, Vector3d &AccMeasured, Vector3d &GyrMeasured, Vector3d &MagMeasured)
    {
// ################################################ Scanning & Setting ################################################        
        cout<<fixed; cout.precision(6);
        cout << "Creating XsControl object..." << endl;
        control = XsControl::construct();
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
        {
            control->destruct();
            cout << "No MTi device found. Aborting." << endl;
            cout << "Press [ENTER] to continue." << endl;
            cin.get();
            exit(1);
        }

        cout << "Found a device with ID: " << mtPort.deviceId().toString().toStdString() << " @ port: " << mtPort.portName().toStdString() << ", baudrate: " << mtPort.baudrate() << endl;

        cout << "Opening port..." << endl;
        if (!control->openPort(mtPort.portName().toStdString(), mtPort.baudrate()))
        {
            control->destruct();
            cout << "Could not open port. Aborting." << endl;
            cout << "Press [ENTER] to continue." << endl;
            cin.get();
            exit(1);
        }

        // Get the device object
        device = control->device(mtPort.deviceId());
        assert(device != nullptr);

        cout << "Device: " << device->productCode().toStdString() << ", with ID: " << device->deviceId().toString() << " opened." << endl;

        // Create and attach callback handler to device
        device->addCallbackHandler(&callback);

//######################################################## Configuration #########################################################
	// Put the device into configuration mode before configuring the device
        cout << "Putting device into configuration mode..." << endl;
        if (!device->gotoConfig())
        {
            control->destruct();
            cout << "Could not put device into configuration mode. Aborting." << endl;
            cout << "Press [ENTER] to continue." << endl;
            cin.get();
            exit(1);
        }

        cout << "Configuring the device..." << endl;
        XsOutputConfigurationArray configArray;
        configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
        configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));

        if (device->deviceId().isVru() || device->deviceId().isAhrs())
        {
            #ifdef XSENS_SDK
            configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 0xFFFF));
            #endif

            configArray.push_back(XsOutputConfiguration(XDI_AccelerationHR, 0xFFFF));
            configArray.push_back(XsOutputConfiguration(XDI_RateOfTurnHR, 0xFFFF));
            configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 0xFFFF));
        }

        else
        {
            control->destruct();
            cout << "Unknown device while configuring. Aborting." << endl;
            cout << "Press [ENTER] to continue." << endl;
            cin.get();
            exit(1);
        }

        if (!device->setOutputConfiguration(configArray))
        {
            control->destruct();
            cout << "Could not configure MTi device. Aborting." << endl;
            cout << "Press [ENTER] to continue." << endl;
            cin.get();
            exit(1);
        }


        cout << "Putting device into measurement mode..." << endl;
        if (!device->gotoMeasurement())
        {
            control->destruct();
            cout << "Could not put device into measurement mode. Aborting." << endl;
            cout << "Press [ENTER] to continue." << endl;
            cin.get();
            exit(1);
        }

        cout << "\nMain loop." << endl;
        cout << string(79, '-') << endl;

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
    }

    void initializeSDKFilter(XsQuaternion &quaternion, Quaterniond &first_quaternion)
    {
        bool isSDKFilterinitialized = false;

        while (!isSDKFilterinitialized)
        {
            if (callback.packetAvailable())
            {
                XsDataPacket packet = callback.getNextPacket();
                if (packet.containsOrientation())
                {
                    quaternion = packet.orientationQuaternion();
                    cout << "\r"
                        << "q0:" << quaternion.w()
                        << ", q1:" << quaternion.x()
                        << ", q2:" << quaternion.y()
                        << ", q3:" << quaternion.z();
                    first_quaternion.w() = quaternion.w(); first_quaternion.x() = quaternion.x(); first_quaternion.y() = quaternion.y(); 
                    first_quaternion.z() = quaternion.z(); 

                    isSDKFilterinitialized = true;
                }
            }
        }
        cout << "SDK Filter Initialized !" << endl;
        cout << "First quaternion : " << first_quaternion.w() << " " << first_quaternion.x() << " " << first_quaternion.y() << first_quaternion.z() << endl;
    }
    
};