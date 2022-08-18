#include "CallbackHandler.hpp"
#include "Initialize_IMU.hpp"
#include "RosRawDataPublisher.hpp"
#include <Eigen/Dense>
#include "Mahonyfilter.hpp"
#include "XsensSdkPublisher.hpp"
#include "EKF.hpp"
#include <chrono>
using namespace std;
using namespace Eigen;

#define MAHONY
// #define EKF_
//#define XSENS_SDK


int main(int argc, char** argv)
{
	XsVector accHR, gyrHR, mag;
    Vector3d AccMeasured;
	Vector3d GyrMeasured;
	Vector3d MagMeasured;
	#ifdef XSENS_SDK
	Vector4d xsens_sdk_orientation = Vector4d::Zero();
	#endif
	
	Initialize_IMU initialize_imu;
	initialize_imu.initialize(accHR, gyrHR, mag, AccMeasured, GyrMeasured, MagMeasured);

// ################################################### For ROS ##################################################################
	ros::init(argc, argv, "visualize_node");
	RosRawDataPublisher ros_raw_data_publisher;

//############################################## Choose the filter ############################################################
	#ifdef MAHONY
	MahonyFilter mahony(1, 0.3, 1, 1, 1e-3, initialize_imu.firstAcc, initialize_imu.firstMag, 0.5, 0.1, 5); //gain tuning is needed !!
	#endif

	#ifdef EKF_
	EKF ekf(1e-3, 1e-2, 5e-2, 1e-2, 0.0, 0.0, 0.2, 0.1, 0.17, initialize_imu.firstAcc, initialize_imu.firstMag); //gain tuning is needed !!
	#endif

	#ifdef XSENS_SDK
	XsensSdkPublisher xsens_pub;
	#endif
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
		if (initialize_imu.callback.packetAvailable())
		{
			cout << setw(5) << fixed << setprecision(2);

			// Retrieve a packet
			XsDataPacket packet = initialize_imu.callback.getNextPacket();
            
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

			#ifdef XSENS_SDK
			if (packet.containsOrientation())
			{
				XsQuaternion quaternion = packet.orientationQuaternion();
				cout << "\r"
					<< "q0:" << quaternion.w()
					<< ", q1:" << quaternion.x()
					<< ", q2:" << quaternion.y()
					<< ", q3:" << quaternion.z();
				xsens_sdk_orientation(0) = quaternion.w(); xsens_sdk_orientation(1) = quaternion.x(); xsens_sdk_orientation(2) = quaternion.y(); 
				xsens_sdk_orientation(3) = quaternion.z(); 
			}
			#endif
		}
//###################################################### Filtering ############################################################
		#ifdef MAHONY
		mahony.Estimate(GyrMeasured, AccMeasured, MagMeasured);
		mahony.RosPublishQuaternion();
		mahony.PrintQuaternion();
		#endif

		#ifdef EKF_
		ekf.Estimate(GyrMeasured, AccMeasured, MagMeasured);
		ekf.RosPublishQuaternion();
		#endif

		#ifdef XSENS_SDK
		xsens_pub.RosPublishQuaternion(xsens_sdk_orientation);
		#endif

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
//###############################################################################################################################
	return 0;
}
