#pragma once
#include <xsensdeviceapi.h>

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>

using namespace std;

class Ros_Publisher
{
private:
    ros::NodeHandle n;
	ros::Publisher acc_pub;
    ros::Publisher gyr_pub;
	ros::Publisher mag_pub;
    
	visualization_msgs::Marker acc_msgs;
    visualization_msgs::Marker gyr_msgs;
    visualization_msgs::Marker mag_msgs;


public:
    Ros_Publisher()
    {
        acc_pub = n.advertise<visualization_msgs::Marker>("/measured_acc_data", 5);
        gyr_pub = n.advertise<visualization_msgs::Marker>("/measured_gyr_data", 5);
        mag_pub = n.advertise<visualization_msgs::Marker>("/measured_mag_data", 5);
    }


    void RosPublic(const XsVector& acc, const XsVector& gyro, const XsVector& mag)
	{
		if (acc.size() > 0)
        {
            acc_msgs.header.frame_id = "world";
            acc_msgs.header.stamp = ros::Time();
            acc_msgs.ns = "my_namespace";
            acc_msgs.id = 0;
            acc_msgs.type = visualization_msgs::Marker::ARROW;
            acc_msgs.pose.orientation.w = 1;
            acc_msgs.points.resize(2);
            acc_msgs.points.at(0).x = 0.0;
            acc_msgs.points.at(0).y = 0.0;
            acc_msgs.points.at(0).z = 0.0;
            acc_msgs.points.at(1).x = acc.at(0)*0.1;
            acc_msgs.points.at(1).y = acc.at(1)*0.1;
            acc_msgs.points.at(1).z = acc.at(2)*0.1;
            acc_msgs.scale.x = 0.1;
            acc_msgs.scale.y = 0.15;
            acc_msgs.color.a = 1.0;
            acc_msgs.color.r = 1.0;
            acc_msgs.color.g = 0.0;
            acc_msgs.color.b = 0.0;
            acc_pub.publish(acc_msgs);
        }
        if (gyro.size() > 0)
        {
            gyr_msgs.header.frame_id = "world";
            gyr_msgs.header.stamp = ros::Time();
            gyr_msgs.ns = "my_namespace";
            gyr_msgs.id = 0;
            gyr_msgs.type = visualization_msgs::Marker::ARROW;
            gyr_msgs.pose.orientation.w = 1;
            gyr_msgs.points.resize(2);
            gyr_msgs.points.at(0).x = 0.0;
            gyr_msgs.points.at(0).y = 0.0;
            gyr_msgs.points.at(0).z = 0.0;
            gyr_msgs.points.at(1).x = gyro.at(0);
            gyr_msgs.points.at(1).y = gyro.at(1);
            gyr_msgs.points.at(1).z = gyro.at(2);
            gyr_msgs.scale.x = 0.1;
            gyr_msgs.scale.y = 0.15;
            gyr_msgs.color.a = 1.0;
            gyr_msgs.color.r = 0.0;
            gyr_msgs.color.g = 1.0;
            gyr_msgs.color.b = 0.0;
            gyr_pub.publish(gyr_msgs);
        }
        if (mag.size() > 0)
        {       
            mag_msgs.header.frame_id = "world";
            mag_msgs.header.stamp = ros::Time();
            mag_msgs.ns = "my_namespace";
            mag_msgs.id = 0;
            mag_msgs.type = visualization_msgs::Marker::ARROW;
            mag_msgs.pose.orientation.w = 1;
            mag_msgs.points.resize(2);
            mag_msgs.points.at(0).x = 0.0;
            mag_msgs.points.at(0).y = 0.0;
            mag_msgs.points.at(0).z = 0.0;
            mag_msgs.points.at(1).x = mag.at(0);
            mag_msgs.points.at(1).y = mag.at(1);
            mag_msgs.points.at(1).z = mag.at(2);
            mag_msgs.scale.x = 0.1;
            mag_msgs.scale.y = 0.15;
            mag_msgs.color.a = 1.0;
            mag_msgs.color.r = 0.0;
            mag_msgs.color.g = 0.0;
            mag_msgs.color.b = 1.0;
            mag_pub.publish(mag_msgs);
        }
        // imu_msg.header.frame_id = "world";
        // imu_msg.header.stamp = ros::Time::now();
        // imu_msg.orientation.x = 0;
        // imu_msg.orientation.y = 0;
        // imu_msg.orientation.z = 0;
        // imu_msg.orientation.w = 1;

		// if (acc.size() > 0)
        // {
        //     imu_msg.linear_acceleration.x = acc[0];
        //     imu_msg.linear_acceleration.y = acc[1];
        //     imu_msg.linear_acceleration.z = acc[2];     
        // }

        // else if (gyro.size() > 0)
        // {
        //     imu_msg.angular_velocity.x = gyro[0];
        //     imu_msg.angular_velocity.y = gyro[1];
        //     imu_msg.angular_velocity.z = gyro[2];
        // }

        // else
        // {
        //     imu_msg.linear_acceleration.x = 0;
        //     imu_msg.linear_acceleration.y = 0;
        //     imu_msg.linear_acceleration.z = 0;
        //     imu_msg.angular_velocity.x = 0;
        //     imu_msg.angular_velocity.y = 0;
        //     imu_msg.angular_velocity.z = 0;
        // }

		// mag_msg.header.frame_id = "world";
        // mag_msg.header.stamp = ros::Time::now();

        // if (mag.size() > 0)
        // {
        //     mag_msg.magnetic_field.x = mag[0];
        //     mag_msg.magnetic_field.y = mag[1];
        //     mag_msg.magnetic_field.z = mag[2];
        // }

        // else
        // {
        //     mag_msg.magnetic_field.x = 0;
        //     mag_msg.magnetic_field.y = 0;
        //     mag_msg.magnetic_field.z = 0;
        // }

        // imu_pub.publish(imu_msg);
        // mag_pub.publish(mag_msg);
	}
    

};