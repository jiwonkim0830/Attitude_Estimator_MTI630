#pragma once
#include <xsensdeviceapi.h>

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using namespace std;

class RosRawDataPublisher
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
    RosRawDataPublisher()
    {
        acc_pub = n.advertise<visualization_msgs::Marker>("/measured_acc_data", 5);
        gyr_pub = n.advertise<visualization_msgs::Marker>("/measured_gyr_data", 5);
        mag_pub = n.advertise<visualization_msgs::Marker>("/measured_mag_data", 5);
    }


    void RosRawDataPublish(const XsVector& acc, const XsVector& gyro, const XsVector& mag)
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
	}
    

};