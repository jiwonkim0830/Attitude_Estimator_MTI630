#pragma once

#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class XsensSdkPublisher
{
private:

public:
    XsensSdkPublisher()
    {}

    void RosPublishQuaternion(Vector4d quat_input)
    {
        
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        tf::Quaternion q;
        q.setW(quat_input(0)); q.setX(quat_input(1)); q.setY(quat_input(2)); q.setZ(quat_input(3));
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "xsen_sdk_frame"));
    }
};