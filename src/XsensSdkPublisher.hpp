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

    void RosPublishQuaternion(Quaterniond quat_input)
    {
        
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        tf::Quaternion q;
        q.setW(quat_input.w()); q.setX(quat_input.x()); q.setY(quat_input.y()); q.setZ(quat_input.z());
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "xsen_sdk_frame"));
    }
};