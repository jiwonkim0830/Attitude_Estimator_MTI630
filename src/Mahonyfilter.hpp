#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace Eigen;

class MahonyFilter
{
private:
    const double dt;
    double Kp, Ki, Ka, Km; //filter gain
    Quaterniond quat_prev, quat_now, inv_quat_prev;
    Vector3d acc_hat, mag_hat, correction_term, A; //for correction term
    Matrix4d Omega_A; //for correction term
    const Vector3d world_gravity, world_mag;
    Vector3d normalized_world_acc; 
    Vector3d normalized_world_mag;


    const double threshold_a, threshold_m, threshold_dip;              //thresholds
    double dip_angle;
    const double world_dip_angle;

    ros::NodeHandle n;

public:
    MahonyFilter(double Kp_input, double Ki_input, double Ka_input, double Km_input, double dt_input, Vector3d measured_acc, Vector3d measured_mag,
                 double threshold_a_input, double threshold_m_input, double threshold_dip_input)
    //Kp, Ki, Ka, Km : gain tuning is needed !!
    : Kp(Kp_input), Ki(Ki_input), Ka(Ka_input), Km(Km_input), dt(dt_input), quat_prev(Quaterniond::Identity()), quat_now(Quaterniond::Identity()),
      inv_quat_prev(Quaterniond::Identity()), acc_hat(measured_acc), mag_hat(measured_mag), correction_term(Vector3d::Zero()), A(Vector3d::Zero()), 
      Omega_A(Matrix4d::Zero()), world_gravity(measured_acc), world_mag(measured_mag),

      threshold_a(threshold_a_input), threshold_m(threshold_m_input), threshold_dip(threshold_dip_input),
      world_dip_angle( acos(measured_acc.dot(measured_mag) / (measured_acc.norm() * measured_mag.norm())) )
    //Set initial pose as (1, 0, 0, 0)
    {
        normalized_world_acc = measured_acc.normalized();
        normalized_world_mag = measured_mag.normalized();
    }

    Matrix3d getSkewFromVec(const Vector3d &vec)
    {
        Matrix3d mat; mat.setZero();
        mat << 0, -vec(2), vec(1),
               vec(2), 0, -vec(0),
               -vec(1), vec(0), 0;
        return mat;
    }

    Vector3d getVecFromSkew(const Matrix3d &mat)
    {
        Vector3d vec; vec.setZero();
        vec << mat(2, 1), mat(0, 2), mat(1,0);
        return vec;
    }

    Quaterniond getQuatFromVec(const Vector3d &vec)
    {
        Quaterniond quat; quat.setIdentity();
        quat.w() = 0;
        quat.vec() = vec;
        return quat;
    }

    Matrix4d getOmegaFromVec(const Vector3d &vec)
    {
        Matrix4d omega; omega.setZero();
        omega << 0,     -vec.transpose(),
                vec,    -getSkewFromVec(vec);
        return omega;
    }

    Matrix4d Euler_Rodrigues(const Vector3d vec)
    {
        Matrix4d ER_mat = cos(vec.norm()*dt/2)*Matrix4d::Identity() + (1/vec.norm())*sin(vec.norm()*dt/2)*getOmegaFromVec(vec);
        return ER_mat;
    }

    Matrix4d First_Order_Approx(const Vector3d vec)
    {
        cout << fixed; cout.precision(6);
        Matrix4d FO_mat = Matrix4d::Zero();
        //cout << "OmegaFromVec: " << "\n" << OmegaFromVec << endl;
        FO_mat = Matrix4d::Identity() + 0.5*dt*getOmegaFromVec(vec);
        //cout << "FO_mat: " << "\n" << FO_mat << endl;
        return FO_mat;
    }


    Vector3d TransformByQuat(const Quaterniond &q, const Vector3d &vec)
    {
        Quaterniond quat_by_vec = Quaterniond::Identity(); quat_by_vec.w()=0; quat_by_vec.vec() = vec;

        Quaterniond result_quat = (q * quat_by_vec) * q.inverse();
        Vector3d result_vec = result_quat.vec();
        return result_vec;
    }

    Quaterniond MatQuatMult(const Matrix4d &mat, const Quaterniond &quat)
    {
        Quaterniond result; result = Quaterniond::Identity();
        result.w() = mat(0,0)*quat.w() + mat(0,1)*quat.x() + mat(0,2)*quat.y() + mat(0,3)*quat.z();
        result.x() = mat(1,0)*quat.w() + mat(1,1)*quat.x() + mat(1,2)*quat.y() + mat(1,3)*quat.z(); 
        result.y() = mat(2,0)*quat.w() + mat(2,1)*quat.x() + mat(2,2)*quat.y() + mat(2,3)*quat.z(); 
        result.z() = mat(3,0)*quat.w() + mat(3,1)*quat.x() + mat(3,2)*quat.y() + mat(3,3)*quat.z();  

        return result;
    }

    void Estimate(const Vector3d &ang_vel_measured, const Vector3d &acc_measure, const Vector3d &mag_measure)
    {
        cout << fixed; cout.precision(6);
        Vector3d normalized_acc = acc_measure.normalized();
        Vector3d normalized_mag = mag_measure.normalized();
        

        acc_hat = ((quat_prev.toRotationMatrix()).transpose()) * normalized_world_acc;
        cout << "\nmeasured_acc : " << normalized_acc << endl;
        cout << "\nacc_hat : " << acc_hat << endl;
        mag_hat = ((quat_prev.toRotationMatrix()).transpose()) * normalized_world_mag;
        cout << "\nmeasured_mag : " << normalized_mag << endl;
        cout << "\nmag_hat : " << mag_hat << endl;
        
        //cout << "\n" << "Mag hat : " << mag_hat[0] << " " << mag_hat[1] << " " << mag_hat[2] << endl; 
        //cout << "Acc hat : " << acc_hat[0] << " " << acc_hat[1] << " " << acc_hat[2] << endl; 

        //dip_angle = acos(((quat_prev.toRotationMatrix()*mag_measure).dot(world_gravity)) / (mag_measure.norm() * world_gravity.norm()));
        //if (fabs(acc_measure.norm() - world_gravity.norm()) < threshold_a && (fabs(mag_measure.norm() - world_mag.norm()) < threshold_m) && (fabs(dip_angle - world_dip_angle) < threshold_dip))
        //{
        correction_term = -getVecFromSkew((Ka/2)*((normalized_acc)*(acc_hat.transpose()) - acc_hat*((normalized_acc).transpose()))
                                                   + (Km/2)*((normalized_mag)*(mag_hat.transpose()) - mag_hat*(normalized_mag.transpose())));
        //}
        // else
        // {
        //     correction_term << 0.0, 0.0, 0.0;
        //     cout << "\nmeasurements is ignored !!" << endl;
        // }

        cout << "\nCorrection term : " << correction_term[0] << " " << correction_term[1] << " " << correction_term[2] << endl;

        A = ang_vel_measured + (Kp+Ki*dt)*correction_term;
        //A = ang_vel_measured + (Kp)*correction_term;
        //cout << "A : " << A[0] << " " << A[1] << " " << A[2] << endl;

        Omega_A = First_Order_Approx(A);
        //cout << "Omega_A : " << "\n" << Omega_A << endl;

        quat_now = MatQuatMult(Omega_A, quat_prev);
        quat_now.normalize();
        quat_prev = quat_now;
    }

    void PrintQuaternion()
    {
        cout << "\nquaternion : "<<quat_now.w() << " " << quat_now.x() << " " << quat_now.y() << " " << quat_now.z() << endl;
    }

    void RosPublishQuaternion()
    {
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        tf::Quaternion q;
        q.setW(quat_now.w()); q.setX(quat_now.x()); q.setY(quat_now.y()); q.setZ(quat_now.z());
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "mahony_frame"));
    }
};