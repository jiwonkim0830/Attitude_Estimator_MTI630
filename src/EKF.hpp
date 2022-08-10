#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <limits.h>

using namespace std;
using namespace Eigen;

class EKF
{
private:

    double dt;
    double sigma_gyro, sigma_acc, sigma_mag, sigma_acc_w, sigma_mag_w; //gains
    const double sigma_acc_const, sigma_mag_const;
    const double threshold_a, threshold_m, threshold_dip;                             //thresholds
    double dip_angle;
    const double world_dip_angle;

    double scalar_part; Vector3d vector_part;                          //of quaternion
    
    Matrix<double, 10, 1> xHat, x;                                     //predicted, corrected state (quaternion, acc_bias, mag_bias)
    Matrix<double, 6, 1> z;                                            //mesurements

    Matrix<double, 10, 10> Phi, Q;                                     //Motion model matrix, Motion model covariance matrix
    Matrix<double, 10, 10> PHat, P;                                    //predicted, corrected error covarivance matrix
    Matrix<double, 4, 3> Xi;                                           //to mapping gyro error to motion error
    Matrix<double, 6, 10> F;                                           //Linearized measurement model
    Matrix<double, 10, 6> KalmanGain;                                  //Kalman Gain
    Matrix<double, 6, 6> R;                                            //Measurement model covariance matrix

    Vector3d world_gravity, world_mag;

public:

    EKF(double dt_input, double sigma_gyro_input, double sigma_acc_input, double sigma_mag_input, double sigma_acc_w_input, double sigma_mag_w_input, 
    double threshold_a_input, double threshold_m_input, double threshold_dip_input, Vector3d first_acc, Vector3d first_mag)
    : dt(dt_input), 
      sigma_gyro(sigma_gyro_input), sigma_acc_const(sigma_acc_input), sigma_mag_const(sigma_mag_input), sigma_acc_w(sigma_acc_w_input), sigma_mag_w(sigma_mag_w_input),
      scalar_part(1), vector_part(Vector3d::Zero()), 
      world_dip_angle( acos(first_acc.dot(first_mag) / (first_acc.norm() * first_mag.norm())) ),
      world_gravity(first_acc), world_mag(first_mag),
      threshold_a(threshold_a_input), threshold_m(threshold_m_input), threshold_dip(threshold_dip_input)
    {
        InitializeFilter(first_acc, first_mag);
        getPhi(Vector3d::Zero());  //for Phi
        getQ();                    //for Q
        P = Q;
    }

    void InitializeFilter(Vector3d &acc, Vector3d &mag)
    {
        x << 1, 0, 0, 0,  //quaternion
             0, 0, 0,     //acc bias
             0, 0, 0;     //mag bias

        z << acc,
             mag;
    }


    Matrix3d getCovarianceMat(const double &sigma)
    {
        return pow(sigma, 2) * Matrix3d::Identity(); 
    }

    Matrix3d getSkewFromVec(const Vector3d &vec)
    {
        Matrix3d mat; mat.setZero();
        mat << 0, vec(2), vec(1),
                vec(2), 0, -vec(0),
                -vec(1), vec(0), 0;
        return mat;
    }

    Matrix4d getOmegaFromVec(const Vector3d &vec)
    {
        Matrix4d omega; omega.setZero();
        omega << 0,     -vec.transpose(),
                vec,    getSkewFromVec(vec);
        return omega;
    }

    Matrix3d getRotMatfromQuat(const Vector4d &q)
    {
        Matrix3d RotMat; RotMat.setZero();

        RotMat << 2 * (q(0) * q(0) + q(1) * q(1)) - 1, 2 * (q(1) * q(2) - q(0) * q(3)),     2 * (q(1) * q(3) + q(0) * q(2)),
                  2 * (q(1) * q(2) + q(0) * q(3)),     2 * (q(0) * q(0) + q(2) * q(2)) - 1, 2 * (q(2) * q(3) - q(0) * q(1)),
                  2 * (q(1) * q(3) - q(0) * q(2)),     2 * (q(2) * q(3) + q(0) * q(1)),     2 * (q(0) * q(0) + q(3) * q(3)) - 1;
        
    }

    Matrix4d firstOrderApprox(const Vector3d &ang_vel)
    {
        Matrix4d FO_mat = Matrix4d::Zero();
        Matrix4d OmegaFromVec = getOmegaFromVec(ang_vel);
        FO_mat = Matrix4d::Identity() + 0.5*dt*OmegaFromVec;
        return FO_mat;
    }
    
    void getPhi(const Vector3d &ang_vel)
    {
        Phi << firstOrderApprox(ang_vel), Matrix3d::Zero(),     Matrix3d::Zero(),
               Matrix3d::Zero(),            Matrix3d::Identity(), Matrix3d::Zero(),
               Matrix3d::Zero(),            Matrix3d::Zero(),     Matrix3d::Identity();
    }

    void getQ()
    {
        scalar_part = x(0); vector_part << x(1), x(2), x(3);
        Xi << getSkewFromVec(vector_part) + scalar_part * Matrix3d::Identity(),
              -vector_part.transpose();
        
        Q << pow((dt*0.5), 2)*Xi*getCovarianceMat(sigma_gyro)*Xi.transpose(), Matrix3d::Zero(),                 Matrix3d::Zero(),
             Matrix3d::Zero(),                                                dt*getCovarianceMat(sigma_acc_w), Matrix3d::Zero(),
             Matrix3d::Zero(),                                                Matrix3d::Zero(),                 dt*getCovarianceMat(sigma_mag_w);
    }

    void getMeasurements(Vector3d acc_measured, Vector3d mag_measured)
    {
        z << acc_measured,
             mag_measured;
    }

    void getF() //Measurement model linearization - Jacobian is calculated from matlab
    {
        // rotMat = quaternion.toRotationMatrix();
        // MeasurementModel << (rotMat.transpose()*world_gravity + acc_bias),
        //                     (rotMat.transpose()*world_mag + mag_bias);
        
        F = Matrix<double, 6, 10>::Zero();

        F(0,0) = 4*world_gravity(0)*xHat(0) + 2*world_gravity(1)*xHat(3) - 2*world_gravity(2)*xHat(2);  //4*g0*q0 + 2*g1*q3 - 2*g2*q2
        F(0,1) = 4*world_gravity(0)*xHat(1) + 2*world_gravity(1)*xHat(2) + 2*world_gravity(2)*xHat(3);  //4*g0*q1 + 2*g1*q2 + 2*g2*q3
        F(0,2) = 2*world_gravity(1)*xHat(1) - 2*world_gravity(2)*xHat(0);                               //2*g1*q1 - 2*g2*q0
        F(0,3) = 2*world_gravity(1)*xHat(0) + 2*world_gravity(2)*xHat(1);                               //2*g1*q0 + 2*g2*q1
        F(0,4) = 1;  F(0,5) = 0;  F(0,6) = 0;  F(0,7) = 0;  F(0,8) = 0;  F(0,9) = 0;                    //1 0 0 0 0 0

        F(1,0) = 4*world_gravity(1)*xHat(0) - 2*world_gravity(0)*xHat(3) + 2*world_gravity(2)*xHat(1);  //4*g1*q0 - 2*g0*q3 + 2*g2*q1
        F(1,1) = 2*world_gravity(0)*xHat(2) + 2*world_gravity(2)*xHat(0);                               //2*g0*q2 + 2*g2*q0
        F(1,2) = 2*world_gravity(0)*xHat(1) + 4*world_gravity(1)*xHat(2) + 2*world_gravity(2)*xHat(3);  //2*g0*q1 + 4*g1*q2 + 2*g2*q3
        F(1,3) = 2*world_gravity(2)*xHat(2) - 2*world_gravity(0)*xHat(0);                               //2*g2*q2 - 2*g0*q0
        F(1,4) = 0;  F(1,5) = 1;  F(1,6) = 0;  F(1,7) = 0;  F(1,8) = 0;  F(1,9) = 0;                    //0 1 0 0 0 0

        F(2,0) = 2*world_gravity(0)*xHat(2) - 2*world_gravity(1)*xHat(1) + 4*world_gravity(2)*xHat(0);  //2*g0*q2 - 2*g1*q1 + 4*g2*q0
        F(2,1) = 2*world_gravity(0)*xHat(3) - 2*world_gravity(1)*xHat(0);                               //2*g0*q3 - 2*g1*q0
        F(2,2) = 2*world_gravity(0)*xHat(0) + 2*world_gravity(1)*xHat(3);                               //2*g0*q0 + 2*g1*q3
        F(2,3) = 2*world_gravity(0)*xHat(1) + 2*world_gravity(1)*xHat(2) + 4*world_gravity(2)*xHat(3);  //2*g0*q1 + 2*g1*q2 + 4*g2*q3
        F(2,4) = 0;  F(2,5) = 0;  F(2,6) = 1;  F(2,7) = 0;  F(2,8) = 0;  F(2,9) = 0;                    //0 0 1 0 0 0

        F(3,0) = 4*world_mag(0)*xHat(0) + 2*world_mag(1)*xHat(3) - 2*world_mag(2)*xHat(2);              //4*h0*q0 + 2*h1*q3 - 2*h2*q2
        F(3,1) = 4*world_mag(0)*xHat(1) + 2*world_mag(1)*xHat(2) + 2*world_mag(2)*xHat(3);              //4*h0*q1 + 2*h1*q2 + 2*h2*q3
        F(3,2) = 2*world_mag(1)*xHat(1) - 2*world_mag(2)*xHat(0);                                       //2*h1*q1 - 2*h2*q0
        F(3,3) = 2*world_mag(1)*xHat(0) + 2*world_mag(2)*xHat(1);                                       //2*h1*q0 + 2*h2*q1
        F(3,4) = 0;  F(3,5) = 0;  F(3,6) = 0;  F(3,7) = 1;  F(3,8) = 0;  F(3,9) = 0;                    //0 0 0 1 0 0

        F(4,0) = 4*world_mag(1)*xHat(0) - 2*world_mag(0)*xHat(3) + 2*world_mag(2)*xHat(1);              //4*h1*q0 - 2*h0*q3 + 2*h2*q1
        F(4,1) = 2*world_mag(0)*xHat(2) + 2*world_mag(2)*xHat(0);                                       //2*h0*q2 + 2*h2*q0
        F(4,2) = 2*world_mag(0)*xHat(1) + 4*world_mag(1)*xHat(2) + 2*world_mag(2)*xHat(3);              //2*h0*q1 + 4*h1*q2 + 2*h2*q3
        F(4,3) = 2*world_mag(2)*xHat(2) - 2*world_mag(0)*xHat(0);                                       //2*h2*q2 - 2*h0*q0
        F(4,4) = 0;  F(4,5) = 0;  F(4,6) = 0;  F(4,7) = 0;  F(4,8) = 1;  F(4,9) = 0;                    //0 0 0 0 1 0

        F(5,0) = 2*world_mag(0)*xHat(2) - 2*world_mag(1)*xHat(1) + 4*world_mag(2)*xHat(0);              //2*h0*q2 - 2*h1*q1 + 4*h2*q0
        F(5,1) = 2*world_mag(0)*xHat(3) - 2*world_mag(1)*xHat(0);                                       //2*h0*q3 - 2*h1*q0
        F(5,2) = 2*world_mag(0)*xHat(0) + 2*world_mag(1)*xHat(3);                                       //2*g0*q0 + 2*g1*q3
        F(5,3) = 2*world_mag(0)*xHat(1) + 2*world_mag(1)*xHat(2) + 4*world_mag(2)*xHat(3);              //2*h0*q1 + 2*h1*q2 + 4*h2*q3
        F(5,4) = 0;  F(5,5) = 0;  F(5,6) = 0;  F(5,7) = 0;  F(5,8) = 0;  F(5,9) = 1;                    //0 0 1 0 0 0
    }

    void getR()
    {
        R << pow(sigma_acc,2)*Matrix3d::Identity(), Matrix3d::Zero(),
             Matrix3d::Zero(),                      pow(sigma_mag,2)*Matrix3d::Identity();
    }

    void testMeasurements(Vector3d acc_measured, Vector3d mag_measured)
    {
        Vector4d qHat(xHat(0), xHat(1), xHat(2), xHat(3));
        dip_angle = acos(((getRotMatfromQuat(qHat)*mag_measured).dot(world_gravity)) / (mag_measured.norm() * world_gravity.norm()));

        if (fabs(acc_measured.norm() - world_gravity.norm()) < threshold_a)
            sigma_acc = sigma_acc_const;
        else 
            sigma_acc = DBL_MAX;

        if ((fabs(mag_measured.norm() - world_mag.norm()) < threshold_m) && (fabs(dip_angle - world_dip_angle) < threshold_dip))
            sigma_mag = sigma_mag_const;
        else
            sigma_mag = DBL_MAX;
    }


    void Estimate(const Vector3d &ang_vel_measured, const Vector3d &acc_measure, const Vector3d &mag_measure)
    {
        
    }

};