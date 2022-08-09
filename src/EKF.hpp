#pragma once


#include <iostream>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

using namespace std;
using namespace Eigen;

class EKF
{
private:

    double dt;
    double sigma_gyro, sigma_acc, sigma_mag, sigma_acc_w, sigma_mag_w; //gain

    double scalar_part; Vector3d vector_part;                          //of quaternion
    
    Matrix<double, 10, 1> xHat, x;                                     //predicted, corrected state (quaternion, acc_bias, mag_bias)
    Matrix<double, 6, 1> z;                                            //mesurements
    Vector3d acc_bias, mag_bias;
    Quaterniond quaternion;                                            //for transformation
    Matrix3d rotMat;                                                   //from quaternion

    Matrix<double, 10, 10> Phi, Q;                                     //Motion model matrix, Motion model covariance matrix
    Matrix<double, 10, 10> PHat, P;                                    //predicted, corrected error covarivance matrix
    Matrix<double, 4, 3> Xi;                                           //to mapping gyro error to motion error
    Matrix<double, 6, 10> F;                                           //Linearized measurement model
    Matrix<double, 10, 6> K;                                           //Kalman Gain
    Matrix<double, 6, 6> R;                                            //Measurement model covariance matrix

    Vector3d world_gravity, world_mag;

public:

    EKF(double dt_input, double sigma_gyro_input, double sigma_acc_input, double sigma_mag_input, double sigma_acc_w_input, double sigma_mag_w_input, Vector3d first_acc, Vector3d first_mag)
    :dt(dt_input), 
     sigma_gyro(sigma_gyro_input), sigma_acc(sigma_acc_input), sigma_mag(sigma_mag_input), sigma_acc_w(sigma_acc_w_input), sigma_mag_w(sigma_mag_w_input),
     scalar_part(1), vector_part(Vector3d::Zero()), 
     quaternion(Quaterniond::Identity()), rotMat(Matrix3d::Identity()), acc_bias(Vector3d::Zero()), mag_bias(Vector3d::Zero()),
     world_gravity(first_acc), world_mag(first_mag)
    {

        getPhi(Vector3d::Zero());  //for Phi
        getXi();                   //for Xi
        getQ();                    //for Q

        InitializeFilter();
    }

    void InitializeFilter()
    {
        x << 1, 0, 0, 0,  //quaternion
             0, 0, 0,     //acc bias
             0, 0, 0;     //mag bias
        P = Q;
    }

    void getQuaternionFromHat()
    {
        quaternion.w() = xHat(0);
        quaternion.x() = xHat(1);
        quaternion.y() = xHat(2);
        quaternion.z() = xHat(3);
    }

    void getAccBiasFromHat()
    {
        acc_bias << xHat(4), xHat(5), xHat(6);
    }

    void getMagBiasFromHat()
    {
        mag_bias << xHat(7), xHat(8), xHat(9);
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

    Matrix4d first_Order_Approx(const Vector3d &ang_vel)
    {
        Matrix4d FO_mat = Matrix4d::Zero();
        Matrix4d OmegaFromVec = getOmegaFromVec(ang_vel);
        FO_mat = Matrix4d::Identity() + 0.5*dt*OmegaFromVec;
        return FO_mat;
    }
    
    void getPhi(const Vector3d &ang_vel)
    {
        Phi << first_Order_Approx(ang_vel), Matrix3d::Zero(),     Matrix3d::Zero(),
               Matrix3d::Zero(),            Matrix3d::Identity(), Matrix3d::Zero(),
               Matrix3d::Zero(),            Matrix3d::Zero(),     Matrix3d::Identity();
    }

    void getXi()
    {
        scalar_part = x(0); vector_part << x(1), x(2), x(3);
        Xi << getSkewFromVec(vector_part) + scalar_part * Matrix3d::Identity(),
              -vector_part.transpose();
    }

    void getQ()
    {
        Q << pow((dt*0.5),2)*Xi*getCovarianceMat(sigma_gyro)*Xi.transpose(), Matrix3d::Zero(),                 Matrix3d::Zero(),
             Matrix3d::Zero(),                                               dt*getCovarianceMat(sigma_acc_w), Matrix3d::Zero(),
             Matrix3d::Zero(),                                               Matrix3d::Zero(),                 dt*getCovarianceMat(sigma_mag_w);
    }

    Matrix<double, 6, 1> measurementModel()
    {
        Matrix<double, 6, 1> MeasurementModel;
        getQuaternionFromHat(); getAccBiasFromHat(); getMagBiasFromHat();
        rotMat = quaternion.toRotationMatrix();

        MeasurementModel << (rotMat.transpose()*world_gravity + acc_bias),
                            (rotMat.transpose()*world_mag + matlmag_bias);
        return MeasurementModel;
    }

    void getF() //Measurement model linearization - Jacobian is calculated from matlab
    {
        getQuaternionFromHat(); getAccBiasFromHat(); getMagBiasFromHat();
        F(0,0) = 4*xHat
    }

    void getR()
    {
        R << pow(sigma_acc,2)*Matrix3d::Identity(), Matrix3d::Zero(),
             Matrix3d::Zero(),                      pow(sigma_mag,2)*Matrix3d::Identity();
    }

    void testAcc()
    {
        
    }

    void testMag()
    {

    }

    void Estimate()
    {

    }

};