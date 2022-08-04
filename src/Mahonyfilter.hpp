#pragma once

#include <iostream>
#include <vector>
#include <mutex>
#include <Eigen/Dense>

using namespace Eigen;

class MahonyFilter
{
private:
    double dt;
    double Kp, Ki, Ka, Km;
    Quaterniond quat_prev, quat;
    Matrix3d R_hat0;
    bool isInitialized;
    Vector3d world_gravity;
    Vector3d world_mag;

public:
    MahonyFilter(double Kp_input, double Ki_input, double Ka_input, double Km_input, double dt_input, Vector3d measured_mag)
    : Kp(Kp_input), Ki(Ki_input), Ka(Ka_input), Km(Km_input), dt(dt_input), quat_prev(1, 0, 0, 0), isInitialized{false},
      world_gravity(0, 0, 1), world_mag(measured_mag)
    {
    }

    Matrix3d getSkew(const Vector3d &vec)
    {
        Matrix3d mat; mat.setZero();
        mat << 0, vec(2), vec(1),
                vec(2), 0, -vec(0),
                -vec(1), vec(0), 0;
        return mat;
    }

    Vector3d getVec(const Matrix3d &mat)
    {
        Vector3d vec; vec.setZero();
        vec << mat(2, 1), mat(0, 2), mat(1,0);
        return vec;
    }

    Quaterniond getQuat(const Vector3d &vec)
    {
        Quaterniond quat; quat.setIdentity();
        quat.w() = 0;
        quat.vec() = vec;
        return quat;
    }

    Matrix4d getOmega(const Vector3d &vec)
    {
        Matrix4d omega; omega.setZero();
        omega << 0,     -vec.transpose(),
                vec,    getSkew(vec);
        return omega;
    }

    Matrix4d Euler_Rodrigues(const Vector3d vec)
    {
        Matrix4d ER_mat = cos(vec.norm()*dt/2)*Matrix4d::Identity() + (1/vec.norm())*sin(vec.norm()*dt/2)*getOmega(vec);
        return ER_mat;
    }

    Matrix4d First_Order_Approx(const Vector3d vec)
    {
        Matrix4d FO_mat = Matrix4d::Identity() + (1/2)*getOmega(vec)*dt;
        return FO_mat;
    }

    Quaterniond QuatMult(const Quaterniond &q1, const Quaterniond &q2)
    {
        Quaterniond result;
        result.w() = q1.w()*q2.w() - q1.vec().dot(q2.vec());
        result.vec() = q1.w()*q2.vec() + q2.w()*q1.vec() + q1.vec().cross(q2.vec());

        return result;       
    }

    Vector3d TransformByQuat(const Quaterniond &q, const Vector3d &vec)
    {
        Quaterniond quat_by_vec; quat_by_vec.w()=0; quat_by_vec.vec() = vec;
        Quaterniond result_quat;
        result_quat = QuatMult(QuatMult(q, quat_by_vec), q.inverse());
        return result_quat.vec();

    }

    void Estimate(const Vector3d &ang_vel_measured, const Vector3d &normalized_acc_measure, const Vector3d &normalized_mag_measure)
    {
        
        if (!isInitialized)
        {
            quat_prev = Quaterniond::Identity();
        }

        else
        {
            Vector3d acc_hat = TransformByQuat(quat_prev, normalized_acc_measure);
            Vector3d mag_hat = TransformByQuat(quat_prev, normalized_mag_measure);
        }
    }

};