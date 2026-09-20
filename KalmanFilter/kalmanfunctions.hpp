#pragma once
#include "stm32h7xx_hal.h"
#include "helperfunctions.hpp"
#include "Eigen/Dense"
using namespace Eigen;

extern Matrix3d I3;


Matrix<double,18,18> StateTransition(Vector3d w, Vector3d a, Vector3d wP, Vector3d aP, Quaterniond q, Quaterniond qP) {
    Quaterniond qAvg = QuaternionAverage(qP, q);
    Matrix3d wx = -SkewSymmetric((wP+w)/2.0);
    Matrix3d Cbif = ((-Quat2DCM(q)*SkewSymmetric(aP)) + (-Quat2DCM(qP)*SkewSymmetric(a)))/2.0;
    Matrix3d Cbi = -Quat2DCM(qAvg);
    Matrix<double,18,18> F = Matrix<double,18,18>::Zero();
    F.block<3,3>(0,0) = wx;
    F.block<3,3>(3,0) = Cbif;
    F.block<3,3>(6,3) = I3;
    F.block<3,3>(0,9) = I3*-1.0;
    F.block<3,3>(3,12) = Cbi;
    // print_matrix2(qAvg.coeffsScalarFirst());
    return F;
}

Vector<double,10> InertialIntegration(Vector<double,10> x, Vector3d wRaw, Vector3d wBias, Vector3d aRaw, Vector3d aBias, double dt) {
    double dt2 = dt/2.0;
    Matrix4d I4 = Matrix4d::Identity();
    Vector3d w = wRaw - wBias;
    Vector3d a = aRaw - aBias;
    Quaterniond q = Quaterniond::FromCoeffsScalarFirst (x(0), x(1), x(2), x(3));

    // Quaterniond q1 {
    //     q.w() - dt2*w.x()*q.x() - dt2*w.y()*q.y() - dt2*w.z()*q.z(),
    //     q.x() + dt2*w.x()*q.w() - dt2*w.y()*q.z() + dt2*w.z()*q.y(),
    //     q.y() + dt2*w.x()*q.z() + dt2*w.y()*q.w() - dt2*w.z()*q.x(),
    //     q.z() - dt2*w.x()*q.y() + dt2*w.y()*q.x() + dt2*w.z()*q.w()
    // };
    // q1.normalize();

    Matrix4d omega {    {0.0, -w.x(), -w.y(), -w.z()},
                        {w.x(), 0.0, w.z(), -w.y()},
                        {w.y(), -w.z(), 0.0, w.x()},
                        {w.z(), w.y(), -w.x(), 0.0}
                    };
    double wN = w.norm();
    Vector4d q1Vec = (cos(wN*dt2)*I4 + (1.0/wN)*sin(wN*dt2)*omega)*q.coeffsScalarFirst();
    Quaterniond q1 = Quaterniond::FromCoeffsScalarFirst (q1Vec(0), q1Vec(1), q1Vec(2), q1Vec(3));
    q1.normalize();
    Vector3d aG = q1*a;
    aG.z() -= 9.80665; // remove gravity
    Vector3d V = x.segment<3>(7) + aG*dt;
    return Vector<double,10> {
        q1.w(),
        q1.x(),
        q1.y(),
        q1.z(),
        x(4) + V.x()*dt,
        x(5) + V.y()*dt,
        x(6) + V.z()*dt,
        V.x(),
        V.y(),
        V.z()
    };
}

Matrix<double,18,18> noiseCovariance(double dT, double pnw, double pna, double Ba, double Bw, double Bm) {
    Matrix<double,18,18> Q = Matrix<double,18,18>::Zero();
    Matrix3d Iw = I3 * (pnw * pnw);
    Matrix3d Ia = I3 * (pna * pna);
    Matrix3d IBw = I3 * (Bw * Bw);
    Matrix3d IBa = I3 * (Ba * Ba);
    Matrix3d IBm = I3 * (Bm * Bm);

    Q.block<3,3>(0,0) = Iw*dT + IBw*(dT*dT*dT/3.0);
    Q.block<3,3>(9,0) = -IBw*(dT*dT/2.0);
    Q.block<3,3>(3,3) = Ia*dT + IBa*(dT*dT*dT/3.0);
    Q.block<3,3>(6,3) = Ia*(dT*dT/2.0) + IBa*(dT*dT*dT*dT/8.0);
    Q.block<3,3>(12,3) = -IBa*(dT*dT/2.0);
    Q.block<3,3>(3,6) = Ia*(dT*dT/2.0) + IBa*(dT*dT*dT*dT/8.0);
    Q.block<3,3>(6,6) = Ia*(dT*dT*dT/3.0) + IBa*(dT*dT*dT*dT*dT/20.0);
    Q.block<3,3>(12,6) = -IBa*(dT*dT*dT/6.0);
    Q.block<3,3>(0,9) = -IBw*(dT*dT/2.0);
    Q.block<3,3>(9,9) = IBw*(dT*dT/2.0);
    Q.block<3,3>(3,12) = -IBa*(dT*dT/2.0);
    Q.block<3,3>(6,12) = -IBa*(dT*dT*dT/6.0);
    Q.block<3,3>(12,12) = IBa*dT;
    Q.block<3,3>(15,15) = IBm*dT;

    return Q;
}