#pragma once
#include "stm32h7xx_hal.h"
#include "Eigen/Dense"
#include "abstract.h"
using namespace Eigen;

uint8_t usbTxBuf2[256];

Matrix3d I3 = Matrix3d::Identity();


void print_matrix2(Eigen::MatrixXd X)
{
    uint16_t bufLen = 0;

    uint8_t nrow = X.rows();
    uint8_t ncol = X.cols();
    for (uint8_t i = 0; i < nrow; i++)
    {
        bufLen += sprintf((char *)usbTxBuf2 + bufLen, "[");

        for (uint8_t j = 0; j < ncol; j++)
        {
            bufLen += sprintf((char *)usbTxBuf2 + bufLen, "%.5f", X(i, j));

            if (j < ncol - 1)
                bufLen += sprintf((char *)usbTxBuf2 + bufLen, ",");
        }
        bufLen += sprintf((char *)usbTxBuf2 + bufLen, "]\n");
    }
    SerialPrintln(usbTxBuf2);
}

Quaterniond QuaternionAverage(Quaterniond q1, Quaterniond q2)
{
    if (std::abs((q1.conjugate() * q2).w()) > 0.9999) // if the quaternions are the same, return q1
    {
        return q1;
    }

    Quaterniond r = q1.conjugate() * q2;
    // print_matrix2(r.coeffsScalarFirst());
    double uMag = fabs(2.0 * acos(r.w()));
    Vector3d u = r.vec() * (uMag / sin(uMag / 2.0));
    r = Quaterniond(cos(uMag / 4.0), u / uMag * sin(uMag / 4.0));

    return (q1 * r).normalized();
}

Matrix3d SkewSymmetric(Vector3d v)
{
    return Matrix3d{ {0, -v(2), v(1)},
                    {v(2), 0, -v(0)},
                    {-v(1), v(0), 0} };
}

Matrix3d Quat2DCM(Quaterniond q) {
    // return Matrix3d { {q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z(),     2*(q.x()*q.y() - q.w()*q.z()),                  2*(q.x()*q.z() + q.w()*q.y())},
    //                 {2*(q.x()*q.y() + q.w()*q.z()),                  q.w()*q.w() - q.x()*q.x() + q.y()*q.y() - q.z()*q.z(),      2*(q.y()*q.z() - q.w()*q.x())},
    //                 {2*(q.x()*q.z() - q.w()*q.y()),                              2*(q.y()*q.z() + q.w()*q.x()),         q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z()}};
    return Matrix3d { 2.0*q.vec()*q.vec().transpose() + I3*(q.w()*q.w() - q.vec().transpose()*q.vec()) - (2*q.w() * SkewSymmetric(-q.vec())) };
}

Vector3d dllh2denu(Vector3d llh0, Vector3d llh) {
    const double a = 6378137.0;
    const double b = 6356752.3142;
    const double e2 = 1.0 - (b / a) * (b / a);
    
    // Reference point in radians
    const double phi = llh0(0) * M_PI / 180.0;
    const double lam = llh0(1) * M_PI / 180.0;
    const double h = llh0(2);

    // Point relative to reference
    const double dphi = llh(0) * M_PI / 180.0 - phi;
    const double dlam = llh(1) * M_PI / 180.0 - lam;
    const double dh = llh(2) - h;

    // Useful terms
    const double tmpl = std::sqrt(1.0 - e2 * std::sin(phi) * std::sin(phi));
    const double cp = std::cos(phi);
    const double sp = std::sin(phi);

    // Transformations
    const double de = (a / tmpl + h) * cp * dlam
                     - (a * (1.0 - e2) / (tmpl * tmpl * tmpl) + h) * sp * dphi * dlam
                     + cp * dlam * dh;

    const double dn = (a * (1.0 - e2) / (tmpl * tmpl * tmpl) + h) * dphi
                     + 1.5 * cp * sp * a * e2 * dphi * dphi
                     + sp * sp * dh * dphi
                     + 0.5 * sp * cp * (a / tmpl + h) * dlam * dlam;

    const double du = dh
                     - 0.5 * (a - 1.5 * a * e2 * cp * cp + 0.5 * a * e2 + h) * dphi * dphi
                     - 0.5 * cp * cp * (a / tmpl - h) * dlam * dlam;

    return Vector3d(de, dn, du);
}