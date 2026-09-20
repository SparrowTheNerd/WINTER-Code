#pragma once
#include "stm32h7xx_hal.h"
#include "Eigen/Dense"
#include "MMC5983.h"
#include "ICM42688.h"
#include "MS5607.h"
#include "ADXL375.h"
#include "SparkFun_u-blox_GNSS_v3.h"

using namespace Eigen;

#define DEG2RAD(x) (x*(M_PI/180.0))

class KalmanFilter {
    public:

        void init(MMC5983* mag, ICM42688* imu, MS5607* baro, ADXL375* highG, SFE_UBLOX_GNSS* gnss);
        void predict(double dt);
        void update();

        Vector<double,10> intState; // inertial state vector (quaternion, position, velocity)
        Vector<double,18> errState;  // error state vector (angle error, velocity error, position error, gyro bias, accel bias, mag bias)

    private:
        Vector<double,10> intStatePriori; // prior inertial state vector
        Matrix<double,18,18> P;      // error covariance matrix
        Matrix<double,18,18> Pn1n; // prior error covariance matrix
        Matrix<double,18,18> F;      // state transition matrix
        Matrix<double,18,18> Phi;    // state transition matrix second-order integration
        Matrix<double,18,18> Q;      // process noise covariance

        Matrix<double,1,18> hB; // barometer measurement matrix
        Vector<double,18> kB; // barometer Kalman gain
        double dB; // barometer innovation

        Matrix<double,2,18> hG; // GPS measurement matrix
        Matrix<double,18,2> kG; // GPS Kalman gain
        Vector2d dG; // GPS innovation
        Vector2d ll0; // reference lat/lon for GPS
        Vector2d ll; // current lat/lon for GPS

        Matrix<double,2,18> hV; // GPS velocity measurement matrix
        Matrix<double,18,2> kV; // GPS velocity Kalman gain
        Vector2d dV; // GPS velocity innovation
        Vector2d vGPS; // GPS velocity measurement

        Vector3d wBias = Vector3d::Zero();
        Vector3d aBias = Vector3d::Zero();

        Matrix<double,18,18> I18 = Matrix<double,18,18>::Identity(); // 18x18 identity matrix

        Vector3d wP; // prior gyro reading
        Vector3d aP; // prior accel reading
        Quaterniond qP; // prior quaternion

        MMC5983* mag;
        ICM42688* imu;
        MS5607* baro;
        ADXL375* highG;
        SFE_UBLOX_GNSS* gnss;

        // noise parameters

        double sigAccel = 0.000515; // m/s^2 / sqrt(hz)
        double sigGyro = DEG2RAD(0.002086); // rad/s / sqrt(hz)
        double sigMag = 0.000053; // Gauss / sqrt(hz)
        double sigBaro = 0.354508; // meters / sqrt(hz)
        double sigGPS = 3.0; // meters rms
        double sigGPSVel = 0.3; // m/s rms
        double sigBa = 0.000078;
        double sigBw = 0.00022;
        double sigBm = 0.00009;


        double RBaro = sigBaro*sigBaro;
        Matrix3d RMag = Matrix3d::Identity() * sigMag*sigMag;
        Matrix2d RGPS = Matrix2d::Identity() * sigGPS*sigGPS;
        Matrix2d RGPSVel = Matrix2d::Identity() * sigGPSVel*sigGPSVel;

};