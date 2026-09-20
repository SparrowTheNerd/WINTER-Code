#include "kalman.h"
#include "kalmanfunctions.hpp"
#include "sensorfunctions.hpp"
#include "abstract.h"

void KalmanFilter::init(MMC5983* mag, ICM42688* imu, MS5607* baro, ADXL375* highG, SFE_UBLOX_GNSS* gnss) {
    this->mag = mag;
    this->imu = imu;
    this->baro = baro;
    this->highG = highG;
    this->gnss = gnss;
    double gOfst[3] = {0,0,0};
    double accelInit[3] = {0,0,0};

    for(int i=0; i<250; i++) {
        imu->ReadIMU();
        for(int j=0; j<3; j++) {
            gOfst[j] += imu->gyro_dps[j];
            accelInit[j] += imu->accel_ms2[j];
        }
        baro->GetData();
        HAL_Delay(10);
    }
    for(int j=0; j<3; j++) {
        imu->gyro_ofst[j] = gOfst[j]/250.0;
        accelInit[j] /= 250.0;
    }

    baro->Convert();
        
    qP = Quaterniond::FromTwoVectors(MapIMU(accelInit), Vector3d {0,0,9.80665});
    intState << qP.w(), qP.x(), qP.y(), qP.z(), 0.0, 0.0, baro->alt, 0.0, 0.0, 0.0;

    ll0 << (double)gnss->getLatitude()/10000000.0, (double)gnss->getLongitude()/10000000.0;

    errState.setZero();
}
void KalmanFilter::predict(double dt) {
    imu->ReadIMU();
    // highG->ReadAccel();

    Vector3d aRaw = MapIMU(imu->accel_ms2);
    Vector3d wRaw = MapIMU(imu->gyro_dps)*M_PI/180.0;

    intStatePriori = InertialIntegration(intState, wRaw, wBias, aRaw, aBias, dt);

    Quaterniond q  = Quaterniond::FromCoeffsScalarFirst (intStatePriori(0), intStatePriori(1), intStatePriori(2), intStatePriori(3));
    F = StateTransition(wRaw, aRaw, wP, aP, q, qP);
    Phi = I18 + F*dt + 0.5*F*F*dt*dt;
    Q = noiseCovariance(dt, sigGyro, sigAccel, sigBa, sigBw, sigBm);
    Pn1n = Phi*P*Phi.transpose() + Q;
    // print_matrix2(F);

    wP = wRaw; aP = aRaw; 

    // // code for doing raw integration tests
    // qP = q;
    // intState = intStatePriori;
    // P = Pn1n;

}
void KalmanFilter::update() {
    errState = Vector<double,18>::Zero();
    P=Pn1n;
    baro->GetData();
    if(baro->available) {
        baro->Convert();
        BaroMeas(intStatePriori(6), baro->alt, &hB, &dB);
        kB = P*hB.transpose() / ((hB * P * hB.transpose()) + RBaro); 
        // print_matrix2(P);
        errState = errState + kB*dB;
        P = (I18 - kB * hB) * P;

        ll << (double)gnss->getLatitude()/10000000.0, (double)gnss->getLongitude()/10000000.0;
        GPSMeas(intStatePriori.segment<2>(4), ll, ll0, &hG, &dG);
        kG = P*hG.transpose() * (hG * P * hG.transpose() + RGPS).inverse();
        errState = errState + kG*dG;
        P = (I18 - kG * hG) * P;

        vGPS << (double)gnss->getNedNorthVel()/1000.0, (double)gnss->getNedEastVel()/1000.0;
        GPSVelMeas(intStatePriori.segment<2>(7), vGPS, &hV, &dV);
        kV = P*hV.transpose() * (hV * P * hV.transpose() + RGPSVel).inverse();
        errState = errState + kV*dV;
        P = (I18 - kV * hV) * P;
    }
    

    // ... other sensor updates
    intState.segment<4>(0) = (Quaterniond(intStatePriori(0), intStatePriori.segment<3>(1)).normalized() * Quaterniond {1, errState.segment<3>(0)/2.0}.normalized()).normalized().coeffsScalarFirst(); // apply angle error as a quaternion rotation
    intState.segment<3>(4) = intStatePriori.segment<3>(4) + errState.segment<3>(6); // apply velocity error
    intState.segment<3>(7) = intStatePriori.segment<3>(7) + errState.segment<3>(3); // apply position error
    wBias += errState.segment<3>(9);
    aBias += errState.segment<3>(12);
    // sprintf((char*)usbTxBuf2, "gX: %.5f\r\ngY: %.5f\r\ngZ: %.5f", wBias.x(), wBias.y(), wBias.z());
    // SerialPrintln(usbTxBuf2);
    qP = Quaterniond(intState(0), intState.segment<3>(1)); // eigen takes either (w,xyz) or (x,y,z,w), stupid
    
    // {
    //     intState = intStatePriori;
    //     P = Pn1n;
    // }
}