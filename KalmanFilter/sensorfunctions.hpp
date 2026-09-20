#pragma once
#include "stm32h7xx_hal.h"
#include "helperfunctions.hpp"
#include "Eigen/Dense"
using namespace Eigen;

Vector3d MapIMU(double input[3]) {
    return Vector3d(-input[2], input[1], input[0]); // map board axes (X up) to body axes (Z up)
}

void BaroMeas(double posZ, double baroAlt, Matrix<double,1,18>* hB, double* dB) {
    // measurement matrix
    hB->setZero();
    (*hB)(8) = 1.0; // position Z

    // innovation
    (*dB) = baroAlt - posZ;
}

void GPSMeas(Vector2d pos, Vector2d latlon, Vector2d latlon0, Matrix<double,2,18>* hG, Vector2d* dG) {
    // measurement matrix
    Vector3d llh0 = Vector3d(latlon0(0), latlon0(1), 0.0);
    Vector3d llh = Vector3d(latlon(0), latlon(1), 0.0);
    Vector2d dENU = dllh2denu(llh0, llh).head<2>();
    hG->setZero();
    (*hG)(0,6) = 1.0; 
    (*hG)(1,7) = 1.0; 
    // innovation
    (*dG) = dENU - pos;
}

void GPSVelMeas(Vector2d vel, Vector2d velENU, Matrix<double,2,18>* hV, Vector2d* dV) {
    // measurement matrix
    hV->setZero();
    (*hV)(0,3) = 1.0; 
    (*hV)(1,4) = 1.0; 
    // innovation
    (*dV) = velENU - vel;
}