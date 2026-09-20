#pragma once

#include <stm32h7xx_hal.h>
#include <fatfs.h>
#include "abstract.h"
#include <string.h>
#include <stdio.h>
#include <spi.h>

struct packet {
    // uint8_t startByte = 0xFC;
    float time; // millisecond runtime
    uint32_t time_unix; // GPS UTC time
    uint8_t state; // state machine state
    float ekfState[10]; // inertial state vector (quaternion, position, velocity)
    float errState[18]; // error state vector (angle error, velocity error, position error, gyro bias, accel bias, mag bias)
    float pDiag[9]; // diagonal of P matrix not including sensor biases
    float accel[3]; // accelerometer 
    float highG[3]; // high-G accelerometer
    float gyro[3]; // gyroscope
    float mag[3]; // magnetometer
    float lat; // GPS latitude
    float lon; // GPS longitude
    float gpsAlt; // GPS altitude
    uint8_t numsats; // number of satellites used in fix
    float baroAlt; // barometer altitude
    float baroTemp; // barometer temperature
    uint8_t voltage; // battery voltage in decivolts
    uint8_t pyros[6]; // pyro channels
    uint8_t servos[4]; // servo PWM values
    uint8_t pad[19] = {0xFF}; // padding to make 256 bytes
} __attribute__((packed));

extern packet dataPacket;

typedef enum { // SD card state machine states
    SD_IDLE = 0,
    SD_OPEN,
    SD_STREAMING,
    SD_CLOSE
} SD_State_t;

void sdInit();
void logMachine();
void logData();
void SD_Busy_Wait();
void SD_Ready_Callback();
