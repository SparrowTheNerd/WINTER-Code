#include "cpp_main.h"
#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "fmac.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>

#include "abstract.h"

#include "MMC5983.h"
#include "ICM42688.h"
#include "MS5607.h"
#include "ADXL375.h"
#include "SparkFun_u-blox_GNSS_v3.h"
#include "SX1262.h"

#include "sdDMA.h"

#include "kalman.h"

#include <Eigen/Dense>
using namespace Eigen;

#define CYCLE_TIME 5000 // 5.000 ms of cycles



uint8_t usbTxBuf[USBBUF_MAXLEN];
uint16_t usbTxBufLen;

MMC5983 mag(&hi2c3, MMC5983::_400Hz, MMC5983::_200hz, MMC5983::_75);
ICM42688 imu(&hi2c3, (double[3]){-0.007883,-0.034859,0.199367}, (double[3]){0,0,0}, ICM42688::_1000dps, ICM42688::_1kHz, ICM42688::_16g, ICM42688::_1kHz);
MS5607 baro(&hi2c3, 0);
ADXL375 highG(&hi2c3, 0b1100, (int8_t[3]){0,0,0});
SFE_UBLOX_GNSS myGNSS;

KalmanFilter ekf;

Vector3d dllh2denu2(Vector3d llh0, Vector3d llh);

uint16_t bufLen = 0;

void print_matrix(Eigen::MatrixXd X);

double prevTime;
double dt;

float lipoVoltage = 0.0f;
float voltsPerBit = 3.3f / 65535.f;

int cpp_main()
{   	
    HAL_ADC_Start(&hadc1);
    while (myGNSS.begin(hi2c3)==false) {
        SerialPrintln((uint8_t*)"GNSS I2C connection failed, retrying...");
        HAL_Delay(1000);
    }

    myGNSS.setI2COutput(COM_TYPE_UBX); // Set the I2C port to output UBX only (turn off NMEA noise)
  
    myGNSS.setNavigationFrequency(20); // Solution rate in Hz (1-40Hz)
    myGNSS.setNavigationRate(1); // How many solutions to produce a measurement (1-127)
    
    myGNSS.setAutoPVT(true); // Tell the GNSS to output each solution periodically
    myGNSS.setDynamicModel(DYN_MODEL_PEDESTRIAN); // Set the dynamic model to airborne 1G

    HAL_Delay(50);

    while(imu.Init() != HAL_OK) {
        SerialPrintln((uint8_t*)"IMU Init Failed"); HAL_Delay(1000);
    }
    while(mag.Init() != HAL_OK) {
        SerialPrintln((uint8_t*)"Mag Init Failed"); HAL_Delay(1000);
    }
    while(highG.Init() != HAL_OK) {
        SerialPrintln((uint8_t*)"High-G Accel Init Failed"); HAL_Delay(1000);
    }
    while(baro.Init() != HAL_OK) {
        SerialPrintln((uint8_t*)"Barometer Init Failed"); HAL_Delay(1000);
    }
    HAL_Delay(2000);

    sdInit();

    // Enable DWT Cycle Counter
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55; 
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;


    uint32_t start = DWT->CYCCNT; // Get current cycle count
    uint32_t curr = start;
    float t = 0;
    uint32_t cycles = (HAL_RCC_GetHCLKFreq() / 1000000) * CYCLE_TIME; // 5.000 ms of cycles
    

    // while(myGNSS.getSIV() < 4) {
    //     SerialPrintln((uint8_t*)"Waiting for fix...");
    //     HAL_Delay(1000);
    // }

    ekf.init(&mag, &imu, &baro, &highG, &myGNSS);
    prevTime = (double)(HAL_GetTick())/1000.0;
    uint8_t printCounter = 0;


	for(uint8_t i=0; i<100; i++)
	{   

        if(DWT->CYCCNT - start >= cycles) {
            dt = (double)(DWT->CYCCNT - curr) / (double)HAL_RCC_GetHCLKFreq(); // Calculate elapsed time in s
            curr = DWT->CYCCNT; // Set current time
            t += dt;
            lipoVoltage = (float)analogReadSE(ADC_CHANNEL_10)*voltsPerBit*2.f; 

            // ekf.predict(dt);
            // ekf.update();

            // printCounter++;
            imu.ReadIMU();
            mag.ReadMag();
            baro.GetData();
            if (baro.available) {
                baro.Convert();
            }
           
            // dataPacket.startByte = 0xFC;
            dataPacket.time = 0.0f;
            dataPacket.time_unix = 0.0f;
            memcpy(dataPacket.accel, (float*)imu.accel_ms2, 12);
            memcpy(dataPacket.gyro, (float*)imu.gyro_dps, 12);
            memcpy(dataPacket.mag, (float*)mag.mag_gauss.data(), 12);
            dataPacket.lat = myGNSS.getLatitude();
            dataPacket.lon = myGNSS.getLongitude();
            dataPacket.gpsAlt = myGNSS.getAltitude();
            dataPacket.numsats = myGNSS.getSIV();
            dataPacket.baroAlt = (float)baro.alt;
            dataPacket.baroTemp = (float)baro.temp;
            dataPacket.voltage = (uint8_t)(lipoVoltage*10); // Convert voltage to decivolts for storage
            // dataPacket.crcpad = 0xFFFF; 

            logMachine();
            
            // if(printCounter > 20) {
            //     print_matrix(ekf.intState.transpose());
            //     // HAL_Delay(2);
            //     // print_matrix(ekf.errState.transpose());
            //     // sprintf((char*)usbTxBuf,"Pressure: %d Pa",baro.pres);
            //     // SerialPrintln(usbTxBuf);

            //     // HAL_Delay(1);
            //     // sprintf((char*)usbTxBuf, "%.5f \t %.5f \t %.5f    ",imu.accel_ms2[0], imu.accel_ms2[1], imu.accel_ms2[2]);
            //     // bufLen = sprintf((char*)usbTxBuf, ">3D|Orientation:S:cube:P:0:0:0:Q:%.5f:%.5f:%.5f:%.5f:W:1:H:1:D:1:C:#ff0000\r\n", -ekf.intState(1), ekf.intState(3), ekf.intState(2), ekf.intState(0));
            //     // bufLen += sprintf((char*)usbTxBuf+bufLen, ">aX: %.5f\r\n>aY: %.5f\r\n>aZ: %.5f\r\n", imu.accel_ms2[0], imu.accel_ms2[1], imu.accel_ms2[2]);
            //     // bufLen += sprintf((char*)usbTxBuf+bufLen, ">gX: %.5f\r\n>gY: %.5f\r\n>gZ: %.5f", imu.gyro_dps[0], imu.gyro_dps[1], imu.gyro_dps[2]);
            //     // SerialPrintln(usbTxBuf);
            //     printCounter = 0;
            // } 
        }
        else {
            i--;
            HAL_Delay(5);
        }

		// HAL_Delay(5); */
	}
    SerialPrintln((uint8_t*)"Done!");
    return 1;
}

void print_matrix(Eigen::MatrixXd X)  
{
    uint16_t bufLen = 0;

    uint8_t nrow = X.rows();
    uint8_t ncol = X.cols();
    for (uint8_t i=0; i<nrow; i++) {   
        bufLen+=sprintf((char*)usbTxBuf+bufLen, "[");

        for (uint8_t j=0; j<ncol; j++) {
            bufLen+=sprintf((char*)usbTxBuf+bufLen, "%.5f", X(i,j));

            if(j<ncol-1) bufLen+=sprintf((char*)usbTxBuf+bufLen, ",");
        }
        bufLen+=sprintf((char*)usbTxBuf+bufLen, "]\n");
    }
    SerialPrintln(usbTxBuf);
}
