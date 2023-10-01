/*
 * MPU6050.h
 *
 *  Created on: Aug 6, 2023
 *      Author: plabon
 */


#ifndef MPU6050_H_
#define MPU6050_H_

/*----------------------------------Includes----------------------------------*/
#include "IfxI2c_I2c.h"
#include "IfxI2c.h"
#include "stdint.h"


/*Baud rate*/
#define     MPU6050_BAUDRATE    400000
/*MPU I2C Address*/
#define     MPU_I2C_ADDR         0xD0
/*WHO_AM_I Register Address (used to verify the identity of the device) */
#define     WHO_AM_I            0x75
/*WHO_AM_I Return Value*/
#define     MPU_I_AM            0x68
/*MPU Registers*/
#define     SELF_TEST_X         0x0D
#define     SELF_TEST_Y         0x0E
#define     SELF_TEST_Z         0x0F
#define     SELF_TEST_A         0x10
#define     SMPLRT_DIV          0x19
#define     CONFIG              0x1A
#define     GYRO_CONFIG         0x1B
#define     ACCEL_CONFIG        0x1C
#define     FIFO_EN             0x23
#define     I2C_MST_CTRL        0x24
#define     I2C_SLV0_ADDR       0X25
#define     I2C_SLV0_REG        0X26
#define     I2C_SLV0_CTRL       0x27
#define     I2C_SLV1_ADDR       0x28

#define     ACCEL_XOUT_H        0x3B
#define     ACCEL_XOUT_L        0x3C
#define     ACCEL_YOUT_H        0x3D
#define     ACCEL_YOUT_L        0x3E
#define     ACCEL_ZOUT_H        0x3F
#define     ACCEL_ZOUT_L        0x40
#define     TEMP_OUT_H          0x41
#define     TEMP_OUT_L          0x42
#define     GYRO_XOUT_H         0x43
#define     GYRO_XOUT_L         0x44
#define     GYRO_YOUT_H         0x45
#define     GYRO_YOUT_L         0x46
#define     GYRO_ZOUT_H         0x47
#define     GYRO_ZOUT_L         0x48

#define     USER_CTRL           0x6A
#define     PWR_MGMT_1          0x6B
#define     PWR_MGMT_2          0x6C

/*Sample Rates*/
#define     MPU6050_SampleRate_8KHz     0
#define     MPU6050_SampleRate_4KHz     1
#define     MPU6050_SampleRate_2KHz     3
#define     MPU6050_SampleRate_1KHz     7
#define     MPU6050_SampleRate_500Hz    15
#define     MPU6050_SampleRate_250Hz    31
#define     MPU6050_SampleRate_125Hz    63
#define     MPU6050_SampleRate_100Hz    79

/*MPU_6060 Accelerometer Sensitivities*/
/*The values ​​of the accelerometer sensitivities are defined, in LSB/g for the multipliers of the readings.*/

#define     MPU6050_ACCELEROMETER_SENSITIVITY_2G    ((float)16384)
#define     MPU6050_ACCELEROMETER_SENSITIVITY_4G    ((float)8192)
#define     MPU6050_ACCELEROMETER_SENSITIVITY_8G    ((float)4096)
#define     MPU6050_ACCELEROMETER_SENSITIVITY_16G   ((float)2048)



/*MPU_6060 Gyroscope Sensitivities*/
/*The values ​​of the Gyroscope sensitivities are defined, in LSB/g for the multipliers of the readings.*/

#define     MPU6050_GYROSCOPE_SENSITIVITY_250    ((float)131)
#define     MPU6050_GYROSCOPE_SENSITIVITY_500    ((float)65.5)
#define     MPU6050_GYROSCOPE_SENSITIVITY_1000   ((float)32.8)
#define     MPU6050_GYROSCOPE_SENSITIVITY_2000   ((float)16.4)


/*-----------------------Typedefs-------------------*/

/*MPU_6050 Return values ​​of functions*/
typedef enum
{
    MPU6050_OK  =   0x01,                   /*No problem*/
    MPU6050_ERROR,                          /*Unknown error*/
    MPU6050_DeviceDisconnected,             /*The device is not connected*/
    MPU6050_InvalidAddress,                 /*The device has a different address than the one you want to connect to*/
}MPU6050_Result;


/*Accelerometer operating ranges in g; 1g = 9.81 m/s2*/
typedef enum
{
    MPU6050_Accelerometer_range_2g =   0x00,   /*Range +-2g */
    MPU6050_Accelerometer_range_4g =   0x01,   /*Range +-4g */
    MPU6050_Accelerometer_range_8g =   0x02,   /*Range +-8g */
    MPU6050_Accelerometer_range_16g=   0x03    /*Range +-16g */
}MPU6050_Accelerometer_Range;

/*Gyroscope range in °/s */
typedef enum
{
    MPU6050_Gyroscope_range_250  =   0x00,   /*Range +- 250 °/s */
    MPU6050_Gyroscope_range_500,             /*Range +- 500 °/s */
    MPU6050_Gyroscope_range_1000,            /*Range +- 1000 °/s */
    MPU6050_Gyroscope_range_2000             /*Range +- 2000 °/s */
}MPU6050_Gyroscope_Range;

/*Union that stores the readings made by the accelerometer or gyroscope. The total reading is stored in two 8-bit registers.*/
/*union allows different data types to be stored in the same memory location*/
typedef union {
   int16_t TotalReading;                 /*Stores the total reading, that is, the 16 bits*/
   struct {
           uint8_t LSB;                  /*Modifies the least significant 8 bits of the reading*/
           uint8_t MSB;                  /*Modifies the most significant 8 bits of the reading*/
   } HalfReading;
}Reading;

/*Structure where the data referring to the measurements are stored*/
typedef struct
{
   uint8_t Address;
   float Accelerometer_Multiplier;
   float Gyroscope_Multiplier;
   Reading TemperatureReading;

   Reading Acceleration_X;
   Reading Acceleration_Y;
   Reading Acceleration_Z;

   Reading Gyroscope_X;
   Reading Gyroscope_Y;
   Reading Gyroscope_Z;
   float Temperature;
}MPU6050_Typedef;



/*----------------------------MPU_6050_Functions-----------------------*/

void INIT_MPU6050_I2C(void);

/*Function to initialize and configure the device*/
MPU6050_Result MPU6050_Init(IfxI2c_I2c_Device *I2C_Handler,
        MPU6050_Typedef *MPU6050_Values,
        MPU6050_Accelerometer_Range Accelerometer_Range,
        MPU6050_Gyroscope_Range Gyroscope_Range);

/*Function to configure the sample rate*/
MPU6050_Result MPU6050_SampleRateConfig(IfxI2c_I2c_Device *I2C_Handler,
        MPU6050_Typedef *MPU6050_Values,
        uint8_t SampleRate);

/*Function to configure the accelerometer*/
MPU6050_Result MPU6050_AcelerometroConfig(IfxI2c_I2c_Device *I2C_Handler,
        MPU6050_Typedef *MPU6050_Values,
        MPU6050_Accelerometer_Range Accelerometer_Range);

/*Function to configure the gyroscope*/
MPU6050_Result MPU6050_GyroscopeConfig(IfxI2c_I2c_Device *I2C_Handler,
        MPU6050_Typedef *MPU6050_Values,
        MPU6050_Gyroscope_Range Gyroscope_Range);

/*Function to receive accelerometer readings*/
MPU6050_Result MPU6050_READ_ACCELEROMETER(IfxI2c_I2c_Device *I2C_Handler, MPU6050_Typedef *MPU6050_Values);

/*Function to receive gyroscope readings*/
MPU6050_Result MPU6050_READ_GYROSCOPE(IfxI2c_I2c_Device *I2C_Handler, MPU6050_Typedef *MPU6050_Values);

/*Function to receive temperature readings*/
MPU6050_Result MPU6050_READ_TEMPERATURE(IfxI2c_I2c_Device *I2C_Handler, MPU6050_Typedef *MPU6050_Values);

/*Function to receive accelerometer, temperature and gyroscope readings at the same time*/
MPU6050_Result MPU6050_READ(IfxI2c_I2c_Device *I2C_Handler, MPU6050_Typedef *MPU6050_Values);


#endif /* MPU6050_H_ */
