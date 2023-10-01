#include "MPU6050.h"
#include "stdint.h"
#include "IfxI2c_I2c.h"
#include "UART.h"

#define ADDRESS_LEN   1
#define pi 3.14159
void INIT_MPU6050_I2C(void){

}

MPU6050_Result MPU6050_Init(IfxI2c_I2c_Device *I2C_Handler,
        MPU6050_Typedef *MPU6050_Values,
        MPU6050_Accelerometer_Range Accelerometer_Range,
        MPU6050_Gyroscope_Range Gyroscope_Range)
{
    /*Set Device Address*/
    MPU6050_Values->Address = MPU_I2C_ADDR;
    I2C_Handler->deviceAddress = MPU6050_Values->Address;
    uint8_t RegisterAddress[ADDRESS_LEN] = {WHO_AM_I};
    uint8_t data[2] = {0,0};

    /*check device*/
    while(IfxI2c_I2c_write(I2C_Handler, RegisterAddress, ADDRESS_LEN) == IfxI2c_I2c_Status_nak);
    while(IfxI2c_I2c_read(I2C_Handler, &data[0], 1) == IfxI2c_I2c_Status_nak);
    println("WHO AM I REG RETURNED: %0x",data[0]);
    if(data[0] != MPU_I_AM ){
        return MPU6050_InvalidAddress;
    }

    /*Wake up MPU6050*/
    uint8_t ToWrite[2] = {PWR_MGMT_1,0x00};
    while(IfxI2c_I2c_write(I2C_Handler, ToWrite, 2) == IfxI2c_I2c_Status_nak);

    /*Sampling rate configuration at 1KHz*/
    MPU6050_SampleRateConfig(I2C_Handler, MPU6050_Values, MPU6050_SampleRate_1KHz);

    /*Accelerometer configuration*/
    MPU6050_AcelerometroConfig(I2C_Handler, MPU6050_Values, Accelerometer_Range);

    /*Gyroscope configuration*/
    MPU6050_GyroscopeConfig(I2C_Handler, MPU6050_Values, Gyroscope_Range);

    return MPU6050_OK;
}

MPU6050_Result MPU6050_SampleRateConfig(IfxI2c_I2c_Device *I2C_Handler,
        MPU6050_Typedef *MPU6050_Values,
        uint8_t SampleRate)
{
    uint8_t ToWrite[2] = {SMPLRT_DIV, SampleRate};
    if(IfxI2c_I2c_write(I2C_Handler, ToWrite, 2) != IfxI2c_I2c_Status_ok)
    {
        return MPU6050_ERROR;
    }
    return MPU6050_OK;
}


MPU6050_Result MPU6050_AcelerometroConfig(IfxI2c_I2c_Device *I2C_Handler,
        MPU6050_Typedef *MPU6050_Values,
        MPU6050_Accelerometer_Range Accelerometer_Range)
{
    uint8_t data[2] = {ACCEL_CONFIG, 0x00};

    while(IfxI2c_I2c_write(I2C_Handler, &data[0], ADDRESS_LEN) == IfxI2c_I2c_Status_nak);
    while(IfxI2c_I2c_read(I2C_Handler, &data[1], 1) == IfxI2c_I2c_Status_nak);
    data[1] = (data[1] & 0xE7) | (uint8_t)(Accelerometer_Range << 3);
    while(IfxI2c_I2c_write(I2C_Handler, data, 2) == IfxI2c_I2c_Status_nak);

    /*Set Sensitivity Multiplier*/
    switch (Accelerometer_Range)
    {
        case MPU6050_Accelerometer_range_2g:
            MPU6050_Values->Accelerometer_Multiplier = (float)1 / MPU6050_ACCELEROMETER_SENSITIVITY_2G;
            break;
        case MPU6050_Accelerometer_range_4g:
            MPU6050_Values->Accelerometer_Multiplier = (float)1 / MPU6050_ACCELEROMETER_SENSITIVITY_4G;
            break;
        case MPU6050_Accelerometer_range_8g:
            MPU6050_Values->Accelerometer_Multiplier = (float)1 / MPU6050_ACCELEROMETER_SENSITIVITY_8G;
            break;
        case MPU6050_Accelerometer_range_16g:
            MPU6050_Values->Accelerometer_Multiplier = (float)1 / MPU6050_ACCELEROMETER_SENSITIVITY_16G;
            break;
    }

    return MPU6050_OK;
}

MPU6050_Result MPU6050_GyroscopeConfig(IfxI2c_I2c_Device *I2C_Handler,
        MPU6050_Typedef *MPU6050_Values,
        MPU6050_Gyroscope_Range Gyroscope_Range)
{
    uint8_t data[] = {GYRO_CONFIG, 0x00};

    while(IfxI2c_I2c_write(I2C_Handler, &data[0], ADDRESS_LEN) == IfxI2c_I2c_Status_nak);
    while(IfxI2c_I2c_read(I2C_Handler, &data[1], 1) == IfxI2c_I2c_Status_nak);
    data[1] = (data[1] & 0xE7) | (uint8_t)(Gyroscope_Range<< 3);
    while(IfxI2c_I2c_write(I2C_Handler, data, 2) == IfxI2c_I2c_Status_nak);

    /*Set Sensitivity Multiplier*/
    switch (Gyroscope_Range)
    {
        case MPU6050_Gyroscope_range_250:
            MPU6050_Values->Gyroscope_Multiplier = (float)1 / MPU6050_GYROSCOPE_SENSITIVITY_250;
            break;
        case MPU6050_Gyroscope_range_500:
            MPU6050_Values->Gyroscope_Multiplier = (float)1 / MPU6050_GYROSCOPE_SENSITIVITY_500;
            break;
        case MPU6050_Gyroscope_range_1000:
            MPU6050_Values->Gyroscope_Multiplier = (float)1 / MPU6050_GYROSCOPE_SENSITIVITY_1000;
            break;
        case MPU6050_Gyroscope_range_2000:
            MPU6050_Values->Gyroscope_Multiplier = (float)1 / MPU6050_GYROSCOPE_SENSITIVITY_2000;
            break;
    }

    return MPU6050_OK;

}

MPU6050_Result MPU6050_READ_ACCELEROMETER(IfxI2c_I2c_Device *I2C_Handler, MPU6050_Typedef *MPU6050_Values)
{
    uint8_t data[7] = {ACCEL_XOUT_H, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    while(IfxI2c_I2c_write(I2C_Handler, &data[0], 1) == IfxI2c_I2c_Status_nak);
    while(IfxI2c_I2c_read(I2C_Handler, &data[1], 6) == IfxI2c_I2c_Status_nak);

    /*Save data in structure*/
    MPU6050_Values->Acceleration_X.HalfReading.MSB = data[1];
    MPU6050_Values->Acceleration_X.HalfReading.LSB = data[2];
    MPU6050_Values->Acceleration_Y.HalfReading.MSB = data[3];
    MPU6050_Values->Acceleration_Y.HalfReading.LSB = data[4];
    MPU6050_Values->Acceleration_Z.HalfReading.MSB = data[5];
    MPU6050_Values->Acceleration_Z.HalfReading.LSB = data[6];

    return MPU6050_OK;
}

MPU6050_Result MPU6050_READ_GYROSCOPE(IfxI2c_I2c_Device *I2C_Handler, MPU6050_Typedef *MPU6050_Values)
{
    uint8_t data[7] = {GYRO_XOUT_H , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    while(IfxI2c_I2c_write(I2C_Handler, &data[0], 1) == IfxI2c_I2c_Status_nak);
    while(IfxI2c_I2c_read(I2C_Handler, &data[1], 6) == IfxI2c_I2c_Status_nak);

    /*Save data in structure*/
    MPU6050_Values->Gyroscope_X.HalfReading.MSB = data[1];
    MPU6050_Values->Gyroscope_X.HalfReading.LSB = data[2];
    MPU6050_Values->Gyroscope_Y.HalfReading.MSB = data[3];
    MPU6050_Values->Gyroscope_Y.HalfReading.LSB = data[4];
    MPU6050_Values->Gyroscope_Z.HalfReading.MSB = data[5];
    MPU6050_Values->Gyroscope_Z.HalfReading.LSB = data[6];

    return MPU6050_OK;
}

MPU6050_Result MPU6050_READ_TEMPERATURE(IfxI2c_I2c_Device *I2C_Handler, MPU6050_Typedef *MPU6050_Values)
{
    uint8_t data[3] = {TEMP_OUT_H, 0x00, 0x00};

    while(IfxI2c_I2c_write(I2C_Handler, &data[0], 1) == IfxI2c_I2c_Status_nak);
    while(IfxI2c_I2c_read(I2C_Handler, &data[1], 2) == IfxI2c_I2c_Status_nak);

    /*Save raw data in structure*/
    MPU6050_Values->TemperatureReading.HalfReading.MSB = data[1];
    MPU6050_Values->TemperatureReading.HalfReading.LSB = data[2];

    /*Conversion to °C*/
    MPU6050_Values->Temperature = (float)((((float)MPU6050_Values->TemperatureReading.TotalReading)/340) + 36.53);

    return MPU6050_OK;
}

MPU6050_Result MPU6050_READ(IfxI2c_I2c_Device *I2C_Handler, MPU6050_Typedef *MPU6050_Values)
{
    uint8_t data[15] = {ACCEL_XOUT_H, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    while(IfxI2c_I2c_write(I2C_Handler, &data[0], 1) == IfxI2c_I2c_Status_nak);
    while(IfxI2c_I2c_read(I2C_Handler, &data[1], 14) == IfxI2c_I2c_Status_nak);

    /*Save data in structure*/
    MPU6050_Values->Acceleration_X.HalfReading.MSB = data[1];
    MPU6050_Values->Acceleration_X.HalfReading.LSB = data[2];
    MPU6050_Values->Acceleration_Y.HalfReading.MSB = data[3];
    MPU6050_Values->Acceleration_Y.HalfReading.LSB = data[4];
    MPU6050_Values->Acceleration_Z.HalfReading.MSB = data[5];
    MPU6050_Values->Acceleration_Z.HalfReading.LSB = data[6];
    MPU6050_Values->TemperatureReading.HalfReading.MSB = data[7];
    MPU6050_Values->TemperatureReading.HalfReading.LSB = data[8];
    MPU6050_Values->Gyroscope_X.HalfReading.MSB = data[9];
    MPU6050_Values->Gyroscope_X.HalfReading.LSB = data[10];
    MPU6050_Values->Gyroscope_Y.HalfReading.MSB = data[11];
    MPU6050_Values->Gyroscope_Y.HalfReading.LSB = data[12];
    MPU6050_Values->Gyroscope_Z.HalfReading.MSB = data[13];
    MPU6050_Values->Gyroscope_Z.HalfReading.LSB = data[14];

    MPU6050_Values->Temperature = (float)((((float)MPU6050_Values->TemperatureReading.TotalReading)/340) + 36.53);

    return MPU6050_OK;
}

