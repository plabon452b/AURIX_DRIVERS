#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "MPU6050.h"
#include "UART.h"
#include "IfxI2c_I2c.h"

static void i2c_init(void);
static void i2c_Device_init(void);

IfxCpu_syncEvent g_cpuSyncEvent = 0;

//MPU6050 I2C
IfxI2c_I2c I2C_Handler;
IfxI2c_I2c_Config I2C_ConfigHandler;
IfxI2c_I2c_Device MPU6050_I2C;
IfxI2c_I2c_deviceConfig MPU6050_I2C_Config;
MPU6050_Typedef MPU6050;



void core0_main(void)
{
    IfxCpu_enableInterrupts();
    
    /* !!WATCHDOG0 AND SAFETY WATCHDOG ARE DISABLED HERE!!
     * Enable the watchdogs and service them periodically if it is required
     */
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());
    
    /* Wait for CPU sync event */
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);
    
    /* Initialize the UART communication */
    initSerialInterface();

    i2c_init();
    i2c_Device_init();

    MPU6050_Init(&MPU6050_I2C, &MPU6050, MPU6050_Accelerometer_range_4g, MPU6050_Gyroscope_range_250);

    while(1)
    {
        MPU6050_READ(&MPU6050_I2C, &MPU6050);
        print("TEMP %.2f",MPU6050.Temperature);
        println(" ACC_X: %.2lf ACC_Y: %.2lf ACC_Z: %.2lf  GY_X: %.2lf GY_Y: %.2lf GY_Z: %.2lf",
                MPU6050.Acceleration_X.TotalReading*MPU6050.Accelerometer_Multiplier,
                MPU6050.Acceleration_Y.TotalReading*MPU6050.Accelerometer_Multiplier,
                MPU6050.Acceleration_Z.TotalReading*MPU6050.Accelerometer_Multiplier,
                MPU6050.Gyroscope_X.TotalReading*MPU6050.Gyroscope_Multiplier,
                MPU6050.Gyroscope_Y.TotalReading*MPU6050.Gyroscope_Multiplier,
                MPU6050.Gyroscope_Z.TotalReading*MPU6050.Gyroscope_Multiplier
        );
        for(int i=0;i<50000000;i++){}
    }
}


static void i2c_init(void)
{
    IfxI2c_I2c_initConfig(&I2C_ConfigHandler, &MODULE_I2C0);

    static IfxI2c_Pins Pins = {
            &IfxI2c0_SCL_P13_1_INOUT,
            &IfxI2c0_SDA_P13_2_INOUT,
            IfxPort_PadDriver_cmosAutomotiveSpeed1
    };

    I2C_ConfigHandler.pins = &Pins;
    I2C_ConfigHandler.baudrate = MPU6050_BAUDRATE;

    IfxI2c_I2c_initModule(&I2C_Handler, &I2C_ConfigHandler);
}

static void i2c_Device_init(void)
{
    IfxI2c_I2c_initDeviceConfig(&MPU6050_I2C_Config, &I2C_Handler);

    MPU6050_I2C.deviceAddress = MPU_I2C_ADDR;

    IfxI2c_I2c_initDevice(&MPU6050_I2C, &MPU6050_I2C_Config);
}
