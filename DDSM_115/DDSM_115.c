/*
 * DDSM_115.c
 *
 *  Created on: Sep 22, 2023
 *      Author: plabon
 */

#include "DDSM_115.h"

#include "Ifx_Types.h"
#include "IfxAsclin_Asc.h"
#include "Ifx_Shell.h"
#include "Ifx_Console.h"
#include "IfxPort.h"
#include <stdio.h>
#include <stdarg.h>
#include "CRC.h"
#include "UART.h"
#include "Bsp.h"
#include <math.h>

/*-----------Macros--------------*/
/* Communication parameters */

#define ISR_PRIORITY_ASCLIN_TX      3                                       /* Priority for interrupt ISR Transmit  */
#define ISR_PRIORITY_ASCLIN_RX      2                                       /* Priority for interrupt ISR Receive   */
#define ISR_PRIORITY_ASCLIN_ER      6                                       /* Priority for interrupt ISR Errors    */
#define ASC_TX_BUFFER_SIZE          256                                     /* Define the TX buffer size in byte    */
#define ASC_RX_BUFFER_SIZE          256                                     /* Define the RX buffer size in byte    */
#define ASC_BAUDRATE                115200                                  /* Define the UART baud rate            */




/*-----------Global variables-------------*/
IfxStdIf_DPipe  ddsm_ascStandardInterface;                                     /* Standard interface object            */
IfxAsclin_Asc   ddsm_asclin;                                                   /* ASCLIN module object                 */
uint8 ddsm_uartTxBuffer[ASC_TX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];
uint8 ddsm_uartRxBuffer[ASC_RX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];

//uint8_t commandBuffer[10];
//uint8_t responseBuffer[10];



/* Macro to define Interrupt Service Routine */
IFX_INTERRUPT(asc0TxISR, 0, ISR_PRIORITY_ASCLIN_TX);

void asc0TxISR(void)
{
    IfxStdIf_DPipe_onTransmit(&ddsm_ascStandardInterface);
}

IFX_INTERRUPT(asc0RxISR, 0, ISR_PRIORITY_ASCLIN_RX);

void asc0RxISR(void)
{
    IfxStdIf_DPipe_onReceive(&ddsm_ascStandardInterface);
}

IFX_INTERRUPT(asc0ErrISR, 0, ISR_PRIORITY_ASCLIN_ER);

void asc0ErrISR(void)
{
    IfxStdIf_DPipe_onError(&ddsm_ascStandardInterface);
}



/* Function to initialize ASCLIN module */
void DDSM_initSerial(void)
{
    IfxAsclin_Asc_Config ascConf;

    /* Set default configurations */
    IfxAsclin_Asc_initModuleConfig(&ascConf, &MODULE_ASCLIN0); /* Initialize the structure with default values      */

    /* Set the desired baud rate */
    ascConf.baudrate.baudrate = ASC_BAUDRATE;                                   /* Set the baud rate in bit/s       */
    ascConf.baudrate.oversampling = IfxAsclin_OversamplingFactor_16;            /* Set the oversampling factor      */

    /* Configure the sampling mode */
    ascConf.bitTiming.medianFilter = IfxAsclin_SamplesPerBit_three;             /* Set the number of samples per bit*/
    ascConf.bitTiming.samplePointPosition = IfxAsclin_SamplePointPosition_8;    /* Set the first sample position    */

    /* ISR priorities and interrupt target */
    ascConf.interrupt.txPriority = ISR_PRIORITY_ASCLIN_TX;  /* Set the interrupt priority for TX events             */
    ascConf.interrupt.rxPriority = ISR_PRIORITY_ASCLIN_RX;  /* Set the interrupt priority for RX events             */
    ascConf.interrupt.erPriority = ISR_PRIORITY_ASCLIN_ER;  /* Set the interrupt priority for Error events          */
    ascConf.interrupt.typeOfService = IfxSrc_Tos_cpu0;

    /* Pin configuration */
    const IfxAsclin_Asc_Pins pins = {
               .cts        = NULL_PTR,                         /* CTS pin not used                                     */
               .ctsMode    = IfxPort_InputMode_pullUp,
               .rx         = &IfxAsclin0_RXB_P15_3_IN ,        /* Select the pin for RX  0     */
               .rxMode     = IfxPort_InputMode_pullUp,         /* RX pin                                               */
               .rts        = NULL_PTR,                         /* RTS pin not used                                     */
               .rtsMode    = IfxPort_OutputMode_pushPull,
               .tx         = &IfxAsclin0_TX_P15_2_OUT,         /* Select the pin for TX 0     */
               .txMode     = IfxPort_OutputMode_pushPull,      /* TX pin                                               */
               .pinDriver  = IfxPort_PadDriver_cmosAutomotiveSpeed1
       };
    ascConf.pins = &pins;

    /* FIFO buffers configuration */
    ascConf.txBuffer = ddsm_uartTxBuffer;                      /* Set the transmission buffer                          */
    ascConf.txBufferSize = ASC_TX_BUFFER_SIZE;              /* Set the transmission buffer size                     */
    ascConf.rxBuffer = ddsm_uartRxBuffer;                      /* Set the receiving buffer                             */
    ascConf.rxBufferSize = ASC_RX_BUFFER_SIZE;              /* Set the receiving buffer size                        */

    /* Init ASCLIN module */
    IfxAsclin_Asc_initModule(&ddsm_asclin, &ascConf);          /* Initialize the module with the given configuration   */

    /* Initialize the Standard Interface */
    IfxAsclin_Asc_stdIfDPipeInit(&ddsm_ascStandardInterface, &ddsm_asclin);


}
/* prototypes */
void DDSM_write(void *data, uint8_t length);
void DDSM_send(uint8_t *commandBuffer);
boolean DDSM_receive(uint8_t *data, uint8_t length);
void parse(DDSM_Typedef *wheel, ddsm115_protocol_t protocol, uint8_t *responseBuffer);


/* Write to DDSM115 serial without CRC */
void DDSM_write(void *data, uint8_t length)
{
    Ifx_SizeT ifx_len = (Ifx_SizeT) length;
    IfxStdIf_DPipe_write(&ddsm_ascStandardInterface, data ,&ifx_len, TIME_INFINITE);
}


/*send commands to DDSM115 along with calculated CRC */
void DDSM_send(uint8_t *commandBuffer){
    commandBuffer[9] = crc8(commandBuffer, 9);
    DDSM_write(commandBuffer,10);
}

/* Read from DDSM115 serial and check CRC */
boolean DDSM_receive(uint8_t *data, uint8_t length)
{
    Ifx_SizeT ifx_len = (Ifx_SizeT) length;
    if(IfxStdIf_DPipe_read(&ddsm_ascStandardInterface, data ,&ifx_len, IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, 10))){
       //print("CRC %x",crc8(data,9));
       return crc8(data,9)==data[9];
    }
    return 0;
}


/* Set ID to a single wheel connected to the bus */
void DDSM_setID(uint8_t id){
    uint8_t cmdBuf[] = {0xAA, 0x55, 0x53, id, 0, 0, 0, 0, 0, 0};

    for(int i = 0; i < 5; i++){
        DDSM_write(cmdBuf,10);
        waitTime(IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, 10));
    }
}
/* Get ID of a single wheel connected to the bus */
uint8_t DDSM_getID(){
    uint8_t rcvBuf[10];
    uint8_t cmdBuf[] = {0xC8, 0x64, 0, 0, 0, 0, 0, 0, 0, 0};
    DDSM_send(cmdBuf);
    if(DDSM_receive(rcvBuf,10)){
        return rcvBuf[0];
    }
    return 0;
}

/* Set motor mode */
void DDSM_setMode(DDSM_Typedef *wheel, ddsm115_mode_t mode){
    uint8_t cmdBuf[] = {wheel->id, 0xA0, 0, 0, 0, 0, 0, 0, 0, mode};
    DDSM_write(cmdBuf,10);
}

/* Get motor feedback and parse through protocol v2 */
boolean DDSM_getFeedback(DDSM_Typedef *wheel){
    uint8_t rcvBuf[10];
    uint8_t cmdBuf[] = {wheel->id, 0x74, 0, 0, 0, 0, 0, 0, 0, 0};
    DDSM_send(cmdBuf);
    if(DDSM_receive(rcvBuf,10)){
        parse(wheel,DDSM115_PROTOCOL_V2,rcvBuf);
        return 1;
    }
    return 0;
}

void parse(DDSM_Typedef *wheel, ddsm115_protocol_t protocol, uint8_t *responseBuffer){

    if(wheel->id == responseBuffer[0]){

        wheel->mode = (ddsm115_mode_t)responseBuffer[1];
        uint16_t curr = (uint16_t)(responseBuffer[2]) << 8 | (uint16_t)(responseBuffer[3]);
        //In current loop mode -32767 to 32767 corresponds to the range -8A to 8A torque current
        int current = curr;
        if (current  > 32767){ current -= 0xFFFF; current--; }
        if (current >= 0) {
            wheel->current = (float)current * (float)MAX_CURRENT / 32767.0;
        } else {
            wheel->current = (float)current * (float)MIN_CURRENT / -32767.0;
        }

        uint16_t spd = (uint16_t)(responseBuffer[4]) << 8 | (uint16_t)(responseBuffer[5]);
        //In speed loop mode -32767 to 32767 corresponds to -330 to 330 RPM
        int16_t speed = spd;
        if (speed  > MAX_SPEED){ speed -= 0xFFFF; speed--; }
        wheel->speed = speed;

        if (protocol == DDSM115_PROTOCOL_V1){
            uint16_t pos = (uint16_t)(responseBuffer[6]) << 8 | (uint16_t)(responseBuffer[7]);
            int16_t position = pos;
            if (position  > 32767){ position -= 0xFFFF; position--; }
            if (position >= 0) {
                wheel->angle = round((float)position * (float)MAX_ANGLE / 32767.0);
            } else {
                wheel->angle = round((float)position * (float)MIN_ANGLE / -32767.0);
            }
        }

        if (protocol == DDSM115_PROTOCOL_V2){
            wheel->winding_temp = responseBuffer[6];
            wheel->angle = round((float)responseBuffer[7] * (float)MAX_ANGLE / 255.0);
        }

        wheel->error = responseBuffer[8];
   }
}

boolean DDSM_setBrakes(DDSM_Typedef *wheel){
    uint8_t rcvBuf[10];
    uint8_t cmdBuf[] = {wheel->id, 0x64, 0, 0, 0, 0, 0, 0xff, 0, 0};

    DDSM_send(cmdBuf);
        if(DDSM_receive(rcvBuf,10)){
            parse(wheel,DDSM115_PROTOCOL_V1,rcvBuf);
            return 1;
        }
        return 0;
}

boolean DDSM_setCurrent(DDSM_Typedef *wheel, float current){
    if (current > MAX_CURRENT) current = MAX_CURRENT;
    if (current < MIN_CURRENT) current = MIN_CURRENT;
    uint16_t currentRecalc = (uint16_t)(round(abs(current) * 32767.0 / (float)MAX_CURRENT));
    if (current < 0 && currentRecalc != 0) currentRecalc = 0xFFFF - currentRecalc + 1;
    uint8_t currentHighByte = (uint8_t)(currentRecalc >> 8) & 0xFF;
    uint8_t currentLowByte = (uint8_t)(currentRecalc) & 0xFF;
    uint8_t rcvBuf[10];
    uint8_t cmdBuf[] = {wheel->id, 0x64, currentHighByte, currentLowByte, 0, 0, 0, 0, 0, 0};

    DDSM_send(cmdBuf);
       if(DDSM_receive(rcvBuf,10)){
           parse(wheel,DDSM115_PROTOCOL_V1,rcvBuf);
           return 1;
       }
       return 0;
}

/* set motor speed */
/* acceleration_time: The acceleration time per 1 RPM (unit: 0.1ms) | when set to 1 ,the acceleration time
 * per 1 RPM is 0.1ms | When set to 10 the acceleration time per 1 RPM is 10*0.1ms = 1ms. when set to 0 default is 1*/
boolean DDSM_setSpeed(DDSM_Typedef *wheel, int16_t speed, uint8_t acceleration_time){
    if (speed > MAX_SPEED) speed = MAX_SPEED;
    if (speed < MIN_SPEED) speed = MIN_SPEED;
    uint16_t speedRecalc = abs(speed);
    if (speed < 0 && speedRecalc != 0) speedRecalc = 0xFFFF - speedRecalc + 1;
    uint8_t speedHighByte = (uint8_t)(speedRecalc >> 8) & 0xFF;
    uint8_t speedLowByte = (uint8_t)(speedRecalc) & 0xFF;
    uint8_t rcvBuf[10];
    uint8_t cmdBuf[] = {wheel->id, 0x64, speedHighByte, speedLowByte, 0, 0, acceleration_time, 0, 0, 0};
    DDSM_send(cmdBuf);
       if(DDSM_receive(rcvBuf,10)){
           parse(wheel,DDSM115_PROTOCOL_V1,rcvBuf);
           return 1;
       }
       return 0;
}

boolean DDSM_setPosition(DDSM_Typedef *wheel, int16_t angle){
    if (angle > MAX_ANGLE) angle = MAX_ANGLE;
    if (angle < MIN_ANGLE) angle = MIN_ANGLE;
    uint16_t angleRecalc = (uint16_t)(round(abs((float)angle) * 32767.0 / (float)MAX_ANGLE));
    if (angle < 0 && angleRecalc != 0) angleRecalc = 0xFFFF - angleRecalc + 1;
    uint8_t angleHighByte = (uint8_t)(angleRecalc >> 8) & 0xFF;
    uint8_t angleLowByte = (uint8_t)(angleRecalc) & 0xFF;
    uint8_t rcvBuf[10];
    uint8_t cmdBuf[] = {wheel->id, 0x64, angleHighByte, angleLowByte, 0, 0, 0, 0, 0, 0};
    DDSM_send(cmdBuf);
       if(DDSM_receive(rcvBuf,10)){
           parse(wheel,DDSM115_PROTOCOL_V1,rcvBuf);
           return 1;
       }
       return 0;
}




