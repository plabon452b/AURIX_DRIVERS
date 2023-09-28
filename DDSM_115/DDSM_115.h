/*
 * DDSM_115.h
 *
 *  Created on: Sep 22, 2023
 *      Author: plabon
 */

#ifndef DDSM_115_H_
#define DDSM_115_H_
#include "Ifx_Types.h"
#include <stdint.h>

#define MAX_CURRENT             8
#define MIN_CURRENT             -8
#define MAX_SPEED               330
#define MIN_SPEED               -330
#define MAX_ANGLE               360
#define MIN_ANGLE               0


/*DDSM115 modes*/
typedef enum {
    CURRENT_LOOP = 1,
    SPEED_LOOP = 2,
    POSITION_LOOP = 3,
} ddsm115_mode_t;

/*DDSM115 protocols*/
typedef enum {
  DDSM115_PROTOCOL_V1 = 1,
  DDSM115_PROTOCOL_V2 = 2,
} ddsm115_protocol_t;

/*DDSM115 errors*/
typedef enum {
  DDSM115_TROUBLESHOOTING = 0x10,
  DDSM115_STALL_ERROR = 0x08,
  DDSM115_PHASE_OVERCURRENT_ERROR = 0x04,
  DDSM115_OVERCURRENT_ERROR = 0x02,
  DDSM115_SENSOR_ERROR = 0x01,
} ddsm115_error_t;

/*Structure where the received data from DDSM115 are stored*/
typedef struct{
        uint8_t id;
        ddsm115_mode_t mode;
        float current;
        int16_t speed;
        int16_t angle;
        uint8_t winding_temp;
        int16_t position;
        uint8_t error;
}DDSM_Typedef;



void DDSM_initSerial(void);
void DDSM_setID(uint8_t id);
uint8_t DDSM_getID();
void DDSM_setMode(DDSM_Typedef *wheel, ddsm115_mode_t mode);
boolean DDSM_getFeedback(DDSM_Typedef *wheel);
boolean DDSM_setBrakes(DDSM_Typedef *wheel);
boolean DDSM_setCurrent(DDSM_Typedef *wheel, float current);
boolean DDSM_setSpeed(DDSM_Typedef *wheel, int16_t speed, uint8_t acceleration_time);
boolean DDSM_setPosition(DDSM_Typedef *wheel, int16_t angle);

#endif /* DDSM_115_H_ */
