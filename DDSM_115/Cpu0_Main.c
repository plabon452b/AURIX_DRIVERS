#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "UART.h"
#include "DDSM_115.h"
#include <Bsp.h>
#include <stdint.h>

IfxCpu_syncEvent g_cpuSyncEvent = 0;




void core0_main(void)
{
    IfxCpu_enableInterrupts();
    
    /* !!WATCHDOG0 AND SAFETY WATCHDOG ARE DISABLED HERE!!
     * Enable the watchdogs  service them periodically if it is required
     */
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());
    
    /* Wait for CPU sync event */
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);
    
    /* Initialize the UART communication */
    initSerialInterface();
    DDSM_initSerial();

    DDSM_Typedef wheel1;
    wheel1.id = DDSM_getID();


    int16_t speed = 20;

    DDSM_setMode(&wheel1, SPEED_LOOP);
    DDSM_setSpeed(&wheel1, speed, 10);

    while(1)
    {
        DDSM_getFeedback(&wheel1);
        print("id: %x  ",wheel1.id);
        print("mode: %d  ",wheel1.mode);
        print("current: %f  ",wheel1.current);
        print("speed: %d  ",wheel1.speed);
        print("error: %d ",wheel1.error);
        print("winding_temp: %d ",wheel1.winding_temp);
        println("angle: %d ",wheel1.angle);

        waitTime(IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, 1000));
    }
}


