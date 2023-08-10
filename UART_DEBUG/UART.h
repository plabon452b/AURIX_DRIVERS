#ifndef UART_H_
#define UART_H_

#include "Ifx_Types.h"

/*--------Macros--------*/
#define ENDLINE     "\n\r"

/*--------Function Prototypes----------*/
void initSerialInterface(void);
void print(pchar format, ...);
void println(pchar format, ...);

#endif /* UART_H_ */
