#ifndef __CMDSERIAL_H
#define __CMDSERIAL_H

#include "main.h"
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>




#define FuncType    10
#define TypeNum     10

// Error Massage
#define EM_NOTACHIEVABLE    1

// Function return Massage
#define RM_COMPLETE         1



void CmdProcessing(UART_HandleTypeDef *huart);


#endif
