#include "main.h"
#include "CmdSerial.h"
#include "motor.h"


char chcmd[64];


uint16_t cmd_i = 0;

extern float ref_angle[3];
extern Motor motor[3];
void set_angle(Motor motor[], float32_t target_angle[]);

void CmdProcessing(UART_HandleTypeDef* huart)
{
	static char mode;
	static uint16_t cmd_start = 0;

	static float fPm1, fPm2, fPm3;
	static int iPm1, iPm2, iPm3;

	if(chcmd[cmd_i] == '\n')
	{
		chcmd[cmd_i + 1] = 0;
		switch(chcmd[cmd_start])
		{
			case 'G':
				sscanf(chcmd + 1, "%f\n", &fPm3);
				ref_angle[0] = fPm3;
				ref_angle[1] = -fPm3;
				set_angle(motor, ref_angle);
				break;
			default:
				break;
		}
		cmd_i = 0;
	}
	else
	{
		cmd_i++;
	}
	HAL_UART_Receive_IT(huart, (uint8_t*)&chcmd[cmd_i], 1);
	
}




