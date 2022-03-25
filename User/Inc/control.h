#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "main.h"
#include "motor.h"
#include "gpio.h"
extern Motor motor[3];

#define motor1_init_flag HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9) //0为初始化完成
#define motor2_init_flag HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11)
#define motor3_init_flag HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)

void turn(uint32_t r_info);
void func(uint32_t r_info);
void func1(uint32_t r_info, uint32_t r_info2);
void chassis_init(void);
float get_chassis_pos(float angle);   //得到三个轮的角度。在-90°到90°之间
void pos_control(int i, float angle);
void cmd_all(float32_t ref_angle[], float input_vel[]);
#endif