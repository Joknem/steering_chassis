#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "main.h"
#include "bsp_can.h"

#define ratio 19.1
typedef struct{
    uint8_t id;      
    float32_t speed;
    float32_t target_speed;
    float32_t angle;
    float32_t target_angle;
    float32_t already_angle;
		int16_t circle;
		float32_t Iout;
    float32_t absolute_angle;//指从开机到现在为止所转过的角度
}Motor;
// extern arm_pid_instance_f32 speed_pid, pos_pid;

void set_speed(Motor *motor, float32_t speed);
void speed_control(Motor motor[]);
void angle_control(Motor motor[]);
void new_angle_control(Motor motor[], int time);
void set_angle(Motor motor[], float32_t target_angle[]);
void angle_detect(Motor *motor);
void absolute_angle_control(Motor motor[]);
void init_motor(Motor motor[]);
void set_t_motor_speed(OdriveAxisSetState_t odrive_set_axis[3], float speed[]);
float32_t f32_abs(float32_t num);

#endif