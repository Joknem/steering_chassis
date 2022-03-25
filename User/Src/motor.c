#include "motor.h"
#include "bsp_can.h"
#include "tim.h"

extern int count[3];
uint16_t status[3];
extern UART_HandleTypeDef huart1;
extern int uart_printf(char *fmt ,...);
float32_t offset[3];

volatile float32_t last_angle[3];
int init_circle[3];
float32_t init_angle[3];
int init_ok;

arm_pid_instance_f32 speed_pid[3] = {
	{
		.Kp = 10.0,
		.Ki = 0.001,
		.Kd = 0.0
	},
	{
		.Kp = 10.0,
		.Ki = 0.001,
		.Kd = 10.5
	},
	{
		.Kp = 10.0,
		.Ki = 0.001,
		.Kd = 10.5
	}
};
arm_pid_instance_f32 angle_pid[3] = {
	{
		.Kp = 15.0,
		.Ki = 0.001,
		.Kd = 10.0
	},
	{
		.Kp = 15.0,
		.Ki = 0.001,
		.Kd = 5.0
	},
	{
		.Kp = 15.0,
		.Ki = 0.001,
		.Kd = 10.0
	}
};
void angle_detect(Motor *motor){
	motor->circle = get_chassis_motor_measure_point(motor->id)->circle;
} 
void angle_control(Motor motor[]){
	for(int i = 0;i < 3; i++){
		angle_detect(motor+i);
	}
	float angle_err[3];
	float angle_out[3];
	float speed_err[3];
	int16_t I_out[3];
	for(int i = 0; i < 3; i++){
	angle_err[i] = motor[i].target_angle - motor[i].absolute_angle;
	angle_out[i] = arm_pid_f32(angle_pid+i, angle_err[i]);
	speed_err[i] = angle_out[i] - motor[i].speed;
	I_out[i] = (int16_t)(arm_pid_f32(&speed_pid[i],speed_err[i]));
	I_out[i] = (I_out[i] > 10000) ? 10000 : (I_out[i] < -10000) ? -10000 : I_out[i];
	}
	CAN_cmd_chassis(I_out[0], I_out[1], I_out[2], 0);
}
void set_angle(Motor motor[], float32_t target_angle[]){
	for(int i = 0; i < 3; i++){
		angle_detect(motor+i);	
	}
	for(int i = 0; i < 3; i++){
		motor[i].target_angle = target_angle[i];
		last_angle[i] = (motor[i].circle * 360 + motor[i].angle - offset[i]) * ratio;	
	}
}

void set_speed(Motor *motor, float32_t speed){
	motor->target_speed = speed;
	}

float32_t f32_abs(float32_t num){
	arm_abs_f32(&num, &num, 1);
	return num;
}
void init_motor(Motor motor[]){	//init_motor之后，开始记录already_angle和absolute_angle
	init_ok = 1;
	HAL_Delay(5);
	for(int i = 0; i < 3; i++){
		init_angle[i] = motor[i].angle;
	}
}
void speed_control(Motor *motor){
	float speed_err[3];
	int I_out[3];
	for(int i=0; i < 3; i++){
		speed_err[i] = motor->target_speed - motor->speed;
		I_out[i] = arm_pid_f32(&speed_pid[i], speed_err[i]);
		I_out[i] = (I_out[i] > 10000) ? 10000 : (I_out[i] < -10000) ? -10000 : I_out[i];
	}
	CAN_cmd_chassis(I_out[0], I_out[1], I_out[2], 0);
}

