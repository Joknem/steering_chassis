#include "control.h"
#include "motor.h"
#include "bsp_can.h"
#include "main.h"


extern uint32_t channel_info[12];
extern Motor motor[3];
extern UART_HandleTypeDef huart1;
extern void uart_printf(UART_HandleTypeDef *huart, char *fmt ,...);
extern volatile uint16_t status[3];
extern float input_vel[3];
extern OdriveAxisSetState_t odrive_set_axis[3];
float last_angle1;
float temp = 0.0f;
float target_angle1 = 0.0f;
float last_info = 1499.0f;
float target_speed[3];
float chassis_pos;
float32_t ref_angle[3];
extern int mode_flag;	//1:速度控制模式   0:位置控制模式


void func1(uint32_t r_info1, uint32_t r_info2){ //r_info1为左右信息，r_info2为上下信息
	if(r_info1 && r_info2){
		if(f32_abs((float)r_info1 - 1500) < 30 && f32_abs((float)r_info2 - 1500) < 30){
			for(int i = 0; i < 3; i++){
				ref_angle[i] = 0.0;
				input_vel[i] = 0.0;
			}
		}
		else if(f32_abs((float)r_info1 - 1500) > 30 && f32_abs((float)r_info2 - 1500) < 30){
			if((float)r_info1 - 1500 > 0){
				for(int i = 0; i < 3; i++){
					ref_angle[i] = get_chassis_pos(90);
					input_vel[i] = (float)(r_info1 - 1500) / 6.25;
			}
		}
			else{
				for(int i = 0; i < 3; i++){
				ref_angle[i] = get_chassis_pos(-90); 
				input_vel[i] = (float)(1500 - r_info1) / 6.25;
			}
		}
	}
		else if(f32_abs((float)r_info1 - 1500) < 30 && f32_abs((float)r_info2 - 1500) > 30){
			if((float)r_info2 - 1500 > 0){
				for(int i = 0; i < 3; i++){
					ref_angle[i] = 0.0;
					input_vel[i] = (float)(r_info2 - 1500) / 6.25;
		}
	}
			else{
				for(int i = 0; i < 3; i++){
					ref_angle[i] = 0.0;
					input_vel[i] = -(float)(1500 - r_info2) / 6.25;
		}
			}
		}
		else{
			if((float)r_info2 - 1500 > 0){
				for(int i = 0; i < 3; i++){
					ref_angle[i] = get_chassis_pos(atan(((float)r_info1 - 1500) / ((float)r_info2 - 1500))) * 180 / PI;
					input_vel[i] = sqrt(pow(f32_abs((float)r_info1 - 1500), 2.0) + pow((float)r_info2 - 1500, 2.0))/ 8.83;
			}
			}
			else{
				for(int i = 0; i < 3; i++){
					ref_angle[i] = get_chassis_pos(atan(((float)r_info1 - 1500) / ((float)r_info2 - 1500))) * 180 / PI;
					input_vel[i] = -sqrt(pow(f32_abs((float)r_info1 - 1500), 2.0) + pow((float)r_info2 - 1500, 2.0))/ 8.83;
			}
			}
		}
		}
	}

	
void cmd_all(float32_t ref_angle[], float input_vel[]){
	for(int i = 0; i<3; i++){
		motor[i].target_angle = get_chassis_pos(ref_angle[i]*120);
		odrive_set_axis[i].input_vel = input_vel[i];
	}
	odrv_write_msg(AXIS_0, MSG_SET_INPUT_VEL);
	odrv_write_msg(AXIS_1, MSG_SET_INPUT_VEL);
	odrv_write_msg(AXIS_2, MSG_SET_INPUT_VEL);
}
void chassis_init(void){
	mode_flag = 1;
	for(int i = 0; i < 3; i++){
		target_speed[i] = 1500.0f;
		set_speed(motor+i, target_speed[i]);
	}
	angle_detect(motor);
} 

float get_chassis_pos(float angle){
		chassis_pos = angle;	//对电机绝对角度进行处理，转化成在底盘上的角度
		return chassis_pos;
}