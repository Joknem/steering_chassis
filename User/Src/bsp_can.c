#include "bsp_can.h"
#include "main.h"
#include "motor.h"

// FDCAN_RxHeaderTypeDef FDCAN1_RxHeader;
// FDCAN_TxHeaderTypeDef FDCAN1_TxHeader;
FDCAN_RxHeaderTypeDef FDCAN2_RxHeader;
FDCAN_TxHeaderTypeDef FDCAN2_TxHeader;

OdriveAxisSetState_t odrive_set_axis[3];

extern int uart_printf(char *fmt, ...);
extern Motor motor[3];
//extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern OdriveAxisSetState_t odrive_set_axis[3];

static motor_measure_t motor_chassis[7];
static FDCAN_TxHeaderTypeDef chassis_tx_message;
static FDCAN_RxHeaderTypeDef chassis_rx_message;
static uint8_t chassis_can_send_data[8];
//static motor_measure_t motor_chassis[7];
extern int motor_ecd;


typedef union _float_to_uint8_t
{
uint8_t raw[8];
float value[2];
uint32_t u32_data[2];
int32_t int32_data[2];
}float_to_uint8_t;

void fdcanfilter(void)
{
	FDCAN_FilterTypeDef FDCAN2_RXFilter;
    // FDCAN1_RXFilter.IdType=FDCAN_STANDARD_ID;
	// FDCAN1_RXFilter.FilterIndex=0;                                  
	// FDCAN1_RXFilter.FilterType=FDCAN_FILTER_MASK;
	// FDCAN1_RXFilter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;
	// FDCAN1_RXFilter.FilterID1=0x000;
	// FDCAN1_RXFilter.FilterID2=0x000;  
	FDCAN2_RXFilter.IdType=FDCAN_STANDARD_ID;
	FDCAN2_RXFilter.FilterIndex=0;                                  
	FDCAN2_RXFilter.FilterType=FDCAN_FILTER_MASK;
	FDCAN2_RXFilter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;
	FDCAN2_RXFilter.FilterID1=0x000;
	FDCAN2_RXFilter.FilterID2=0x000;                               
	if(HAL_FDCAN_ConfigFilter(&hfdcan2,&FDCAN2_RXFilter)!=HAL_OK) 
	{
		Error_Handler();
	}
	HAL_FDCAN_Start(&hfdcan2);      
	HAL_FDCAN_EnableTxBufferRequest(&hfdcan2, FDCAN_TX_BUFFER0);                       
	HAL_FDCAN_EnableTxBufferRequest(&hfdcan2, FDCAN_TX_BUFFER1);                       
	HAL_FDCAN_EnableTxBufferRequest(&hfdcan2, FDCAN_TX_BUFFER2);                       
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
	FDCAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
	switch(rx_header.Identifier)
	{
		case CAN_3508_M1_ID:
		case CAN_3508_M2_ID:
		case CAN_3508_M3_ID:
        {
					
        static uint8_t i = 0;
        i = rx_header.Identifier - CAN_3508_M1_ID;
        get_motor_measure(&motor_chassis[i], rx_data);
        motor[i].speed = motor_chassis[i].speed_rpm;
					
        
					if (motor_chassis[i].ecd - motor_chassis[i].last_ecd > 4096)
						motor_chassis[i].circle--;
					else if (motor_chassis[i].ecd - motor_chassis[i].last_ecd < -4096)
						motor_chassis[i].circle++;
							
					motor[i].angle = motor_chassis[i].ecd * 360 / 8192.0;
					motor[i].circle = motor_chassis[i].circle;
					motor[i].already_angle = (motor[i].angle + motor[i].circle * 360) / ratio; 
					motor[i].absolute_angle = motor[i].angle + motor[i].circle * 360 / ratio;
            } break;
        default:
        {
        break;
        }
	}

	
}
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{	
	chassis_tx_message.Identifier = CAN_CHASSIS_ALL_ID;
	chassis_tx_message.IdType = FDCAN_STANDARD_ID;
	chassis_tx_message.TxFrameType = FDCAN_DATA_FRAME;
	chassis_tx_message.DataLength = FDCAN_DLC_BYTES_8;
	chassis_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
	chassis_tx_message.FDFormat = FDCAN_CLASSIC_CAN;
	chassis_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	chassis_tx_message.MessageMarker = 0;
	chassis_can_send_data[0] = motor1 >> 8;
	chassis_can_send_data[1] = motor1;
	chassis_can_send_data[2] = motor2 >> 8;
	chassis_can_send_data[3] = motor2;
	chassis_can_send_data[4] = motor3 >> 8;
	chassis_can_send_data[5] = motor3;
	chassis_can_send_data[6] = motor4 >> 8;
	chassis_can_send_data[7] = motor4;
	
	while(1){
		FDCAN_TX_SEND_BUFFER(0);
		FDCAN_TX_SEND_BUFFER(1);
		FDCAN_TX_SEND_BUFFER(2);
		FDCAN_TX_SEND_BUFFER(3);
		FDCAN_TX_SEND_BUFFER(4);
		FDCAN_TX_SEND_BUFFER(5);
	}
}
uint8_t odrv_write_msg(Axis_t axis, Odrive_Command cmd)
{
FDCAN_TxHeaderTypeDef header;
uint32_t send_mail_box;
  OdriveAxisSetState_t *odrive_set;

uint8_t data[8] = {0};
float_to_uint8_t pack;  
uint8_t tmp_word[4];
header.IdType = FDCAN_STANDARD_ID;
if(axis == AXIS_0){
    header.Identifier = AXIS0_NODE_ID  + cmd;
    odrive_set = odrive_set_axis;
}
else if(axis == AXIS_1){
    header.Identifier = AXIS1_NODE_ID  + cmd;
    odrive_set = odrive_set_axis + 1;
}
	else if(axis == AXIS_2){
		header.Identifier = AXIS2_NODE_ID  + cmd;
    odrive_set = odrive_set_axis + 2;
	}
else{
    return 0;    //
}
    switch(cmd)
{
    case MSG_ODRIVE_ESTOP:
        /* TODO: Implement */
        break;
    case MSG_GET_MOTOR_ERROR:
        header.TxFrameType = FDCAN_REMOTE_FRAME;
        header.DataLength = FDCAN_DLC_BYTES_0;
        break;
    case MSG_GET_ENCODER_ERROR:
        header.TxFrameType = FDCAN_REMOTE_FRAME;
        header.DataLength = FDCAN_DLC_BYTES_0;
        break;
    case MSG_GET_SENSORLESS_ERROR:
        /* TODO: Implement */
        break;
    case MSG_SET_AXIS_NODE_ID:
        /* TODO: Implement */
        break;
    case MSG_SET_AXIS_REQUESTED_STATE:
        memcpy(data, &(odrive_set->requested_state), 4);
        header.TxFrameType = FDCAN_DATA_FRAME;
        header.DataLength = FDCAN_DLC_BYTES_4;
        break;
    case MSG_SET_AXIS_STARTUP_CONFIG:
        /* TODO: Implement */
        break;
    case MSG_GET_ENCODER_ESTIMATES:
        header.TxFrameType = FDCAN_REMOTE_FRAME;
        header.DataLength = FDCAN_DLC_BYTES_0;
        break;
    case MSG_GET_ENCODER_COUNT:
        header.TxFrameType = FDCAN_REMOTE_FRAME;
        header.DataLength = FDCAN_DLC_BYTES_0;
        break;
    case MSG_SET_CONTROLLER_MODES:
        data[0] = odrive_set->control_mode;
        data[4] = odrive_set->input_mode;
        header.TxFrameType = FDCAN_DATA_FRAME;
        header.DataLength = FDCAN_DLC_BYTES_8;
        break;
    case MSG_SET_INPUT_POS:
        memcpy(data, &(odrive_set->input_pos), 4);
        data[4] = odrive_set->vel_ff & 0x00FF;
        data[5] = odrive_set->vel_ff >> 8;
        data[6] = odrive_set->current_ff & 0x00FF;
        data[7] = odrive_set->current_ff >> 8;
        header.TxFrameType = FDCAN_DATA_FRAME;
        header.DataLength = FDCAN_DLC_BYTES_8;
        break;
    case MSG_SET_INPUT_VEL:
        pack.value[0] = odrive_set->input_vel; // odrive_set_axis0.input_vel;   
        pack.value[1] = odrive_set->torque_vel; // odrive_set_axis0.torque_vel;
        header.TxFrameType = FDCAN_DATA_FRAME;
        header.DataLength = FDCAN_DLC_BYTES_8;
        break;
    case MSG_SET_INPUT_CURRENT:
        memcpy(data, &(odrive_set->input_current), 4);
        header.TxFrameType = FDCAN_DATA_FRAME;
        header.DataLength = FDCAN_DLC_BYTES_4;
        break;
    case MSG_SET_VEL_LIMIT:
        memcpy(data, &(odrive_set->vel_limit), 4);
        header.TxFrameType = FDCAN_DATA_FRAME;
        header.DataLength = FDCAN_DLC_BYTES_4;
        break;
    case MSG_START_ANTICOGGING:
        header.TxFrameType = FDCAN_REMOTE_FRAME;
        header.DataLength = FDCAN_DLC_BYTES_0;
        break;
    case MSG_SET_TRAJ_VEL_LIMIT:
        memcpy(data, &(odrive_set->traj_vel_limit), 4);
        header.TxFrameType = FDCAN_DATA_FRAME;
        header.DataLength = FDCAN_DLC_BYTES_4;
        break;
    case MSG_SET_TRAJ_ACCEL_LIMITS:
        memcpy(data, &(odrive_set->traj_accel_limit), 4);
        memcpy(tmp_word, &(odrive_set->traj_decel_limit), 4);
        data[4] = tmp_word[0];
        data[5] = tmp_word[1];
        data[6] = tmp_word[2];
        data[7] = tmp_word[3];
        header.TxFrameType = FDCAN_DATA_FRAME;
        header.DataLength = FDCAN_DLC_BYTES_4;
        break;
    case MSG_SET_TRAJ_A_PER_CSS:
        memcpy(data, &(odrive_set->traj_a_per_css), 4);
        header.TxFrameType = FDCAN_DATA_FRAME;
        header.DataLength = FDCAN_DLC_BYTES_4;
        break;
    case MSG_GET_IQ:
        /* TODO: Implement */
        break;
    case MSG_GET_SENSORLESS_ESTIMATES:
        /* TODO: Implement */
        break;
    case MSG_RESET_ODRIVE:
        header.TxFrameType = FDCAN_REMOTE_FRAME;
        header.DataLength = FDCAN_DLC_BYTES_0;
        break;
    case MSG_GET_VBUS_VOLTAGE:
        header.TxFrameType = FDCAN_REMOTE_FRAME;
        header.DataLength = FDCAN_DLC_BYTES_0;
        break;
    case MSG_CLEAR_ERRORS:
        header.TxFrameType = FDCAN_REMOTE_FRAME;
        header.DataLength = FDCAN_DLC_BYTES_0;
        break;
    case MSG_CO_HEARTBEAT_CMD:
        /* TODO: Implement */
        break;
    default:
        break;
  }
	while(1){
		FDCAN_TX_ODRV_SEND_BUFFER(0);
		FDCAN_TX_ODRV_SEND_BUFFER(1);
		FDCAN_TX_ODRV_SEND_BUFFER(2);
		FDCAN_TX_ODRV_SEND_BUFFER(3);
		FDCAN_TX_ODRV_SEND_BUFFER(4);
		FDCAN_TX_ODRV_SEND_BUFFER(5);
	}
  return 0;
} 
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
	return &motor_chassis[(i & 0x03)];
}
