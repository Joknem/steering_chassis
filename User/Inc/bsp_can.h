#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "main.h"

#define CHASSIS_CAN hfdcan1
#define AXIS0_NODE_ID  (3<<5)
#define AXIS1_NODE_ID  (0<<5)
#define AXIS2_NODE_ID  (1<<5)

#define FDCAN_TX_SEND_BUFFER(x) \
						if(HAL_FDCAN_IsTxBufferMessagePending(&hfdcan2, FDCAN_TX_BUFFER##x)==0){\
							HAL_FDCAN_AddMessageToTxBuffer(&hfdcan2, &chassis_tx_message, chassis_can_send_data, FDCAN_TX_BUFFER##x); \
							HAL_FDCAN_EnableTxBufferRequest(&hfdcan2, FDCAN_TX_BUFFER##x); \
							break;\
						}
#define FDCAN_TX_ODRV_SEND_BUFFER(x) \
						if(HAL_FDCAN_IsTxBufferMessagePending(&hfdcan2, FDCAN_TX_BUFFER##x)==0){ \
							HAL_FDCAN_AddMessageToTxBuffer(&hfdcan2, &header, pack.raw, FDCAN_TX_BUFFER##x); \
							HAL_FDCAN_EnableTxBufferRequest(&hfdcan2, FDCAN_TX_BUFFER##x); \
							return 1;\
						}
#define get_motor_measure(ptr, data)                                   \
	{                                                                  \
		(ptr)->last_ecd = (ptr)->ecd;                                  \
		(ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
		(ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
		(ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
		(ptr)->temperate = (data)[6];                                  \
}
typedef enum
{
    AXIS_0 = 0,
    AXIS_1 = 1,
		AXIS_2 = 2
} Axis_t;
typedef struct{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
		int16_t circle;
} motor_measure_t;
typedef enum{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,
    CAN_ODRV1_AXIS1_ID = 0x010
} can_msg_id_e;
typedef struct{
    uint32_t axis_error;
    uint32_t axis_current_state;
    uint32_t motor_error;
    uint32_t encoder_error;
    uint32_t sensorless_error;
    float encoder_pos_estimate;
    float encoder_vel_estimate;
    int32_t encoder_shadow_count;
    int32_t encoder_cpr_count;
    float iq_setpoint;
    float iq_measured;
    float sensorless_pos_estimate;
    float sensorless_vel_estimate;
    float vbus_voltage;
} OdriveAxisGetState_t;
typedef struct
{
    uint16_t axis_node_id;
    uint32_t requested_state;
    int32_t control_mode;
    int32_t input_mode;
    int16_t vel_ff;
    int16_t current_ff;
    int32_t input_pos;
    float input_vel;
    float torque_vel;
    int32_t input_current;
    float vel_limit;
    float traj_vel_limit;
    float traj_accel_limit;
    float traj_decel_limit;
    float traj_a_per_css;
} OdriveAxisSetState_t;
typedef enum
{
    MSG_CO_NMT_CTRL = 0x000, // CANOpen NMT Message REC
    MSG_ODRIVE_HEARTBEAT,
    MSG_ODRIVE_ESTOP,
    MSG_GET_MOTOR_ERROR, // Errors
    MSG_GET_ENCODER_ERROR,
    MSG_GET_SENSORLESS_ERROR,
    MSG_SET_AXIS_NODE_ID,
    MSG_SET_AXIS_REQUESTED_STATE,
    MSG_SET_AXIS_STARTUP_CONFIG,
    MSG_GET_ENCODER_ESTIMATES,
    MSG_GET_ENCODER_COUNT,
    MSG_SET_CONTROLLER_MODES,
    MSG_SET_INPUT_POS,
    MSG_SET_INPUT_VEL = 0x00D,
    MSG_SET_INPUT_CURRENT,
    MSG_SET_VEL_LIMIT,
    MSG_START_ANTICOGGING,
    MSG_SET_TRAJ_VEL_LIMIT,
    MSG_SET_TRAJ_ACCEL_LIMITS,
    MSG_SET_TRAJ_A_PER_CSS,
    MSG_GET_IQ,
    MSG_GET_SENSORLESS_ESTIMATES,
    MSG_RESET_ODRIVE,
    MSG_GET_VBUS_VOLTAGE,
    MSG_CLEAR_ERRORS,
    MSG_CO_HEARTBEAT_CMD = 0x700, // CANOpen NMT Heartbeat  SEND
} Odrive_Command;

extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
void fdcanfilter(void);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
uint8_t odrv_write_msg(Axis_t axis, Odrive_Command cmd);
uint8_t odrv_send_msg(uint8_t* send_data, uint32_t ID, uint32_t CMD, uint8_t len);
#endif