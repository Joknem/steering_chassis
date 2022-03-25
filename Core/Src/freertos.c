/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "log.h"
#include "motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern Motor motor[3];
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTaskA */
osThreadId_t myTaskAHandle;
const osThreadAttr_t myTaskA_attributes = {
  .name = "myTaskA",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for myTaskB */
osThreadId_t myTaskBHandle;
const osThreadAttr_t myTaskB_attributes = {
  .name = "myTaskB",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for myTaskC */
osThreadId_t myTaskCHandle;
const osThreadAttr_t myTaskC_attributes = {
  .name = "myTaskC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTaskA(void *argument);
void StartTaskB(void *argument);
void StartTaskC(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTaskA */
  myTaskAHandle = osThreadNew(StartTaskA, NULL, &myTaskA_attributes);

  /* creation of myTaskB */
  myTaskBHandle = osThreadNew(StartTaskB, NULL, &myTaskB_attributes);

  /* creation of myTaskC */
  myTaskCHandle = osThreadNew(StartTaskC, NULL, &myTaskC_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    // HAL_GPIO_TogglePin(LD_O_GPIO_Port, LD_O_Pin);
    // ST_LOG("\033[0;42;%dm%d\033[0m was selected", 31 + cnt % 4, cnt);
    // osDelay(500);
    // cnt++;
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTaskA */
/**
* @brief Function implementing the myTaskA thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskA */
void StartTaskA(void *argument)
{
  /* USER CODE BEGIN StartTaskA */
  /* Infinite loop */
  for(;;)
  {
    // HAL_GPIO_TogglePin(LD_O_GPIO_Port, LD_O_Pin);
    // ST_LOG("this is A,time is %d", HAL_GetTick());  
    // osDelay(500);
  }
  /* USER CODE END StartTaskA */
}

/* USER CODE BEGIN Header_StartTaskB */
/**
* @brief Function implementing the myTaskB thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskB */
void StartTaskB(void *argument)
{
  /* USER CODE BEGIN StartTaskB */
  /* Infinite loop */
  for(;;)
  {
    // ST_LOG("this is B,time is %d", HAL_GetTick());
    // osDelay(100);
    // uart_printf("this task b\r\n");
    // osDelay(500);
  }
  /* USER CODE END StartTaskB */
}

/* USER CODE BEGIN Header_StartTaskC */
/**
* @brief Function implementing the myTaskC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskC */
void StartTaskC(void *argument)
{
  /* USER CODE BEGIN StartTaskC */
  /* Infinite loop */
  for(;;)
  {
    // ST_LOGE("ang:%.2f, tar:%.2f", motor[0].absolute_angle, motor[0].target_angle);  
  }
  /* USER CODE END StartTaskC */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
