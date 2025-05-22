/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "LED_control.h"
#include "referee_usart_task.h"
#include "iwdg.h"
#include "CAN_Re_Se.h"
#include "chassis_move.h"
#include "gimbal.h"
#include "global_status.h"
#include "control_setting.h"
#include "dm_driver.h"
#include "IMU_updata.h"
#include "shoot.h"
#include "NUC_communication.h"
#include "SEGGER_RTT.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Flash_LED_Task */
osThreadId_t Flash_LED_TaskHandle;
const osThreadAttr_t Flash_LED_Task_attributes = {
  .name = "Flash_LED_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CAN_sendTask */
osThreadId_t CAN_sendTaskHandle;
const osThreadAttr_t CAN_sendTask_attributes = {
  .name = "CAN_sendTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for RemoteTask */
osThreadId_t RemoteTaskHandle;
const osThreadAttr_t RemoteTask_attributes = {
  .name = "RemoteTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for ChassisTask */
osThreadId_t ChassisTaskHandle;
const osThreadAttr_t ChassisTask_attributes = {
  .name = "ChassisTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for GimbalTask */
osThreadId_t GimbalTaskHandle;
const osThreadAttr_t GimbalTask_attributes = {
  .name = "GimbalTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for NUCcontrolTask */
osThreadId_t NUCcontrolTaskHandle;
const osThreadAttr_t NUCcontrolTask_attributes = {
  .name = "NUCcontrolTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for ErrorDetectTask */
osThreadId_t ErrorDetectTaskHandle;
const osThreadAttr_t ErrorDetectTask_attributes = {
  .name = "ErrorDetectTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for RefereeTask */
osThreadId_t RefereeTaskHandle;
const osThreadAttr_t RefereeTask_attributes = {
  .name = "RefereeTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Flash_LED_Task_callback(void *argument);
void CAN_sendTask_callback(void *argument);
void RemoteTask_callback(void *argument);
void ChassisTask_callback(void *argument);
void GimbalTask_callback(void *argument);
void NUCcontrolTask_callback(void *argument);
void ErrorDetectTask_callback(void *argument);
void RefereeTask_callback(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

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
  /* creation of Flash_LED_Task */
  Flash_LED_TaskHandle = osThreadNew(Flash_LED_Task_callback, NULL, &Flash_LED_Task_attributes);

  /* creation of CAN_sendTask */
  CAN_sendTaskHandle = osThreadNew(CAN_sendTask_callback, NULL, &CAN_sendTask_attributes);

  /* creation of RemoteTask */
  RemoteTaskHandle = osThreadNew(RemoteTask_callback, NULL, &RemoteTask_attributes);

  /* creation of ChassisTask */
  ChassisTaskHandle = osThreadNew(ChassisTask_callback, NULL, &ChassisTask_attributes);

  /* creation of GimbalTask */
  GimbalTaskHandle = osThreadNew(GimbalTask_callback, NULL, &GimbalTask_attributes);

  /* creation of NUCcontrolTask */
  NUCcontrolTaskHandle = osThreadNew(NUCcontrolTask_callback, NULL, &NUCcontrolTask_attributes);

  /* creation of ErrorDetectTask */
  ErrorDetectTaskHandle = osThreadNew(ErrorDetectTask_callback, NULL, &ErrorDetectTask_attributes);

  /* creation of RefereeTask */
  RefereeTaskHandle = osThreadNew(RefereeTask_callback, NULL, &RefereeTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Flash_LED_Task_callback */
/**
  * @brief  Function implementing the Flash_LED_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Flash_LED_Task_callback */
void Flash_LED_Task_callback(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN Flash_LED_Task_callback */
  /* Infinite loop */
  for(;;)
  {
    led_show(RED);
    osDelay(500);
    led_show(GREEN);
    osDelay(500);
    led_show(BLUE);
    osDelay(500);
  }
  /* USER CODE END Flash_LED_Task_callback */
}

/* USER CODE BEGIN Header_CAN_sendTask_callback */
/**
 * 
* @brief Function implementing the CAN_sendTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_sendTask_callback */
void CAN_sendTask_callback(void *argument)
{
  /* USER CODE BEGIN CAN_sendTask_callback */
  /* Infinite loop */
  for(;;)
  {

//    CAN1_send_current();
	  CAN2_send_current();
    osDelay(5);
  }
  /* USER CODE END CAN_sendTask_callback */
}

/* USER CODE BEGIN Header_RemoteTask_callback */
/**
* @brief Function implementing the RemoteTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RemoteTask_callback */
void RemoteTask_callback(void *argument)
{
  /* USER CODE BEGIN RemoteTask_callback */
  /* Infinite loop */
  for(;;)
  {
    remote_control_task();
    Mode_change();
    osDelay(1);
  }
  /* USER CODE END RemoteTask_callback */
}

/* USER CODE BEGIN Header_ChassisTask_callback */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassisTask_callback */
void ChassisTask_callback(void *argument)
{
  /* USER CODE BEGIN ChassisTask_callback */
  /* Infinite loop */
  for(;;)
  {
    get_global_chassis_input();
//    chassis_moto_speed_calc();
//		Send_to_Chassis();
		
		Send_to_Chassis_1();
		Send_to_Chassis_2();
    Send_to_Chassis_3();

//    osDelay(1);
		osDelay(1);
  }
  /* USER CODE END ChassisTask_callback */
}

/* USER CODE BEGIN Header_GimbalTask_callback */
/**
* @brief Function implementing the GimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GimbalTask_callback */
void GimbalTask_callback(void *argument)
{
  /* USER CODE BEGIN GimbalTask_callback */
  /* Infinite loop */
  for(;;)
  {
    gimbal_update(); 
//		SEGGER_RTT_printf(0, "gimbal set:%f\r\n", gimbal.yaw.set);
//    SEGGER_RTT_printf(0, "gimbal now:%f\r\n", gimbal.yaw.now);

	  gimbal_pid_cal();
	  shoot_update();
	  shoot_pid_cal();
    osDelay(1);
  }
  /* USER CODE END GimbalTask_callback */
}

/* USER CODE BEGIN Header_NUCcontrolTask_callback */
/**
* @brief Function implementing the NUCcontrolTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_NUCcontrolTask_callback */
void NUCcontrolTask_callback(void *argument)
{
  /* USER CODE BEGIN NUCcontrolTask_callback */
  /* Infinite loop */
  for(;;)
  {
    Navigation_send_message();
    osDelay(1);
  }
  /* USER CODE END NUCcontrolTask_callback */
}

/* USER CODE BEGIN Header_ErrorDetectT
ask_callback */
/**
* @brief Function implementing the ErrorDetectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ErrorDetectTask_callback */
void ErrorDetectTask_callback(void *argument)
{
  /* USER CODE BEGIN ErrorDetectTask_callback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ErrorDetectTask_callback */
}

/* USER CODE BEGIN Header_RefereeTask_callback */
/**
* @brief Function implementing the RefereeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RefereeTask_callback */
void RefereeTask_callback(void *argument)
{
  /* USER CODE BEGIN RefereeTask_callback */
  /* Infinite loop */
  for(;;)
  {
    referee_usart_task();
    Required_Data();
    osDelay(1);
  }
  /* USER CODE END RefereeTask_callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

