/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "string.h"
#include "usbd_cdc_if.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "buzz.h"
#include "gpio.h"
#include "pid.h"
#include "ctrl.h"
#include "fonts.h"
#include "u8g_arm.h"
#include "coffee_params.h"
#include <stdio.h>
#include <math.h>
#include "MAX31865.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
MAX31865_SPI max31865_t1;
MAX31865_SPI max31865_t2;
MAX31865_SPI max31865_t3;
MAX31865_SPI max31865_t4;
MAX31865_SPI max31865_t5;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
struct GUI_dev GUI = GUI_INIT();
struct SYS_dev SYS = SYS_INIT();
struct PID_dev BOILER_PID = PID_BOILER_INIT();
struct PID_dev STEAMER_PID = PID_STEAMER_INIT();
struct PUMP_dev PUMP = PUMP_INIT();
struct BOILER_dev BOILER = BOILER_INIT();
struct BOILER_dev STEAMER = BOILER_INIT();

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
float temperature1;
float temperature2;
float temperature3;
float temperature4;
float temperature5;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for temperatureTask */
osThreadId_t temperatureTaskHandle;
const osThreadAttr_t temperatureTask_attributes = {
  .name = "temperatureTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for oledTask */
osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
  .name = "oledTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for sleepTask */
osThreadId_t sleepTaskHandle;
const osThreadAttr_t sleepTask_attributes = {
  .name = "sleepTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for bldcTask */
osThreadId_t bldcTaskHandle;
const osThreadAttr_t bldcTask_attributes = {
  .name = "bldcTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for statemachineTas */
osThreadId_t statemachineTasHandle;
const osThreadAttr_t statemachineTas_attributes = {
  .name = "statemachineTas",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for guiTask */
osThreadId_t guiTaskHandle;
const osThreadAttr_t guiTask_attributes = {
  .name = "guiTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTemperatureTask(void *argument);
void StartOledTask(void *argument);
void StartSleepTask(void *argument);
void StartBldcTask(void *argument);
void StartStateMachine(void *argument);
void StartGuiTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  // adc init


  //HAL_ADC_Stop_DMA(&hadc1);
  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 2);

  // encoder
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

  // servo/buzzer PWM
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  // PWM
  // heater 1
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  // heater 2
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);


  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);


  TIM2->CCR4 = 0;
  TIM3->CCR1 = 0;
  TIM3->CCR2 = 0;
  TIM3->CCR3 = 0;
  TIM3->CCR4 = 0;
  TIM4->CCR4 = 0;

  TIM1->CNT = 0;


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

  /* creation of temperatureTask */
  temperatureTaskHandle = osThreadNew(StartTemperatureTask, NULL, &temperatureTask_attributes);

  /* creation of oledTask */
  oledTaskHandle = osThreadNew(StartOledTask, NULL, &oledTask_attributes);

  /* creation of sleepTask */
  sleepTaskHandle = osThreadNew(StartSleepTask, NULL, &sleepTask_attributes);

  /* creation of bldcTask */
  bldcTaskHandle = osThreadNew(StartBldcTask, NULL, &bldcTask_attributes);

  /* creation of statemachineTas */
  statemachineTasHandle = osThreadNew(StartStateMachine, NULL, &statemachineTas_attributes);

  /* creation of guiTask */
  guiTaskHandle = osThreadNew(StartGuiTask, NULL, &guiTask_attributes);

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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */

  for(;;)
  {
	HAL_GPIO_WritePin(STAT_GPIO_Port, STAT_Pin, 1);
	osDelay(500);

	HAL_GPIO_WritePin(STAT_GPIO_Port, STAT_Pin, 0);
	osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTemperatureTask */
/**
* @brief Function implementing the temperatureTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTemperatureTask */
void StartTemperatureTask(void *argument)
{
  /* USER CODE BEGIN StartTemperatureTask */
  /* Infinite loop */

  HAL_GPIO_WritePin(WL_GPIO_Port, WL_Pin, 1);
  osDelay(100);

  MAX31865_init(&max31865_t1, SPI2_CS1_GPIO_Port, SPI2_CS1_Pin, &hspi2, WIRE2, 0);
  MAX31865_init(&max31865_t2, SPI2_CS2_GPIO_Port, SPI2_CS2_Pin, &hspi2, WIRE2, 3.7 + 0.7);
  MAX31865_init(&max31865_t3, SPI2_CS3_GPIO_Port, SPI2_CS3_Pin, &hspi2, WIRE2, 2.2 + 0.9);
  MAX31865_init(&max31865_t4, SPI2_CS4_GPIO_Port, SPI2_CS4_Pin, &hspi2, WIRE2, 0);
  MAX31865_init(&max31865_t5, SPI2_CS5_GPIO_Port, SPI2_CS5_Pin, &hspi2, WIRE2, 0);

  osDelay(100);

  HAL_GPIO_WritePin(WL_GPIO_Port, WL_Pin, 0);

  osDelay(2000);

  uint32_t counter = 0;
  for(;;)
  {
	HAL_GPIO_WritePin(WL_GPIO_Port, WL_Pin, 1);
	temperature1 = MAX31865_temperature(&max31865_t1);
	temperature2 = MAX31865_temperature(&max31865_t2);
	temperature3 = MAX31865_temperature(&max31865_t3);
	temperature4 = MAX31865_temperature(&max31865_t4);
	temperature5 = MAX31865_temperature(&max31865_t5);
	//BOILER.temperature = (float)temperature1;
	if (counter % (PID_INTERVAL / TEMPERATURE_READOUT_INTERVAL) == 0){
		compute(&BOILER_PID, temperature3);
		compute(&STEAMER_PID, temperature2);
	}

	printf("%lu, %3.2f, %3.2f, %3.2f, %3.2f, %3.2f, %u\n",
			HAL_GetTick(), temperature1, temperature2, temperature3, temperature4, temperature5, BOILER_PID.pwr);
	osDelay(TEMPERATURE_READOUT_INTERVAL / 2);
	HAL_GPIO_WritePin(WL_GPIO_Port, WL_Pin, 0);
    osDelay(TEMPERATURE_READOUT_INTERVAL / 2);
    counter++;
  }
  /* USER CODE END StartTemperatureTask */
}

/* USER CODE BEGIN Header_StartOledTask */
/**
* @brief Function implementing the oledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOledTask */
void StartOledTask(void *argument)
{
  /* USER CODE BEGIN StartOledTask */
  /* Infinite loop */
  char buffer[20];
  char state_names[][20] = {"IDLE", "HEATING", "READY", "PRE_INFUSE", "EXTRACTING", "DEBUG_EXTRACTION"};

  static u8g_t u8g;
  u8g_InitComFn(&u8g, &u8g_dev_ssd1306_128x64_i2c, u8g_com_hw_i2c_fn); //here we initialise our u8glib driver

  u8g_FirstPage(&u8g);
  do
  {
	u8g_SetFont(&u8g, u8g_font_profont12);//set current font
	u8g_DrawStr(&u8g, 35, 40, "Bijou");//write string - you set coordinates and string
	//u8g_DrawBox(&u8g, 30, 30, 35, 35);//draw some box
  } while ( u8g_NextPage(&u8g) );
  u8g_Delay(1000);


  u8g_FirstPage(&u8g);
  do
  {
	 sprintf(buffer, "T1 config: %d", MAX31865_configuration(&max31865_t1));
	 u8g_DrawStr(&u8g, 0, 12, buffer);
	 sprintf(buffer, "T2 config: %d", MAX31865_configuration(&max31865_t2));
	 u8g_DrawStr(&u8g, 0, 22, buffer);
	 sprintf(buffer, "T3 config: %d", MAX31865_configuration(&max31865_t3));
	 u8g_DrawStr(&u8g, 0, 32, buffer);
	 sprintf(buffer, "T4 config: %d", MAX31865_configuration(&max31865_t4));
	 u8g_DrawStr(&u8g, 0, 42, buffer);
	 sprintf(buffer, "T5 config: %d", MAX31865_configuration(&max31865_t5));
	 u8g_DrawStr(&u8g, 0, 52, buffer);
  } while ( u8g_NextPage(&u8g) );
  u8g_Delay(1000);



  /* Infinite loop */
  for(;;)
  {

	u8g_FirstPage(&u8g);
	do
	{
		u8g_SetFont(&u8g, u8g_font_profont11); //set current font
		sprintf(buffer, "T: %dC PROBE: %3.1fC", GUI.target_set, temperature1);
		u8g_DrawStr(&u8g, 0, 11, buffer);
		sprintf(buffer, "GH: %3.1fC TB: %3.1fC", temperature2, temperature3);
		u8g_DrawStr(&u8g, 0, 22, buffer);
		sprintf(buffer, "P: %ld D: %ld", (int32_t)BOILER_PID.p, (int32_t)BOILER_PID.d);
		u8g_DrawStr(&u8g, 0, 32, buffer);
		sprintf(buffer, "hpwr: %ld pwr: %ld", (int32_t)TIM3->CCR1, (int32_t)TIM3->CCR3);
		u8g_DrawStr(&u8g, 0, 44, buffer);
		sprintf(buffer, "c: %u p: %u s: %d", (uint16_t)TIM1->CNT, (uint16_t)PUMP.motor_v, SYS.state);
		u8g_DrawStr(&u8g, 0, 55, buffer);
		u8g_SetFont(&u8g, u8g_font_04b_03b); //set current font
		u8g_DrawStr(&u8g, 0, 63, state_names[SYS.state - IDLE]);

	} while ( u8g_NextPage(&u8g) );

	osDelay(OLED_INTERVAL);
  }
  /* USER CODE END StartOledTask */
}

/* USER CODE BEGIN Header_StartSleepTask */
/**
  * @brief  Function implementing the sleepTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSleepTask */
void StartSleepTask(void *argument)
{
  /* USER CODE BEGIN StartSleepTask */
  /* Infinite loop */
  uint16_t cnt = 0;
  for(;;)
  {
	cnt += 20;
	switch(SYS.state){
		case IDLE:
			TIM3->CCR4 = 0;
			break;
		case HEATING:
			TIM3->CCR4 = (uint16_t) (512 - 512 * cosf((float)cnt / 512.0 * M_PI)); // LED1
			break;
		case READY:
			TIM3->CCR4 = 1023;
			break;
		case PRE_INFUSE:
			// fast blinking
			TIM3->CCR4 = 1023;
			osDelay(250);
			TIM3->CCR4 = 0;
			osDelay(250 - SLEEP_INTERVAL);
			break;
		case EXTRACTING:
			// slow blinking
			TIM3->CCR4 = 1023;
			osDelay(500);
			TIM3->CCR4 = 0;
			osDelay(500 - SLEEP_INTERVAL);
			break;
		default:
			TIM3->CCR4 = 0;
			break;
	}
    osDelay(SLEEP_INTERVAL);
  }
  /* USER CODE END StartSleepTask */
}

/* USER CODE BEGIN Header_StartBldcTask */
/**
* @brief Function implementing the bldcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBldcTask */
void StartBldcTask(void *argument)
{
  /* USER CODE BEGIN StartBldcTask */
  /* Infinite loop */
  for(;;)
  {
	  switch(PUMP.state){
		  case IDLE:
			  PUMP.motor_en = 0;
			  PUMP.motor_v = 0;
			  break;
		  case HEATING:
			  PUMP.motor_en = 1;
			  PUMP.motor_v = (uint16_t)(1024.0 / 100 * GUI.pwr_set);
			  break;
		  case PRE_INFUSE:
			  PUMP.motor_en = 1;
			  PUMP.motor_v = 100;
			  osDelay(4000);
			  SYS.state = EXTRACTING;
			  break;
		  case EXTRACTING:
			  PUMP.motor_en = 1;
			  PUMP.motor_v = 1024;
			  osDelay(25000);
			  SYS.state = IDLE;
			  break;
		  case DEBUG_EXTRACTION:
			  PUMP.motor_en = 1;
			  PUMP.motor_v = (uint16_t)(1024.0 / 100 * GUI.pwr_set);
		  default:
			  PUMP.motor_en = 0;
			  PUMP.motor_v = 0;
			  PUMP.state = IDLE;
			  break;
	  }
	  osDelay(BLDC_INTERVAL);

  }
  /* USER CODE END StartBldcTask */
}

/* USER CODE BEGIN Header_StartStateMachine */
/**
* @brief Function implementing the statemachineTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStateMachine */
void StartStateMachine(void *argument)
{
  /* USER CODE BEGIN StartStateMachine */
	uint16_t BOILER_PWR = 0;
	uint16_t STEAMER_PWR = 0;
	HAL_GPIO_WritePin(BLDC_DIR_GPIO_Port, BLDC_DIR_Pin, 0);
	HAL_GPIO_WritePin(BLDC_EN_GPIO_Port, BLDC_EN_Pin, 1);
	STEAMER.active = 0;
  /* Infinite loop */
  for(;;)
  {
	  switch(SYS.state){
		  case IDLE:
			  PUMP.state = IDLE;
			  BOILER_PWR = 0;
			  STEAMER_PWR = 0;
			  SYS.state = HEATING;
			  break;
		  case HEATING:
			  PUMP.state = HEATING;
			  BOILER_PWR = BOILER_PID.pwr;
			  STEAMER_PWR = STEAMER_PID.pwr;
			  // check if boiler is ready
			  if ((BOILER.temperature < BOILER_PID.target * 1.05)
							  && (BOILER.temperature > BOILER_PID.target * 0.95)){
				  BOILER.state = READY;
			  }
			  // check if steamer is not active or ready
			  if ((STEAMER.active == 0) ||
					  ((STEAMER.temperature < STEAMER_PID.target * 1.05)
							  && (STEAMER.temperature > STEAMER_PID.target * 0.95))){
				  STEAMER.state = READY;
			  }

			  if ((STEAMER.state == READY) && (BOILER.state == READY)){
				  SYS.state = READY;
			  }
			  break;
		  case READY:
			  PUMP.state = IDLE;
			  BOILER_PWR = BOILER_PID.pwr;
			  STEAMER_PWR = STEAMER_PID.pwr;
			  if (GUI.lever_button_state == PRESSED){
				  if (DEBUG){
					  SYS.state = DEBUG_EXTRACTION;
				  } else {
					  SYS.state = PRE_INFUSE;
				  }
			  }
			  break;
		  case PRE_INFUSE:
			  PUMP.state = PRE_INFUSE;
			  BOILER_PWR = BOILER_PID.pwr;
			  STEAMER_PWR = STEAMER_PID.pwr;
			  break;
		  case EXTRACTING:
			  PUMP.state = EXTRACTING;
			  BOILER_PWR = 1023;
			  STEAMER_PWR = 1023;
			  if ((SYS.lever_button == 0) || (PUMP.state == IDLE)){
				  PUMP.state = IDLE;
				  SYS.state = HEATING;
			  }
			  break;
		  case DEBUG_EXTRACTION:
			  PUMP.state = EXTRACTING;
			  BOILER_PWR = (uint16_t)(1024.0 / 100 * GUI.pwr_set);
			  STEAMER_PWR = (uint16_t)(1024.0 / 100 * GUI.pwr_set);
			  if ((SYS.lever_button == 0) || (PUMP.state == IDLE)){
				  PUMP.state = IDLE;
				  SYS.state = HEATING;
			  }
			  break;
		  default:
			  SYS.state = IDLE;
			  PUMP.state = IDLE;
			  BOILER_PWR = 0;
			  STEAMER_PWR = 0;
			  break;
	  }

	  // update Output
	  TIM3->CCR1 = BOILER_PWR;
	  TIM3->CCR2 = STEAMER_PWR;
	  TIM3->CCR3 = PUMP.motor_v;
	  HAL_GPIO_WritePin(BLDC_EN_GPIO_Port, BLDC_EN_Pin, 1 - PUMP.motor_en);

	  osDelay(STATE_MACHINE_INTERVAL);
  }
  /* USER CODE END StartStateMachine */
}

/* USER CODE BEGIN Header_StartGuiTask */
/**
* @brief Function implementing the guiTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGuiTask */
void StartGuiTask(void *argument)
{
  /* USER CODE BEGIN StartGuiTask */
  /* Infinite loop */
  for(;;)
  {
	switch (GUI.enc_mode){
		case PWR_MODE:
			if (GUI.enc_button_state == PRESSED){
				GUI.enc_mode = TARGET_MODE;
				TIM1->CNT = GUI.target_set - 20;
			} else {
				GUI.pwr_set = TIM1->CNT;
			}
			break;
		case TARGET_MODE:
			if (GUI.enc_button_state == PRESSED){
				GUI.enc_mode = PWR_MODE;
				TIM1->CNT = GUI.pwr_set;
			} else {
				GUI.target_set = TIM1->CNT + 20;
				BOILER_PID.target = (float)GUI.target_set;
			}
			break;
		default:
			GUI.enc_mode = PWR_MODE;
			break;
	}

	switch (GUI.enc_button_state){
		case NOT_PRESSED:
			if (HAL_GPIO_ReadPin(ENC_SW_GPIO_Port, ENC_SW_Pin) == 0){
				GUI.enc_button_state = PRESSED;
			}
			break;
		case PRESSED:
			if (HAL_GPIO_ReadPin(ENC_SW_GPIO_Port, ENC_SW_Pin) == 1){
				GUI.enc_button_state = NOT_PRESSED;
			}
			break;
		default:
			GUI.enc_button_state = NOT_PRESSED;
			break;
	}

	switch (GUI.lever_button_state){
		case NOT_PRESSED:
			if (HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin) == 0){
				GUI.lever_button_state = PRESSED;
			}
			break;
		case PRESSED:
			if (HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin) == 1){
				GUI.lever_button_state = NOT_PRESSED;
			}
			break;
		default:
			GUI.lever_button_state = NOT_PRESSED;
			break;
	}

	switch (GUI.steam_button_state){
		case NOT_PRESSED:
			if (HAL_GPIO_ReadPin(STEAM_GPIO_Port, STEAM_Pin) == 0){
				GUI.steam_button_state = PRESSED;
			}
			break;
		case PRESSED:
			if (HAL_GPIO_ReadPin(STEAM_GPIO_Port, STEAM_Pin) == 1){
				GUI.steam_button_state = NOT_PRESSED;
			}
			break;
		default:
			GUI.steam_button_state = NOT_PRESSED;
			break;
	}
    osDelay(GUI_INTERVAL);
  }
  /* USER CODE END StartGuiTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

