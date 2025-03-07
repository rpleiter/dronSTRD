/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2021 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "i2c.h"
#include "spi.h"
#include "gpio.h"
#include "acelerometro.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

  #define TRUE 1
  #define FALSE 0

/* USER CODE BEGIN Variables */

/*Prioridades de las Tareas Periodicas*/
#define PR_TAREA1_ALTITUD 1
#define PR_TAREA2_HORIZONTALIDAD 2
#define PR_TAREA3_MOTORES 3
#define PR_TAREA4_VIBRACIONES 4

/*Periodos de las tareas*/
#define T_TAREA1_ALTITUD 300
#define T_TAREA2_HORIZONTALIDAD 200
#define T_TAREA3_MOTORES 150
#define T_TAREA4_VIBRACIONES 350


#define MAX_ROTATION 10
#define MIN_ROTATION -10
#define T_TAREA_ALTITUD 300
#define ALTITUD_DESTINO 400

 /* USER CODE BEGIN RTOS_MUTEX */
 
   /*********************Semaforo 1*********************/ ;
   SemaphoreHandle_t Semaforo_1 = NULL;
	 
	 /* Declarar aqu� las variables protegidas por el sem�foro 1 */
   double Altitud = 0; // Altitud  
   double RX = 0; // Inclinacion eje X 
   double RY = 0; // Inclinacion eje Y
	 int Lectura_ADC1 = 0; //Valor leido
	 double aX, aY, aZ = 0;
   /* Tarea y mutex creados con CMSIS 
   osThreadId Tarea1Handle;
   osMutexId mutex1Handle; */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   void EncenderMotores(void const * argument);
	 void TareaLeerHorizontabilidad(void const * argument);
	 void TareaAltitud(void const * argument);
	 void TareaVibraciones(void const * argument);
	
/* Variables para depuracion */
   int ContTarea1 = 0;
	 int ContTarea2 = 0;
	 int enciende_motor_1 = 0;
	 int enciende_motor_2 = 0;
	 int enciende_motor_3 = 0;
	 int enciende_motor_4 = 0;
/*Señales*/
	 int signal_altitud_perdida = 0; 
	 int signal_inclinación_x_plus = 0;
	 int signal_inclinación_x_minus = 0;
	 int signal_inclinación_y_plus = 0;
	 int signal_inclinación_y_minus = 0;
	 int signal_start = 0;
	 int signal_wait = 0;
	 int risk = 0;
	 int recover = 0;
	 
/* USER CODE BEGIN StartTarea1 */

	 
/* USER CODE END FunctionPrototypes */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);



/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_CAN1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_I2C2_Init();
	
  /* USER CODE BEGIN Init */
  Inicializa_Acelerometro();
  /* USER CODE END Init */

  /* Create the mutex using CMSIS
  osMutexDef(mutex1);
  mutex1Handle = osMutexCreate(osMutex(mutex1)); */

  /* Create the thread using CMSIS
  osThreadDef(Tarea1, StartTarea1, osPriorityNormal, 0, 128);
  Tarea1Handle = osThreadCreate(osThread(Tarea1), NULL); */

  /* USER CODE BEGIN RTOS_THREADS using FreeRTOS */
	xTaskCreate(TareaAltitud, "Comprobar horizontabilidad", configMINIMAL_STACK_SIZE, NULL, PR_TAREA1_ALTITUD, NULL);
  xTaskCreate(TareaLeerHorizontabilidad, "Comprobar horizontabilidad", configMINIMAL_STACK_SIZE, NULL, PR_TAREA2_HORIZONTALIDAD, NULL);
  xTaskCreate(EncenderMotores, "Encender motores", configMINIMAL_STACK_SIZE, NULL, PR_TAREA3_MOTORES, NULL);

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  while (1) {
  }

}   /* main END */


/* USER CODE StartTarea1 */

void EncenderMotores(void const * argument)
{
  /* USER CODE BEGIN StartTarea1 */

	 TickType_t lastWakeTime;
   lastWakeTime = xTaskGetTickCount();
 	
	 /*HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);	*/

  /* Infinite loop */
  for(;;)
  {
		
		if(signal_altitud_perdida == 0){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		}			
		if(signal_inclinación_x_plus == 1){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
		} else{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		}
		if(signal_inclinación_x_minus == 1)
		{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
		}
		if(signal_inclinación_y_plus  == 1)
		{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		}
		if(signal_inclinación_y_minus  == 1)
		{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		}
		
		if(signal_altitud_perdida == 1){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		}

		
	
		
		/*
		if(enciende_motor_1 == 1)
		{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	  }else{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		}
		if(enciende_motor_2 == 1)
		{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
	  }else{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
		}
		if(enciende_motor_3 == 1)
		{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	  }else{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		}
		if(enciende_motor_4 == 1)
		{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	  }else{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		}
		*/
		
		ContTarea1 ++;
    // osDelay(1);
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(T_TAREA1_ALTITUD));
  }
  /* USER CODE END 5 */
	
  /* USER CODE END StartTarea1 */
}

void TareaLeerHorizontabilidad(void const * argument)
{
	double rotationX = 0;
	double rotationY = 0;
	
	TickType_t lastWakeTime;
  lastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
		rotationX = Calculate_RotationX();
		rotationY = Calculate_RotationY();
		
		if (rotationX > MAX_ROTATION){
			signal_inclinación_x_plus = 1;
			signal_inclinación_x_minus = 0;
		}
		else if (rotationX < MIN_ROTATION){
			signal_inclinación_x_minus = 1;
			signal_inclinación_x_plus = 0;
		}
		else{
			signal_inclinación_x_minus = 0;
			signal_inclinación_x_plus = 0;
		}
		
		if (rotationY > MAX_ROTATION) {
			signal_inclinación_y_minus = 0;
			signal_inclinación_y_plus = 1;
		}
		else if (rotationY < MIN_ROTATION) {
			signal_inclinación_y_minus = 1;
			signal_inclinación_y_plus = 0;
		}
		else {
			signal_inclinación_y_minus = 0;
			signal_inclinación_y_plus = 0;
		}
		ContTarea2 ++;
    // osDelay(1);
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(T_TAREA2_HORIZONTALIDAD));
  }
  /* USER CODE END 5 */
	
  /* USER CODE END StartTarea1 */
}

void TareaAltitud(void const * argument)
{ 
	TickType_t xLastWakeTime;
	
	/* Inicializa canal 1 del ADC1 */
	ADC_ChannelConfTypeDef sConfigN = {0}; // Variable local en la tarea
	sConfigN.Channel = ADC_CHANNEL_1; // selecciona el canal 1
	sConfigN.Rank = 1;
	sConfigN.SamplingTime = ADC_SAMPLETIME_28CYCLES;
	HAL_ADC_ConfigChannel(&hadc1, &sConfigN); // configura ADC1-Canal_1
	for(;;)
	{// Activación de la lectura
		xLastWakeTime = xTaskGetTickCount();
		HAL_ADC_Start(&hadc1); // comienza la conversón AD
		if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK){
			Lectura_ADC1 = HAL_ADC_GetValue(&hadc1); // leemos el valor
			Altitud = Lectura_ADC1; // actualizamos una variable global 
			if (Altitud < ALTITUD_DESTINO - 2 ){
				signal_altitud_perdida = 1;
				
			} 
			else{
				signal_altitud_perdida = 0;
			}
		}
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( T_TAREA_ALTITUD ));
	}
} // fin de TareaN

void TareaVibraciones(void const * argument)
{ 
	int UltimaAltitud = 2147483647;
	int diff = 0;
	TickType_t xLastWakeTime;
	
	double lastX, lastY, lastZ, currentX, currentY, currentZ;
	int counter = 0;
  RefreshAcc(&lastX , &lastY, &lastZ);
	for(;;)
	{
		RefreshAcc(&currentX, &currentY, &currentZ);
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
