/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "MPU6050.h"
#include "motion_fx.h"
#include "motion_fx_cm0p.h"
#include "motion_gc.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define VERSION_STR_LENG 35
//float SAMPLE_FREQUENCY = 50.0f;
#define SAMPLE_FREQUENCY 50.0f
#define MFX_STR_LENG 35
#define STATE_SIZE (size_t)(2450)

char lib_version[VERSION_STR_LENG];
MGC_knobs_t knobs;
MGC_output_t start_gyro_bias;
float sample_freq;
//MGC_input_t data_in;
//MGC_output_t data_out;

int start = 0;
int bias_update;
float gyro_cal_x, gyro_cal_y, gyro_cal_z, corrX, corrY, corrZ;
float freq = (float) SAMPLE_FREQUENCY;
static uint8_t mfxstate[STATE_SIZE];
MFX_input_t data_in;
MFX_output_t data_out;
float dT;
float *q;
float CurrentTime, LastTime;
float accZ = 0;

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_I2C2_Init();

	/* USER CODE BEGIN 2 */

	MPU6050_Initialization();

	start++;
	//MotionGC_Initialize(MGC_MCU_STM32, &freq);
	MotionFX_enable_6X(mfxstate, MFX_ENGINE_ENABLE);
	start++;
	MotionFX_initialize((MFXState_t *)mfxstate);
	start++;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		CurrentTime = HAL_GetTick();
		dT = CurrentTime - LastTime;
		LastTime = CurrentTime;

		start++;
		MPU6050_ProcessData(&MPU6050);

		data_in.gyro[0] = MPU6050.gyro_x;
		data_in.gyro[1] = MPU6050.gyro_y;
		data_in.gyro[2] = MPU6050.gyro_z;
		data_in.acc[0] = MPU6050.acc_x;
		data_in.acc[1] = MPU6050.acc_y;
		data_in.acc[2] = MPU6050.acc_z;
		/* Get acceleration X/Y/Z in g */
		//	MEMS_Read_AccValue(data_in.Acc[0], data_in.Acc[1], data_in.Acc[2]);
		//	/* Get angular rate X/Y/Z in dps */
		//	MEMS_Read_GyroValue(data_in.Gyro[0], data_in.Gyro[1], data_in.Gyro[2]);
		/* Gyroscope calibration algorithm update */
		//MotionGC_Update(&data_in, &data_out, &bias_update);
		MotionFX_propagate(mfxstate, &data_out, &data_in, &dT);
		MotionFX_update(mfxstate, &data_out, &data_in, &dT, NULL);
		/* Apply correction */
//		corrX = data_out.GyroBiasX;
//		corrY = data_out.GyroBiasY;
//		corrZ = data_out.GyroBiasZ;
//		gyro_cal_x = (data_in.Gyro[0] - data_out.GyroBiasX);
//		gyro_cal_y = (data_in.Gyro[1] - data_out.GyroBiasY);
//		gyro_cal_z = (data_in.Gyro[2] - data_out.GyroBiasZ);
		q = data_out.quaternion;

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
