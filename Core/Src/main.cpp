/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : 春ロボ2022のメインプログラム
  * @author         : naokichi
  * @date           : 2022/01/13~
  * TODO ジャイロセンサ
  * TODO LRボタン
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "MPU9250.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t DS3_BTNS_DATMAP_BYTE[16] = {
    2, 2, 2, 2, 2, 2,/* 0, 0,*/ 2, 2, 2, 1, 1, 1, 1, 1
};

uint8_t DS3_BTNS_DATMAP_BITMASK[16] = {
    0x03, 0x0C, 0x03, 0x0C, 0x03, 0x0C,/* 0x00, 0x00,*/
    0x10, 0x40, 0x20, 0x01, 0x04, 0x10, 0x02, 0x08,
};

uint8_t DS3_BTNS_DATMAP_FLAGBIT[16] = {
    0x01, 0x04, 0x02, 0x08, 0x03, 0x0C,/* 0x00, 0x00,*/
    0x10, 0x40, 0x20, 0x01, 0x04, 0x10, 0x02, 0x08,
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU9250 mpu9250;

uint8_t SBDBTRxData[8];
int buttonsIsPressed[16] = {0};
int LX = 64;
int LY = 64;
int RX = 64;
int RY = 64;

float theta = 45;

float vx_r = 0;
float vy_r = 0;
float omega_r = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Analyze_Raw_Data();
void Swerve();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	setbuf(stdout, NULL);

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
  MX_CAN1_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM7_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  LL_TIM_EnableCounter(TIM2);
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH3);
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH4);
  LL_TIM_OC_SetCompareCH1(TIM2, 1000);
  LL_TIM_OC_SetCompareCH2(TIM2, 1000);
  LL_TIM_OC_SetCompareCH3(TIM2, 1000);
  LL_TIM_OC_SetCompareCH4(TIM2, 1000);

  LL_TIM_EnableCounter(TIM5);
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_SetCompareCH1(TIM5, 1000);
  LL_TIM_OC_SetCompareCH2(TIM5, 1000);

  LL_TIM_EnableCounter(TIM12);
  LL_TIM_CC_EnableChannel(TIM12, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM12, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_SetCompareCH1(TIM12, 500);
  LL_TIM_OC_SetCompareCH2(TIM12, 500);

  LL_TIM_EnableCounter(TIM13);
  LL_TIM_CC_EnableChannel(TIM13, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_SetCompareCH1(TIM13, 500);

  LL_TIM_EnableCounter(TIM14);
  LL_TIM_CC_EnableChannel(TIM14, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_SetCompareCH1(TIM14, 500);

  mpu9250.init(hspi3, SPI3_CS_GPIO_Port, SPI3_CS_Pin, ACCEL_RANGE_16G, GYRO_RANGE_1000dps);
  HAL_UART_Receive_IT(&huart1, SBDBTRxData, 8);
  printf("Calibration in progress...\r\n");
  mpu9250.calibration();
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

  LL_TIM_EnableCounter(TIM7);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  LL_TIM_SetCounter(TIM7, 0);

	  mpu9250.update9DOF();
	  theta += mpu9250.gz*0.0123f;		// とりあえず0.0123
	  //printf("%f\r\n", theta);

	  vx_r = (RX-64) * 7.0f;	// とりあえずスティックの値の7倍
	  vy_r = (64-RY) * 7.0f;	// とりあえずスティックの値の7倍
	  //printf("%f\r\n", sqrtf(vx_r*vx_r+vy_r*vy_r));

	  if(buttonsIsPressed[12]){
		  omega_r = 1000.0f;	// とりあえず1000
	  }else if(buttonsIsPressed[13]){
		  omega_r = -1000.0f;	// とりあえず-1000
	  }else{
		  omega_r = 0;
	  }

	  if(buttonsIsPressed[1]){
		  LL_TIM_OC_SetCompareCH1(TIM5, 1400);
	  }else if(buttonsIsPressed[3]){
		  LL_TIM_OC_SetCompareCH1(TIM5, 600);
	  }else{
		  LL_TIM_OC_SetCompareCH1(TIM5, 1000);
	  }

	  Swerve();

	  int count = LL_TIM_GetCounter(TIM7);
	  printf("%d[us]\r\n", count);

	  HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
extern "C" int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 10);
    return len;
}

/**
  * @brief UARTの受信コールバック
  * @note  一度に8バイト受信する
  */
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    HAL_UART_Receive_IT(&huart1, SBDBTRxData, 8);
    Analyze_Raw_Data();
}

/**
  * @brief SBDBTから送られてきたデータを解析する
  * @param[out] buttonIsPressed コントローラのボタンが押されているか
  * @param[out] LX, LY, RX, RY  コントローラのスティックの状態
  */
void Analyze_Raw_Data()
{
    for(int i=0; i<16; i++){
    	int f = (SBDBTRxData[DS3_BTNS_DATMAP_BYTE[i]] & DS3_BTNS_DATMAP_BITMASK[i]) == DS3_BTNS_DATMAP_FLAGBIT[i];
        buttonsIsPressed[i] = f;
    }
    LX = SBDBTRxData[3];
    LY = SBDBTRxData[4];
    RX = SBDBTRxData[5];
    RY = SBDBTRxData[6];
}

/**
  * @brief Swerve
  * @param[in] vx_r x軸速度の目標値
  * @param[in] vy_r y軸速度の目標値
  * @param[in] omega_r 回転の角速度の目標値
  * @note esaのSwerveDriveの記事を参照してください
  */
void Swerve()
{
	const float R = 0.283f;		// ロボット中心からホイールまでの距離[m]

	float vx[4];				// i番目のホイールのx軸速度
	float vy[4];				// i番目のホイールのy軸速度
	float v[4];					// i番目のホイールの速さ
	float steering_angle[4];	// i番目のホイールのステアリング角度

	for(int i=0; i<4; i++){
		vx[i] = vx_r - R*omega_r*sinf((theta + i*90.0f) * (M_PI / 180.0f));
		//vx[i] = vx_r;
		vy[i] = vy_r + R*omega_r*cosf((theta + i*90.0f) * (M_PI / 180.0f));
		//vy[i] = vy_r;

		v[i] = sqrtf(vx[i]*vx[i] + vy[i]*vy[i]);

		steering_angle[i] = atan2f(vy[i], vx[i]);
		steering_angle[i] *= 180.0f / M_PI;	// [rad]→[deg]
		steering_angle[i] += 45.0f - theta;	// ローカル座標に変換
		steering_angle[i] = fmodf(steering_angle[i], 180.0f);	// -180°~180°にする
		if(steering_angle[i] < 0){			// 負の時は
			steering_angle[i] += 180.0f;	// 反対向きにして
			v[i] *= -1;						// 逆転する
		}
		//printf("%d:%f ", i, steering_angle[i]);
		steering_angle[i] *= 2000.0f / 202.5f;	// [deg]→[サーボモータへの指令値]
	}
	//printf("\r\n");

	if(buttonsIsPressed[0]){
		LL_TIM_OC_SetCompareCH1(TIM2, 1000 + (int)v[0]);
		LL_TIM_OC_SetCompareCH1(TIM12, 500 + (int)steering_angle[0]);

		LL_TIM_OC_SetCompareCH2(TIM2, 1000 + (int)v[1]);
		LL_TIM_OC_SetCompareCH2(TIM12, 500 + (int)steering_angle[1]);

		LL_TIM_OC_SetCompareCH3(TIM2, 1000 + (int)v[2]);
		LL_TIM_OC_SetCompareCH1(TIM13, 500 + (int)steering_angle[2]);

		LL_TIM_OC_SetCompareCH4(TIM2, 1000 + (int)v[3]);
		LL_TIM_OC_SetCompareCH1(TIM14, 500 + (int)steering_angle[3]);
	}else{
		LL_TIM_OC_SetCompareCH1(TIM2, 1000);
		LL_TIM_OC_SetCompareCH1(TIM12, 500 + (int)steering_angle[0]);
		//LL_TIM_OC_SetCompareCH1(TIM12, 500);

		LL_TIM_OC_SetCompareCH2(TIM2, 1000);
		LL_TIM_OC_SetCompareCH2(TIM12, 500 + (int)steering_angle[1]);
		//LL_TIM_OC_SetCompareCH2(TIM12, 500);

		LL_TIM_OC_SetCompareCH3(TIM2, 1000);
		LL_TIM_OC_SetCompareCH1(TIM13, 500 + (int)steering_angle[2]);
		//LL_TIM_OC_SetCompareCH1(TIM13, 500);

		LL_TIM_OC_SetCompareCH4(TIM2, 1000);
		LL_TIM_OC_SetCompareCH1(TIM14, 500 + (int)steering_angle[3]);
		//LL_TIM_OC_SetCompareCH1(TIM14, 500);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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

