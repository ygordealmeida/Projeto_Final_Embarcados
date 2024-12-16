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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "bitmap.h"
#include "mpu6050.h"
#include "stdio.h"
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
MPU6050_t MPU6050;

//uint8_t StartMSG1[] = " \n";
char Buffer1[32];
char Buffer2[32];
char Buffer3[32];
uint8_t tela=0;
uint8_t flagit=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void System_Start(void);
void Screens();
void Update_Screen();
static void Pos_Servo();
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  SSD1306_Init ();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_ADC_Start(&hadc1);
  while (MPU6050_Init(&hi2c1) == 1);
  System_Start();
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(flagit==1){
		  Update_Screen();
	  }
	  Pos_Servo();
	  HAL_Delay(10);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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


void Screens(){

	if(tela ==0){

		SSD1306_Clear();
		SSD1306_GotoXY (0,0);
		SSD1306_Puts ("Giroscopio:",&Font_7x10, 1);

		SSD1306_GotoXY (0,17);
		SSD1306_Puts ("X:",&Font_7x10, 1);
		SSD1306_DrawCircle(74, 18, 2, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY (77,17);
		SSD1306_Puts ("/s",&Font_7x10, 1);


		SSD1306_GotoXY (0,29);
		SSD1306_Puts ("Y:",&Font_7x10, 1);
		SSD1306_DrawCircle(74, 30, 2, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY (77,29);
		SSD1306_Puts ("/s",&Font_7x10, 1);

		SSD1306_GotoXY (0,41);
		SSD1306_Puts ("Z:",&Font_7x10, 1);
		SSD1306_DrawCircle(74, 42, 2, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY (77,41);
		SSD1306_Puts ("/s",&Font_7x10, 1);

		SSD1306_GotoXY (0,53);
		SSD1306_Puts ("Tmp:22",&Font_7x10, 1);
		SSD1306_DrawCircle(53-7, 53, 2, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY (56-7,53);
		SSD1306_Puts ("C",&Font_7x10, 1);

		SSD1306_GotoXY (64,53);
		SSD1306_Puts ("P:103",&Font_7x10, 1);
		SSD1306_GotoXY (99,53);
		SSD1306_Puts ("kPa",&Font_7x10, 1);
		SSD1306_UpdateScreen();
	}

	if(tela ==1){
			SSD1306_Clear();
			SSD1306_GotoXY (0,0);
			SSD1306_Puts ("Acelerometro:",&Font_7x10, 1);

			SSD1306_GotoXY (0,17);
			SSD1306_Puts ("X:",&Font_7x10, 1);
			SSD1306_GotoXY (65,17);
			SSD1306_Puts ("g",&Font_7x10, 1);


			SSD1306_GotoXY (0,29);
			SSD1306_Puts ("Y:",&Font_7x10, 1);
			SSD1306_GotoXY (65,29);
			SSD1306_Puts ("g",&Font_7x10, 1);

			SSD1306_GotoXY (0,41);
			SSD1306_Puts ("Z:",&Font_7x10, 1);
			SSD1306_GotoXY (65,41);
			SSD1306_Puts ("g",&Font_7x10, 1);

			SSD1306_GotoXY (0,53);
			SSD1306_Puts ("Tmp:22",&Font_7x10, 1);
			SSD1306_DrawCircle(46, 54, 2, SSD1306_COLOR_WHITE);
			SSD1306_GotoXY (49,53);
			SSD1306_Puts ("C",&Font_7x10, 1);

			SSD1306_GotoXY (64,53);
			SSD1306_Puts ("P:103",&Font_7x10, 1);
			SSD1306_GotoXY (99,53);
			SSD1306_Puts ("kPa",&Font_7x10, 1);
			SSD1306_UpdateScreen();
		}

	if(tela ==2){
		SSD1306_Clear();
		SSD1306_GotoXY (0,0);
		SSD1306_Puts ("Inclinacao:",&Font_7x10, 1);


		SSD1306_GotoXY (0,17);
		SSD1306_Puts ("X:",&Font_7x10, 1);
		SSD1306_GotoXY (65,17);
		SSD1306_DrawCircle(74, 18, 2, SSD1306_COLOR_WHITE);

		SSD1306_GotoXY (0,29);
		SSD1306_Puts ("Y:",&Font_7x10, 1);
		SSD1306_DrawCircle(74, 30, 2, SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
	}


}

void System_Start(void){
	SSD1306_Init ();
	SSD1306_Clear();
	SSD1306_DrawBitmap(0, 0, aviao, 128, 64, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();
	HAL_Delay(2000);
	tela=0;
	Screens();

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == B1_Pin){
		tela = (tela+1)%3;
		Screens();
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){

	if (hi2c->Instance == I2C1)
	    {

	        MPU6050_Process_Data(&MPU6050);
	    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	flagit=1;

}


void Update_Screen(){
	MPU6050_Read_All(&hi2c1);

	if(tela ==0){
		flagit=0;
		sprintf(Buffer1, "%.2f", MPU6050.Gx);
		sprintf(Buffer2, "%.2f", MPU6050.Gy);
		sprintf(Buffer3, "%.2f", MPU6050.Gz);
		SSD1306_GotoXY (21,17);
		SSD1306_Puts (Buffer1,&Font_7x10, 1);
		SSD1306_GotoXY (21,29);
		SSD1306_Puts (Buffer2,&Font_7x10, 1);
		SSD1306_GotoXY (21,41);
		SSD1306_Puts (Buffer3,&Font_7x10, 1);
		SSD1306_UpdateScreen();
	}

	if(tela ==1){
		flagit=0;
		sprintf(Buffer1, "%.2f", MPU6050.Ax);
		sprintf(Buffer2, "%.2f", MPU6050.Ay);
		sprintf(Buffer3, "%.2f", MPU6050.Az);
		SSD1306_GotoXY (21,17);
		SSD1306_Puts (Buffer1,&Font_7x10, 1);
		SSD1306_GotoXY (21,29);
		SSD1306_Puts (Buffer2,&Font_7x10, 1);
		SSD1306_GotoXY (21,41);
		SSD1306_Puts (Buffer3,&Font_7x10, 1);
		SSD1306_UpdateScreen();
			}

	if(tela ==2){
		flagit=0;
		sprintf(Buffer1, "%.2f", MPU6050.KalmanAngleX);
		sprintf(Buffer2, "%.2f", MPU6050.KalmanAngleY);
		SSD1306_GotoXY (21,17);
		SSD1306_Puts (Buffer1,&Font_7x10, 1);
		SSD1306_GotoXY (21,29);
		SSD1306_Puts (Buffer2,&Font_7x10, 1);
		SSD1306_UpdateScreen();
	}
}


static void Pos_Servo(void){
	uint32_t leitura = HAL_ADC_GetValue(&hadc1);

	//uint32_t cast = leitura*3.3/4095;
	//uint16_t arr = 2000*cast/3.3;

	uint16_t arr = (1900/3.3)*(leitura*3.3/4095) + 700;
			//2000*leitura/4095;
	//static uint16_t arr = 100;

	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,arr);
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
