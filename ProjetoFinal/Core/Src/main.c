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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "bitmap.h"
#include "mpu6050.h"
#include "stdio.h"
#include "bmp280.h"
#include <math.h>

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
float temperatura, pressao, altitude; // variáveis para utilizar no sensor de pressão e temperatura BMP280

// os buffers realizam a conversão dos dados para string, para que sejam passados para o display OLED
char Buffer1[32];
char Buffer2[32];
char Buffer3[32];
char Buffer4[32];

// variáveis para controle de feedback do sistema
uint8_t tela=0;
uint8_t flagit=0, flagBmp=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void System_Start(void);
void Screens();
void Update_MPU6050();
void Update_BMP_HCSR04();
static void alerta();
static void Pos_Servo();
static float Ler_Nivel();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
BMP280_HandleTypedef bmp280;
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // inicialização do sensor de pressão e temperatura BMP280
  bmp280_init_default_params(&bmp280.params);
  bmp280.addr = BMP280_I2C_ADDRESS_0;
  bmp280.i2c = &hi2c1;
  while(!bmp280_init(&bmp280, &bmp280.params))

  SSD1306_Init (); // inicialização do display OLED
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // inicialização de PWM
  HAL_ADC_Start(&hadc1); // inicialização de ADC
  while (MPU6050_Init(&hi2c1) == 1); //Inicialização do MPU6050
  System_Start(); // inicialização do sistema

  HAL_TIM_Base_Start(&htim3); // inicialização de timer
  HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);  // garantia de que o TRIG do HC-SR04 inicie com valor baixo

  HAL_TIM_Base_Start_IT(&htim6); // inicialização de timer em modo de interrupção
  HAL_TIM_Base_Start_IT(&htim2); // inicialização de timer em modo de interrupção

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if(flagit==1){
		Update_MPU6050();
	}

	else if(flagBmp==1){
		Update_BMP_HCSR04();
	}

	Pos_Servo();
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

// função para a organização das telas no display OLED
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
		SSD1306_Puts ("Tmp:",&Font_7x10, 1);
		SSD1306_DrawCircle(46, 54, 2, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY (49,53);
		SSD1306_Puts ("C",&Font_7x10, 1);

		SSD1306_GotoXY (64,53);
		SSD1306_Puts ("P:",&Font_7x10, 1);
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
		SSD1306_Puts ("Tmp:",&Font_7x10, 1);
		SSD1306_DrawCircle(46, 54, 2, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY (49,53);
		SSD1306_Puts ("C",&Font_7x10, 1);

		SSD1306_GotoXY (64,53);
		SSD1306_Puts ("P:",&Font_7x10, 1);
		SSD1306_GotoXY (99,53);
		SSD1306_Puts ("kPa",&Font_7x10, 1);
		SSD1306_UpdateScreen();
	}

	if(tela ==2){
		SSD1306_Clear();
		SSD1306_GotoXY (0,0);
		SSD1306_Puts ("Angulo & Altitude:",&Font_7x10, 1);


		SSD1306_GotoXY (0,17);
		SSD1306_Puts ("X:",&Font_7x10, 1);
		SSD1306_GotoXY (65,17);
		SSD1306_DrawCircle(74, 18, 2, SSD1306_COLOR_WHITE);

		SSD1306_GotoXY (0,29);
		SSD1306_Puts ("Y:",&Font_7x10, 1);
		SSD1306_DrawCircle(74, 30, 2, SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();

		SSD1306_GotoXY (0,41);
		SSD1306_Puts ("h:",&Font_7x10, 1);
		SSD1306_GotoXY (74,41);
		SSD1306_Puts ("m",&Font_7x10, 1);
		SSD1306_UpdateScreen();

		SSD1306_GotoXY (0,53);
		SSD1306_Puts ("Tanque:",&Font_7x10, 1);
		SSD1306_GotoXY (81,53);
		SSD1306_Puts ("L",&Font_7x10, 1);
	}
}

// função para a inicialização do sistema implementado
void System_Start(void){
	SSD1306_Init ();
	SSD1306_Clear();
	SSD1306_DrawBitmap(0, 0, aviao, 128, 64, SSD1306_COLOR_WHITE);
	SSD1306_UpdateScreen();
	HAL_Delay(2000);
	tela=0;
	Screens();
}

// função de interrupção externa do botão
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == B1_Pin){
		tela = (tela+1)%3;
		Screens();
		Update_BMP_HCSR04();
	}
}

// função de callback I2C para o sensor MPU6050 via DMA
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
	if (hi2c->Instance == I2C1){
	    MPU6050_Process_Data(&MPU6050);
	}
}

// função de callback com timer 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2){
		flagit=1;
	}

	if (htim->Instance == TIM6){
		flagBmp=1;
	}
}

// Atualiza o sensor BMP280 e o nível do líquido a partir da medição através do HC-SR04
void Update_BMP_HCSR04(){
	double volume = Ler_Nivel(); // variável que recebe o valor da medição atráves do HC-SR04

	// estrutura condicional para evitar leituras negativas (errôneas) do sensor ultrassônico
	if(volume < 0){
		volume = 0;
	}

	bmp280_read_float(&bmp280, &temperatura, &pressao, NULL); // função que faz a leitura das medições do sensor BMP280
	altitude = 44330.0 * (1.0 - pow((pressao / (100*1013.25)), 0.1903)); // cálculo da altitude a partir da leitura do sensor BMP280
	sprintf(Buffer1, "%.0f", temperatura);
	sprintf(Buffer2, "%.0f", pressao/1000);
	sprintf(Buffer3, "%.1f", altitude);
	sprintf(Buffer4, "%.2f", volume);

	// atualiza a altitude e o nível do líquido na tela 3
	if(tela==2){
		SSD1306_GotoXY (21,41);
		SSD1306_Puts (Buffer3,&Font_7x10, 1);
		SSD1306_GotoXY (49,53);
		SSD1306_Puts (Buffer4,&Font_7x10, 1);
	}

	// atualiza temperatura e pressão nas telas
	else{
		SSD1306_GotoXY (28,53);
		SSD1306_Puts (Buffer1,&Font_7x10, 1);
		SSD1306_GotoXY (78,53);
		SSD1306_Puts (Buffer2,&Font_7x10, 1);
	}
	SSD1306_UpdateScreen();
	flagBmp=0;
}

// função que atualiza as medidas em cada tela e emite alertas sonoros através do buzzer
void Update_MPU6050(){
	MPU6050_Read_All(&hi2c1);

	if(temperatura > 27){
		alerta();
	}

	else{
		HAL_GPIO_WritePin(GPIOB, BUZZER_Pin, 0);
		HAL_GPIO_WritePin(GPIOA, LD2_Pin, 0);
	}

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

	else if(tela ==1){
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

	else if(tela ==2){
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

// função que gerencia o controle do servo-motor a partir da leitura com ADC de um potenciômetro
static void Pos_Servo(void){
    static float arr_suavizado =  460;  // Valor inicial

    uint32_t leitura = HAL_ADC_GetValue(&hadc1);
    float arr;

    // Calcula o valor arr diretamente com base na leitura do ADC
    arr = (1900.0 / 3.3) * (leitura * 3.3 / 4095.0) + 460;

    // Aplica suavização exponencial (filtro de baixa frequência)
    arr_suavizado = arr_suavizado * 0.85 + arr * 0.1;  // Fator de suavização (0.9 e 0.1)

    // Atualiza o PWM com o valor suavizado
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)arr_suavizado);
}

// função que emite alertas sonoros (através do buzzer) e, além disso, luminosos, utilizando o LED da F446RE
static void alerta(void){
	static uint8_t contador = 0; // contador para controle dos ticks do buzzer
	HAL_GPIO_TogglePin(GPIOA, LD2_Pin);

	if(contador == 7){
		HAL_GPIO_WritePin(GPIOB, BUZZER_Pin, 1);
	}

	else if(contador == 15){
		HAL_GPIO_WritePin(GPIOB, BUZZER_Pin, 0);
		contador = 0;
	}

	contador++;
}

// função que implementa a leitura do nível de líquido que foi consumido e retorna o volume restante no tanque
static float Ler_Nivel(){
	static uint32_t tmili;
	static uint32_t Valor1 = 0;
	static uint32_t Valor2 = 0;
	static uint16_t Distancia  = 0;
	const double capacidade = 28.4; // altura em cm
	const double raio = 3.75; // diametro em cm
	double volume;
	double restante;
	const double PI = 3.141592;

	HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_SET);  // TRIG com valor alto para enviar o pulso sonoro
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	while (__HAL_TIM_GET_COUNTER (&htim3) < 10);  // espera de 10 milissegundos de acordo com o datasheet
	HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);  // TRIG com valor baixo para parar de enviar o pulso sonoro

	tmili = HAL_GetTick(); // evita loop infinito
	while (!(HAL_GPIO_ReadPin (ECHO_GPIO_Port, ECHO_Pin)) && tmili + 10 >  HAL_GetTick()); // espera até o ECHO tem valor alto
	Valor1 = __HAL_TIM_GET_COUNTER (&htim3); // recebe o instante de tempo em que o ECHO recebe valor alto

	tmili = HAL_GetTick(); // evita loop infinito

	while ((HAL_GPIO_ReadPin (ECHO_GPIO_Port, ECHO_Pin)) && tmili + 50 > HAL_GetTick()); // espera até o ECHO ter valor baixo
	Valor2 = __HAL_TIM_GET_COUNTER (&htim3); // recebe o instante de tempo em que o ECHO recebeu valor baixo

	Distancia = (Valor2-Valor1)* 0.0343/2; // cálculo da distância

	restante = capacidade - Distancia; // cálculo do nível de líquido restante
	volume = PI * raio * raio * restante/1000; // cálculo do volume de líquido restante
	return(volume);
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
