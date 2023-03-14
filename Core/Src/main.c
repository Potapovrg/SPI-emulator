/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "exec_time.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	uint8_t target;
	uint8_t button;
	int8_t mouse_x;
	int8_t mouse_y;
	int8_t wheel;
	uint8_t modifier;
	uint8_t reserved;
	uint8_t keycode1;
	uint8_t keycode2;
	uint8_t keycode3;
	uint8_t keycode4;
	uint8_t keycode5;
	uint8_t keycode6;
} bufferSPI;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEST
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t kbdPress[9] = {0x02,0,0,0x10,0x0E,0x08,0x28,0,0};
uint8_t kbdRelease[9] = {0x02,0,0,0,0,0,0,0,0};

uint8_t spi_transmit_buffer[9]= {0x01,0,0,0,0,0,0,0,0};

uint8_t spi_dummy_buffer[9]= {0x01,0,0,0,0,0xFF,0,0,0};
uint8_t spi_receive_buffer[9]= {0,0,0,0,0,0,0,0,0};

bufferSPI otg_on={0b00000100,0x00,0,0,0,0b00000000,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
bufferSPI otg_off={0b00000000,0x00,0,0,0,0b00000000,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
bufferSPI check={0b00001000,0x00,0,0,0,0b00000000,0x00,0x00,0x00,0x00,0x00,0x00,0xFF};
bufferSPI crc_check={0b00000000,0x00,0,0,0,0b00000000,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

bufferSPI testbuf_1[4]={
		{0b00000111,0x00,50,0,0,0b00000000,0x00,0x10,0x0E,0x08,0x28,0x00,0x00},
		{0b00000111,0x00,0,-50,0,0b00000000,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0b00000111,0x00,-50,0,0,0b00100000,0x00,0x10,0x0E,0x08,0x28,0x00,0x00},
		{0b00000111,0x00,0,50,0,0b00000000,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
					};
bufferSPI testbuf[4]={
		{0b00000111,0x00,50,0,0,0b00000000,0x00,0x10,0x0E,0x08,0x28,0x00,0x00},
		{0b00000111,0x00,0,-50,0,0b00000000,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
		{0b00000111,0x00,-50,0,0,0b00100000,0x00,0x10,0x0E,0x08,0x28,0x00,0x00},
		{0b00000111,0x00,0,50,0,0b00000000,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
					};//MKE Test

bufferSPI testbuf_3[4]={
		{0b00001001,0x00,0,0,0,0b00000000,0x00,0x00,0x00,0x00,0x00,0x00,0xFF},
		{0b00001010,0x00,0,0,0,0b00000000,0x00,0x00,0x00,0x00,0x00,0x00,0xFF},
		{0b00001100,0x00,0,0,0,0b00000000,0x00,0x00,0x00,0x00,0x00,0x00,0xFF},
		{0b00001111,0x00,0,0,0,0b00000000,0x00,0x00,0x00,0x00,0x00,0x00,0xFF},
					};
bufferSPI testbuf_4[4]={
		{0b00000001,0x00,0,0,0,0b00000000,0x00,0x00,0x00,0x00,0x00,0x00,0xFF},
		{0b00000010,0x00,0,0,0,0b00000000,0x00,0x00,0x00,0x00,0x00,0x00,0xFF},
		{0b00000100,0x00,0,0,0,0b00000000,0x00,0x00,0x00,0x00,0x00,0x00,0xFF},
		{0b00000111,0x00,0,0,0,0b00000000,0x00,0x00,0x00,0x00,0x00,0x00,0xFF},
					};

bufferSPI testbuf_5[4]={
		{0b00000101,0x00,0,0,0,0b00000000,0x00,0x00,0x00,0x00,0x00,0x00,0xFF},
		{0b00000000,0x00,0,0,0,0b00000000,0x00,0x00,0x00,0x00,0x00,0x00,0xFF},
		{0b00000110,0x00,0,0,0,0b00000000,0x00,0x00,0x00,0x00,0x00,0x00,0xFF},
		{0b00000000,0x00,0,0,0,0b00000000,0x00,0x00,0x00,0x00,0x00,0x00,0xFF},
					};//ADB-OTG Test

uint8_t rcv_buf=0;
uint8_t crc8=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void test_1(void);
void test_2(void);
uint8_t CRC_Calculate_software(uint8_t *Data, uint8_t Buffer_lenght);
float exec_time;
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
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  test_1();
	  //start_exec_time();
	  //HAL_Delay(1000);
	  //exec_time=stop_exec_time();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void test_1(void)
{
	//HAL_SPI_Transmit(&hspi1,&spi_dummy_buffer,sizeof(spi_dummy_buffer),10);
	//HAL_Delay(50);
	for (uint8_t i=0;i<4;i++)
	{
#ifdef TEST
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
#endif
		//rcv_buf=0;
		//start_exec_time();
		HAL_SPI_Transmit(&hspi1,&testbuf[i],sizeof(testbuf[i]),10);
		//exec_time=stop_exec_time_float();
		//exec_time=stop_exec_time();
		crc8=CRC_Calculate_software(&testbuf[i],sizeof(testbuf[i]));
		HAL_Delay(10);
		//HAL_SPI_Transmit(&hspi1,&check,sizeof(check),10);
		HAL_SPI_TransmitReceive(&hspi1,&check,&spi_receive_buffer,sizeof(check),10);
		HAL_Delay(500);

	}
}
void test_2(void)
{
	uint8_t testvar=0b11000001;
	HAL_SPI_Transmit(&hspi1,&testvar,sizeof(testvar),10);
	crc8=CRC_Calculate_software(&testvar,sizeof(testvar));
	HAL_Delay(10);

}

uint8_t CRC_Calculate_software(uint8_t *Data, uint8_t Buffer_lenght) {
	uint8_t CRC8 = 0x00;
	uint8_t size = (sizeof(*Data));
	while (Buffer_lenght--) {
		CRC8 ^= *Data++;
		for (uint8_t i = 0; i < (sizeof(*Data) * 8); i++) {
			if (CRC8 & 0x80) {
				CRC8 = (CRC8 << 1u) ^ 07;
			} else {
				CRC8 = (CRC8 << 1u);
			}
		}
	}
	return CRC8;
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
