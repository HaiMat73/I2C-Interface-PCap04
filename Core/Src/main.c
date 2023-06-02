/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "user_i2c_interface.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WR_MEM 		0xA0
#define RD_MEM 		0x20
#define WR_CONFIG 	0xA3C0	//byte wise
#define RD_CONFIG 	0x23C0	//byte wise
#define RD_RESULT 	0x40

#define POR 		0x88
#define INIT 		0x8A
#define CDC_START 	0x8C
#define RDC_START 	0x8E
#define DSP_TRIG 	0x8D
#define NV_STORE 	0x96
#define NV_RECALL 	0x99
#define NV_ERASE 	0x9C
#define TEST_READ 	0x7E
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

LPTIM_HandleTypeDef hlptim1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

HAL_StatusTypeDef ret = HAL_OK;

volatile uint16_t dev_addr = 0x50;
volatile uint32_t MyResult = 0;
volatile uint32_t MyRES0 = 0;
volatile uint32_t MyRES1 = 0;
volatile uint32_t MyRES2 = 0;

uint32_t My_i2c_timeout = 10;
uint8_t My_buf[256] = {0x99,0x22};
uint32_t My_ratio = 0;

uint32_t bending_cfg[13] = {
		0x1038111D,
		0x08110CA0,
		0x0007D000,
		0x04000427,
		0x00005427,
		0x340BFF00,
		0x08520077,
		0x0881005A,
		0x40470008,
		0x71000000,
		0x00080000,
		0x01000000,
		0x00000000

};

//read triggered configuration
uint8_t bending_cfg_bytewise[52] = {
		0x1D, 0x11, 0x38, 0x10,
		0xA0, 0x0C, 0x11, 0x08,
		0x00, 0xD0, 0x07, 0x00,
		0x27, 0x04, 0x00, 0x04,
		0x27, 0x54, 0x00, 0x00,
		0x00, 0xFF, 0x0B, 0x34,
		0x77, 0x00, 0x52, 0x08,
		0x5A, 0x00, 0x81, 0x08,
		0x08, 0x00, 0x47, 0x40,
		0x00, 0x00, 0x00, 0x71,
		0x00, 0x00, 0x08, 0x00,
		0x00, 0x00, 0x00, 0x01,
		0x00, 0x00, 0x00, 0x00
};

uint8_t standard_cfg_bytewise[52] = {
		0x1D, 0x00, 0x58, 0x10,
		0x10, 0x00, 0x0F, 0x20,
		0x00, 0xD0, 0x07, 0x00,
		0x00, 0x08, 0xFF, 0x03,
		0x00, 0x24, 0x00, 0x00,
		0x00, 0x01, 0x50, 0x30,
		0x73, 0x04, 0x50, 0x08,
		0x5A, 0x00, 0x82, 0x08,
		0x08, 0x00, 0x47, 0x40,
		0x00, 0x00, 0x00, 0x71,
		0x00, 0x00, 0x08, 0x00,
		0x00, 0x00, 0x00, 0x01,
		0x00, 0x00, 0x00, 0x00
};

uint8_t bending_fw[167] = {
		0x24, 0x05, 0x7A, 0xC3, 0xFF, 0xE6, 0x43, 0x7A, 0xE0, 0x34, 0x00, 0x7A, 0xE7, 0x43, 0x41, 0xA0,
		0x01, 0x20, 0x80, 0x7A, 0xD8, 0x41, 0x7A, 0xE1, 0x45, 0x7A, 0xDC, 0x41, 0x7A, 0xE2, 0x45, 0x7A,
		0xDE, 0x41, 0x20, 0x95, 0x7A, 0xE3, 0x45, 0x7A, 0xD8, 0x41, 0x7A, 0xDE, 0x52, 0x7A, 0xE4, 0x45,
		0x7A, 0xDC, 0x41, 0x7A, 0xDE, 0x52, 0x7A, 0xE5, 0x45, 0x7A, 0xD8, 0x41, 0x7A, 0xDE, 0x5A, 0x7A,
		0xE4, 0x7A, 0xE6, 0x45, 0x7A, 0xDC, 0x41, 0x7A, 0xDE, 0x5A, 0x7A, 0xE7, 0x45, 0x7A, 0xD8, 0x41,
		0x7A, 0xDC, 0x5A, 0x7A, 0xE8, 0x45, 0xC1, 0xD8, 0x24, 0x3E, 0x84, 0x2C, 0x7A, 0xD4, 0x43, 0x7A,
		0xE5, 0x44, 0x7A, 0xD5, 0x43, 0x7A, 0xE6, 0x44, 0x7A, 0xD6, 0x43, 0x7A, 0xE7, 0x44, 0x7A, 0xD7,
		0x43, 0x7A, 0xE8, 0x44, 0xC1, 0xD4, 0x24, 0x57, 0x7A, 0xC8, 0xE1, 0x43, 0xE0, 0x3A, 0x92, 0x08,
		0x7A, 0xC0, 0xE1, 0x43, 0x7A, 0xE0, 0x3A, 0x02, 0x01, 0x90, 0x00, 0x7F, 0x66, 0x67, 0x76, 0x77,
		0x66, 0x99, 0xFD, 0x7B, 0x01, 0xC0, 0xCF, 0xDF, 0x27, 0x0C, 0x6A, 0xCE, 0x41, 0x6A, 0xCF, 0x52,
		0x52, 0x6A, 0xD0, 0x52, 0x47, 0x47, 0x02
};

uint8_t standard_fw[548] = {
		0x24, 0x05, 0xA0, 0x01, 0x20, 0x55, 0x42, 0x5C, 0x48, 0xB1, 0x07, 0x92, 0x02, 0x20, 0x13, 0x02,
		0x20, 0x93, 0x02, 0xB2, 0x02, 0x78, 0x20, 0x54, 0xB3, 0x06, 0x91, 0x00, 0x7F, 0x20, 0x86, 0x20,
		0x54, 0xB6, 0x03, 0x72, 0x62, 0x20, 0x54, 0xB7, 0x00, 0x00, 0x42, 0x5C, 0xA1, 0x00, 0x49, 0xB0,
		0x00, 0x49, 0x40, 0xAB, 0x5D, 0x92, 0x1C, 0x90, 0x02, 0x7F, 0x20, 0x86, 0x66, 0x67, 0x76, 0x77,
		0x66, 0x7A, 0xCF, 0xCD, 0xE6, 0x43, 0xF1, 0x44, 0x29, 0xE0, 0x7A, 0xDC, 0xE7, 0x41, 0x32, 0xAA,
		0x01, 0x99, 0xFD, 0x7B, 0x01, 0x7A, 0xCF, 0xEB, 0xE6, 0x43, 0xF1, 0x44, 0x29, 0xE0, 0x7A, 0xC1,
		0xE7, 0x41, 0x32, 0x6A, 0xDE, 0x44, 0x7A, 0xCF, 0xEA, 0xE6, 0x43, 0xF1, 0x44, 0x29, 0xE0, 0x6A,
		0xDF, 0x44, 0x7A, 0xC4, 0xE7, 0x41, 0x32, 0xAB, 0x05, 0x7A, 0xC1, 0xE1, 0x43, 0xE0, 0x3A, 0x7A,
		0xC0, 0xE1, 0x43, 0xE0, 0x3A, 0x02, 0x7A, 0xCF, 0xE6, 0xE6, 0x43, 0xF1, 0x44, 0x29, 0xE0, 0x7A,
		0xEF, 0x44, 0x02, 0x20, 0x9D, 0x84, 0x01, 0x21, 0x2E, 0x21, 0x74, 0x20, 0x37, 0xC8, 0x7A, 0xE7,
		0x43, 0x49, 0x11, 0x6A, 0xD4, 0x44, 0x7A, 0xC1, 0xD8, 0xE6, 0x43, 0xE9, 0x44, 0x1C, 0x43, 0x13,
		0xAB, 0x63, 0x6A, 0xDE, 0x41, 0xAB, 0x0B, 0x46, 0x46, 0x46, 0x7A, 0xDF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xE3, 0x41, 0x32, 0x1C, 0x44, 0xE9, 0x13, 0x6A, 0xD4, 0x13, 0x41, 0xAA, 0xDF, 0x7A, 0xC5, 0xE1,
		0x43, 0x49, 0xE0, 0x34, 0x7A, 0xCF, 0xE3, 0xE6, 0x43, 0xF1, 0x44, 0x29, 0xE0, 0xDB, 0xC0, 0x27,
		0xE5, 0x6A, 0xDF, 0x43, 0x7A, 0xC8, 0xE7, 0x41, 0x30, 0xAB, 0x03, 0x86, 0x01, 0x92, 0x37, 0x7A,
		0xC6, 0xE7, 0x41, 0x7A, 0xFA, 0xE7, 0x43, 0xEA, 0x44, 0x7A, 0xC1, 0xE1, 0xE6, 0x43, 0xE9, 0x44,
		0x25, 0xE0, 0x7A, 0xC6, 0xE7, 0x41, 0x7A, 0xFA, 0xE7, 0x43, 0xEA, 0x44, 0x7A, 0xC0, 0xE7, 0x43,
		0xE9, 0x44, 0x25, 0xE0, 0x92, 0x10, 0x7A, 0xE1, 0x44, 0xE2, 0x44, 0xE3, 0x44, 0xE4, 0x44, 0xE5,
		0x44, 0xE6, 0x44, 0xE7, 0x44, 0xE8, 0x44, 0xC1, 0xD8, 0x24, 0x3E, 0x92, 0xFF, 0x02, 0x7A, 0xCF,
		0xD7, 0xE6, 0x43, 0xF1, 0x44, 0x7A, 0xD0, 0xE7, 0x43, 0x2A, 0x2A, 0x32, 0xAB, 0x03, 0x42, 0x5C,
		0x92, 0x03, 0x7A, 0xC0, 0xE1, 0x43, 0xD9, 0x27, 0x90, 0x6A, 0xDF, 0x43, 0x7A, 0xC8, 0xE7, 0x41,
		0x32, 0xAB, 0x03, 0x86, 0x01, 0x92, 0x11, 0x7A, 0xC2, 0x43, 0x7A, 0xE7, 0x44, 0x6A, 0xC6, 0x44,
		0x7A, 0xC3, 0x43, 0x7A, 0xE8, 0x44, 0x6A, 0xC7, 0x44, 0xC1, 0xD4, 0x24, 0x57, 0x7A, 0xC8, 0xE1,
		0x43, 0xE0, 0x3A, 0x02, 0x7A, 0xCF, 0xE7, 0xE6, 0x43, 0xF1, 0x44, 0x29, 0xE0, 0x7A, 0xC7, 0xE1,
		0x41, 0x6A, 0xD4, 0x45, 0x5A, 0x25, 0x36, 0x46, 0x46, 0x46, 0x46, 0x7A, 0xE9, 0x44, 0x7A, 0xC0,
		0xE7, 0x43, 0x55, 0x7A, 0xEA, 0x45, 0x7A, 0xE9, 0x51, 0x1C, 0x43, 0x6A, 0xCA, 0x44, 0x1D, 0x43,
		0x6A, 0xCB, 0x44, 0x7A, 0xC1, 0xCA, 0xE6, 0x43, 0xE9, 0x44, 0x7A, 0xC1, 0xE1, 0x43, 0x7A, 0xCC,
		0xE0, 0xE6, 0x41, 0x2C, 0x42, 0x7A, 0xC5, 0xE1, 0x43, 0x49, 0xE0, 0x34, 0x7A, 0xC1, 0xCC, 0xE6,
		0x43, 0xE9, 0x44, 0x7A, 0xC1, 0xE1, 0x43, 0x2C, 0x70, 0x7A, 0xCC, 0x43, 0x7A, 0xCF, 0x44, 0x7A,
		0xCD, 0x43, 0x7A, 0xCE, 0x44, 0x6A, 0xCA, 0x43, 0xC1, 0xCA, 0x7A, 0xE6, 0x41, 0xE9, 0x45, 0x2B,
		0xAE, 0xEE, 0x44, 0x7A, 0xC1, 0xCA, 0xE6, 0x43, 0xE9, 0x44, 0x7A, 0xC1, 0xE1, 0x43, 0x7A, 0xCC,
		0xEC, 0xE6, 0x41, 0x2C, 0x42, 0x7A, 0xC5, 0xE1, 0x43, 0x49, 0xE0, 0x34, 0x7A, 0xC1, 0xCC, 0xE6,
		0x43, 0xE9, 0x44, 0x7A, 0xC1, 0xE1, 0x43, 0x2C, 0x70, 0x7A, 0xCC, 0x43, 0x7A, 0xCF, 0x44, 0x7A,
		0xCD, 0x43, 0x7A, 0xCE, 0x44, 0x6A, 0xCB, 0x43, 0xC1, 0xCA, 0x7A, 0xE6, 0x41, 0xE9, 0x45, 0x2B,
		0xAE, 0xED, 0x44, 0x02
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_LPTIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // Test I2C Communication Interface by sending Test Opcode
  ret = I2C_Write_Opcode(dev_addr, TEST_READ);
  ret = HAL_I2C_Master_Receive(&hi2c1, dev_addr, My_buf, 1, 1);

  // POR + INIT
  ret = I2C_Write_Opcode(dev_addr, POR);
  HAL_Delay(500);
  ret = I2C_Write_Opcode(dev_addr, INIT);

  // write firmware (10x 16 Bytes plus 7 Bytes)
  //ret = I2C_Memory_Access(dev_addr, WR_MEM, 0x00, bending_fw, 167);
  ret = I2C_Memory_Access(dev_addr, WR_MEM, 0x00, standard_fw, 548);

  ret = I2C_Memory_Access(dev_addr, RD_MEM, 0x00, My_buf, 167);

  // write configuration (52 Bytes)
 // ret = I2C_Config_Access(dev_addr, WR_CONFIG, 0x00, bending_cfg_bytewise, 52);
  ret = I2C_Config_Access(dev_addr, WR_CONFIG, 0x00, standard_cfg_bytewise, 52);

  //deactivate PC input ports
  //ret = I2C_Config_Access(dev_addr, WR_CONFIG, 0x06, 0x00, 1);

  ret = I2C_Config_Access(dev_addr, RD_CONFIG, 0x00, My_buf, 52);

  ret = I2C_Write_Opcode(dev_addr, INIT);

  // start CDC measurement
  ret = I2C_Write_Opcode(dev_addr, 0x8C);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	// read result register
	MyRES0 = I2C_Read_Result(dev_addr, 0x40, 0x00);
	MyRES1 = I2C_Read_Result(dev_addr, 0x40, 0x01);

	//internal Reference at RES2
	MyResult = I2C_Read_Result(dev_addr, 0x40, 0x02);

	HAL_Delay(20);

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

  /** Initializes the CPU, AHB and APB busses clocks
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPTIM1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_PCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SSN_GPIO_Port, SSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : INTN_Pin */
  GPIO_InitStruct.Pin = INTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SSN_Pin */
  GPIO_InitStruct.Pin = SSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SSN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

	uint16_t i2c_error_code = 0;
	i2c_error_code = HAL_I2C_GetError(&hi2c1);

	if(i2c_error_code == HAL_I2C_ERROR_NONE)      {
		printf("!< No error              */");
	}
	if(i2c_error_code == HAL_I2C_ERROR_BERR)      {
		printf("!< BERR error            */");
	}
	if(i2c_error_code == HAL_I2C_ERROR_ARLO)      {
		printf("!< ARLO error            */");
	}
	if(i2c_error_code == HAL_I2C_ERROR_AF)        {
		printf("*!< ACKF error            */");
	}
	if(i2c_error_code == HAL_I2C_ERROR_OVR)       {
		printf("*!< OVR error             */");
	}
	if(i2c_error_code == HAL_I2C_ERROR_DMA)       {
		printf("*!< DMA transfer error    */");
	}
	if(i2c_error_code == HAL_I2C_ERROR_TIMEOUT)   {
		printf("*!< Timeout error         */");
	}
	if(i2c_error_code == HAL_I2C_ERROR_SIZE)      {
		printf("*!< Size Management error */");
	}
	if(i2c_error_code == HAL_I2C_ERROR_DMA_PARAM) {
		printf("!< DMA Parameter Error   */");
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
