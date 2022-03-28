/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs_sd.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PROGRAM_SECTOR_START FLASH_SECTOR_4 // start sector is number 4
#define APPLICATION_ADDRESS  (uint32_t)((FLASH_BASE + (PROGRAM_SECTOR_START * 16 * 1024)))
#define BLOCK_SIZE_MAX 512
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef void (*pFunction)(void);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
SPI_HandleTypeDef SD_SPI_HANDLE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void JumpToApplication();
void UpdateFirmware();
HAL_StatusTypeDef EaraseAppFlash();
void WriteAppFlash(uint32_t offset, uint32_t address, TCHAR *data, size_t size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int32_t file, uint8_t *ptr, int32_t len)
{
    for (int i = 0; i < len; i++)
    {
        ITM_SendChar(*ptr++);
    }
    return len;
}
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
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  //UpdateFirmware();
  GPIO_PinState state = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
  if (state == GPIO_PIN_RESET)
  {
	  state = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
	  if (state == GPIO_PIN_RESET)
	  {
		  UpdateFirmware();
	  }
  }
  JumpToApplication();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
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
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void JumpToApplication(void)
{
	printf("Start MainApp at:0x%.8x\n", (unsigned int)APPLICATION_ADDRESS);
	pFunction appEntry;
	uint32_t appStack;

	/* Get the application stack pointer (First entry in the application vector table) */
	appStack = (uint32_t) *((__IO uint32_t*)APPLICATION_ADDRESS);

	/* Get the application entry point (Second entry in the application vector table) */
	appEntry = (pFunction) *(__IO uint32_t*) (APPLICATION_ADDRESS + 4);

	/* Reconfigure vector table offset register to match the application location */
	SCB->VTOR = APPLICATION_ADDRESS;

	/* Set the application stack pointer */
	__set_MSP(appStack);

	/* Start the application */
	appEntry();
}
//-------------------
void UpdateFirmware()
{
	FATFS fatfs;
	FIL file;
	FRESULT fres;

	fres = f_mount(&fatfs, "", 0);
	if (fres == FR_OK)
	{
	  fres = f_open(&file, "firmware.bin", FA_READ);
	  if (fres == FR_OK)
	  {
		  if (EaraseAppFlash() != HAL_OK)
		  {
			  printf("An Error earasing flash program!\n");
			  Error_Handler();
		  }

		  printf("An earasing flash is successfully!\n");

		  FSIZE_t fsize = f_size(&file);
		  if (fsize <= 0)
		  {
			  printf("A firmware file is empty!");
			  return;
		  }

		  printf("A file size is %u\n", (unsigned int)fsize);

		  uint32_t rbytes = 0;
		  while (rbytes < fsize)
		  {
			  TCHAR buffer[BLOCK_SIZE_MAX];
			  UINT len = 0;
			  fres = f_lseek(&file, rbytes);
			  if (fres != FR_OK)
			  {
				  printf("An error offset position in file!\n");
				  Error_Handler();
			  }
			  f_read(&file, buffer, sizeof(buffer), &len);
			  if (len > 0)
			  {
				  for (int i = 0; i < len; i += 4)
				  {
					  printf("BYTE = 0x%02x%02x%02x%02x\n", buffer[i], buffer[i + 1], buffer[i + 2], buffer[i + 3]);
				  }
				  WriteAppFlash(APPLICATION_ADDRESS, rbytes, buffer, len);
				  rbytes += len;
				  printf("A reading is %u bytes (curent = %u, total = %u bytes)\n", (unsigned int)len, (unsigned int)rbytes, (unsigned int)fsize);
			  }
			  else
			  {
				  printf("A reading data is empty!\n");
				  Error_Handler();
			  }
		  }
		  if (rbytes == fsize)
		  {
			  printf("A firmware is loading successfully (size = %u)!\n", (unsigned int)rbytes);
			  for (uint8_t i = 0; i < 10; i++)
			  {
				  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				  HAL_Delay(100);
			  }
		  }
		  else
		  {
			  printf("An error loading firmware (%u from %u)\n", (unsigned int)rbytes, (unsigned int)fsize);
			  Error_Handler();
		  }
	  }
	  f_close(&file);
	}
}
//--------------------------------
HAL_StatusTypeDef EaraseAppFlash()
{
	HAL_FLASH_Unlock();
	HAL_StatusTypeDef status;
	uint32_t sector_error;

	FLASH_EraseInitTypeDef flash_erase =
	{
		.TypeErase = FLASH_TYPEERASE_SECTORS,
		.Sector = PROGRAM_SECTOR_START,
		.NbSectors = FLASH_SECTOR_TOTAL - PROGRAM_SECTOR_START,
		.VoltageRange = FLASH_VOLTAGE_RANGE_1
	};
	status = HAL_FLASHEx_Erase(&flash_erase, &sector_error);
	HAL_FLASH_Lock();

	return status;
}
//-----------------------------------------------------------------------------
void WriteAppFlash(uint32_t offset, uint32_t address, TCHAR *data, size_t size)
{
	HAL_FLASH_Unlock();

	for (uint16_t i = 0; i < size; i++)
	{
		uint32_t addr = offset + address + i;
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr, data[i]);
	}

	HAL_FLASH_Lock();
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
