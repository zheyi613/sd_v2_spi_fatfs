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
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "diskio.h"
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
uint16_t Timer1, Timer2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  uint8_t msg[200], len;
  char buff[512];
  char check_buff[512];
  memset(buff, 0, 512);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_Delay(50); /* wait at least 35 ms to ramp up power */
  // int status = disk_initialize(0);
  // if (status)
  //   Error_Handler();
  // disk_ioctl(0, MMC_GET_CSD, buff);
  // disk_ioctl(0, MMC_GET_CID, buff);
  // disk_ioctl(0, MMC_GET_OCR, buff);
  // disk_ioctl(0, MMC_GET_SDSTAT, buff);
  FATFS fs;
  FATFS *pfs;
  FIL fil;
  FRESULT fres;
  DWORD fre_clust;
  uint32_t total, free;
  UINT buff_size, remain;
  fres = f_mount(&fs, "", 0);
  if (fres != FR_OK) {
    Error_Handler();
  }
  HAL_GPIO_TogglePin(TEST_GPIO_Port, TEST_Pin);
  fres = f_open(&fil, "test1.txt", FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
  if (fres != FR_OK) {
    Error_Handler();
  }
  fres = f_getfree("", &fre_clust, &pfs);
  if (fres != FR_OK) {
    Error_Handler();
  }
  total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
  free = (uint32_t)(fre_clust * pfs->csize * 0.5);
  len = snprintf((char *)msg, 200, "total: %ld, free: %ld\n", total, free);
  CDC_Transmit_FS(msg, len);
  if (free < 1) {
    Error_Handler();
  }
  buff_size = snprintf(buff, 512, "xxxxx,xxxx,xxxx,xxxx,xxxx,xxxx,xxxx,xxxx,"
                                  "xxxx,xxxx,xxxx,xxxx,xxxx\n"
                                  "xxxx,xxxx,xxxx,xxxx,xxxx,xxxx,xxxx,xxxx,"
                                  "xxxx,xxxx,xxxx,xxxx,xxxx%s", "\n");
  for (uint16_t i = 0; i < 500; i++) {
    fres = f_write(&fil, buff, buff_size, &remain);
    if (fres != FR_OK) {
      Error_Handler();
    }
    // f_puts(buff, &fil);
  }
  fres = f_close(&fil);
  if (fres != FR_OK) {
    Error_Handler();
  }
  fres = f_open(&fil, "test1.txt", FA_READ);
  if (fres != FR_OK) {
    Error_Handler();
  }
  while (f_gets(check_buff, sizeof(check_buff), &fil)) {
    CDC_Transmit_FS((uint8_t *)check_buff, 512);
  }
  fres = f_close(&fil);
  if (fres != FR_OK) {
    Error_Handler();
  }
  HAL_GPIO_TogglePin(TEST_GPIO_Port, TEST_Pin);
  fres = f_mount(NULL, "", 0);
  if (fres != FR_OK) {
    Error_Handler();
  }
  len = snprintf((char *)msg, 200, "Complete!!%s", "\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    CDC_Transmit_FS(msg, len);
    HAL_Delay(1000);
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
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim)
{
  uint16_t count;

  count = Timer1;
  if (count)
    Timer1 = --count;
  count = Timer2;
  if (count)
    Timer2 = --count;
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
