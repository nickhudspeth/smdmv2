/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "userdefs.h"
#include "libsparse.h"
#include "version.h"
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
sparse_ParserTypeDef *parser;
volatile uint8_t new_cmd_flag = 0;
char it_cmd_buffer[256] = {'\0'};
volatile uint8_t it_cmd_buf_cnt = 0;
char *sys_cmd_buffer = NULL;
extern uint32_t bootloader_flag;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* Application Callback Function Prototypes */
sparse_StatusTypeDef SetAccelerationCallback(sparse_ArgPack *a);
sparse_StatusTypeDef SetCurrentCallback(sparse_ArgPack *a);
sparse_StatusTypeDef MoveContinuousCallback(sparse_ArgPack *a);
sparse_StatusTypeDef GetEndstopStatusCallback(sparse_ArgPack *a);
sparse_StatusTypeDef SetMicrostepResolutionCallback(sparse_ArgPack *a);
sparse_StatusTypeDef GetVersionCallback(sparse_ArgPack *a);
sparse_StatusTypeDef SetAccelerationDetailsCallback(sparse_ArgPack *a);
sparse_StatusTypeDef HomeCallback(sparse_ArgPack *a);
sparse_StatusTypeDef MoveAbsoluteCallback(sparse_ArgPack *a);
sparse_StatusTypeDef MoveRelativeCallback(sparse_ArgPack *a);
sparse_StatusTypeDef MoveStepsCallback(sparse_ArgPack *a);
sparse_StatusTypeDef MoveMicrostepsCallback(sparse_ArgPack *a);
sparse_StatusTypeDef GetPositionCallback(sparse_ArgPack *a);
sparse_StatusTypeDef StopCallback(sparse_ArgPack *a);
sparse_StatusTypeDef GetTemperatureCallback(sparse_ArgPack *a);
sparse_StatusTypeDef SetVelocityCallback(sparse_ArgPack *a);
sparse_StatusTypeDef SetMotionParametersCallback(sparse_ArgPack *a);
sparse_StatusTypeDef SystemResetCallback(sparse_ArgPack *a);
sparse_StatusTypeDef BootloaderCallback(sparse_ArgPack *a);

/** System Function Prototypes */
void ProcessCmdbuffer(void);
void LED_SetErrorState(uint32_t state);
void JumpToBootloader(void);
void ts_write(const char *s);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  char c = it_cmd_buffer[it_cmd_buf_cnt];
  if (c == '\n') {
    new_cmd_flag = 1;
  } else {
    it_cmd_buf_cnt = ((it_cmd_buf_cnt + 1) < 255) ? it_cmd_buf_cnt + 1 : 255;
    if (HAL_UART_Receive_IT(&huart1, &it_cmd_buffer[it_cmd_buf_cnt], 1) !=
        HAL_OK) {
      _Error_Handler(__FILE__, __LINE__);
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  /* Toggle LED pin states on expiry of LED update timer. */
  if (htim == &htim16) {
    HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
    HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
    HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  switch (GPIO_Pin) {
    default:
      break;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  MX_SPI1_Init();
  MX_TIM16_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  /* Configure command parser */
  parser = sparse_New();
  parser->delimiter = ',';
  parser->line_end = '\n';

  sparse_RegisterCallback(parser, 'A', 1, SetAccelerationCallback);
  sparse_RegisterCallback(parser, 'B', 1, SetCurrentCallback);
  sparse_RegisterCallback(parser, 'C', 2, MoveContinuousCallback);
  sparse_RegisterCallback(parser, 'D', 0, GetEndstopStatusCallback);
  sparse_RegisterCallback(parser, 'E', 1, SetMicrostepResolutionCallback);
  sparse_RegisterCallback(parser, '*', 1, GetVersionCallback);
  sparse_RegisterCallback(parser, 'F', 6, SetAccelerationDetailsCallback);
  sparse_RegisterCallback(parser, 'H', 5, HomeCallback);
  sparse_RegisterCallback(parser, 'J', 2, MoveAbsoluteCallback);
  sparse_RegisterCallback(parser, 'K', 2, MoveRelativeCallback);
  sparse_RegisterCallback(parser, 'M', 1, MoveStepsCallback);
  sparse_RegisterCallback(parser, 'N', 1, MoveMicrostepsCallback);
  sparse_RegisterCallback(parser, 'P', 1, GetPositionCallback);
  sparse_RegisterCallback(parser, 'S', 0, StopCallback);
#if INTERNAL_TEMPERATURE_SENSOR_ENABLED
  sparse_RegisterCallback(parser, 'T', 0, GetTemperatureCallback);
#endif
  sparse_RegisterCallback(parser, 'V', 1, SetVelocityCallback);
  sparse_RegisterCallback(parser, 'X', 3, SetMotionParametersCallback);
  sparse_RegisterCallback(parser, 'Z', 0, SystemResetCallback);
  sparse_RegisterCallback(parser, '#', 0, BootloaderCallback);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* Start timer for status LED*/
  if (HAL_TIM_OC_Start(&htim16, TIM_CHANNEL_1) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (new_cmd_flag) {
      ProcessCmdBuffer();
      sparse_Exec(parser, sys_cmd_buffer);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection =
      RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* Application Callback Functions */
sparse_StatusTypeDef SetAccelerationCallback(sparse_ArgPack *a) {
  float accel = atof(a->arg_list[0]);

  ts_write("@,a");
  return SPARSE_OK;
}
sparse_StatusTypeDef SetCurrentCallback(sparse_ArgPack *a) {
  float drive_current = atof(a->arg_list[0]);
  float hold_current = atof(a->arg_list[1]);

  ts_write("@,b");
  return SPARSE_OK;
}
sparse_StatusTypeDef MoveContinuousCallback(sparse_ArgPack *a) {
  uint8_t dir = atoi(a->arg_list[0]);
  uint32_t vel = (uint32_t) ((atof(a->arg_list[1])));

  ts_write("@,c");
  return SPARSE_OK;
}
sparse_StatusTypeDef GetEndstopStatusCallback(sparse_ArgPack *a) {
  return SPARSE_OK;
}
sparse_StatusTypeDef SetMicrostepResolutionCallback(sparse_ArgPack *a) {
  return SPARSE_OK;
}
sparse_StatusTypeDef GetVersionCallback(sparse_ArgPack *a) {
  ts_write(git_sha);
  return SPARSE_OK;
  return SPARSE_OK;
}
sparse_StatusTypeDef SetAccelerationDetailsCallback(sparse_ArgPack *a) {
  return SPARSE_OK;
}
sparse_StatusTypeDef HomeCallback(sparse_ArgPack *a) { return SPARSE_OK; }
sparse_StatusTypeDef MoveAbsoluteCallback(sparse_ArgPack *a) {
  return SPARSE_OK;
}
sparse_StatusTypeDef MoveRelativeCallback(sparse_ArgPack *a) {
  return SPARSE_OK;
}
sparse_StatusTypeDef MoveStepsCallback(sparse_ArgPack *a) { return SPARSE_OK; }
sparse_StatusTypeDef MoveMicrostepsCallback(sparse_ArgPack *a) {
  return SPARSE_OK;
}
sparse_StatusTypeDef GetPositionCallback(sparse_ArgPack *a) {
  return SPARSE_OK;
}
sparse_StatusTypeDef StopCallback(sparse_ArgPack *a) { return SPARSE_OK; }
sparse_StatusTypeDef GetTemperatureCallback(sparse_ArgPack *a) {
  return SPARSE_OK;
}
sparse_StatusTypeDef SetVelocityCallback(sparse_ArgPack *a) {
  return SPARSE_OK;
}
sparse_StatusTypeDef SetMotionParametersCallback(sparse_ArgPack *a) {
  return SPARSE_OK;
}
sparse_StatusTypeDef SystemResetCallback(sparse_ArgPack *a) {
  return SPARSE_OK;
}
sparse_StatusTypeDef BootloaderCallback(sparse_ArgPack *a) { return SPARSE_OK; }

void ProcessCmdBuffer(void) {
  uint32_t i = 0;
  if (sys_cmd_buffer != NULL) {
    free(sys_cmd_buffer);
  }
  /* Keep a count of the number of illegal characters prepended
   * to the string using variable i. */
  while ((it_cmd_buffer[i] < 33) || (it_cmd_buffer[i] > 126)) {
    i++;
  }
  /* Allocate space for the message contained in it_cmd_buf, minus
   * the number of illegal characters counted. */
  sys_cmd_buffer = calloc(it_cmd_buf_cnt - i + 1, sizeof(char));
  /* Copy message between buffers using counted number of illegal
   * characters as offset into it_cmd_buffer. */
  memcpy(sys_cmd_buffer, it_cmd_buffer + i,
         (it_cmd_buf_cnt - i) * sizeof(char));
  /* Append null-terminator to end of sys_cmd_buffer. */
  sys_cmd_buffer[it_cmd_buf_cnt - i] = '\0';
#if APPLICATION_DEBUG
  printf("Received string \"%s\"\n", it_cmd_buffer);
#endif
  /* Clear it_cmd_buffer for next message. */
  memset(it_cmd_buffer, '\0', (it_cmd_buf_cnt + 1));
  /* Resetting the count here means that commands sent in rapid succession
   * may be missed by controller. A sufficient processing timeout should
   * be measured and specified in the usage manual for this firmware revision.
   */
  it_cmd_buf_cnt = 0;
  new_cmd_flag = 0;
  if (HAL_UART_Receive_IT(&huart1, &it_cmd_buffer[it_cmd_buf_cnt], 1) !=
      HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }
}

void LED_SetErrorState(uint32_t state) {
  uint8_t r = 0, g = 0, b = 0, mult = 0;
  r = (state & (1UL << 2));
  g = (state & (1UL << 1));
  b = (state & 1UL);
  mult = (state >> 3);

  /** Set timer to blink LED at desired frequency. */
  __HAL_TIM_SET_AUTORELOAD(&htim16, LED_TIMER_BASE_PERIOD * mult);
  /** Set LED to desired color */
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, r);
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, g);
  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, b);
}

void JumpToBootloader(void) {
  void (*SysMemBootJump)(void);
  /* Set memory address */
  volatile uint32_t addr = 0x1FFF0000;
/* Disable RCC */
#if defined(USE_HAL_DRIVER)
  HAL_RCC_DeInit();
#endif
#if defined(USE_STDPERIPH_DRIVER)
  RCC_DeInit();
#endif
  /* Disable SysTick timer and reset to default values */
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;
  /* Disable all interrupts */
  __disable_irq();
  /* Remap start of system memory to address 0x00000000*/
  __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
  SysMemBootJump = (void (*)(void))(*((uint32_t *)(addr + 4)));
  __set_MSP(*(uint32_t *)addr);
  SysMemBootJump();
}

void ts_write(const char *s) {
  char buff[512];
  float time_s = HAL_GetTick() / 1000.0f;
  int n = sprintf(buff, "%s,%f\n", s, time_s);
  HAL_UART_Transmit_IT(&huart1, (uint8_t *)&buff, n);
}

void _Error_Handler(char *file, uint32_t line) {
  while (1) {
    asm("NOP");
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
   number,
   tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
