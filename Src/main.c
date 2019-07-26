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
#include "adc.h"
#include "i2c.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "userdefs.h"
#include "libsparse.h"
#include "version.h"
#include "errorstates.h"
#include "TMC4361A.h"
#include "TMC2160.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
ConfigurationTypeDef TMC4361A_Config, TMC2160_Config;
TMC4361ATypeDef TMC4361A;
TMC2160TypeDef TMC2160;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define FIXED_23_8_MAKE(a) (int32_t)((a * (1ul << 8ul)))
#define FIXED_22_2_MAKE(a) (int32_t)((a * (1ul << 2ul)))
#define FIXED_24_0_MAKE(a) (int32_t)(lround(a*0x7FFFFF) & 0xFFFFFF)
#define INT_24_TRUNCATE(a)(int32_t)(a & 0xFFFFFF)
#define DMA_TX_BUF_LENGTH 16
#define DMA_RX_BUF_LENGTH 16
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
sparse_ParserTypeDef *parser;
volatile uint8_t new_cmd_flag = 0;
uint8_t it_cmd_buffer[256] = { '\0' };
volatile uint8_t it_cmd_buf_cnt = 0;
char *sys_cmd_buffer = NULL;
extern uint32_t bootloader_flag;
struct flags_t {
	volatile uint8_t target_reached;
	volatile uint8_t run_periodic_job;
	volatile uint8_t rs485_bus_free;
	volatile uint8_t internal_temp_ready;
	volatile uint8_t address_requested;
	volatile uint8_t intr_triggered;
	volatile uint8_t diag0_triggered;
	volatile uint8_t diag1_triggered;
	volatile uint8_t home_reached;
	volatile uint8_t home_error;
	volatile uint8_t cover_done;
} flags;

struct params_t {
	volatile uint16_t microstep_resolution;
	volatile float spr;
	volatile float lead;
	volatile int8_t sgt;
	volatile uint32_t device_id;
} params;

struct status_t {
	volatile uint8_t motion_params_cfgd;
	volatile uint8_t homed;
} status;

struct testresults_t {
	volatile uint8_t led;
	volatile uint8_t spi;
	volatile uint8_t eeprom_storage;
	volatile uint8_t eeprom_address;

} testresults;

uint8_t dma_tx_buf[DMA_TX_BUF_LENGTH] = { 0 };
uint8_t dma_rx_buf[DMA_RX_BUF_LENGTH] = { 0 };
float internal_temp = 0.0f;

volatile int res;
volatile int velcur;
volatile int sgstat;
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
sparse_StatusTypeDef HomeCallback(sparse_ArgPack *a);
sparse_StatusTypeDef MoveAbsoluteCallback(sparse_ArgPack *a);
sparse_StatusTypeDef MoveRelativeCallback(sparse_ArgPack *a);
sparse_StatusTypeDef MoveStepsCallback(sparse_ArgPack *a);
sparse_StatusTypeDef MoveMicrostepsCallback(sparse_ArgPack *a);
sparse_StatusTypeDef GetPositionCallback(sparse_ArgPack *a);
sparse_StatusTypeDef ResumeCallback(sparse_ArgPack *a);
sparse_StatusTypeDef StopCallback(sparse_ArgPack *a);
sparse_StatusTypeDef GetTemperatureCallback(sparse_ArgPack *a);
sparse_StatusTypeDef SetVelocityCallback(sparse_ArgPack *a);
sparse_StatusTypeDef SetMotionParametersCallback(sparse_ArgPack *a);
sparse_StatusTypeDef SystemResetCallback(sparse_ArgPack *a);
sparse_StatusTypeDef BootloaderCallback(sparse_ArgPack *a);
sparse_StatusTypeDef GetVersionCallback(sparse_ArgPack *a);
sparse_StatusTypeDef EmergencyStopCallback(sparse_ArgPack *a);

/** System Function Prototypes */
void ProcessCmdBuffer(void);
void LED_SetErrorState(uint32_t state);
void JumpToBootloader(void);
void ts_write(const char *s);
void SendDeviceAddress(void);
void Test_RunSuite(void);
void Test_LEDFunction(void);
void Test_EEPROMStorageFunction();
void Test_EEPROMAddressFunction();
void System_InitDefaultState(void);
void System_RunSelfTest(void);

HAL_StatusTypeDef EEPROM_GetAddress(uint64_t *result);
HAL_StatusTypeDef EEPROM_ReadData(uint8_t addr, uint8_t* data, uint32_t size);
HAL_StatusTypeDef EEPROM_WriteData(uint8_t addr, uint8_t *data, uint32_t size);

int32_t ConvertDistanceToFullStepsInt(float distance);
int32_t ConvertDistanceToMicroStepsInt(float distance);
float ConvertDistanceToFullStepsFloat(float distance);

void Trinamic_init(void);
void Trinamic_ConfigureInputFilters(void);
void Trinamic_ConfigureSPIOutput(void);
void Trinamic_ConfigureInterrupts(void);
void Trinamic_ConfigureChopper(void);
void Trinamic_ConfigureCoolstep(void);
void Trinamic_ConfigureDCStep(void);
void Trinamic_ConfigureEndstops(void);
void Trinamic_ConfigureMisc(void);
void Trinamic_ConfigureStallguard(void);
void Trinamic_ConfigureStealthchop(void);
void tmc4361A_readWriteArray(uint8_t channel, uint8_t *data, size_t length);
uint8_t TMC4361A_Reset(void);
uint8_t TMC4361A_Restore(void);
void TMC4361A_ConfigCallback(TMC4361ATypeDef *tmc4361A, ConfigState state);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	flags.rs485_bus_free = 0;
	char c = it_cmd_buffer[it_cmd_buf_cnt];
	if (c == '\n') {
		new_cmd_flag = 1;
	} else {
		it_cmd_buf_cnt =
				((it_cmd_buf_cnt + 1) < 255) ? it_cmd_buf_cnt + 1 : 255;
		if (HAL_UART_Receive_IT(&huart1, &it_cmd_buffer[it_cmd_buf_cnt], 1)
				!= HAL_OK) {
			_Error_Handler(__FILE__, __LINE__);
		}
	}
}

void UART_IdleLineCallback(UART_HandleTypeDef *huart) {
	flags.rs485_bus_free = 1;
	__HAL_UART_CLEAR_IDLEFLAG(huart);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim15) {
		flags.run_periodic_job = 1;
	}
	/* Toggle LED pin states on expiry of LED update timer. */
	if (htim == &htim16) {
		HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
		HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
		HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
	case TARGET_REACHED_Pin:
		flags.target_reached = 1;
		break;
	case ADDR_SEL_Pin:
		flags.address_requested = 1;
		break;
	case INTR_Pin:
		flags.intr_triggered = 1;
		break;
	case DIAG0_Pin:
		flags.diag0_triggered = 1;
		break;
	case DIAG1_Pin:
		flags.diag1_triggered = 1;
		break;
	default:
		break;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	/** @todo: Internal temperature should be adjusted using
	 * ST's recommended calibration procedure and converted
	 * to degrees Celsius.
	 */
	internal_temp = (float) HAL_ADC_GetValue(hadc);
	flags.internal_temp_ready = 1;
}

int32_t ConvertDistanceToFullStepsInt(float distance) {
	int32_t ret = 0;
	if ((params.spr != 0.0f) && (params.lead != 0.0f)) {
		ret = lround(distance * (params.spr / params.lead));
	}
	return ret;
}
int32_t ConvertDistanceToMicroStepsInt(float distance) {
	int32_t ret = 0;
	if ((params.spr != 0.0f) && (params.lead != 0.0f)) {
		ret = lround(distance * (params.spr / params.lead));
	}
	return params.microstep_resolution * ret;
}

float ConvertDistanceToFullStepsFloat(float distance) {
	float ret = 0.0f;
	if ((params.spr != 0.0f) && (params.lead != 0.0f)) {
		ret = distance * (params.spr / params.lead);
	}
	return ret;
}

float ConvertDistanceToMicroStepsFloat(float distance) {
	float ret = 0.0f;
	if ((params.spr != 0.0f) && (params.lead != 0.0f)) {
		ret = params.microstep_resolution * distance
				* (params.spr / params.lead);
	}
	return ret;
}

float ConvertMicroStepsToDistanceFloat(int32_t microsteps) {
	float ret = 0.0f;
	if (status.motion_params_cfgd) {
		ret = microsteps * params.lead
				/ (params.microstep_resolution * params.spr);
	}

	return ret;
}

void ProcessEventFlags(void) {
	char buf[64];
	uint32_t tmc4361a_events, tmc2160_status;
	if (flags.intr_triggered) {
		/* Read events register. Events register is cleared after this read. */
		tmc4361a_events = tmc4361A_readInt(&TMC4361A, TMC4361A_EVENTS);
		if (tmc4361a_events & TMC4361A_TARGET_REACHED_MASK) {
			flags.target_reached = 1;

		}
		if (tmc4361a_events & TMC4361A_POS_COMP_REACHED_MASK) {

		}
		if (tmc4361a_events & TMC4361A_VEL_REACHED_MASK) {

		}
		if (tmc4361a_events & TMC4361A_VEL_STATE_00_MASK) {

		}
		if (tmc4361a_events & TMC4361A_VEL_STATE_01_MASK) {

		}
		if (tmc4361a_events & TMC4361A_VEL_STATE_10_MASK) {

		}
		if (tmc4361a_events & TMC4361A_RAMP_STATE_00_MASK) {

		}
		if (tmc4361a_events & TMC4361A_RAMP_STATE_01_MASK) {

		}
		if (tmc4361a_events & TMC4361A_RAMP_STATE_10_MASK) {

		}
		if (tmc4361a_events & TMC4361A_MAX_PHASE_TRAP_MASK) {

		}
		if (tmc4361a_events & TMC4361A_FROZEN_MASK) {

		}
		if (tmc4361a_events & TMC4361A_STOPL_EVENT_MASK) {

		}
		if (tmc4361a_events & TMC4361A_STOPR_EVENT_MASK) {

		}
		if (tmc4361a_events & TMC4361A_VSTOPL_ACTIVE_MASK) {

		}
		if (tmc4361a_events & (TMC4361A_VSTOPL_ACTIVE_MASK << 1)) {
			/* VSTOPR event mask is not defined in the TMC4361A library,
			 * so we make the appropriate mask by shifting VSTOPL_ACTIVE_MASK */

		}
		if (tmc4361a_events & TMC4361A_HOME_ERROR_MASK) {
			flags.home_error = 1;

		}
		if (tmc4361a_events & TMC4361A_XLATCH_DONE_MASK) {

		}
		if (tmc4361a_events & TMC4361A_FS_ACTIVE_MASK) {

		}
		if (tmc4361a_events & TMC4361A_ENC_FAIL_MASK) {

		}
		if (tmc4361a_events & TMC4361A_N_ACTIVE_MASK) {

		}
		if (tmc4361a_events & TMC4361A_ENC_DONE_MASK) {

		}
		if (tmc4361a_events & TMC4361A_SER_ENC_DATA_FAIL_MASK) {

		}
		if (tmc4361a_events & TMC4361A_SER_DATA_DONE_MASK) {

		}
		if (tmc4361a_events & TMC4361A_SERIAL_ENC_FLAGS_MASK) {

		}
		if (tmc4361a_events & TMC4361A_COVER_DONE_MASK) {
			flags.cover_done = 1;

		}
		if (tmc4361a_events & TMC4361A_ENC_VEL0_MASK) {

		}
		if (tmc4361a_events & TMC4361A_CL_MAX_MASK) {

		}
		if (tmc4361a_events & TMC4361A_CL_FIT_MASK) {

		}
		if (tmc4361a_events & TMC4361A_STOP_ON_STALL_MASK) {

		}
		if (tmc4361a_events & TMC4361A_MOTOR_EV_MASK) {

		}
		if (tmc4361a_events & TMC4361A_RST_EV_MASK) {

		}
	}
	if (flags.diag0_triggered) {
		tmc2160_status = tmc2160_readInt(&TMC2160, TMC2160_DRV_STATUS);
		if (tmc2160_status & TMC2160_STALLGUARD_MASK) {
			/** Wait 500ms and restart motor */
			HAL_Delay(500);
			TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_REFERENCE_CONF,
					TMC4361A_DRV_AFTER_STALL_MASK,
					TMC4361A_DRV_AFTER_STALL_SHIFT, 0x01);
			TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_REFERENCE_CONF,
					TMC4361A_DRV_AFTER_STALL_MASK,
					TMC4361A_DRV_AFTER_STALL_SHIFT, 0x00);
		}
		flags.diag0_triggered = 0;
	}
	if (flags.diag1_triggered) {

		flags.diag1_triggered = 0;
	}
	if (flags.target_reached) {
		//flags.target_reached = 0;
	}
	if (flags.run_periodic_job) {
		tmc4361A_periodicJob(&TMC4361A, HAL_GetTick());
		flags.run_periodic_job = 0;
	}
	if (flags.internal_temp_ready) {
		sprintf(buf, "@t,%f", internal_temp);
		/* Create and send CDXBUS frame with internal temperature. */
		ts_write(buf);
		flags.internal_temp_ready = 0;
	}
	if (flags.address_requested) {
		SendDeviceAddress();
		flags.address_requested = 0;
	}
}

uint8_t TMC4361A_Reset(void) {
// Pulse the low-active hardware reset pin
	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);

	tmc4361A_reset(&TMC4361A);
	/* Perform software reset by writing special value to reset register */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_RESET_REG,
			TMC4361A_RESET_REG_MASK, TMC4361A_RESET_REG_SHIFT, 0x525354);

	return 1;
}

uint8_t TMC2160_Reset(void) {
	tmc2160_reset(&TMC2160);

	return 1;
}

uint8_t TMC2160_Restore(void) {
	tmc2160_restore(&TMC2160);

	return 1;
}

uint8_t TMC4361A_Restore(void) {
// Pulse the low-active hardware reset pin
	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);

	tmc4361A_restore(&TMC4361A);

	return 1;
}

void TMC4361A_ConfigCallback(TMC4361ATypeDef *tmc4361A, ConfigState state) {
	uint8 driver = 0x0C, dataLength = 0;
	uint32 value;

// Setup SPI
	value = 0x44400040 | (dataLength << 13) | (driver << 0);
	tmc4361A_writeInt(tmc4361A, TMC4361A_SPIOUT_CONF, value);

// Reset/Restore driver
	if (state == CONFIG_RESET) {
		tmc4361A->config->reset();
	} else {
		tmc4361A->config->restore();
	}
}

void TMC2160_ConfigCallback(TMC2160TypeDef *tmc2160, ConfigState state) {

	if (state == CONFIG_RESET) {
		tmc2160->config->reset();
	} else {
		tmc2160->config->restore();
	}

}

void Trinamic_init(void) {
	TMC4361A_Reset();

	/** Initialize TMC4361A */
	tmc4361A_init(&TMC4361A, 0, &TMC4361A_Config,
			&tmc4361A_defaultRegisterResetState[0]);
	TMC4361A.config->reset = TMC4361A_Reset;
	TMC4361A.config->restore = TMC4361A_Restore;
	tmc4361A_setCallback(&TMC4361A, TMC4361A_ConfigCallback);

	/** Initialize TMC2160 */
	TMC2160_Reset();
	tmc2160_init(&TMC2160, 0, &TMC2160_Config,
			&tmc2160_defaultRegisterResetState[0]);
	TMC2160.config->reset = TMC2160_Reset;
	TMC2160.config->restore = TMC2160_Restore;
	tmc2160_setCallback(&TMC2160, TMC2160_ConfigCallback);

	HAL_GPIO_WritePin(FREEZE_GPIO_Port, FREEZE_Pin, GPIO_PIN_SET);
	/** Enable motion controller and gate driver outputs */
	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);

	/* SET EXTERNAL CLOCK FREQUENCY to 16MHz*/
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_CLK_FREQ, TMC4361A_CLK_FREQ_MASK,
			TMC4361A_CLK_FREQ_SHIFT, 0x00F42400);
	Trinamic_ConfigureInputFilters();
	Trinamic_ConfigureEndstops();
	Trinamic_ConfigureSPIOutput();
	Trinamic_ConfigureInterrupts();
	Trinamic_ConfigureMisc();
	Trinamic_ConfigureChopper();
	//Trinamic_ConfigureStealthChop();
	//Trinamic_ConfigureDCStep();
	Trinamic_ConfigureStallguard();
	//Trinamic_ConfigureCoolstep();

	/** Read events register to clear */
	tmc4361A_readInt(&TMC4361A, TMC4361A_EVENTS);

	/** Enable periodic job timer */
	//  if (HAL_TIM_OC_Start_IT(&htim15, TIM_CHANNEL_1) != HAL_OK) {
	//    _Error_Handler(__FILE__, __LINE__);
	//  }
}

void Trinamic_ConfigureInputFilters(void) {
	/* CONFIGURE DIGITAL INPUT FILTERING */
	/* Set sample rate for encoder interface pins to FCLK/16 */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_INPUT_FILT_CONF,
			TMC4361A_SR_ENC_IN_MASK, TMC4361A_SR_ENC_IN_SHIFT, 4);
	/* Set sample rate for reference input pins to FCLK/16 */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_INPUT_FILT_CONF,
			TMC4361A_SR_REF_MASK, TMC4361A_SR_REF_SHIFT, 4);
	/* Set sample rate for start input pin to FCLK/16 */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_INPUT_FILT_CONF,
			TMC4361A_SR_S_MASK, TMC4361A_SR_S_SHIFT, 4);
	/* Set sample rate for master clock input pins of encoder output interface to FCLK/16 */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_INPUT_FILT_CONF,
			TMC4361A_SR_ENC_OUT_MASK, TMC4361A_SR_ENC_OUT_SHIFT, 4);
	/* Set filter length for encoder interface pins to FCLK/16 */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_INPUT_FILT_CONF,
			TMC4361A_FILT_L_ENC_IN_MASK, TMC4361A_FILT_L_ENC_IN_SHIFT, 3);
	/* Set filter length for reference input pins to FCLK/16 */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_INPUT_FILT_CONF,
			TMC4361A_FILT_L_REF_MASK, TMC4361A_FILT_L_REF_SHIFT, 3);
	/* Set filter length for start input pin to FCLK/16 */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_INPUT_FILT_CONF,
			TMC4361A_FILT_L_S_MASK, TMC4361A_FILT_L_S_SHIFT, 3);
	/* Set filter length for master clock input pins of encoder output interface to FCLK/16 */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_INPUT_FILT_CONF,
			TMC4361A_FILT_L_ENC_OUT_MASK, TMC4361A_FILT_L_ENC_OUT_SHIFT, 3);
	/* Set sample rate & filter length for step/dir input pins to match encoder interface pin setting */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_INPUT_FILT_CONF,
			TMC4361A_SD_FILT0_MASK, TMC4361A_SD_FILT0_SHIFT, 1);
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_INPUT_FILT_CONF,
			TMC4361A_SD_FILT0_MASK, TMC4361A_SD_FILT1_SHIFT, 0);
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_INPUT_FILT_CONF,
			TMC4361A_SD_FILT0_MASK, TMC4361A_SD_FILT2_SHIFT, 0);
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_INPUT_FILT_CONF,
			TMC4361A_SD_FILT0_MASK, TMC4361A_SD_FILT3_SHIFT, 0);
}
void Trinamic_ConfigureSPIOutput(void) {
	/* CONFIGURE SPI OUTPUT FORMAT */
	/* Set SPI clock low time (4 clock cycles) */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_SPIOUT_CONF,
			TMC4361A_SPI_OUT_LOW_TIME_MASK, TMC4361A_SPI_OUT_LOW_TIME_SHIFT,
			0x04);
	/* Set SPI clock high time (4 clock cycles) */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_SPIOUT_CONF,
			TMC4361A_SPI_OUT_HIGH_TIME_MASK, TMC4361A_SPI_OUT_HIGH_TIME_SHIFT,
			0x04);
//  /* Set SPI clock block time (8 clock cycles.) Not supported in API??  */
//  TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_SPIOUT_CONF,
//                           TMC4361A_SPI_OUT_BLOCK_TIME_MASK, TMC4361A_SPI_OUT_BLOCK_TIME_SHIFT, 0x04);
	/* Set cover data length to zero for TMC drivers (40-bit cover datagrams) */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_SPIOUT_CONF,
			TMC4361A_COVER_DATA_LENGTH_MASK, TMC4361A_COVER_DATA_LENGTH_SHIFT,
			0x00);
	/* Set SPI output format to support attached TMC2160 in S/D mode */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_SPIOUT_CONF,
			TMC4361A_SPI_OUTPUT_FORMAT_MASK, TMC4361A_SPI_OUTPUT_FORMAT_SHIFT,
			0x0C);
	/* Enable cover done update only for cover register */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_SPIOUT_CONF,
			TMC4361A_COVER_DONE_ONLY_FOR_COVER_MASK,
			TMC4361A_COVER_DONE_ONLY_FOR_COVER_SHIFT, 0x01);

}

void Trinamic_ConfigureInterrupts(void) {
	/* Configure TMC4361A INTR pin for low-active polarity */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_GENERAL_CONF,
			TMC4361A_INTR_POL_MASK, TMC4361A_INTR_POL_SHIFT, 0x00);
	/* Configure TMC4361A INTR pin as strongly-driven output*/
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_GENERAL_CONF,
			TMC4361A_INTR_TR_PU_PD_EN_MASK, TMC4361A_INTR_TR_PU_PD_EN_SHIFT,
			0x00);

	/** Enable DIAG0 output on driver error (overtemperature, short to ground, undervoltage chargepump) */
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_GCONF,
			TMC2160_DIAG0_ERROR_ONLY_WITH_SD_MODE1_MASK,
			TMC2160_DIAG0_ERROR_ONLY_WITH_SD_MODE1_SHIFT, 0x01);
	/** Enable DIAG0 output on overtemperature prewarning */
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_GCONF,
			TMC2160_DIAG0_OTPW_ONLY_WITH_SD_MODE1_MASK,
			TMC2160_DIAG0_OTPW_ONLY_WITH_SD_MODE1_SHIFT, 0x01);
	/** Enable DIAG0 output on motor stall. Make sure TCOOLTHRS is set before using this feature. */
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_GCONF, TMC2160_DIAG0_STALL_MASK,
			TMC2160_DIAG0_STALL_SHIFT, 0x01);
	/** Enable DIAG1 output when steps skipped > 0.  */
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_GCONF,
			TMC2160_DIAG1_STEPS_SKIPPED_MASK, TMC2160_DIAG1_STEPS_SKIPPED_SHIFT,
			0x01);
	/** Configure DIAG0 and DIAG1 outputs as open collectors */
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_GCONF,
			TMC2160_DIAG0_INT_PUSHPULL_MASK, TMC2160_DIAG0_INT_PUSHPULL_SHIFT,
			0x00);
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_GCONF,
			TMC2160_DIAG1_POSCOMP_PUSHPULL_MASK,
			TMC2160_DIAG1_POSCOMP_PUSHPULL_SHIFT, 0x00);
}

void Trinamic_ConfigureChopper(void) {
	/** Set delay time (clock cycles) after velocity goes to zero to switch from IRUN to IHOLD */
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_TPOWERDOWN, TMC2160_TPOWERDOWN_MASK,
			TMC2160_TPOWERDOWN_SHIFT, 0x0A);
	/* Set upper velocity for StealthChop voltage PWM mode */
//  TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_TPWMTHRS, TMC2160_TPWMTHRS_MASK,
//                       TMC2160_TPWMTHRS_SHIFT, 0x01);
	/** Set threshold below which CoolStep & StallGuard are disabled */
//	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_TCOOLTHRS, TMC2160_TCOOLTHRS_MASK,
//			TMC2160_TCOOLTHRS_SHIFT, 0xC8);
	/** Set velocity threshold at which the driver should switch into/out of high speed mode*/
//	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_THIGH, TMC2160_THIGH_MASK,
//			TMC2160_THIGH_SHIFT, 0x32);
	/** Configure StealthChop settings via PWMCONF */
//  TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_PWMCONF, TMC2160_PWM_AMPL_MASK,
//                       TMC2160_PWM_AMPL_SHIFT, 0xFF);
//  TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_PWMCONF, TMC2160_PWM_GRAD_MASK,
//                       TMC2160_PWM_GRAD_SHIFT, 0x04);
//  TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_PWMCONF, TMC2160_PWM_AUTOSCALE_MASK,
//                       TMC2160_PWM_AUTOSCALE_SHIFT, 0x01);
	/** Configure chopper settings via CHOPCONF */
//  TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_CHOPCONF, TMC2160_TOFF_MASK,
//                       TMC2160_TOFF_SHIFT, 0x03);
//  TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_CHOPCONF, TMC2160_HSTRT_MASK,
//                       TMC2160_HSTRT_SHIFT, 0x02);
//  TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_CHOPCONF, TMC2160_HEND_MASK,
//                       TMC2160_HEND_SHIFT, 0x02);
//  TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_CHOPCONF, TMC2160_TBL_MASK,
//                       TMC2160_TBL_SHIFT, 0x02);
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_PWMCONF, TMC2160_PWM_AMPL_MASK,
			TMC2160_PWM_AMPL_SHIFT, 0xC8);
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_PWMCONF, TMC2160_PWM_GRAD_MASK,
			TMC2160_PWM_GRAD_SHIFT, 0x01);
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_PWMCONF, TMC2160_PWM_FREQ_MASK,
			TMC2160_PWM_FREQ_SHIFT, 0x02);
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_PWMCONF, TMC2160_PWM_AUTOSCALE_MASK,
			TMC2160_PWM_AUTOSCALE_SHIFT, 0x01);
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_PWMCONF, TMC2160_PWM_SYMMETRIC_MASK,
			TMC2160_PWM_SYMMETRIC_SHIFT, 0x01);
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_PWMCONF, TMC2160_PWM_SYMMETRIC_MASK,
			TMC2160_PWM_SYMMETRIC_SHIFT, 0x01);

	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_CHOPCONF, TMC2160_TOFF_MASK,
			TMC2160_TOFF_SHIFT, 0x03);
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_CHOPCONF, TMC2160_HSTRT_MASK,
			TMC2160_HSTRT_SHIFT, 0x05);
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_CHOPCONF, TMC2160_HEND_MASK,
			TMC2160_HEND_SHIFT, 0x02);
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_CHOPCONF, TMC2160_CHM_MASK,
			TMC2160_CHM_SHIFT, 0x00);
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_CHOPCONF, TMC2160_TBL_MASK,
			TMC2160_TBL_SHIFT, 0x02);
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_CHOPCONF, TMC2160_VHIGHFS_MASK,
			TMC2160_VHIGHFS_SHIFT, 0x00);
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_CHOPCONF, TMC2160_VHIGHCHM_MASK,
			TMC2160_VHIGHCHM_SHIFT, 0x00);
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_CHOPCONF, TMC2160_TFD_ALL_MASK,
			TMC2160_TFD_ALL_SHIFT, 0x04);
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_CHOPCONF, TMC2160_MRES_MASK,
			TMC2160_MRES_SHIFT, 0x00);
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_CHOPCONF, TMC2160_INTPOL_MASK,
			TMC2160_INTPOL_SHIFT, 0x00);
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_CHOPCONF, TMC2160_DEDGE_MASK,
			TMC2160_DEDGE_SHIFT, 0x00);
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_CHOPCONF, TMC2160_DISS2G_MASK,
			TMC2160_DISS2G_SHIFT, 0x00);
}

void Trinamic_ConfigureStallguard(void) {
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_COOLCONF, TMC2160_SFILT_MASK,
			TMC2160_SFILT_SHIFT, 0x01);
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_COOLCONF, TMC2160_SGT_MASK,
			TMC2160_SGT_SHIFT, 20);
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_VSTALL_LIMIT_WR,
			TMC4361A_VSTALL_LIMIT_MASK, TMC4361A_VSTALL_LIMIT_SHIFT, 0x6300);

	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_COOLCONF, TMC2160_SEMIN_MASK,
			TMC2160_SEMIN_SHIFT, 0x05);
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_COOLCONF, TMC2160_SEMAX_MASK,
			TMC2160_SEMAX_SHIFT, 0x02);
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_COOLCONF, TMC2160_SEDN_MASK,
			TMC2160_SEDN_SHIFT, 0x01);

	/** Enable stop on stall */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_REFERENCE_CONF, 0x04000000, 26,
			0x01);
}

void Trinamic_ConfigureCoolstep(void) {
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_COOLCONF, TMC2160_SEUP_MASK,
			TMC2160_SEUP_SHIFT, 0x01);

	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_COOLCONF, TMC2160_SEIMIN_MASK,
			TMC2160_SEIMIN_SHIFT, 0x01);
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_COOLCONF, TMC2160_SEIMIN_MASK,
			TMC2160_SEIMIN_SHIFT, 0x01);
}

void Trinamic_ConfigureDCStep(void) {
	/** Configure DCStep commutation settings */
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_DCCTRL, TMC2160_DC_TIME_MASK,
			TMC2160_DC_TIME_SHIFT, 0x25);

	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_DCCTRL, TMC2160_DC_SG_MASK,
			TMC2160_DC_SG_SHIFT, 0x70000);

	/** Activate DCStep */
	//TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_GENERAL_CONF, TMC4361A_DCSTEP_MODE_MASK, TMC4361A_DCSTEP_MODE_SHIFT, 0x02);
}

void Trinamic_ConfigureEndstops(void) {
	/** Configure endstops for active high polarity */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_REFERENCE_CONF,
			TMC4361A_POL_STOP_LEFT_MASK, TMC4361A_POL_STOP_LEFT_SHIFT, 1);
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_REFERENCE_CONF,
			TMC4361A_POL_STOP_RIGHT_MASK, TMC4361A_POL_STOP_RIGHT_SHIFT, 1);
	/** Enable endstops */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_REFERENCE_CONF,
			TMC4361A_STOP_LEFT_EN_MASK, TMC4361A_STOP_LEFT_EN_SHIFT, 1);
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_REFERENCE_CONF,
			TMC4361A_STOP_RIGHT_EN_MASK, TMC4361A_STOP_RIGHT_EN_SHIFT, 1);
	/** Configure motion controller to hard stop on switch activation */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_REFERENCE_CONF,
			TMC4361A_SOFT_STOP_EN_MASK, TMC4361A_SOFT_STOP_EN_SHIFT, 0x00);
	/** Configure motion controller to generate an interrupt on endstop
	 * activation */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_REFERENCE_CONF,
			TMC4361A_LATCH_X_ON_ACTIVE_L_MASK,
			TMC4361A_LATCH_X_ON_ACTIVE_L_SHIFT, 0x01);
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_REFERENCE_CONF,
			TMC4361A_LATCH_X_ON_ACTIVE_R_MASK,
			TMC4361A_LATCH_X_ON_ACTIVE_R_SHIFT, 0x01);

}

void Trinamic_ConfigureMisc(void) {

	/** Disable direct mode */
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_GCONF, TMC2160_DIRECT_MODE_MASK,
			TMC2160_DIRECT_MODE_SHIFT, 0x00);

	/** CONFIGURE HARDWARE SWITCHES */

	/** Initialize velocity, current position, and target position to zero*/
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_VMAX, TMC4361A_VMAX_MASK,
			TMC4361A_VMAX_SHIFT, 0);
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_XACTUAL, TMC4361A_XACTUAL_MASK,
			TMC4361A_XACTUAL_SHIFT, 0);
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_X_TARGET, TMC4361A_XTARGET_MASK,
			TMC4361A_XTARGET_SHIFT, 0);

}

void Trinamic_ConfigureStealthchop(void) {
	/** Set en_pwm_mode */
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_GCONF, TMC2160_EN_PWM_MODE_MASK,
			TMC2160_EN_PWM_MODE_SHIFT, 0x01);
	/** Set pwm_autoscale */
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_PWMCONF, TMC2160_PWM_AUTOSCALE_MASK,
			TMC2160_PWM_AUTOSCALE_SHIFT, 0x01);
	/** Set pwm_autograd */
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_PWMCONF, TMC2160_PWM_SYMMETRIC_SHIFT,
			TMC2160_PWM_SYMMETRIC_MASK, 0x01);
	/** Set pwm_freq to 2*F_CLK/1024 = 31250Hz @ 16MHz F_CLK */
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_PWMCONF, TMC2160_PWM_FREQ_MASK,
			TMC2160_PWM_FREQ_SHIFT, 0x00);
	/* Enable chopper using basic config TOFF = 5, TBL = 2, HSTART = 4, HEND = 0 */
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_CHOPCONF, TMC2160_TOFF_MASK,
			TMC2160_TOFF_SHIFT, 0x05);
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_CHOPCONF, TMC2160_TBL_MASK,
			TMC2160_TBL_SHIFT, 0x02);
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_CHOPCONF, TMC2160_HSTRT_MASK,
			TMC2160_HSTRT_SHIFT, 0x05);
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_CHOPCONF, TMC2160_HEND_MASK,
			TMC2160_HEND_SHIFT, 0x00);

}

void tmc4361A_readWriteArray(uint8_t channel, uint8_t *data, size_t length) {
	volatile HAL_StatusTypeDef res = HAL_OK;

	memcpy(dma_tx_buf, data, length);
	/* Drive NSS low */
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
//HAL_SPI_TransmitReceive_DMA(&hspi1, dma_tx_buf, dma_rx_buf, length);
	if ((res = HAL_SPI_TransmitReceive(&hspi1, dma_tx_buf, dma_rx_buf, length,
			100)) != HAL_OK) {
		asm("BKPT");
	}
	/* Drive NSS high */
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
	memcpy(data, dma_rx_buf, length);

}

void tmc2160_readWriteArray(uint8_t channel, uint8_t *data, size_t length) {
	tmc4361A_readWriteCover(&TMC4361A, data, length);
}

void SendDeviceAddress(void) {
	uint64_t eui64;
	/** Device address should have been set during POST.
	 * Check to make sure valid device address was retrieved.
	 * If not, retry address assignment.
	 */
	if (!params.device_id) {
		if ((EEPROM_GetAddress(&eui64) != HAL_OK) || (!eui64)) {
			LED_SetErrorState(ERRSTATE_SELFCHECK_FAIL);
			/* Send error message */
		} else {
			params.device_id = ((uint32_t) eui64) & 0x00FFFFFF;
		}
	} else {
		/** Generate CDXBus frame and send device ID to master**/

	}

}

void TuneStallGuard(void) {
//params.sgt =
}

void Test_LEDFunction(void) {
	testresults.led = 0;
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
	for (int i = 0; i < 5; i++) {
		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
		HAL_Delay(300);
		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
		HAL_Delay(300);
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
		HAL_Delay(300);
		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
		HAL_Delay(300);
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
		HAL_Delay(300);
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
		HAL_Delay(300);
	}

	LED_SetErrorState(ERRSTATE_NONE);
	HAL_Delay(3000);
	LED_SetErrorState(ERRSTATE_PAUSED);
	HAL_Delay(3000);
	LED_SetErrorState(ERRSTATE_ESTOP);
	HAL_Delay(3000);
	LED_SetErrorState(ERRSTATE_OVERHEAT);
	HAL_Delay(3000);
	LED_SetErrorState(ERRSTATE_OVERCURRENT);
	HAL_Delay(3000);
	LED_SetErrorState(ERRSTATE_UNINITIALIZED);
	HAL_Delay(3000);
	LED_SetErrorState(ERRSTATE_SELFCHECK_FAIL);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
	testresults.led = 1;
}

void Test_SPIFunction(void) {
	/** Test SPI connection to motion controller.
	 * Write random values to XACTUAL register and read back
	 * values to ensure that data matches.  */
	HAL_StatusTypeDef status = HAL_OK;
	uint32_t rand = 0, read = 0;
	testresults.spi = 0;
	status = HAL_RNG_GenerateRandomNumber(&hrng, &rand);
	if (status != HAL_OK) {
		/** Error: could not generate random number for SPI check. */
		LED_SetErrorState(ERRSTATE_SELFCHECK_FAIL);
		return;
	}
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_XACTUAL, TMC4361A_XACTUAL_MASK,
			TMC4361A_XACTUAL_SHIFT, rand);
	read = TMC4361A_FIELD_READ(&TMC4361A, TMC4361A_XACTUAL,
			TMC4361A_XACTUAL_MASK, TMC4361A_XACTUAL_SHIFT);
	if (read != rand) {
		/** Error: test value could not be written to motion controller properly. */
		LED_SetErrorState(ERRSTATE_SELFCHECK_FAIL);
		return;
	}

	/** Test SPI connection to gate driver.
	 * Write random values to XXX register via TMC4361A cover
	 * register and read back values via TMC4361A cover register
	 * to ensure that data matches.  */
	status = HAL_RNG_GenerateRandomNumber(&hrng, &rand);
	if (status != HAL_OK) {
		/** Error: could not generate random number for SPI check. */
		LED_SetErrorState(ERRSTATE_SELFCHECK_FAIL);
		return;
	}
	rand = (rand % 127) - 64; /** Bound rand to range [-64, 63] */
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_COOLCONF, TMC2160_SGT_MASK,
			TMC2160_SGT_SHIFT, rand);
	read = TMC2160_FIELD_READ(&TMC2160, TMC2160_COOLCONF, TMC2160_SGT_MASK,
			TMC2160_SGT_SHIFT);
	if (read != rand) {
		/** Error: test value could not be written to gate driver properly. */
		LED_SetErrorState(ERRSTATE_SELFCHECK_FAIL);
		return;
	}

	testresults.spi = 1;
}

void Test_EEPROMStorageFunction() {
	/** Test communication with AT24C01D. Last four bytes of
	 * EEPROM address space is used as test area */
	HAL_StatusTypeDef status = HAL_OK;
	uint32_t rand = 0, read = 0;
	testresults.eeprom_storage = 0;
	/** Generate random value to write to EEPROM */
	while (1) {
		status = HAL_RNG_GenerateRandomNumber(&hrng, &rand);
		if (status != HAL_OK) {
			/** Error: could not generate random number for SPI check. */
			LED_SetErrorState(ERRSTATE_SELFCHECK_FAIL);
			return;
		}

		/** Write random number to EEPROM test area */
		if ((status = EEPROM_WriteData(0, &rand, 4)) != HAL_OK) {
			/** Error: could not communicate with storage EEPROM. */
			LED_SetErrorState(ERRSTATE_SELFCHECK_FAIL);
			return;
		}

		/** Read byte written to the EEPROM test area */
		if ((status = EEPROM_ReadData(0, &read, 4)) != HAL_OK) {
			/** Error: could not communicate with storage EEPROM. */
			LED_SetErrorState(ERRSTATE_SELFCHECK_FAIL);
			return;
		}
		if (rand != read) {
			LED_SetErrorState(ERRSTATE_SELFCHECK_FAIL);
			return;
		}
	}
	testresults.eeprom_storage = 1;
}

void Test_EEPROMAddressFunction() {
	testresults.eeprom_address = 0;
	/** Test communication with 24AA025E48T-I/OT */
	/** Generate EUI-64 address from EUI-48 address stored on EEPROM */
	uint64_t eui64 = 0;
	/** Read byte written to the EEPROM test area */
	if (EEPROM_GetAddress(&eui64) != HAL_OK) {
		/** Error: could not communicate with EUI address EEPROM. */
		LED_SetErrorState(ERRSTATE_SELFCHECK_FAIL);
		return;
	}
	if (eui64 == 0) {
		/** Error: invalid value returned from EUI address EEPROM. */
		LED_SetErrorState(ERRSTATE_SELFCHECK_FAIL);
		return;
	}
	/* Set internal device address to unique lower 24-bits of 64-bit EUI address */
	params.device_id = ((uint32_t) eui64) & 0x00FFFFFF;
	testresults.eeprom_address = 1;
}

HAL_StatusTypeDef EEPROM_GetAddress(uint64_t *result) {
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t eui48[6] = { 0 }, eui64[8] = { 0 }, extender[2] = { 0xFF, 0xFE },
			control_byte = 0;
	control_byte |= (0x0A << 3); /* Set control code '0b1010' << 3 */
	control_byte |= (0x03); /* Set I2C address based on pin configuration */
	control_byte = control_byte << 1UL; /* Left-shift control byte to make valid 7-bit I2C address*/
	/** Read 48-bit eui address from EEPROM */
	if ((status = HAL_I2C_Mem_Read(&hi2c1, control_byte, EEPROM_EUI_ADDR,
	I2C_MEMADD_SIZE_8BIT, eui48, 6, 100)) != HAL_OK) {
		/** Error: could not communicate with EUI address EEPROM. */
		return status;
	}
	/* Create unique EUI-64 address from EUI-48 address stored on EEPROM*/
	memcpy(eui64, eui48, 3);
	memcpy(&(eui64[3]), extender, 2);
	memcpy(&(eui64[5]), &(eui48[3]), 3);
	/* Reverse ordering of bytes for address */
	for (int i = 0; i < 8; i++) {
		((uint8_t *) result)[i] = eui64[7 - i];
	}

	return status;
}

HAL_StatusTypeDef EEPROM_ReadData(uint8_t addr, uint8_t* data, uint32_t size) {
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t control_byte = 0;
	/** Bounds check on read parameters */
	if (addr > 127) {
		return HAL_ERROR;
	}
	if (size > 128) {
		return HAL_ERROR;
	}
	if ((addr + size) > 127) {
		return HAL_ERROR;
	}
	control_byte |= (0x0A << 4); /* Set control code '0b1010' << 4 */

	/* @TODO: Replace multiple iterations in single-byte read mode with optimized
	 * page read mode routine. */
	for (int i = 0; i < size; i++) {
		if ((status = HAL_I2C_Mem_Read(&hi2c1, control_byte,
				(uint16_t *) (addr + i),
				I2C_MEMADD_SIZE_8BIT, (uint8_t *) (data + i), 1, 100))
				!= HAL_OK) {
			/** Error: could not communicate with EUI address EEPROM. */
			return status;
		}
	}
	return status;
}

HAL_StatusTypeDef EEPROM_WriteData(uint8_t addr, uint8_t *data, uint32_t size) {
	HAL_StatusTypeDef status = HAL_OK;
	uint32_t n_boundaries = 0, offset = 0, page_start = 0, dlen = 0,
			bytes_written = 0;
	uint8_t device_addr = 0, page_buffer[8] = { 0 };

	/* Bounds check on memory address. AT24C01D has 128 bytes addressable */
	if (addr > 127) {
		return HAL_ERROR;
	}
	/* Bounds check on data length. Make sure we are not attempting to write out of the valid address space*/
	if ((addr + size) > 128) {
		return HAL_ERROR;
	}

	device_addr = (0x0A) << 3;

	/* @TODO: Replace multiple iterations in single-byte read mode with optimized
	 * page read mode routine. */
	for (int i = 0; i < size; i++) {
		if ((status = HAL_I2C_Mem_Write(&hi2c1, (device_addr << 1UL),
				(uint16_t *) (addr + i),
				I2C_MEMADD_SIZE_8BIT, (uint8_t *) (data + i), 1, 500))
				!= HAL_OK) {
			/** Error: could not communicate with storage EEPROM. */
			return status;
		}
	}

//	offset = addr % 8; /* Calculate address offset into page */
//	n_boundaries = (size + offset - 1) / 8; /* Calculate number of page boundaries crossed */
//	page_start = addr - offset; /* Calculate address of page start */
//
//	if (size > 1) {
//		/** Write to EEPROM in page write mode */
//
//		/** Write first few bytes */
//		/* Read page into buffer */
//		if ((status = HAL_I2C_Mem_Read(&hi2c1, device_addr, page_start,
//		I2C_MEMADD_SIZE_8BIT, page_buffer, 8, 100)) != HAL_OK) {
//			/** Error: could not communicate with storage EEPROM. */
//			return status;
//		}
//		memcpy(&page_buffer[offset], &addr, 8 - offset);
//		bytes_written = 8 - offset;
//		page_start += 8;
//
//		for (int i = 0; i < n_boundaries; i++) {
//			/* Read page into buffer */
//			if ((status = HAL_I2C_Mem_Read(&hi2c1, device_addr, page_start,
//			I2C_MEMADD_SIZE_8BIT, page_buffer, 8, 100)) != HAL_OK) {
//				/** Error: could not communicate with storage EEPROM. */
//				return status;
//			}
//			dlen = ((size - bytes_written) < 8) ? (size - bytes_written) : 8;
//			memcpy(&page_buffer[offset], (addr + bytes_written), dlen);
//			bytes_written += dlen;
//
//		}
//	} else {
//		/** Write to EEPROM in single byte write mode */
//
//		if ((status = HAL_I2C_Mem_Write(&hi2c1, device_addr, addr,
//		I2C_MEMADD_SIZE_8BIT, data, 1, 100)) != HAL_OK) {
//			/** Error: could not communicate with storage EEPROM. */
//			return status;
//		}
//	}
	return status;
}

void System_InitDefaultState(void) {
	/** Set system state defaults */
	flags.target_reached = 1;
	flags.run_periodic_job = 0;
	flags.rs485_bus_free = 1;
	flags.internal_temp_ready = 0;
	flags.address_requested = 0;
	flags.intr_triggered = 0;
	flags.home_reached = 0;
	params.microstep_resolution = 256;
	params.spr = 0.0f;
	params.lead = 0.0f;
	status.motion_params_cfgd = 0;
	status.homed = 0;
}
void System_RunSelfTest(void) {
	//	Test_LEDFunction();
	//Test_SPIFunction();
	Test_EEPROMAddressFunction();
	Test_EEPROMStorageFunction();
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM16_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM15_Init();
  MX_ADC1_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */

	/* Configure command parser */
	parser = sparse_New();
	parser->delimiter = ',';
	parser->line_end = '\n';

	sparse_RegisterCallback(parser, 'A', 6, SetAccelerationCallback);
	sparse_RegisterCallback(parser, 'B', 2, SetCurrentCallback);
	sparse_RegisterCallback(parser, 'C', 2, MoveContinuousCallback);
	sparse_RegisterCallback(parser, 'D', 0, GetEndstopStatusCallback);
	sparse_RegisterCallback(parser, 'E', 1, SetMicrostepResolutionCallback);
	sparse_RegisterCallback(parser, 'H', 5, HomeCallback);
	sparse_RegisterCallback(parser, 'J', 2, MoveAbsoluteCallback);
	sparse_RegisterCallback(parser, 'K', 2, MoveRelativeCallback);
	sparse_RegisterCallback(parser, 'M', 1, MoveStepsCallback);
	sparse_RegisterCallback(parser, 'N', 1, MoveMicrostepsCallback);
	sparse_RegisterCallback(parser, 'P', 1, GetPositionCallback);
	sparse_RegisterCallback(parser, 'R', 0, ResumeCallback);
	sparse_RegisterCallback(parser, 'S', 0, StopCallback);
#if INTERNAL_TEMPERATURE_SENSOR_ENABLED
	sparse_RegisterCallback(parser, 'T', 0, GetTemperatureCallback);
#endif
	sparse_RegisterCallback(parser, 'V', 1, SetVelocityCallback);
	sparse_RegisterCallback(parser, 'X', 3, SetMotionParametersCallback);
	sparse_RegisterCallback(parser, 'Z', 0, SystemResetCallback);
	sparse_RegisterCallback(parser, '#', 0, BootloaderCallback);
	sparse_RegisterCallback(parser, '*', 1, GetVersionCallback);
	sparse_RegisterCallback(parser, '!', 0, EmergencyStopCallback);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//HAL_TIM_Base_Start_IT(&htim1);
	/* Initialize motor controller and gate driver */
	Trinamic_init();
	System_InitDefaultState();
	System_RunSelfTest();

	/* Start UART Peripheral */
	if (HAL_UART_Receive_IT(&huart1, &it_cmd_buffer[0], 1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		// EN_PWM_MODE=1 enables stealthChop�, MULTISTEP_FILT=1, DIRECT_MODE=0 (off)
//		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_HIGH_WR, TMC2160_GCONF | 0x80);
//		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_LOW_WR, 0x0000000C);
//		HAL_Delay(1); // COVER_DONE flag: ~90�s -> 1 ms more than enough
//
//		// TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (spreadCycle�)
//		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_HIGH_WR, TMC2160_CHOPCONF | 0x80);
//		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_LOW_WR, 0x000100C3);
//		HAL_Delay(1); // COVER_DONE flag: ~90�s -> 1 ms more than enough
//
//		// IHOLD=8, IRUN=15 (max. current), IHOLDDELAY=6
//		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_HIGH_WR, TMC2160_IHOLD_IRUN | 0x80);
//		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_LOW_WR, 0x00080F0A);
//		HAL_Delay(1); // COVER_DONE flag: ~90�s -> 1 ms more than enough
//
//		// TPOWERDOWN=10: Delay before power down in stand still
//		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_HIGH_WR, TMC2160_TPOWERDOWN | 0x80);
//		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_LOW_WR, 0x0000000A);
//		HAL_Delay(1); // COVER_DONE flag: ~90�s -> 1 ms more than enough
//
//		// TPWMTHRS=5000
//		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_HIGH_WR, TMC2160_TPWMTHRS| 0x80);
//		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_LOW_WR, 0x00001388);
//		//SPI send: 0xEC000100C3; // CHOPCONF: TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (spreadCycle)
//		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_HIGH_WR, TMC2160_CHOPCONF | 0x80);
//		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_LOW_WR, 0x000100C3);
//		//SPI send: 0x90; // IHOLD_IRUN: IHOLD=10, IRUN=31 (max. current), IHOLDDELAY=6
//		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_HIGH_WR, TMC2160_IHOLD_IRUN | 0x80);
//		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_LOW_WR, 0x00061F0A);
//		//SPI send: 0x910000000A; // TPOWERDOWN=10: Delay before power down in stand still
//		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_HIGH_WR, TMC2160_TPOWERDOWN | 0x80);
//		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_LOW_WR, 0x0000000A);
//		//SPI send: 0x8000000004; // EN_PWM_MODE=1 enables stealthChop (with default PWM_CONF)
//		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_HIGH_WR, TMC2160_GCONF | 0x80);
//		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_LOW_WR, 0x00000004);
//		//SPI send: 0x93000001F4; // TPWM_THRS=500 yields a switching velocity about 35000 = ca. 30RPM
//		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_HIGH_WR, TMC2160_TPWMTHRS| 0x80);
//		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_LOW_WR, 0x000001F4);
//
		//sendData(0x80, 0x00000080); // GCONF -> Activate diag0_stall (Datasheet Page 31)
//		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_HIGH_WR,
//		TMC2160_GCONF | 0x80);
//		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_LOW_WR, 0x00000080);
		//sendData(0xED, 0x00000000); // SGT -> Needs to be adapted to get a StallGuard2 event
//		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_HIGH_WR, 0x6D | 0x80);
//		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_LOW_WR, 0x00000007);
		//sendData(0x94, 0x00000040); // TCOOLTHRS -> TSTEP based threshold = 55 (Datasheet Page 38)
		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_HIGH_WR, 0x94);
		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_LOW_WR, 0x00000040);
		//sendData(0x89, 0x00010606);      // SHORTCONF
		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_HIGH_WR, 0x89);
		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_LOW_WR, 0x00010606);
		//sendData(0x8A, 0x00080400);      // DRV_CONF
		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_HIGH_WR, 0x8A);
		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_LOW_WR, 0x00080400);
		//sendData(0x90, 0x00080303);      // IHOLD_IRUN
		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_HIGH_WR, 0x90);
		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_LOW_WR, 0x00080303);
		//sendData(0x91, 0x0000000A);      // TPOWERDOWN
		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_HIGH_WR, 0x91);
		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_LOW_WR, 0x0000000A);
		//sendData(0xAB, 0x00000001);      // VSTOP
		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_HIGH_WR, 0xAB);
		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_LOW_WR, 0x00000001);
		//sendData(0xBA, 0x00000001);      // ENC_CONST
		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_HIGH_WR, 0xBA);
		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_LOW_WR, 0x00000001);
		//sendData(0xEC, 0x15410153);      // CHOPCONF
		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_HIGH_WR, 0xEC);
		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_LOW_WR, 0x15410153);
		//sendData(0xF0, 0xC40C001E);      // PWMCONF
		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_HIGH_WR, 0xF0);
		tmc4361A_writeInt(&TMC4361A, TMC4361A_COVER_LOW_WR, 0xC40C001E);

		sparse_Exec(parser, "X,1,200,1");
		sparse_Exec(parser, "A,500,500,500,500,4,4");
		sparse_Exec(parser, "B,400,400");
		sparse_Exec(parser, "C,1,1");

		while (1) {
			ProcessEventFlags();
			res = TMC2160_FIELD_READ(&TMC2160, TMC2160_DRV_STATUS,
					TMC2160_SG_RESULT_MASK, TMC2160_SG_RESULT_SHIFT);
			velcur = TMC4361A_FIELD_READ(&TMC4361A, TMC4361A_VACTUAL,
					TMC4361A_VACTUAL_MASK, TMC4361A_VACTUAL_SHIFT);
			int status = tmc4361A_readInt(&TMC4361A, TMC4361A_STATUS);
			int events = tmc4361A_readInt(&TMC4361A, TMC4361A_EVENTS);
			sgstat = TMC4361A_FIELD_READ(&TMC4361A, TMC4361A_STATUS, 0x01000000,
					24);
			int refconf = tmc4361A_readInt(&TMC4361A, TMC4361A_REFERENCE_CONF);
			int coolconf = tmc2160_readInt(&TMC2160, TMC2160_COOLCONF);
			int chopconf = tmc2160_readInt(&TMC2160, TMC2160_CHOPCONF);
			int gconf2160 = tmc2160_readInt(&TMC2160, TMC2160_GCONF);
			HAL_Delay(10);
		}

//		if (new_cmd_flag) {
//			ProcessCmdBuffer();
//			sparse_Exec(parser, sys_cmd_buffer);
//		}
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RNG|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_2);
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* Application Callback Functions */
sparse_StatusTypeDef SetAccelerationCallback(sparse_ArgPack *a) {
	int32_t amax = FIXED_22_2_MAKE(
			ConvertDistanceToFullStepsFloat(atof(a->arg_list[0])));
	int32_t dmax = FIXED_22_2_MAKE(
			ConvertDistanceToFullStepsFloat(atof(a->arg_list[1])));
	int32_t bow1 = INT_24_TRUNCATE(
			ConvertDistanceToFullStepsInt(atof(a->arg_list[2])));
	int32_t bow2 = INT_24_TRUNCATE(
			ConvertDistanceToFullStepsInt(atof(a->arg_list[3])));
	int32_t bow3 = INT_24_TRUNCATE(
			ConvertDistanceToFullStepsInt(atof(a->arg_list[4])));
	int32_t bow4 = INT_24_TRUNCATE(
			ConvertDistanceToFullStepsInt(atof(a->arg_list[5])));

	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_AMAX,
			TMC4361A_FREQUENCY_MODE_MASK, TMC4361A_FREQUENCY_MODE_SHIFT, amax);
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_DMAX,
			TMC4361A_FREQUENCY_MODE_MASK, TMC4361A_FREQUENCY_MODE_SHIFT, dmax);
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_BOW1,
			TMC4361A_FREQUENCY_MODE_MASK, TMC4361A_FREQUENCY_MODE_SHIFT, bow1);
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_BOW2,
			TMC4361A_FREQUENCY_MODE_MASK, TMC4361A_FREQUENCY_MODE_SHIFT, bow2);
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_BOW3,
			TMC4361A_FREQUENCY_MODE_MASK, TMC4361A_FREQUENCY_MODE_SHIFT, bow3);
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_BOW4,
			TMC4361A_FREQUENCY_MODE_MASK, TMC4361A_FREQUENCY_MODE_SHIFT, bow4);

	/* Default ramp generator values*/
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_ASTART,
			TMC4361A_FREQUENCY_MODE_MASK, TMC4361A_FREQUENCY_MODE_SHIFT, 0);
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_DFINAL,
			TMC4361A_FREQUENCY_MODE_MASK, TMC4361A_FREQUENCY_MODE_SHIFT, 0);
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_VSTART,
			TMC4361A_FREQUENCY_MODE_MASK, TMC4361A_FREQUENCY_MODE_SHIFT, 0);
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_VSTOP,
			TMC4361A_FREQUENCY_MODE_MASK, TMC4361A_FREQUENCY_MODE_SHIFT, 0);

	ts_write("@,a");
	return SPARSE_OK;
}
sparse_StatusTypeDef SetCurrentCallback(sparse_ArgPack *a) {
	/*
	 Requested current = mA = I_rms/1000
	 Equation for current:
	 I_rms = GLOBALSCALER/256 * (CS+1)/32 * V_fs/R_sense * 1/sqrt(2)
	 Solve for GLOBALSCALER ->

	 32 * 256 * sqrt(2) * I_rms * R_sense    |

	 GLOBALSCALER = ------------------------------------    |

	 (CS + 1) * V_fs               | V_fs = 0.325

	 */
	float drive_current = atof(a->arg_list[0]);
	float hold_current = atof(a->arg_list[1]);

	uint32_t V_fs = 325; // 0.325 * 1000
	uint8_t CS = 31;
	uint32_t scaler = 0; // = 256
	const uint16_t RS_scaled = R_SENSE * 0xFFFF; // Scale to 16b
	uint32_t numerator = 11585; // 32 * 256 * sqrt(2)
	numerator *= RS_scaled;
	numerator >>= 8;
	numerator *= drive_current;
	do {
		uint32_t denominator = V_fs * 0xFFFF >> 8;
		denominator *= CS + 1;
		scaler = numerator / denominator;
		if (scaler > 255)
			scaler = 0; // Maximum
		else if (scaler < 128)
			CS--;  // Try again with smaller CS
	} while (0 < scaler && scaler < 128);
	//GLOBAL_SCALER(scaler);
	/* Set global scaler */
	TMC2160_FIELD_UPDATE(&TMC2160, 0x0B, 0x000000FF, 0, scaler);
	/* Bounds check on current values */
	drive_current = (drive_current > 31) ? 31 : drive_current;
	hold_current = (hold_current > 31) ? 31 : hold_current;
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_IHOLD_IRUN, TMC2160_IRUN_MASK,
			TMC2160_IRUN_SHIFT, CS);
	TMC2160_FIELD_UPDATE(&TMC2160, TMC2160_IHOLD_IRUN, TMC2160_IHOLD_MASK,
			TMC2160_IHOLD_SHIFT, (int)(CS * HOLD_CURRENT_MULTIPLIER));

	ts_write("@,b");
	return SPARSE_OK;
}

sparse_StatusTypeDef MoveContinuousCallback(sparse_ArgPack *a) {
	/** If motion parameters have not yet been configured, indicate an error
	 * condition and return */
	if (!status.motion_params_cfgd) {
		ts_write("!,C1");
	}
	/** Since the TMC4361A takes a signed value for velocity, the dir parameter can be deprecated. */
	//uint8_t dir = atoi(a->arg_list[0]);
	float vel = atof(a->arg_list[1]);

	/** Configure ramp for motion (S-shaped ramp in constant velocity mode) */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_RAMPMODE,
			TMC4361A_RAMP_PROFILE_MASK, TMC4361A_RAMP_PROFILE_SHIFT, 0x02);

	/** Set velocity and start motion */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_VMAX, TMC4361A_VMAX_MASK,
			TMC4361A_VMAX_SHIFT,
			FIXED_23_8_MAKE( ConvertDistanceToMicroStepsFloat(vel)));

	flags.target_reached = 0;
	ts_write("@,c");
	return SPARSE_OK;
}

sparse_StatusTypeDef GetEndstopStatusCallback(sparse_ArgPack *a) {
	ts_write("@,d");
	return SPARSE_OK;
}

sparse_StatusTypeDef SetMicrostepResolutionCallback(sparse_ArgPack *a) {
	uint16_t res = (uint16_t) (atof(a->arg_list[0]) + 0.5f);
	uint8_t i = 0;
	uint16_t msr[] = { 256, 128, 64, 32, 16, 8, 4, 2, 1 };
	/** If desired microstep resolution is greater than 256, set the resolution
	 * to 256, which is the maximum resolution supported by the driver. Else,
	 * traverse the list of possible settings and find the index of the closest
	 * selectable value.
	 */

	/** @todo: Generate warning if selected microstep resolution does not match
	 * assigned microstep resolution. */
	if (res < 256) {
		while ((res < msr[i])) {
			if (res > msr[i + 1]) {
				i = ((msr[i] - res) < (res - msr[i + 1])) ? i : ++i;
				break;
			}
		}
	}
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_STEP_CONF,
			TMC4361A_MSTEP_PER_FS_MASK, TMC4361A_MSTEP_PER_FS_SHIFT, i);
	params.microstep_resolution = msr[i];
	ts_write("@,e");
	return SPARSE_OK;
}

sparse_StatusTypeDef HomeCallback(sparse_ArgPack *a) {
	uint8_t dir = atoi(a->arg_list[0]);
	float home_vel_primary = fabs(atof(a->arg_list[1]));
	float retract_distance = fabs(atof(a->arg_list[3])); /* mm */
	float home_offset = atof(a->arg_list[4]); /* mm*/

	if (!status.motion_params_cfgd) {
		ts_write("!,H1");
		return SPARSE_ERROR;
	}
	/* Set homing direction and corresponding endstop */
	if (dir == 0) {
		TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_REFERENCE_CONF,
				TMC4361A_STOP_LEFT_IS_HOME_MASK,
				TMC4361A_STOP_LEFT_IS_HOME_SHIFT, 0x01);
	} else {
		/* TMC4361A_STOP_RIGHT_IS_HOME_MASK and TMC4361A_STOP_RIGHT_IS_HOME_SHIFT
		 * are not defined in the TMC4361A library, so we make the appropriate mask
		 * by shifting TMC4361A_STOP_LEFT_IS_HOME_MASK and adding one to
		 * TMC4361A_STOP_LEFT_IS_HOME_SHIFT*/
		TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_REFERENCE_CONF,
				(TMC4361A_STOP_LEFT_IS_HOME_MASK << 1),
				(TMC4361A_STOP_LEFT_IS_HOME_SHIFT + 1), 0x01);
	}
	/** HOME_REF == 0 indicates positive direction to the right of X_HOME. */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_REFERENCE_CONF,
			TMC4361A_HOME_EVENT_MASK, TMC4361A_HOME_EVENT_SHIFT, 0x0C);

	/** Configure ramp for primary homing motion (S-shaped ramp in constant velocity mode) */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_RAMPMODE,
			TMC4361A_RAMP_PROFILE_MASK, TMC4361A_RAMP_PROFILE_SHIFT, 0x02);

	/** Enable home tracking mode */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_REFERENCE_CONF,
			TMC4361A_START_HOME_TRACKING_MASK,
			TMC4361A_START_HOME_TRACKING_SHIFT, 0x01);

	/** Start motion toward home switch at home_vel_primary*/
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_VMAX, TMC4361A_VMAX_MASK,
			TMC4361A_VMAX_SHIFT,
			FIXED_23_8_MAKE( ConvertDistanceToFullStepsFloat(home_vel_primary)));

	/* Wait until home event has been triggered */
	while (!flags.home_reached) {

	}

	/** Update current position with home offset */
	if (home_offset) {
		TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_VACTUAL,
				TMC4361A_VACTUAL_MASK, TMC4361A_VACTUAL_SHIFT,
				ConvertDistanceToMicroStepsInt(home_offset));
	}
	status.homed = 1;
	ts_write("@,h");
	return SPARSE_OK;
}

sparse_StatusTypeDef MoveAbsoluteCallback(sparse_ArgPack *a) {
	if (!status.motion_params_cfgd) {
		ts_write("!,J1");
	}

	float dist = atof(a->arg_list[0]);
	uint32_t vel = atof(a->arg_list[1]);
	/** Configure ramp for motion */
	/* S-shaped ramp in position mode */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_RAMPMODE,
			TMC4361A_RAMP_PROFILE_MASK, TMC4361A_RAMP_PROFILE_SHIFT, 0x06);
	/* Set velocity */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_VMAX, TMC4361A_VMAX_MASK,
			TMC4361A_VMAX_SHIFT,
			FIXED_23_8_MAKE( ConvertDistanceToFullStepsFloat(vel)));
	/* Start motion */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_X_TARGET, TMC4361A_XTARGET_MASK,
			TMC4361A_XTARGET_SHIFT,
			FIXED_23_8_MAKE( ConvertDistanceToFullStepsFloat(dist)));
	flags.target_reached = 0;
	ts_write("@,j");
	return SPARSE_OK;
}

sparse_StatusTypeDef MoveRelativeCallback(sparse_ArgPack *a) {
	/** If motion parameters have not yet been configured, indicate an error
	 * condition and return */
	if (!status.motion_params_cfgd) {
		ts_write("!,K1");
	}
	float dist = atof(a->arg_list[0]);
	uint32_t vel = FIXED_23_8_MAKE(
			ConvertDistanceToMicroStepsFloat(atof(a->arg_list[1])));
	//uint8_t dir = atoi(a->arg_list[2]);  // Distance is signed. this field should be deprecated.
	uint32_t cpos_us;

	/** Get current position (microsteps) */
	cpos_us = TMC4361A_FIELD_READ(&TMC4361A, TMC4361A_XACTUAL,
			TMC4361A_XACTUAL_MASK, TMC4361A_XACTUAL_SHIFT);

	/** Configure ramp for motion (S-shaped ramp in positioning mode) */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_RAMPMODE,
			TMC4361A_RAMP_PROFILE_MASK, TMC4361A_RAMP_PROFILE_SHIFT, 0x06);
	/** Set XTARGET */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_X_TARGET, TMC4361A_XTARGET_MASK,
			TMC4361A_XTARGET_SHIFT,
			cpos_us + ConvertDistanceToMicroStepsInt(dist));
	/** Set velocity and start motion */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_VMAX, TMC4361A_VMAX_MASK,
			TMC4361A_VMAX_SHIFT, vel);

	flags.target_reached = 0;
	ts_write("@,k");
	return SPARSE_OK;
}

sparse_StatusTypeDef MoveStepsCallback(sparse_ArgPack *a) {
	/** If motion parameters have not yet been configured, indicate an error
	 * condition and return */
	if (!status.motion_params_cfgd) {
		ts_write("!,M1");
	}
	uint32_t steps = atoi(a->arg_list[0]) * params.microstep_resolution;
	uint32_t vel = FIXED_23_8_MAKE(
			ConvertDistanceToMicroStepsFloat(atof(a->arg_list[1])));

	/** Get current position (microsteps) */
	uint32_t cpos_us = TMC4361A_FIELD_READ(&TMC4361A, TMC4361A_XACTUAL,
			TMC4361A_XACTUAL_MASK, TMC4361A_XACTUAL_SHIFT);
	/** Set XTARGET */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_X_TARGET, TMC4361A_XTARGET_MASK,
			TMC4361A_XTARGET_SHIFT, (cpos_us + steps));

	/** Configure ramp for motion (S-shaped ramp in positioning mode) */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_RAMPMODE,
			TMC4361A_RAMP_PROFILE_MASK, TMC4361A_RAMP_PROFILE_SHIFT, 0x06);
	/** Set velocity and start motion */
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_VMAX, TMC4361A_VMAX_MASK,
			TMC4361A_VMAX_SHIFT, vel);

	flags.target_reached = 0;
	ts_write("@,m");
	return SPARSE_OK;
}

sparse_StatusTypeDef MoveMicrostepsCallback(sparse_ArgPack *a) {
	/** If motion parameters have not yet been configured, indicate an error
	 * condition and return */
	if (!status.motion_params_cfgd) {
		ts_write("!,N1");
	}
	flags.target_reached = 0;
	ts_write("@,n");
	return SPARSE_OK;
}

sparse_StatusTypeDef GetPositionCallback(sparse_ArgPack *a) {
	char buf[32] = { 0 };
	/* Read internal position out of XTARGET register, convert from
	 * steps to linear units and return*/
	int32_t x = TMC4361A_FIELD_READ(&TMC4361A, TMC4361A_XACTUAL,
			TMC4361A_XACTUAL_MASK, TMC4361A_XACTUAL_SHIFT);
	sprintf(buf, "@,p,%ld", x);
	ts_write(buf);
	return SPARSE_OK;
}

sparse_StatusTypeDef ResumeCallback(sparse_ArgPack *a) {
	ts_write("@,r");
	return SPARSE_OK;
}

sparse_StatusTypeDef StopCallback(sparse_ArgPack *a) {
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_VMAX, TMC4361A_VMAX_MASK,
			TMC4361A_VMAX_SHIFT, 0);
	ts_write("@,s");
	return SPARSE_OK;
}

sparse_StatusTypeDef GetTemperatureCallback(sparse_ArgPack *a) {
	HAL_ADC_Start_IT(&hadc1);
	ts_write("@,t");
	return SPARSE_OK;
}

sparse_StatusTypeDef SetVelocityCallback(sparse_ArgPack *a) {
	int32_t vmax = FIXED_23_8_MAKE(
			ConvertDistanceToFullStepsFloat(atof(a->arg_list[0])));
	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_VMAX, TMC4361A_VMAX_MASK,
			TMC4361A_VMAX_SHIFT, vmax);
	ts_write("@,v");
	return SPARSE_OK;
}

sparse_StatusTypeDef SetMotionParametersCallback(sparse_ArgPack *a) {
	float screw_lead = atof(a->arg_list[0]);
	uint32_t spr = atoi(a->arg_list[1]);
	uint8_t dir = atoi(a->arg_list[2]);

	params.lead = screw_lead;
	params.spr = spr;

	TMC4361A_FIELD_UPDATE(&TMC4361A, TMC4361A_STEP_CONF,
			TMC4361A_FS_PER_REV_MASK, TMC4361A_FS_PER_REV_SHIFT, spr);

	status.motion_params_cfgd = 1;
	ts_write("@,x");
	return SPARSE_OK;
}

sparse_StatusTypeDef SystemResetCallback(sparse_ArgPack *a) {
	/* Software reset all parameters to OTP defaults. */
	TMC4361A_Reset();
	NVIC_SystemReset();
	return SPARSE_OK;
}

sparse_StatusTypeDef BootloaderCallback(sparse_ArgPack *a) {
	bootloader_flag = 1;
	NVIC_SystemReset();
	/* Should never get here */
	return SPARSE_OK;
}

sparse_StatusTypeDef GetVersionCallback(sparse_ArgPack *a) {
	ts_write(git_sha);
	return SPARSE_OK;
}

sparse_StatusTypeDef EmergencyStopCallback(sparse_ArgPack *a) {
	/** Pull FREEZE pin low to disable motion controller and gate driver outputs.
	 * Pulling freeze pin low on TMC4361A activates freeze event and requires
	 * reset to begin motion again. */
	HAL_GPIO_WritePin(FREEZE_GPIO_Port, FREEZE_Pin, GPIO_PIN_RESET);
	LED_SetErrorState(ERRSTATE_ESTOP);
	ts_write("!");
	return SPARSE_OK;
}

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
	if (HAL_UART_Receive_IT(&huart1, &it_cmd_buffer[it_cmd_buf_cnt], 1)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
}

void LED_SetErrorState(uint32_t state) {
	static uint8_t timer_active = 0;
	if (!timer_active) {
		/** Start timer for status LED blink frequency*/
		if (HAL_TIM_OC_Start_IT(&htim16, TIM_CHANNEL_1) != HAL_OK) {
			_Error_Handler(__FILE__, __LINE__);
		}
		timer_active = 1;
	}
	GPIO_PinState r = GPIO_PIN_RESET, g = GPIO_PIN_RESET, b = GPIO_PIN_RESET;
	uint8_t mult = 0;
	r = ((state & (1UL << 2)) >> 2) ? GPIO_PIN_RESET : GPIO_PIN_SET;
	g = ((state & (1UL << 1)) >> 1) ? GPIO_PIN_RESET : GPIO_PIN_SET;
	b = (state & 1UL) ? GPIO_PIN_RESET : GPIO_PIN_SET;
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
	SysMemBootJump = (void (*)(void)) (*((uint32_t *) (addr + 4)));
	__set_MSP(*(uint32_t *) addr);
	SysMemBootJump();
}

void ts_write(const char *s) {
	char buff[512];
	float time_s = HAL_GetTick() / 1000.0f;
	int n = sprintf(buff, "%s,%f\n", s, time_s);
	HAL_UART_Transmit_IT(&huart1, (uint8_t *) &buff, n);
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line
	 number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
