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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../modbus/mb.h"
#include "../../modbus/mbport.h"
#include "../../dcmotor/dcmotor.h"

#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define REG_HOLDING_START 1
#define REG_HOLDING_NREGS 52
#define REG_INPUT_START 1
#define REG_INPUT_NREGS 24
#define TEMPERATURE_ADC_BUFLEN 5

#define M1 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static USHORT usRegHoldingStart = REG_HOLDING_START;
static USHORT usRegHoldingBuf[REG_HOLDING_NREGS];
static USHORT usRegInputStart = REG_INPUT_START;
static USHORT usRegInputBuf[REG_INPUT_NREGS];

union {
	float f;
	LONG l;
	USHORT us[2];
	uint32_t ul;
} __32bitsRegBuf;

enum {
	HREG_M1_RUN,
	HREG_M2_RUN,
	HREG_M1_DIR,
	HREG_M2_DIR,
	HREG_M1_SPEED,
	HREG_M2_SPEED,
	HREG_M1_LIMIT_STATE,
	HREG_M2_LIMIT_STATE
} holdingRegs_t;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

uint8_t fetchInputRegsData(int iRegIndex);
uint8_t fetchHoldingRegsData(int iRegIndex);
uint8_t validateWriteHoldingRegs(int iRegIndex, uint16_t usNRegs);
void writeHoldingRegs(int iRegIndex, uint16_t tempReg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

uint16_t data[1];
uint32_t DestAddress = (uint32_t) &(TIM4->CCR1);

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
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM7_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  for (size_t i = 0; i < REG_HOLDING_NREGS; ++i) {
	  usRegHoldingBuf[i] = 0;
  }

  for (size_t i = 0; i < REG_INPUT_NREGS; ++i) {
	  usRegInputBuf[i] = 0;
  }

  dcmotor_Init(M1);

  eMBInit(MB_RTU, 4, 3, 9600, MB_PAR_NONE);
  eMBEnable();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	eMBPoll();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint8_t fetchInputRegsData(int iRegIndex)
{
	uint8_t numUs = 1;
	return numUs;
}

uint8_t fetchHoldingRegsData(int iRegIndex)
{
	uint8_t numUs = 1;

	switch (iRegIndex) {
		case HREG_M1_RUN:
			break;
		case HREG_M2_RUN:
			break;
		case HREG_M1_DIR:
			break;
		case HREG_M2_DIR:
			break;
		case HREG_M1_SPEED:
			break;
		case HREG_M2_SPEED:
			break;
		default:
			numUs = 0;
			break;
	}

	return numUs;
}

uint8_t validateWriteHoldingRegs(int iRegIndex, uint16_t usNRegs)
{
	for (size_t i = 0; i < usNRegs; ++i) {
		switch (iRegIndex + i) {
			case HREG_M1_RUN:
				break;
			case HREG_M2_RUN:
				break;
			case HREG_M1_DIR:
				break;
			case HREG_M2_DIR:
				break;
			case HREG_M1_SPEED:
				break;
			case HREG_M2_SPEED:
				break;
			default:
				if (i == 0) return 0;
				break;
		}
	}

	return 1;
}

void writeHoldingRegs(int iRegIndex, uint16_t tempReg)
{
	switch (iRegIndex) {
		case HREG_M1_RUN:
			usRegHoldingBuf[HREG_M1_RUN] = tempReg;
			if(usRegHoldingBuf[HREG_M1_RUN] == 1)
			{
				dcmotor_Start(M1, usRegHoldingBuf[HREG_M1_DIR], usRegHoldingBuf[HREG_M1_SPEED]);
			}
			else
			{
				dcmotor_Stop(M1);
			}
			break;
		case HREG_M1_DIR:
			usRegHoldingBuf[HREG_M1_DIR] = tempReg;
			dcmotor_setDirection(M1, tempReg);
			break;
		case HREG_M1_SPEED:
			usRegHoldingBuf[HREG_M1_SPEED] = tempReg;
			dcmotor_setSpeed(M1, tempReg);
			break;
		default:
			__32bitsRegBuf.us[0] = tempReg;
			break;
	}
}

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
	eMBErrorCode    eStatus = MB_ENOERR;
	int             iRegIndex;

	if( ( usAddress >= REG_INPUT_START )
		&& ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
	{
		iRegIndex = ( int )( usAddress - usRegInputStart );
		while( usNRegs > 0 )
		{
			uint8_t numUs = fetchInputRegsData(iRegIndex);
			if (numUs < 1)
			{
				return MB_ENORES;
			}
			for (size_t i = 0; i < numUs; ++i) {
				if (usNRegs > 0)
				{
					*pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
					*pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
					iRegIndex++;
					usNRegs--;
				}
				else
				{
					return MB_ENORES;
				}
			}
		}
	}
	else
	{
		eStatus = MB_ENOREG;
	}

	return eStatus;
}

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                 eMBRegisterMode eMode )
{
	eMBErrorCode    eStatus = MB_ENOERR;
	int             iRegIndex;

	if (eMode == MB_REG_READ)
	{
		if( ( usAddress >= REG_HOLDING_START )
			&& ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
		{
			iRegIndex = ( int )( usAddress - usRegHoldingStart );
			while( usNRegs > 0 )
			{
				uint8_t numUs = fetchHoldingRegsData(iRegIndex);
				if (numUs < 1)
				{
					return MB_ENORES;
				}
				for (size_t i = 0; i < numUs; ++i) {
					if (usNRegs > 0)
					{
						*pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
						*pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
						iRegIndex++;
						usNRegs--;
					}
					else
					{
						return MB_ENORES;
					}
				}
			}
		}
		else
		{
			eStatus = MB_ENOREG;
		}
	}

	if (eMode == MB_REG_WRITE)
	{
		if( ( usAddress >= REG_HOLDING_START )
			&& ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
		{
			iRegIndex = ( int )( usAddress - usRegHoldingStart );
			if (validateWriteHoldingRegs(iRegIndex, usNRegs) == 0)
			{
//				printf("Bad Request!\r\n");
				return MB_EINVAL;
			}
			while( usNRegs > 0 )
			{
				writeHoldingRegs(iRegIndex, (USHORT) ( ((unsigned int) *pucRegBuffer << 8) | ((unsigned int) *(pucRegBuffer+1)) ));
				pucRegBuffer+=2;
				usNRegs--;
				iRegIndex++;
			}
		}
		else
		{
			eStatus = MB_ENOREG;
		}
	}

	return eStatus;
}


eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
               eMBRegisterMode eMode )
{
	return MB_ENOREG;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
	return MB_ENOREG;
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

