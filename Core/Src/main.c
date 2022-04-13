/*USER CODE BEGIN Header */
/**
 ******************************************************************************
 *@file           : main.c
 *@brief          : Main program body
 ******************************************************************************
 *@attention
 *
 *Copyright (c) 2022 STMicroelectronics.
 *All rights reserved.
 *
 *This software is licensed under terms that can be found in the LICENSE file
 *in the root directory of this software component.
 *If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/*USER CODE END Header */
/*Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/*Private includes ----------------------------------------------------------*/
/*USER CODE BEGIN Includes */
#include "../../modbus/mb.h"
#include "../../modbus/mbport.h"
#include "../../dcmotor/dcmotor.h"
#include "../../w25qxx/w25qxx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/*USER CODE END Includes */

/*Private define ------------------------------------------------------------*/
/*USER CODE BEGIN PD */
// modbus config
#define SECTOR_MODBUS_ADDRESS 1
#define REG_HOLDING_START 1
#define REG_HOLDING_NREGS 22
static USHORT usRegHoldingStart = REG_HOLDING_START;
static USHORT usRegHoldingBuf[REG_HOLDING_NREGS];
enum
{
    HREG_M1_RUN,
    HREG_M2_RUN,
    HREG_M1_DIR,
    HREG_M2_DIR,
    HREG_M1_SPEED,
    HREG_M2_SPEED,
    HREG_NEW_MODBUS_ADDRESS
} holdingRegs_t;

// define motor instance
#define M1 0
#define M2 1
uint8_t pwm_data[1];
/*USER CODE END PD */

/*Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/*USER CODE BEGIN PFP */
int getDeviceAddress();
void writeDeviceAddress(char new_address[]);
uint8_t fetchInputRegsData(int iRegIndex);
uint8_t fetchHoldingRegsData(int iRegIndex);
uint8_t validateWriteHoldingRegs(int iRegIndex, uint16_t usNRegs);
void writeHoldingRegs(int iRegIndex, uint16_t tempReg);
/*USER CODE END PFP */

/*Private user code ---------------------------------------------------------*/
/*USER CODE BEGIN 0 */
/*USER CODE END 0 */

/**
 *@brief  The application entry point.
 *@retval int
 */
int main(void)
{
    /*USER CODE BEGIN 1 */
    /*USER CODE END 1 */

    /*MCU Configuration--------------------------------------------------------*/

    /*Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /*USER CODE BEGIN Init */
    /*USER CODE END Init */

    /*Configure the system clock */
    SystemClock_Config();

    /*USER CODE BEGIN SysInit */
    /*USER CODE END SysInit */

    /*Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_I2C2_Init();
    MX_USART2_UART_Init();
    MX_TIM4_Init();
    MX_DMA_Init();
    MX_TIM7_Init();
    MX_TIM1_Init();
    /*USER CODE BEGIN 2 */
   	// set 0 holding & input regs
    for (size_t i = 0; i < REG_HOLDING_NREGS; i++) usRegHoldingBuf[i] = 0;

    W25qxx_Init();
    dcmotor_Init(M1);

    eMBInit(MB_RTU, getDeviceAddress(), 3, 9600, MB_PAR_NONE);
    eMBEnable();
    /*USER CODE END 2 */

    /*Infinite loop */
    /*USER CODE BEGIN WHILE */
    while (1)
    {
        eMBPoll();
        /*USER CODE END WHILE */
        /*USER CODE BEGIN 3 */
    }
    /*USER CODE END 3 */
}

/**
 *@brief System Clock Configuration
 *@retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /**Configure the main internal regulator output voltage
     */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    /**Initializes the RCC Oscillators according to the specified parameters
     *in the RCC_OscInitTypeDef structure.
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

    /**Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/*USER CODE BEGIN 4 */
/**
 * @brief Get the Modbus Device Address object. 
 * If not present, default address = 80.
 * 
 * @return int
 */
int getDeviceAddress()
{
    uint8_t mb_default_address[] = "80";
    uint8_t sector_cont[sizeof(mb_default_address)-1];
    if (W25qxx_IsEmptySector(SECTOR_MODBUS_ADDRESS, sizeof(mb_default_address), sizeof(mb_default_address)-1) == false)
        W25qxx_ReadSector(sector_cont, SECTOR_MODBUS_ADDRESS, sizeof(mb_default_address), sizeof(mb_default_address)-1);

    else memcpy(sector_cont, mb_default_address, sizeof(mb_default_address)-1);

    uint8_t v[sizeof(sector_cont)];
    memcpy(v, sector_cont, sizeof(sector_cont));
    return atoi(v);
}

/**
 * @brief Write the Modbus Device Address object.
 * 
 * @param new_address 
 */
void writeDeviceAddress(char new_address[])
{
    // char s[strlen(new_address)];
    // memcpy(s, new_address, strlen(new_address));
    // strcpy(s, new_address);
    W25qxx_EraseSector(SECTOR_MODBUS_ADDRESS);
    W25qxx_WriteSector(new_address, SECTOR_MODBUS_ADDRESS, SECTOR_MODBUS_ADDRESS+strlen(new_address), strlen(new_address));

    HAL_Delay(50);
    Error_Handler();
}

/**
 *@brief Fetching modbus input register data.
 * 
 *@param iRegIndex
 *@return uint8_t 
 */
uint8_t fetchInputRegsData(int iRegIndex)
{
    uint8_t numUs = 1;
    return numUs;
}

/**
 *@brief Fetching modbus holding register data.
 * 
 *@param iRegIndex 
 *@return uint8_t 
 */
uint8_t fetchHoldingRegsData(int iRegIndex)
{
    uint8_t numUs = 1;

    switch (iRegIndex)
    {
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
        case HREG_NEW_MODBUS_ADDRESS:
            break;
        default:
            numUs = 0;
            break;
    }

    return numUs;
}

/**
 *@brief Validate modbus writing holding register data.
 * 
 *@param iRegIndex 
 *@param usNRegs 
 *@return result
 */
uint8_t validateWriteHoldingRegs(int iRegIndex, uint16_t usNRegs)
{
    uint8_t result = 1;

    for (size_t i = 0; i < usNRegs; ++i)
    {
        switch (iRegIndex + i)
        {
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
            case HREG_NEW_MODBUS_ADDRESS:
                break;
            default:
                if (i == 0) return 0;
                break;
        }
    }

    return result;
}

/**
 *@brief Writing modbus holding registers.
 * 
 *@param iRegIndex 
 *@param tempReg
 */
void writeHoldingRegs(int iRegIndex, uint16_t tempReg)
{
    switch (iRegIndex)
    {
        case HREG_M1_RUN:
            if (tempReg > 1) break;
            usRegHoldingBuf[HREG_M1_RUN] = tempReg;
            dcmotor_Stop(M1);
            if (usRegHoldingBuf[HREG_M1_RUN] == 1)
            {
                dcmotor_Start(M1);
                dcmotor_setDirection(M1, usRegHoldingBuf[HREG_M1_DIR], (uint32_t)pwm_data);
            }
            break;
        case HREG_M1_DIR:
            usRegHoldingBuf[HREG_M1_DIR] = tempReg;
            dcmotor_setDirection(M1, usRegHoldingBuf[HREG_M1_DIR], (uint32_t)pwm_data);
            break;
        case HREG_M1_SPEED:
            usRegHoldingBuf[HREG_M1_SPEED] = tempReg;
            pwm_data[0] = usRegHoldingBuf[HREG_M1_SPEED];
            break;
        case HREG_NEW_MODBUS_ADDRESS:
            if (tempReg < 10 || tempReg > 99) break;
            usRegHoldingBuf[HREG_NEW_MODBUS_ADDRESS] = tempReg;
            char value[3];
            writeDeviceAddress(itoa(usRegHoldingBuf[HREG_NEW_MODBUS_ADDRESS], value, 10));
            break;
        default:
            // usRegHoldingBuf[iRegIndex] = tempReg;
            break;
    }
}

eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{
    eMBErrorCode eStatus = MB_ENOERR;
    int iRegIndex;

    if (eMode == MB_REG_READ)
    {
        if ((usAddress >= REG_HOLDING_START) &&
            (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS))
        {
            iRegIndex = (int)(usAddress - usRegHoldingStart);
            while (usNRegs > 0)
            {
                uint8_t numUs = fetchHoldingRegsData(iRegIndex);
                if (numUs < 1)
                {
                    return MB_ENORES;
                }

                for (size_t i = 0; i < numUs; ++i)
                {
                    if (usNRegs > 0)
                    {
                        *pucRegBuffer++ = (unsigned char)(usRegHoldingBuf[iRegIndex] >> 8);
                        *pucRegBuffer++ = (unsigned char)(usRegHoldingBuf[iRegIndex] &0xFF);
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
        if ((usAddress >= REG_HOLDING_START) && (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS))
        {
            iRegIndex = (int)(usAddress - usRegHoldingStart);
            if (validateWriteHoldingRegs(iRegIndex, usNRegs) == 0)
                return MB_EINVAL; // bad request

            while (usNRegs > 0)
            {
                writeHoldingRegs(iRegIndex, (USHORT)(((unsigned int) *pucRegBuffer << 8) | ((unsigned int) *(pucRegBuffer + 1))));
                pucRegBuffer += 2;
                usNRegs--;
                iRegIndex++;
            }
        }
        else eStatus = MB_ENOREG;
    }

    return eStatus;
}

eMBErrorCode eMBRegInputCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
    return MB_ENOREG;
}

eMBErrorCode eMBRegCoilsCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode)
{
    return MB_ENOREG;
}

eMBErrorCode eMBRegDiscreteCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
{
    return MB_ENOREG;
}
/*USER CODE END 4 */

/**
 *@brief  This function is executed in case of error occurrence.
 *@retval None
 */
void Error_Handler(void)
{
    /*USER CODE BEGIN Error_Handler_Debug */
    /*User can add his own implementation to report the HAL error return state */
    __disable_irq();
    HAL_NVIC_SystemReset();
    while (1) {}

    /*USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 *@brief  Reports the name of the source file and the source line number
 *        where the assert_param error has occurred.
 *@param  file: pointer to the source file name
 *@param  line: assert_param error line source number
 *@retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /*USER CODE BEGIN 6 */
    /*User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /*USER CODE END 6 */
}
#endif /*USE_FULL_ASSERT */
