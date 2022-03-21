/*
 * dcmotor.c
 *
 *  Created on: Mar 16, 2022
 *      Author: User
 */

#include "main.h"
#include "dcmotor.h"
#include "dcmotor_config.h"

void dcmotor_Init(uint8_t au8_MOTOR_Instance)
{
	HAL_TIM_PWM_Start(DC_MOTOR_CfgParam[au8_MOTOR_Instance].TIM_Instance, DC_MOTOR_CfgParam[au8_MOTOR_Instance].EN1_TIM_CH);
	HAL_TIM_PWM_Start(DC_MOTOR_CfgParam[au8_MOTOR_Instance].TIM_Instance, DC_MOTOR_CfgParam[au8_MOTOR_Instance].EN2_TIM_CH);
	HAL_TIM_OC_Start(DC_MOTOR_CfgParam[au8_MOTOR_Instance].OC_Instance, DC_MOTOR_CfgParam[au8_MOTOR_Instance].OC_TIM_CH);
	__HAL_TIM_ENABLE_DMA(DC_MOTOR_CfgParam[au8_MOTOR_Instance].OC_Instance, DC_MOTOR_CfgParam[au8_MOTOR_Instance].DMA_CC);
}

void dcmotor_Start(uint8_t au8_MOTOR_Instance)
{
	__HAL_TIM_ENABLE_DMA(DC_MOTOR_CfgParam[au8_MOTOR_Instance].OC_Instance, DC_MOTOR_CfgParam[au8_MOTOR_Instance].DMA_CC);
}

void dcmotor_setDirection(uint8_t au8_MOTOR_Instance, uint8_t au8_DIR, uint32_t pwm_data)
{
	HAL_DMA_Abort_IT(DC_MOTOR_CfgParam[au8_MOTOR_Instance].DMA_Instance);

	// direction cw
	if(au8_DIR == 0)
	{
		__HAL_TIM_SET_COMPARE(DC_MOTOR_CfgParam[au8_MOTOR_Instance].TIM_Instance, DC_MOTOR_CfgParam[au8_MOTOR_Instance].EN2_TIM_CH, 0);
		HAL_DMA_Start_IT(DC_MOTOR_CfgParam[au8_MOTOR_Instance].DMA_Instance, (uint32_t)pwm_data, DC_MOTOR_CfgParam[au8_MOTOR_Instance].CC1_DEST, 1);
	}

	// direction ccw
	else if(au8_DIR == 1)
	{
		__HAL_TIM_SET_COMPARE(DC_MOTOR_CfgParam[au8_MOTOR_Instance].TIM_Instance, DC_MOTOR_CfgParam[au8_MOTOR_Instance].EN1_TIM_CH, 0);
		HAL_DMA_Start_IT(DC_MOTOR_CfgParam[au8_MOTOR_Instance].DMA_Instance, (uint32_t)pwm_data, DC_MOTOR_CfgParam[au8_MOTOR_Instance].CC2_DEST, 1);
	}
}

void dcmotor_Stop(uint8_t au8_MOTOR_Instance)
{
	HAL_DMA_Abort_IT(DC_MOTOR_CfgParam[au8_MOTOR_Instance].DMA_Instance);
	__HAL_TIM_DISABLE_DMA(DC_MOTOR_CfgParam[au8_MOTOR_Instance].OC_Instance, DC_MOTOR_CfgParam[au8_MOTOR_Instance].DMA_CC);
	__HAL_TIM_SET_COMPARE(DC_MOTOR_CfgParam[au8_MOTOR_Instance].TIM_Instance, DC_MOTOR_CfgParam[au8_MOTOR_Instance].EN1_TIM_CH, 0);
	__HAL_TIM_SET_COMPARE(DC_MOTOR_CfgParam[au8_MOTOR_Instance].TIM_Instance, DC_MOTOR_CfgParam[au8_MOTOR_Instance].EN2_TIM_CH, 0);
}
