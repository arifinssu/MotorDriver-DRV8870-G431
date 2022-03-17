/*
 * dcmotor.c
 *
 *  Created on: Mar 16, 2022
 *      Author: User
 */

#include "main.h"
#include "dcmotor.h"
#include "dcmotor_config.h"

int motor_dir = 0;
int motor_speed = 0;

void dcmotor_Init(uint8_t au8_MOTOR_Instance)
{
	HAL_TIM_PWM_Start_IT(DC_MOTOR_CfgParam[au8_MOTOR_Instance].TIM_Instance, DC_MOTOR_CfgParam[au8_MOTOR_Instance].EN1_TIM_CH);
	HAL_TIM_PWM_Start_IT(DC_MOTOR_CfgParam[au8_MOTOR_Instance].TIM_Instance, DC_MOTOR_CfgParam[au8_MOTOR_Instance].EN2_TIM_CH);
}

void dcmotor_Start(uint8_t au8_MOTOR_Instance, uint8_t au8_DIR, uint16_t au16_SPEED)
{
	if(au8_DIR == 0)
	{
		__HAL_TIM_SET_COMPARE(DC_MOTOR_CfgParam[au8_MOTOR_Instance].TIM_Instance, DC_MOTOR_CfgParam[au8_MOTOR_Instance].EN1_TIM_CH, au16_SPEED);
		__HAL_TIM_SET_COMPARE(DC_MOTOR_CfgParam[au8_MOTOR_Instance].TIM_Instance, DC_MOTOR_CfgParam[au8_MOTOR_Instance].EN2_TIM_CH, 0);
	}
	if(au8_DIR == 1)
	{
		__HAL_TIM_SET_COMPARE(DC_MOTOR_CfgParam[au8_MOTOR_Instance].TIM_Instance, DC_MOTOR_CfgParam[au8_MOTOR_Instance].EN1_TIM_CH, 0);
		__HAL_TIM_SET_COMPARE(DC_MOTOR_CfgParam[au8_MOTOR_Instance].TIM_Instance, DC_MOTOR_CfgParam[au8_MOTOR_Instance].EN2_TIM_CH, au16_SPEED);
	}

	motor_dir = au8_DIR;
}

void dcmotor_setSpeed(uint8_t au8_MOTOR_Instance, uint16_t au16_SPEED)
{
	motor_speed = au16_SPEED;
	dcmotor_Start(au8_MOTOR_Instance, motor_dir, motor_speed);
}

void dcmotor_setDirection(uint8_t au8_MOTOR_Instance, uint8_t au8_DIR)
{
	motor_dir = au8_DIR;
	dcmotor_Start(au8_MOTOR_Instance, motor_dir, motor_speed);
}

void dcmotor_Stop(uint8_t au8_MOTOR_Instance)
{
	__HAL_TIM_SET_COMPARE(DC_MOTOR_CfgParam[au8_MOTOR_Instance].TIM_Instance, DC_MOTOR_CfgParam[au8_MOTOR_Instance].EN1_TIM_CH, 0);
	__HAL_TIM_SET_COMPARE(DC_MOTOR_CfgParam[au8_MOTOR_Instance].TIM_Instance, DC_MOTOR_CfgParam[au8_MOTOR_Instance].EN2_TIM_CH, 0);
}
