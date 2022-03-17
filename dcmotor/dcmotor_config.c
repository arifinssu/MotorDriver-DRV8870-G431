/*
 * dcmotor_config.c
 *
 *  Created on: Mar 16, 2022
 *      Author: User
 */

#include "main.h"
#include "dcmotor.h"
#include "tim.h"

const DC_MOTOR_CfgType DC_MOTOR_CfgParam[DC_MOTOR_UNITS] =
{
	// DC MOTOR 1 Configurations
    {
	    &htim4,
		TIM_CHANNEL_1,
		TIM_CHANNEL_2,
		M1_LIMIT_GPIO_Port,
		M1_LIMIT_Pin,
		LED_ST1_GPIO_Port,
		LED_ST1_Pin
	}
};
