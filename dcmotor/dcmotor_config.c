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
		&htim1,
		TIM_CHANNEL_1,
		&hdma_tim1_ch1,
		TIM_DMA_CC1,
		(uint32_t) &(TIM4->CCR1),
		(uint32_t) &(TIM4->CCR2),
		M1_LIMIT_GPIO_Port,
		M1_LIMIT_Pin,
		LED_ST1_GPIO_Port,
		LED_ST1_Pin
	}
	// // DC MOTOR 2 Configurations
    // {
	//     &htim4,
	// 	TIM_CHANNEL_3,
	// 	TIM_CHANNEL_4,
	// 	&htim1,
	// 	TIM_CHANNEL_1,
	// 	&hdma_tim1_ch1,
	// 	TIM_DMA_CC2,
	// 	(uint32_t) &(TIM4->CCR3),
	// 	(uint32_t) &(TIM4->CCR4),
	// 	M1_LIMIT_GPIO_Port,
	// 	M1_LIMIT_Pin,
	// 	LED_ST2_GPIO_Port,
	// 	LED_ST2_Pin
	// }
};
