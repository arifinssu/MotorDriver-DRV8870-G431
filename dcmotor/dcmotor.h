/*
 * dcmotor.h
 *
 *  Created on: Mar 16, 2022
 *      Author: User
 */

#ifndef DCMOTOR_H_
#define DCMOTOR_H_

#define HAL_TIM_MODULE_ENABLED

#include "stm32g4xx_hal.h"

// DC Motor Rotation Directions
#define DIR_CW    0
#define DIR_CCW   1

// DC Motor PWM Properties
#define DC_MOTOR_PWM_RES  10
#define DC_MOTOR_F_PWM    500

// The Number OF DC MOTORs To Be Used In The Project
#define DC_MOTOR_UNITS  2

typedef struct
{
	TIM_HandleTypeDef* TIM_Instance;
	uint32_t EN1_TIM_CH;
	uint32_t EN2_TIM_CH;
	GPIO_TypeDef* LIMIT_GPIO;
	uint16_t LIMIT_PIN;
	GPIO_TypeDef* LED_GPIO;
	uint16_t LED_PIN;
}DC_MOTOR_CfgType;

/*-----[ Prototypes For All Functions ]-----*/
void dcmotor_Init(uint8_t au8_MOTOR_Instance);
void dcmotor_Start(uint8_t au8_MOTOR_Instance, uint8_t au8_DIR, uint16_t au16_SPEED);
void dcmotor_setSpeed(uint8_t au8_MOTOR_Instance, uint16_t au16_SPEED);
void dcmotor_setDirection(uint8_t au8_MOTOR_Instance, uint8_t au8_DIR);
void dcmotor_Stop(uint8_t au8_MOTOR_Instance);

#endif /* DCMOTOR_H_ */
