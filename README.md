# MotorDriver-DRV8870-G431
> Driver motor dc (DRV8870) controlled with STM32G431

Controlled dual-pwm motor dc with enabled DMA interrupt + TIM OC

### Configures
- HSE: 8MHz, Clock Frequency: 180MHz
- TIM1 CH1 OC No Output DMA, PSC: 0, ARR: 100, Circular, DIR: Memory to Peripheral, Global Interrupt
- TIM4 CH1 & CH2 PWM, PSC: 170-1, ARR: (Period of MotorDC Speed), Global Interrupt
- TIM7 Modbus Clock, PSC: 170-1, ARR: 50-1, Global Interrupt

### DCMOTOR Instance Header Configures
```c
DC_MOTOR_CfgParam {
	TIM_HandleTypeDef*  TIM_Instance;
	uint32_t            EN1_TIM_CH;
	uint32_t            EN2_TIM_CH;
	TIM_HandleTypeDef*  OC_Instance;
	uint32_t            OC_TIM_CH;
	DMA_HandleTypeDef*  DMA_Instance;
	uint32_t            DMA_CC;
	uint32_t            CC1_DEST;
	uint32_t            CC2_DEST;
	GPIO_TypeDef*       LIMIT_GPIO;
	uint16_t            LIMIT_PIN;
	GPIO_TypeDef*       LED_GPIO;
	uint16_t            LED_PIN;
} DC_MOTOR_CfgType;
```

### DCMOTOR Prototypes Functions
```c
void dcmotor_Init(uint8_t au8_MOTOR_Instance);
void dcmotor_Start(uint8_t au8_MOTOR_Instance);
void dcmotor_setDirection(uint8_t au8_MOTOR_Instance, uint8_t au8_DIR, uint32_t pwm_data);
void dcmotor_Stop(uint8_t au8_MOTOR_Instance);
```
