/*
 * blue_hal.h
 *
 *  Created on: Jan 20, 2023
 *      Author: jonathanlow
 */

#ifndef INC_BLUE_HAL_H_
#define INC_BLUE_HAL_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>

typedef enum {
	MOTOR_LEFT,
	MOTOR_RIGHT
} MOTOR_TYPE;

typedef enum {
	DIR_FORWARD,
	DIR_BACKWARD,
	DIR_STOP_HARD,
	DIR_STOP_SOFT
} MOTOR_DIR_TYPE;

typedef enum {
	DIST_L,
	DIST_FL,
	DIST_R,
	DIST_FR
} IR_TYPE;

typedef enum {
	ROT_CLOCKWISE,
	ROT_COUNTER_CLOCKWISE
} MOTOR_ROT_TYPE;

typedef enum {
	LED_RED,
	LED_GREEN,
	LED_BLUE
} bh_led_t;

bool Init(ADC_HandleTypeDef* adc1, TIM_HandleTypeDef* tim2, UART_HandleTypeDef* uart_handle, TIM_HandleTypeDef* tim3, TIM_HandleTypeDef* tim4);

bool Set_LED(bh_led_t led, bool state);

bool Set_Motor_Dir(MOTOR_TYPE motor, MOTOR_DIR_TYPE dir);
bool Set_Motor_PWM(MOTOR_TYPE motor, uint16_t dc);

bool Spin_Motor_By_Enc_Ticks(MOTOR_TYPE motor, uint16_t enc_ticks);
bool Straight_Line_Encoder_Test(uint16_t encTicks, uint16_t targetSpeed);
bool Rotate_Mouse_By_Enc_Ticks(MOTOR_ROT_TYPE dir, uint16_t encTicks, uint16_t speed);

void Move_One_Cell(uint16_t encTicks, uint16_t targetSpeed, uint16_t irLeftOffset, uint16_t irRightOffset);

bool Reset_Enc_Count(MOTOR_TYPE motor);
bool Reset_Enc_Count_To_Max(MOTOR_TYPE motor);

uint16_t Get_Enc_Count(MOTOR_TYPE motor);
uint16_t Measure_IR_Dist(IR_TYPE ir);
uint16_t Measure_Avg_IR_Dist(IR_TYPE ir);

bool bh_uart_tx_str(uint8_t* buf);
bool bh_ble_tx(uint8_t *buf, uint16_t num_bytes);
bool bh_set_buzzer(uint16_t tone, bool state); /* TODO: Verify params */

#endif /* INC_BLUE_HAL_H_ */
