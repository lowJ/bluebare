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

typedef enum {
	MOTOR_LEFT,
	MOTOR_RIGHT
} bh_motor_t;

typedef enum {
	DIR_FORWARD,
	DIR_BACKWARD,
	DIR_STOP_HARD,
	DIR_STOP_SOFT
} bh_motor_dir_t;

typedef enum {
	DIST_L,
	DIST_FL,
	DIST_R,
	DIST_FR
} bh_dist_t;

typedef enum {
	LED_RED,
	LED_GREEN,
	LED_BLUE
} bh_led_t;
bool bh_init(ADC_HandleTypeDef* adc1, TIM_HandleTypeDef* tim2, UART_HandleTypeDef* uart_handle, TIM_HandleTypeDef* tim3, TIM_HandleTypeDef* tim4, ADC_HandleTypeDef* adc2);
uint16_t bh_measure_dist(bh_dist_t dist);
bool bh_set_motor_dir(bh_motor_t motor, bh_motor_dir_t dir);
bool bh_set_motor_pwm(bh_motor_t motor, uint16_t dc);
bool bh_uart_tx_str(uint8_t* buf);
bool bh_ble_tx(uint8_t *buf, uint16_t num_bytes);
bool bh_set_led(bh_led_t led, bool state);
bool bh_set_buzzer(uint16_t tone, bool state); /* TODO: Verify params */
uint16_t bh_get_enc_cnt(bh_motor_t motor);
bool bh_reset_enc_cnt(bh_motor_t motor);
uint16_t bh_measure_dist_avg(bh_dist_t dist_sensor);
uint16_t bh_measure_gyro_outz();
uint16_t bh_measure_gyro_vref();
#endif /* INC_BLUE_HAL_H_ */
