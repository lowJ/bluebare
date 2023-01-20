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
	DIST_RL
} bh_dist_t;

typedef enum {
	LED_RED,
	LED_GREEN,
	LED_BLUE
} bh_led_t;

bool bh_init();
uint16_t bh_get_distance(bh_dist_t dist);
bool bh_set_motor_pwm(bh_motor_t motor, bh_motor_dir_t dir);
bool bh_uart_tx(uint8_t* buf, uint16_t num_bytes);
bool bh_ble_tx(uint8_t *buf, uint16_t num_bytes);
bool bh_set_led(bh_led_t led, bool state);
bool bh_set_buzzer(uint16_t tone, bool state); /* TODO: Verify params */

#endif /* INC_BLUE_HAL_H_ */
