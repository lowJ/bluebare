/*
 * nav.h
 *
 *  Created on: Apr 12, 2023
 *      Author: jonathanlow
 */

#ifndef INC_NAV_H_
#define INC_NAV_H_

#include <stdbool.h>
#include <stdint.h>

#include "blue_hal.h"

#define LEFT_TURN_90_CNTS 175
#define LEFT_TURN_180_CNTS 340

bool nav_init();
bool is_left_wall_detected();
bool is_right_wall_detected();
void straight(uint16_t cells);
void straight_till_wall();
uint16_t cnt_per_ms(bh_motor_t motor, bh_motor_dir_t dir);
void turn_left(uint16_t cnts);

#endif /* INC_NAV_H_ */