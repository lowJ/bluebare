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

bool nav_init();
bool is_left_wall_detected();
bool is_right_wall_detected();
void straight();

#endif /* INC_NAV_H_ */