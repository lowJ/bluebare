/*
 * blue_hal.c
 *
 *  Created on: Jan 18, 2023
 *      Author: jonathanlow
 */


#include "blue_hal.h"

bool bh_init() {
    /* TODO: */
    return false;
}
uint16_t bh_get_distance(bh_dist_t dist) {
    return 0;
}

bool bh_set_motor_pwm(bh_motor_t motor, bh_motor_dir_t dir) {
    return false;
}

bool bh_uart_tx(uint8_t* buf, uint16_t num_bytes) {
    return false;
}

bool bh_ble_tx(uint8_t *buf, uint16_t num_bytes) {
    return false;
}

bool bh_set_led(bh_led_t led, bool state) {
    return false;
}
bool bh_set_buzzer(uint16_t tone, bool state) {
    return false;
}
