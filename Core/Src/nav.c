/*
 * nav.c
 *
 *  Created on: Apr 12, 2023
 *      Author: jonathanlow
 */

#include "nav.h"
#include "blue_hal.h"

#define WALL_DETECTION_PADDING 200 /* TODO: Adjust this value*/

uint16_t left_wall_dist_thresh = 0;
uint16_t right_wall_dist_thresh = 0;

bool nav_init() {
    left_wall_dist_thresh = bh_measure_dist_avg(DIST_L);
    right_wall_dist_thresh = bh_measure_dist_avg(DIST_R);
    return true;
}

bool is_left_wall_detected(){
    uint16_t m = bh_measure_dist_avg(DIST_L);
    if(m > left_wall_dist_thresh - WALL_DETECTION_PADDING) {
       bh_set_led(LED_BLUE, 1);
       return true;
    } else {
        bh_set_led(LED_BLUE, 0);
        return false;
    }
}

bool is_right_wall_detected(){
    uint16_t m = bh_measure_dist_avg(DIST_R);
    if(m > right_wall_dist_thresh - WALL_DETECTION_PADDING) {
        bh_set_led(LED_RED, 1);
        return true;
    } else {
        bh_set_led(LED_RED,0);
        return false;
    }
}