/*
 * nav.c
 *
 *  Created on: Apr 12, 2023
 *      Author: jonathanlow
 */

#include "nav.h"
#include "blue_hal.h"
#include <stdlib.h>

#define WALL_DETECTION_PADDING 200 /* TODO: Adjust this value*/

uint16_t left_wall_middle = 0;
uint16_t right_wall_middle = 0;
static uint16_t cnt_per_ms_lt(uint16_t* spd_l, uint16_t* spd_r);

bool nav_init() {
    left_wall_middle = bh_measure_dist_avg(DIST_L);
    left_wall_middle = bh_measure_dist_avg(DIST_R);
    return true;
}

bool is_left_wall_detected(){
    uint16_t m = bh_measure_dist_avg(DIST_L);
    if(m > left_wall_middle - WALL_DETECTION_PADDING) {
       bh_set_led(LED_BLUE, 1);
       return true;
    } else {
        bh_set_led(LED_BLUE, 0);
        return false;
    }
}

bool is_right_wall_detected(){
    uint16_t m = bh_measure_dist_avg(DIST_R);
    if(m > left_wall_middle - WALL_DETECTION_PADDING) {
        bh_set_led(LED_RED, 1);
        return true;
    } else {
        bh_set_led(LED_RED,0);
        return false;
    }
}

#define LEFT_BASE_SPD 1450
#define RIGHT_BASE_SPD 1400
#define g_P 0.2
#define g_D 0
#define TICKS_PER_CELL 679
void straight(uint16_t cells){
    int32_t left_right_offset =  left_wall_middle - right_wall_middle;
    int32_t p_error = 0;
    int32_t d_error = 0;
    int32_t p_error_old;
    int32_t total_error;

    bh_set_motor_dir(MOTOR_LEFT, DIR_FORWARD);
    bh_set_motor_dir(MOTOR_RIGHT, DIR_FORWARD);

    uint16_t goal_cnt = 65535 - (cells * 680); 

     
    bh_reset_enc_cnt(MOTOR_LEFT);
    bh_reset_enc_cnt(MOTOR_RIGHT);

    
    //while(1){
    while(bh_get_enc_cnt(MOTOR_LEFT) > goal_cnt || bh_get_enc_cnt(MOTOR_LEFT) == 0) {
        if(is_left_wall_detected() && is_right_wall_detected()) {
            bool lr_diff_is_negative = false;
            int32_t lr_diff = bh_measure_dist_avg(DIST_L) - bh_measure_dist_avg(DIST_R);
            if(lr_diff < 0) lr_diff_is_negative = true;

            p_error = abs(lr_diff) - abs(left_right_offset);
            if(lr_diff_is_negative) p_error = p_error * -1;
            d_error = p_error - p_error_old;
        } else if (is_left_wall_detected()) {

        } else if (is_right_wall_detected()) {

        } else {

        }


        total_error = (g_P * p_error) + (g_D * d_error);
        p_error_old = p_error;
        //Positive error = veer right, negative error = veer left

        bh_set_motor_pwm(MOTOR_LEFT, LEFT_BASE_SPD + total_error);
        bh_set_motor_pwm(MOTOR_RIGHT, RIGHT_BASE_SPD - total_error);
        HAL_Delay(2);

    }
    bh_set_motor_pwm(MOTOR_LEFT, 0);
    bh_set_motor_pwm(MOTOR_RIGHT, 0);



}



#define LEFT_TURN_L_BASE_SPD 1400
#define LEFT_TURN_R_BASE_SPD 1450
#define LEFT_TURN_CNT 190
#define g_P_lt 0.2
#define g_D_lt 0
void turn_left(){
    int32_t p_error = 0;
    int32_t d_error = 0;
    int32_t p_error_old;
    int32_t total_error;

    bh_set_motor_dir(MOTOR_LEFT, DIR_BACKWARD);
    bh_set_motor_dir(MOTOR_RIGHT, DIR_FORWARD);

    uint16_t l_goal_cnt = LEFT_TURN_CNT; 
    uint16_t r_goal_cnt = 65535 - LEFT_TURN_CNT; 

    bh_reset_enc_cnt(MOTOR_LEFT);
    bh_reset_enc_cnt(MOTOR_RIGHT);

    uint16_t l_spd;
    uint16_t r_spd;
    int16_t lr_diff;
    int16_t error;
    while(bh_get_enc_cnt(MOTOR_LEFT) < l_goal_cnt || bh_get_enc_cnt(MOTOR_LEFT) == 0) {

        cnt_per_ms_lt(&l_spd, &r_spd); //12MS delay here

        lr_diff = l_spd - r_spd;

        error = lr_diff * g_P_lt;

        bh_set_motor_pwm(MOTOR_LEFT, LEFT_TURN_L_BASE_SPD + error);
        bh_set_motor_pwm(MOTOR_RIGHT, LEFT_TURN_R_BASE_SPD - error);
    }
    bh_set_motor_pwm(MOTOR_LEFT, 0);
    bh_set_motor_pwm(MOTOR_RIGHT, 0);
}

uint16_t cnt_per_ms_lt(uint16_t* spd_l, uint16_t* spd_r){
    uint16_t start_cnt_l = bh_get_enc_cnt(MOTOR_LEFT);
    uint16_t start_cnt_r = bh_get_enc_cnt(MOTOR_RIGHT);
    HAL_Delay(12);
    uint16_t end_cnt_l = bh_get_enc_cnt(MOTOR_LEFT);
    uint16_t end_cnt_r = bh_get_enc_cnt(MOTOR_RIGHT);
    //Left is backwards
    uint16_t l_cnts;
    if(end_cnt_l < start_cnt_l) { //Handle overflow case
        l_cnts = ((65535-start_cnt_l)+1) + end_cnt_l;
    } else {
        l_cnts = end_cnt_l - start_cnt_l;
    }
    *spd_l = l_cnts;
    //Right is forwards
    uint16_t r_cnts;
    if(end_cnt_r > start_cnt_r) { //Handle overflow case
        r_cnts = ((65535-end_cnt_r)+1) + start_cnt_r;
    } else {
        r_cnts = start_cnt_r - end_cnt_r;
    }
    *spd_r = r_cnts;
}
/* Does not handle overflow*/
uint16_t cnt_per_ms(bh_motor_t motor, bh_motor_dir_t dir){
    uint16_t start_cnt = bh_get_enc_cnt(motor);
    HAL_Delay(12);
    uint16_t end_cnt = bh_get_enc_cnt(motor);
    if(dir == DIR_FORWARD){
        //Values should be decreasing
        uint16_t cnts;
        if(end_cnt > start_cnt) { //Handle overflow case
            cnts = ((65535-end_cnt)+1) + start_cnt;
        } else {
            cnts = start_cnt - end_cnt;
        }
        return cnts;

    } else {
        //Values should be increasing
        uint16_t cnts;
        if(end_cnt < start_cnt) { //Handle overflow case
            cnts = ((65535-start_cnt)+1) + end_cnt;
        } else {
            cnts = end_cnt - start_cnt;
        }
        return cnts;

    }
}
/*Green Ye basic pid example*/
/*
void PID(void)       
{    
    if((leftSensor > hasLeftWall && rightSensor > hasRightWall))//has both walls
    {  //ccw direction is positive
        errorP = rightSensor - leftSensor - 63;//63 is the offset between left and right sensor when mouse in the middle of cell
        errorD = errorP - oldErrorP;
    }        
    else if((leftSensor > hasLeftWall))//only has left wall
    {
        errorP = 2 * (leftMiddleValue – leftSensor);
        errorD = errorP - oldErrorP;
    }
    else if((rightSensor > hasRightWall))//only has right wall
    {
        errorP = 2 * (rightSensor – rightMiddleValue);
        errorD = errorP – oldErrorP;
    }
    else if((leftSensor < hasLeftWall && rightSensor <hasRightWall))//no wall, use encoder or gyro
    {
        errorP = 0;//(leftEncoder – rightEncoder*1005/1000)*3;
        errorD = 0;
    }
    totalError = P * errorP + D * errorD;
    oldErrorP = errorP;
    setLeftPwm(leftBaseSpeed – totalError);
    setRightPwm(rightBaseSpeed + totalError);    
} */