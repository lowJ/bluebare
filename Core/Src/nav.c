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
    right_wall_middle = bh_measure_dist_avg(DIST_R);
    int32_t off = left_wall_middle - right_wall_middle;
    bh_set_led(LED_RED, 0);
    bh_set_led(LED_GREEN, 0);
    if(off > -1) {
        bh_set_led(LED_GREEN, 1);
        HAL_Delay(500);
        bh_set_led(LED_GREEN, 0);
        HAL_Delay(500);
        bh_set_led(LED_GREEN, 1);
        HAL_Delay(500);
        bh_set_led(LED_GREEN, 0);
    } else {
        bh_set_led(LED_RED, 1);
        HAL_Delay(500);
        bh_set_led(LED_RED, 0);
        HAL_Delay(500);
        bh_set_led(LED_RED, 1);
        HAL_Delay(500);
        bh_set_led(LED_RED, 0);

    }
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

#define LEFT_BASE_SPD 1400
#define RIGHT_BASE_SPD 1400
#define g_P 0.3
#define g_D 0.5         //P= 0.2
#define TICKS_PER_CELL 679
void straight(uint16_t cells){
    int32_t left_right_offset =  left_wall_middle - right_wall_middle;
    int32_t p_error = 0;
    int32_t d_error = 0;
    int32_t p_error_old;
    int32_t total_error;

    bh_set_led(LED_BLUE, 0);
    bh_set_led(LED_GREEN, 0);

    if(left_right_offset > -1){
        bh_set_led(LED_GREEN, 1);
    } else {
        bh_set_led(LED_BLUE, 1);
    }

    bh_set_motor_dir(MOTOR_LEFT, DIR_FORWARD);
    bh_set_motor_dir(MOTOR_RIGHT, DIR_FORWARD);

    uint16_t goal_cnt = 65535 - (cells * 680); 

     
    bh_reset_enc_cnt(MOTOR_LEFT);
    bh_reset_enc_cnt(MOTOR_RIGHT);

    
    //while(1){
    while(bh_get_enc_cnt(MOTOR_LEFT) > goal_cnt || bh_get_enc_cnt(MOTOR_LEFT) == 0 ||
            bh_get_enc_cnt(MOTOR_RIGHT) > goal_cnt || bh_get_enc_cnt(MOTOR_RIGHT) == 0) {
        //if(fal) {
        if(bh_measure_dist_avg(DIST_FR) > 1240){
            break;

        }
        if(is_left_wall_detected() && is_right_wall_detected()) {
            bool lr_diff_is_negative = false;
            int32_t lr_diff = bh_measure_dist_avg(DIST_L) - bh_measure_dist_avg(DIST_R);
            if(lr_diff < 0) {
                p_error = abs(lr_diff) + abs(left_right_offset);
                p_error = p_error * -1;
                d_error = p_error - p_error_old;
            } else {
                p_error = abs(lr_diff) - abs(left_right_offset);
                d_error = p_error - p_error_old;
            }

            total_error = (g_P * p_error) + (g_D * d_error);
            p_error_old = p_error;
            //Positive error = veer right, negative error = veer left

            bh_set_motor_pwm(MOTOR_LEFT, LEFT_BASE_SPD + total_error);
            bh_set_motor_pwm(MOTOR_RIGHT, RIGHT_BASE_SPD - total_error);
            HAL_Delay(2);


        } else if (is_left_wall_detected()) {
            uint16_t left =  bh_measure_dist_avg(DIST_L);
            int32_t diff = left_wall_middle - left; //diff=negative too lcose to left wall
                                                    //diff = positive too far to left wall 
            p_error = diff * -1;
            //d_error = p_error - p_error_old;

            total_error = (g_P * p_error);
            
            bh_set_motor_pwm(MOTOR_LEFT, LEFT_BASE_SPD + total_error);
            bh_set_motor_pwm(MOTOR_RIGHT, RIGHT_BASE_SPD - total_error);
            HAL_Delay(2);



        } else if (is_right_wall_detected()) {

            uint16_t right =  bh_measure_dist_avg(DIST_R);
            int32_t diff = right_wall_middle - right; //diff=negative too lcose to right wall
                                                    //diff = positive too far to right wall 

            p_error = diff;
            total_error = (g_P * p_error);
            
            bh_set_motor_pwm(MOTOR_LEFT, LEFT_BASE_SPD + total_error);
            bh_set_motor_pwm(MOTOR_RIGHT, RIGHT_BASE_SPD - total_error);
            HAL_Delay(2);
            //d_error = p_error - p_error_old;

        } else {

            bh_set_motor_pwm(MOTOR_LEFT, LEFT_BASE_SPD);
            bh_set_motor_pwm(MOTOR_RIGHT, RIGHT_BASE_SPD);



        }


    }
    bh_set_motor_dir(MOTOR_LEFT, DIR_STOP_HARD);
    bh_set_motor_dir(MOTOR_RIGHT, DIR_STOP_HARD);
    bh_set_motor_pwm(MOTOR_LEFT, 0);
    bh_set_motor_pwm(MOTOR_RIGHT, 0);



}

void Move_One_Cell(uint16_t encTicks, uint16_t targetSpeed, uint16_t irLeftOffset, uint16_t irRightOffset)
{
	bool hasRightWall = (Measure_IR_Dist(DIST_R)) > 200 ? 1 : 0;
	bool hasLeftWall = (Measure_IR_Dist(DIST_L)) > 200 ? 1 : 0;
	bool hasFrontWall = (Measure_IR_Dist(DIST_FL)) > 200 ? 1 : 0;

	Reset_Enc_Count_To_Max(MOTOR_RIGHT);
	Reset_Enc_Count_To_Max(MOTOR_LEFT);

	Set_Motor_Dir(MOTOR_RIGHT, DIR_FORWARD);
	Set_Motor_Dir(MOTOR_LEFT, DIR_FORWARD);

	Set_Motor_PWM(MOTOR_RIGHT, 0);
	Set_Motor_PWM(MOTOR_LEFT, 0);

	uint16_t encMaxTicks = 65535;
	uint16_t encRightTicks = Get_Enc_Count(MOTOR_RIGHT);
	uint16_t encLeftTicks = Get_Enc_Count(MOTOR_LEFT);
	uint16_t encOffset = encLeftTicks - encRightTicks;

	uint16_t irOffset = irLeftOffset - irRightOffset;

	float ENC_Kp = 4;
	float ENC_Kd = 0.5;
	float ENC_Ki = 0.2;

	float ENC_output = 0;
	float ENC_error = 0;
	float ENC_prev_error = 0;
	float ENC_integral = 0;

	float IR_Kp = 0.2;
	float IR_Kd = 0;
	float IR_Ki = 0;

	float IR_output = 0;
	float IR_error = 0;
	float IR_prev_error = 0;
	float IR_integral = 0;

	while ((encMaxTicks - encLeftTicks) < encTicks || (encMaxTicks - encRightTicks) < encTicks)
	{
		encLeftTicks = Get_Enc_Count(MOTOR_LEFT);
		encRightTicks = Get_Enc_Count(MOTOR_RIGHT);

		hasRightWall = (Measure_Avg_IR_Dist(DIST_R)) > 200 ? 1 : 0;
		hasLeftWall = (Measure_Avg_IR_Dist(DIST_L)) > 200 ? 1 : 0;
		hasFrontWall = (Measure_Avg_IR_Dist(DIST_FL)) > 300 ? 1 : 0;

		if (!(!hasRightWall && !hasLeftWall && !hasFrontWall))
		{
			encOffset = encLeftTicks - encRightTicks;
		}

		if (hasLeftWall && hasRightWall && !hasFrontWall)
		{
			IR_error = Measure_Avg_IR_Dist(DIST_L) - Measure_Avg_IR_Dist(DIST_R) - irOffset;

			IR_output = IR_error * IR_Kp;
			IR_prev_error = IR_error;

			Set_Motor_PWM(MOTOR_LEFT, targetSpeed + IR_output);
			Set_Motor_PWM(MOTOR_RIGHT, targetSpeed);
			HAL_Delay(2);
		}
		else if (hasLeftWall && !hasRightWall && !hasFrontWall)
		{
			IR_error = Measure_Avg_IR_Dist(DIST_L) - irLeftOffset;

			IR_output = IR_error * IR_Kp;
			IR_prev_error = IR_error;

			Set_Motor_PWM(MOTOR_LEFT, targetSpeed + IR_output);
			Set_Motor_PWM(MOTOR_RIGHT, targetSpeed);
			HAL_Delay(2);
		}
		else if (!hasLeftWall && hasRightWall && !hasFrontWall)
		{
			IR_error = Measure_Avg_IR_Dist(DIST_R) - irRightOffset;

			IR_output = IR_error * IR_Kp;
			IR_prev_error = IR_error;

			Set_Motor_PWM(MOTOR_LEFT, targetSpeed - IR_output);
			Set_Motor_PWM(MOTOR_RIGHT, targetSpeed);
			HAL_Delay(2);
		}
		else if (!hasLeftWall && !hasRightWall && !hasFrontWall)
		{
			ENC_error = encLeftTicks - encRightTicks - encOffset;
			ENC_integral += ENC_error * ENC_Ki;

			ENC_output = ENC_error * ENC_Kp + (ENC_prev_error - ENC_error) * ENC_Kd + ENC_integral;
			ENC_prev_error = ENC_error;

			Set_Motor_PWM(MOTOR_LEFT, targetSpeed + ENC_output);
			Set_Motor_PWM(MOTOR_RIGHT, targetSpeed);
			HAL_Delay(2);
		}
		else
		{
			Set_Motor_PWM(MOTOR_RIGHT, 0);
			Set_Motor_PWM(MOTOR_LEFT, 0);
			break;
		}

	}

	Set_Motor_PWM(MOTOR_RIGHT, 0);
	Set_Motor_PWM(MOTOR_LEFT, 0);
}

#define FRONT_EMIT_GOAL_DIST 700
void straight_till_wall(){
    int32_t left_right_offset =  left_wall_middle - right_wall_middle;
    int32_t p_error = 0;
    int32_t d_error = 0;
    int32_t p_error_old;
    int32_t total_error;

    bh_set_motor_dir(MOTOR_LEFT, DIR_FORWARD);
    bh_set_motor_dir(MOTOR_RIGHT, DIR_FORWARD);

    bh_reset_enc_cnt(MOTOR_LEFT);
    bh_reset_enc_cnt(MOTOR_RIGHT);

    
    //while(1){
    while(bh_measure_dist_avg(DIST_FR) < FRONT_EMIT_GOAL_DIST) {
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


#define LEFT_TURN_L_BASE_SPD 1350
#define LEFT_TURN_R_BASE_SPD 1400
#define LEFT_TURN_CNT 170 //227?
#define g_P_lt 0.1
#define g_D_lt 0
void turn_left(uint16_t cnts){
    int32_t p_error = 0;
    int32_t d_error = 0;
    int32_t p_error_old;
    int32_t total_error;

    bh_set_motor_dir(MOTOR_LEFT, DIR_BACKWARD);
    bh_set_motor_dir(MOTOR_RIGHT, DIR_FORWARD);

    uint16_t l_goal_cnt = cnts; 
    uint16_t r_goal_cnt = 65535 - cnts; 

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
    bh_set_motor_dir(MOTOR_LEFT, DIR_STOP_HARD);
    bh_set_motor_dir(MOTOR_RIGHT, DIR_STOP_HARD);
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