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

bool nav_init() {
    left_wall_middle = Measure_Avg_IR_Dist(DIST_L);
    right_wall_middle = Measure_Avg_IR_Dist(DIST_R);
    return true;
}

bool is_left_wall_detected(){
    uint16_t m = Measure_Avg_IR_Dist(DIST_L);
    if(m > left_wall_middle - WALL_DETECTION_PADDING) {
       Set_LED(LED_BLUE, 1);
       return true;
    } else {
        Set_LED(LED_BLUE, 0);
        return false;
    }
}

bool is_right_wall_detected(){
    uint16_t m = Measure_Avg_IR_Dist(DIST_R);
    if(m > left_wall_middle - WALL_DETECTION_PADDING) {
        Set_LED(LED_RED, 1);
        return true;
    } else {
        Set_LED(LED_RED,0);
        return false;
    }
}

#define LEFT_BASE_SPD 1450
#define RIGHT_BASE_SPD 1400
#define g_P 0.2
#define g_D 0

void Celebrate_UCI_Demo(uint16_t irLeftOffset, uint16_t irRightOffset)
{
    irLeftOffset = left_wall_middle;
    irRightOffset = right_wall_middle;
	Move_One_Cell(680 * 3, 1400, irLeftOffset, irRightOffset);
	osDelay(500);
	Rotate_Mouse_By_Enc_Ticks(ROT_CLOCKWISE, 222, 1400);
	osDelay(500);
	Move_One_Cell(680 * 1, 1400, irLeftOffset, irRightOffset);
	osDelay(500);
	Rotate_Mouse_By_Enc_Ticks(ROT_CLOCKWISE, 222, 1400);
	osDelay(500);

	Move_One_Cell(680 * 3, 1400, irLeftOffset, irRightOffset);
	osDelay(500);
	Rotate_Mouse_By_Enc_Ticks(ROT_CLOCKWISE, 222, 1400);
	osDelay(500);
	Move_One_Cell(680 * 1, 1400, irLeftOffset, irRightOffset);
	osDelay(500);
	Rotate_Mouse_By_Enc_Ticks(ROT_CLOCKWISE, 222, 1400);

	//Straight_Line_Encoder_Test(700, 1400);
	//osDelay(500);
//	Straight_Line_Encoder_Test(700, 1400);
//	Rotate_Mouse_By_Enc_Ticks(ROT_CLOCKWISE, 232, 1400);
//	Straight_Line_Encoder_Test(700, 1400);
//	straight();
}

void MoveStraight()
{

}

void straight(){
    int32_t left_right_offset =  left_wall_middle - right_wall_middle;
    int32_t p_error = 0;
    int32_t d_error = 0;
    int32_t p_error_old;
    int32_t total_error;

    Set_Motor_Dir(MOTOR_LEFT, DIR_FORWARD);
    Set_Motor_Dir(MOTOR_RIGHT, DIR_FORWARD);

    uint16_t goal_cnt = 65535 - 680; 

    Reset_Enc_Count_To_Max(MOTOR_LEFT);
    Reset_Enc_Count_To_Max(MOTOR_RIGHT);
    
    //while(1){
    while(Get_Enc_Count(MOTOR_LEFT) > goal_cnt || Get_Enc_Count(MOTOR_LEFT) == 0)
    {
        if(is_left_wall_detected() && is_right_wall_detected())
        {
            bool lr_diff_is_negative = false;

            int32_t lr_diff = Measure_Avg_IR_Dist(DIST_L) - Measure_Avg_IR_Dist(DIST_R);

            if (lr_diff < 0)
            {
            	lr_diff_is_negative = true;
            }

            p_error = abs(lr_diff) - abs(left_right_offset);

            if(lr_diff_is_negative)
			{
            	p_error = p_error * -1;
			}

            d_error = p_error - p_error_old;
        }

        else if (is_left_wall_detected())
        {

        }

        else if (is_right_wall_detected())
        {

        }

        else
        {

        }

        total_error = (g_P * p_error) + (g_D * d_error);
        p_error_old = p_error;
        //Positive error = veer right, negative error = veer left

        Set_Motor_PWM(MOTOR_LEFT, LEFT_BASE_SPD + total_error);
        //bh_set_motor_pwm(MOTOR_RIGHT, RIGHT_BASE_SPD - total_error);
        Set_Motor_PWM(MOTOR_RIGHT, RIGHT_BASE_SPD - total_error);

        HAL_Delay(2);

    }
    Set_Motor_PWM(MOTOR_LEFT, 0);
    Set_Motor_PWM(MOTOR_RIGHT, 0);

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
