/*
 * blue_hal.c
 *
 *  Created on: Jan 18, 2023
 *      Author: jonathanlow
 */


#include "blue_hal.h"
#include "stm32f1xx_hal.h"
#include "main.h"
#include "string.h"

#define EMITTER_PULSE_TIME_MS 1/* TODO: Find Smallest Pulse Time */
#define TX_TIMEOUT 10 /* TODO: Try makig 0*/
#define NUM_AVG_SAMPLES 1

/* Private variables ---------------------------------------------------------*/
static ADC_HandleTypeDef* hadc1;
static TIM_HandleTypeDef* htim2;
static UART_HandleTypeDef* uart_cable;

/* Private function prototypes -----------------------------------------------*/
static void ADC1_Select_CH4();
static void ADC1_Select_CH5();
static void ADC1_Select_CH8();
static void ADC1_Select_CH9();

bool Init(ADC_HandleTypeDef* adc1, TIM_HandleTypeDef* tim2, UART_HandleTypeDef* uart_handle, 
            					   TIM_HandleTypeDef* tim3, TIM_HandleTypeDef* tim4)
{
    hadc1 = adc1;
    htim2 = tim2;
    uart_cable = uart_handle;
    HAL_TIM_Encoder_Start(tim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(tim3, TIM_CHANNEL_ALL);
    return false;
}

uint16_t Measure_IR_Dist(IR_TYPE dist)
{
    GPIO_TypeDef* emitter_port;
    uint16_t emitter_pin;
    GPIO_TypeDef* receiver_port;
    uint16_t receiver_pin;

    switch(dist)
    {
        case DIST_FL:
            emitter_port    = EMIT_FL_GPIO_Port;
            emitter_pin     = EMIT_FL_Pin;
            receiver_port   = RECIV_FL_GPIO_Port;
            receiver_pin    = RECIV_FL_Pin;
            ADC1_Select_CH9();
            break;

        case DIST_FR:
            emitter_port    = EMIT_FR_GPIO_Port;
            emitter_pin     = EMIT_FR_Pin;
            receiver_port   = RECIV_FR_GPIO_Port;
            receiver_pin    = RECIV_FR_Pin;
            ADC1_Select_CH4();
            break;

        case DIST_L:
            emitter_port    = EMIT_L_GPIO_Port;
            emitter_pin     = EMIT_L_Pin;
            receiver_port   = RECIV_L_GPIO_Port;
            receiver_pin    = RECIV_L_Pin;
            ADC1_Select_CH8();
            break;

        case DIST_R:
            emitter_port    = EMIT_R_GPIO_Port;
            emitter_pin     = EMIT_R_Pin;
            receiver_port   = RECIV_R_GPIO_Port;
            receiver_pin    = RECIV_R_Pin;
            ADC1_Select_CH5();
            break;

        default :
            /* TODO: Handle Unknown Direction */
            break;
    }

    HAL_GPIO_WritePin(emitter_port, emitter_pin, GPIO_PIN_SET);
    HAL_Delay(EMITTER_PULSE_TIME_MS); /* TODO: If using free rtos*/
    HAL_GPIO_WritePin(receiver_port, receiver_pin, GPIO_PIN_SET);
    HAL_ADC_Start(hadc1);
    HAL_ADC_PollForConversion(hadc1, HAL_MAX_DELAY);

    uint16_t adc_val = HAL_ADC_GetValue(hadc1);
    HAL_ADC_Stop(hadc1);
    HAL_GPIO_WritePin(emitter_port, emitter_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(receiver_port, receiver_pin, GPIO_PIN_RESET);

    return adc_val;
}

bool Set_Motor_Dir(MOTOR_TYPE motor, MOTOR_DIR_TYPE dir)
{
    GPIO_PinState fwd_pin_state;
    GPIO_PinState back_pin_state;

    GPIO_TypeDef* fwd_port;
    uint16_t fwd_pin;
    GPIO_TypeDef* back_port;
    uint16_t back_pin;

    switch(dir) {
        case DIR_FORWARD:
            fwd_pin_state = GPIO_PIN_SET;
            back_pin_state = GPIO_PIN_RESET;
            break;

        case DIR_BACKWARD:
            fwd_pin_state = GPIO_PIN_RESET;
            back_pin_state = GPIO_PIN_SET;
            break;

        case DIR_STOP_HARD:
            /* TODO: Verify diff between HARD and SOFT stopping */
            fwd_pin_state = GPIO_PIN_RESET;
            back_pin_state = GPIO_PIN_RESET;
            break;

        case DIR_STOP_SOFT:
            /* TODO: Implement */
            break;

        default:
            break;
         /*TODO: Handle*/

    }
    
    switch(motor)
    {
        case MOTOR_LEFT:
            fwd_port  = ML_FWD_GPIO_Port;
            fwd_pin   = ML_FWD_Pin;
            back_port = ML_BACK_GPIO_Port;
            back_pin  = ML_BACK_Pin;
            break;

        case MOTOR_RIGHT:
            fwd_port  = MR_FWD_GPIO_Port;
            fwd_pin   = MR_FWD_Pin;
            back_port = MR_BACK_GPIO_Port;
            back_pin  = MR_BACK_Pin;
            break;

        default:
            break;
            /* TODO: Handle */
    }

    HAL_GPIO_WritePin(fwd_port, fwd_pin, fwd_pin_state);
    HAL_GPIO_WritePin(back_port, back_pin, back_pin_state);

    return false;
}

bool Set_Motor_PWM(MOTOR_TYPE motor, uint16_t dc)
{
    uint32_t motor_pwm_channel;
    /* Convert DC to CCR3/CCR4 value and do error checking */
    switch(motor)
    {
        case MOTOR_LEFT:
            motor_pwm_channel = TIM_CHANNEL_4;
            TIM2->CCR4 = dc;
            break;

        case MOTOR_RIGHT:
            motor_pwm_channel = TIM_CHANNEL_3;
            TIM2->CCR3 = dc;
            break;
    }

    HAL_TIM_PWM_Start(htim2, motor_pwm_channel);

    return false;
}

bool bh_uart_tx_str(uint8_t* buf)
{
    HAL_UART_Transmit(uart_cable, buf, strlen((char*)buf) + 1 , TX_TIMEOUT);
    return false;
}

bool bh_ble_tx(uint8_t *buf, uint16_t num_bytes)
{
    return false;
}

bool Set_LED(bh_led_t led, bool state)
{
    GPIO_TypeDef* led_port;
    uint16_t led_pin;

    switch(led) {
        case LED_GREEN:
            led_port = LED_GREEN_GPIO_Port;
            led_pin = LED_GREEN_Pin;
            break;

        case LED_BLUE:
            led_port = LED_BLUE_GPIO_Port;
            led_pin = LED_BLUE_Pin;
            break;

        case LED_RED:
            led_port = LED_RED_GPIO_Port;
            led_pin = LED_RED_Pin;
            break;
    }

    GPIO_PinState led_state = state ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(led_port, led_pin, led_state);

    return false;
}

bool bh_set_buzzer(uint16_t tone, bool state)
{
    return false;
}

uint16_t Get_Enc_Count(MOTOR_TYPE motor)
{
    TIM_TypeDef* enc_TIMx;
    if(motor == MOTOR_LEFT)
    {
        enc_TIMx = TIM3;
    }
    else
    {
        //TODO: Error handling or a assert
        enc_TIMx = TIM4;
    }

    return enc_TIMx->CNT;
}

bool Reset_Enc_Count(MOTOR_TYPE motor)
{
    TIM_TypeDef* enc_TIMx;
    if(motor == MOTOR_LEFT)
    {
        enc_TIMx = TIM3;
    }
    else
    {
        //TODO: Error handling or a assert
        enc_TIMx = TIM4;
    }

	enc_TIMx->CNT = 0;
    return true;
}

bool Reset_Enc_Count_To_Max(MOTOR_TYPE motor)
{
    TIM_TypeDef* enc_TIMx;
    if(motor == MOTOR_LEFT)
    {
        enc_TIMx = TIM3;
    }
    else
    {
        //TODO: Error handling or a assert
        enc_TIMx = TIM4;
    }

    enc_TIMx->CNT = 65535;
    return true;
}


uint16_t Measure_Avg_IR_Dist(IR_TYPE dist_sensor)
{
    uint32_t acc = 0;
    for(int i = 0; i < NUM_AVG_SAMPLES; ++i)
    {
        acc += Measure_IR_Dist(dist_sensor);
    }
    return (uint16_t)(acc/NUM_AVG_SAMPLES);
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

	float IR_Kp = 0.3;
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
		hasFrontWall = (Measure_Avg_IR_Dist(DIST_FR)) > 1200 ? 1 : 0;

		if(hasFrontWall)
		{
			Set_LED(LED_GREEN, 1);
			break;
		}
		else
		{
			Set_LED(LED_GREEN, 0);
		}

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


bool Rotate_Mouse_By_Enc_Ticks(MOTOR_ROT_TYPE dir, uint16_t encTicks, uint16_t speed)
{
	//bh_uart_tx_str((uint8_t *)"rotating motor\r\n");
	bool firstCount = true;

	uint16_t start_right_enc_count = 0;
	uint16_t start_left_enc_count = 0;

	uint16_t curr_right_enc_count = 0;
	uint16_t curr_left_enc_count = 0;

	// set relevant motor directions and encoder count
	// when motor spins "forward", encoder count decreases
	if (dir == ROT_CLOCKWISE)
	{
		Reset_Enc_Count_To_Max(MOTOR_LEFT);
		Reset_Enc_Count(MOTOR_RIGHT);

		Set_Motor_Dir(MOTOR_LEFT, DIR_FORWARD);
		Set_Motor_Dir(MOTOR_RIGHT, DIR_BACKWARD);
	}
	else
	{
		Set_Motor_Dir(MOTOR_LEFT, DIR_BACKWARD);
		Set_Motor_Dir(MOTOR_RIGHT, DIR_FORWARD);
	}

	bool right_done = false;
	bool left_done = false;

	//char buf[100] = {0};

	Set_Motor_PWM(MOTOR_LEFT, speed);
	Set_Motor_PWM(MOTOR_RIGHT, speed);

	while(!right_done || !left_done)
	{
		if (firstCount)
		{
			start_left_enc_count = Get_Enc_Count(MOTOR_LEFT);
			start_right_enc_count = Get_Enc_Count(MOTOR_RIGHT);

			curr_right_enc_count = start_right_enc_count;
			curr_left_enc_count = start_left_enc_count;
			firstCount = false;
		}

		//snprintf(buf, 100, "start R: %d, curr R: %d | start L: %d, curr L %d\r\n", (int)start_right_enc_count, (int)curr_right_enc_count, (int)start_left_enc_count, (int)curr_left_enc_count);
		//bh_uart_tx_str((uint8_t *)buf);
		//HAL_UART_Transmit(uart_cable, buf, strlen((char*)buf) + 1 , TX_TIMEOUT);

		// spin right motor until encTicks is reached
		if(!right_done)
		{
			if (abs(curr_right_enc_count - start_right_enc_count) < encTicks)
			{
				curr_right_enc_count = Get_Enc_Count(MOTOR_RIGHT);
			}
			else
			{
				Set_Motor_PWM(MOTOR_RIGHT, 0);
				right_done = true;
			}
		}

		// spin left motor until encTicks is reached
		if (!left_done)
		{
			if (abs(curr_left_enc_count - start_left_enc_count) < encTicks)
			{
				curr_left_enc_count = Get_Enc_Count(MOTOR_LEFT);
			}
			else
			{
				Set_Motor_PWM(MOTOR_LEFT, 0);
				left_done = true;
			}
		}
	}

	return 0;
}

bool Straight_Line_Encoder_Test(uint16_t encTicks, uint16_t targetSpeed)
{
	Reset_Enc_Count_To_Max(MOTOR_LEFT);
	Reset_Enc_Count_To_Max(MOTOR_RIGHT);

	Set_Motor_Dir(MOTOR_LEFT, DIR_FORWARD);
	Set_Motor_Dir(MOTOR_RIGHT, DIR_FORWARD);

	Set_Motor_PWM(MOTOR_LEFT, 0);
	Set_Motor_PWM(MOTOR_RIGHT, 0);

	float kp = 4;
	float kd = 0.5;
	float ki = 0.2;

	float integral = 0;
	float error = 0;
	float prevError = 0;

	uint16_t encRightStart = Get_Enc_Count(MOTOR_RIGHT);
	uint16_t encRight = encRightStart;

	uint16_t encLeftStart = Get_Enc_Count(MOTOR_LEFT);
	uint16_t encLeft = encLeftStart;

	uint16_t maxSpeed = 1800;

	if (targetSpeed > maxSpeed)
	{
		targetSpeed = maxSpeed;
	}

	int leftSpeed = targetSpeed;
	int output;

	// buf is used for debugging
	char buf[72] = {0};

	while(abs(encRightStart - encRight) < encTicks || abs(encLeftStart - encLeft) < encTicks)
	{
		snprintf(buf, 72, "ERR: %d, encLeft: %d, encRight: %d, lspd: %d, rspd: %d \r\n", (int)error, encLeft, encRight, leftSpeed, targetSpeed);
		bh_uart_tx_str((uint8_t *)buf);

		Set_Motor_PWM(MOTOR_LEFT, leftSpeed);
		Set_Motor_PWM(MOTOR_RIGHT, targetSpeed);

		encLeft = Get_Enc_Count(MOTOR_LEFT);
		encRight = Get_Enc_Count(MOTOR_RIGHT);

		error = (encLeft - encRight);

		integral += ki * error;
		output = error * kp + (prevError - error) * kd + integral;

		// only adjust speed of one motor relative to the other
		leftSpeed = targetSpeed + output;


		// make sure speeds don't go crazy from
		if (leftSpeed > maxSpeed)
		{
			leftSpeed = maxSpeed;
		}
		else if (leftSpeed < 0)
		{
			leftSpeed = 0;
		}

		prevError = error;
	}

	Set_Motor_PWM(MOTOR_LEFT, 0);
	Set_Motor_PWM(MOTOR_RIGHT, 0);

	return 0;
}

bool Spin_Motor_By_Enc_Ticks(MOTOR_TYPE motor, uint16_t enc_ticks)
{
	bool firstCount = true;

    uint16_t start_enc_count = 0;
    uint16_t curr_enc_count = start_enc_count;

    //char buf[50] = {0};
    //snprintf(buf, 50, "start tick: %d, curr tick: %d\r\n", start_enc_count, curr_enc_count);

    Set_Motor_Dir(motor, DIR_FORWARD);
    Set_Motor_PWM(motor, 1400);

    while (abs(curr_enc_count - start_enc_count) < enc_ticks || firstCount)
    {
    	if (firstCount)
    	{
    		start_enc_count = Get_Enc_Count(motor);
    		firstCount = false;
    	}

    	//snprintf(buf, 50, "start tick: %d, curr tick: %d%d\r\n", (int)start_enc_count, (int)curr_enc_count);
    	//HAL_UART_Transmit(uart_cable, buf, strlen((char*)buf) + 1 , TX_TIMEOUT);
        curr_enc_count = Get_Enc_Count(motor);
    }

    Set_Motor_Dir(motor, DIR_STOP_HARD);
    Set_Motor_PWM(motor, 0);

    return false;
}

/* Private functions ---------------------------------------------------------*/
static void ADC1_Select_CH4(void) {
  	ADC_ChannelConfTypeDef sConfig = {0};

  	sConfig.Channel = ADC_CHANNEL_4;
  	sConfig.Rank = 1;
  	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  	if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK)
  	{
  		Error_Handler();
  	}
}

static void ADC1_Select_CH5(void) {
  	ADC_ChannelConfTypeDef sConfig = {0};

  	sConfig.Channel = ADC_CHANNEL_5;
  	sConfig.Rank = 1;
  	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  	if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK)
  	{
  		Error_Handler();
  	}
}

static void ADC1_Select_CH8(void) {
  	ADC_ChannelConfTypeDef sConfig = {0};

  	sConfig.Channel = ADC_CHANNEL_8;
  	sConfig.Rank = 1;
  	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  	if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK)
  	{
  		Error_Handler();
  	}
}

static void ADC1_Select_CH9(void) {
  	ADC_ChannelConfTypeDef sConfig = {0};

  	sConfig.Channel = ADC_CHANNEL_9;
  	sConfig.Rank = 1;
  	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  	if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK)
  	{
  		Error_Handler();
  	}
}
