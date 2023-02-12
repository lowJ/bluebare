/*
 * blue_hal.c
 *
 *  Created on: Jan 18, 2023
 *      Author: jonathanlow
 */


#include "blue_hal.h"
#include "stm32f1xx_hal.h"
#include "main.h"

#define EMITTER_PULSE_TIME_MS 1000 /* TODO: Find Smallest Pulse Time */

/* Private variables ---------------------------------------------------------*/
static ADC_HandleTypeDef* hadc1;
static TIM_HandleTypeDef* htim2;

/* Private function prototypes -----------------------------------------------*/
static void ADC1_Select_CH4();
static void ADC1_Select_CH5();
static void ADC1_Select_CH8();
static void ADC1_Select_CH9();


bool bh_init(ADC_HandleTypeDef* adc1, TIM_HandleTypeDef* tim2) {
    hadc1 = adc1;
    htim2 = tim2;
    return false;
}

uint16_t bh_measure_dist(bh_dist_t dist) {
    GPIO_TypeDef* emitter_port;
    uint16_t emitter_pin;
    GPIO_TypeDef* receiver_port;
    uint16_t receiver_pin;

    switch(dist) {
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
    HAL_Delay(EMITTER_PULSE_TIME_MS);
    HAL_GPIO_WritePin(receiver_port, receiver_pin, GPIO_PIN_SET);
    HAL_ADC_Start(hadc1);
    HAL_ADC_PollForConversion(hadc1, HAL_MAX_DELAY);

    uint16_t adc_val = HAL_ADC_GetValue(hadc1);
    HAL_ADC_Stop(hadc1);
    HAL_GPIO_WritePin(emitter_port, emitter_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(receiver_port, receiver_pin, GPIO_PIN_RESET);

    return adc_val;
}

bool bh_set_motor_dir(bh_motor_t motor, bh_motor_dir_t dir) {
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
    
    switch(motor) {
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

bool bh_set_motor_pwm(bh_motor_t motor, uint16_t dc) {
    uint32_t motor_pwm_channel;
    /* Convert DC to CCR3/CCR4 value and do error checking */
    switch(motor) {
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