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

/* Private function prototypes -----------------------------------------------*/
static void ADC1_Select_CH4();
static void ADC1_Select_CH5();
static void ADC1_Select_CH8();
static void ADC1_Select_CH9();


bool bh_init(ADC_HandleTypeDef* adc1) {
    hadc1 = adc1;
    return false;
}

uint16_t bh_measure_dist(bh_dist_t dist) {
    GPIO_TypeDef* emitter_port;
    uint16_t emitter_pin;
    GPIO_TypeDef* receiver_port;
    uint16_t receiver_pin;

    switch(dist) {
        case (DIST_FL):
            emitter_port    = EMIT_FL_GPIO_Port;
            emitter_pin     = EMIT_FL_Pin;
            receiver_port   = RECIV_FL_GPIO_Port;
            receiver_pin    = RECIV_FL_Pin;
            ADC1_Select_CH9();
            break;
        case (DIST_FR):
            emitter_port    = EMIT_FR_GPIO_Port;
            emitter_pin     = EMIT_FR_Pin;
            receiver_port   = RECIV_FR_GPIO_Port;
            receiver_pin    = RECIV_FR_Pin;
            //ADC1_Select_CH4()
            ADC1_Select_CH4();
            break;
        case (DIST_L):
            emitter_port    = EMIT_L_GPIO_Port;
            emitter_pin     = EMIT_L_Pin;
            receiver_port   = RECIV_L_GPIO_Port;
            receiver_pin    = RECIV_L_Pin;
            ADC1_Select_CH8();
            break;
        case (DIST_R):
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