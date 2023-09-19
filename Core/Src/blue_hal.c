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
#include <math.h>
#define EMITTER_PULSE_TIME_MS 1/* TODO: Find Smallest Pulse Time */
#define TX_TIMEOUT 10 /* TODO: Try makig 0*/
#define NUM_AVG_SAMPLES 1
#define DC_FOR_100_DC 1800

/* Private variables ---------------------------------------------------------*/
static ADC_HandleTypeDef* hadc1;
static ADC_HandleTypeDef* hadc2;
static TIM_HandleTypeDef* htim2;
static UART_HandleTypeDef* uart_cable;

/* Private function prototypes -----------------------------------------------*/
static void ADC1_Select_CH4();
static void ADC1_Select_CH5();
static void ADC1_Select_CH8();
static void ADC1_Select_CH9();
static void ADC2_Select_CH0();
static void ADC2_Select_CH1();


bool bh_init(ADC_HandleTypeDef* adc1, TIM_HandleTypeDef* tim2, UART_HandleTypeDef* uart_handle, 
            TIM_HandleTypeDef* tim3, TIM_HandleTypeDef* tim4, ADC_HandleTypeDef* adc2) {
    hadc1 = adc1;
    hadc2 = adc2;
    htim2 = tim2;
    uart_cable = uart_handle;
    HAL_TIM_Encoder_Start(tim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(tim3, TIM_CHANNEL_ALL);
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
            fwd_pin_state = GPIO_PIN_SET;
            back_pin_state = GPIO_PIN_SET;
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

    if(dir == DIR_STOP_HARD) {
        bh_set_motor_pwm(motor, DC_FOR_100_DC);
    }

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

bool bh_uart_tx_str(uint8_t* buf) {
    HAL_UART_Transmit(uart_cable, buf, strlen((char*)buf) + 1 , TX_TIMEOUT);
    return false;
}

bool bh_ble_tx(uint8_t *buf, uint16_t num_bytes) {
    return false;
}

bool bh_set_led(bh_led_t led, bool state) {
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
bool bh_set_buzzer(uint16_t tone, bool state) {
    return false;
}

uint16_t bh_get_enc_cnt(bh_motor_t motor) {
    TIM_TypeDef* enc_TIMx;
    if(motor == MOTOR_LEFT){
        enc_TIMx = TIM3;
    } else {
        //TODO: Error handling or a assert
        enc_TIMx = TIM4;
    }


    return enc_TIMx->CNT;
}
bool bh_reset_enc_cnt(bh_motor_t motor) {
    TIM_TypeDef* enc_TIMx;
    if(motor == MOTOR_LEFT){
        enc_TIMx = TIM3;
    } else {
        //TODO: Error handling or a assert
        enc_TIMx = TIM4;
    }

    enc_TIMx->CNT = 0;
    return true;
}

uint16_t bh_measure_dist_avg(bh_dist_t dist_sensor){
    uint32_t acc = 0;
    for(int i = 0; i < NUM_AVG_SAMPLES; ++i) {
        acc += bh_measure_dist(dist_sensor);
    }
    return (uint16_t)(acc/NUM_AVG_SAMPLES);
}

uint16_t bh_measure_gyro_outz(){
    ADC2_Select_CH1();
    HAL_ADC_Start(hadc2);
    HAL_ADC_PollForConversion(hadc2, HAL_MAX_DELAY);

    uint16_t adc_val = HAL_ADC_GetValue(hadc2);

    HAL_ADC_Stop(hadc2);
    return adc_val;
}

uint16_t bh_measure_gyro_vref(){
    ADC2_Select_CH0();
    HAL_ADC_Start(hadc2);
    HAL_ADC_PollForConversion(hadc2, HAL_MAX_DELAY);

    uint16_t adc_val = HAL_ADC_GetValue(hadc2);

    HAL_ADC_Stop(hadc2);
    return adc_val;
}

int32_t bh_measure_gyro_diffz(){
    int32_t ret = bh_measure_gyro_outz() - bh_measure_gyro_vref();
    return ret;
}

int32_t bh_get_angle(){
    uint32_t conv_factor = 1124;//1ADC = 1.124deg/s, Scaled up to avoid decimal

    uint32_t acc = 0; 
    uint32_t goal_acc = 90000; //90deg scaled
    while(1){
        uint32_t sample = conv_factor * (abs(1800 - bh_measure_gyro_outz()));
        acc += sample/1000;


        // char buf[10];
        // snprintf(buf, 10, "%d\r\n", acc);
        // bh_uart_tx_str((uint8_t *)buf);
        
        if(acc >= goal_acc/2){
            bh_uart_tx_str("AT 90 DONE");

            break;
        }
        HAL_Delay(1);
    }

    return 1 ;
}

int32_t bh_gyro_avg(){
    float avg = 0;
    for(int i = 0; i < 6000; i++) {
        int32_t new_sample = bh_measure_gyro_diffz();
        avg -= avg / 6000;
        avg += new_sample / 6000;

        HAL_Delay(1);
    }
    
    return (int32_t)avg;

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

static void ADC2_Select_CH0() {
  	ADC_ChannelConfTypeDef sConfig = {0};

  	sConfig.Channel = ADC_CHANNEL_0;
  	sConfig.Rank = 1;
  	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  	if (HAL_ADC_ConfigChannel(hadc2, &sConfig) != HAL_OK)
  	{
  		Error_Handler();
  	}
}

static void ADC2_Select_CH1() {
  	ADC_ChannelConfTypeDef sConfig = {0};

  	sConfig.Channel = ADC_CHANNEL_1;
  	sConfig.Rank = 1;
  	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  	if (HAL_ADC_ConfigChannel(hadc2, &sConfig) != HAL_OK)
  	{
  		Error_Handler();
  	}
}