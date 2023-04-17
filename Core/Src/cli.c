#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "cmsis_os.h"
#include "cli.h"
#include "blue_hal.h"
#include "nav.h"

#define CLI_CMD_BUF_SIZE 30

#define CHAR_DEL_ASCII 8
#define CHAR_ENTER_ASCII 13

#define TX_TIMEOUT 10
#define RX_TIMEOUT 0
#define REPEATER_DELAY 100

static uint8_t cmd_buf[CLI_CMD_BUF_SIZE] = {0};
static uint16_t cmd_buf_end = 0;

UART_HandleTypeDef* cli_uart;

static bool buf_is_full();
static void handle_cmd();
static void handle_cmd_prefix();
static void handle_hw_cmd();
static void handle_led_cmd();
static void handle_sm_cmd();
static void handle_dist_cmd();
static void handle_enc_cmd();
static void handle_spd_cmd();

void cli_init(UART_HandleTypeDef* uart_handle) {
    cli_uart = uart_handle;
}
void cli_update() {
    uint8_t received;
    HAL_StatusTypeDef res = HAL_UART_Receive(cli_uart, &received, 1, RX_TIMEOUT);
    if(res == HAL_OK){
        if(received == CHAR_ENTER_ASCII) {
            handle_cmd();
        } else if(received == CHAR_DEL_ASCII) {
            if(cmd_buf_end > 0) {
                cmd_buf_end--;
            }
        } else {
            if(buf_is_full()) {
                HAL_UART_Transmit(cli_uart, (uint8_t*)"Buffer Full. Please Retype.\r\n", 29, TX_TIMEOUT);
                cmd_buf_end = 0;
            } else {
                cmd_buf[cmd_buf_end++] = received;
            }
        }
    }
}

static bool buf_is_full() {
    if(cmd_buf_end >= CLI_CMD_BUF_SIZE-1) {
        return true;
    }
    return false;
}

static void handle_cmd() {

    cmd_buf[cmd_buf_end] = '\0';
    const char delim[2] = " ";

    uint8_t save_buf[CLI_CMD_BUF_SIZE];
    strncpy((char*)save_buf, (char*)cmd_buf, sizeof(save_buf));

    char* token = strtok((char*)cmd_buf, delim);

    if(token == NULL) {
        bh_uart_tx_str((uint8_t *)"\r\n");
        goto exit;
    }

    bool repeater = false;
    if(strcmp("rr", token) == 0) {
        repeater = true;
    }

    if(repeater) {
        bool stop_repeat = false;

        while(!stop_repeat){
            token = strtok(NULL, delim);

            if(token == NULL) {
                bh_uart_tx_str((uint8_t *)"No command given to repeater\r\n");
                goto exit;
            }

            handle_cmd_prefix(token);

            uint8_t received;
            HAL_StatusTypeDef res = HAL_UART_Receive(cli_uart, &received, 1, 0);
            if(res != HAL_TIMEOUT){ //If keypress detected, stop repeating command
                stop_repeat = true;
            } else {
                osDelay(REPEATER_DELAY);
                strncpy((char*)cmd_buf, (char*)save_buf, sizeof(cmd_buf));
                token = strtok((char*)cmd_buf, delim); //Parse the rr first before looping
            }
        }

    } else {
        handle_cmd_prefix(token);

    }

exit:
    cmd_buf_end = 0;
}

//When calling this function, strtok must have just finished parsing the command prefix.
static void handle_cmd_prefix(char* token) {
        if(strcmp("hw", token) == 0) {
            handle_hw_cmd();
        } else if(strcmp("sm", token) == 0) {
            handle_sm_cmd();
        } else if(strcmp("dist", token) == 0) {
            handle_dist_cmd();
        } else if(strcmp("led", token) == 0) {
            handle_led_cmd();
        } else if(strcmp("enc", token) ==0) {
            handle_enc_cmd();
        } else if(strcmp("spd", token) ==0) {
            handle_spd_cmd();
        } else {
            bh_uart_tx_str((uint8_t *)"Invalid Command! \r\n");
        }
}

static void handle_hw_cmd() {
    bh_uart_tx_str((uint8_t *)"Hello World \r\n");
}

static void handle_led_cmd() {
    const char delim[2] = " ";
    char* token = strtok(NULL, delim);

    if(token == NULL) {
        bh_uart_tx_str((uint8_t *)"Usage 'led [r/g/b] [0/1]'\r\n");

        goto exit;
    }

    bh_led_t led = LED_BLUE;
    if(strcmp("r", token) == 0){
        led = LED_RED;
    } else if(strcmp("g", token) == 0){
        led = LED_GREEN;
    } else if(strcmp("b", token) == 0){
        led = LED_BLUE;
    } else {
        bh_uart_tx_str((uint8_t *)"Invalid LED [r/g/b]\r\n");
        led = LED_BLUE;
    }

    bool state = false;
    token = strtok(NULL, delim);
    if(strcmp("1", token) == 0){
        state = true;
    } else if (strcmp("0", token) == 0){
        state = false;
    } else {
        bh_uart_tx_str((uint8_t *)"Invalid LED state [0/1]\r\n");
        state = false;
    } 
    bh_set_led(led, state);

    exit: ;
}

static void handle_sm_cmd() {
    const char delim[2] = " ";
    char* token = strtok(NULL, delim);

    if(token == NULL) {
        bh_uart_tx_str((uint8_t *)"Usage 'sm [l/r] [f/b/s] [speed] (1200+)'\r\n");
        goto exit;
    }

    bh_motor_t motor_type; // Left or Right
    if(strcmp("l", token) == 0){
        motor_type = MOTOR_LEFT;
    } else if(strcmp("r", token) == 0){
        motor_type = MOTOR_RIGHT;
    } else {
        bh_uart_tx_str((uint8_t *)"Invalid motor. Use [l/r]\r\n");
        goto exit;
    }

    token = strtok(NULL, delim);
    if(token == NULL) {
        bh_uart_tx_str((uint8_t *)"Usage 'sm [l/r] [f/b/s] [speed] (1200+)'\r\n");
        goto exit;
    }

    bh_motor_dir_t motor_dir;
    if(strcmp("f", token) == 0){
        motor_dir = DIR_FORWARD;
    } else if(strcmp("b", token) == 0){
        motor_dir = DIR_BACKWARD;
    } else if(strcmp("s", token) == 0){
        motor_dir = DIR_STOP_HARD;
    } else {
        bh_uart_tx_str((uint8_t *)"Invalid motor direction\r\n");
        goto exit;
    }

    token = strtok(NULL, delim);
    uint16_t speed = (uint16_t)atoi(token);

    if(token == NULL) {
        bh_uart_tx_str((uint8_t *)"Usage 'sm [l/r] [f/b/s] [speed] (1200+)'\r\n");
        goto exit;
    }

    uint8_t res = 0; //SetMotor(motor_type, speed) /* TODO: Handle return type */

    if(res == 0) {
        char buf[50] = {0};
        bh_set_motor_dir(motor_type, motor_dir);
        bh_set_motor_pwm(motor_type, speed);
        snprintf(buf, 50, "Set Motor to %d, dir %d, motorT %d\r\n", speed, motor_dir, motor_type);
        bh_uart_tx_str((uint8_t *)buf);
    } else {
        bh_uart_tx_str((uint8_t *)"Error occured setting motor");
    }

    exit: ;
}

static void handle_dist_cmd() {
    const char delim[2] = " ";
    char* token = strtok(NULL, delim);

    if(token == NULL) {
        bh_uart_tx_str((uint8_t *)"Usage 'dist [fr/l/r/fl]'\r\n");
        goto exit;
    }

    char buf[10];
    uint16_t dist_val = 0;

    if(strcmp("fr", token) == 0){
        dist_val = bh_measure_dist(DIST_FR);
    } else if(strcmp("l", token) == 0){
        dist_val = bh_measure_dist(DIST_L);
    } else if(strcmp("r", token) == 0){
        dist_val = bh_measure_dist(DIST_R);
    } else if(strcmp("fl", token) == 0){
        dist_val = bh_measure_dist(DIST_FL);
    } else {
        bh_uart_tx_str((uint8_t *)"Invalid direction\r\n");
        goto exit;
    }

    snprintf(buf, 10, "%dh\r\n", dist_val);
    bh_uart_tx_str((uint8_t *)buf);

exit: ;
}

static void handle_enc_cmd() {
    const char delim[2] = " ";
    char* token = strtok(NULL, delim);

    if(token == NULL) {
        bh_uart_tx_str((uint8_t *)"Usage enc [l/r] [g/r](g=get, r=reset)\r\n");
        goto exit;
    }

    bh_motor_dir_t motor;

    if(strcmp("l", token) == 0) {
        //Left motor
        motor = MOTOR_LEFT;
    } else if (strcmp("r", token) == 0) {
        //Right motor
        motor = MOTOR_RIGHT;
    } else {
        bh_uart_tx_str((uint8_t *)"Invalid motor. Use [l/r]\r\n");
        goto exit;
    }

    token = strtok(NULL, delim);
    if(token == NULL) {
        bh_uart_tx_str((uint8_t *)"Usage enc [l/r] [g/r](g=get, r=reset) \r\n");
        goto exit;
    }

    if(strcmp("g", token) == 0) {
        //Get encoder count
        char buf[10];
        uint16_t enc_cnt = bh_get_enc_cnt(motor);
        snprintf(buf, 10, "%dh\r\n", enc_cnt);
        bh_uart_tx_str((uint8_t *)buf);

    } else if (strcmp("r", token) == 0) {
        //Reset encoder count
        bh_reset_enc_cnt(motor);
        //TODO: check res
    } else {
        bh_uart_tx_str((uint8_t *)"Invalid operation. Use [g/r](g=get, r=reset)\r\n");
        goto exit;
    }

exit: ;
}


static void handle_spd_cmd() {
    const char delim[2] = " ";
    char* token = strtok(NULL, delim);

    if(token == NULL) {
        bh_uart_tx_str((uint8_t *)"Usage spd [l/r] [f/b] \r\n");
        goto exit;
    }

    bh_motor_dir_t motor;

    if(strcmp("l", token) == 0) {
        //Left motor
        motor = MOTOR_LEFT;
    } else if (strcmp("r", token) == 0) {
        //Right motor
        motor = MOTOR_RIGHT;
    } else {
        bh_uart_tx_str((uint8_t *)"Invalid motor. Use [l/r]\r\n");
        goto exit;
    }

    token = strtok(NULL, delim);
    if(token == NULL) {
        bh_uart_tx_str((uint8_t *)"Usage spd [l/r] [f/b] \r\n");
        goto exit;
    }

    if(strcmp("f", token) == 0) {
        uint16_t spd = cnt_per_ms(motor, DIR_FORWARD);
        char buf[10];
        snprintf(buf, 10, "%d\r\n", spd);
        bh_uart_tx_str((uint8_t *)buf);
        
    } else if (strcmp("b", token) == 0) {
        uint16_t spd = cnt_per_ms(motor, DIR_BACKWARD);
        char buf[10];
        snprintf(buf, 10, "%d\r\n", spd);
        bh_uart_tx_str((uint8_t *)buf);

    } else {
        bh_uart_tx_str((uint8_t *)"Usage spd [l/r] [f/b] \r\n");
        goto exit;
    }

exit: ;
}