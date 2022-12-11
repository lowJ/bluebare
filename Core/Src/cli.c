#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "cli.h"

#define CLI_CMD_BUF_SIZE 30

#define CHAR_DEL_ASCII 8
#define CHAR_ENTER_ASCII 13

#define TX_TIMEOUT 10
#define RX_TIMEOUT 0

static uint8_t cmd_buf[CLI_CMD_BUF_SIZE] = {0};
static uint16_t cmd_buf_end = 0;

UART_HandleTypeDef* cli_uart;

static bool buf_is_full();
static void handle_cmd();
static void handle_hw_cmd();
static void handle_sm_cmd();
static void handle_dist_cmd();

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
    char* token = strtok((char*)cmd_buf, delim);

    if(token == NULL) {
        HAL_UART_Transmit(cli_uart, (uint8_t*)"\r\n", 2, TX_TIMEOUT);
        goto exit;
    }

    if(strcmp("hw", token) == 0) {
        handle_hw_cmd();
    } else if(strcmp("sm", token) == 0) {
        handle_sm_cmd();
    } else if(strcmp("dist", token) == 0) {
        handle_dist_cmd();
    } else {
        HAL_UART_Transmit(cli_uart, (uint8_t*)"Invalid Command!\r\n", 18, TX_TIMEOUT);
    }

exit:
    cmd_buf_end = 0;
}

static void handle_hw_cmd() {
    HAL_UART_Transmit(cli_uart, (uint8_t*)"Hello World \r\n", 14, TX_TIMEOUT);
}

static void handle_sm_cmd() {
    const char delim[2] = " ";
    char* token = strtok(NULL, delim);

    if(token == NULL) {
        HAL_UART_Transmit(cli_uart, (uint8_t*)"Usage 'sm [l/r] [speed]'\r\n", 26, TX_TIMEOUT);
        goto exit;
    }

    uint8_t motor_type; // Left or Right
    if(strcmp("l", token) == 0){
        motor_type = 0; /*TODO: Handle Motor type */
    } else if(strcmp("r", token) == 0){
        motor_type = 1;
    }

    token = strtok(NULL, delim);
    uint16_t speed = (uint16_t)atoi(token);
    //uint16_t res  strtol()

    if(token == NULL) {
        HAL_UART_Transmit(cli_uart, (uint8_t*)"Usage 'sm [l/r] [speed]'\r\n", 26, TX_TIMEOUT);
        goto exit;
    }

    uint8_t res = 0; //SetMotor(motor_type, speed) /* TODO: Handle return type */

    if(res == 0) {
        char buf[30] = {0};
        snprintf(buf, 30, "Set Motor to %d\r\n", speed);
        HAL_UART_Transmit(cli_uart, (uint8_t*)buf, strlen(buf), TX_TIMEOUT);
    } else {
        HAL_UART_Transmit(cli_uart, (uint8_t*)"Error occured setting motor", 27, TX_TIMEOUT);
    }

    exit: ;
}

static void handle_dist_cmd() {
    const char delim[2] = " ";
    char* token = strtok(NULL, delim);

    if(token == NULL) {
        HAL_UART_Transmit(cli_uart, (uint8_t*)"Usage 'dist [fr/l/r/fl]'\r\n", 26, TX_TIMEOUT);
        goto exit;
    }

    char buf[10];
    uint16_t dist_val = 0;

    if(strcmp("fr", token) == 0){
        dist_val = 10; /*TODO: Call distance function here*/
    } else if(strcmp("l", token) == 0){
        dist_val = 20; /*TODO: Call distance function here*/
    } else if(strcmp("r", token) == 0){
        dist_val = 30; /*TODO: Call distance function here*/
    } else if(strcmp("fl", token) == 0){
        dist_val = 65535; /*TODO: Call distance function here*/
    } else {
        HAL_UART_Transmit(cli_uart, (uint8_t*)"Invalid direction\r\n", 19, TX_TIMEOUT);
        goto exit;
    }

    snprintf(buf, 10, "%d\r\n", dist_val);
    HAL_UART_Transmit(cli_uart, (uint8_t*)buf, strlen(buf), TX_TIMEOUT);

exit: ;
}