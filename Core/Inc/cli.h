/*
 * cli.h
 *
 *  Created on: Dec 9, 2022
 *      Author: jonathanlow
 */

#ifndef INC_CLI_H_
#define INC_CLI_H_

#include "stm32f1xx_hal.h"
//#include "stm32f1xx_hal_uart.h"

void cli_init(UART_HandleTypeDef* uart_handle);
void cli_update();
void check_to_run_cmd();

#endif /* INC_CLI_H_ */
