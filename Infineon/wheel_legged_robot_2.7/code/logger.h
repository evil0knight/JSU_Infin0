/*
 * logger.h
 *
 *  Created on: 2025年4月26日
 *      Author: 17104
 */

#ifndef CODE_LOGGER_H_
#define CODE_LOGGER_H_

#include <Base.h>

#define LOGGER_UART_INDEX         (UART_3)          // 日志串口对应使用的串口号
#define LOGGER_UART_BUAD_RATE     (1000000)          // 日志串口对应使用的串口波特率
#define LOGGER_UART_TX_PIN        (UART3_RX_P20_3)  // 日志串口对应模块的 TX 要接到单片机的 RX
#define LOGGER_UART_RX_PIN        (UART3_TX_P20_0)  // 日志串口对应模块的 RX 要接到单片机的 TX
#define LOGGER_UART_RTS_PIN       (P20_9)           // 日志串口对应模块的 RTS 引脚

extern int8 logger_send_flag;
extern int8 logger_save_flag;
extern uint32 time_stamp;
void logger_init(void);
void logger_send(void);
void logger_save(void);

#endif /* CODE_LOGGER_H_ */
