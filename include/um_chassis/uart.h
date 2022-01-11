/*
 * @Author: DahlMill
 * @Date: 2020-10-01 10:47:18
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2020-10-01 21:00:49
 */
#ifndef __UM_UART_H__
#define __UM_UART_H__

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>

int set_port_attr(int fd, int baudrate, int databit, const char *stopbit, char parity, int vtime, int vmin);

#endif