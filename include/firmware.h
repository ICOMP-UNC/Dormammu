/** @file firmware.h
 * @brief Firmware header file for the project.
 */

#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#ifdef DEBUG_BUILD
#include <unistd.h>


int _write(int file, char *ptr, int len);
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>

/* UART configuration */
#define BAUD_RATE 115200
#define WORD_LENGTH 8

