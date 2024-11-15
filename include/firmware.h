/** @file firmware.h
 * @brief Firmware header file for the project.
 */

#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#ifdef DEBUG_BUILD
#include <unistd.h>

int _write(int file, char* ptr, int len);
#endif

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "stdio.h"
#include "task.h"
#include "timers.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>

/* UART configuration */
#define BAUD_RATE   115200
#define WORD_LENGTH 8

#define TIMER_PRESCALE_VALUE   72000
#define TIMER_PERIOD           60000
#define BUFFER_SIZE            64
#define NUMBER_OF_ADC_CHANNELS 2
#define ADC_BUFFER_TOTAL_SIZE  (BUFFER_SIZE * NUMBER_OF_ADC_CHANNELS)
#define PLL_CLOCK              RCC_CLOCK_HSE8_72MHZ
#define SECOND_DELAY           1000
#define ADC_FULL_SCALE         4096.0

#define tskLED_PRIORITY             tskIDLE_PRIORITY + 1
#define tskGROUND_HUMIDITY_PRIORITY tskIDLE_PRIORITY + 2
#define tskTEMPERATURE_PRIORITY     tskIDLE_PRIORITY + 2
#define tskCOMMUNICATION_PRIORITY   tskIDLE_PRIORITY + 2
#define BUFFER_MESSAGE_SIZE 64

enum
{
    ADC_CHANNEL_GROUND_HUMIDITY = 0,
    ADC_CHANNEL_TEMPERATURE = 1
};

/**
 * @brief Initialize and configure hardware.
 * Configures system clocks, GPIO, ADC
 */
void prvSetupHardware(void);
/**
 * @brief Initialize and configure tasks.
 *
 */
void prvSetupTasks(void);

/* ------------------ Task Definitions --------------*/
/**
 * @brief Test function to toggle LED with blinking rate proportional to ground humidity
 * @param args Task arguments (not used)
 */
void xTaskLedSwitching(void* args __attribute__((unused)));
/**
 * @brief Task to monitor ground humidity, gathered by the ADC and transferred to memory by DMA
 * @param args Task arguments (not used)
 * | Humidity | Voltage | ADC Value |
 * |----------|---------|-----------|
 * | 0%       | 3.3V    | 4096      |
 * | 100%     | 0V      | 0         |
 */
void xTaskGroundHumidity(void* args __attribute__((unused)));
/**
 * @brief Task to send messages over UART, checks if there's a message in the queue and sends it
 * @param args Task arguments (not used)
 */
void xTaskSendMessage(void* args __attribute__((unused)));

/**
 * @brief Task to monitor temperature using LM35 sensor and ADC channel 1
 * @param args Task arguments (not used)
 */
void xTaskTemperature(void* args __attribute__((unused)));