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
#include "string.h"
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
#define BAUD_RATE                     115200
#define WORD_LENGTH                   8
#define FIRE_ALARM_DELAY              5 * SECOND_DELAY
#define REPORT_DELAY                  15 * SECOND_DELAY
#define MEASURE_DELAY                 5 * SECOND_DELAY
#define TIMER_PERIOD_1S               10000
#define TIMER_PRESCALE_VALUE          71999
#define BUFFER_SIZE                   64
#define NUMBER_OF_ADC_CHANNELS        4
#define ADC_BUFFER_TOTAL_SIZE         (BUFFER_SIZE * NUMBER_OF_ADC_CHANNELS)
#define PLL_CLOCK                     RCC_CLOCK_HSE8_72MHZ
#define SECOND_DELAY                  1000
#define ADC_FULL_SCALE                4096.0
#define TEMPERATURE_THRESHOLD         25.0
#define PWM_TIMER_PERIOD              1000
#define DUTY_CYCLE_START              0
#define tskLED_PRIORITY               tskIDLE_PRIORITY + 1
#define tskGROUND_HUMIDITY_PRIORITY   tskIDLE_PRIORITY + 2
#define tskTEMPERATURE_PRIORITY       tskIDLE_PRIORITY + 2
#define tskCOMMUNICATION_PRIORITY     tskIDLE_PRIORITY + 2
#define tskFIRE_ALARM_PRIORITY        tskIDLE_PRIORITY + 3
#define tskRAIN_GAUGE_PRIORITY        tskIDLE_PRIORITY + 2
#define tskANALOG_READ_PRIORITY       tskIDLE_PRIORITY + 2
#define BUFFER_MESSAGE_SIZE           64
#define TIME_STRING_SIZE              9
#define UART_BUFFER_SIZE              64
#define ADC_TEMP_MAX_VALUE            1861
#define MAX_TEMP                      150
#define NUMBER_OF_DECIMALS            2
#define FIRE_ALARM_REPEATS            15
#define COMMUNICATION_TASK_STACK_SIZE 256
#define WATER_SENSOR_K                0.001924 // mm of water per Volt
#define ANEMOMETER_K                  18.361   // m/s per Volt
#define HOUR_IN_MS                    60000 //lower for testing purposes, actual value 1000ms*60s*60min
#define MAX_WIND_VALUE_ANALOG         2482
#define MAX_WIND_SPEED                50 // m/s
#define NATURAL_DRAINAGE_PER_HOUR     1 // mm
/* PWM definitions */
#define MIN_TEMPERATURE 20.0
#define MAX_TEMPERATURE 60.0
#define MIN_DUTY_CYCLE  0
#define MAX_DUTY_CYCLE  1000
enum
{
    ADC_CHANNEL_GROUND_HUMIDITY = 0,
    ADC_CHANNEL_TEMPERATURE = 1,
    ADC_CHANNEL_WATER_LEVEL = 2,
    ADC_CHANNEL_WIND = 3
};

extern struct timekeeper
{
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
} time;

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

void xTaskLedSwitching(void* args __attribute__((unused)));
void xTaskGroundHumidity(void* args __attribute__((unused)));
/**
 * @brief Task to send messages over UART, checks if there's a message in the queue and sends it
 * @param args Task arguments (not used)
 */
void xTaskSendMessage(void* args __attribute__((unused)));
/**
 * @brief Function to update Timer1 OC value to change PWM duty cycle
 * @param duty_cycle Duty cycle value to be set
 */
void updatePWM(uint16_t duty_cycle);
/**
 * @brief Function to map temperature to PWM duty cycle, linear mapping between MIN_TEMPERATURE and MAX_TEMPERATURE
 * @param temperature Temperature value to be mapped
 */
uint16_t mapTemperatureToDutyCycle(float temperature);
/**
 * @brief Function to process received message from UART
 * Currently parses checking for 'clk' to update time
 */
void process_received_message(void);
/**
 * @brief function to create a string with the current time
 * @param time Timekeeper struct with time values to be converted to string
 */
char* get_time(struct timekeeper time);

/**
 * @brief Task to create fire SOS messages and send them over UART
 * Fire alarm dies once it's triggered enough time and fire is not detected
 * @param args Task arguments (not used)
 */
void xtaskFireAlarm(void* args __attribute__((unused)));

/**
 * @brief Task to create a report with the current time and send it over UART
 * @param args Task arguments (not used)
 */
void xTaskCreateReport(void* args __attribute__((unused)));

/**
 * @brief function that takes a float and converts it to a string
 * Casts the float to an integer and decimal part, then converts them to string and concatenates them into two elements
 * in the returned array
 * @param f Float value to be converted
 * @param number_of_decimals Number of decimals to be used
 */
char* float_to_string(float f, int number_of_decimals);

/**
 * @brief Function to process messages recieved via UART
 * @param args Task arguments (not used)
 */
void xTaskProcessMessage(void* args __attribute__((unused)));
/**
 * @brief Task to monitor ground humidity, gathered by the ADC and transferred to memory by DMA
 * @param args Task arguments (not used)
 * | Humidity | Voltage | ADC Value |
 * |----------|---------|-----------|
 * | 0%       | 3.3V    | 4096      |
 * | 100%     | 0V      | 0         |
 *
 *  | Temperature | Voltage | ADC Value |
 *  |-------------|---------|-----------|
 *  | 0º C        | 0V      | 0         |
 *  | 150 ºC      | 1.5V    | 1861      |
 *
 *
 * | Wind speed | Voltage | ADC Value |
 * |------------|---------|-----------|
 * | 0 m/s      | 0V      | 0         |
 * | 50 m/s     | 2V      | 2482      |
 */
void xTaskAnalogRead(void* args __attribute__((unused)));
/**
 * @brief Task to measure water level every hour and calculate rainfall
 * Uses the water level variation to calculate rainfall, if the water level decreases, it means it rained
 * @param args Task arguments (not used)
 * | Rain gauge | Voltage | ADC Value |
 * |------------|---------|-----------|
 * | 0 mm       | 3.3V    | 4096      |
 * | 5 mm       | 0V      | 0         |
 */
void xTaskRainfall(void* args __attribute__((unused)));
