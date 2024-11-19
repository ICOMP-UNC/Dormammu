/** @file firmware.c
 * @brief Firmware file for the project.
 */

#include <firmware.h>
float ground_humidity, temperature, water_level, wind_speed, old_water_level, rainfall;
volatile uint16_t adc_buffer[ADC_BUFFER_TOTAL_SIZE];
static QueueHandle_t xUartSendQueue;
static QueueHandle_t xUartReceiveQueue;
// semaphore for uart queue
SemaphoreHandle_t xCommunicationSemaphore;
SemaphoreHandle_t xTimeSemaphore;
struct timekeeper time, sunset, sunrise = {0, 0, 0};
uint8_t sun_debounce_flag = 0;
TaskHandle_t xFireHandler = NULL;
volatile char uart_buffer[UART_BUFFER_SIZE];
volatile uint8_t uart_buffer_index = 0;
void vApplicationStackOverflowHook(TaskHandle_t xTask __attribute__((unused)), char* pcTaskName __attribute__((unused)))
{
    (void)xTask;
    (void)pcTaskName;
    while (true)
    {
#ifdef DEBUG_BUILD
        printf("Stack overflow in task %s\n", pcTaskName);
#endif
    }
}

void prvSetupHardware(void)
{
    internal_clock_setup();
    // Clock timer config
    timer2_config();
    // TIMER1 config for PWM
    timer1_config();
    // GPIO config
    gpio_config();
    // DMA config
    dma_config();
    /* USART config*/
    usart_config();
    // ADC config
    adc_setup();
    timer3_config();
    exti_setup();
}

void prvSetupTasks(void)
{

    xUartReceiveQueue = xQueueCreate(10, BUFFER_MESSAGE_SIZE);
    xUartSendQueue = xQueueCreate(10, BUFFER_MESSAGE_SIZE);
    xCommunicationSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(xCommunicationSemaphore);
    if (xUartSendQueue == NULL || xUartReceiveQueue == NULL)
    {
#ifdef DEBUG_BUILD
        printf("Queue creation failed\n");
#endif
        while (1);
    }
    xTaskCreate(xTaskAnalogRead, "GroundHumidityMonitor", configMINIMAL_STACK_SIZE, tskANALOG_READ_PRIORITY, 1, NULL);
    xTaskCreate(xTaskSendMessage, "SendMessage", configMINIMAL_STACK_SIZE, tskCOMMUNICATION_PRIORITY, 1, NULL);
    xTaskCreate(xTaskRainfall, "RainfallMonitor", configMINIMAL_STACK_SIZE, tskRAIN_GAUGE_PRIORITY, 1, NULL);
    xTaskCreate(xTaskCreateReport, "CreateReport", configMINIMAL_STACK_SIZE, tskCOMMUNICATION_PRIORITY, 1, NULL);
    xTaskCreate(
        xTaskProcessMessage, "ProcessMessageInputs", COMMUNICATION_TASK_STACK_SIZE, tskCOMMUNICATION_PRIORITY, 1, NULL);
}

void xtaskFireAlarm(void* args __attribute__((unused)))
{
    char message[BUFFER_MESSAGE_SIZE];
    uint8_t fire_alarm_times_counter = 0;
    while (true)
    {
        fire_alarm_times_counter++;
        snprintf(message, sizeof(message), "SOS! Fire alarm! Temperature: ");
        strncat(message, float_to_string(temperature, NUMBER_OF_DECIMALS), sizeof(message));
        if (xSemaphoreTake(xCommunicationSemaphore, portMAX_DELAY) == pdTRUE)
        {
            xQueueSend(xUartSendQueue, message, portMAX_DELAY);
            xSemaphoreGive(xCommunicationSemaphore);
        }
        if (fire_alarm_times_counter > FIRE_ALARM_REPEATS && gpio_get(GPIOA, GPIO2))
        {
            exti_reset_request(EXTI2);
            nvic_enable_irq(NVIC_EXTI2_IRQ);
            vTaskDelete(xFireHandler);
        }
        vTaskDelay(pdMS_TO_TICKS(FIRE_ALARM_DELAY));
    }
}

void xTaskAnalogRead(void* args __attribute__((unused)))
{
    while (true)
    {
        ground_humidity = 0.0;
        for (int i = ADC_CHANNEL_GROUND_HUMIDITY; i < ADC_BUFFER_TOTAL_SIZE; i += NUMBER_OF_ADC_CHANNELS)
        {
            ground_humidity += ((ADC_FULL_SCALE - adc_buffer[i]) / ADC_FULL_SCALE) * 100;
        }
        ground_humidity /= BUFFER_SIZE;
        temperature = 0.0;
        for (int i = ADC_CHANNEL_TEMPERATURE; i < ADC_BUFFER_TOTAL_SIZE; i += NUMBER_OF_ADC_CHANNELS)
        {
            temperature += adc_buffer[i];
        }
        temperature = (MAX_TEMP * temperature) / (ADC_TEMP_MAX_VALUE * BUFFER_SIZE);
        uint16_t duty_cycle = mapTemperatureToDutyCycle(temperature);
        updatePWM(duty_cycle);
        wind_speed = 0.0;
        for (int i = ADC_CHANNEL_WIND; i < ADC_BUFFER_TOTAL_SIZE; i += NUMBER_OF_ADC_CHANNELS)
        {
            wind_speed += adc_buffer[i];
        }
        wind_speed = (wind_speed / (BUFFER_SIZE * MAX_WIND_VALUE_ANALOG)) * MAX_WIND_SPEED;
        vTaskDelay(pdMS_TO_TICKS(MEASURE_DELAY));
    }
}

void xTaskRainfall(void* args __attribute__((unused)))
{
    water_level = 0.0;
    while (true)
    {
        water_level = 0.0;
        for (int i = ADC_CHANNEL_WATER_LEVEL; i < ADC_BUFFER_TOTAL_SIZE; i += NUMBER_OF_ADC_CHANNELS)
        {
            water_level += adc_buffer[i];
        }
        water_level = (ADC_FULL_SCALE - (water_level / BUFFER_SIZE)) * WATER_SENSOR_K;
        if ((water_level - old_water_level) > NATURAL_DRAINAGE_PER_HOUR)
        {
            rainfall = (water_level - old_water_level - NATURAL_DRAINAGE_PER_HOUR);
        }
        else
        {
            rainfall = 0;
        }
        old_water_level = water_level;
        vTaskDelay(pdMS_TO_TICKS(HOUR_IN_MS));
    }
}

void xTaskSendMessage(void* args __attribute__((unused)))
{
    char message[BUFFER_MESSAGE_SIZE];
    while (true)
    {
        if (xQueueReceive(xUartSendQueue, message, portMAX_DELAY) == pdTRUE)
        {
            printf("%s\n", message);
        }
    }
}

void xTaskCreateReport(void* args __attribute__((unused)))
{
    char message[BUFFER_MESSAGE_SIZE];
    while (true)
    {
        xSemaphoreTake(xCommunicationSemaphore, portMAX_DELAY);
        sprintf(message, "Sunrise: %s ", get_time(sunrise));
        xQueueSend(xUartSendQueue, message, portMAX_DELAY);
        sprintf(message, "Sunset: %s ", get_time(sunset));
        xQueueSend(xUartSendQueue, message, portMAX_DELAY);

        sprintf(message, "Ground humidity: ");
        strncat(message, float_to_string(ground_humidity, NUMBER_OF_DECIMALS), sizeof(message));
        strncat(message, "%", sizeof(message));
        xQueueSend(xUartSendQueue, message, portMAX_DELAY);
        sprintf(message, "Temperature: ");
        strncat(message, float_to_string(temperature, NUMBER_OF_DECIMALS), sizeof(message));
        strncat(message, " C", sizeof(message));
        xQueueSend(xUartSendQueue, message, portMAX_DELAY);
        sprintf(message, "Rain gauge: ");
        strncat(message, float_to_string(rainfall, NUMBER_OF_DECIMALS), sizeof(message));
        strncat(message, " mm", sizeof(message));
        xQueueSend(xUartSendQueue, message, portMAX_DELAY);
        sprintf(message, "Wind speed: ");
        strncat(message, float_to_string(wind_speed, NUMBER_OF_DECIMALS), sizeof(message));
        strncat(message, " m/s", sizeof(message));
        xQueueSend(xUartSendQueue, message, portMAX_DELAY);
        xSemaphoreGive(xCommunicationSemaphore);
        vTaskDelay(pdMS_TO_TICKS(REPORT_DELAY));
    }
}

#ifdef DEBUG_BUILD
int _write(int file, char* ptr, int len)
{
    (void)file;

    for (int i = 0; i < len; i++)
    {
        if (ptr[i] == '\n')
        {
            usart_send_blocking(USART1, '\r');
        }
        usart_send_blocking(USART1, ptr[i]);
    }
    return len;
}
#endif
/**
 * @brief ADC1 ISR to handle the ADC conversion
 * If the ADC conversion is complete, it clears the interrupt flag
 */
void dma1_channel1_isr(void)
{
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TCIF))
    {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TCIF);
    }
}

uint16_t mapTemperatureToDutyCycle(float temperature)
{
    if (temperature <= MIN_TEMPERATURE)
    {
        return MIN_DUTY_CYCLE;
    }
    else if (temperature >= MAX_TEMPERATURE)
    {
        return MAX_DUTY_CYCLE;
    }
    else
    {
        return (uint16_t)((temperature - MIN_TEMPERATURE) * (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE) /
                              (MAX_TEMPERATURE - MIN_TEMPERATURE) +
                          MIN_DUTY_CYCLE);
    }
}

void updatePWM(uint16_t duty_cycle)
{
    timer_set_oc_value(TIM1, TIM_OC1, duty_cycle);
}
/**
 * @brief Timer2 ISR to keep track of system time
 * Increments the time struct every second
 */
void tim2_isr()
{
    if (timer_get_flag(TIM2, TIM_SR_UIF))
    {
        timer_clear_flag(TIM2, TIM_SR_UIF);
        time.seconds++;
        if (time.seconds == 60)
        {
            time.seconds = 0;
            time.minutes++;
            if (time.minutes == 60)
            {
                time.minutes = 0;
                time.hours++;
                if (time.hours == 24)
                {
                    time.hours = 0;
                }
            }
        }
    }
}

char* get_time(struct timekeeper printtime)
{
    static char time_str[TIME_STRING_SIZE];
    snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", printtime.hours, printtime.minutes, printtime.seconds);

    return time_str;
}
/**
 * @brief USART1 ISR to handle the received messages
 * If the received character is a newline, carriage return or the buffer is full, it sends the message to the queue to
 * be processed. If the message is not complete, it appends the character to the buffer
 */
void usart1_isr(void)
{
    static BaseType_t xHigherPriorityTaskWoken;
    if (usart_get_flag(USART1, USART_SR_RXNE))
    {
        char received_char = usart_recv(USART1);
        if (received_char == '\n' || received_char == '\r' || uart_buffer_index >= UART_BUFFER_SIZE - 1)
        {

            uart_buffer_index = 0;
            xQueueSendFromISR(xUartReceiveQueue, &uart_buffer, xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            for (int i = 0; i < UART_BUFFER_SIZE; i++)
            {
                uart_buffer[i] = 0;
            }
        }
        else
        {
            uart_buffer[uart_buffer_index++] = received_char;
        }
    }
}
/**
 * @brief EXTI3 ISR to handle the sensor input
 * If the sensor input is triggered, it starts a timer to debounce the input
 * Clears isr flag and sets the debounce sunlight sensor flag to 1
 *
 */
void exti3_isr()
{
    if (exti_get_flag_status(EXTI3))
    {
        timer_set_counter(TIM3, 0);
        timer_enable_counter(TIM3);
        sun_debounce_flag = 1;
        exti_reset_request(EXTI3);
    }
}
/**
 * @brief EXTI2 ISR to handle the fire alarm
 * If the fire alarm is triggered, it creates a task to send an SOS message over UART
 * Clears isr flag and disables the EXTI2 interrupt to avoid multiple triggers while the task is running
 */
void exti2_isr()
{
    if (xFireHandler == NULL)
    {
        xTaskCreate(xtaskFireAlarm, "FireAlarm", configMINIMAL_STACK_SIZE, tskFIRE_ALARM_PRIORITY, 1, xFireHandler);
    }
    exti_reset_request(EXTI2);
    nvic_disable_irq(NVIC_EXTI2_IRQ);
}

/**
 * @brief Timer3 ISR to debounce the sensor input
 * Checks whether the sun is setting or rising based on the sensor input if the sensor input is high it means it is now
 * dark and the sun is setting, if the sensor input is low it means it is now light and the sun is rising implements a
 * debounce mechanism to avoid multiple triggers that can be expanded to avoid overwriting the earliest detected value
 * uses a flag in preparation for future debouncing needs
 */
void tim3_isr()
{
    if (timer_get_flag(TIM3, TIM_SR_UIF))
    {
        timer_clear_flag(TIM3, TIM_SR_UIF);
        if (sun_debounce_flag == 1)
        {

            if (gpio_get(GPIOA, GPIO3)) // sensor send high signal when it is dark
            {
                sunset = time;
            }
            else
            {
                sunrise = time;
            }
            sun_debounce_flag = 0;
        }
        timer_disable_counter(TIM3);
    }
}

char* float_to_string(float f, int number_of_decimals)
{
    static char buffer[32];
    int whole = (int)f;
    int decimal = (int)((f - whole) * 10 * number_of_decimals);
    sprintf(buffer, "%d.%d", whole, decimal);
    return buffer;
}

void xTaskProcessMessage(void* args __attribute__((unused)))
{
    char message[BUFFER_MESSAGE_SIZE];
    while (true)
    {
        if (xQueueReceive(xUartReceiveQueue, message, portMAX_DELAY) == pdTRUE)
        {
            if (strncmp((const char*)message, "clk", 3) == 0)
            {
                int hours, minutes, seconds;
                // Extract the next 6 characters and parse them as HHMMSS
                if (sscanf((const char*)message + 3, "%02d%02d%02d", &hours, &minutes, &seconds) == 3)
                {

                    // Update the clock variables
                    if (hours < 24)
                    {
                        time.hours = hours;
                    }
                    if (minutes < 60)
                    {
                        time.minutes = minutes;
                    }
                    if (seconds < 60)
                    {
                        time.seconds = seconds;
                    }
                }
            }
            else
            {
                if (strncmp((const char*)message, "rclk", 4) == 0)
                {
                    // reports the clock time via UART
                    sprintf(message, "System time: %s ", get_time(time));
                    xSemaphoreTake(xCommunicationSemaphore, portMAX_DELAY);
                    xQueueSend(xUartSendQueue, message, portMAX_DELAY);
                    xSemaphoreGive(xCommunicationSemaphore);
                }
            }
        }
    }
}

void timer1_config(void)
{
    rcc_periph_reset_pulse(RST_TIM1);
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM1, TIMER_PRESCALE_VALUE);
    timer_set_period(TIM1, PWM_TIMER_PERIOD);
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_value(TIM1, TIM_OC1, DUTY_CYCLE_START);
    timer_set_oc_polarity_high(TIM1, TIM_OC1);
    timer_enable_break_main_output(TIM1);
    timer_enable_oc_output(TIM1, TIM_OC1);
    timer_enable_preload(TIM1);
    timer_enable_counter(TIM1);
}

void dma_config(void)
{
    dma_channel_reset(DMA1, DMA_CHANNEL1);
    dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_VERY_HIGH);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&ADC_DR(ADC1));
    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)adc_buffer);
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, ADC_BUFFER_TOTAL_SIZE);
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);
    dma_enable_circular_mode(DMA1, DMA_CHANNEL1);
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
    dma_enable_channel(DMA1, DMA_CHANNEL1);
}
void internal_clock_setup(void)
{
    // Set up system clock at the defined frequency
    rcc_clock_setup_pll(&rcc_hse_configs[PLL_CLOCK]);

    // Enable peripheral clocks
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_DMA1);
    rcc_periph_clock_enable(RCC_TIM1);
}
void gpio_config(void)
{
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0 | GPIO1);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO8);
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0 | GPIO1);
}

void exti_setup(void)
{
    // exti_setup on PA3
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO3);
    exti_select_source(EXTI3, GPIOA);
    exti_set_trigger(EXTI3, EXTI_TRIGGER_BOTH);
    exti_enable_request(EXTI3);
    nvic_enable_irq(NVIC_EXTI3_IRQ);

    // exti_setup on PA2
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO2);
    exti_select_source(EXTI2, GPIOA);
    exti_set_trigger(EXTI2, EXTI_TRIGGER_FALLING);
    exti_enable_request(EXTI2);
    nvic_enable_irq(NVIC_EXTI2_IRQ);
}

void timer3_config(void)
{
    // TIM3 config for ISR rebounding
    rcc_periph_clock_enable(RCC_TIM3);
    timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM3, TIMER_PRESCALE_VALUE);
    timer_set_period(TIM3, TIMER_PERIOD_1S);
    timer_enable_irq(TIM3, TIM_DIER_UIE);
    nvic_enable_irq(NVIC_TIM3_IRQ);
}

void usart_config(void)
{
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);
    usart_set_baudrate(USART1, BAUD_RATE);
    usart_set_databits(USART1, WORD_LENGTH);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable_rx_interrupt(USART1);
    nvic_enable_irq(NVIC_USART1_IRQ);
    usart_enable(USART1);
}

void adc_setup(void)
{
    adc_power_off(ADC1);
    adc_enable_dma(ADC1);
    adc_disable_eoc_interrupt(ADC1);
    adc_enable_scan_mode(ADC1);
    adc_set_continuous_conversion_mode(ADC1);
    adc_set_right_aligned(ADC1);

    // Set up ADC channel sequence and sample time
    const uint8_t adc_channels[NUMBER_OF_ADC_CHANNELS] = {ADC_CHANNEL0, ADC_CHANNEL1, ADC_CHANNEL8, ADC_CHANNEL9};
    adc_set_regular_sequence(ADC1, NUMBER_OF_ADC_CHANNELS, adc_channels);
    adc_set_sample_time(ADC1, ADC_CHANNEL0, ADC_SMPR_SMP_55DOT5CYC);
    adc_set_sample_time(ADC1, ADC_CHANNEL1, ADC_SMPR_SMP_55DOT5CYC);
    adc_set_sample_time(ADC1, ADC_CHANNEL8, ADC_SMPR_SMP_55DOT5CYC);
    adc_set_sample_time(ADC1, ADC_CHANNEL9, ADC_SMPR_SMP_55DOT5CYC);
    // Power on ADC and calibrate
    adc_power_on(ADC1);
    adc_reset_calibration(ADC1);
    adc_calibrate(ADC1);
    adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_SWSTART);
    adc_start_conversion_regular(ADC1);
}
void timer2_config(void)
{
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    rcc_periph_clock_enable(RCC_TIM2);
    timer_set_prescaler(TIM2, TIMER_PRESCALE_VALUE);
    timer_set_period(TIM2, TIMER_PERIOD_1S);
    nvic_enable_irq(NVIC_TIM2_IRQ);
    timer_enable_irq(TIM2, TIM_DIER_UIE);
    timer_enable_counter(TIM2);
}
