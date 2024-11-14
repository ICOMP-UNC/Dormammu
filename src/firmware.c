/** @file firmware.c
 * @brief Firmware file for the project.
 */

#include <firmware.h>
float ground_humidity;
volatile uint16_t adc_buffer[BUFFER_SIZE];
static QueueHandle_t xUartQueue;
//semaphore for uart queue
SemaphoreHandle_t xSemaphore;
void vApplicationStackOverflowHook(TaskHandle_t xTask __attribute__((unused)), char *pcTaskName __attribute__ ((unused))) {
    (void)xTask;
    (void)pcTaskName;
    while (true){
        #ifdef DEBUG_BUILD
        printf("Stack overflow in task %s\n", pcTaskName);
        #endif
    }
}

void prvSetupHardware(void){
    // Set up system clock at the defined frequency
    rcc_clock_setup_pll(&rcc_hse_configs[PLL_CLOCK]);

    // Enable peripheral clocks
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_DMA1);
    rcc_periph_clock_enable(RCC_TIM1);


    // GPIO config
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

    // DMA config
    dma_channel_reset(DMA1, DMA_CHANNEL1);
    dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_VERY_HIGH);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&ADC_DR(ADC1));
    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)adc_buffer);
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, BUFFER_SIZE);
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);
    dma_enable_circular_mode(DMA1, DMA_CHANNEL1);
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
    dma_enable_channel(DMA1, DMA_CHANNEL1);
     

    /* USART config*/
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);
    usart_set_baudrate(USART1, BAUD_RATE);
    usart_set_databits(USART1, WORD_LENGTH);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable(USART1); 

    // Timer config
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM1, TIMER_PRESCALE_VALUE); 
    timer_set_period(TIM1, TIMER_PERIOD);
    timer_enable_counter(TIM1);
    timer_enable_irq(TIM1, TIM_DIER_UIE);


    // ADC config
    adc_power_off(ADC1);
    adc_enable_dma(ADC1);
    adc_disable_eoc_interrupt(ADC1);
    adc_enable_scan_mode(ADC1);
    adc_disable_temperature_sensor();
    adc_set_continuous_conversion_mode(ADC1);
    adc_set_right_aligned(ADC1);

    // Set up ADC channel sequence and sample time
    adc_set_regular_sequence(ADC1, NUMBER_OF_ADC_CHANNELS, ADC_CHANNEL0);
    adc_set_sample_time(ADC1, DMA_CHANNEL1, ADC_SMPR_SMP_55DOT5CYC);

    // Power on ADC and calibrate
    adc_power_on(ADC1);
    adc_reset_calibration(ADC1);
    adc_calibrate(ADC1);
    adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_SWSTART);
    adc_start_conversion_regular(ADC1);
    

}

void prvSetupTasks(void){
    xUartQueue = xQueueCreate(10, BUFFER_MESSAGE_SIZE);
    if(xUartQueue == NULL){
        #ifdef DEBUG_BUILD
        printf("Queue creation failed\n");
        #endif
        while(1);
    }
    xTaskCreate(xTaskLedSwitching, "LED_Switching", configMINIMAL_STACK_SIZE, tskLED_PRIORITY, 1, NULL);
    xTaskCreate(xTaskGroundHumidity, "GroundHumidityMonitor", configMINIMAL_STACK_SIZE, tskGROUND_HUMIDITY_PRIORITY, 1, NULL);
    xTaskCreate(xTaskSendMessage, "SendMessage", configMINIMAL_STACK_SIZE, tskIDLE_PRIORITY, 1, NULL);
}

void xTaskLedSwitching(void *args __attribute__ ((unused))){
    // LED blinks proportionally to ground humidity
    while (true){

        gpio_toggle(GPIOC, GPIO13);

        printf("LED toggled\n");
        if(ground_humidity>0){
            vTaskDelay(pdMS_TO_TICKS((ground_humidity/100)*SECOND_DELAY));
        }
        
        
        
    }

}

void xTaskGroundHumidity(void *args __attribute__ ((unused))){
        char message[BUFFER_MESSAGE_SIZE];
    while (true){
        ground_humidity=0.0;
        for (int i=0; i<BUFFER_SIZE; i++){
            ground_humidity+=((ADC_FULL_SCALE-adc_buffer[i])/ADC_FULL_SCALE)*100;
        }
        ground_humidity/=BUFFER_SIZE;
        snprintf(message, BUFFER_MESSAGE_SIZE, "Ground humidity: %.2f%%", ground_humidity);
        xQueueSend(xUartQueue, "Ground humidity: %f", portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(SECOND_DELAY));
    }
}
void xTaskSendMessage(void *args __attribute__ ((unused))){
    char message[BUFFER_MESSAGE_SIZE];
    while (true){
        if(xQueueReceive(xUartQueue, message, portMAX_DELAY) == pdTRUE){
            printf("%s\n", message);
        }
    }
}


#ifdef DEBUG_BUILD
int _write(int file, char *ptr, int len){
    (void)file;
    
    for(int i=0; i<len; i++){
        if(ptr[i] == '\n'){
            usart_send_blocking(USART1, '\r');
        }
        usart_send_blocking(USART1, ptr[i]);
    }
    return len;
    
}
#endif

void dma1_channel1_isr(void){
    if(dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TCIF)){
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TCIF);
    }
}

