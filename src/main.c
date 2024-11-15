/** @file main.c
 * @brief Main file for the project.
 */

#include <firmware.h>

int main(void)
{
    // Initialize the firmware
    prvSetupHardware();

    // Initiate RTOS tasks
    prvSetupTasks();

    // Start the RTOS scheduler
    vTaskStartScheduler();

    // Main loop
    while (1)
    {

        // Do something
    }
}