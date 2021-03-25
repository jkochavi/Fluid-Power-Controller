/** @file main.cpp
 *    This file contains the main program structure for the vehicle's 
 *    interface controller. It creates three RTOS tasks and begins the 
 *    task scheduler. The three tasks are used with the custom Interface
 *    header file.
 *
 *  @author  Jordan Kochavi
 * 
 *  @date    8 Dec 2020 Original file
 */

#include <Arduino.h>                              // Include Arduino library
#include <PrintStream.h>                          // Include PrintStream library
#include <STM32FreeRTOS.h>                        // Include FreeRTOS library
#include "Interface.h"                            // Include custom interface library

/** @brief   Arduino setup function which runs once at program startup.
 *  @details This function sets up a serial port for communication and creates
 *           the tasks which will be run.
 */
void setup()                                      
{
    Serial.begin(115200);                         // Begin Serial @ 115200 baud
    delay(500);                                   // Pause 0.5 seconds
    Serial.println("Starting program...");        // Print for debugging
    
    xTaskCreate (task_HALL1,                      // Create task for user interface
                 "Bike Speed",                    // Name for printouts
                 512,                             // Stack size
                 NULL,                            // Parameters for task fn.
                 1,                               // Priority
                 NULL);                           // Task handle

    xTaskCreate (task_CAN,                        // Create task for user interface
                 "CAN communication",             // Name for printouts
                 1024,                            // Stack size
                 NULL,                            // Parameters for task fn.
                 2,                               // Priority
                 NULL);                           // Task handle

    xTaskCreate (task_display,                    // Create task for user interface
                 "Nextion communication",         // Name for printouts
                 1024,                             // Stack size
                 NULL,                            // Parameters for task fn.
                 3,                               // Priority
                 NULL);                           // Task handle
    
    
    
    vTaskStartScheduler ();                       // Begin task scheduler
}

/** @brief   Arduino's low-priority loop function, which we don't use.
 *  @details A non-RTOS Arduino program runs all of its continuously running
 *           code in this function after @c setup() has finished. When using
 *           FreeRTOS, @c loop() implements a low priority task on most
 *           microcontrollers, and crashes on some others, so we'll not use it.
 */
void loop()
{

}