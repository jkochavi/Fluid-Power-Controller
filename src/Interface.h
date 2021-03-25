/** @file Interface.h
 *    This is the header file for the inteface task functions.
 *  @author Jordan Kochavi
 *  @date   8 Dec 2020 Original file
 */

#ifndef UI_H
#define UI_H
#include "Arduino.h"        // Include Arduino library  
#include "PrintStream.h"    // Include PrintStream libary
#include "STM32FreeRTOS.h"  // Include FreeRTOS library
//  Some compiler definitions that make coding a bit easier        
/// @brief Macro definition for direct drive button state.          
#define DIRECT    1
/// @brief Macro definition for coast mode button state.
#define COAST     2
/// @brief Macro definition for regenerative braking button state.
#define REGEN     3
/// @brief Macro definition for boost mode button state.
#define BOOST     4
/// @brief Macro definition for pedal charge button state.
#define PEDAL     5
/** @brief   Macro definition for CAN error code.
 *  @details If a message is unable to be received from the 
 *           ECDR 0506-A, then the CAN task will assign the 
 *           stored message variable as this error code.
 *           This is used to alert the user if an error has
 *           occured in the CAN transmision.
 */
#define CAN_ERROR 9999
/** @brief   Macro definition for diagnosing other tranmsission errors.
 *  @details When debugging CAN transmission problems, it is sometimes
 *           helpful to determine if the physical connection is the 
 *           problem, or if the problem is software-related. The 
 *           CAN_ERROR message is used to diagnose hardware problems,
 *           either with the physical CAN wires or the MCP2515. This
 *           message is used if there isn't a problem with the 
 *           physical link, but a different message was received
 *           from what the CAN task was looking for.
 */
#define CAN_OTHER 9998

// Task functions
void task_display(void* params); // The display task function
void task_CAN(void* params);     // The CAN task function
void task_HALL1(void* params);   // The hall effect task function

#endif // UI_H