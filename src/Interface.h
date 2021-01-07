/** @file Interface.h
 *    This is the header file for the inteface task functions.
 *  @author Jordan Kochavi
 * 
 *  @date   8 Dec 2020 Original file
 */

#ifndef UI_H
#define UI_H
#include <Arduino.h>        // Include Arduino library  
#include <PrintStream.h>    // Include PrintStream libary
#include <STM32FreeRTOS.h>  // Include FreeRTOS library
// Some compiler definitions that make coding a bit easier
#define COAST  0                     
#define DIRECT 1
#define BOOST  2 
#define REGEN  3

/// Task functions
void task_display(void* params); // The display task function
void task_CAN(void* params);     // The CAN task function
void task_HALL1(void* params);   // The hall effect task function
#endif // UI_H