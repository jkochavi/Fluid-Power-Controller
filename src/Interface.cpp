/** @file Interface.cpp
 *    This file contains the function files for the RTOS
 *    tasks used with the interface controller. It manages
 *    three tasks: display, CAN, and hall effect. In the 
 *    display task, the controller interfaces with a 
 *    Nextion display using the EasyNextionLibrary header
 *    file. The controller communicates with the Nextion
 *    using UART. In the CAN task, the controller interfaces
 *    with an MCP2515 CAN controller using SPI, which outputs
 *    a CAN signal to the CAN bus. In the hall effect task,
 *    the controller reads a GPIO pin using a polling method.
 *
 *  @author  Jordan Kochavi
 * 
 *  @date    8 Dec 2020 Original file
 */

#include "Interface.h"           // Include custom interface library
#include "taskshare.h"           // Include libary for inter-task variables
#include "taskqueue.h"           // Include library for inter-task buffers
#include "EasyNextionLibrary.h"  // Include Nextion library
#include <STM32FreeRTOS.h>       // Include FreeRTOS library
#include "SPI.h"                 // Include SPI library
#include "mcp2515.h"             // Include MCP2515 library
#define coastButton  PA15        // GPIO pin for coast mode button
#define directButton PB3         // GPIO pin for direct drive button
#define boostButton  PB4         // GPIO pin for boost mode button
#define regenButton  PB5         // GPIO pin for regen mode button
#define HALL1        PB10        // GPIO pin for hall effect sensor
#define CS           PA4         // Chip select pin for MCP2515

/// This variable keeps track of which drive mode the vehicle is in. 
/// This value is when a button is pressed. 
uint8_t buttonState = 0;                                    // 0->Coast, 1->Direct, 2->Boost, 3->Regen
/// This variable is changed by the display task, and keeps
/// track of when the drive mode text on the Nextion display 
/// needs to be updated.
uint8_t previousButtonState_DSP = 255;                      // Init to force display update
/// This variable is changed by the CAN task, and keeps
/// track of when the controller needs to push an update
/// to the PLC.
uint8_t previousButtonState_CAN = 255;                      // Init to force CAN transmission
/// Queue for storing accumulator pressure values. Storing
/// these values in a buffer makes up for any mismatch
/// in transmission frequencies between the PLC and this
/// controller. 
Queue <int32_t> accumulatorPressure(10,"buffer");
/// Queue for storing bike speed values. Storing these values 
/// in a buffer makes up for any mismatch in transmission 
/// frequencies between the display and hall effect tasks.
Queue <int32_t> bikeSpeed(100,"buffer");

/** @brief   Function to debounce a signal from a GPIO pin.
 *  @details This function reads the value from a GPIO pin for a 
 *           user-specified duration to prevent signal noise. When a 
 *           drive mode button is pressed, we need to ensure that the 
 *           rider is pressing a button intentionally, instead of an
 *           accidental press or noise from vibration. This function
 *           returns true if the function takes enough readings to 
 *           conclude that a button has been pressed.
 *  @param   buttonPin The GPIO pin to read.
 *  @param   length How many readings to take to debounce the signal.
 */
bool debounce(uint8_t buttonPin, uint8_t length)
{
    uint8_t count = 0;                      // Arbitrary counting variable
    for (uint8_t k = 0; k < length; k++)    // Do the following *length* amount of times...
    {                                       //
        if (digitalRead(buttonPin))         //      If the GPIO pin reads HIGH...
        {                                   //          
            count ++;                       //          Increment count
        }                                   //
    }                                       //
    if (count > length-2)                   // If we have taken enough HIGH readings... 
    {                                       //
        return true;                        //      Then we have a press! Return true
    }                                       //
    else                                    // Else...
    {                                       //  
        return false;                       //      We don't have a press, return false
    }
}

/** @brief   Interrupt service routine for the coast mode button.
 *  @details This function is triggered by the rising edge of the 
 *           signal from the coast button GPIO pin. First, it 
 *           debounces the signal to make sure that the button
 *           was actually pressed, then it sets buttonState 
 *           accordingly.
 */
void ISRcoast()
{
    if (debounce(coastButton,10)) {buttonState = COAST;}
}

/** @brief   Interrupt service routine for the direct drive button.
 *  @details This function is triggered by the rising edge of the 
 *           signal from the direct drive button GPIO pin. First, it 
 *           debounces the signal to make sure that the button
 *           was actually pressed, then it sets buttonState 
 *           accordingly.
 */
void ISRdirect()
{
    if (debounce(directButton,10)) {buttonState = DIRECT;}
}

/** @brief   Interrupt service routine for the boost mode button.
 *  @details This function is triggered by the rising edge of the 
 *           signal from the boost button GPIO pin. First, it 
 *           debounces the signal to make sure that the button
 *           was actually pressed, then it sets buttonState 
 *           accordingly.
 */
void ISRboost()
{
    if (debounce(boostButton,10)) {buttonState = BOOST;}
}

/** @brief   Interrupt service routine for the regen mode button.
 *  @details This function is triggered by the rising edge of the 
 *           signal from the regen button GPIO pin. First, it 
 *           debounces the signal to make sure that the button
 *           was actually pressed, then it sets buttonState 
 *           accordingly.
 */
void ISRregen()
{
    if (debounce(regenButton,10)) {buttonState = REGEN;}
}

/** @brief   Function to transmit the current button state to the PLC.
 *  @details This function sends a CAN message to the CAN bus based on 
 *           the current button state. This function only transmits a 
 *           message if a button was recently pressed. To do this,
 *           we use the global variable previousButtonState_CAN, which 
 *           changes only when buttonState changes. This way, we only
 *           send one transmission to the CAN bus whenever a button is
 *           pressed, instead of continuously. To sent the message, we
 *           construct a CAN message that is only one byte long, with 
 *           the ID of 0x0F6, an arbitrary hex number. On the other end,
 *           the PLC will first read the ID of the message, and then 
 *           interpret its data. 
 *  @param node The MCP2515 controller that we are interfacing with.
 */
void CAN_sendPress(MCP2515 &node)
{
    if (buttonState != previousButtonState_CAN)
    {
        struct can_frame canMSG;                              // Create a CAN message structure
        canMSG.can_id = 0x181;                                // COB-ID for transmitting a PDO message
        canMSG.can_dlc = 1;                                   // Define the message length as 1 byte 
        if      (buttonState == 0) {canMSG.data[0] = 0x01;}   // If buttonState is 0...      then send a 1
        else if (buttonState == 1) {canMSG.data[0] = 0x02;}   // Else if buttonState is 1... then send a 2
        else if (buttonState == 2) {canMSG.data[0] = 0x03;}   // Else if buttonState is 2... then send a 3
        else if (buttonState == 3) {canMSG.data[0] = 0x04;}   // Else if buttonState is 3... then send a 4
        node.sendMessage(&canMSG);                            // Send the message
        previousButtonState_CAN = buttonState;                // Reset to prevent continuous transmitting
    }
}

/** @brief   Function to read the accumulator pressure from the PLC.
 *  @details This function reads the CAN bus for the accumulator pressure.
 *           The PLC transmits a pressure reading in a 2 byte message with 
 *           ID 0x036, another arbitrary hex number. The message needs to be 
 *           2 bytes long because the accumulator pressure can be up to 
 *           3000 psi, which requires 2 bytes. To retrieve the measurement,
 *           we need to read each byte, one at a time. The PLC sends the low
 *           byte first, and then the second. After we read both bytes of data,
 *           we concatenate them into a single number by left-shifting the high
 *           byte by 8, and adding it to the low byte. The function returns a 
 *           32-bit integer for printing to the Nextion.
 *  @param node The MCP2515 controller that we are interfacing with.
 */
int32_t CAN_readPressure(MCP2515 &node)
{
    int32_t returnVal = 9999;                           // Arbitrary error value
    struct can_frame CANmsg;                            // Create a CAN message structure
    if (node.readMessage(&CANmsg)== MCP2515::ERROR_OK)  // If we are able to read the message without error...
    {                                                   //      Then, check the message ID.
        if (CANmsg.can_id == 385)                       //      If the ID is 0x301, COB-ID for receiving PDO2 message...
        {                                               //
            int32_t highByte = CANmsg.data[0];           //              Then store the low byte
            int32_t lowByte = CANmsg.data[1];          //              And store the high byte
            returnVal = lowByte | (highByte<<8);        //              Concatenate into one number
        }                                               //
        else
        {
            returnVal = CANmsg.can_id;
        }
    }                                                   //
    return returnVal;                                   // Return the number
}

/** @brief   Function to update the drive mode on the Nextion.
 *  @details This function updates the text that displays which drive
 *           mode the vehicle is in. To do this, we first need to make
 *           sure that the Nextion is on the right page. Otherwise, 
 *           we would send data that applies to other variables on other
 *           pages. Based on the current buttonState, the text is updated
 *           accordingly.
 * @param display The Nextion display that we are interfacing with.
 */ 
void updateDriveMode(EasyNex &display)
{
    /// This variable stores the value of va1, a global variable on
    /// the Nextion. va1 is only 0 if the Nextion is actively displaying
    /// the home page. 
    uint32_t checkPage = display.readNumber("va1.val");             // Read the value of va1.
    if (buttonState != previousButtonState_DSP && checkPage == 0)   // If we are on the home page and a button was pressed...
    {                                                               //
        if (buttonState == COAST)                                   //      If we're in coast mode...
        {                                                           //
            display.writeStr("t0.txt","Coast");                     //          Then, update the text accordingly.
            display.writeNum("va0.val",0);                          //          Update Nextion's corresponding counter variable.
        }                                                           //
        else if (buttonState == DIRECT)                             //      Else if we're in direct mode...
        {                                                           //          
            display.writeStr("t0.txt","Direct");                    //          Then, update the text accordingly
            display.writeNum("va0.val",1);                          //          Update Nextion's corresponding counter variable.
        }                                                           // 
        else if (buttonState == BOOST)                              //      Else if we're in boost mode...
        {                                                           //      
            display.writeStr("t0.txt","Boost");                     //          Then, update the text accordingly
            display.writeNum("va0.val",2);                          //          Update Nextion's corresponding counter variable
        }                                                           // 
        else if (buttonState == REGEN)                              //      Else if we're in regen mode...
        {                                                           //      
            display.writeStr("t0.txt","Regen");                     //          Then, update the text accordingly
            display.writeNum("va0.val",3);                          //          Update Nextion's corresponding counter variable
        }                                                           //
        previousButtonState_DSP = buttonState;                      // Reset to prevent continuous transmitting
    }
}

/** @brief   Task which interacts with the Nextion display. 
 *  @details This task updates various data on the Nextion, which includes
 *           the accumulator pressure and the speed of the vehicle. 
 *  @param   p_params A pointer to function parameters which we don't use.
 */
void task_display (void* p_params)
{
    (void)p_params;                                                          // Does nothing but shut up a compiler warning
    HardwareSerial NEXSerial(PA3, PA2);                                      // Create serial port at desired GPIO pins
    NEXSerial.begin(9600);                                                   // Init the serial port at 9600 baud
    EasyNex myNextion(NEXSerial);                                            // Create a Nextion object
    int32_t localVar_accumulatorPressure = 0;                                // Local variable for current accumulator pressure
    int32_t localVar_bikeSpeed = 0;                                          // Local variable for current bike speed
    pinMode(coastButton, INPUT_PULLUP);                                      // Configure button pin for input
    pinMode(directButton, INPUT_PULLUP);                                     // Configure button pin for input
    pinMode(boostButton, INPUT_PULLUP);                                      // Configure button pin for input
    pinMode(regenButton, INPUT_PULLUP);                                      // Configure button pin for input
    attachInterrupt(digitalPinToInterrupt(coastButton),ISRcoast,RISING);     // Attach pin to ISR
    attachInterrupt(digitalPinToInterrupt(directButton),ISRdirect,RISING);   // Attach pin to ISR
    attachInterrupt(digitalPinToInterrupt(boostButton),ISRboost,RISING);     // Attach pin to ISR
    attachInterrupt(digitalPinToInterrupt(regenButton),ISRregen,RISING);     // Attach pin to ISR
    for (;;)                                                                 // A forever loop...
    {                                                                        //
        updateDriveMode(myNextion);                                          //     Update the drive mode text
        accumulatorPressure.get(localVar_accumulatorPressure);               //     Pull accumulator pressure from buffer
        bikeSpeed.get(localVar_bikeSpeed);                                   //     Pull speed from buffer
        myNextion.writeNum("n0.val",localVar_bikeSpeed);                     //     Write the speed to the display
        myNextion.writeNum("n1.val",localVar_accumulatorPressure);           //     Write the pressure to the display
        vTaskDelay(1);                                                       //     Delay 1 RTOS tick (15ms)
    }
}

/** @brief   Task which interacts with the CAN bus. 
 *  @details This task sends the current user-chosen drive mode to the 
 *           PLC, and reads the accumulator pressure, using the CAN bus.
 *           We interface with the CAN bus using an MCP2515 controller.
 *  @param   p_params A pointer to function parameters which we don't use.
 */
void task_CAN (void* p_params)
{
    (void)p_params;                                    // Does nothing but shut up a compiler warning
    int32_t localVarPressure;                          // Local variable for accumulator pressure
    MCP2515 my2515(CS);                                // Create MCP2515 object
    my2515.reset();                                    // Initalize the controller
    my2515.setBitrate(CAN_250KBPS);                    // Set the bitrate 
    my2515.setNormalMode();                            // Set mode as normal
    for (;;)                                           // A forever loop...
    {                                                  //
        CAN_sendPress(my2515);                         //       Send the current drive mode
        localVarPressure = CAN_readPressure(my2515);   //       Read the current pressure
        if (localVarPressure > 0)                      // If the obtained pressure did not generate errors... 
        {                                              //
            accumulatorPressure.put(localVarPressure); //       Push current pressure to the buffer
        }                                              //
        vTaskDelay(1);                                 //       Delay 2 RTOS ticks
    }
}

/** @brief   Task which reads a hall effect sensor. 
 *  @details This task polls the GPIO pin for the hall effect sensor. 
 *           Whenever the signal reads high, the task retains the elapsed
 *           time, in milliseconds, between pulses. We use the circumference of
 *           the wheel to calculate the distance traveled between pulses,
 *           and then divide by the time between pulses to calculate speed, which
 *           is subsequently converted to RPM. 
 *  @param   p_params A pointer to function parameters which we don't use.
 */
void task_HALL1 (void* p_params)
{
    (void)p_params;                                                       // Does nothing but shut up a compiler warning
    pinMode(HALL1, INPUT_PULLUP);                                         // Configure pin for input
    float wheelDiameter = 26;                                             // Wheel diameter in inches
    float distanceTraveled = PI*wheelDiameter;                            // Wheel circumference in inches
    float speed = 0.0;                                                    // Local variable for vehicle speed
    unsigned long previousTime;                                           // Local variables to keep track of 
    unsigned long currentTime;                                            // time between pulses
    for (;;)                                                              // A forever loop...
    {                                                                     //    
        /*                                                                //
        if (debounce(HALL1,5))
        {
            currentTime = millis();
            speed = distanceTraveled / (currentTime - previousTime);
            speed = speed*1000*3600/63360;
            previousTime = currentTime;
        }
        */
        bikeSpeed.put((int32_t)speed);
        speed += 1;
        if (speed > 50)
        {
            speed = 0;
        }
        vTaskDelay(1);
    }
}