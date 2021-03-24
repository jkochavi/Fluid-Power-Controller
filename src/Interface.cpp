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
/// @brief Macro definition for GPIO pin attached to coast button.
#define coastButton  PA15        // GPIO pin for coast mode button
/// @brief Macro definition for GPIO pin attached to direct drive button.
#define directButton PB3         // GPIO pin for direct drive button
/// @brief Macro definition for GPIO pin attached to boost mode button
#define boostButton  PB4         // GPIO pin for boost mode button
/// @brief Macro definition for GPIO pin attached to regenerative braking button.
#define regenButton  PB5         // GPIO pin for regen mode button
/// @brief Macro definition for GPIO pin attached to hall effect sensor
#define HALL1        PB12        // GPIO pin for hall effect sensor
/// @brief Macro definition for GPIO pin atached to the MCP2515 chip select
#define CS           PA4         // Chip select pin for MCP2515

/** @brief This variable keeps track of which drive mode the vehicle is in. 
 *         This value is when a button is pressed. 
 */
uint8_t buttonState = 1;                                    // 1->Direct 2->Coast 3->Regen 4->Boost 5->Pedal
/** @brief This variable is changed by the display task, and keeps
 *         track of when the drive mode text on the Nextion display 
 *         needs to be updated.
 */
uint8_t previousButtonState_DSP = 255;                      // Init to force display update
/** @brief This variable is changed by the CAN task, and keeps
 *  track of when the controller needs to push an update
 * to the PLC.
 */
uint8_t previousButtonState_CAN = 255;                      // Init to force CAN transmission
/** @brief   Queue for storing accumulator pressure values. 
 *  @details Storing these values in a buffer makes up for any mismatch
 *           in transmission frequencies between the PLC and this
 *           controller. 
 */
Queue <int32_t> accumulatorPressure(20,"pressure buffer");
/** @brief   Queue for storing bike speed values. 
 *  @details Storing these values in a buffer makes up for any 
 *           mismatch in transmission frequencies between the 
 *           display and hall effect tasks.
 */
Queue <int32_t> bikeSpeed(20,"speed buffer");
/** @brief   Share that stores status of CAN connection.
 *  @details A value of TRUE means that a CAN connection is established,
 *           and a value of FALSE means that an error has occured.
 */
Share <bool> CANconnected;

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
 *  @param   sign Which signal to look for: active-high or active-low.
 *           This parameter is TRUE by default, which debounces an 
 *           active-high signal. If FALSE is passed here, then this
 *           method will debounce an active-low signal.
 *  @return  A boolean. A debounced trigger returns TRUE. Otherwise,
 *           it returns FALSE. 
 */
bool debounce(uint8_t buttonPin, uint8_t length, bool sign=true)
{
    uint8_t count = 0;                      // Arbitrary counting variable
    for (uint8_t k = 0; k < length; k++)    // Do the following *length* amount of times...
    {                                       //
        if (digitalRead(buttonPin) == sign) //      If the GPIO pin reads HIGH...
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
 *           the ID of 0x181, an arbitrary hex number. On the other end,
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
        if      (buttonState == 1) {canMSG.data[0] = 0x01;}   // If buttonState is 1...      then send a 1
        else if (buttonState == 2) {canMSG.data[0] = 0x02;}   // Else if buttonState is 2... then send a 2
        else if (buttonState == 3) {canMSG.data[0] = 0x03;}   // Else if buttonState is 3... then send a 3
        else if (buttonState == 4) {canMSG.data[0] = 0x04;}   // Else if buttonState is 4... then send a 4
        else if (buttonState == 5) {canMSG.data[0] = 0x05;}   // Else if buttonState is 5... then send a 5
        node.sendMessage(&canMSG);                            // Send the message
        previousButtonState_CAN = buttonState;                // Reset to prevent continuous transmitting
    }
}

/** @brief   Function to read the accumulator pressure from the PLC.
 *  @details This function reads the CAN bus for the accumulator pressure.
 *           The PLC transmits a pressure reading in a 2 byte message with 
 *           ID 0x181, another arbitrary hex number. The message needs to be 
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
    // A signed 32-bit integer that the CAN_readPressure() method returns.
    int32_t returnVal = CAN_ERROR;                      // Error value
    // The CAN frame object that a transmission is stored in.
    struct can_frame CANmsg;                            // Create a CAN message structure
    if (node.readMessage(&CANmsg)== MCP2515::ERROR_OK)  // If we are able to read the message without error...
    {                                                   //      Then, check the message ID.
        returnVal = CAN_OTHER;                          //      Initialize to message-received case code.
        if (CANmsg.can_id == 385)                       //      If the ID is 385, COB-ID for receiving PDO1 message...
        {                                               //
            int32_t highByte = CANmsg.data[0];          //              Then store the low byte
            int32_t lowByte = CANmsg.data[1];           //              And store the high byte
            returnVal = lowByte | (highByte<<8);        //              Concatenate into one number with bit-shifting
        }                                               //
    }                                                   //
    return returnVal;                                   // Return the number
}

/** @brief   Function to update the drive mode on the Nextion.
 *  @details This function updates the drive mode on the Nextion
 *           display. The user can change the drive mode in two ways:
 *           by pressing one of the four buttons, or pressing one
 *           of five virtual switches on the touch screen (the fifth
 *           button is for pedal charge mode). This function first
 *           checks if the user has toggled a switch on the screen 
 *           by checking a flag in the Nextion software: page2.va1.val.
 *           va1 is a boolean flag in the Nextion software that belongs
 *           to Page 2 (the switch board page). This function checks
 *           its value by reading the variable's attribute: val. 
 * @param display The Nextion display that we are interfacing with.
 */ 
void updateDriveMode(EasyNex &display)
{
    // This variable stores the value of va1, a global variable on
    // the Nextion. va1 is only 0 if the Nextion is actively displaying
    // the home page. 
    uint32_t checkTouchPress = display.readNumber("page2.va1.val"); // Check the Nextion display to see if a drive-mode was selected there
    if (buttonState != previousButtonState_DSP)                     // If a button was pressed...
    {                                                               //
        if (buttonState == DIRECT)                                  //      If we're in direct mode...
        {                                                           //
            display.writeStr("page0.t9.txt","Direct");              //          Then, update the text accordingly.
            display.writeNum("page0.va0.val",1);                    //          Update Nextion's corresponding counter variable.
        }                                                           //
        else if (buttonState == COAST)                              //      Else if we're in coast mode...
        {                                                           //          
            display.writeStr("page0.t9.txt","Coast");               //          Then, update the text accordingly
            display.writeNum("page0.va0.val",2);                    //          Update Nextion's corresponding counter variable.
        }                                                           // 
        else if (buttonState == REGEN)                              //      Else if we're in regen mode...
        {                                                           //      
            display.writeStr("page0.t9.txt","Regen");               //          Then, update the text accordingly
            display.writeNum("page0.va0.val",3);                    //          Update Nextion's corresponding counter variable
        }                                                           // 
        else if (buttonState == BOOST)                              //      Else if we're in boost mode...
        {                                                           //      
            display.writeStr("page0.t9.txt","Boost");               //          Then, update the text accordingly
            display.writeNum("page0.va0.val",4);                    //          Update Nextion's corresponding counter variable
        }                                                           // 
        else if (buttonState == PEDAL)                              //      Else if we're in pedal charge mode...
        {                                                           //      
            display.writeStr("page0.t9.txt","Pedel");               //          Then, update the text accordingly
            display.writeNum("page0.va0.val",5);                    //          Update Nextion's corresponding counter variable
        }                                                           //
        previousButtonState_DSP = buttonState;                      //      Reset to prevent continuous transmitting
        if (display.readNumber("page2.va0.val")==1)                 //      If we're on the drive mode select page...
        {                                                           //
            display.writeStr("page 2");                             //          Then refresh the page completely
        }                                                           //
    }                                                               //
    else if (checkTouchPress)                                       // Else if a switch was toggled on the display...
    {                                                               //
        buttonState = display.readNumber("page2.va1.val");          //      Then assign buttonState to what it was assigned to on the display 
        display.writeNum("page2.va1.val",0);                        //      Lower the Nextion flag
        previousButtonState_DSP = buttonState;                      //      Reset to prevent continuous transmitting
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
    // Initialise the xLastWakeTime variable with the current time.
    // It will be used to run the task at precise intervals
    TickType_t xLastWakeTime = xTaskGetTickCount();
    // A UART serial object used to communicate with the display.
    HardwareSerial NEXSerial(PA3, PA2);                                      // Create serial port at desired GPIO pins
    NEXSerial.begin(9600);                                                   // Init the serial port at 9600 baud
    // A Nextion class object used to communicate with the display.
    EasyNex myNextion(NEXSerial);                                            // Create a Nextion object
    /* A local variable for storing the accumulator pressure.
     * The CAN task retrieves the accumulator pressure from 
     * the ECDR 0506-A and stores it in an inter-task buffer.
     * The display task pulls the top value from the buffer
     * and stores it in this signed 32-bit integer. This
     * variable is pushed to the display for printing.
     */ 
    int32_t localVar_accumulatorPressure = 0;                                // Local variable for current accumulator pressure
    /* A local variable for storing the bike speed.
     * The Hall Effect task calculates the speed of the vehicle
     * and pushes it to an inter-task buffer. The display task
     * pulls the top value from the buffer and stores it in this
     * signed 32-bit integer. This variable is pushed to the 
     * display for printing.
     */
    int32_t localVar_bikeSpeed = 0;                                          // Local variable for current bike speed
    pinMode(coastButton, INPUT_PULLUP);                                      // Configure button pin for input
    pinMode(directButton, INPUT_PULLUP);                                     // Configure button pin for input
    pinMode(boostButton, INPUT_PULLUP);                                      // Configure button pin for input
    pinMode(regenButton, INPUT_PULLUP);                                      // Configure button pin for input
    attachInterrupt(digitalPinToInterrupt(coastButton),ISRcoast,RISING);     // Attach pin to ISR
    attachInterrupt(digitalPinToInterrupt(directButton),ISRdirect,RISING);   // Attach pin to ISR
    attachInterrupt(digitalPinToInterrupt(boostButton),ISRboost,RISING);     // Attach pin to ISR
    attachInterrupt(digitalPinToInterrupt(regenButton),ISRregen,RISING);     // Attach pin to ISR
    /* A local variable for storing the status of CAN connection.
     * When there is an error in CAN communication, such as if the
     * lines are not connected, the CAN task will update the 
     * inter-task variable: CANconnected. The display task 
     * reads the status of that variable and stores it in this 
     * boolean. If this variable is FALSE, then this task
     * alerts the display to print an error message to the user.
     */ 
    bool CANstatus = false;                                                  // Define local variable for CAN status
    for (;;)                                                                 // A forever loop...
    {                                                                        //
        CANconnected.get(CANstatus);                                         // Get latest CAN status
        updateDriveMode(myNextion);                                          // Update the drive mode text
        if (CANstatus)                                                       // If CAN is connected...
        {                                                                    //
            myNextion.writeStr("page0.t7.txt","Connected");                  //     Modify screen text to show connected
            accumulatorPressure.get(localVar_accumulatorPressure);           //     Pull accumulator pressure from buffer
            myNextion.writeNum("page0.n0.val",localVar_accumulatorPressure); //     Write the pressure to the display
        }                                                                    //
        else                                                                 // Otherwise...
        {                                                                    //
            myNextion.writeStr("page0.t7.txt","Not Connected");              //     Modify screen text to show not connected
        }                                                                    // 
        bikeSpeed.get(localVar_bikeSpeed);                                   // Pull speed from buffer
        Serial.println(localVar_bikeSpeed);
        myNextion.writeNum("page0.n1.val",localVar_bikeSpeed);               // Write the speed to the display
        // This type of delay waits until the given number of RTOS ticks have
        // elapsed since the task previously began running. This prevents 
        // inaccuracy due to not accounting for how long the task took to run
        vTaskDelayUntil (&xLastWakeTime, 100);
    }
}

/** @brief   Task which interacts with the CAN bus. 
 *  @details This task sends the current user-chosen drive mode to the 
 *           PLC, and reads the accumulator pressure, using the CAN bus.
 *           We interface with the CAN bus using an MCP2515 controller.
 *           On the custom PCB, the MCP2515 controller uses an 8 MHz 
 *           crystal to clock the CAN bitrate. However, the mcp2515.cpp and
 *           mcp2515.h files clock the bitrate using a 16 MHz crystal. 
 *           This parameter was altered in those files to set the correct
 *           bitrate. That modification is not documented, as the files are
 *           from a different source.
 *  @param   p_params A pointer to function parameters which we don't use.
 */
void task_CAN (void* p_params)
{
    (void)p_params;                                    // Does nothing but shut up a compiler warning
    // Initialise the xLastWakeTime variable with the current time.
    // It will be used to run the task at precise intervals
    TickType_t xLastWakeTime = xTaskGetTickCount();
    /* A local variable for storing the accumulator pressure.
     * This task retrieves the accumulator pressure from 
     * the ECDR 0506-A and stores it in this signed 32-bit.
     * This task pushes the value to an inter-task buffer, 
     * which is used by the display task to print to the user.
     */ 
    int32_t localVarPressure;                          // Local variable for accumulator pressure
    // An MCP2515 object used to communicate with the CAN controller.
    MCP2515 my2515(CS);                                // Create MCP2515 object
    my2515.reset();                                    // Initalize the controller
    my2515.setBitrate(CAN_250KBPS);                    // Set the bitrate 
    my2515.setNormalMode();                            // Set mode as normal
    for (;;)                                           // A forever loop...
    {                                                  //
        CAN_sendPress(my2515);                         //       Send the current drive mode
        localVarPressure = CAN_readPressure(my2515);   //       Read the current pressure
        if (localVarPressure != CAN_ERROR)             // If the obtained pressure did not generate errors... 
        {                                              //
            if (localVarPressure != CAN_OTHER)         //       If the appropriate message was received...
            {                                          //
                accumulatorPressure.put(localVarPressure); //   Push current pressure to the buffer
            }                                          //
            CANconnected.put(true);                    //       Alert CAN status as connected
        }                                              //
        else                                           // Otherwise... 
        {                                              //
            CANconnected.put(false);                   //       Alert CAN status as not connected
        }                                              //
        // This type of delay waits until the given number of RTOS ticks have
        // elapsed since the task previously began running. This prevents 
        // inaccuracy due to not accounting for how long the task took to run
        vTaskDelayUntil (&xLastWakeTime, 30);
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
    // Initialise the xLastWakeTime variable with the current time.
    // It will be used to run the task at precise intervals
    TickType_t xLastWakeTime = xTaskGetTickCount();
    pinMode(HALL1, INPUT);                                                // Configure pin for input
    // A floating point number that stores the bicycle wheel diameter measured, in inches.
    float wheelDiameter = 26.0;                                           // Wheel diameter in inches
    // A floating point number that stores the wheel circumference, in inches.
    float distanceTraveled = PI*wheelDiameter;                            // Wheel circumference in inches
    // A floating point number that stores the current calculated speed, in mph.
    float speed = 0.0;                                                    // Local variable for vehicle speed
    /* Timeout period, in milliseconds, before speed reverts to zero.
     * As the period in between hall effect pulses increases, the 
     * calculated speed will reduce. This variable represents the 
     * maximum time, in milliseconds, that this task should wait
     * before deciding that the vehicle is not moving. It is
     * set to 3000, which means that if a pulse is not received  
     * after 3000 ms, then the calculated speed will be zero.
     */ 
    uint16_t timeOut = 3000;                                              // Local variable for speed timeout
    // Stores timer ticks, in milliseconds, at the last pulse.
    unsigned long previousTime = 0;                                       // Local variables to keep track of 
    // Stores timer ticks, in milliseconds, at the current pulse. 
    unsigned long currentTime = 0;                                        // time between pulses
    for (;;)                                                              // A forever loop...
    {                                                                     //    
                                                                          //
        currentTime = millis();   
        bikeSpeed.put(currentTime - previousTime);
        previousTime = currentTime;
        /*
        if ((currentTime - previousTime) > timeOut)
        {
            bikeSpeed.put(0);
            previousTime = currentTime;
        }
        else if (debounce(HALL1,5,false))                                 // If a trigger is identified...
        {                                                                 //
            bikeSpeed.put(currentTime-previousTime);
            //speed = distanceTraveled / (currentTime - previousTime);         //   Compute speed based on circumfrence of circle
            //speed = speed*1000*3600/63360;                                //   Convert speed to miles per hour
            previousTime = currentTime;                                      //   Set previousTime for next trigger
            //bikeSpeed.put((int32_t)speed);                                //   Put the speed in shared task variable
        }                                                                 //
        
                                    
        speed += 1;
        if (speed > 50)
        {
            speed = 0;
        }
        */

        // This type of delay waits until the given number of RTOS ticks have
        // elapsed since the task previously began running. This prevents 
        // inaccuracy due to not accounting for how long the task took to run
        vTaskDelayUntil (&xLastWakeTime, 20);
        
    }
}