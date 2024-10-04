/**********************************************************************
  PCA9685_Servo_Menu.cpp

  Robots-For-All (R4A)
  PCA9685 servo menu support
**********************************************************************/

#include "R4A_I2C.h"

//****************************************
// PCA9685 Servo menu
//****************************************

// Define the table index values
#define SERVO_PAN       0
#define SERVO_TILT      1

const R4A_MENU_ENTRY r4aPca9685ServoMenuTable[] =
{
    // Command  menuRoutine                 menuParam       HelpRoutine                     align   HelpText
    {"d",       r4aPca9685ServoMenuDisplay, 0,              nullptr,                        0,      "Display the servo states"},    // 0
    {"p",       r4aPca9685ServoMenuMove,    SERVO_PAN,      r4aPca9685ServoMenuHelpDdd,     4,      "Pan to ddd degrees"},          // 1
    {"s",       r4aPca9685ServoMenuSet,     0,              r4aPca9685ServoMenuHelpSDdd,    6,      "Set servo s to ddd degrees"},  // 2
    {"t",       r4aPca9685ServoMenuMove,    SERVO_TILT,     r4aPca9685ServoMenuHelpDdd,     4,      "Tilt to ddd degrees"},         // 3
    {"x",       nullptr,                    R4A_MENU_MAIN,  nullptr,                        0,      "Return to the main menu"},     // 4
};                                                                                                                                  // 5
