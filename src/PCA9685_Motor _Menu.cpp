/**********************************************************************
  PCA9685_Motor_Menu.cpp

  Robots-For-All (R4A)
  PCA9685 motor menu support
**********************************************************************/

#include "R4A_I2C.h"

//****************************************
// PCA9685 Motor menu
//****************************************

const R4A_MENU_ENTRY r4aPca9685MotorMenuTable[] =
{
    // Command  menuRoutine                 menuParam               HelpRoutine         align   HelpText
    {"b",       r4aPca9685MotorMenuBrake,   (intptr_t)"mm ssss",    r4aMenuHelpSuffix,  7,      "Motor mm brake at speed ssss"},            // 0
    {"c",       r4aPca9685MotorMenuCoast,   (intptr_t)"mm",         r4aMenuHelpSuffix,  2,      "Motor mm coasting"},                       // 1
    {"d",       r4aPca9685MotorMenuDisplay, 0,                      nullptr,            0,      "Display the motor states"},                // 2
    {"f",       r4aPca9685MotorMenuForward, (intptr_t)"mm ssss",    r4aMenuHelpSuffix,  7,      "Motor mm drive forward at speed ssss"},    // 3
    {"r",       r4aPca9685MotorMenuReverse, (intptr_t)"mm ssss",    r4aMenuHelpSuffix,  7,      "Motor mm drive in reverse at speed ssss"}, // 4
    {"s",       r4aPca9685MotorMenuStop,    0,                      nullptr,            0,      "Stop all motors"},                         // 5
    {"x",       nullptr,                    R4A_MENU_MAIN,          nullptr,            0,      "Return to the main menu"},                 // 6
};                                                                                                                                          // 7
