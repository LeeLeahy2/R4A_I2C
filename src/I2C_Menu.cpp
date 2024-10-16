/**********************************************************************
  I2C_Menu.cpp

  Robots-For-All (R4A)
  Generic I2C menu support
**********************************************************************/

#include "R4A_I2C.h"

//****************************************
// I2C menu
//****************************************

const R4A_MENU_ENTRY r4aI2cMenuTable[] =
{
    // Command  menuRoutine             menuParam               HelpRoutine         align   HelpText
    {"e",       r4aI2cMenuEnumerate,    0,                      nullptr,            0,      "Enumerate the I2C bus"},                                   // 0
    {"r",       r4aI2cMenuRead,         (intptr_t)"Aa Rr",      r4aMenuHelpSuffix,  5,      "Read 1 byte from I2C device 0xaa register 0xrr"},          // 1
    {"w",       r4aI2cMenuWrite,        (intptr_t)"Aa Rr dd",   r4aMenuHelpSuffix,  8,      "Write 1 byte (0xdd) to I2C device 0xaa register 0xrr"},    // 2
    {"x",       nullptr,                R4A_MENU_MAIN,          nullptr,            0,      "Return to the main menu"},                                 // 3
};                                                                                                                                                      // 4
