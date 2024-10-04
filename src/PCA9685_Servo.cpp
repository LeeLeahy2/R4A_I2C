/**********************************************************************
  PCA9685_Servo.cpp

  Robots-For-All (R4A)
  Servo support using the PCA9685
**********************************************************************/

#include "R4A_I2C.h"

//*********************************************************************
// Get the position of the servo
// Returns the position of the servo
uint8_t R4A_PCA9685_SERVO::positionGet(Print * display)
{
    uint16_t onTime[2];

    // Read the channel registers
    if (_pca9685->readRegisters(_pca9685->channelToRegister(_channel),
                                (uint8_t *)onTime,
                                sizeof(onTime),
                                display))
    {
        // Convert the onTime value to degrees
        return _pca9685->servoOnTicksToDegrees(onTime[1]);
    }

    display->printf("ERROR: Failed to read the PCA9685 registers for channel %d\r\n",
                    _channel);
    return 0;
}

//*********************************************************************
// Get the maximum position of the servo
// Returns the maximum position of the servo
uint8_t R4A_PCA9685_SERVO::positionMax()
{
    return _pca9685->servoOnTicksToDegrees(_pca9685->getMaximum(_channel));
}

//*********************************************************************
// Get the minimum position of the servo
// Returns the minimum position of the servo
uint8_t R4A_PCA9685_SERVO::positionMin()
{
    return _pca9685->servoOnTicksToDegrees(_pca9685->getMinimum(_channel));
}

//*********************************************************************
// Set the position of the servo
// Returns true if successful and false otherwise
bool R4A_PCA9685_SERVO::positionSet(uint8_t degrees, Print * display)
{
    return _pca9685->servoPosition(_channel, degrees, display);
}

//****************************************
// PCA9685 Servo Menu API
//****************************************

//*********************************************************************
// Get the degree value
bool r4aPca9685ServoGetDegrees(const R4A_MENU_ENTRY * menuEntry,
                               const char * command,
                               int * values,
                               uint8_t *servo,
                               uint8_t * degrees)
{
    int degs;
    int s;

    // Get the parameter name
    String line = String(&command[strlen(menuEntry->command)]);

    // Strip white space from the beginning of the name
    line.trim();

    // Get the servo number
    s = menuEntry->menuParameter;

    // Get the value
    *values = sscanf(line.c_str(), "%d", &degs);

    // Determine if the value is within range
    if ((*values == 1)
        && (s >= 0)
        && (s < r4aPca9685ServoTableEntries)
        && (degs >= r4aPca9685ServoTable[s]->_minimum)
        && (degs <= r4aPca9685ServoTable[s]->_maximum))
    {
        *servo = s;
        *degrees = degs;
        return true;
    }
    return false;
}

//*********************************************************************
// Get the servo and degree values
bool r4aPca9685ServoGetServoDegrees(const R4A_MENU_ENTRY * menuEntry,
                                    const char * command,
                                    int * values,
                                    uint8_t * servo,
                                    uint8_t * degrees)
{
    int degs;
    int s;

    // Get the parameter name
    String line = String(&command[strlen(menuEntry->command)]);

    // Strip white space from the beginning of the name
    line.trim();

    // Get the values
    *values = sscanf(line.c_str(), "%d %d", &s, &degs);

    // Determine if the values are within range
    if ((*values == 2)
        && (s >= 0)
        && (s < r4aPca9685ServoTableEntries)
        && (degs >= r4aPca9685ServoTable[s]->_minimum)
        && (degs <= r4aPca9685ServoTable[s]->_maximum))
    {
        *servo = s;
        *degrees = degs;
        return true;
    }
    else if (*values == 1)
        *servo = s;
    return false;
}

//*********************************************************************
// Display the help text with ddd
void r4aPca9685ServoMenuHelpDdd(const struct _R4A_MENU_ENTRY * menuEntry,
                                const char * align,
                                Print * display)
{
    display->printf("%s ddd: %s%s\r\n",
                    menuEntry->command, align, menuEntry->helpText);
}

//*********************************************************************
// Display the help text with s and ddd
void r4aPca9685ServoMenuHelpSDdd(const struct _R4A_MENU_ENTRY * menuEntry,
                                 const char * align,
                                 Print * display)
{
    display->printf("%s s ddd: %s%s\r\n",
                    menuEntry->command, align, menuEntry->helpText);
}

//*********************************************************************
// Display the servo states
void r4aPca9685ServoMenuDisplay(const R4A_MENU_ENTRY * menuEntry,
                                const char * command,
                                Print * display)
{
    display->println();
    display->println("Servo Positions");
    display->println("---------------");
    for (int servo = 0; servo < r4aPca9685ServoTableEntries; servo++)
        display->printf("%d:  %3d degrees\r\n", servo,
                        r4aPca9685ServoTable[servo]->positionGet());
}

//*********************************************************************
// Set the servo specified by the menu parameter to ddd degrees
void r4aPca9685ServoMenuMove(const R4A_MENU_ENTRY * menuEntry,
                             const char * command,
                             Print * display)
{
    uint8_t degrees;
    uint8_t servo;
    int values;

    if (r4aPca9685ServoGetDegrees(menuEntry, command, &values, &servo, &degrees))
        r4aPca9685ServoTable[servo]->positionSet(degrees);
}

//*********************************************************************
// Set servo s to ddd degrees
void r4aPca9685ServoMenuSet(const R4A_MENU_ENTRY * menuEntry,
                            const char * command,
                            Print * display)
{
    uint8_t degrees;
    uint8_t servo;
    int values;

    if (r4aPca9685ServoGetServoDegrees(menuEntry, command, &values, &servo, &degrees))
        r4aPca9685ServoTable[servo]->positionSet(degrees);
}
