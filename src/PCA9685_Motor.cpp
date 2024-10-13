/**********************************************************************
  PCA9685_Motor.cpp

  Robots-For-All (R4A)
  Motor support using the PCA9685
**********************************************************************/

#include "R4A_I2C.h"

//****************************************
// Constants
//****************************************

#define MOTOR_DEBUG_DISPLAY         0

#define MOTOR_REGS_PER_MOTOR        (R4A_PCA9685_REGS_PER_CHANNEL * MOTOR_CHANNELS_PER_MOTOR)

// The DRV8833 defines the following states
//
//      State       xIn1    xIn2
//      Coast        0       0
//      Backward     0       1
//      Forward      1       0
//      Brake        1       1
//

//****************************************
// Locals
//****************************************

uint8_t motorLowestChannel;

//*********************************************************************
// Brake the motor, apply plus to both channels.  When the brakes are
// not applied, the motor is coasting.
// Returns true if successful, false otherwise.  Always returns false for
// single channel operation.
bool R4A_PCA9685_MOTOR::brake(int16_t brakeValue, Print * display)
{
    // Buffer the brake values
    if (_dualChannel)
        return _pca9685->bufferLedOnOff(_plusChannel, brakeValue, display)
            && _pca9685->bufferLedOnOff(_minusChannel, brakeValue, display);
    else if (display)
        display->println("ERROR: Single channel motor unable to brake!");
    return false;
}

//*********************************************************************
// Buffers the coast request
// Returns true if successful, false otherwise.
bool R4A_PCA9685_MOTOR::coast(Print * display)
{
    return speed(0, display);
}

//*********************************************************************
// Display the motor state
void R4A_PCA9685_MOTOR::display(Print * display)
{
    uint16_t minusOnOff[2];
    uint32_t percentageX1000;
    uint16_t plusOnOff[2];

    // Read the PCA9685 plus channel registers
    if (_pca9685->readRegisters(_pca9685->channelToRegister(_plusChannel),
                                (uint8_t *)&plusOnOff[0],
                                sizeof(plusOnOff),
                                display) == sizeof(plusOnOff))
    {
        // Read the minus channel registers
        if (_pca9685->readRegisters(_pca9685->channelToRegister(_minusChannel),
                                    (uint8_t *)&minusOnOff[0],
                                    sizeof(minusOnOff),
                                    display) == sizeof(minusOnOff))
        {
            // Determine if the motor is coasting
            if ((plusOnOff[0] == 0) && (plusOnOff[1] == 4096)
                && (minusOnOff[0] == 0) && (minusOnOff[1] == 4096))
                display->printf("Coasting\r\n");

            // Determine if the motor is braking
            else if ((plusOnOff[0] == minusOnOff[0])
                && (plusOnOff[1] == minusOnOff[1]))
            {
                percentageX1000 = plusOnOff[1] ? plusOnOff[1] : plusOnOff[0];
                percentageX1000 = (100 * 1000 * percentageX1000) >> 12;
                display->printf("Braking applied %d, %3d.%03d %%\r\n",
                                plusOnOff[1],
                                percentageX1000 / 1000, percentageX1000 % 1000);
            }

            // Determine if the motor is moving forward
            else if ((minusOnOff[0] == 0) && (minusOnOff[1] == 4096))
            {
                percentageX1000 = plusOnOff[1] ? plusOnOff[1] : plusOnOff[0];
                percentageX1000 = (100 * 1000 * percentageX1000) >> 12;
                display->printf("Forward speed %d, %3d.%03d %%\r\n",
                                plusOnOff[1],
                                percentageX1000 / 1000, percentageX1000 % 1000);
            }

            // The motor is moving in reverse
            else
            {
                percentageX1000 = minusOnOff[1] ? minusOnOff[1] : minusOnOff[0];
                percentageX1000 = (100 * 1000 * percentageX1000) >> 12;
                display->printf("Reverse speed %d, %3d.%03d %%\r\n",
                                minusOnOff[1],
                                percentageX1000 / 1000, percentageX1000 % 1000);
            }
        }
    }
}

//*********************************************************************
// Brake the motor
// Start bit, I2C device address, ACK, register address, ACK, 8 data bytes
// with ACKs and a stop bit, all at 400 KHz
// 230 uSec = (1+8+1+8+1+((8+1)×8)+1)÷(400×1000)
// Returns true if successful, false otherwise
bool r4aPca9685MotorBrake(uint8_t motor, int16_t speed, Print * display)
{
    // Validate the motor value
    if (motor < r4aPca9685MotorTableEntries)
        // Set the braking value
        return r4aPca9685MotorTable[motor]->brake(speed, display)
            && r4aPca9685MotorTable[motor]->write(display);

    // Invalid motor value
    if (display)
        display->printf("ERROR: Invalid motor value: %d, range (0 - %d)!\r\n",
                        motor, r4aPca9685MotorTableEntries - 1);
    return false;
}

//*********************************************************************
// Brake all the motors
// Start bit, I2C device address, ACK, register address, ACK, 32 data bytes
// with ACKs and a stop bit, all at 400 KHz
// 770 uSec = (1+8+1+8+1+((8+1)×32)+1)÷(400×1000)
// Returns true if successful, false otherwise
bool r4aPca9685MotorBrakeAll(int16_t speed, Print * display)
{
    bool success;

    // Apply the brakes to all of the motors
    success = true;
    for (int motor = 0; motor < r4aPca9685MotorTableEntries; motor++)
        success &= r4aPca9685MotorTable[motor]->brake(speed, display);
    for (int motor = 0; motor < r4aPca9685MotorTableEntries; motor++)
        success &= r4aPca9685MotorTable[motor]->write(display);
    return success;
}

//*********************************************************************
// Place the motor in the coasting state
// Start bit, I2C device address, ACK, register address, ACK, 8 data bytes
// with ACKs and a stop bit, all at 400 KHz
// 230 uSec = (1+8+1+8+1+((8+1)×8)+1)÷(400×1000)
// Returns true if successful, false otherwise
bool r4aPca9685MotorCoast(uint8_t motor, Print * display)
{
    // Validate the motor value
    if (motor < r4aPca9685MotorTableEntries)
        // Set the coasting state
        return r4aPca9685MotorTable[motor]->coast(display)
            && r4aPca9685MotorTable[motor]->write(display);

    // Invalid motor value
    if (display)
        display->printf("ERROR: Invalid motor value: %d, range (0 - %d)!\r\n",
                        motor, r4aPca9685MotorTableEntries - 1);
    return false;
}

//*********************************************************************
// Place the motors in the coasting state
// Start bit, I2C device address, ACK, register address, ACK, 4 data bytes
// with ACKs and a stop bit, all at 400 KHz
// 410 uSec = (1+8+1+8+1+((8+1)×16)+1)÷(400×1000)
// Returns true if successful, false otherwise
bool r4aPca9685MotorCoastAll(Print * display)
{
    bool success;

    // Set all motors coasting
    success = true;
    for (int motor = 0; motor < r4aPca9685MotorTableEntries; motor++)
        success &= r4aPca9685MotorTable[motor]->speed(0, display);
    for (int motor = 0; motor < r4aPca9685MotorTableEntries; motor++)
        success &= r4aPca9685MotorTable[motor]->write(display);
    return success;
}

//*********************************************************************
// Display the motor state
void r4aPca9685MotorDisplayState(Print * display)
{
    display->println();
    display->println("Motor State");
    display->println("-----------");
    display->println();
    for (int motor = 0; motor < r4aPca9685MotorTableEntries; motor++)
    {
        display->printf("Motor %d: ", motor);
        r4aPca9685MotorTable[motor]->display(display);
    }
}

//*********************************************************************
// Drive the motor forward
// Start bit, I2C device address, ACK, register address, ACK, 8 data bytes
// with ACKs and a stop bit, all at 400 KHz
// 230 uSec = (1+8+1+8+1+((8+1)×8)+1)÷(400×1000)
// Returns true if successful, false otherwise
bool r4aPca9685MotorForward(uint8_t motor, int16_t speed, Print * display)
{
    // Validate the motor value
    if (motor < r4aPca9685MotorTableEntries)
        // Set the speed state
        return r4aPca9685MotorTable[motor]->speed(speed, display)
            && r4aPca9685MotorTable[motor]->write(display);

    // Invalid motor value
    if (display)
        display->printf("ERROR: Invalid motor value: %d, range (0 - %d)!\r\n",
                        motor, r4aPca9685MotorTableEntries - 1);
    return false;
}

//*********************************************************************
// Drive the motor reverse
// Start bit, I2C device address, ACK, register address, ACK, 8 data bytes
// with ACKs and a stop bit, all at 400 KHz
// 230 uSec = (1+8+1+8+1+((8+1)×8)+1)÷(400×1000)
// Returns true if successful, false otherwise
bool r4aPca9685MotorReverse(uint8_t motor, int16_t speed, Print * display)
{
    // Validate the motor value
    if (motor < r4aPca9685MotorTableEntries)
        // Set the reverse motor speed
        return r4aPca9685MotorTable[motor]->speed(-speed, display)
            && r4aPca9685MotorTable[motor]->write(display);

    // Invalid motor value
    if (display)
        display->printf("ERROR: Invalid motor value: %d, range (0 - %d)!\r\n",
                        motor, r4aPca9685MotorTableEntries - 1);
    return false;
}

//*********************************************************************
// Set the motor speed
// Start bit, I2C device address, ACK, register address, ACK, 8 data bytes
// with ACKs and a stop bit, all at 400 KHz
// 230 uSec = (1+8+1+8+1+((8+1)×8)+1)÷(400×1000)
// Returns true if successful, false otherwise
bool r4aPca9685MotorSetSpeed(uint8_t motor, int16_t speed, Print * display)
{
    // Validate the motor value
    if (motor < r4aPca9685MotorTableEntries)
        // Set the speed value
        return r4aPca9685MotorTable[motor]->speed(speed, display)
            && r4aPca9685MotorTable[motor]->write(display);

    // Invalid motor value
    if (display)
        display->printf("ERROR: Invalid motor value: %d, range (0 - %d)!\r\n",
                        motor, r4aPca9685MotorTableEntries - 1);
    return false;
}

//*********************************************************************
// Initialize the motors
bool r4aPca9685MotorSetup(Print * display)
{
    // Apply the brakes
    return r4aPca9685MotorBrakeAll(R4A_PCA9685_MOTOR_SPEED_MAX, display);
}


//*********************************************************************
// Get the minus channel associated with this motor
// Outputs:
//   Returns the minus channel number
uint8_t R4A_PCA9685_MOTOR::getMinusChannel()
{
    return _minusChannel;
}

//*********************************************************************
// Get the plus channel associated with this motor
// Outputs:
//   Returns the plus channel number
uint8_t R4A_PCA9685_MOTOR::getPlusChannel()
{
    return _plusChannel;
}

//*********************************************************************
// Buffers the motor speed value
// Returns true if successful, false otherwise.
bool R4A_PCA9685_MOTOR::speed(int16_t speedValue, Print * display)
{
    if (_dualChannel)
    {
        if (speedValue >= 0)
            return _pca9685->bufferLedOnOff(_plusChannel, speedValue, display)
                && _pca9685->bufferLedOnOff(_minusChannel, 0, display);
        else
            return _pca9685->bufferLedOnOff(_plusChannel, 0, display)
                && _pca9685->bufferLedOnOff(_minusChannel, -speedValue, display);
    }
    return _pca9685->bufferLedOnOff(_plusChannel, speedValue, display);
}

//*********************************************************************
// Writes the buffered values to the PCA9685.
// Returns true if successful, false otherwise.
bool R4A_PCA9685_MOTOR::write(Print * display)
{
    return _pca9685->writeBufferedRegisters(display);
}

//****************************************
// PCA9685 Motor Menu API
//****************************************

//*********************************************************************
// Display the help text with mm and ssss
void r4aPca9685MotorMenuHelpMm(const struct _R4A_MENU_ENTRY * menuEntry,
                               const char * align,
                               Print * display)
{
    display->printf("%s mm: %s%s\r\n",
                    menuEntry->command, align, menuEntry->helpText);
}

//*********************************************************************
// Display the help text with mm and ssss
void r4aPca9685MotorMenuHelpMmSsss(const struct _R4A_MENU_ENTRY * menuEntry,
                                   const char * align,
                                   Print * display)
{
    display->printf("%s mm ssss: %s%s\r\n",
                    menuEntry->command, align, menuEntry->helpText);
}

//*********************************************************************
// Get the motor number
bool r4aPca9685MotorMenuGetMotor(const R4A_MENU_ENTRY * menuEntry,
                                 const char * command,
                                 int * values,
                                 uint8_t * motor)
{
    int m;

    // Get the parameter name
    String line = String(&command[strlen(menuEntry->command)]);

    // Strip white space from the beginning of the name
    line.trim();

    // Get the value
    *values = sscanf(line.c_str(), "%d", &m);

    // Determine if the value is within range
    if ((*values == 1)
        && (m >= 0)
        && (m < r4aPca9685MotorTableEntries));
    {
        *motor = m;
        return true;
    }
    return false;
}

//*********************************************************************
// Get the motor number and speed value
bool r4aPca9685MotorMenuGetMotorSpeed(const R4A_MENU_ENTRY * menuEntry,
                                      const char * command,
                                      int * values,
                                      uint8_t * motor,
                                      uint16_t * speed)
{
    int m;
    int spd;

    // Get the parameter name
    String line = String(&command[strlen(menuEntry->command)]);

    // Strip white space from the beginning of the name
    line.trim();

    // Get the values
    *values = sscanf(line.c_str(), "%d %d", &m, &spd);

    // Determine if the values are within range
    if ((*values == 2)
        && (m >= 0)
        && (m < r4aPca9685MotorTableEntries)
        && (spd >= 0)
        && (spd <= R4A_PCA9685_MOTOR_SPEED_MAX))
    {
        *motor = m;
        *speed = spd;
        return true;
    }
    else if (*values == 1)
        *motor = m;
    return false;
}

//*********************************************************************
// Brake the motors
void r4aPca9685MotorMenuBrake(const R4A_MENU_ENTRY * menuEntry,
                              const char * command,
                              Print * display)
{
    uint8_t motor;
    uint16_t speed;
    int values;

    // Parse the command line
    if (r4aPca9685MotorMenuGetMotorSpeed(menuEntry, command, &values, &motor, &speed))
        r4aPca9685MotorTable[motor]->brake(speed);
    else if (values <= 0)
        display->printf("Please specify a motor (0 - %d) for mm\r\n", r4aPca9685MotorTableEntries - 1);
    else if (values == 1)
        display->printf("Please specify a speed (0 - %d) for ssss\r\n", R4A_PCA9685_MOTOR_SPEED_MAX);
    else if ((motor < 0) || (motor > r4aPca9685MotorTableEntries))
        display->printf("Invalid motor value, use (0 - %d)!\r\n", r4aPca9685MotorTableEntries - 1);
    else
        display->printf("Invalid speed value, use (0 - %d)!\r\n", R4A_PCA9685_MOTOR_SPEED_MAX);
}

//*********************************************************************
// Coast the motors
void r4aPca9685MotorMenuCoast(const R4A_MENU_ENTRY * menuEntry,
                              const char * command,
                              Print * display)
{
    uint8_t motor;
    int values;

    // Parse the command line
    if (r4aPca9685MotorMenuGetMotor(menuEntry, command, &values, &motor))
        r4aPca9685MotorTable[motor]->coast();
    else
        display->printf("Invalid motor value, use (0 - %d)!\r\n", r4aPca9685MotorTableEntries - 1);
}

//*********************************************************************
// Display the motors
void r4aPca9685MotorMenuDisplay(const R4A_MENU_ENTRY * menuEntry,
                          const char * command,
                          Print * display)
{
    // Display the motor states
    for (int motor = 0; motor < r4aPca9685MotorTableEntries; motor++)
        r4aPca9685MotorTable[motor]->display(display);
}

//*********************************************************************
// Drive the motors forward
void r4aPca9685MotorMenuForward(const R4A_MENU_ENTRY * menuEntry,
                                const char * command,
                                Print * display)
{
    uint8_t motor;
    uint16_t speed;
    int values;

    // Parse the command line
    if (r4aPca9685MotorMenuGetMotorSpeed(menuEntry, command, &values, &motor, &speed))
        r4aPca9685MotorForward(motor, speed);
    else if (values <= 0)
        display->printf("Please specify a motor (0 - %d) for mm\r\n", r4aPca9685MotorTableEntries - 1);
    else if (values == 1)
        display->printf("Please specify a speed (0 - %d) for ssss\r\n", R4A_PCA9685_MOTOR_SPEED_MAX);
    else if ((motor < 0) || (motor > r4aPca9685MotorTableEntries))
        display->printf("Invalid motor value, use (0 - %d)!\r\n", r4aPca9685MotorTableEntries - 1);
    else
        display->printf("Invalid speed value, use (0 - %d)!\r\n", R4A_PCA9685_MOTOR_SPEED_MAX);
}

//*********************************************************************
// Drive the motors in reverse
void r4aPca9685MotorMenuReverse(const R4A_MENU_ENTRY * menuEntry,
                                const char * command,
                                Print * display)
{
    uint8_t motor;
    uint16_t speed;
    int values;

    // Parse the command line
    if (r4aPca9685MotorMenuGetMotorSpeed(menuEntry, command, &values, &motor, &speed))
        r4aPca9685MotorReverse(motor, speed);
    else if (values <= 0)
        display->printf("Please specify a motor (0 - %d) for mm\r\n", r4aPca9685MotorTableEntries - 1);
    else if (values == 1)
        display->printf("Please specify a speed (0 - %d) for ssss\r\n", R4A_PCA9685_MOTOR_SPEED_MAX);
    else if ((motor < 0) || (motor > r4aPca9685MotorTableEntries))
        display->printf("Invalid motor value, use (0 - %d)!\r\n", r4aPca9685MotorTableEntries - 1);
    else
        display->printf("Invalid speed value, use (0 - %d)!\r\n", R4A_PCA9685_MOTOR_SPEED_MAX);
}

//*********************************************************************
// Stop the motors
void r4aPca9685MotorMenuStop(const R4A_MENU_ENTRY * menuEntry,
                                const char * command,
                                Print * display)
{
    r4aPca9685MotorBrakeAll(R4A_PCA9685_MOTOR_SPEED_MAX);
}
