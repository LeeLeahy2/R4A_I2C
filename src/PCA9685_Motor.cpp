/**********************************************************************
  PCA9685_Motor.cpp

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
