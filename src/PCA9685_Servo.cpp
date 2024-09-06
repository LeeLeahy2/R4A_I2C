/**********************************************************************
  Servo.ino

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
