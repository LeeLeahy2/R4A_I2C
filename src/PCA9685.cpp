/**********************************************************************
  PCA9685.ino

  LED controller support
**********************************************************************/

#include "R4A_I2C.h"

//****************************************
// Constants
//****************************************

#define PCA9685_DEBUG_ON_OFF_TIMES      0
#define PCA9685_DEBUG_DISPLAY           0

#define PCA9685_REG_LED0_ON_L           6

// Compute the shifted number of ticks in 10% of the interval
static const double tenPercentTicks = ((double)(4096. / 10.));

// Compute the shifted number of ticks in 2.5% of the interval
static const double twoPointFivePercentTicks = ((double)(tenPercentTicks / 4.));

//****************************************
// Macros
//****************************************

//                      1          4096 ticks     1 cycle
//      degrees * ------------- * ------------ * --------- = ticks
//                 180 degrees       cycle           10
//
#define RCA_PCA9685_TICKS(degrees)  round(((double)degrees * (4096. / (10. * 180.))) + twoPointFivePercentTicks)
#define RCA_PCA9685_DEGREES(ticks)  round(((double)ticks - twoPointFivePercentTicks) * (180. / tenPercentTicks))

//****************************************
// Data types
//****************************************

union PCA9685_Mode1Register
{
    struct
    {
        uint8_t allcall : 1;
        uint8_t sub3 : 1;
        uint8_t sub2 : 1;
        uint8_t sub1 : 1;
        uint8_t sleep : 1;
        uint8_t ai : 1;
        uint8_t extclk : 1;
        uint8_t restart : 1;
    } fields;
    uint8_t data;
};

union PCA9685_Mode2Register
{
    struct
    {
        uint8_t outne : 2;
        uint8_t outdrv : 1;
        uint8_t och : 1;
        uint8_t invrt : 1;
        uint8_t space : 3;
    } fields;
    uint8_t data;
};

//****************************************
// Macros
//****************************************

// Convert from uSec to ticks for LEDn_ON / LEDn_OFF values
#define PCA9685_USEC_TO_TICKS(uSec) \
(((((uint64_t)uSec) * (uint64_t)_clockHz * 4095ull) + (500ull * 1000ull)) / (1ull * 1000ull * 1000ull))

// Convert from ticks for LEDn_ON / LEDn_OFF values to uSec
#define PCA9685_TICKS_TO_USEC(ticks) \
(((((uint64_t)ticks) * 1000ull * 1000ull) + ((uint64_t)_clockHz * 4096ull / 2ull)) / ((uint64_t) _clockHz * 4096ull))

//*********************************************************************
// Initialize the PCA9685 LED controller
R4A_PCA9685::R4A_PCA9685(R4A_I2C_BUS * i2cBus,
                         uint8_t i2cAddress,
                         uint32_t scanClockHertz,
                         uint32_t externalClockHertz)
{
    _i2cBus = i2cBus;
    _i2cAddress = i2cAddress;
    _clockHz = scanClockHertz;
    _externalClockHz = externalClockHertz;
}

//*********************************************************************
// Initialize the LED controller
// Return true if successful, false otherwise
bool R4A_PCA9685::begin()
{
    PCA9685_Mode1Register mode1;
    uint8_t prescale;
    bool success = false;
    int value;

    do
    {
        // Assume all channels are driving LEDs
        for (int channel = 0; channel < R4A_PCA9685_CHANNEL_COUNT; channel++)
        {
            _max[channel] = 4096;
            _min[channel] = 0;
        }

        // Map the frequency to a value that fits into the PCA9685 prescaler
        value = _clockHz << 12;
        value = ((_externalClockHz + (value >> 1)) / value) - 1;
        if (value < 3)
            value = 3;
        else if (value > 255)
            value = 255;
        prescale = value;

        // Set the prescale value while the device is sleeping
        if (!writeRegisters(254,
                            &prescale,
                            sizeof(prescale)))
        {
            Serial.println("ERROR: Failed to set the PCA9685 prescale value!");
            break;
        }

        // Wake the PCA9685
        if (readRegisters(0,
                          &mode1.data,
                          sizeof(mode1.data)) != 1)
        {
            Serial.println("ERROR: Failed to read PCA9685 mode 0 register!");
            break;
        }
        mode1.fields.sleep = 0; // Wake the PCA9685
        mode1.fields.ai = 1;    // Enable auto increment
        if (!writeRegisters(0,
                            &mode1.data,
                            sizeof(mode1.data)))
        {
            Serial.println("ERROR: Failed to write PCA9685 mode 0 register!");
            break;
        }

        // Determine if the PCA9685 is restarting
        if (mode1.fields.restart)
        {
            delay(1);
            mode1.fields.restart = 1;  // Clear the restart
            if (!writeRegisters(0,
                                &mode1.data,
                                sizeof(mode1.data)))
            {
                Serial.println("ERROR: Failed to clear PCA9685 restart!");
                break;
            }
        }

        // Successful initialization
        success = true;
    } while (0);

    return success;
}

//*********************************************************************
// Buffer a copy of the LED on and off times which will be written to the
// PCA9685 at a later time
bool R4A_PCA9685::bufferLedOnOff(uint8_t channel,
                                 uint16_t onTime,
                                 Print * display)
{
    uint16_t onOff[2];

    // Validate the channel
    if (channel >= R4A_PCA9685_CHANNEL_COUNT)
    {
        display->println("ERROR: Invalid channel number, use (0 - 15)!\r\n");
        return false;
    }

    // Validate the minimum and maximum
    if ((onTime < _min[channel]) || (onTime > _max[channel]))
    {
        display->printf("ERROR: onTime (%d) is invalid, range (%d - %d)!\r\n",
                        onTime, _min[channel], _max[channel]);
        return false;
    }

    // Is LED on all the time?
    if (onTime >= 4096)
    {
        onOff[0] = 0x1000;
        onOff[1] = 0;
    }

    // Is LED off all the time?
    else if (onTime == 0)
    {
        onOff[0] = 0;
        onOff[1] = 0x1000;
    }

    // Use PWM to determine LED value
    else
    {
        onOff[0] = 0;
        onOff[1] = onTime;
    }

    // onTime (0 - 4095) = time to off-->on edge
    // offTime (0 - 4095) = time to on-->off edge
    //
    // Display the on and off times
    if (PCA9685_DEBUG_ON_OFF_TIMES)
    {
        display->printf("LED%d_ON_L: 0x%02x\r\n", channel, onOff[0] & 0xff);
        display->printf("LED%d_ON_H: 0x%02x\r\n", channel, onOff[0] >> 8);
        display->printf("LED%d_OFF_L: 0x%02x\r\n", channel, onOff[1] & 0xff);
        display->printf("LED%d_OFF_H: 0x%02x\r\n", channel, onOff[1] >> 8);
    }

    // Update the local value of the registers
    memcpy(&_channelRegs[channel << 2], (uint8_t *)onOff, sizeof(onOff));
    return true;
}

//*********************************************************************
// Buffer a copy of the servo position will be written to the PCA9685 at
// a later time
bool R4A_PCA9685::bufferServoPosition(uint8_t channel,
                                      uint8_t degrees,
                                      Print * display)
{
    return bufferLedOnOff(channel, servoDegreesToOnTicks(degrees), display);
}

//*********************************************************************
// Convert channel number into a PCA9685 register address
uint8_t R4A_PCA9685::channelToRegister(uint8_t channel)
{
    return (channel << 2) + PCA9685_REG_LED0_ON_L;
}

//*********************************************************************
// Copy the buffered register data into another buffer
void R4A_PCA9685::copyBufferedRegisters(uint8_t * destBuffer)
{
    memcpy(destBuffer, _channelRegs, sizeof(_channelRegs));
}

//*********************************************************************
// Display mode 1 register
void R4A_PCA9685::displayMode1(Print * display)
{
    PCA9685_Mode1Register mode1;

    // Read the mode 1 register
    if (readRegisters(0,
                      &mode1.data,
                      sizeof(mode1.data),
                      display) != 1)
        display->println("ERROR: Failed to read mode 1 register!");
    else
    {
        // Display the mode 1 register
        display->printf("0x%02x: mode 1", mode1.data);
        if (mode1.data)
            display->print(":");
        if (mode1.fields.restart)
            display->print(" RESTART");
        if (mode1.fields.extclk)
            display->print(" EXTCLK");
        if (mode1.fields.ai)
            display->print(" AI");
        if (mode1.fields.sleep)
            display->print(" SLEEP");
        if (mode1.fields.sub1)
            display->print(" SUB1");
        if (mode1.fields.sub2)
            display->print(" SUB2");
        if (mode1.fields.sub3)
            display->print(" SUB3");
        if (mode1.fields.allcall)
            display->print(" ALLCALL");
        display->println();
    }
}

//*********************************************************************
// Display mode 2 register
void R4A_PCA9685::displayMode2(Print * display)
{
    PCA9685_Mode2Register mode2;

    // Read the mode 2 register
    if (readRegisters(1,
                      &mode2.data,
                      sizeof(mode2.data),
                      display) != 1)
        display->println("ERROR: Failed to read mode 2 register!");
    else
    {
        // Display the mode 2 register
        display->printf("0x%02x: mode 2", mode2.data);
        if (mode2.data)
            display->print(":");
        if (mode2.fields.invrt)
            display->print(" INVRT");
        display->printf(" Change on %s", mode2.fields.och ? "ACK" : "STOP");
        if (mode2.fields.outdrv)
            display->print(", Totem pole");
        else
            display->print(", Open collector");
        display->print(", LEDn=");
        if (mode2.fields.outne == 0)
            display->print("0");
        else if (mode2.fields.outne == 1)
            display->print("1");
        else
            display->print("Off");
        display->println();
    }
}

//*********************************************************************
// Display the current state of the PCA9685 channel
void R4A_PCA9685::displayLedOnOff(uint8_t channel,
                                  Print * display)
{
    int degrees;
    int degreesMax;
    int degreesMin;
    uint16_t onOff[2];
    uint8_t firstRegister;
    int uSecDelay;
    int uSecOn;

    // Validate the channel
    if (channel < R4A_PCA9685_CHANNEL_COUNT)
    {
        // Read the PCA9685 LEDn_ON and LEDn_OFF registers
        firstRegister = channelToRegister(channel);
        if (PCA9685_DEBUG_DISPLAY)
            display->printf("firstRegister: %d\r\n", firstRegister);
        if (readRegisters(firstRegister,
                          (uint8_t *)&onOff[0],
                          sizeof(onOff),
                          display) == sizeof(onOff))
        {
            if (PCA9685_DEBUG_DISPLAY)
                display->printf("onOff[0]: 0x%04x (%d), onOff[1]: 0x%04x (%d)\r\n",
                                onOff[0], onOff[0], onOff[1], onOff[1]);
            if (onOff[1] == 4096)
                display->printf("    CH%02d: Off\r\n", channel);
            else if (onOff[0] == 4096)
                display->printf("    CH%02d: On\r\n", channel);
            else
            {
                // Display the delay and on time
                uSecDelay = PCA9685_TICKS_TO_USEC(onOff[0] & 0x1fff);
                uSecOn = PCA9685_TICKS_TO_USEC(onOff[1] & 0x1fff);
                if (PCA9685_DEBUG_DISPLAY)
                {
                    display->printf("uSecDelay: %d\r\n", uSecDelay);
                    display->printf("uSecOn: %d\r\n", uSecOn);
                }
                display->printf("    CH%02d: 0x%04x 0x%04x, Delay: %3d.%03d mSec, On: %3d.%03d mSec",
                                channel,
                                onOff[0],
                                onOff[1],
                                uSecDelay / 1000, uSecDelay % 1000,
                                uSecOn / 1000, uSecOn % 1000);

                // If not on all the time and value is with servo range,
                // display the degrees
                if (PCA9685_DEBUG_DISPLAY)
                {
                    display->println();
                    display->printf("min[%d]: %d\r\n", channel, _min[channel]);
                    display->printf("max[%d]: %d\r\n", channel, _max[channel]);
                }
                if ((onOff[1] >= _min[channel])
                    && (onOff[1] <= _max[channel]))
                {
                    // Convert from ticks to degrees
                    degreesMin = servoOnTicksToDegrees(_min[channel]);
                    degreesMax = servoOnTicksToDegrees(_max[channel]);
                    degrees = servoOnTicksToDegrees(onOff[1]);
                    display->printf(", %3d degrees (%d - %d)",
                                    degrees, degreesMin, degreesMax);
                }
                display->println();
            }
        }
    }
}

//*********************************************************************
// Display the PCA9685 mode registers
void R4A_PCA9685::displayRegisters(Print * display)
{
    display->println();
    display->println("PCA9685 Mode Registers");
    display->println("----------------------");
    display->println();
    displayMode1(display);
    displayMode2(display);
    for (uint8_t channel = 0; channel <= R4A_PCA9685_CHANNEL_COUNT; channel++)
        displayLedOnOff(channel, display);
}

//*********************************************************************
// Dump all of the PCA9685 registers in hexadecimal
void R4A_PCA9685::dumpRegisters(Print * display)
{
    size_t bytesToRead;
    uint8_t data[32];
    int offset;

    do
    {
        // Display the registers
        display->println("PCA9685 Registers");
        display->println("-----------------");
        display->println();

        // Read the PCA9685 registers 0 - 69
        offset = 0;
        while (offset < 70)
        {
            // Determine how many butes to read
            bytesToRead = 70 - offset;
            if (bytesToRead > sizeof(data))
                bytesToRead = sizeof(data);

            // Read the bytes from the PCA9685
            if (readRegisters(offset,
                              data,
                              bytesToRead,
                              display) != bytesToRead)
            {
                display->printf("ERROR: Failed to read registers %d - %d!\r\n",
                                offset, offset + bytesToRead - 1);
                break;
            }

            // Display the bytes
            r4aDumpBuffer(offset, data, bytesToRead, display);
            offset += bytesToRead;
        }

        // Exit early if an error occurred
        if (offset != 70)
            break;

        // Read the PCA9685 registers 250 - 255
        if (readRegisters(250,
                          data,
                          6,
                          display) != 6)
        {
            display->println("ERROR: Failed to read registers 250 - 255!");
            break;
        }
        r4aDumpBuffer(250, data, 6, display);
    } while (0);
}

//*********************************************************************
// Set the LED on and off times
bool R4A_PCA9685::ledOnOff(uint8_t channel,
                           uint16_t onTime,
                           Print * display,
                           bool debug)
{
    uint8_t firstRegister;

    if (!bufferLedOnOff(channel, onTime, display))
        return false;

    // Set the on/off times for the LED
    firstRegister = channelToRegister(channel);
    return writeBufferedRegisters(firstRegister,
                                  R4A_PCA9685_REGS_PER_CHANNEL,
                                  display,
                                  debug);
}

//*********************************************************************
// Read one or more PCA9685 registers
// Return the number of bytes read
size_t R4A_PCA9685::readRegisters(uint8_t firstRegisterAddress,
                                  uint8_t * dataBuffer,
                                  size_t dataByteCount,
                                  Print * display,
                                  bool debug)
{
    size_t bytesRead;

    // Read the data from the PCA9685
    bytesRead = _i2cBus->read(_i2cAddress,
                              &firstRegisterAddress,
                              sizeof(firstRegisterAddress),
                              dataBuffer,
                              dataByteCount,
                              debug);

    // Display the final results
    if (debug)
        Serial.printf("PCA9685 0x%02x --> %d bytes\r\n", firstRegisterAddress, bytesRead);

    // Return the number of bytes read
    return bytesRead;
}

//*********************************************************************
// Convert from degrees (0 - 180) to onTicks for servo positioning
uint16_t R4A_PCA9685::servoDegreesToOnTicks(uint8_t degrees)
{
    uint16_t onTicks;

    // Typically servos are provided a pulse every 20 mSec.  The minimum
    // pulse width is 2.5% or 0.5 milliseconds.  The maximum pulse width
    // is 12.5% or 2.5 milliseconds.  The pulse width varies from 0.5 to
    // 2.5 milliseconds depending on the servo position.
    //
    //            ticks      cycles         Sec             ticks
    //          -------- * --------- * -------------- =  -------------
    //            cycle       Sec       milliseconds      millisecond
    //
    //
    //               ticks                   millisecond
    //           -------------- * degrees * ------------- = ticks
    //            milliseconds                 degrees
    //

    // Convert ticks to degrees
    onTicks = RCA_PCA9685_TICKS(degrees);
/*
Serial.printf("degrees: %d\r\n", degrees);
double value;
    value = (double)degrees;
Serial.printf("degrees: %f\r\n");
    value *= 4096 / (10. * 180.);
    value += twoPointFivePercentTicks;
Serial.printf("degrees: %f\r\n");
//#define RCA_PCA9685_TICKS(degrees)  round(((double)degrees * (4096. / 10. * 180.)) + twoPointFivePercentTicks)
    onTicks = (uint16_t)value;
Serial.printf("onTicks: %d\r\n", onTicks);
*/
    return onTicks;
}

//*********************************************************************
// Convert from onTicks for servo positioning to degrees (0 - 180)
uint8_t R4A_PCA9685::servoOnTicksToDegrees(uint16_t onTicks)
{
    uint8_t degrees;

    // Typically servos are provided a pulse every 20 mSec.  The minimum
    // pulse width is 2.5% or 0.5 milliseconds.  The maximum pulse width
    // is 12.5% or 2.5 milliseconds.  The pulse width varies from 0.5 to
    // 2.5 milliseconds depending on the servo position.
    //
    //              onTicks
    //            ------------ * degrees = degrees
    //             totalTicks
    //
    // Convert from ticks to degreesShift the onTicks to keep the fractional digits
    degrees = RCA_PCA9685_DEGREES(onTicks);
    return degrees;
}

//*********************************************************************
// Set the servo position
bool R4A_PCA9685::servoPosition(uint8_t channel,
                                uint8_t degrees,
                                Print * display,
                                bool debug)
{
    uint8_t firstRegister;

    if (!bufferServoPosition(channel, degrees, display))
        return false;

    // Set the on/off times for the LED
    firstRegister = channelToRegister(channel);
    return writeRegisters(firstRegister,
                          &_channelRegs[channel << 2],
                          R4A_PCA9685_REGS_PER_CHANNEL,
                          display,
                          debug);
}

//*********************************************************************
// Write the buffered register data to the PCB9685 registers
// Returns true if successful, false otherwise
bool R4A_PCA9685::writeBufferedRegisters(uint8_t firstChannel,
                                         size_t dataByteCount,
                                         Print * display,
                                         bool debug)
{
    uint8_t firstRegisterAddress;

    // Display the transaction for debugging
    firstRegisterAddress = channelToRegister(firstChannel);
    if (debug)
        Serial.printf("PCA9685 0x%02x <-- %d bytes\r\n", firstRegisterAddress, dataByteCount);

    // Write the PCA9685 registers
    return _i2cBus->write(_i2cAddress,
                          &firstRegisterAddress,
                          sizeof(firstRegisterAddress),
                          &_channelRegs[firstChannel << 2],
                          dataByteCount,
                          debug);
}

//*********************************************************************
// Write data to the PCB9685 registers
// Return true if successful, false otherwise
bool R4A_PCA9685::writeRegisters(uint8_t firstRegisterAddress,
                                 uint8_t * dataBuffer,
                                 size_t dataByteCount,
                                 Print * display,
                                 bool debug)
{
    // Display the transaction for debugging
    if (debug)
        Serial.printf("PCA9685 0x%02x <-- %d bytes\r\n", firstRegisterAddress, dataByteCount);

    // Write the PCA9685 registers
    return _i2cBus->write(_i2cAddress,
                          &firstRegisterAddress,
                          sizeof(firstRegisterAddress),
                          dataBuffer,
                          dataByteCount,
                          debug);
}
