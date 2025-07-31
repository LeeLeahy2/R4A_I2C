/**********************************************************************
  R4A_UBLOX_I2C.cpp

  Support for the u-blox I2C driver
**********************************************************************/

/*
  An Arduino Library which allows you to communicate seamlessly with u-blox GNSS modules using the Configuration Interface

  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/15136
  https://www.sparkfun.com/products/16481
  https://www.sparkfun.com/products/16344
  https://www.sparkfun.com/products/18037
  https://www.sparkfun.com/products/18719
  https://www.sparkfun.com/products/18774
  https://www.sparkfun.com/products/19663
  https://www.sparkfun.com/products/17722

  Original version by Nathan Seidle @ SparkFun Electronics, September 6th, 2018
  v2.0 rework by Paul Clark @ SparkFun Electronics, December 31st, 2020
  v3.0 rework by Paul Clark @ SparkFun Electronics, December 8th, 2022

  https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3

  This library is an updated version of the popular SparkFun u-blox GNSS Arduino Library.
  v3 uses the u-blox Configuration Interface (VALSET and VALGET) to:
  detect the module (during begin); configure message intervals; configure the base location; etc..

  This version of the library will not work with older GNSS modules.
  It is specifically written for newer modules like the ZED-F9P, ZED-F9R and MAX-M10S.
  For older modules, please use v2 of the library: https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.19

  SparkFun code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).
  The MIT License (MIT)
  Copyright (c) 2018 SparkFun Electronics
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
  associated documentation files (the "Software"), to deal in the Software without restriction,
  including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the Software is furnished to
  do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial
  portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
  NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <R4A_I2C.h>

//*********************************************************************
// Constructor
R4A_UBLOX_I2C::R4A_UBLOX_I2C(R4A_I2C_BUS * i2cBus,
                             R4A_I2C_ADDRESS_t i2cAddress)
        : _i2cBus{i2cBus}, _i2cAddress(i2cAddress)
{
}

//*********************************************************************
// Checks how many bytes are waiting in the GNSS's I2C buffer
uint16_t R4A_UBLOX_I2C::available()
{
    uint8_t buffer[2];
    uint16_t bytesAvailable;
    uint8_t registerAddress;

    if (!_i2cBus)
        return false;

    // Get the number of bytes available from the module
    bytesAvailable = 0;
    registerAddress = 0xfd;
    if (r4aI2cBusWriteRead(_i2cBus,
                           _i2cAddress,
                           &registerAddress,
                           sizeof(registerAddress),
                           buffer,
                           sizeof(buffer)) == false)
        return (0); // Sensor did not ACK

    uint8_t msb = buffer[0];
    uint8_t lsb = buffer[1];
    bytesAvailable = (uint16_t)msb << 8 | lsb;
    return bytesAvailable;
}

//*********************************************************************
// Determine if the GNSS device is connected to the I2C bus
bool R4A_UBLOX_I2C::ping()
{
    bool present;
    TwoWire * twoWire;

    if (!_i2cBus)
        return false;

    present = r4aI2cBusEnumerateDevice(_i2cBus, _i2cAddress);
    return present;
}

//*********************************************************************
// Read data from the GNSS device
uint8_t R4A_UBLOX_I2C::readBytes(uint8_t *data, uint8_t length)
{
    size_t bytesReturned;

    bytesReturned = 0;
    if (_i2cBus && length)
        r4aI2cBusRead(_i2cBus, _i2cAddress, data, length, &bytesReturned);
    return (uint8_t)bytesReturned;
}

//*********************************************************************
// Write data to the GNSS device
uint8_t R4A_UBLOX_I2C::writeBytes(uint8_t *data, uint8_t length)
{
    uint8_t bytesWritten;
    TwoWire * twoWire;

    if (_i2cBus && length)
    {
        if (r4aI2cBusWrite(_i2cBus, _i2cAddress, data, length))
            return length;
    }
    return 0;
}

//****************************************
// Unused functions required by the GNSSDeviceBus class
//****************************************

//*********************************************************************
void R4A_UBLOX_I2C::startWriteReadByte()
{
};

//*********************************************************************
uint8_t R4A_UBLOX_I2C::writeReadBytes(const uint8_t *data,
                       uint8_t *readData,
                       uint8_t length)
{
    // Reference the parameters
    (void)data;
    (void)readData;
    (void)length;

    // Return the number of bytes read
    return 0;
}

//*********************************************************************
void R4A_UBLOX_I2C::writeReadByte(const uint8_t *data, uint8_t *readData)
{
    // Reference the parameters
    (void)data;
    (void)readData;
}

//*********************************************************************
void R4A_UBLOX_I2C::writeReadByte(const uint8_t data, uint8_t *readData)
{
    // Reference the parameters
    (void)data;
    (void)readData;
}

//*********************************************************************
void R4A_UBLOX_I2C::endWriteReadByte()
{
};
