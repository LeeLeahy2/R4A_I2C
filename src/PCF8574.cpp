/**********************************************************************
  PCF8574.cpp

  I2C GPIO expander support
**********************************************************************/

#include "R4A_I2C.h"

//*********************************************************************
// Read data from the PCF8574 port
// Returns true if the data byte was read successfully and false otherwise
bool R4A_PCF8574::read(uint8_t *data)
{
    return (_i2cBus->read(_i2cAddress, nullptr, 0, data, sizeof(*data))
        == sizeof(*data));
}

//*********************************************************************
// Write data to the PCF8574 port
// Returns true if the data byte was successfully written and false otherwise
bool R4A_PCF8574::write(uint8_t data)
{
    return _i2cBus->write(_i2cAddress, nullptr, 0, &data, sizeof(data));
}
