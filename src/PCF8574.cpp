/**********************************************************************
  PCF8574.cpp

  Robots-For-All (R4A)
  I2C GPIO expander support
**********************************************************************/

#include "R4A_I2C.h"

//*********************************************************************
// Constructor
// Inputs:
//   i2cBus: Address of an R4A_I2C object
//   i2cAddress: Address of the PA9685 on the I2C bus
R4A_PCF8574::R4A_PCF8574(R4A_I2C_BUS * i2cBus, uint8_t i2cAddress)
    : _i2cBus{i2cBus}, _i2cAddress{i2cAddress}
{
}

//*********************************************************************
// Destructor
R4A_PCF8574::~R4A_PCF8574()
{
}

//*********************************************************************
// Read data from the PCF8574 port
// Returns true if the data byte was read successfully and false otherwise
bool R4A_PCF8574::read(uint8_t *data)
{
    return (_i2cBus->_read(_i2cBus,
                           _i2cAddress,
                           nullptr,
                           0,
                           data,
                           sizeof(*data),
                           nullptr,
                           true)
        == sizeof(*data));
}

//*********************************************************************
// Write data to the PCF8574 port
// Returns true if the data byte was successfully written and false otherwise
bool R4A_PCF8574::write(uint8_t data)
{
    return r4aI2cBusWrite(_i2cBus,
                          _i2cAddress,
                          nullptr,
                          0,
                          &data,
                          sizeof(data),
                          nullptr,
                          true);
}
