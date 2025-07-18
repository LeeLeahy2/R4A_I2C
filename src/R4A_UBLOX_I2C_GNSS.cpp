/**********************************************************************
  R4A_UBLOX_I2C_GNSS.cpp

  Support for the u-blox GNSS I2C device
**********************************************************************/

#include <R4A_I2C.h>

//*********************************************************************
// Constructor
R4A_UBLOX_I2C_GNSS::R4A_UBLOX_I2C_GNSS(R4A_I2C_BUS * i2cBus,
                                       R4A_I2C_ADDRESS_t i2cAddress)
    : _ubloxI2cBus{R4A_UBLOX_I2C(i2cBus, i2cAddress)}
{
    _commType = COMM_TYPE_I2C;
    setCommunicationBus(_ubloxI2cBus);
}

//*********************************************************************
// This method is called to initialize the SFE_UBLOX_GNSS library.  This
// method must be called before calling any other method that interacts
// with the device.
bool R4A_UBLOX_I2C_GNSS::begin(uint16_t maxWait, bool assumeSuccess)
{
    // Initialize the system - return results
    return this->DevUBLOXGNSS::init(maxWait, assumeSuccess);
}
