/**********************************************************************
  R4A_UBLOX_I2C.cpp

  Support for the u-blox I2C driver
**********************************************************************/

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
    uint16_t bytesAvailable;
    uint8_t data[2];
    uint8_t registerNumber;

    do
    {
        bytesAvailable = 0;

        // Verify that the R4A_UBLOX_I2C bus was initialized.
        if (!_i2cBus)
            break;

        // Read registers 0xFD(msb) and 0xFE(lsb) to get the available bytes
        //
        // From the u-blox integration manual:
        // "There are two forms of DDC read transfer. The "random access"
        // form includes a peripheral register address and thus allows any
        // register to be read. The second "current address" form omits the
        // register address. If this second form is used, then an address
        // pointer in the receiver is used to determine which register to
        // read. This address pointer will increment after each read unless
        // it is already pointing at register 0xFF, the highest addressable
        // register, in which case it remains unaltered."
        // Set the register address
        registerNumber = 0xfd;
        if (writeBytes(&registerNumber, (uint8_t)sizeof(registerNumber)) == 0)
            break;

        // Read the number of bytes available
        if (readBytes(data, (uint8_t)sizeof(data)) == 0)
            break;

        // Swap the data bytes
        bytesAvailable = (((uint16_t)data[0]) << 8) | data[1];
    } while (0);

    // Return the number of GNSS bytes available to be read
    return bytesAvailable;
}

//*********************************************************************
// Determine if the GNSS device is connected to the I2C bus
bool R4A_UBLOX_I2C::ping()
{
    return r4aI2cBusIsDevicePresent(_i2cBus, _i2cAddress);
}

//*********************************************************************
// Read data from the GNSS device
uint8_t R4A_UBLOX_I2C::readBytes(uint8_t *data, uint8_t length)
{
    // Read the data from the GNSS device
    bool success = r4aI2cBusRead(_i2cBus,
                                 _i2cAddress,
                                 data,
                                 length);
    return success ? length : 0;
}

//*********************************************************************
// Write data to the GNSS device
uint8_t R4A_UBLOX_I2C::writeBytes(uint8_t *data, uint8_t length)
{
    // Write data to the GNSS device
    bool success = r4aI2cBusWrite(_i2cBus,
                                  _i2cAddress,
                                  data,
                                  length);
    return success ? length : 0;
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
