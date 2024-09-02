/**********************************************************************
  I2C.cpp

  Generic I2C support
**********************************************************************/

#include "R4A_I2C.h"

//*********************************************************************
// Enumerate the I2C bus
void R4A_I2C_BUS::enumerate(Print * display)
{
    bool deviceFound;
    int index;
    uint32_t timer;

    // Display the device addresses
    deviceFound = false;
    for (uint8_t addr = 0; addr <= 0x7f; addr++)
    {
        timer = millis();
        if (isDevicePresent(addr))
        {
            if (deviceFound == false)
            {
                display->println();
                display->println("I2C Devices:");
                deviceFound = true;
            }

            // Look up the display name
            for (index = 0; index < _deviceTableEntries; index++)
                if (_deviceTable && (_deviceTable[index].deviceAddress == addr))
                {
                    deviceFound = true;
                    break;
                }

            if (index < _deviceTableEntries)
                display->printf("    0x%02x: %s\r\n", addr, _deviceTable[index].displayName);
            else if (addr == 0)
                display->printf("    0x%02x: General Call\r\n", addr);
            else
                display->printf("    0x%02x: ???\r\n", addr);
        }
        else if ((millis() - timer) > 50)
        {
            display->println("ERROR: I2C bus not responding!");
            return;
        }
    }

    // Determine if any devices are on the bus
    if (!deviceFound)
        display->println("ERROR: No devices found on the I2C bus!");
}

//*********************************************************************
// Ping an I2C device and see if it responds
// Return true if device detected, false otherwise
bool R4A_I2C_BUS::isDevicePresent(uint8_t deviceAddress)
{
    int status;

    // Single thread the I2C requests
    r4aLockAcquire(&_lock);

    // Check for an I2C device
    _i2cBus->beginTransmission(deviceAddress);
    status = _i2cBus->endTransmission();

    // Release the lock
    r4aLockRelease(&_lock);

    // Return the I2C device found status
    if (status == 0)
        return true;
    return false;
}

//*********************************************************************
// Send data to an I2C peripheral
// Return true upon success, false otherwise
bool R4A_I2C_BUS::write(uint8_t deviceI2cAddress,
                        const uint8_t * cmdBuffer,
                        size_t cmdByteCount,
                        const uint8_t * dataBuffer,
                        size_t dataByteCount,
                        bool debug,
                        bool releaseI2cBus,
                        Print * display)
{
    bool status;

    // Single thread the I2C requests
    r4aLockAcquire(&_lock);

    // Perform the I2C write operation
    status = writeWithLock(deviceI2cAddress,
                           cmdBuffer,
                           cmdByteCount,
                           dataBuffer,
                           dataByteCount,
                           debug,
                           releaseI2cBus,
                           display);

    // Release the lock
    r4aLockRelease(&_lock);

    // Return the write status
    return status;
}
