/**********************************************************************
  I2C.cpp

  Robots-For-All (R4A)
  Generic I2C support
**********************************************************************/

#include "R4A_I2C.h"

//*********************************************************************
// Enumerate the I2C bus
void r4aI2cBusEnumerate(R4A_I2C_BUS * i2cBus, Print * display)
{
    bool deviceFound;
    int index;
    uint8_t mask;
    bool present;
    uint32_t timer;

    // Walk all of the I2C addresses
    deviceFound = false;
    for (R4A_I2C_ADDRESS_t addr = 0; addr < R4A_I2C_ADDRESSES; addr++)
    {
        present = false;
        timer = millis();
        if (r4aI2cBusEnumerateDevice(i2cBus, addr))
        {
            present = true;
            if (deviceFound == false)
            {
                if (display)
                {
                    display->println();
                    display->println("I2C Devices:");
                }
                deviceFound = true;
            }

            // Look up the display name
            for (index = 0; index < i2cBus->_deviceTableEntries; index++)
                if (i2cBus->_deviceTable && (i2cBus->_deviceTable[index].i2cAddress == addr))
                {
                    deviceFound = true;
                    break;
                }

            if (display)
            {
                if (index < i2cBus->_deviceTableEntries)
                    display->printf("    0x%03x: %s\r\n", addr, i2cBus->_deviceTable[index].displayName);
                else if (addr == 0)
                    display->printf("    0x%03x: General Call\r\n", addr);
                else
                    display->printf("    0x%03x: ???\r\n", addr);
            }
        }
        else if ((millis() - timer) > 50)
        {
            if (display)
                display->println("ERROR: I2C bus not responding!");
            return;
        }

        // Update the present bit
        mask = 1 << (addr & 7);
        if (present)
            i2cBus->_present[addr / 8] |= mask;
        else
            i2cBus->_present[addr / 8] &= ~mask;
    }

    // Successful enumeration
    i2cBus->_enumerated = true;

    // Determine if any devices are on the bus
    if ((!deviceFound) && display)
        display->println("ERROR: No devices found on the I2C bus!");
}

//*********************************************************************
// Ping an I2C device and see if it responds
// Return true if device detected, false otherwise
bool r4aI2cBusEnumerateDevice(R4A_I2C_BUS * i2cBus, R4A_I2C_ADDRESS_t i2cAddress)
{
    int status;

    // Single thread the I2C requests
    r4aLockAcquire(&i2cBus->_lock);

    // Check for an I2C device
    i2cBus->_twoWire->beginTransmission(i2cAddress);
    status = i2cBus->_twoWire->endTransmission();

    // Release the lock
    r4aLockRelease(&i2cBus->_lock);

    // Return the I2C device found status
    if (status == 0)
        return true;
    return false;
}

//*********************************************************************
// Get the TwoWire pointer
//
// Warning: Using the I2C bus outside of these routines will break the
// I2C controller synchronization leading to hangs, crashes and unspecified
// behavior!
//
// Outputs:
//   Returns the TwoWire i2cBus address
TwoWire * r4aI2cBusGetTwoWire(R4A_I2C_BUS * i2cBus)
{
    return i2cBus->_twoWire;
}

//*********************************************************************
// Check if an I2C device was seen during the enumeration
// Return true if device detected, false otherwise
bool r4aI2cBusIsDevicePresent(R4A_I2C_BUS * i2cBus, R4A_I2C_ADDRESS_t i2cAddress)
{
    if (!i2cBus->_enumerated)
        r4aI2cBusEnumerate(i2cBus, nullptr);
    return i2cBus->_present[i2cAddress / 8] & (1 << (i2cAddress & 7));
}

//*********************************************************************
// Send data to an I2C peripheral
// Return true upon success, false otherwise
bool r4aI2cBusWrite(R4A_I2C_BUS * i2cBus,
                        R4A_I2C_ADDRESS_t i2cAddress,
                        const uint8_t * cmdBuffer,
                        size_t cmdByteCount,
                        const uint8_t * dataBuffer,
                        size_t dataByteCount,
                        Print * display,
                        bool releaseI2cBus)
{
    bool status;

    // Single thread the I2C requests
    r4aLockAcquire(&i2cBus->_lock);

    // Perform the I2C write operation
    status = i2cBus->_writeWithLock(i2cBus,
                                    i2cAddress,
                                    cmdBuffer,
                                    cmdByteCount,
                                    dataBuffer,
                                    dataByteCount,
                                    display,
                                    releaseI2cBus);

    // Release the lock
    r4aLockRelease(&i2cBus->_lock);

    // Return the write status
    return status;
}

//****************************************
// I2C menu API
//****************************************

//*********************************************************************
// Get the I2C address and register numbers
bool r4aI2cMenuGetAddressRegister(const R4A_MENU_ENTRY * menuEntry,
                                  const char * command,
                                  int * values,
                                  R4A_I2C_ADDRESS_t * i2cAddress,
                                  uint8_t * i2cRegister)
{
    int a;
    int r;

    // Get the parameter name
    String line = r4aMenuGetParameters(menuEntry, command);

    // Get the values
    *values = sscanf(line.c_str(), "%x %x", &a, &r);

    // Determine if the values are within range
    if ((*values == 2)
        && (a >= 0)
        && (a < R4A_I2C_ADDRESSES)
        && (r >= 0)
        && (r <= 0xff))
    {
        *i2cAddress = a;
        *i2cRegister = r;
        return true;
    }
    else if (*values == 1)
    {
        *i2cAddress = a;
        return true;
    }
    return false;
}

//*********************************************************************
// Get the device, register and data values
bool r4aI2cMenuGetAddressRegisterData(const R4A_MENU_ENTRY * menuEntry,
                                      const char * command,
                                      int * values,
                                      R4A_I2C_ADDRESS_t * i2cAddress,
                                      uint8_t * i2cRegister,
                                      uint8_t * data)
{
    int a;
    int r;
    int w;

    // Get the parameter name
    String line = r4aMenuGetParameters(menuEntry, command);

    // Get the values
    *values = sscanf(line.c_str(), "%x %x %x", &a, &r, &w);

    // Determine if the values are within range
    if ((*values == 3)
        && (a >= 0)
        && (a < R4A_I2C_ADDRESSES)
        && (r >= 0)
        && (r <= 0xff)
        && (w >= 0)
        && (w <= 0xff))
    {
        *i2cAddress = a;
        *i2cRegister = r;
        *data = w;
        return true;
    }
    else if (*values == 2)
    {
        *i2cAddress = a;
        *data = r;
        return true;
    }
    else if (*values == 1)
        *i2cAddress = a;
    return false;
}

//*********************************************************************
// Enumerate the I2C bus
void r4aI2cMenuEnumerate(const R4A_MENU_ENTRY * menuEntry,
                         const char * command,
                         Print * display)
{
    r4aI2cBusEnumerate(r4aI2cBus, display);
}

//*********************************************************************
// Read data from the I2C device
void r4aI2cMenuRead(const R4A_MENU_ENTRY * menuEntry,
                    const char * command,
                    Print * display)
{
    int bytesRead;
    uint8_t data;
    R4A_I2C_ADDRESS_t i2cAddress;
    uint8_t i2cRegister;
    int values;

    // Parse the command line
    if (r4aI2cMenuGetAddressRegister(menuEntry,
                                     command, &values, &i2cAddress, &i2cRegister))
    {
        bytesRead = r4aI2cBus->_read(r4aI2cBus,
                                     i2cAddress,
                                     (values == 2) ? &i2cRegister : nullptr,
                                     (values == 2) ? sizeof(i2cRegister) : 0,
                                     &data,
                                     sizeof(data),
                                     nullptr,
                                     true);       // End of transaction
        if (bytesRead != sizeof(data))
            display->println("Failed to read register!");
        else if (values == 1)
            display->printf("0x%03x: 0x%02x (%d)\r\n",
                            i2cAddress,
                            data, data);
        else
            display->printf("0x%03x[0x%02x]: 0x%02x (%d)\r\n",
                            i2cAddress,
                            i2cRegister,
                            data, data);
    }
    else if (values <= 0)
        display->println("Please specify the I2C address (0 - 0x3ff) for aa");
}

//*********************************************************************
// Write data to the I2C device
void r4aI2cMenuWrite(const R4A_MENU_ENTRY * menuEntry,
                     const char * command,
                     Print * display)
{
    int bytesWritten;
    uint8_t data;
    R4A_I2C_ADDRESS_t i2cAddress;
    uint8_t i2cRegister;
    int values;

    // Parse the command line
    if (r4aI2cMenuGetAddressRegisterData(menuEntry, command, &values, &i2cAddress, &i2cRegister, &data))
    {
        bytesWritten = r4aI2cBusWrite(r4aI2cBus,
                                      i2cAddress,
                                      (values == 3) ? &i2cRegister : nullptr,
                                      (values == 3) ? sizeof(i2cRegister) : 0,
                                      &data,
                                      sizeof(data),
                                      nullptr,
                                      true);       // End of transaction
        if (bytesWritten != (values - 1))
            display->println("Failed to write register!");
    }
    else if (values <= 0)
        display->println("Please specify the I2C address (0 - 0x3ff) for aa");
    else if (values == 1)
        display->println("Please specify the I2C register (0 - 0xff) for rr");
}
